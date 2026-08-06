"""Pydantic-Datenmodell für die Hexapod-Konfiguration."""

from __future__ import annotations

from enum import StrEnum
from typing import Annotated, Literal

from pydantic import BaseModel, ConfigDict, Field, model_validator


class LegSide(StrEnum):
    LEFT = "left"
    RIGHT = "right"


class LegRow(StrEnum):
    FRONT = "front"
    MID = "mid"
    BACK = "back"


class Joint(StrEnum):
    COXA = "coxa"
    FEMUR = "femur"
    TIBIA = "tibia"


class CameraAxis(StrEnum):
    PAN = "pan"
    TILT = "tilt"


class StrictBase(BaseModel):
    model_config = ConfigDict(extra="forbid", validate_assignment=True)


# Name des Busses, den ein Servo oder Sensor bekommt, wenn nichts dabeisteht.
# Für Aufbauten mit nur einem Controller bleibt die YAML damit knapp.
DEFAULT_BUS = "main"


# =====================================================================
# Globale Servo-Sicherheitsgrenzen — eine einzige Quelle der Wahrheit
# =====================================================================


class ServoLimits(StrictBase):
    """Globale Hardware-Sicherheitsgrenzen für alle Servos.

    Diese Werte werden NIEMALS überschritten, egal was pro Servo
    konfiguriert ist. Sie schützen vor versehentlicher Beschädigung
    durch Tippfehler in der YAML oder Bugs im Code.
    """

    absolute_min_us: float = Field(default=400.0, gt=0.0)
    absolute_max_us: float = Field(default=2600.0, gt=0.0)

    @model_validator(mode="after")
    def _validate_ordering(self) -> ServoLimits:
        if self.absolute_min_us >= self.absolute_max_us:
            raise ValueError(
                f"absolute_min_us ({self.absolute_min_us}) muss < "
                f"absolute_max_us ({self.absolute_max_us}) sein."
            )
        return self


# =====================================================================
# Busse — ein Bus ist ein physischer Controller
# =====================================================================


class MaestroBus(StrictBase):
    """Ein Pololu Maestro am USB.

    Kann Servos ansteuern UND auf den Kanälen 0..11 analog messen —
    daher hängen die Fußsensoren an genau so einem Bus.
    """

    type: Literal["maestro"] = "maestro"
    port: str = Field(default="/dev/maestro_cmd", min_length=1)
    num_channels: int = Field(default=24, gt=0, le=24)
    timeout: float = Field(default=1.0, gt=0.0)


class Pca9685Bus(StrictBase):
    """Die beiden PCA9685 der Freenove-Platine (nur Servos, keine Eingänge)."""

    type: Literal["pca9685"] = "pca9685"
    i2c_bus: int = Field(default=1, ge=0)
    num_channels: int = Field(default=32, gt=0, le=32)
    freq_hz: float = Field(default=50.0, gt=0.0)


class SimulatorBus(StrictBase):
    """Kein echter Controller — hält Positionen im RAM (Tests, Entwicklung)."""

    type: Literal["simulator"] = "simulator"
    num_channels: int = Field(default=24, gt=0, le=32)


BusConfig = Annotated[
    MaestroBus | Pca9685Bus | SimulatorBus,
    Field(discriminator="type"),
]

# Nur diese Bus-Typen haben Analogeingänge.
ANALOG_CAPABLE_BUS_TYPES = ("maestro", "simulator")


class LegGeometry(StrictBase):
    coxa_length: float = Field(gt=0.0)
    femur_length: float = Field(gt=0.0)
    tibia_length: float = Field(gt=0.0)


class LegConfig(StrictBase):
    name: str = Field(min_length=1)
    side: LegSide
    row: LegRow
    mount_x: float
    mount_y: float
    mount_angle_deg: float = Field(ge=-180.0, le=180.0)
    z_trim: float = Field(
        default=0.0,
        ge=-30.0,
        le=30.0,
        description=(
            "Höhen-Korrektur pro Bein in mm. Wird auf das Standpose-Fuß-Z "
            "addiert, um ungleichen Bodenkontakt (Kalibrier-/Mechanik-Toleranz) "
            "auszugleichen. Positiv = Fuß tiefer (Bein drückt mehr nach unten)."
        ),
    )


class BodyConfig(StrictBase):
    leg_geometry: LegGeometry
    legs: list[LegConfig] = Field(min_length=6, max_length=6)

    @model_validator(mode="after")
    def _validate_leg_names_unique(self) -> BodyConfig:
        names = [leg.name for leg in self.legs]
        if len(set(names)) != len(names):
            duplicates = {n for n in names if names.count(n) > 1}
            raise ValueError(f"Bein-Namen müssen eindeutig sein. Doppelt: {duplicates}")
        return self


class _ServoCommon(StrictBase):
    bus: str = Field(default=DEFAULT_BUS, min_length=1)
    channel: int = Field(ge=0, lt=32)
    center_us: float = Field(gt=0.0)
    direction: Literal[-1, 1] = Field(default=1)
    min_us: float = Field(gt=0.0)
    max_us: float = Field(gt=0.0)
    range_us: float = Field(gt=0.0)

    @property
    def address(self) -> tuple[str, int]:
        """Eindeutige Adresse: Kanalnummern wiederholen sich über Busse hinweg."""
        return (self.bus, self.channel)

    @model_validator(mode="after")
    def _validate_us_ordering(self) -> _ServoCommon:
        if self.min_us >= self.max_us:
            raise ValueError(f"min_us ({self.min_us}) muss < max_us ({self.max_us}) sein.")
        if not (self.min_us <= self.center_us <= self.max_us):
            raise ValueError(
                f"center_us ({self.center_us}) muss zwischen "
                f"min_us ({self.min_us}) und max_us ({self.max_us}) liegen."
            )
        return self


class LegServoConfig(_ServoCommon):
    kind: Literal["leg"] = "leg"
    leg: str = Field(min_length=1)
    joint: Joint


class CameraServoConfig(_ServoCommon):
    kind: Literal["camera"] = "camera"
    axis: CameraAxis


ServoConfig = Annotated[
    LegServoConfig | CameraServoConfig,
    Field(discriminator="kind"),
]


# =====================================================================
# Fußsensoren (Hall-Sensor + federbelastete Schubstange)
# =====================================================================

# Der Maestro liefert Analogwerte als 10-Bit-Zahl (0 = 0 V, 1023 = 5 V).
ANALOG_MAX = 1023.0

# Nur die Kanäle 0..11 eines Mini Maestro können analog messen; 12..23
# sind reine Digital-Ein-/Ausgänge und liefern nur 0 oder 1023.
MAX_ANALOG_CHANNEL = 11

# Kleinste noch brauchbare Spanne zwischen "unbelastet" und "Anschlag".
# Darunter ist das Nutzsignal im Rauschen des Hall-Sensors untergegangen.
MIN_CALIBRATION_SPAN = 15.0


class FootSensorCalibration(StrictBase):
    """Messbereich eines Fußsensors — der mechanische Vollweg der Schubstange.

    Der Hall-Sensor ist ein Wegaufnehmer, kein Taster: Er misst, *wie weit*
    die federbelastete Schubstange eingedrückt ist. Weil die Feder linear
    arbeitet, ist dieser Weg ein Maß für die Auflagekraft — und die ist beim
    leichten Antippen im Schwung eine ganz andere als im Stand auf sechs
    Beinen oder beim Abfangen im Gait.

    Kalibriert werden deshalb nur die beiden **mechanischen** Endpunkte:

        raw_unloaded  Stange ganz ausgefahren (Feder entspannt, Bein frei)
        raw_full      Stange ganz eingedrückt (mechanischer Anschlag)

    Das ist ein fester physikalischer Bezug. Er ändert sich nicht, wenn der
    Roboter mal steht und mal läuft, und jeder Betriebszustand liegt
    irgendwo dazwischen:

        pegel = (roh - raw_unloaded) / (raw_full - raw_unloaded)

    0.0 = unbelastet, 1.0 = am Anschlag. Ob der Rohwert beim Eindrücken
    steigt oder fällt, ergibt sich aus den beiden Werten — Einbaulage und
    Magnetpolung sind damit egal.

    Bewusst NICHT hier: eine Schwelle "hat Boden". Die hängt vom Kontext ab
    und gehört dorthin, wo dieser Kontext bekannt ist (Gait, Kletterlogik),
    nicht in die Sensor-Kalibrierung.
    """

    raw_unloaded: float = Field(ge=0.0, le=ANALOG_MAX)
    raw_full: float = Field(ge=0.0, le=ANALOG_MAX)

    @model_validator(mode="after")
    def _validate_span(self) -> FootSensorCalibration:
        span = abs(self.raw_full - self.raw_unloaded)
        if span < MIN_CALIBRATION_SPAN:
            raise ValueError(
                f"Messbereich zwischen raw_unloaded ({self.raw_unloaded}) und "
                f"raw_full ({self.raw_full}) ist nur {span:.1f} Zähler — "
                f"mindestens {MIN_CALIBRATION_SPAN} nötig. Sitzt der Magnet richtig?"
            )
        return self

    @property
    def span(self) -> float:
        """Vorzeichenbehaftete Spanne unbelastet → Anschlag."""
        return self.raw_full - self.raw_unloaded

    @property
    def counts_per_percent(self) -> float:
        """ADC-Zähler pro Prozent Federweg — die effektive Auflösung."""
        return abs(self.span) / 100.0

    def level(self, raw: float) -> float:
        """Rohwert → normierter Federweg, hart auf [0, 1] begrenzt."""
        value = (raw - self.raw_unloaded) / self.span
        return min(1.0, max(0.0, value))


class FootSensorConfig(StrictBase):
    """Ein Fußsensor an einem Analogeingang eines Maestro-Busses."""

    leg: str = Field(min_length=1)
    bus: str = Field(default=DEFAULT_BUS, min_length=1)
    channel: int = Field(ge=0, le=MAX_ANALOG_CHANNEL)
    enabled: bool = Field(default=True)
    calibration: FootSensorCalibration | None = Field(default=None)

    @property
    def address(self) -> tuple[str, int]:
        return (self.bus, self.channel)


class FootSensorsConfig(StrictBase):
    """Alle Fußsensoren. Leer = Roboter hat (noch) keine."""

    poll_hz: float = Field(default=50.0, gt=0.0, le=200.0)
    samples: int = Field(default=3, ge=1, le=15)
    travel_mm: float | None = Field(
        default=None,
        gt=0.0,
        le=50.0,
        description=(
            "Mechanischer Vollweg der Schubstange in mm — die Strecke, die "
            "raw_unloaded von raw_full trennt. Rechnet Federweg-Anteile in "
            "Millimeter um, was der Bein-Hoehenabgleich braucht. Eine "
            "Konstruktionsgroesse, die man einmal misst; fehlt sie, muss "
            "das Werkzeug sie schaetzen, und das ist deutlich unzuverlaessiger."
        ),
    )
    sensors: list[FootSensorConfig] = Field(default_factory=list)

    @model_validator(mode="after")
    def _validate_unique(self) -> FootSensorsConfig:
        legs = [s.leg for s in self.sensors]
        if len(set(legs)) != len(legs):
            duplicates = {leg for leg in legs if legs.count(leg) > 1}
            raise ValueError(f"Pro Bein nur ein Fußsensor. Doppelt: {sorted(duplicates)}")
        addresses = [s.address for s in self.sensors]
        if len(set(addresses)) != len(addresses):
            dups = {a for a in addresses if addresses.count(a) > 1}
            raise ValueError(
                f"Fußsensor-Kanäle müssen je Bus eindeutig sein. Doppelt: {sorted(dups)}"
            )
        return self

    def get(self, leg_name: str) -> FootSensorConfig:
        for sensor in self.sensors:
            if sensor.leg == leg_name:
                return sensor
        raise KeyError(f"Kein Fußsensor für Bein {leg_name}.")

    @property
    def active(self) -> list[FootSensorConfig]:
        """Nur die eingeschalteten Sensoren."""
        return [s for s in self.sensors if s.enabled]

    @property
    def active_buses(self) -> list[str]:
        """Busse, auf denen aktive Sensoren liegen — in Konfig-Reihenfolge."""
        seen: list[str] = []
        for sensor in self.active:
            if sensor.bus not in seen:
                seen.append(sensor.bus)
        return seen


class RobotConfig(StrictBase):
    name: str = Field(default="Hexapod", min_length=1)
    description: str = ""
    servo_limits: ServoLimits = Field(default_factory=ServoLimits)
    buses: dict[str, BusConfig] = Field(min_length=1)
    body: BodyConfig
    servos: list[ServoConfig] = Field(min_length=1)
    foot_sensors: FootSensorsConfig = Field(default_factory=FootSensorsConfig)

    # ---- Zugriff auf Busse ----

    def get_bus(self, name: str) -> BusConfig:
        try:
            return self.buses[name]
        except KeyError:
            raise KeyError(
                f"Unbekannter Bus {name!r}. Bekannt: {sorted(self.buses)}"
            ) from None

    def servos_on(self, bus_name: str) -> list[LegServoConfig | CameraServoConfig]:
        """Alle Servos an einem Bus, in Konfigurationsreihenfolge."""
        return [s for s in self.servos if s.bus == bus_name]

    @property
    def servo_buses(self) -> list[str]:
        """Busse, an denen mindestens ein Servo hängt — in Konfig-Reihenfolge."""
        seen: list[str] = []
        for servo in self.servos:
            if servo.bus not in seen:
                seen.append(servo.bus)
        return seen

    # ---- Validatoren ----

    @model_validator(mode="after")
    def _validate_bus_references(self) -> RobotConfig:
        for servo in self.servos:
            if servo.bus not in self.buses:
                raise ValueError(
                    f"Servo auf Kanal {servo.channel} verweist auf unbekannten Bus "
                    f"{servo.bus!r}. Bekannt: {sorted(self.buses)}"
                )
        for sensor in self.foot_sensors.sensors:
            if sensor.bus not in self.buses:
                raise ValueError(
                    f"Fußsensor {sensor.leg} verweist auf unbekannten Bus "
                    f"{sensor.bus!r}. Bekannt: {sorted(self.buses)}"
                )
        return self

    @model_validator(mode="after")
    def _validate_channel_uniqueness(self) -> RobotConfig:
        """Kanäle müssen je Bus eindeutig sein — über Busse hinweg dürfen sie sich
        wiederholen, denn Kanal 3 auf 'left' ist eine andere Buchse als auf 'right'."""
        addresses = [s.address for s in self.servos]
        if len(set(addresses)) != len(addresses):
            duplicates = {a for a in addresses if addresses.count(a) > 1}
            raise ValueError(
                f"Servo-Kanäle müssen je Bus eindeutig sein. Doppelt: {sorted(duplicates)}"
            )
        return self

    @model_validator(mode="after")
    def _validate_leg_servo_references(self) -> RobotConfig:
        leg_names = {leg.name for leg in self.body.legs}
        for s in self.servos:
            if isinstance(s, LegServoConfig) and s.leg not in leg_names:
                raise ValueError(
                    f"Servo auf Kanal {s.channel} referenziert unbekanntes Bein "
                    f"{s.leg}. Bekannte Beine: {sorted(leg_names)}"
                )
        return self

    @model_validator(mode="after")
    def _validate_complete_leg_coverage(self) -> RobotConfig:
        for leg in self.body.legs:
            joints_for_leg = {
                s.joint for s in self.servos
                if isinstance(s, LegServoConfig) and s.leg == leg.name
            }
            missing = set(Joint) - joints_for_leg
            if missing:
                raise ValueError(
                    f"Bein {leg.name} fehlen Servos für Gelenk(e): "
                    f"{sorted(j.value for j in missing)}"
                )
        return self

    @model_validator(mode="after")
    def _validate_channels_within_bus(self) -> RobotConfig:
        for servo in self.servos:
            bus = self.buses[servo.bus]
            if servo.channel >= bus.num_channels:
                raise ValueError(
                    f"Servo-Kanal {servo.channel} liegt außerhalb von Bus "
                    f"{servo.bus!r} [0..{bus.num_channels - 1}]."
                )
        return self

    @model_validator(mode="after")
    def _validate_servo_limits(self) -> RobotConfig:
        """Alle per-Servo min/max müssen innerhalb der globalen Grenzen liegen."""
        lim = self.servo_limits
        for s in self.servos:
            if s.min_us < lim.absolute_min_us:
                raise ValueError(
                    f"Servo Kanal {s.channel}: min_us ({s.min_us}) unterschreitet "
                    f"absolute_min_us ({lim.absolute_min_us})."
                )
            if s.max_us > lim.absolute_max_us:
                raise ValueError(
                    f"Servo Kanal {s.channel}: max_us ({s.max_us}) überschreitet "
                    f"absolute_max_us ({lim.absolute_max_us})."
                )
        return self

    @model_validator(mode="after")
    def _validate_foot_sensors(self) -> RobotConfig:
        """Fußsensoren: bekannte Beine, analogfähiger Bus, und kein Kanal,
        auf dem ein Servo hängt.

        Letzteres ist die wichtigste Prüfung überhaupt: Ein als Eingang
        konfigurierter Kanal, auf dem trotzdem ein Servo steckt, bekommt
        keinen Puls mehr — das Bein fällt zusammen.
        """
        leg_names = {leg.name for leg in self.body.legs}
        servo_addresses = {s.address for s in self.servos}
        for sensor in self.foot_sensors.sensors:
            if sensor.leg not in leg_names:
                raise ValueError(
                    f"Fußsensor auf Kanal {sensor.channel} referenziert unbekanntes "
                    f"Bein {sensor.leg}. Bekannte Beine: {sorted(leg_names)}"
                )
            bus = self.buses[sensor.bus]
            if bus.type not in ANALOG_CAPABLE_BUS_TYPES:
                raise ValueError(
                    f"Fußsensor {sensor.leg} liegt auf Bus {sensor.bus!r} vom Typ "
                    f"{bus.type!r} — der hat keine Analogeingänge."
                )
            if sensor.address in servo_addresses:
                raise ValueError(
                    f"Fußsensor-Kanal {sensor.channel} auf Bus {sensor.bus!r} "
                    f"({sensor.leg}) ist bereits durch einen Servo belegt."
                )
            if sensor.channel >= bus.num_channels:
                raise ValueError(
                    f"Fußsensor-Kanal {sensor.channel} liegt außerhalb von Bus "
                    f"{sensor.bus!r} [0..{bus.num_channels - 1}]."
                )
        return self

    # ---- Zugriffshelfer ----

    def get_leg(self, name: str) -> LegConfig:
        for leg in self.body.legs:
            if leg.name == name:
                return leg
        raise KeyError(f"Bein {name} nicht in der Konfiguration.")

    def get_leg_servo(self, leg_name: str, joint: Joint) -> LegServoConfig:
        for s in self.servos:
            if isinstance(s, LegServoConfig) and s.leg == leg_name and s.joint == joint:
                return s
        raise KeyError(f"Kein Servo für Bein {leg_name} Gelenk {joint.value}.")

    def get_camera_servo(self, axis: CameraAxis) -> CameraServoConfig:
        for s in self.servos:
            if isinstance(s, CameraServoConfig) and s.axis == axis:
                return s
        raise KeyError(f"Kein Kamera-Servo für Achse {axis.value}.")
