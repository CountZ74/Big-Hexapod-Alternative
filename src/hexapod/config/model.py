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


# =====================================================================
# Globale Servo-Sicherheitsgrenzen — eine einzige Quelle der Wahrheit
# =====================================================================


class ServoLimits(StrictBase):
    """Globale Hardware-Sicherheitsgrenzen für alle Servos.

    Diese Werte werden NIEMALS überschritten, egal was pro Servo
    konfiguriert ist. Sie schützen vor versehentlicher Beschädigung
    durch Tippfehler in der YAML oder Bugs im Code.
    """

    absolute_min_us: float = Field(
        default=400.0,
        gt=0.0,
        description="Untere absolute Grenze [µs] — nie unterschritten.",
    )
    absolute_max_us: float = Field(
        default=2600.0,
        gt=0.0,
        description="Obere absolute Grenze [µs] — nie überschritten.",
    )

    @model_validator(mode="after")
    def _validate_ordering(self) -> ServoLimits:
        if self.absolute_min_us >= self.absolute_max_us:
            raise ValueError(
                f"absolute_min_us ({self.absolute_min_us}) muss < "
                f"absolute_max_us ({self.absolute_max_us}) sein."
            )
        return self


class DriverConfig(StrictBase):
    """Haupttreiber — der Maestro mit den 18 Beinservos."""

    type: Literal["maestro", "simulator"] = Field(default="maestro")
    port: str = Field(default="/dev/maestro_cmd")
    num_channels: int = Field(default=24, gt=0, le=24)
    timeout: float = Field(default=1.0, gt=0.0)


class CameraDriverConfig(StrictBase):
    """Optionaler zweiter Treiber nur für die Kamera-Servos.

    Auf der Freenove-Platine hängen Pan/Tilt an den beiden PCA9685
    (Anschluss 29 und 30). Ist dieser Block gesetzt, liegen die
    `kind: camera`-Servos auf DIESEM Bus — ihre Kanalnummern sind
    dann unabhängig von den Maestro-Kanälen.

    Fehlt der Block, bleibt alles beim Alten: auch die Kamera-Servos
    hängen am Haupttreiber (rückwärtskompatibel).
    """

    type: Literal["pca9685", "simulator"] = Field(default="pca9685")
    bus: int = Field(default=1, ge=0, description="I2C-Busnummer (Pi: immer 1).")
    num_channels: int = Field(default=32, gt=0, le=32)
    freq_hz: float = Field(default=50.0, gt=0.0)


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
    channel: int = Field(ge=0, lt=32)
    center_us: float = Field(gt=0.0)
    direction: Literal[-1, 1] = Field(default=1)
    min_us: float = Field(gt=0.0)
    max_us: float = Field(gt=0.0)
    range_us: float = Field(gt=0.0)

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

# Nur die Kanäle 0..11 des Mini Maestro 24 können analog messen; 12..23
# sind reine Digital-Ein-/Ausgänge und liefern nur 0 oder 1023.
MAX_ANALOG_CHANNEL = 11

# Kleinste noch brauchbare Spanne zwischen "kein Kontakt" und "Kontakt".
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

    raw_unloaded: float = Field(
        ge=0.0,
        le=ANALOG_MAX,
        description="Rohwert bei ganz ausgefahrener Schubstange (keine Last).",
    )
    raw_full: float = Field(
        ge=0.0,
        le=ANALOG_MAX,
        description="Rohwert bei ganz eingedrückter Schubstange (Anschlag).",
    )

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
        """Rohwert → normierter Federweg, hart auf [0, 1] begrenzt.

        Über den Anschlag hinaus geht es mechanisch nicht, unter die
        entspannte Feder auch nicht — Werte außerhalb sind Rauschen.
        """
        value = (raw - self.raw_unloaded) / self.span
        return min(1.0, max(0.0, value))


class FootSensorConfig(StrictBase):
    """Ein Fußsensor an einem Analogeingang des Maestro."""

    leg: str = Field(min_length=1)
    channel: int = Field(
        ge=0,
        le=MAX_ANALOG_CHANNEL,
        description=(
            "Maestro-Kanal, in der Maestro Control Center als 'Input' "
            "konfiguriert. Nur 0..11 können analog messen."
        ),
    )
    enabled: bool = Field(default=True)
    calibration: FootSensorCalibration | None = Field(
        default=None,
        description="Fehlt sie, liefert der Sensor nur Rohwerte (keinen Pegel).",
    )


class FootSensorsConfig(StrictBase):
    """Alle Fußsensoren. Leer = Roboter hat (noch) keine."""

    poll_hz: float = Field(
        default=50.0,
        gt=0.0,
        le=200.0,
        description="Abtastrate für den Telemetrie-Thread.",
    )
    samples: int = Field(
        default=3,
        ge=1,
        le=15,
        description="Anzahl Einzelmessungen pro Abtastung (Median gegen Ausreißer).",
    )
    sensors: list[FootSensorConfig] = Field(default_factory=list)

    @model_validator(mode="after")
    def _validate_unique(self) -> FootSensorsConfig:
        legs = [s.leg for s in self.sensors]
        if len(set(legs)) != len(legs):
            duplicates = {leg for leg in legs if legs.count(leg) > 1}
            raise ValueError(f"Pro Bein nur ein Fußsensor. Doppelt: {sorted(duplicates)}")
        channels = [s.channel for s in self.sensors]
        if len(set(channels)) != len(channels):
            duplicates_ch = {c for c in channels if channels.count(c) > 1}
            raise ValueError(
                f"Fußsensor-Kanäle müssen eindeutig sein. Doppelt: {sorted(duplicates_ch)}"
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


class RobotConfig(StrictBase):
    name: str = Field(default="Hexapod", min_length=1)
    description: str = ""
    servo_limits: ServoLimits = Field(default_factory=ServoLimits)
    driver: DriverConfig = Field(default_factory=DriverConfig)
    camera_driver: CameraDriverConfig | None = Field(default=None)
    body: BodyConfig
    servos: list[ServoConfig] = Field(min_length=1)
    foot_sensors: FootSensorsConfig = Field(default_factory=FootSensorsConfig)

    # ---- Bus-Zuordnung ----

    @property
    def camera_on_own_bus(self) -> bool:
        """True, wenn die Kamera-Servos an einem eigenen Treiber hängen."""
        return self.camera_driver is not None

    @property
    def main_bus_servos(self) -> list[LegServoConfig | CameraServoConfig]:
        """Servos am Haupttreiber (Maestro)."""
        if self.camera_on_own_bus:
            return [s for s in self.servos if isinstance(s, LegServoConfig)]
        return list(self.servos)

    @property
    def camera_bus_servos(self) -> list[CameraServoConfig]:
        """Servos am Kamera-Treiber (leer, wenn es keinen gibt)."""
        if not self.camera_on_own_bus:
            return []
        return [s for s in self.servos if isinstance(s, CameraServoConfig)]

    # ---- Validatoren ----

    @model_validator(mode="after")
    def _validate_channel_uniqueness(self) -> RobotConfig:
        """Kanäle müssen je Bus eindeutig sein (zwei Busse dürfen sich überlappen)."""
        for bus_name, servos in (
            ("Haupttreiber", self.main_bus_servos),
            ("Kamera-Treiber", self.camera_bus_servos),
        ):
            channels = [s.channel for s in servos]
            if len(set(channels)) != len(channels):
                duplicates = {c for c in channels if channels.count(c) > 1}
                raise ValueError(
                    f"Servo-Kanäle am {bus_name} müssen eindeutig sein. "
                    f"Doppelt: {sorted(duplicates)}"
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
    def _validate_channels_within_driver(self) -> RobotConfig:
        for s in self.main_bus_servos:
            if s.channel >= self.driver.num_channels:
                raise ValueError(
                    f"Servo-Kanal {s.channel} außerhalb der Driver-Kanäle "
                    f"[0..{self.driver.num_channels - 1}]."
                )
        cam = self.camera_driver
        if cam is not None:
            for cs in self.camera_bus_servos:
                if cs.channel >= cam.num_channels:
                    raise ValueError(
                        f"Kamera-Servo-Kanal {cs.channel} außerhalb der "
                        f"Kamera-Treiber-Kanäle [0..{cam.num_channels - 1}]."
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
        """Fußsensoren: bekannte Beine, und kein Kanal, auf dem ein Servo hängt.

        Das ist die wichtigste Prüfung überhaupt: Ein als Eingang
        konfigurierter Kanal, auf dem trotzdem ein Servo steckt, bekommt
        keinen Puls mehr — das Bein fällt zusammen.
        """
        leg_names = {leg.name for leg in self.body.legs}
        servo_channels = {s.channel for s in self.main_bus_servos}
        for sensor in self.foot_sensors.sensors:
            if sensor.leg not in leg_names:
                raise ValueError(
                    f"Fußsensor auf Kanal {sensor.channel} referenziert unbekanntes "
                    f"Bein {sensor.leg}. Bekannte Beine: {sorted(leg_names)}"
                )
            if sensor.channel in servo_channels:
                raise ValueError(
                    f"Fußsensor-Kanal {sensor.channel} ({sensor.leg}) ist am "
                    f"Haupttreiber bereits durch einen Servo belegt."
                )
            if sensor.channel >= self.driver.num_channels:
                raise ValueError(
                    f"Fußsensor-Kanal {sensor.channel} außerhalb der Driver-Kanäle "
                    f"[0..{self.driver.num_channels - 1}]."
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
