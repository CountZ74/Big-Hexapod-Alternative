# Verkabelung

## Controller

| Bus | Board | idProduct | Symlink | Körperseite |
|-----|-------|-----------|---------|-------------|
| `left` | Mini Maestro **24CH** | `008c` | `/dev/maestro_left_cmd` | links |
| `right` | Mini Maestro **12CH** | `008a` | `/dev/maestro_right_cmd` | rechts |
| `camera` | 2× PCA9685 (Freenove-Platine) | — | I²C-Bus 1 | Pan/Tilt |

Beide Maestros hängen per USB am Pi, beide im Modus **USB Dual Port**
(Werkseinstellung). Kein Daisy-Chain, keine Gerätenummern, kein Pololu-Protokoll
— jedes Board hat sein eigenes Device-File. Die Stabilität der Namen macht
`99-maestro.rules` über die Produkt-ID.

## Kanalbelegung (auf beiden Boards identisch)

```
ch 0  Fußsensor  |  ch 1 Coxa   ch 2 Femur   ch 3 Tibia   -- hinteres Bein
ch 4  Fußsensor  |  ch 5 Coxa   ch 6 Femur   ch 7 Tibia   -- mittleres Bein
ch 8  Fußsensor  |  ch 9 Coxa  ch 10 Femur  ch 11 Tibia   -- vorderes Bein
```

Am 24CH bleiben die Kanäle 12–23 frei (Klauen o. ä.). Am 12CH ist alles belegt.

Die Sensorkanäle müssen im **Maestro Control Center** von „Servo" auf „Input"
gestellt und im Gerät gespeichert werden. Nur die Kanäle 0–11 haben einen ADC;
12–23 sind reine Digitalein-/-ausgänge.

## Stromversorgung

Servos werden **komplett vom Original-Freenove-Board** versorgt (eigener
2S-Pack). Am Maestro liegen nur Signalleitungen plus eine gemeinsame Masse.
Der Maestro ist damit ein reiner Signalgeber — über seine Masse fließen nur
Signalrückströme, weshalb seine ADC-Referenz stillsteht, während nebenan
18 Servos ihre Ampere ziehen. Genau das macht die Fußsensor-Messung erst
brauchbar.

Auf jedem Maestro ist ein **5V(out)-Pin auf die Servoschiene gebrückt**, damit
die Hallsensoren ihre Versorgung aus demselben dreipoligen Kanalstecker
beziehen wie ihr Signal. Sensor-VCC und ADC-Referenz sind dadurch derselbe
Knoten — die Ratiometrie des Hallsensors kürzt Versorgungsschwankungen exakt
weg.

> **Achtung:** Durch diese Brücke liegt die komplette Servoschiene beider
> Boards auf USB-5 V. Es darf deshalb **niemals** Servospannung an die
> Servo-Power-Klemme eines Maestro gelegt und **kein vollständig belegter
> dreipoliger Servostecker** auf einen Kanal gesteckt werden — der Servo würde
> seinen Strom aus dem USB-Port des Pi ziehen. Am Maestro-Ende gilt: mittlerer
> Pin bleibt leer, nur Signal und Masse.
