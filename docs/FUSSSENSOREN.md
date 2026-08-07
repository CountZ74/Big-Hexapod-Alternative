# Fußsensoren: Stand, Messwerte, offene Punkte

Arbeitsnotiz zum Sensor-Ausbau. Sie hält vor allem die **gemessenen Zahlen**
und die **Begründungen** fest — der Code sagt, *was* passiert, hier steht,
*warum* es so und nicht anders ist.

Stand: August 2026, Commits bis `04db728`.

---

## 1. Aufbau

Jedes Bein hat einen Hall-Sensor an einer federbelasteten Schubstange. Unten
sitzt der Fuß, oben der Magnet. Setzt das Bein auf, schiebt sich die Stange
gegen die Feder nach oben, der Magnet wandert am Sensor vorbei.

Das ist ein **Wegaufnehmer, kein Taster**. Die Größe, mit der wir arbeiten,
ist der normierte Federweg zwischen zwei mechanischen Endpunkten:

- `0.0` = Stange ausgefahren, keine Last
- `1.0` = Stange am Anschlag

Zwei Maestros: 24CH = linke Seite, 12CH = rechte Seite. Reihenfolge auf
beiden Boards: Sensor, Coxa, Femur, Tibia — hinten, Mitte, vorne. Die
Kamera läuft weiter über die PCA9685 der Originalplatine.

Analogeingänge gibt es nur auf den Kanälen 0–11. Ein Servokanal, der
versehentlich als Input konfiguriert wird, bekommt keinen Puls mehr — das
Bein fällt zusammen. Die Config lehnt solche Kollisionen deshalb ab.

**Verkabelung:** Sensor-VCC und ADC-Referenz müssen derselbe Knoten sein.
Der Sensor ist ratiometrisch (SS49E-Klasse), sein Ausgang skaliert mit der
Versorgungsspannung. Pro Maestro geht deshalb eine Brücke von einem
+5V-Pin auf die Servoschiene.

---

## 2. Gemessene Werte

Alle Zahlen stammen vom Roboter, nicht aus dem Datenblatt.

| Größe | Wert | Herkunft |
|---|---|---|
| Federweg der Schubstange | 5,5 mm | von Hand nachgemessen |
| `level_per_mm` | 0,068 | Regression der Sensorantwort über aufgebrachte Korrekturen |
| Absacken beim Schwung | ~6,2 mm | `walk --touch 5` auf ebenem Boden |
| Aufschlag durch die Feder | ~2,6 mm | 15-mm-Klotz wurde als 17,6 mm gemeldet |
| Messwiederholbarkeit | ~2 % Federweg | wiederholte Messreihen im Stand |
| Deadband im auto-trim | 3 % | knapp über der Wiederholbarkeit |
| Mechanischer Anschlag | ~97 % | Hand-Drucktest |

**`level_per_mm` ist nicht der Federweg.** Das war einmal verwechselt und
hat zu 2,7-facher Unterkorrektur geführt. Die 5,5 mm sind eine mechanische
Abmessung. `level_per_mm` ist die Antwort des Sensors auf eine Änderung der
*Beinhöhe* — und die ist kleiner, weil sich die Last beim Verstellen eines
Beins auf die anderen fünf umverteilt und der Körper mitgeht.

**Die Kennlinie ist keine Gerade.** Ein Magnetfeld fällt im Fernfeld mit
1/r³ ab, nahe der Polfläche ist es fast flach — insgesamt eine S-Kurve.
Das `auto-trim` funktioniert trotzdem, weil es nur *lokale* Linearität an
seinem Arbeitspunkt (~21 % Federweg) braucht. Eine Linearisierung über
Stützstellen ist möglich, war aber bisher nicht nötig.

**Die Kalibrierung ist um ~3 % veraltet.** Beim Nachkalibrieren gilt:
drücken, bis der Wert sich nicht mehr ändert — nicht so fest wie möglich.

---

## 3. Was die Sensoren können

1. Erkennen, ob ein Fuß in der Luft ist
2. Leichte Berührung erkennen (in einer Richtung)
3. Bei sechs Beinen am Boden unterscheiden, wie viel Last auf welchem liegt
4. Beine relativ zueinander vergleichen
5. Geländehöhe in Millimetern messen

Was sie **nicht** können:

- Seitliche Kräfte (die Stange misst nur axial)
- Lastauflösung während des Dreibeinstands — dafür ist der Federweg zu kurz.
  Steifere Federn oder mehr Weg wären nötig.

---

## 4. Aufsetzerkennung im Gang

Vier Entscheidungen, die alle aus Fehlern am Roboter entstanden sind.

### Erst scharf, nachdem der Fuß frei war

Beim Anheben verlässt der Fuß den Boden **nicht sofort**. Erst entspannt
sich die Feder, und der Körper sackt nach, weil die anderen Beine die Last
übernehmen. Das Bein ist also noch belastet, während sein Offset längst
steigt.

Die erste Fassung prüfte dort auf Kontakt und fror die Beine direkt beim
Abheben ein — der Roboter hob die Füße gar nicht mehr an. Kontakt zählt
jetzt erst, wenn das Bein zwischendurch frei war, ist also eindeutig ein
*Wieder*-Aufsetzen.

### Eigene Mindesthöhe für den Gang (12 mm)

`settle_to_stance` hebt immer nur *ein* Bein — dort liegt der erste Kontakt
bei 1–4 mm über der Standpose, 5 mm Grenze passt. Im Gang hängt eine ganze
Dreiergruppe in der Luft, der Körper sackt weiter ab, und der erste Kontakt
liegt bei ~6,2 mm.

Mit der 5-mm-Grenze mitten in diesem Bereich entschied die Serienstreuung,
welches Bein anhält. Der Gang hat deshalb eine eigene Grenze von 12 mm.

**Folge:** Stufen unter ~12 mm werden nicht erkannt. Das ist in Ordnung —
die knapp 6 mm Federweg fangen sie ohnehin ab.

### Geländehöhe relativ zur Gruppe, nicht absolut

Während eine Gruppe schwingt, sackt der Körper ab — für alle drei Beine
gleich. Absolute Höhen mischen deshalb "der Boden ist hier höher" mit "der
Roboter steht gerade tiefer". Gemeldet wird nur die Abweichung vom Median
der Gruppe; Beine ohne Halt zählen mit 0,0, weil sie regulär bis in die
Standpose durchgefedert sind.

Ein Bein, das *nicht* angehalten hat, wird nie gemeldet. Es hat die
Standpose erreicht — das ist kein Loch.

### Die Höhe wird gehalten

Ohne das war die Messung wertlos: Nach dem Halbzyklus zielte die nächste
Bahn wieder auf `z = 0`, also unter die Stufe. Vertikal kann das Bein dort
nicht nachgeben, also löst sich der Befehl geometrisch auf und der Fuß
wandert nach außen — sichtbar als aktiv gesteuertes Rutschen.

`walk()` merkt sich jetzt pro Bein die zuletzt gefundene Bodenhöhe:

- **Standbein:** konstant darauf
- **Schwungbein:** die alte Höhe läuft bis zum Scheitel auf null aus. Es
  hebt dort ab, wo es steht, setzt aber auf der nominalen Ebene wieder auf
  und misst damit unvoreingenommen neu.

Dadurch korrigiert sich der Wert beim Verlassen der Stufe von selbst.
"Stufe zu Ende" muss nicht gesondert erkannt werden.

**Das Gedächtnis ist genau einen Schwung lang.** Genau deshalb hält der
Gang auch keilförmige und verrutschende Hindernisse aus: Es wird nie eine
Karte gebaut, die veralten könnte.

---

## 5. Bekannte Grenzen

- **Zwei Beine derselben Gruppe auf gleicher Stufe** — dann ist die Stufe
  der Median und gilt als Boden. Ein rein relatives Maß kann das nicht
  auflösen. Dafür braucht es den MPU6050, der echte Neigung sieht.
- **Löcher** werden nicht erkannt (kein Kontakt bis zum Bahnende).
- **Der Körper folgt der Stufe**, statt waagerecht zu bleiben — die
  Standbeine gleichen die Neigung nicht aus.
- Die gemeldete Höhe **überschätzt** die Stufe um ~2,6 mm.

---

## 6. Offene Punkte

1. **Gyro / Reflexe.** `mpu6050.py` liest bisher nur den
   Beschleunigungssensor. Die Konstanten für das Gyroskop stehen schon da
   (`REG_GYRO_XOUT_H = 0x43`, `GYRO_SCALE = 131.0`), aber es fehlen
   `read_gyro`, die Bias-Schätzung und die Sensorfusion. Damit ließe sich
   ein nicht kommandiertes Kippen erkennen — der Schritt könnte dann
   abgebrochen oder angepasst werden.
2. **Lasterfassung in drei Phasen** für `settle_to_stance`: finden, auf
   Lastanteil bringen, Reflex. Das Lastziel muss *relativ* zu den anderen
   tragenden Beinen sein, nicht als fester Wert in der Config.
3. **Lochererkennung.**
4. **Andere Gangarten** (Tetrapod, Ripple, Wave) und `command_tripod.py`
   für die Web-UI — teilen sich den Executor, sind aber noch nicht
   verdrahtet.
5. **Sensoren nachkalibrieren** (siehe oben).
6. **Kennlinie linearisieren** — optional, bisher nicht nötig.

---

## 7. Arbeitsweise

Qualitätsschranken, die grün bleiben müssen:

```
uv run pytest          # aktuell 1027 Tests
uv run mypy src        # strict, muss sauber bleiben
uv run ruff check src  # Basislinie 43 vorbestehende Fehler
```

Deutsche Docstrings und Kommentare, Conventional Commits.

**Sicherheitsregel: Bewegungsskripte auf echter Hardware startet immer der
Mensch, nie der Assistent.** Rein lesende Kommandos (`foot-monitor`,
Telemetrie) sind unkritisch.

### Zwei wiederkehrende Fallen

**Feste Sensorwerte in Tests sind unphysikalisch.** Ein Bein in der Luft
meldet keine Last. Wer im Test einen konstanten Wert setzt, prüft einen
Zustand, den es am Roboter nicht gibt — so blieb der Einfrier-Fehler beim
Abheben im Simulator unsichtbar. Tests benutzen deshalb ein Bodenmodell, in
dem der Federweg von der Beinhöhe abhängt.

**Nicht gegen Zwischenstände vergleichen.** Mehrfach sind falsche
Hypothesen daraus entstanden, dass ein Zustand mit einem halbfertigen
verglichen wurde statt mit der Git-Historie.
