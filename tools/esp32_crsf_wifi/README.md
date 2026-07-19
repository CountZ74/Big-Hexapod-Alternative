# Hexapod CRSF→WiFi-Modul (XIAO ESP32-C6)

Steckt am externen Modulschacht der **RadioMaster TX16S** (EdgeTX: externes Modul = CRSF),
liest die RC-Kanäle und schickt `walk`/`halt`/`pose`/`set_gait` per WLAN an den
Hexapod-Webserver (`POST /command`). Ersetzt den OrangePi-CRSF-Controller —
Mapping und Failsafe sind 1:1 aus `tools/hexapod_crsf.py` portiert (CRSF-Parsing
gegen die Python-Referenz verifiziert).

## Benötigt

- **Board:** Seeed Studio XIAO ESP32-C6
- **Arduino IDE** mit ESP32 Arduino Core ≥ 3.0 (Boards-Manager-URL von Espressif),
  Board auswählen: **XIAO_ESP32C6**
- **Library:** `WiFiManager` von *tzapu* (Library Manager). `WiFi`, `HTTPClient`,
  `Preferences` sind im Core enthalten.

## Verkabelung

Nur **drei** Verbindungen nötig — CRSF ist nicht-invertiertes 3,3-V-TTL, geht
also direkt an einen GPIO. Die TX16S führt im Modulschacht einen geregelten
**5-V-Pin**, aus dem der XIAO direkt versorgt werden kann (kein USB/Step-Down):

| TX16S Modulschacht | XIAO ESP32-C6 |
|---|---|
| **5 V** (geregelt) | **5V**-Pin |
| GND | GND |
| CRSF-Signal (Modul-RX, Radio→Modul) | **D7** (GPIO17) |

> ⚠️ **Pins zuerst messen!** Der Schacht führt *auch* die rohe **Akkuspannung**
> (2S ~7–8,4 V) auf einem Pin — die **killt den XIAO**. Funke einschalten, jeden
> Pin gegen GND messen: **~8 V = VBAT (nicht verwenden)**, **~5,0 V = geregelt
> (hier dran)**, **~3,3 V idle = CRSF-Signal**. Belege die genaue Pin-Position
> erst nach dem Messen.
>
> **Strom:** Der XIAO zieht beim WLAN-Senden kurze Spitzen von mehreren hundert
> mA. Der 5-V-Regler im Schacht ist für ELRS-Module ausgelegt und sollte das
> packen. Resettet der XIAO beim Funken (Brown-out), einen **Stützkondensator**
> (~470 µF) zwischen 5V und GND nah am XIAO setzen.
>
> Alternative ohne Schacht-5V: XIAO per **USB-C** versorgen und nur
> **CRSF-Signal + GND** vom Schacht abgreifen (gemeinsame Masse!).

EdgeTX-Seite: *Model Setup → Internal/External RF → External = CRSF*, Baud 400k.

## Flashen

1. Repo-Ordner `esp32_crsf_wifi/` öffnen, `esp32_crsf_wifi.ino` in der Arduino IDE.
2. Board **XIAO_ESP32C6**, richtigen Port wählen.
3. Hochladen. (Beim ersten Mal evtl. BOOT halten + RESET für den Bootloader.)

## WLAN einrichten (Captive Portal)

Beim **ersten Start** (oder wenn du beim Einschalten die **BOOT-Taste** gedrückt
hältst) öffnet das Modul einen WLAN-Hotspot **`Hexapod-CRSF`**:

1. Mit Handy/Laptop in dieses WLAN verbinden — die Konfigseite öffnet sich.
2. Heim-WLAN auswählen, Passwort eingeben.
3. Feld **Robot-URL** prüfen/setzen (Default `http://hexapod.local:8000`).
4. Speichern → das Modul verbindet sich und merkt sich alles im NVS.

Robot-URL später ändern: BOOT beim Einschalten halten → Portal erneut öffnen.

## Status-LED

| LED | Bedeutung |
|---|---|
| schnell blinken | kein WLAN (verbindet / Portal aktiv) |
| langsam blinken | WLAN ok, aber **kein CRSF-Signal** |
| dauerhaft an | WLAN **und** CRSF ok — betriebsbereit |

## Kanal-Mapping (Default, wie hexapod_crsf.py)

| Kanal | Funktion |
|---|---|
| CH2 (rechter Stick vert) | vorwärts/rückwärts (vx) |
| CH1 (rechter Stick hor) | seitwärts (vy) |
| CH4 (linker Stick hor) | drehen (omega) |
| CH3 (linker Stick vert) | Schritthöhe |

Optional aktivierbar (im Sketch `CH_GAIT/CH_TZ/CH_TX/CH_TY/CH_MODE/CH_HALT` von
`-1` auf den Kanal-Index setzen): Gangart-Schalter, Körper-Translationen, Pose-
Modus (Sticks → roll/pitch/yaw), Not-Halt-Schalter.

**Mapping prüfen:** Über die serielle Konsole (115200) kannst du zum Debuggen
`Serial.print` ergänzen, oder die Achsen vorab mit `hexapod_crsf.py --monitor`
am OrangePi/PC identifizieren. Richtung falsch herum? Das passende `INV_*`-Flag
im Sketch umdrehen.

## Failsafe

Bleibt das CRSF-Signal länger als **0,5 s** aus (Funke aus, außer Reichweite),
sendet das Modul automatisch `halt` — der Roboter bleibt stehen.

## Sicherheit

Das Modul kommandiert echte Bewegung. Roboter beim ersten Test frei hinstellen,
Platz schaffen. Die Limits (`VX_MAX`, `OMEGA_MAX`, …) sind konservativ und oben
im Sketch anpassbar.
