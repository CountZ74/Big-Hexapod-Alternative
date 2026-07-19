# Deploy-Hilfsdateien

## 50-hexapod-network.rules
polkit-Regel, damit der als `sebi` laufende Webserver WLAN/Hotspot über
NetworkManager steuern darf (WebUI-Seite `/network`). Einmalig installieren:

```bash
sudo cp deploy/50-hexapod-network.rules /etc/polkit-1/rules.d/
sudo systemctl restart polkit
```

Voraussetzung: zwei WLAN-Adapter mit AP-fähigen Treibern (hier: rt2800usb +
brcmfmac). Einer kann im Heimnetz bleiben, während der andere einen Hotspot
aufmacht.
