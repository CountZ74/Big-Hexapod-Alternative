#!/usr/bin/env bash
# lgpio in die uv-venv verfuegbar machen (Pi 3 / Debian trixie).
#
# Der HC-SR04-Sonar-Treiber braucht python3-lgpio. Ein Source-Build im venv
# scheitert ohne swig -- daher nutzen wir das fertige System-Paket und linken
# das Modul in die venv (ABI passt, gleiche CPython-Minor-Version).
#
#   sudo apt install -y python3-lgpio   # einmalig
#   ./deploy/link_system_lgpio.sh
set -euo pipefail
cd "$(dirname "$0")/.."
SP=$(.venv/bin/python -c "import site; print(site.getsitepackages()[0])")
for f in $(dpkg -L python3-lgpio | grep -E "lgpio\.py$|_lgpio.*\.so$"); do
    ln -sf "$f" "$SP/$(basename "$f")"
    echo "linked $(basename "$f") -> $SP"
done
.venv/bin/python -c "import lgpio; print('lgpio OK')"
