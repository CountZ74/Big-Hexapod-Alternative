#!/usr/bin/env python3
"""Hexapod-Controller via CRSF (TX16S UART -> WLAN).

Liest CRSF-Kanaele von der TX16S (EdgeTX: externes Modul = CRSF) ueber UART5
(/dev/ttyS5, Header-Pin 11=TX/13=RX) und schickt walk/pose/Gangart-Befehle an
den Hexapod-Webserver. Stromversorgung des OrangePi laeuft ueber die 5V des
Funken-UART-Ports.

  Mapping finden:  python3 hexapod_crsf.py --monitor
  Betrieb:         python3 hexapod_crsf.py --host http://hexapod.local:8000
"""
from __future__ import annotations
import argparse, time
import serial, requests

RC_CHANNELS_PACKED = 0x16

# ===================== MAPPING (Kanal-Index 0-basiert = CH1..CH16) =====================
CH_VX     = 1    # CH2 rechter Stick vert  -> vorwaerts/rueckwaerts (vx)
CH_VY     = 0    # CH1 rechter Stick hor   -> seitwaerts (vy)
CH_OMEGA  = 3    # CH4 linker Stick hor    -> drehen (omega)
CH_HEIGHT = 2    # CH3 linker Stick vert   -> Hoehe
CH_GAIT   = -1   # Schalter -> Gangart (-1 = aus)
CH_TZ     = -1   # Poti -> Koerperhoehe tz
CH_TX     = -1   # Slider -> vor/zurueck tx
CH_TY     = -1   # Poti -> seitlich ty
CH_MODE   = -1   # Schalter -> Pose-Modus (Sticks -> roll/pitch/yaw)
CH_HALT   = -1   # Schalter -> Not-Halt
INV_VX=False; INV_VY=True; INV_OMEGA=True; INV_HEIGHT=False
INV_TZ=False; INV_TX=False; INV_TY=False
VX_MAX=40.0; VY_MAX=40.0; OMEGA_MAX=30.0
H_MIN,H_MAX=15.0,50.0
TZ_MIN,TZ_MAX=-20.0,40.0; TX_MIN,TX_MAX=-30.0,30.0; TY_MIN,TY_MAX=-30.0,30.0
ROLL_MAX=PITCH_MAX=YAW_MAX=18.0
GAITS=["tripod","tetrapod","ripple","wave"]
DEAD=0.06
# ======================================================================================

def crc8(data: bytes) -> int:
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0xD5) & 0xFF if (crc & 0x80) else (crc << 1) & 0xFF
    return crc

def unpack_channels(payload: bytes) -> list[int]:
    bb = nb = 0; out = []
    for byte in payload:
        bb |= byte << nb; nb += 8
        while nb >= 11:
            out.append(bb & 0x7FF); bb >>= 11; nb -= 11
    return out[:16]

def norm(v: int) -> float:        # CRSF 172..1811, Mitte 992 -> -1..+1
    return max(-1.0, min(1.0, (v - 992) / 819.0))

def chn(chs, i): return norm(chs[i]) if 0 <= i < len(chs) else 0.0
def applyv(v, inv, dz=DEAD):
    if inv: v = -v
    if abs(v) < dz: return 0.0
    return ((abs(v) - dz) / (1 - dz)) * (1 if v > 0 else -1)
def pot(chs, i, inv, lo, hi): return lo + (applyv(chn(chs, i), inv, 0.0) + 1) / 2 * (hi - lo)
def sw_on(chs, i): return i >= 0 and chn(chs, i) > 0.3
def r2(v): return round(v * 2) / 2

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="http://hexapod.local:8000")
    ap.add_argument("--port", default="/dev/ttyS5")
    ap.add_argument("--baud", type=int, default=400000)
    ap.add_argument("--rate", type=float, default=15.0)
    ap.add_argument("--monitor", action="store_true")
    args = ap.parse_args()

    ser = serial.Serial(args.port, args.baud, timeout=0.01)
    sess = requests.Session()
    def send(cmd):
        try: sess.post(args.host + "/command", json=cmd, timeout=0.4)
        except Exception: pass

    buf = bytearray(); latest = None
    last_send = 0.0; last_frame = 0.0
    walking = False; gait_idx = -1; last_pose = ""
    print(f"CRSF auf {args.port} @ {args.baud}  ->  {args.host}")
    while True:
        data = ser.read(256)
        if data:
            buf += data
            while len(buf) >= 2:
                length = buf[1]
                if length < 2 or length > 62: buf.pop(0); continue
                if len(buf) < length + 2: break
                frame = bytes(buf[:length + 2])
                if crc8(frame[2:length + 1]) == frame[length + 1]:
                    if frame[2] == RC_CHANNELS_PACKED:
                        latest = unpack_channels(frame[3:length + 1]); last_frame = time.time()
                    del buf[:length + 2]
                else:
                    buf.pop(0)
        now = time.time()
        # Failsafe: kein Signal -> Halt
        if walking and last_frame and now - last_frame > 0.5:
            send({"action": "halt"}); walking = False
        if not latest or now - last_send < 1.0 / args.rate:
            if not data: time.sleep(0.002)
            continue
        last_send = now
        chs = latest
        if args.monitor:
            print("  ".join(f"{i}:{norm(v):+.2f}" for i, v in enumerate(chs)))
            continue
        if CH_HALT >= 0 and sw_on(chs, CH_HALT):
            send({"action": "halt"}); walking = False
        if CH_GAIT >= 0:
            gv = chn(chs, CH_GAIT)
            gi = max(0, min(len(GAITS) - 1, round((gv + 1) / 2 * (len(GAITS) - 1))))
            if gi != gait_idx:
                gait_idx = gi; send({"action": "set_gait", "gait": GAITS[gi]})
        tz = pot(chs, CH_TZ, INV_TZ, TZ_MIN, TZ_MAX) if CH_TZ >= 0 else 0.0
        tx = pot(chs, CH_TX, INV_TX, TX_MIN, TX_MAX) if CH_TX >= 0 else 0.0
        ty = pot(chs, CH_TY, INV_TY, TY_MIN, TY_MAX) if CH_TY >= 0 else 0.0
        if sw_on(chs, CH_MODE):
            roll = applyv(chn(chs, CH_VY), INV_VY) * ROLL_MAX
            pitch = applyv(chn(chs, CH_VX), INV_VX) * PITCH_MAX
            yaw = applyv(chn(chs, CH_OMEGA), INV_OMEGA) * YAW_MAX
            if walking: send({"action": "halt"}); walking = False
        else:
            roll = pitch = yaw = 0.0
            vx = applyv(chn(chs, CH_VX), INV_VX) * VX_MAX
            vy = applyv(chn(chs, CH_VY), INV_VY) * VY_MAX
            om = applyv(chn(chs, CH_OMEGA), INV_OMEGA) * OMEGA_MAX
            height = H_MIN + (applyv(chn(chs, CH_HEIGHT), INV_HEIGHT, 0.0) + 1) / 2 * (H_MAX - H_MIN)
            if abs(vx) > 0 or abs(vy) > 0 or abs(om) > 0:
                send({"action": "walk", "vx": r2(vx), "vy": r2(vy), "omega_deg": r2(om),
                      "height": r2(height), "steps": 30, "rate_hz": 40}); walking = True
            elif walking:
                send({"action": "halt"}); walking = False
        key = f"{r2(roll)},{r2(pitch)},{r2(yaw)},{r2(tx)},{r2(ty)},{r2(tz)}"
        if key != last_pose:
            send({"action": "pose", "roll_deg": r2(roll), "pitch_deg": r2(pitch),
                  "yaw_deg": r2(yaw), "tx": r2(tx), "ty": r2(ty), "tz": r2(tz)})
            last_pose = key

if __name__ == "__main__":
    main()
