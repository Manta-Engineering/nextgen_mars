
# Interface Specification  
## Teledyne Wayfinder DVL Integration – BlueROV2 / Navigator + BlueOS  

*Document ID:* **IF‑BR‑WF‑001**  
*Revision:* **1.3**  
*Date:* 10 Jun 2025  

---

### 1 Scope  

Integrate a **Teledyne Wayfinder Doppler Velocity Log** with a **BlueROV2** that uses the **Navigator** and **BlueOS**, replacing the vendor USB dongle with an **Artekit AK‑3232L** RS‑232↔TTL breakout. citeturn0search1  

---

### 2 References  

| Ref | Title / Link | Use |
|-----|--------------|-----|
| W1 | Wayfinder DVL Integration Guide | OEM wiring |
| W2 | Artekit AK‑3232/AK‑3232L Manual | Pin‑out citeturn1view0 |
| W3 | AK‑3232L Schematics | Channel labels citeturn3view0 |
| W4 | Navigator Hardware Guide | JST‑GH pins |
| W5 | WetLink Penetrator Guide | Sealing |

---

### 3 Hardware Overview  

```
Wayfinder DVL  (RS‑232) <─┐           ┌─> Navigator RX (JST‑GH Pin 3)
   Orange  (TX) ──────────┤ RS232_IN1 │
   Yellow  (RX) ─ RS232_OUT1 ├─────────┐
   Brown   (GND) ───────────┴─ GND     └─> Navigator GND (Pin 6)
                                   ^
                         TTL pads IN1/OUT1 on AK‑3232L
```

Only channel 1 of the **AK‑3232L** is populated; VCC = 3 .3 V from Navigator.

---

### 4 Mechanical Interface  

| Component | Location | Fixings / Torque |
|-----------|----------|------------------|
| DVL head | Port‑side skid | 4 × M5 × 6 mm SS @ 3 N·m |
| AK‑3232L | Inside electronics tray | 2 × M2 × 6 mm nylon standoffs |
| Cable ingress | WetLink Ø 6.5 mm | 3 N·m (W5) |

---

### 5 Electrical & Signal Mapping  

| Function | Wayfinder wire | AK‑3232L pad | Navigator JST‑GH | Note |
|----------|---------------|--------------|------------------|------|
| DVL TX | Orange | RS232_IN1 | RX – Pin 3 | RS‑232 → TTL |
| DVL RX | Yellow | RS232_OUT1 | TX – Pin 2 | TTL → RS‑232 |
| Ground | Brown | GND | GND – Pin 6 | Common |
| VCC (3 .3 V) | — | VCC | Navigator 3 .3 V out | Power AK‑3232L |

The AK‑3232L’s VCC pin accepts 3 – 5.5 V; LEDs provide traffic indication.

---

### 6 Data Path  

| Stage | Interface | Rate | Format |
|-------|-----------|------|--------|
| DVL → Pi | UART | 115 200 bps | Wayfinder binary |
| Pi → Navigator | MAVLink | 10 Hz | `VISION_POSITION_DELTA` |

---

### 7 Software Setup  

1. **BlueOS** → *Serial Devices*: register `/dev/ttySx` @ 115 200 baud.  
2. Run Teledyne driver; pipe to `tcp://127.0.0.1:14777`.  
3. **Navigator**: set `VISO_TYPE=1`, `EK3_GPS_TYPE=3`, `VISO_DELAY_MS=10`, and adjust `PSC_VELXY_*` gains.

---

### 8 Validation  

* LEDs on AK‑3232L flash on activity.  
* QGC shows `VISION_POSITION_DELTA`.  
* Hold test: drift < 0.02 m s⁻¹; open‑water POSHOLD ±0.5 m.

---

### 9 Change Control  

Re‑test after firmware or BlueOS upgrades; update mapping if alternate converter channels are used.
