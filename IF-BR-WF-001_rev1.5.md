
# Interface Specification  
## Teledyne Wayfinder DVL Integration – BlueROV2 / Navigator + BlueOS  

*Document ID:* **IF‑BR‑WF‑001** *Revision:* **1.5** *Date:* 10 Jun 2025  

---

### 1 Scope  

Integrate a **Teledyne Wayfinder Doppler Velocity Log** with a **BlueROV2** using a **Navigator** flight‑controller and **BlueOS**.  
The DVL’s RS‑232 lines are converted to 3 .3 V TTL by an **Artekit AK‑3232L** breakout; wiring is direct—no DB‑9 connector.  

---

### 2 References  

| Ref | Source | Purpose |
|-----|--------|---------|
| W1 | Wayfinder DVL Integration Guide | OEM wiring/mechanics |
| W2 | Artekit AK‑3232L bottom‑view photo | Pin order confirmation |
| W3 | AK‑3232L datasheet | Electrical limits |
| W4 | Navigator hardware docs | JST‑GH pin‑outs |
| W5 | WetLink penetrator guide | Sealing & torque |

---

### 3 Mechanical Summary  

* **DVL:** mount with 4 × M5 × 6 mm SS screws @ 3 N·m; keep ≥ 150 mm clearance.  
* **AK‑3232L:** secure on two M2 nylon stand‑offs inside electronics tray (LEDs visible).  
* **Penetrator:** Ø 6.5 mm WetLink (LC) with EPDM #006 O‑ring torqued to 3 N·m; passes five‑conductor pigtail.  

---

### 4 Electrical Interfaces  

#### 4.1 Power  

| Device | Voltage | Current | Supply |
|--------|---------|---------|--------|
| Wayfinder DVL | 10 – 28 V | 0.5 A | 14.8 V battery rail |
| AK‑3232L | 3 .3 V | < 10 mA | Navigator 3 .3 V pin |

#### 4.2 Signal Mapping — Artekit AK‑3232L (board oriented per bottom‑view photo)  

| AK‑3232L pad | Pin # | Wayfinder wire | Navigator pin | Direction |
|--------------|-------|---------------|--------------|-----------|
| **RS‑232 J1 IN1** | 5 | **Orange** (`TX`) | — | DVL TX in |
| **TTL J2 OUT1** | 5 | — | JST‑GH Pin 3 `RX` | to Navigator |
| **RS‑232 J1 OUT1** | 4 | **Yellow** (`RX`) | — | DVL RX out |
| **TTL J2 IN1** | 4 | — | JST‑GH Pin 2 `TX` | from Navigator |
| **GND (both headers)** | 1 | **Brown** | JST‑GH Pin 6 GND | Common |
| **TTL J2 VCC** | 6 | — | 3 .3 V aux | Board power |

*Only channel 1 is populated; channel 2 pads remain unconnected. The board LEDs tied to channel 1 blink with traffic.*  

---

### 5 Data & Software  

| Stage | Interface | Rate / Format |
|-------|-----------|---------------|
| DVL → Pi | UART 115 200 bps (Wayfinder binary) |
| Pi → Navigator | MAVLink `VISION_POSITION_DELTA` 10 Hz |

1. **BlueOS:** add `/dev/ttySx` (115 200 bps) under *Serial Devices*; label *Wayfinder*.  
2. Run Teledyne driver container; output to `tcp://127.0.0.1:14777`.  
3. **Navigator params:** `VISO_TYPE = 1`, `EK3_GPS_TYPE = 3`, `VISO_DELAY_MS = 10`; tune `PSC_VELXY_P/I/D`.  

---

### 6 Validation Checklist  

| Phase | Pass criterion |
|-------|---------------|
| Bench | AK‑3232L LEDs blink; `VISION_POSITION_DELTA` in QGC |
| Pool | Drift < 0.02 m s⁻¹ static |
| Sea | **POSHOLD** ±0.5 m; ≤ 2 % track error over 100 m |

---

### 7 Change Control  

Re‑test after firmware or BlueOS upgrades, or if wiring migrates to channel 2.
