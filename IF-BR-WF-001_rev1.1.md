
# Interface Specification  
## Teledyne Wayfinder DVL Integration – BlueROV2 / Navigator + BlueOS  

*Document ID:* **IF-BR-WF-001**  
*Revision:* **1.1**  
*Date:* 10 Jun 2025  

---

### Scope  

Integrate a **Teledyne Wayfinder DVL** directly to Navigator (UART, 3.3 V TTL) on a BlueROV2 running BlueOS.  
USB adapters are eliminated; a MAX3232 level shifter is required.

---

### Mechanical Interfaces  

* Wayfinder mounted port‑side with 4 × M5 screws (3 N·m).  
* External cable enters ROV via Ø 6.5 mm **LC BlueRobotics penetrator** (EPDM #006, 3 N·m).

---

### Serial Wiring  

| Signal | Wire colour | To level shifter | Navigator JST‑GH |
|---|---|---|---|
| TX (orange) | RS‑232 Tx → RX‑TTL | Pin 3 (RX‑in) |
| RX (yellow) | RS‑232 Rx ← TX‑TTL | Pin 2 (TX‑out) |
| COM GND (brown) | GND | Pin 6 |
| Power + (red) | 14.8 V rail | — |
| Power – (black) | ROV common | — |

---

### BlueOS Setup  

* **Serial Devices**: `/dev/ttySx` @ 115 200 baud, label *Wayfinder*.  
* Vendor driver forwards bottom‑track → `tcp://127.0.0.1:14777`.  
* Parameters: `VISO_TYPE=1`, `EK3_GPS_TYPE=3`, `VISO_DELAY_MS=10`.

---

### Validation  

1. MAVLink Inspector shows `VISION_POSITION_DELTA` (10 Hz).  
2. Odometry drift < 0.02 m s⁻¹ when static.
