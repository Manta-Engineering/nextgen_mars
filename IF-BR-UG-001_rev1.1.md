
# Interface Specification  
## Water Linked UGPS G2 (Locator‑A1) Integration – BlueROV2 / Navigator + BlueOS  

*Document ID:* **IF-BR-UG-001**  
*Revision:* **1.1**  
*Date:* 10 Jun 2025  

---

### Scope  

Integrate **UGPS G2** with **Locator‑A1** using the modified‑tether kit. UGPS Topside communicates via PLC over tether; no network bridge required.

---

### Mechanical & Penetrators  

* Locator‑A1 fixed to frame rail with kit bracket (2 × M4 × 35 mm, 3 N·m).  
* Green/white tether pair routed through Ø 6.5 mm **LC penetrator** to tether connection PCB.

---

### Data Links  

| Source | Medium | Destination | Format |
|---|---|---|---|
| UGPS Topside | UDP 14401 via PLC | BlueOS extension | NMEA |
| Extension | localhost | Navigator | MAVLink `GPS_INPUT` (5 Hz) |

---

### BlueOS Setup  

* Install **Water Linked UGPS** extension (≥ 1.0.4).  
* Confirm `GPS_INPUT` in MAVLink Inspector.  

---

### Validation  

ROV icon appears in QGroundControl map; position error < 2 m.
