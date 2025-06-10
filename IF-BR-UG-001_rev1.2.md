# Interface Specification  
## Water Linked UGPS G2 with Locator‑A1 – BlueROV2 / Navigator + BlueOS  

*Document ID:* **IF‑BR‑UG‑001**  
*Revision:* **1.4**  
*Date:* 10 Jun 2025  

---

### 1 Scope  
Integrate a **Water Linked Underwater GPS G2** system using **Locator‑A1** with a **BlueROV2** that employs the **Navigator** flight‑controller and **BlueOS**.  

**Key wiring scheme**  
* **Blue / White pair** in the tether is kept for **Fathom‑X PLC** (telemetry, video, general Ethernet).  
* **Green / White pair** in the tether is dedicated end‑to‑end to **Locator‑A1 RS‑485 + 12 V**.  
* Inside the topside FXTI the incoming Green / White is **jumpered to a short Brown / White segment** that terminates at Binder‑770 pins A/B for the UGPS Topside. Colour changes only in this jumper.  
* The **UGPS Topside contains its own PLC circuitry** and therefore joins the vehicle network over the Blue / White pair automatically—no RJ‑45 or OS bridge is required.  

---

### 2 Wiring Overview  
| Segment | Conductor colours | Function | Notes |
|---------|------------------|----------|-------|
| ROV → penetrator → tether | **Green (+), White (–)** | RS‑485 differential + 12 V for Locator‑A1 | From A1 Integration Kit |
| Through tether | **Green / White** | Same RS‑485/12 V | Dedicated, noise‑free |
| PLC pair in tether | **Blue / White** | HomePlug AV (Fathom‑X) | Vehicle Ethernet |
| Inside topside (jumper) | **Green / White → Brown / White** | Colour conversion only | Solder or terminal block |
| Binder‑770 lead | **Brown (+), White (–)** | RS‑485 + 12 V | Pins A/B to UGPS Topside |

---

### 3 Electrical Summary  
| Path | Power | Data |
|------|-------|------|
| Binder A/B ↔ Locator‑A1 | 12 V DC (from Topside) | RS‑485 (Water Linked protocol) |
| PLC Blue / White | 0–60 V DC shared | HomePlug AV Ethernet |

Locator draws < 300 mA; additional tether power < 3 W.  

---

### 4 Mechanical Points  
* **Locator‑A1:** mount to frame rail with kit bracket (2 × M4 × 35 mm SS, 3 N·m).  
* **Cable ingress (ROV):** WetLink Ø 6.5 mm LC, EPDM #006 O‑ring @ 3 N·m.  
* **Topside:** short Green/White → Brown/White jumper inside FXTI; Binder‑770 mates with UGPS Topside enclosure.  

---

### 5 Network & Data Flow  
| Source | Transport | Port / Msg | Destination |
|--------|-----------|-----------|-------------|
| UGPS Topside | HomePlug AV over Blue/White | **UDP 14401** NMEA | BlueOS UGPS extension |
| BlueOS UGPS extension | MAVLink | `GPS_INPUT` 5 Hz | Navigator |
| Navigator | MAVLink | Telemetry | QGroundControl |

---

### 6 Software Configuration  
1. Update BlueOS to ≥ 1.1 and install **Water Linked UGPS** extension (≥ 1.0.4).  
2. Verify `GPS_INPUT` appears in MAVLink Inspector (QGC → Analyze Tools).  
3. Navigator parameters: `GPS_TYPE = 0`; `EK3_GPS_TYPE = 3`.  

---

### 7 Validation Checklist  
| Test | Expected result |
|------|-----------------|
| Power‑up | UGPS Topside reachable on 192.168.2.x; LEDs OK |
| Land | `GPS_INPUT` with HDOP < 5 |
| Tank | Fix retained; depth & heading plausible |
| Sea | Position error ≤ 2 m vs surface GNSS; **POSHOLD** stable |

---

### 8 Change Control  
Re‑validate after firmware/BlueOS updates or if tether pair assignments are modified.
