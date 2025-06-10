
# Software & Control Interface Overview  
## BlueROV2 with Wayfinder DVL & Water Linked UGPS (A1)  

*Document ID:* **IF-BR-SW-001**  
*Revision:* **1.0**  
*Date:* 10 Jun 2025  

---

### 1 Control Chain  

Vehicle firmware (ArduSub 4.1 +) → BlueOS 1.1‑1.3 (MAVLink router & extensions) → Top‑side C2 (QGC, Cockpit, MAVSDK) → optional ROS 2 via BlueOS extension.

---

### 2 MAVLink Path  

```
Navigator FC <--USB--> Pi (BlueOS)
            ├─ Tcp 14777  (Wayfinder driver)
            ├─ UDP 14401 (UGPS NMEA)
            └─ UDP 14550 to topside
```

EKF fuses `GPS_INPUT` + `VISION_POSITION_DELTA` enabling **POSHOLD**.

---

### 3 ROS 2 (optional)  

Install BlueOS ROS2 extension, add UDP Client 127.0.0.1:14771, then:  

```bash
ros2 launch mavros apm.launch fcu_url:=udp://:14771@localhost:14771
```

Publish overrides or missions via standard ROS topics.

---

### 4 Verification Checklist  

| Stage | Check |
|---|---|
| Bench | `GPS_INPUT` & `VISION_POSITION_DELTA` visible |
| Tank | Drift < 0.02 m s⁻¹ |
| Open water | **POSHOLD** within ±0.5 m |

---

### Summary  

1. Use **QGC** or **Cockpit** for piloting.  
2. Introduce **MAVSDK** or **ROS 2** only for scripted autonomy.  

[^upgrade]: Validate after BlueOS/ArduSub upgrades as endpoint defaults may change.
