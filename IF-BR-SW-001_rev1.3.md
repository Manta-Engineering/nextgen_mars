
# Software & Control Interface Overview  
## BlueROV2 with Wayfinder DVL & Water Linked UGPS (A1)  

*Document ID:* **IF-BR-SW-001**  
*Revision:* **1.3**  
*Date:* 10 Jun 2025  

---

### 1 Control chain at a glance  

| Layer | Item | Runs where | Board interconnect* | Purpose |
|-------|------|-----------|---------------------|---------|
| Vehicle firmware | **ArduSub 4.1 +** on Navigator | Navigator PCB stacked on Raspberry Pi | **GPIO header (SPI, I²C, and auxiliary UART)** | Attitude, thrust, flight modes |
| Companion middleware | **BlueOS 1.1 – 1.3** | Raspberry Pi 4 | 14550/14552 UDP out, 14401 UDP in, 14777 TCP | MAVLink routing, extensions |
| Position sensors | Wayfinder DVL & UGPS/A1 | Vehicle & topside | Serial (UART) / UDP | Feed EKF (`VISION_POSITION_DELTA`, `GPS_INPUT`) |
| Top‑side C2 | QGroundControl, Cockpit, MAVProxy, MAVSDK | Operator PC | 14550 UDP in | Pilot UI, scripting |
| Robotics middleware (opt.) | **ROS 2** via BlueOS *ROS2 extension* | Pi or topside PC | 14771 UDP, DDS 7400 | Autonomous control |

\* Navigator plugs directly onto the Pi GPIO pins—no USB cable is used.  
The underlying kernel overlay enables the SPI, I²C, ADC, PWM and LED peripherals. 

---

### 2 Top‑side control software landscape  

| Software | Primary use | Notes |
|----------|-------------|-------|
| **QGroundControl** | Manual pilot interface | Joystick wizard; fused GPS shown |
| **Cockpit** | Browser‑based UI | H.264 stream; `MANUAL_CONTROL` |
| **MAVProxy / MAVSDK** | CLI / API scripting | MAVProxy rebroadcasts |
| **BlueOS ROS2 extension** | ROS 2 workspace | Includes MAVROS & Foxglove |

---

### 3 MAVLink data path  

```
Navigator FC ⇄ Raspberry Pi  [SPI / I²C / AUX UART over GPIO header]
                             │
  Wayfinder driver → TCP 14777 ─┤
                             │
 UGPS extension ← UDP 14401 ───┤
                             ▼
                       MAVLink router
                             ▼
                   UDP 14550 → Top‑side
```

The EKF fuses absolute (`GPS_INPUT`) and relative (`VISION_POSITION_DELTA`) data, enabling **POSHOLD** with inertial lock.

---

### 4 ROS 2 control workflow (optional)  

1. **Install** the *BlueOS ROS2 extension* from Marketplace.  
2. Add a **UDP Client 127.0.0.1:14771** endpoint in BlueOS.  
3. Launch MAVROS:  
   ```bash
   ros2 launch mavros apm.launch fcu_url:=udp://:14771@localhost:14771
   ```  
4. Example RC override:  
   ```bash
   ros2 topic pub --once /mavros/rc/override mavros_msgs/msg/OverrideRCIn \     '{channels: [1500, 1500, 1500, 1500, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]}'
   ```  
   Set `SYSID_MYGCS=1` if controlling from an external GCS.

---

### 5 Verification checklist  

| Stage | Expected outcome | Tool |
|-------|------------------|------|
| Bench | Heartbeat 14550; `GPS_INPUT` & `VISION_POSITION_DELTA` present | QGC MAVLink Inspector |
| Tank | `LOCAL_POSITION_NED` drift < 0.02 m s⁻¹ static | QGC graph |
| Open water | **POSHOLD** ±0.5 m; UGPS vs GNSS error < 2 m | QGC map |
| ROS 2 | `/tf` steady; latency < 40 ms | `ros2 topic echo`, Foxglove |

---

### 6 Summary of recommended setup  

1. Use **QGroundControl** or **Cockpit** as the main pilot interface.  
2. Introduce **MAVSDK** or **ROS 2** only for scripted autonomy or integration.  

[^upgrade]: Always re‑validate after BlueOS or ArduSub upgrades; GPIO overlays and endpoint defaults may change.
