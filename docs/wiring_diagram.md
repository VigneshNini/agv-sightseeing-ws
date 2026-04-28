# AGV Sightseeing Vehicle — Wiring Diagram & Electrical Guide

## Electrical Architecture Overview

```
┌─────────────────────────────────────────────────────────────────┐
│                    48V POWER DISTRIBUTION                        │
│                                                                  │
│  LiFePO4 Battery Pack (20 kWh, 48V nominal)                     │
│         │                                                        │
│         ├──→ Main Contactor (relay, GPIO-controlled)            │
│         │         │                                             │
│         │    ┌────▼────────────────────────────────────┐        │
│         │    │         Power Distribution Board        │        │
│         │    │                                         │        │
│         │    ├──→ Roboteq HDC2460 (Drive Motors) 48V  │        │
│         │    ├──→ Dynamixel PRO Servo (Steering) 24V  │        │
│         │    ├──→ DC-DC 48V→24V (LiDAR, Camera) 24V  │        │
│         │    ├──→ DC-DC 48V→12V (Compute) 12V         │        │
│         │    ├──→ DC-DC 48V→5V (Sensors) 5V           │        │
│         │    └─────────────────────────────────────────┘        │
│         │                                                        │
│         └──→ Battery Management System (BMS)                    │
│              (cell balancing, temperature protection)            │
└─────────────────────────────────────────────────────────────────┘
```

## CAN Bus Wiring

```
Jetson Orin ──USB──→ USB-CAN Adapter (Kvaser Leaf Light)
                              │
                         CAN Bus (120Ω termination)
                         500 kbps
                              │
                    ┌─────────┴─────────────┐
                    │                       │
             Roboteq HDC2460          Roboteq HDC2460
             (Left Drive Motor)       (Right Drive Motor)
             CAN ID: 0x01             CAN ID: 0x02
```

### CAN Bus Pinout (DB9 Connector)
| Pin | Signal |
|-----|--------|
| 2   | CAN_L  |
| 7   | CAN_H  |
| 3   | GND    |
| 9   | 12V (optional) |

## Sensor Connections

### Velodyne VLP-16 LiDAR
```
Velodyne ──Ethernet──→ Jetson Orin (eth0, 192.168.1.100)
                        LiDAR IP: 192.168.1.201
                        Data port: 2368 (UDP)
                        Telemetry: 8308 (UDP)
Power: 12V, 8W (Molex 2-pin)
```

### u-blox ZED-F9P RTK GPS
```
ZED-F9P ──USB──→ Jetson Orin (/dev/ttyUSB0)
               Baud: 115200
               Protocol: UBX + NMEA
               NTRIP corrections: via WiFi/4G
Antenna: Active, TNC connector, roof-mounted
Power: 5V via USB
```

### VectorNav VN-100 IMU
```
VN-100 ──RS232──→ USB-Serial (/dev/ttyUSB1)
               Baud: 115200
               Protocol: VNYMR output
Connector: DE-9 (RS232)
Power: 3.3V–5.5V, <1W
```

### Intel RealSense D435i
```
RealSense ──USB 3.0──→ Jetson Orin (/dev/video0)
                       USB-C to USB-A cable (max 1m)
Power: 900mA @ 5V via USB
```

## GPIO Wiring (Jetson Orin)

| GPIO Pin | Signal | Direction | Description |
|----------|--------|-----------|-------------|
| 17       | BUMPER_FRONT | IN (pullup) | Front bumper contact |
| 18       | BUMPER_REAR  | IN (pullup) | Rear bumper contact |
| 16       | ESTOP_OUT    | OUT | Hardware E-stop relay |
| 20       | ULTRASONIC_0_TRIG | OUT | Ultrasonic front-left trigger |
| 21       | ULTRASONIC_0_ECHO | IN  | Ultrasonic front-left echo |
| 22       | ULTRASONIC_1_TRIG | OUT | Ultrasonic front-center trigger |
| 23       | ULTRASONIC_1_ECHO | IN  | Ultrasonic front-center echo |
| 24       | ULTRASONIC_2_TRIG | OUT | Ultrasonic front-right trigger |
| 25       | ULTRASONIC_2_ECHO | IN  | Ultrasonic front-right echo |
| 26       | ULTRASONIC_3_TRIG | OUT | Ultrasonic rear-center trigger |
| 27       | ULTRASONIC_3_ECHO | IN  | Ultrasonic rear-center echo |
| 15       | STATUS_LED_R | OUT | Status LED red |
| 13       | STATUS_LED_G | OUT | Status LED green |
| 11       | STATUS_LED_B | OUT | Status LED blue |

## Emergency Stop Wiring

```
E-Stop Button (NC contact)
      │
      ├──→ GPIO 16 (ESTOP_OUT) → Relay Coil (+)
      │                          Relay Coil (-) → GND
      │                          
      │         Relay contacts:
      │         NC ──→ Motor Enable Line
      │         COM ──→ +12V
      │         
      └──→ Hardware interlock (series with contactor coil)

When E-Stop pressed:
  1. GPIO goes HIGH → relay opens → motors disabled
  2. Software detects button state change
  3. /agv/emergency_stop published
  4. All nodes enter safe state
```

## Dynamixel PRO Steering Servo

```
Dynamixel PRO ──RS485──→ USB2Dynamixel ──USB──→ Jetson Orin
                          /dev/ttyUSB2
                          Protocol: Dynamixel 2.0
                          Baud: 1000000
                          ID: 1
Power: 24V, via dedicated 24V rail
Encoder: 1,048,575 counts/rev (20-bit)
```

## Network Topology

```
                    ┌─────────────────┐
                    │  4G/LTE Router  │
                    │  (optional)     │
                    └───────┬─────────┘
                            │ WAN
                    ┌───────▼─────────┐
                    │  Managed Switch │
                    │  (TP-Link TL-   │
                    │  SG108)         │
                    └─┬──┬──┬────┬───┘
                      │  │  │    │
              Jetson  │  │  │    │ LiDAR
              Orin    │  │  │    │ 192.168.1.201
           .100       │  │  │    │
                      │  │  │
                      │  │  Passenger
                      │  │  Display WiFi
                      │  │
                      │  Operator Laptop
                      │  (SSH, rviz2)
                      │
                    WiFi AP (2.4GHz + 5GHz)
                    SSID: AGV_CONTROL
                    (for web dashboard access)
```

## Wire Gauge Guidelines

| Circuit | Voltage | Max Current | Gauge |
|---------|---------|-------------|-------|
| Battery to contactor | 48V | 200A | 4 AWG |
| Contactor to Roboteq | 48V | 100A each | 6 AWG |
| DC-DC converters | 48V | 20A | 12 AWG |
| 24V rail (steering) | 24V | 10A | 14 AWG |
| 12V rail (LiDAR) | 12V | 5A | 18 AWG |
| CAN bus | 5V | <1A | 24 AWG (twisted pair) |
| GPIO signals | 3.3V | <50mA | 26 AWG |
| USB cables | 5V | <2A | USB spec |

## Fusing

| Circuit | Fuse Rating |
|---------|------------|
| Main battery | 250A (ANL) |
| Drive motors (each) | 100A (Midi) |
| 24V rail | 20A (Blade) |
| 12V rail | 15A (Blade) |
| 5V rail | 10A (Blade) |
| CAN bus power | 1A (mini) |

## Grounding

- Star grounding topology: all grounds meet at single bus bar
- Battery negative → main ground bus bar
- Motor controllers → ground bus bar (heavy gauge)
- Compute → ground bus bar (isolated from motor ground via DC-DC)
- Chassis → ground bus bar (single point bond)

## Safety Standards Compliance

- ISO 13849: Safety-related control systems (PLd, Cat 2)
- IEC 61508: Functional safety
- IP54 minimum for all exterior components
- All HV wiring in orange conduit (48V = HV in EV standards)
- Warning labels on all HV connectors
