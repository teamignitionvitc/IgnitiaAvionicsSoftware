# Ignitia CanSat Avionics

Drone-drop CanSat flight computer based on RP2040-Zero.

## Features
- **Drone Drop Mode** - Detects freefall and deploys parachute
- **Sensors**: MPU6050 (IMU), BME280 (barometer), NEO-M8M (GPS)
- **Telemetry**: NRF UART radio + USB debug
- **Actuator**: Servo for parachute deployment

## Pin Assignments (RP2040-Zero)

| Pin | Function | Notes |
|-----|----------|-------|
| GP0 | NRF TX | UART0 TX to NRF radio |
| GP1 | NRF RX | UART0 RX from NRF radio |
| GP4 | I2C SDA | MPU6050 + BME280 |
| GP5 | I2C SCL | MPU6050 + BME280 |
| GP8 | GPS TX | UART1 to NEO-M8M |
| GP9 | GPS RX | UART1 from NEO-M8M |
| GP15 | Servo | PWM output |
| GP16 | LED | Status indicator |

## Pico SDK Setup (Windows)

### 1. Install Prerequisites

```powershell
# Install via winget (Windows 11) or download manually
winget install Kitware.CMake
winget install Ninja-build.Ninja
winget install Arm.GnuArmEmbeddedToolchain
winget install Git.Git
```

Or download manually:
- [CMake](https://cmake.org/download/) (add to PATH)
- [Arm GNU Toolchain](https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads) - `arm-none-eabi-*` version
- [Ninja](https://ninja-build.org/) (optional, makes builds faster)
- [Git](https://git-scm.com/)

### 2. Clone Pico SDK

```powershell
cd C:\
git clone https://github.com/raspberrypi/pico-sdk.git
cd pico-sdk
git submodule update --init
```

### 3. Set Environment Variable

```powershell
# Permanent (run as Admin)
[System.Environment]::SetEnvironmentVariable('PICO_SDK_PATH', 'C:\pico-sdk', 'User')

# Or for current session only
$env:PICO_SDK_PATH = "C:\pico-sdk"
```

### 4. Build the Firmware

```powershell
cd E:\IGNITION\Ignitia_Avionics
mkdir build
cd build
cmake -G "NMake Makefiles" ..
# OR with Ninja (faster)
cmake -G Ninja ..

# Build
cmake --build .
# OR
ninja
```

### 5. Flash to RP2040-Zero

1. Hold BOOTSEL button on RP2040-Zero
2. Connect USB cable
3. Release BOOTSEL - appears as USB drive
4. Copy `build/ignitia_avionics.uf2` to the drive
5. Device reboots automatically

## Flight Sequence

```
IDLE ──► ARMED ──► FREEFALL ──► DEPLOYED ──► LANDED
  │        │          │            │           │
  │        │          │            │           └─ Low velocity at ground
  │        │          │            └─ Parachute opened
  │        │          └─ Drop detected (low g / falling)
  │        └─ Attached to drone, waiting for drop
  └─ Power on, calibrating
```

## Commands (USB Serial)

| Key | Command | Description |
|-----|---------|-------------|
| `a` | Arm | Ready for drop |
| `d` | Disarm | Return to idle |
| `t` | Test | Test servo |
| `s` | Status | Show current state |
| `c` | Calibrate | Recalibrate sensors |
| `r` | Reset | Full reset |
| `h` | Help | Show commands |

## Telemetry Packet Format

### NRF Telemetry (binary)
```c
struct TelemetryPacket {
    uint8_t  header;      // 0xAA
    uint8_t  type;        // 0x01
    uint16_t seq;         // Sequence number
    uint32_t timestamp;   // ms since boot
    uint8_t  state;       // Flight state
    int16_t  altitude;    // decimeters
    int16_t  velocity;    // cm/s
    int16_t  accel;       // milli-g
    int16_t  temperature; // 0.1°C
    uint8_t  gps_sats;
    uint8_t  flags;       // Armed, deployed, etc.
    uint8_t  checksum;    // XOR
};
```

### USB Debug (ASCII)
```
$T,12345,ARMED,150.5,0.3,1.02,0
$G,28.572900,-80.649000,152,8
$S,12345,DROP DETECTED
```

## Wiring Diagram

```
                    RP2040-Zero
                   ┌───────────┐
              3V3 ─┤1        22├─ GND
              GND ─┤2        21├─ GP15 ──► Servo Signal
        NRF TX ◄── GP0 ─┤3        20├─ GP14
        NRF RX ──► GP1 ─┤4        19├─ GP13
                  GP2 ─┤5        18├─ GP12
                  GP3 ─┤6        17├─ GP11
    I2C SDA ◄───► GP4 ─┤7        16├─ GP10
    I2C SCL ◄───► GP5 ─┤8        15├─ GP9 ◄── GPS RX
                  GP6 ─┤9        14├─ GP8 ──► GPS TX
                  GP7 ─┤10       13├─ GP16 ──► LED
                 3V3EN─┤11       12├─ GND
                       └───────────┘

    I2C Bus (GP4/GP5):
    ├── MPU6050 (0x68)
    └── BME280 (0x76)
    
    UART0 (GP0/GP1): NRF Radio
    UART1 (GP8/GP9): GPS NEO-M8M
```

## Safety Features

- **Deployment delay** - 500ms after drop detection
- **Safety timeout** - Force deploy after 30s from drop
- **Altitude check** - Don't deploy below 30m
- **Redundant detection** - Accelerometer + barometer

## Development

### Run Tests (host)
```bash
cd test
gcc -DHOST_TEST test_flight_state.c -o test_flight_state && ./test_flight_state
gcc test_filters.c -lm -o test_filters && ./test_filters
```

### Run SIL Simulation
```bash
cd sil
pip install -r ../requirements.txt
python sil_runner.py test_scenarios/nominal_flight.json
```

## License

MIT License - Ignitia Team
