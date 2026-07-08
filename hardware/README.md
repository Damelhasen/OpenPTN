# MATEKSYS H743-Slim — ArduPilot Setup + Live IMU Dashboard

This documents the full process of getting a Matek H743-Slim flight controller
(factory-flashed with Betaflight) running ArduCopter and streaming live
attitude/GPS telemetry into a custom Python 3D dashboard (`instaimu.py`).

---

## 1. Hardware background

- Board: **Matek H743-Slim** (STM32H743 MCU)
- Ships from the factory with **Betaflight preloaded**, not ArduPilot — this
  is the root cause of most "no heartbeat" / "no MAVLink" issues on a fresh
  board.
- USB enumerates as an **STMicroelectronics Virtual COM Port** while running
  Betaflight (or any firmware using the STM32 USB CDC-ACM driver).
- Status LEDs: solid Red = power (3.3V rail), and normal ArduPilot boot state
  is typically a **flashing blue LED** (green/red solid, blue blinking) which
  means "disarmed, no GPS lock" — this is healthy, not a fault. A repeating
  **red-solid + blue/green alternating 3x** pattern specifically indicates the
  gyro/accelerometer wasn't detected on boot — that *is* a fault.

---

## 2. Windows driver / port setup

1. Plug the board into your PC via USB.
2. Open **Device Manager → Ports (COM & LPT)**.
3. Look for **STMicroelectronics Virtual COM Port (COMx)** — that's your board.
   Ignore Bluetooth serial links and unrelated management ports (e.g. Intel AMT).
4. If you get `OSError(22, 'The semaphore timeout period has expired.')` when
   opening the port:
   - Try a different USB cable (the #1 cause — many cables are charge-only)
   - Try a different USB port (avoid hubs/front-panel headers if possible)
   - Confirm nothing else (Mission Planner, another script, Arduino IDE) has
     the port open already

To list all serial ports and confirm which one is your board:
```powershell
py -c "import serial.tools.list_ports; [print(p.device, '-', p.description) for p in serial.tools.list_ports.comports()]"
```

---

## 3. Flashing ArduPilot (from factory Betaflight)

Since the board has no ArduPilot bootloader yet, Mission Planner's normal
"Install Firmware" flow can't talk to it directly. Instead, flash it at the
raw STM32 DFU level:

### 3.1 Download firmware
Get **`arducopter_with_bl.hex`** (bundles the ArduPilot bootloader + Copter
firmware in one file — needed for this first-time flash) from:
```
https://firmware.ardupilot.org/Copter/stable/MatekH743/
```
> Note: `MatekH743` may not always appear in Mission Planner's built-in board
> list depending on your Mission Planner version — the firmware server always
> has it directly, so this manual download path is the reliable option.

### 3.2 Enter DFU mode
1. Unplug USB
2. Hold the board's **BOOT** button (or bridge the BOOT pad per Matek's pinout
   if your revision doesn't have a physical button)
3. Plug in USB while still holding BOOT
4. Release after ~2 seconds
5. Device Manager should now show **"DFU in FS Mode"** under Universal Serial
   Bus devices (not a COM port anymore)

### 3.3 Driver for DFU mode (Zadig)
If STM32CubeProgrammer doesn't detect the board over USB:
1. Open **Zadig**
2. Select the **DFU** device from the dropdown (distinct from the STM32
   Virtual COM Port — don't touch that one)
3. Driver: **WinUSB**
4. Click **Replace Driver** (or **Install Driver**)

> ⚠️ Only use Zadig/WinUSB on the **DFU** device. Never replace the driver on
> the STM32 Virtual COM Port entry — that one needs to stay on `usbser` or
> Mission Planner/serial tools will lose it entirely.

### 3.4 Flash with STM32CubeProgrammer
Download: https://www.st.com/en/development-tools/stm32cubeprog.html

1. Open STM32CubeProgrammer, select **USB** as connection type, click
   **Connect** (should detect the board in DFU mode)
2. Go to **Erasing & Programming** tab
3. Browse to `arducopter_with_bl.hex`
4. Check **"Run after programming"** (jumps straight to the new firmware
   instead of requiring a manual power cycle)
5. Click **Start Programming**
6. Wait for a success message — the board will disconnect automatically once
   it starts running the new firmware

### 3.5 Confirm the flash
Check Device Manager again — you should now see an **ArduPilot** USB
composite device exposing (at least) two COM ports:
- One labeled **MAVLink** — use this one
- One labeled **SLCAN** — ignore this (CAN-bus passthrough, unrelated)

---

## 4. Connecting in Mission Planner

1. Select the **MAVLink** COM port (not SLCAN) in the top-right dropdown
2. Baud: **115200**
3. Click **Connect**
4. It should grab a heartbeat and pull parameters — this is the point where
   it finally works if you were stuck on Betaflight firmware before
5. Run through initial setup: frame type (e.g. Quad X), accelerometer
   calibration, compass calibration

If you still get `Sequence contains no elements` here, it almost always means
the FC isn't sending valid MAVLink yet — re-check the flash succeeded and the
correct (MAVLink, not SLCAN) port is selected.

---

## 5. Wireless connection options

Once wired setup works, to go wireless:

| Method | Range | Notes |
|---|---|---|
| **ESP8266/ESP32 WiFi bridge** | ~tens of m | Cheapest; wire to a spare UART, flash ArduPilot's WiFi bridge firmware, connect via UDP/TCP in Mission Planner |
| **Bluetooth (HC-05/HC-06)** | ~10m | Wire to spare UART, pairs as a normal Bluetooth COM port — works with existing scripts unchanged |
| **SiK radio (RFD900, HolyBro TX/RX)** | km-scale | Standard for actual flight telemetry; one radio on the FC, one on the PC via USB |
| **ExpressLRS/CRSF "Backpack"** | radio-link range | If your Jumper T16 runs ELRS or TBS Crossfire, an ELRS Backpack module taps the CRSF MAVLink passthrough and forwards it over WiFi to Mission Planner — no separate telemetry radio needed |

Check your T16's RF protocol under **Model Setup → Internal/External RF** —
CRSF/ELRS supports the Backpack route above; FrSky (D16/ACCESS) does not
support full MAVLink passthrough and would need one of the other methods.

---

## 6. Live IMU + GPS dashboard (`instaimu.py`)

A PyVista/Qt dashboard that renders a live 3D flying-wing model driven by
MAVLink `ATTITUDE` data, plus a GPS satellite counter and basic servo readout.

### 6.1 Install dependencies
```powershell
pip install pyvista pyvistaqt PyQt5 pymavlink pyserial numpy
```
> If you see `pyvista/pyvistaqt not installed` even after installing them,
> it usually means the Qt binding is missing — `pip install PyQt5` fixes it,
> since `pyvistaqt` needs an actual Qt backend to load, not just the package.

### 6.2 Run
```powershell
py instaimu.py
```
(or the full interpreter path if `py`/`python` aren't on PATH — see
Troubleshooting below)

### 6.3 Using it
- Pick your port from the dropdown (Refresh Ports if it's not listed) and hit
  **Connect**
- **Lock Yaw (Anti-Drift)**: locks the current yaw as a zero-reference so
  displayed yaw is relative, counteracting compass drift
- **Zero IMU (C)**: resets that same yaw reference on demand (also bound to
  the `C` key)
- Right panel shows live Roll/Pitch/Yaw, GPS satellite count + fix type, and a
  servo Left/Right readout (currently a placeholder elevon mix of pitch±roll
  — replace with your actual control mixing once wired to real surfaces)

### 6.4 How the smoothness/rate works
- Requests ATTITUDE @ 50Hz and GPS_RAW_INT @ 5Hz from the FC via
  `MAV_CMD_SET_MESSAGE_INTERVAL` (falls back to the older
  `REQUEST_DATA_STREAM` for old firmware)
- Dead-reckons orientation between MAVLink messages using the gyro rates
  already included in every `ATTITUDE` message, so the render loop (60fps via
  a Qt timer) always has a fresh angle rather than looking jittery between
  telemetry updates
- Rendering is GPU-accelerated via PyVista/VTK, not CPU-redrawn like
  matplotlib, so it holds a smooth frame rate

---

## 7. Troubleshooting quick reference

| Symptom | Likely cause / fix |
|---|---|
| `python was not found... Microsoft Store` | Windows Store alias shadowing real install — use `py` launcher, full interpreter path, or disable the alias in Settings → Apps → Advanced app settings → App execution aliases |
| `Sequence contains no elements` in Mission Planner | No valid MAVLink coming from the FC — usually still running Betaflight, or wrong COM port selected |
| `OSError(22, 'semaphore timeout...')` opening a port | Bad/charge-only USB cable, wrong port, or port already in use elsewhere |
| Port shows "DFU in FS Mode" instead of a COM port | Board is in DFU bootloader mode — expected during flashing, not for normal use |
| No heartbeat after flashing | Confirm you flashed `_with_bl.hex` (not the non-bootloader variant) and power-cycled after flashing |
| Two COM ports appear after flashing (MAVLink + SLCAN) | Normal — always connect to the one labeled **MAVLink** |
| `pyvista/pyvistaqt not installed` despite pip showing them installed | Missing Qt binding — `pip install PyQt5` |

---

## 8. File manifest

- `instaimu.py` — live IMU/GPS dashboard (PyVista + Qt), described in section 6
