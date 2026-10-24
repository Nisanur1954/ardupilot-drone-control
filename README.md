# ArduPilot UAV Control Panel

A Python-based graphical control application for an ArduPilot-powered unmanned aerial vehicle (UAV). The application communicates with the autopilot over **MAVLink** using **PyMAVLink** and provides a **Tkinter** interface for monitoring telemetry and issuing flight commands.

> **Project status:** Developed and tested in an ArduPilot SITL / Gazebo simulation workflow. This is a development and educational project, not a flight-certified control system.

## Features

- MAVLink connection to an ArduPilot vehicle through a configurable endpoint (default: `127.0.0.1:14550`)
- Live flight-mode, arming, altitude, local NED position, and landed-state displays
- Guided takeoff workflow: switch to `GUIDED`, arm, then send a takeoff command
- `LAND` and `RTL` operator controls
- Square-flight mission with configurable altitude, side length, starting corner, and corner wait time
- Mission cancellation and emergency-state reset, subject to safety checks
- Background operations to keep the GUI responsive while waiting for MAVLink operations

## Technology Stack

| Component | Technology |
| --- | --- |
| Language | Python 3 |
| GUI | Tkinter |
| Autopilot | ArduPilot (ArduCopter) |
| Communication | MAVLink via PyMAVLink |
| Simulation | ArduPilot SITL and Gazebo Harmonic |
| MAVLink routing | MAVProxy |
| Development OS | Ubuntu 22.04 |

## Project Structure

```text
ardupilot-drone-control/
├── main.py               # Tkinter graphical user interface
├── drone_controller.py   # MAVLink, telemetry, and flight control logic
├── README.md             # Project documentation
└── .gitignore            # Excluded local/generated files
```

## Prerequisites

- Python 3 and Tkinter
- PyMAVLink
- A running ArduPilot vehicle or ArduPilot SITL instance
- MAVLink telemetry forwarded to the endpoint configured in the GUI

On Ubuntu, install Tkinter if it is not already available:

```bash
sudo apt update
sudo apt install python3-tk python3-venv
```

Create a virtual environment and install PyMAVLink:

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install pymavlink
```

## Running the Application

1. Start ArduPilot SITL and the Gazebo simulation using your existing simulation setup.
2. Ensure MAVProxy or another MAVLink router forwards vehicle telemetry to `127.0.0.1:14550` (or update the address in the GUI).
3. Open a terminal in the project directory and run:

   ```bash
   python3 main.py
   ```

4. Select **Connect** and confirm that the connection and telemetry are active.
5. Use **TAKEOFF**, **LAND**, or **RTL**, or configure and start a square-flight mission.

### Square Mission

The square-flight mission accepts:

- **Altitude (m):** target takeoff altitude
- **Side length (m):** length of each side of the square
- **Corner wait (s):** pause at each corner
- **Starting corner:** `K1`, `K2`, `K3`, or `K4`

The mission is designed to take off, navigate the selected square route, return to its starting area, and land. Actual execution depends on valid positioning data and ArduPilot flight readiness.

## Telemetry and Safety Notes

- Telemetry freshness checks help distinguish recent data from stale measurements.
- Flight commands may be rejected when MAVLink connectivity, flight mode, or vehicle state does not meet the controller's checks.
- **LAND/RTL mode confirmation does not guarantee that landing is complete.** Verify the vehicle state before starting another flight.
- Test all changes in **SITL/Gazebo** before considering use with physical hardware.
- Always follow applicable aviation rules and maintain independent safety measures for real-world operations.

## Future Improvements

- More detailed connection and telemetry diagnostics
- Flight logging and mission history
- Additional waypoint and mission-planning options
- Multi-UAV/swarm control integration

## License

No license has been selected yet. All rights are reserved by default until a license is added.
