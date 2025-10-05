# Copilot Instructions for VSSS-MeuGit

## Project Overview
This repository contains code for a robotics system used in the IEEE Very Small Size Soccer (VSSS) league. It integrates:
- Real robot control (ESP32 firmware, remote control, and Python communication)
- Simulation environments (CoppeliaSim, TraveSim)
- Computer vision and field drawing utilities
- GUI for launching robot processes

## Key Components
- `Corobeu/`: Python code for robot control, vision, and communication. Includes:
  - `Corobeu_ideal.py`, `Corobeu + MediaPipe.py`: Main robot logic and vision integration
  - `configs/config.py`: Centralized robot IPs, IDs, and team settings
  - `DrawField.py`: Field visualization using matplotlib
- `ControleRemoto/`: Arduino/ESP32 code for manual robot control
- `ESP-32/main/`: ESP32 firmware for motor and sensor control
- `CoppeliaSim/codigos/`: Python code for simulation with CoppeliaSim, including low-level API wrappers
- `TRAVESim/`: Python client for interacting with TraveSim and FIRASim simulators
- `gui.py`: Tkinter GUI to launch robot scripts and manage processes

## Developer Workflows
- **Python virtual environments** are recommended for all Python projects. See `TRAVESim/README.md` for setup.
- **Robot IPs and roles** are configured in `Corobeu/configs/config.py`. Always import from this file for consistency.
- **Robot communication** uses TCP sockets, sending 4-byte packed values for speed/direction (see `send_speed` in `sendData.ipynb`).
- **Simulation**: Use `CoppeliaSim/codigos/main.py` to connect and control robots in CoppeliaSim. The API is wrapped in `sims/sim.py`.
- **GUI**: Use `gui.py` to launch robot scripts. It tracks running processes and prevents duplicates.

## Patterns and Conventions
- **Robot scripts** are launched via the GUI or manually. Each robot has a dedicated Python file (e.g., `CorobeuARES.py`).
- **Socket communication**: Always use little-endian byte order for robot commands.
- **Field drawing**: Use `DrawField.py` for consistent field visualization.
- **Simulation API**: All CoppeliaSim calls are wrapped via `sims/sim.py` for portability.
- **PID parameters**: Sent as text strings over TCP (see `send_pid_param` in `sendData.ipynb`).

## Integration Points
- **External dependencies**: Google protobuf, matplotlib, pandas, CoppeliaSim remote API.
- **Robot IPs**: Defined in `Corobeu/configs/config.py` and used throughout for communication.
- **Simulators**: TraveSim and FIRASim clients in `TRAVESim/`.

## Example: Sending Speed to Robot
```python
from configs.config import IP_ARES
send_speed(150, 150, 1, 1, IP_ARES)
```

## Example: Launching a Robot via GUI
```python
# In gui.py
run_robot('ares')
```

## Example: Connecting to CoppeliaSim
```python
import sims.sim as sim
clientID, robot, MotorE, MotorD, ball = connect_CRB(port)
```

---
If any section is unclear or missing, please provide feedback for further refinement.