# UAV Initial Sizing Tool — KBE Assignment
**TU Delft MSc FPP · Knowledge Based Engineering · Q3/Q4**

A knowledge-based ParaPy application for the conceptual sizing and 3-D geometry generation of fixed-wing UAVs. The tool sizes the aircraft from mission requirements (speed, altitude, range, endurance, payload role) using Breguet/Raymer/Roskam empirical methods, then builds a parametric 3-D model in the ParaPy viewer.

---

## Quick Start

### 1. Launch the application

Open a terminal in the `Assignment/` root folder (one level above `Pythonfiles/`) and run:

```bash
python Pythonfiles/main.py
```

This starts the ParaPy GUI with a pre-configured `Drone` instance. All inputs can be changed live in the GUI sidebar.

### 2. Root object

The root class is **`Drone`**, defined in `Pythonfiles/Drone.py`.  
`main.py` instantiates it and passes it to `parapy.gui.display()`.

### 3. Required inputs

These four inputs are mandatory and have no default values — they must be set either in `main.py` or in the GUI sidebar:

| Input | Unit | Valid range | Description |
|---|---|---|---|
| `cruise_speed` | m/s | 10 – 350 | True airspeed at cruise |
| `mission_altitude` | m | 0 – 18 000 | Cruise / loiter altitude above MSL |
| `mission_range` | km | 1 – 25 000 | Total mission range (outbound + return) |
| `mission_endurance` | hr | 0.1 – 120 | Loiter endurance |

### 4. Key optional inputs

These have Roskam/Raymer defaults and can be left unchanged or overridden in `main.py`:

| Input | Default | Description |
|---|---|---|
| `payload_role` | `ISR` | Mission role: ISR / Strike / SEAD / Mapping / COMMS relay / Patrol |
| `weapon_count` | `0` | Number of munitions (0 = unarmed) |
| `wing_naca_input` | `'0012'` | NACA 4-digit code for the main wing (e.g. `'2412'`) |
| `fuel_type` | `'auto'` | `'auto'` \| `'avgas_100ll'` \| `'jet_a'` \| `'jp8'` \| `'lipo_battery'` |
| `fuel_tank_aspect_ratio` | `3.0` | Tank length-to-diameter ratio (1.1 – 10) |
| `wing_taper_ratio` | `0.40` | Wing taper ratio λ = c_tip / c_root (0.20 – 1.00) |
| `fuselage_cylinder_start` | `10.0` | Nosecone / cylinder junction [% fuselage length] |
| `fuselage_cylinder_end` | `70.0` | Cylinder / tail-cone junction [% fuselage length] |

Engine type (Piston / Turboprop / Jet) is derived automatically from cruise Mach number and altitude using Roskam Vol. I §3.2 rules.

---

## Input Files

The application expects an `Inputfiles/` directory at the **working directory** from which you launch the script (i.e. `Assignment/`):

```
Assignment/
├── Inputfiles/
│   └── Airfoils/        ← .dat files are auto-generated here on first run
├── Outputfiles/         ← PDFs, plots, and sweep results written here
├── Pythonfiles/
│   └── ...
└── Q3D/                 ← Q3D MATLAB toolbox (see MATLAB note below)
```

**Airfoil `.dat` files** are generated automatically into `Inputfiles/Airfoils/` the first time a NACA code is used. You do not need to provide them manually. The default NACA 0012 file is created on startup.

No other input files need to be loaded manually before running.

---

## GUI Actions

The following actions are available in the ParaPy sidebar / right-click menu on the `Drone` object:

| Action label | What it does |
|---|---|
| **Show Design Point** | Plots the W/P–W/S diagram and saves a PNG to `Outputfiles/` |
| **Show V-n Diagram** | Plots the V-n manoeuvrability envelope |
| **Run Wing Airfoil Sweep** | Searches NACA 4-series space via Q3D + XFoil; updates wing geometry with the best candidate |
| **Plot Wing XFoil Polars** | Runs XFoil on the current wing airfoil and plots Cl–α and Cl/Cd–α |
| **Print Stability Report** | Prints a static stability summary to the console |
| **Export PDF Report** | Generates a full PDF sizing report in `Outputfiles/` |

### Wing airfoil workflow

1. Type a NACA 4-digit code into the `wing_naca_input` field in the GUI (e.g. `2412`). The `.dat` file is generated and the 3-D wing geometry updates immediately.
2. Alternatively, click **Run Wing Airfoil Sweep** to let the tool search for an aerodynamically optimal NACA 4-series airfoil. This overwrites `wing_naca_input` with the best result.
3. First digit (camber) is capped at 6 — codes with camber 7–9 are rejected with a dialog, as XFoil diverges in that range.

### Infeasibility handling

If the mission is infeasible (fuel fraction ≥ 1, or MTOW > 100 t) a warning dialog appears and the 3-D aircraft geometry is suppressed. Reduce range, endurance, or cruise speed to recover a feasible design.

---

## Software Requirements

### Python

Python **3.12** is required (matching the ParaPy installation).

### Python packages

| Package | Version tested | Install |
|---|---|---|
| `parapy` | *(license-locked — provided by TU Delft)* | see ParaPy install guide |
| `numpy` | ≥ 1.26 | `pip install numpy` |
| `matplotlib` | ≥ 3.8 | `pip install matplotlib` |
| `matlab` (engine) | matching MATLAB R2023b+ | see MATLAB section below |

Standard library modules used (no install needed): `os`, `sys`, `math`, `subprocess`, `json`, `ast`, `glob`, `shutil`, `datetime`, `enum`, `warnings`, `typing`, `dataclasses`, `itertools`.

### ParaPy

ParaPy is a commercial KBE framework licensed through TU Delft. Install it using the installer and licence key provided by the course.  
Documentation: [https://www.parapy.nl](https://www.parapy.nl)

The application uses:
- `parapy.core` — `Input`, `Attribute`, `Part`, `action`, `child`, `validate`
- `parapy.geom` — `GeomBase`, `LoftedSolid`, `Box`, `Cylinder`, `Circle`, `FittedCurve`, `RevolvedSolid`, `Polygon`, `LineSegment`, `translate`, `rotate`, `Vector`, `Point`
- `parapy.gui` — `display`

### XFoil

XFoil **6.99** is included in the repository at `XFOIL6.99/xfoil.exe`. No separate installation is needed. XFoil is called as a subprocess automatically when running the airfoil sweep or polar plot actions.

- XFoil runs are limited to Mach ≤ 0.70 (the panel-method validity limit). A warning dialog appears if the cruise Mach exceeds this threshold.
- XFoil must be run from a working directory that contains the `Inputfiles/Airfoils/` folder (i.e. launch `main.py` from `Assignment/`).

### MATLAB and Q3D

The **Wing Airfoil Sweep** action uses **Q3D**, a vortex-lattice / lifting-line aerodynamic solver that runs inside MATLAB.

Requirements:
- **MATLAB R2023b** (or later) installed and licensed on the machine.
- **MATLAB Engine for Python** configured for the same Python version (3.12). Follow MathWorks' guide: [Call MATLAB from Python](https://www.mathworks.com/help/matlab/matlab_external/install-the-matlab-engine-for-python.html)
- The **Q3D toolbox** must be present in `Assignment/Q3D/` (included in the repository).

`Pythonfiles/Matlab_start.py` starts a shared MATLAB engine at import time and changes its working directory to `Q3D/`. If MATLAB is not configured, importing `Drone.py` will raise an error at startup. Comment out the import in `Liftingsurface.py` to run without Q3D (the sweep action will be unavailable).

---

## Project Structure

```
Pythonfiles/
├── main.py                          # Entry point — instantiates Drone and launches GUI
├── Drone.py                         # Root KBE object; all top-level inputs and actions
├── Matlab_start.py                  # Starts the shared MATLAB engine for Q3D
├── metric_imperial_conversions.py   # Unit conversion helpers
├── generate_uml.py                  # (utility) generates UML class diagram
└── Components/
    ├── Aircraft.py                  # Assembles wing, tail, fuselage, engine, fuel tank
    ├── Fuselage/
    │   ├── Fuselage.py              # Parametric fuselage geometry
    │   └── Undercarriage.py         # Landing gear
    ├── Liftingsurfaces/
    │   ├── Liftingsurface.py        # Wing / tail planform + airfoil sweep action
    │   ├── Airfoil.py               # Airfoil geometry, XFoil interface, .dat I/O
    │   └── Wingbox.py               # Structural wingbox geometry
    ├── Engines/
    │   ├── Engine.py                # Base engine class
    │   ├── PropellerEngine.py       # Piston / turboprop nacelle and propeller
    │   └── JetEngine.py             # Jet nacelle
    ├── Fuel/
    │   └── FuelTank.py              # Fuel tank geometry and sizing
    ├── Payload/
    │   ├── Payload.py               # Payload library and 3-D box geometry
    │   └── Payloadrules.py          # Engineering rules: UAV class, mission objective
    └── Mission/
        ├── mission.py               # Breguet / Raymer / Roskam sizing equations
        ├── WP_WS_diagram.py         # Thrust/weight – wing loading diagram
        ├── ISA_calculator.py        # International Standard Atmosphere
        └── vn_diagram.py            # V-n manoeuvrability envelope
```

---

## Typical Design Feasibility Limits

The sizing model uses UAV-representative empty-weight fraction caps (derived from real UAV data) to bound the Raymer sizing equation:

| Engine type | We/W₀ cap | Max fuel fraction | Approx. max range (no endurance) | Approx. max endurance (no range) |
|---|---|---|---|---|
| Piston | 0.58 | ~0.42 | ~1 000 – 1 500 km | ~6 – 10 hr |
| Turboprop | 0.52 | ~0.48 | ~2 000 – 3 000 km | ~12 – 18 hr |
| Jet | 0.50 | ~0.50 | ~3 000 – 5 000 km | ~10 – 15 hr |

Combining range and endurance simultaneously reduces both limits. These are approximate figures at typical UAV L/D values (8 – 14) from the W/P–W/S diagram.

---

## Notes

- All geometry is built in **SI units** (metres, kilograms, seconds). Imperial conversions for Roskam equations are handled internally in `metric_imperial_conversions.py`.
- The `Outputfiles/` directory is created automatically if it does not exist.
- Previous output files (design point PNGs, PDF reports) are archived automatically with a timestamp suffix before new ones are written.
