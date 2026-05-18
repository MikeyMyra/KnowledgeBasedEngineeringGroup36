# UAV Initial Sizing Tool — README

Knowledge Based Engineering (KBE) assignment, TU Delft, Q3-Q4  
Built with the [ParaPy](https://www.parapy.nl/) KBE framework.

---

## Prerequisites

```
Python ≥ 3.9
ParaPy (licensed)
numpy, scipy, matplotlib, reportlab
```

Install Python dependencies:

```bash
pip install numpy scipy matplotlib reportlab
```

MATLAB must be installed on the machine and be configured with the Python version of the interpreter of this KBE application

---

## Launching the Tool

From the repository root:

```bash
cd Assignment
python Pythonfiles/main.py
```

The ParaPy GUI opens with the 3-D viewer and the attribute tree. Inputs appear in the attribute tree alongside computed outputs.

---

## Setting Mission Inputs

All inputs are editable in the attribute tree. Required inputs:

| Input | Units | Valid range | Default |
|-------|-------|-------------|---------|
| `cruise_speed` | m/s | 10 – 350 | 80 |
| `mission_altitude` | m | 0 – 15 000 | 6 000 |
| `mission_range` | km | 1 – 25 000 | 500 |
| `mission_endurance` | hr | 0.1 – 120 | 8 |
| `payload_role` | — | ISR / Strike / SEAD / Mapping / COMMS relay / Patrol | ISR |
| `weapon_count` | — | 0 – 6 | 0 |

Optional override inputs:

| Input | Purpose |
|-------|---------|
| `uav_class_override` | Force class to `"small"`, `"medium"`, or `"large"` |
| `mission_objective_override` | Force objective to `"High Speed"`, `"High Endurance"`, or `"Low cost"` |
| `fuselage_cylinder_start` | Nosecone / cylinder junction [% of fuselage length] |
| `fuselage_cylinder_end` | Cylinder / tailcone junction [% of fuselage length] |
| `fuel_type` | Fuel key or `"auto"` |
| `fuel_tank_aspect_ratio` | Tank length-to-diameter ratio |
| `wing_taper_ratio` | λ = c_tip / c_root |

If a value is outside the valid range ParaPy raises a `ValueError` and rejects the input. All downstream attributes recompute lazily when you access them or trigger an action.

---

## Available Actions

### Show Design Point
Plots the W/P – W/S (or T/W – W/S) constraint diagram and saves a timestamped PNG to `Outputfiles/`.

### Show V-n Diagram
Plots the V-n diagram and saves a timestamped PNG to `Outputfiles/`.

### Run Wing Airfoil Sweep
Runs a Q3D sweep over NACA 4-series airfoil parameters to find the best aerodynamic shape. Requires MATLAB.

### Plot Wing XFoil Polars
Plots Cl–α, Cd–α, Cd–m & Cl-Cd polars for the wing root airfoil and saves a PNG to `Outputfiles/`.

### Print Stability Report
Prints a longitudinal stability summary to the console (CG, neutral point, static margin, assessment) and saved a TXT in `Outputfiles/`.

### Export PDF Report
Generates a full design-summary PDF in `Outputfiles/` covering mission parameters, weight budget, performance margins, fuel fractions, wing and tail geometry, fuselage, fuel system, and stability.

### Export STP File
Exports the full 3-D geometry to a STEP file in `Outputfiles/`.

---

## Reading the Results

Key attributes in the ParaPy attribute tree:

| Attribute | Description |
|-----------|-------------|
| `MTOW` | Maximum take-off weight [kg] |
| `fuel_weight` | Required fuel mass [kg] |
| `empty_weight` | Structural + systems empty weight [kg] |
| `payload_weight` | Installed payload mass [kg] |
| `wing_area` | Reference wing area [m²] |
| `wing_semi_span` | Half-span [m] |
| `wing_aspect_ratio` | Effective AR (geometry-adjusted) |
| `wing_loading` | W/S at MTOW [N/m²] |
| `thrust_loading` | T/W [—] (jet only) |
| `power_loading` | W/P [kg/W] (piston/turboprop only) |
| `ld_cruise` | Cruise lift-to-drag ratio [—] |
| `engine_type` | `"Piston"`, `"Turboprop"`, or `"Jet"` |
| `uav_class` | `"small"`, `"medium"`, or `"large"` |
| `mission_objective` | `"Low cost"`, `"High Endurance"`, or `"High Speed"` |
| `static_margin` | (NP − CG) / MAC [—] — positive = stable |
| `stability_status` | `"Stable"`, `"Marginal"`, or `"Unstable"` |
| `performance_margins_summary` | Console string showing sizing driver and achievable off-design value |

---

## Output Files

All generated PNGs and PDFs are saved to `Assignment/Outputfiles/`. Previously generated files of the same type are automatically moved to `Outputfiles/data/` so the root always contains only the latest file. Create the `Outputfiles/` folder if it does not exist.

---

## Payload Roles

| Role | Sensor / weapon suite |
|------|-----------------------|
| ISR | EO/IR camera + radar + datalink |
| Strike | EO/IR camera + weapons + datalink |
| SEAD | EO/IR camera + radar + weapons + datalink |
| Mapping | EO/IR camera + LiDAR |
| COMMS relay | Communications relay + datalink |
| Patrol | EO/IR camera + communications |

The correct sensor and weapon variants are selected automatically based on the inferred UAV class.

---

## Engine & Class Inference (summary)

The tool infers engine type and UAV class automatically from the mission inputs — no manual selection is needed unless you use the override inputs.

**Engine type** is selected from Mach number and altitude:
- M ≥ 0.40 / 0.50 / 0.60 (altitude-graduated threshold) → Jet  
- Endurance > 6 hr or altitude > 4 500 m → Turboprop  
- Otherwise → Piston

**UAV class** is the most demanding of four independent constraint floors: range, altitude, endurance, and payload mass.

---

## Known Limitations

- **Extreme altitudes (> 15 000 m)**: wing AR and span are automatically capped to prevent visual overlap with the horizontal tail. Console messages report when this cap fires.
- **XFoil actions**: require a separate XFoil installation on the system PATH.
- **Engine type transition**: changing `cruise_speed` near the Mach threshold causes a discrete jump in MTOW — this is physically correct behaviour.
- **Large weapon counts on non-armed roles**: set `payload_role` to `Strike` or `SEAD` explicitly when carrying weapons.
