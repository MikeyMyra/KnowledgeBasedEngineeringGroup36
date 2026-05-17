# UAV Initial Sizing Tool — Operating Manual

Knowledge Based Engineering (KBE) assignment, TU Delft, Q3-Q4  
Built with the [ParaPy](https://www.parapy.nl/) KBE framework.

---

## Table of Contents

1. [Overview](#1-overview)
2. [Project Structure](#2-project-structure)
3. [Installation & Launch](#3-installation--launch)
4. [GUI Navigation](#4-gui-navigation)
5. [Required Mission Inputs](#5-required-mission-inputs)
6. [Optional / Override Inputs](#6-optional--override-inputs)
7. [Payload Roles](#7-payload-roles)
8. [Automated Design Process](#8-automated-design-process)
9. [Available Actions](#9-available-actions)
10. [Reading the Outputs](#10-reading-the-outputs)
11. [Performance Margin Summary](#11-performance-margin-summary)
12. [Stability Report](#12-stability-report)
13. [Known Limitations & Tips](#13-known-limitations--tips)

---

## 1. Overview

This tool performs **initial (conceptual-level) sizing** of a fixed-wing UAV from a small set of mission requirements. Given cruise speed, altitude, range, endurance, and a mission role, it automatically:

- selects engine type (Piston / Turboprop / Jet) from Mach number and altitude;
- infers UAV class (small / medium / large) from four independent constraint floors;
- sizes the aircraft weight (MTOW, fuel, empty, payload) via the Breguet / weight-fraction method;
- produces a W/P – W/S design-point diagram and selects thrust and wing loading;
- sizes the wing planform (area, AR, taper, sweep) and tail surfaces;
- builds a full 3-D geometry in the ParaPy viewer;
- reports longitudinal static stability (static margin);
- identifies which requirement — range or endurance — is the limiting driver, and calculates the achievable value of the non-limiting metric.

---

## 2. Project Structure

```
Assignment/
├── Pythonfiles/
│   ├── main.py                          ← entry point
│   ├── Drone.py                         ← top-level class (all user inputs)
│   ├── mission.py                       ← Breguet sizing & W/P-W/S constraints
│   ├── ISA_calculator.py                ← ISA atmosphere model
│   ├── WP_WS_diagram.py                 ← design-point constraint diagram
│   ├── metric_imperial_conversions.py
│   ├── Components/
│   │   ├── Aircraft.py                  ← geometry assembly
│   │   ├── Fuselage/
│   │   │   └── Fuselage.py
│   │   ├── Liftingsurfaces/
│   │   │   └── Liftingsurface.py        ← wing & tail geometry + aero
│   │   └── Payload/
│   │       ├── Payload.py               ← payload geometry
│   │       ├── Payloadrules.py          ← UAV-class inference & role mapping
│   │       └── payload_library.json     ← sensor/weapon specifications
├── Outputfiles/                         ← auto-generated PNGs & PDFs land here
└── README.md                            ← this file
```

---

## 3. Installation & Launch

### Prerequisites

```
Python ≥ 3.9
ParaPy (licensed)
numpy, scipy, matplotlib
reportlab          (PDF export)
xfoil              (optional — for wing airfoil analysis)
```

Install Python dependencies:

```bash
pip install numpy scipy matplotlib reportlab
```

### Running the tool

From the repository root:

```bash
cd Assignment
python Pythonfiles/main.py
```

The ParaPy GUI opens automatically. The 3-D viewer and the attribute tree appear on the left; the input panel on the right. The default configuration is:

| Input               | Default value |
|---------------------|---------------|
| cruise_speed        | 80 m/s        |
| mission_altitude    | 6 000 m       |
| mission_range       | 500 km        |
| mission_endurance   | 8 hr          |
| payload_role        | Strike        |
| weapon_count        | 6             |

---

## 4. GUI Navigation

- **Input panel** (right side): shows all `Input` fields. Hover over any field to read the tooltip (`doc=` string) which contains the valid range, units, and engineering basis.
- **Attribute tree** (left side): browse computed attributes (MTOW, fuel weight, wing area, etc.). Click any attribute to force recomputation and display the result.
- **3-D viewer**: shows the assembled drone geometry. Use scroll-wheel to zoom, left-drag to rotate, middle-drag to pan.
- **Actions toolbar** (top): buttons for the five user-triggered analyses — see Section 9.

Changing any `Input` field immediately invalidates all downstream attributes; ParaPy recomputes them lazily when you access them or trigger an action.

---

## 5. Required Mission Inputs

All inputs are validated on entry. ParaPy raises a `ValueError` immediately if an out-of-range value is entered, so the GUI will not accept infeasible inputs.

### `cruise_speed` — Cruise true airspeed [m/s]

**Valid range:** 10 – 350 m/s

Engine type is inferred from the resulting Mach number and altitude:

| Altitude band       | Jet threshold (Mach) |
|---------------------|---------------------|
| h ≤ 9 000 m         | M ≥ 0.40            |
| 9 000 < h ≤ 15 000 m| M ≥ 0.50            |
| h > 15 000 m        | M ≥ 0.60            |

Below the jet threshold: Turboprop if High Endurance objective or h > 4 500 m; Piston otherwise.

### `mission_altitude` — Cruise / loiter altitude [m]

**Valid range:** 0 – 20 000 m

Practical engine ceilings:

| Engine type | Practical ceiling |
|-------------|-------------------|
| Piston      | ≤ 4 500 m         |
| Turboprop   | ≤ 9 000 m         |
| Jet         | up to 20 000 m    |

Above 9 000 m a jet is selected automatically when the Mach threshold is met. At very high altitudes (> 15 000 m) the wing area required by thin-air conditions grows significantly; the AR and span are automatically capped to prevent wing / tail geometric overlap.

### `mission_range` — Total mission range (out + return) [km]

**Valid range:** 1 – 25 000 km

Each cruise leg = range / 2 in the Breguet equation. Drives UAV-class floor:

| Range         | Class floor |
|---------------|-------------|
| < 150 km      | small       |
| 150 – 500 km  | medium      |
| > 500 km      | large       |

### `mission_endurance` — Loiter / on-station duration [hr]

**Valid range:** 0.1 – 120 hr

Drives UAV-class floor:

| Endurance    | Class floor |
|--------------|-------------|
| < 4 hr       | small       |
| 4 – 10 hr    | medium      |
| > 10 hr      | large       |

Also drives mission objective: endurance > 6 hr → "High Endurance" → Turboprop preferred.

---

## 6. Optional / Override Inputs

### `payload_role` — Mission role (dropdown)

See Section 7 for all roles and their sensor suites.

### `weapon_count` — Number of munitions [integer]

**Valid range:** 0 – 6  
Set to 0 for unarmed configurations (weapon category suppressed from payload).  
Requires `payload_role = Strike` or `SEAD` to carry weapons.

### `uav_class_override` — Hard-override UAV class [string or None]

Accepts: `"small"`, `"medium"`, `"large"`, or `None` (default — infer from constraints).  
When set, the class inference is bypassed entirely. Use for "what-if" studies only.

### `mission_objective_override` — Hard-override mission objective [string or None]

Accepts: `"High Speed"`, `"High Endurance"`, `"Low cost"`, or `None` (default — infer).  
Bypasses engine-type inference.

### `fuselage_cylinder_start` — Nosecone / cylinder junction [% of fuselage length]

**Valid range:** 5 – 30 %  
Default: 10 %. Payload bay begins just aft of this station.

### `fuselage_cylinder_end` — Cylinder / tail-cone junction [% of fuselage length]

**Valid range:** 50 – 95 %  
Default: 70 %. Fuel tank sits between the payload bay and this point.

### `fuel_type` — Fuel selection [string]

Default: `"auto"` — selects Avgas for Piston, Jet-A for Turboprop/Jet.  
Override with a key from `fuel_properties.json` if a specific fuel is required.

### `fuel_tank_aspect_ratio` — Tank length-to-diameter ratio [—]

**Valid range:** 1.1 – 10.0  
Default: 3.0 (compact wing-box tank). Increase to 5+ for slender HALE fuselages.

### `wing_taper_ratio` — λ = c_tip / c_root [—]

**Valid range:** 0.20 – 1.00  
Default: 0.40 (Raymer subsonic endurance UAV default). Lower values increase tip wash-out and reduce induced drag.

### `wing_sweep_le` — Leading-edge sweep angle [°]

**Recommended range:** 0 – 30 °  
Default: 5 °. Values above ~30 ° push the swept wing tip far aft and may cause visual overlap with the horizontal tail at high aspect ratios. Subsonic UAVs: 0 – 10 °; transonic jets: 15 – 30 °.

---

## 7. Payload Roles

Select from the dropdown in the GUI. The tool automatically assembles the correct sensor / weapon suite and sizes the payload bay.

| Role        | Payload suite                                |
|-------------|----------------------------------------------|
| ISR         | EO/IR camera + radar + datalink              |
| Strike      | EO/IR camera + weapons + datalink            |
| SEAD        | EO/IR camera + radar + weapons + datalink    |
| Mapping     | EO/IR camera + LiDAR                         |
| COMMS relay | Communications relay + datalink              |
| Patrol      | EO/IR camera + communications                |

The specific sensor or weapon variant (size, mass) is chosen automatically based on the inferred UAV class (small / medium / large). A `flight_computer` and `battery` are always added regardless of role.

---

## 8. Automated Design Process

When any input changes, the following chain runs automatically (lazy re-evaluation):

```
Mission inputs
    │
    ├─ ISA atmosphere (T, p, ρ, a) at mission_altitude
    │
    ├─ PayloadRules
    │       ├─ UAV class  (max of 4 independent constraint floors)
    │       ├─ Mission objective  (Breguet / engine inference)
    │       └─ Payload config  (category → variant per class)
    │
    ├─ Payload geometry & mass
    │
    ├─ Mission (Breguet sizing loop)
    │       ├─ Weight fractions: taxi → climb → cruise → loiter → reserve → landing
    │       ├─ MTOW iteration (Raymer empty-weight fraction power law)
    │       ├─ W/P – W/S constraint diagram
    │       └─ Performance margins (achievable range / endurance beyond requirement)
    │
    └─ Aircraft geometry
            ├─ Fuselage  (Roskam length, fineness ratio)
            ├─ Wing planform  (area, AR, taper, sweep)
            ├─ Horizontal tail  (volume coefficient method)
            ├─ Vertical tail
            └─ Longitudinal CG & neutral point → static margin
```

UAV class is determined by the **most demanding single constraint** (range, altitude, endurance, or payload mass). No averaging — one demanding requirement alone is sufficient to push the class up.

---

## 9. Available Actions

Click these buttons in the top toolbar to trigger on-demand analyses.

### Show Design Point

Displays the W/P – W/S (thrust/power loading vs. wing loading) constraint diagram. The design point is the intersection of all active constraints — stall speed, cruise, landing, takeoff, climb rate, climb gradient, and load factor. The diagram is also saved as a timestamped PNG in `Outputfiles/`.

### Run Wing Airfoil Sweep

Runs XFoil across a range of angles of attack for the root airfoil of the main wing. Requires XFoil to be installed and on the system PATH. Results appear in a pop-up plot.

### Plot Wing XFoil Polars

Plots the Cl–alpha and Cd–alpha polars for the wing root airfoil using XFoil. Saved as a timestamped PNG in `Outputfiles/`.

### Print Stability Report

Prints a longitudinal stability summary to the console. Reports:

- CG location (% MAC)
- Neutral point location (% MAC)
- Static margin (%)
- Stability status (Stable / Marginal / Unstable)

### Export PDF Report

Generates a full design summary PDF in `Outputfiles/`. The report includes:

- Mission parameters table
- Weight budget (MTOW, empty, fuel, payload, fractions)
- Wing & aerodynamic sizing (area, span, AR, L/D, loading)
- Engine information (type, thrust or power loading)
- Longitudinal stability summary
- Embedded W/P – W/S design-point diagram

---

## 10. Reading the Outputs

Key attributes shown in the ParaPy attribute tree:

| Attribute              | Description                                               |
|------------------------|-----------------------------------------------------------|
| `MTOW`                 | Maximum take-off weight [kg]                              |
| `fuel_weight`          | Required fuel mass [kg]                                   |
| `empty_weight`         | Structural + systems empty weight [kg]                    |
| `payload_weight`       | Installed payload mass [kg]                               |
| `wing_area`            | Reference wing area [m²]                                  |
| `wing_semi_span`       | Half-span [m] (full span = 2 ×)                           |
| `wing_aspect_ratio`    | Effective AR used for geometry (may differ from Roskam)   |
| `wing_loading`         | W/S at MTOW [kg/m²]                                       |
| `thrust_loading`       | T/W at MTOW [—] (jet) or W/P [kg/W] (prop)               |
| `ld_cruise`            | Cruise lift-to-drag ratio [—]                             |
| `engine_type`          | Inferred: "Piston", "Turboprop", or "Jet"                 |
| `uav_class`            | Inferred: "small", "medium", or "large"                   |
| `mission_objective`    | Inferred: "Low cost", "High Endurance", or "High Speed"   |
| `static_margin`        | (NP − CG) / MAC [%] — positive = stable                  |
| `stability_status`     | "Stable", "Marginal", or "Unstable"                       |
| `performance_margins_summary` | Formatted string showing limiting driver and achievable non-limiting value |

---

## 11. Performance Margin Summary

Every time the fuel sizing runs, a performance margin analysis is printed to the console and stored in `performance_margins_summary`. Example output:

```
═══════════════════════════════════════════════════════════════
 PERFORMANCE MARGINS
═══════════════════════════════════════════════════════════════
 Required:   range 500.0 km   |   endurance  8.0 hr
 ─────────────────────────────────────────────────────────────
 Range  fuel fraction  : 0.412     Endurance fuel fraction : 0.318
 ─────────────────────────────────────────────────────────────
 LIMITING driver : RANGE  (consumes more fuel)
 With the range-sized fuel load:
   Achievable endurance : 10.3 hr    (required: 8.0 hr  → +28.7%)
═══════════════════════════════════════════════════════════════
```

Interpretation: the aircraft is sized for range (it is the harder requirement). Given the fuel actually loaded, the aircraft could loiter 10.3 hr — well beyond the 8 hr requirement. The endurance requirement has margin; range does not.

If endurance is the harder requirement, the output flips — it shows the achievable range with the endurance-sized fuel load.

---

## 12. Stability Report

The stability report (printed by "Print Stability Report" action) uses the tail volume coefficient method:

- **CG position**: computed from a weighted sum of fuselage, wing, fuel, payload, and tail component CGs.
- **Neutral point**: estimated using the tail volume coefficient and lift-curve slopes.
- **Static margin**: SM = (NP − CG) / MAC × 100 %

Stability thresholds:
- SM > 5 %: Stable
- 0 % < SM ≤ 5 %: Marginal (consider active stability augmentation)
- SM ≤ 0 %: Unstable

---

## 13. Known Limitations & Tips

**Wing / tail geometric overlap at extreme altitudes**  
At altitudes above ~15 000 m, thin air requires very large wings. The AR and span are automatically capped (span ≤ 4 × fuselage length; AR ≤ 16) to prevent the horizontal tail from geometrically overlapping the wing. If you see `[Wing AR]` messages in the console, this cap is active. Consider reducing altitude or increasing cruise speed to select a jet engine (which has lower wing loading).

**Fuselage length vs. wing chord**  
The wing root chord is capped at 40 % of the fuselage cylinder length (Raymer §4.2). If the Roskam AR gives too large a chord, AR is automatically increased. The `[Wing AR]` console message reports when this adjustment fires.

**Engine type transition**  
Changing `cruise_speed` near the Mach threshold (≈ 0.40 at sea level) will flip the engine type, which changes the specific fuel consumption and Breguet equation form — MTOW can change discontinuously. This is physically correct behaviour.

**Large weapon_count without Strike/SEAD role**  
If you set `weapon_count > 0` but leave `payload_role = ISR` (or another non-armed role), the weapons are included in the payload mass but the role categories do not include `"weapon"` — set the role to Strike or SEAD explicitly.

**Circular dependency — wing AR in Mission vs. geometry**  
The Mission sizing object uses `_wing_ar_roskam` (pure empirical AR, independent of fuselage geometry). The geometry then adjusts AR upward if needed. This means the Breguet sizing uses a slightly lower AR than the displayed geometry; the difference is conservative (fuel is slightly over-sized).

**XFoil actions**  
XFoil must be installed separately and available on the system PATH. The airfoil sweep and polar actions will print an error to the console if XFoil is not found.

**Output files**  
All generated PNGs and PDFs are saved to `Assignment/Outputfiles/` with a timestamp in the filename. The folder must exist before exporting; create it if absent.
