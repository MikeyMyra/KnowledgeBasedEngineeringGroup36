"""
engineering_rules.py
====================
Derives UAV class, mission objective, and payload variant selection
from mission performance inputs and payload categories.

Design philosophy
-----------------
UAV class is determined by the MOST DEMANDING single constraint,
not by averaging or scoring.  Each constraint (range, altitude,
endurance, payload mass) independently sets a minimum class floor,
and the final class is the maximum across all floors.

This means:
  - A short-endurance, high-payload drone correctly lands in "large"
  - A long-range, light-payload drone correctly lands in "medium"
  - No single weak constraint can pull the class down

Typical call sequence
---------------------
    rules = PayloadRules(
        cruise_speed      = 80,
        mission_altitude  = 6000,
        mission_range     = 500,
        mission_endurance = 8,
        payload_categories = ["eo_ir", "radar"],
        weapon_count      = 0,
    )
    rules.uav_class           # → "medium"
    rules.mission_objective   # → "High Endurance"
    rules.payload_config      # → [("flight_computer", ...), ("battery", ...), ...]
"""

import enum
import math
import warnings
from dataclasses import dataclass
from typing import Optional

from Pythonfiles.Components.Payload.Payload import PAYLOAD_LIBRARY, resolve_model


# ---------------------------------------------------------------------------
# PAYLOAD ROLE ENUM  — drives the GUI dropdown in Drone.payload_role
#
# Inheriting from (str, enum.Enum) means every value IS a plain string, so
# existing code that reads payload_role as a string continues to work without
# any changes.  ParaPy detects the Enum base class and renders a dropdown
# automatically.
# ---------------------------------------------------------------------------

class PayloadRole(str, enum.Enum):
    ISR         = "ISR"
    Strike      = "Strike"
    SEAD        = "SEAD"
    Mapping     = "Mapping"
    COMMS_relay = "COMMS relay"
    Patrol      = "Patrol"


# ---------------------------------------------------------------------------
# CLASS ORDERING  — used to take the max across constraint floors
# ---------------------------------------------------------------------------

_CLASS_RANK = {"small": 0, "medium": 1, "large": 2}
_RANK_CLASS = {0: "small", 1: "medium", 2: "large"}
ROLE_CATEGORIES = {
    "ISR":         ["eo_ir", "radar", "datalink"],
    "Strike":      ["eo_ir", "weapon", "datalink"],
    "SEAD":        ["eo_ir", "radar", "weapon", "datalink"],
    "Mapping":     ["eo_ir", "lidar"],
    "COMMS relay": ["comms", "datalink"],
    "Patrol":      ["eo_ir", "comms"],
}


def _max_class(*classes: str) -> str:
    """Return the most demanding (highest-rank) class string."""
    return _RANK_CLASS[max(_CLASS_RANK[c] for c in classes)]


# ---------------------------------------------------------------------------
# PER-CONSTRAINT CLASS FLOORS
# ---------------------------------------------------------------------------
# Each function returns the minimum UAV class required by ONE constraint.
# The final class is _max_class() across all active floors.
#
# Range floor
# -----------
# < 150 km  : Group I–II line-of-sight systems (small)
#             e.g. senseFly eBee X: 90 km range, 1.6 kg MTOW
# 150–500 km: Group III MALE territory (medium)
#             e.g. Bayraktar TB2: ~300 km radius, ~650 kg MTOW
# > 500 km  : BLOS SATCOM, large fuel fraction required (large)
#             e.g. MQ-9 Reaper: ~1 850 km, 4 760 kg MTOW
# Source: NATO STANAG 4671; Teal Group World UAV Forecast 2023.
#
# Altitude floor
# --------------
# < 3 000 m : small fixed/rotary-wing, piston-viable
# 3–8 000 m : requires turbocharged/turboprop → Group III (medium)
#             e.g. Hermes 450: 5 500 m ceiling, 550 kg MTOW
# > 8 000 m : HALE territory (large)
#             e.g. Global Hawk: 18 000 m, 14 628 kg MTOW
# Source: FAR 103; Jane's All the World's Aircraft.
#
# Endurance floor
# ---------------
# Endurance is the weakest discriminator alone (a lightweight solar
# drone can achieve 20 h at small-class MTOW), so thresholds are
# deliberately conservative.
# < 4 h  : small class
# 4–10 h : medium (MALE class rule of thumb)
# > 10 h : large (fuel fraction alone drives MTOW up significantly)
# Source: Watts et al., RAND (2012) "The Military Use of UAVs".
#
# Payload mass floor
# ------------------
# Strongest discriminator when payload mass is known.
# Assuming ~15 % payload fraction (Raymer §3, conservative for ISR UAVs):
#   payload < 3 kg  → MTOW < ~20 kg   → small
#   3–90 kg         → MTOW 20–600 kg  → medium
#   > 90 kg         → MTOW > 600 kg   → large
# ---------------------------------------------------------------------------

def _class_from_range(mission_range: float) -> str:
    if mission_range < 150:
        return "small"
    elif mission_range < 500:
        return "medium"
    else:
        return "large"


def _class_from_altitude(mission_altitude: float) -> str:
    if mission_altitude < 3_000:
        return "small"
    elif mission_altitude < 8_000:
        return "medium"
    else:
        return "large"


def _class_from_endurance(mission_endurance: float) -> str:
    if mission_endurance < 4:
        return "small"
    elif mission_endurance < 10:
        return "medium"
    else:
        return "large"


def _class_from_payload_mass(payload_mass_kg: float) -> str:
    if payload_mass_kg < 3:
        return "small"
    elif payload_mass_kg < 90:
        return "medium"
    else:
        return "large"


def infer_uav_class(
    mission_range:     float,
    mission_altitude:  float,
    mission_endurance: float,
    payload_mass_kg:   Optional[float] = None,
) -> str:
    """
    Return the minimum UAV class satisfying ALL active constraints.

    Each constraint sets an independent class floor; the result is the
    most demanding floor.  Payload mass is the strongest discriminator
    when available; mission params act as proxies before it is known.
    """
    floors = [
        _class_from_range(mission_range),
        _class_from_altitude(mission_altitude),
        _class_from_endurance(mission_endurance),
    ]
    if payload_mass_kg is not None:
        floors.append(_class_from_payload_mass(payload_mass_kg))

    return _max_class(*floors)


# ---------------------------------------------------------------------------
# MISSION OBJECTIVE INFERENCE
# ---------------------------------------------------------------------------
# Mirrors Mission.engine_selection() so the inferred string is always
# consistent with the propulsion type Mission will later select.
#
# Jet      : Mach > ~0.4 at altitude  (≈ 120 m/s conservative threshold)
#            → "High Speed"
# Turboprop: subsonic, endurance-driven (> 6 h)
#            → "High Endurance"
# Piston   : subsonic, short endurance, cost-sensitive
#            → "Low cost"
#
# Source: Raymer "Aircraft Design: A Conceptual Approach" ch. 3.
# ---------------------------------------------------------------------------

def infer_mission_objective(
    cruise_speed:      float,
    mission_endurance: float,
) -> str:
    if cruise_speed > 120:
        return "High Speed"
    elif mission_endurance > 6:
        return "High Endurance"
    else:
        return "Low cost"


# ---------------------------------------------------------------------------
# VARIANT SELECTION
# ---------------------------------------------------------------------------

def select_payload_variant(category: str, uav_class: str) -> str:
    """
    Pick the default variant for *category* / *uav_class* using the
    existing default_for mechanism in PAYLOAD_LIBRARY.
    """
    sub = PAYLOAD_LIBRARY.get(category)
    if sub is None:
        raise KeyError(f"Unknown payload category: '{category}'")

    for key, entry in sub.items():
        if uav_class in entry.get("default_for", []):
            return key

    # No class-specific default — use first entry with a warning
    first = next(iter(sub))
    warnings.warn(
        f"No default variant for '{category}' / class '{uav_class}'. "
        f"Falling back to: '{first}'.",
        UserWarning, stacklevel=2,
    )
    return first


# ---------------------------------------------------------------------------
# MANDATORY CATEGORIES  — always included regardless of user input
# ---------------------------------------------------------------------------

_MANDATORY = ["flight_computer", "battery"]


# ---------------------------------------------------------------------------
# MAIN RULES CLASS
# ---------------------------------------------------------------------------

@dataclass
class PayloadRules:
    """
    Single source of truth for all inferred design parameters.

    Required
    --------
    cruise_speed, mission_altitude, mission_range, mission_endurance

    Optional
    --------
    payload_categories : list[str]
        Categories the user wants, e.g. ["eo_ir", "radar", "weapon"].
        The system picks the right variant per category based on UAV class.
        If None → inferred from mission parameters.

    weapon_count : int
        Number of munitions (0 = unarmed; suppresses "weapon" category).

    uav_class_override, mission_objective_override : str
        Hard overrides — bypass inference entirely for that parameter.
    """

    cruise_speed:       float
    mission_altitude:   float
    mission_range:      float
    mission_endurance:  float

    payload_categories: Optional[list] = None
    weapon_count:       int = 0

    uav_class_override:         Optional[str] = None
    mission_objective_override: Optional[str] = None

    # ------------------------------------------------------------------
    # Step 1 — resolve category list
    # Must happen before UAV class because payload mass is a class input.
    # ------------------------------------------------------------------

    @property
    def _active_categories(self) -> list:
        raw = self.payload_categories if self.payload_categories is not None else self._infer_categories()

        cats = []
        for item in raw:
            if item in ROLE_CATEGORIES:              # it's a role string → expand it
                for cat in ROLE_CATEGORIES[item]:
                    if cat not in cats:
                        cats.append(cat)
            else:                                    # it's already a category
                if item not in cats:
                    cats.append(item)

        if self.weapon_count == 0 and "weapon" in cats:
            cats.remove("weapon")

        result = [m for m in _MANDATORY if m not in cats]
        result.extend(cats)
        return result

    def _infer_categories(self) -> list:
        """
        Infer payload categories from mission parameters alone.

          EO/IR    — always (basic situational awareness)
          radar    — endurance > 6 h  (persistent ISR)
          datalink — range > 500 km   (BLOS operation)
          comms    — range ≤ 500 km   (line-of-sight)
          weapon   — weapon_count > 0
        """
        cats = ["eo_ir"]

        if self.mission_endurance > 6:
            cats.append("radar")

        if self.mission_range > 500:
            cats.append("datalink")
        else:
            cats.append("comms")

        if self.weapon_count > 0:
            cats.append("weapon")

        return cats

    # ------------------------------------------------------------------
    # Step 2 — estimate payload mass using a 'small' variant as proxy
    #
    # We resolve variants against "small" here (lower-bound estimate).
    # If actual payload mass in a medium/large variant is higher, that
    # only pushes the class further up — the conservative direction.
    # ------------------------------------------------------------------

    @property
    def _estimated_payload_mass(self) -> float:
        total = 0.0
        for cat in self._active_categories:
            try:
                key   = select_payload_variant(cat, "small")
                entry = PAYLOAD_LIBRARY[cat][key]
                if entry["geometry_type"] == "box":
                    vol = (entry["length"]
                           * entry["width"]
                           * entry["height_box"])
                else:
                    vol = (math.pi / 4
                           * entry["diameter"] ** 2
                           * entry["height_cyl"])
                mass = vol * entry["density"]
                if cat == "weapon":
                    mass *= max(self.weapon_count, 1)
                total += mass
            except Exception:
                pass  # unknown/custom category — skip silently
        return total

    # ------------------------------------------------------------------
    # Step 3 — UAV class
    # ------------------------------------------------------------------

    @property
    def uav_class(self) -> str:
        if self.uav_class_override:
            return self.uav_class_override
        return infer_uav_class(
            mission_range=self.mission_range,
            mission_altitude=self.mission_altitude,
            mission_endurance=self.mission_endurance,
            payload_mass_kg=self._estimated_payload_mass,
        )

    # ------------------------------------------------------------------
    # Step 4 — mission objective
    # ------------------------------------------------------------------

    @property
    def mission_objective(self) -> str:
        if self.mission_objective_override:
            return self.mission_objective_override
        return infer_mission_objective(self.cruise_speed, self.mission_endurance)

    # ------------------------------------------------------------------
    # Step 5 — payload config with final UAV class
    # ------------------------------------------------------------------

    @property
    def payload_config(self) -> list:
        """[(category, model_key), ...] ready to pass to Payload()."""
        return [
            (cat, select_payload_variant(cat, self.uav_class))
            for cat in self._active_categories
        ]

    # ------------------------------------------------------------------
    # Diagnostic summary — shows every constraint floor explicitly
    # ------------------------------------------------------------------

    def summarise(self):
        est_mass = self._estimated_payload_mass
        print("=" * 65)
        print("ENGINEERING RULES SUMMARY")
        print("=" * 65)
        print(f"  Mission inputs:")
        print(f"    cruise speed  : {self.cruise_speed} m/s")
        print(f"    altitude      : {self.mission_altitude} m")
        print(f"    range         : {self.mission_range} km")
        print(f"    endurance     : {self.mission_endurance} hr")
        print()
        print(f"  Constraint floors (each independent):")
        print(f"    range         → {_class_from_range(self.mission_range)}")
        print(f"    altitude      → {_class_from_altitude(self.mission_altitude)}")
        print(f"    endurance     → {_class_from_endurance(self.mission_endurance)}")
        print(f"    payload mass  → {_class_from_payload_mass(est_mass)}"
              f"  (est. {est_mass:.1f} kg)")
        print(f"  ─────────────────────────────────────────────")
        print(f"  UAV class      → {self.uav_class}  "
              f"{'(overridden)' if self.uav_class_override else '(most demanding floor)'}")
        print(f"  Mission obj.   → {self.mission_objective}  "
              f"{'(overridden)' if self.mission_objective_override else '(inferred)'}")
        print()
        print(f"  Payload config:")
        for cat, model in self.payload_config:
            label = PAYLOAD_LIBRARY[cat][model]["label"]
            print(f"    {cat:<20s} → {label}")
        print("=" * 65)


# ---------------------------------------------------------------------------
# Smoke tests
# ---------------------------------------------------------------------------

if __name__ == "__main__":

    print("\n--- 1: Long-range ISR, explicit categories ---")
    PayloadRules(
        cruise_speed=80, mission_altitude=6000,
        mission_range=500, mission_endurance=8,
        payload_categories=["eo_ir", "radar", "datalink"],
    ).summarise()

    print("\n--- 2: Strike UAV ---")
    PayloadRules(
        cruise_speed=150, mission_altitude=8000,
        mission_range=1000, mission_endurance=5,
        payload_categories=["eo_ir", "weapon"],
        weapon_count=2,
    ).summarise()

    print("\n--- 3: Small survey drone, fully inferred ---")
    PayloadRules(
        cruise_speed=20, mission_altitude=500,
        mission_range=30, mission_endurance=1.5,
    ).summarise()

    print("\n--- 4: Short-range but heavy payload pushes class up ---")
    PayloadRules(
        cruise_speed=60, mission_altitude=3000,
        mission_range=100, mission_endurance=3,
        payload_categories=["eo_ir", "radar", "lidar", "weapon"],
        weapon_count=1,
    ).summarise()

    print("\n--- 5: Override class, objective still inferred ---")
    PayloadRules(
        cruise_speed=80, mission_altitude=6000,
        mission_range=200, mission_endurance=8,
        payload_categories=["eo_ir"],
        uav_class_override="large",
    ).summarise()