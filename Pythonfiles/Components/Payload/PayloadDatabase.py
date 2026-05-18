import json
import math
import os
import warnings


# =============================================================================
# COLOUR MAP  — visually distinct colours per category in the ParaPy viewer
# =============================================================================

PAYLOAD_TYPE_COLORS = {
    "flight_computer": "SlateBlue",    # electronics — calm blue-violet
    "battery":         "LimeGreen",    # power storage — bright green
    "eo_ir":           "DeepSkyBlue",  # optical sensor — sky blue
    "radar":           "Orange",       # radar — warning orange
    "lidar":           "Cyan",         # laser rangefinder — cyan
    "comms":           "Gold",         # communication — gold
    "datalink":        "Yellow",       # data-link radio — yellow
    "weapon":          "Firebrick",    # weapon — dark red
}

_DEFAULT_PAYLOAD_COLOR = "LightGray"


# =============================================================================
# JSON DATABASE
# =============================================================================

_DATA_FILE = os.path.join(
    os.path.dirname(__file__), "..", "..", "..", "Inputfiles", "payload_library.json"
)

with open(_DATA_FILE, encoding="utf-8") as _f:
    _DB = json.load(_f)

UAV_CLASSES        = _DB["uav_classes"]
CLASS_SCALE        = _DB["class_scale"]
DENSITY            = _DB["density"]
WEAPON_DIMS        = _DB["weapon_dims"]
MANDATORY_PAYLOADS = _DB["mandatory_payloads"]
PAYLOAD_LIBRARY    = _DB["payload_library"]


# =============================================================================
# HELPERS
# =============================================================================

def resolve_model(category: str, model: str, uav_class: str = None) -> str:
    """Return the canonical model key for *category* closest to *model*.

    Resolution order:
      1. Exact key match.
      2. Case-insensitive substring match on key or label.
      3. Default variant for *uav_class* (with UserWarning).
      4. First entry (with UserWarning).
    """
    sub = PAYLOAD_LIBRARY.get(category, {})
    if not sub:
        raise KeyError(f"Unknown payload category: '{category}'")

    if model in sub:
        return model

    model_lower = model.lower()
    for key, entry in sub.items():
        if model_lower in key.lower() or model_lower in entry["label"].lower():
            return key

    if uav_class:
        for key, entry in sub.items():
            if uav_class in entry.get("default_for", []):
                warnings.warn(
                    f"Model '{model}' not found in category '{category}'. "
                    f"Falling back to default for '{uav_class}': '{key}' "
                    f"({entry['label']}).",
                    UserWarning, stacklevel=3,
                )
                return key

    first_key = next(iter(sub))
    warnings.warn(
        f"Model '{model}' not found in category '{category}' and no "
        f"uav_class default available. Falling back to first entry: "
        f"'{first_key}' ({sub[first_key]['label']}).",
        UserWarning, stacklevel=3,
    )
    return first_key


def weapon_grid(weapon_count: int) -> tuple:
    """Return (n_cols, n_rows) for the most-square grid fitting *weapon_count*
    cylinders packed side-by-side in the y-z plane.

    Examples
    --------
    1  → (1, 1)
    2  → (2, 1)
    3  → (2, 2)   [4 slots, 1 empty]
    4  → (2, 2)
    6  → (3, 2)
    """
    n_cols = math.ceil(math.sqrt(weapon_count))
    n_rows = math.ceil(weapon_count / n_cols)
    return n_cols, n_rows
