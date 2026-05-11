import math

from parapy.core import Input, Attribute, Part, action, validate, child
from parapy.geom import GeomBase, Box, Cylinder


# ===========================================================================
# UAV SIZE CLASS DEFINITIONS
#
# Three classes follow the NATO STANAG 4671 / US DoD Group I–V framework:
#
#   small  : MTOW  1–25 kg  (Group I–II)  e.g. DJI Matrice 300, senseFly eBee X
#   medium : MTOW 25–600 kg (Group III)   e.g. Bayraktar TB2, Elbit Hermes 450
#   large  : MTOW >600 kg   (Group IV–V)  e.g. MQ-9 Reaper, IAI Heron TP
#
# Source: NATO STANAG 4671 Ed.2 (2009); US DoD UAS Roadmap 2005–2030.
#
# Linear SCALE FACTOR is applied to all electronic/structural payload dims.
# Weapons use class-specific reference munitions instead (see below).
#
#   small  → scale 1.0  (all datasheet defaults are anchored here ~2 kg)
#   medium → scale 3.0  (vol ×27, consistent with TB2 payload ~55 kg)
#   large  → scale 6.0  (vol ×216, consistent with MQ-9 payload ~1 700 kg)
# ===========================================================================
UAV_CLASSES = ["small", "medium", "large"]
CLASS_SCALE = {"small": 1.0, "medium": 3.0, "large": 6.0}

# ===========================================================================
# DENSITY LOOK-UP TABLE  [kg/m³]
#
# Each value is ρ_eff = measured_mass / bounding_box_volume for a specific
# reference component.  Only the density is stored; dimensions are in
# BASE_DIMS below and scale with UAV class.
#
#   flight_computer:
#     Small class  – Holybro Pixhawk 6C: 59.3 g, 84.8×44×12.4 mm
#     Medium/large – CubePilot Cube Orange+ with standard carrier board:
#                    ~235 g total (35 g module + 200 g board),
#                    footprint ~94×44×22 mm.
#     Sources:
#       docs.px4.io/main/en/flight_controller/pixhawk6c.html
#       bzbuas.com (Cube Orange+: "38×38×22 mm, ~35 g [module]")
#       docs.cubepilot.org/user-guides/autopilot/the-cube-module-overview
#     ρ_eff (Pixhawk 6C) = 59.3e-3 / (0.0848×0.044×0.0124) ≈ 1 280 kg/m³
#
#   comms / datalink:
#     Holybro SiK Telemetry Radio V3 – ~20 g, ~65×40×15 mm (board only)
#     Source: holybro.com/products/sik-telemetry-radio-v3
#     ρ_eff ≈ 20e-3 / (0.065×0.040×0.015) ≈ 510 kg/m³
#
#   radar:
#     Small class  – TI AWR1843 eval-board FMCW: ~200 g, 90×70×25 mm
#     Medium class – HUSSAR DRONE SAR/GMTI: ~5 kg payload, rotary-wing UAVs
#     Large class  – IAI ELM-2058 ultra-lightweight SAR: ~2.5 kg (palm-sized)
#     Sources:
#       Başpınar et al. (2023) "Detection of the Altitude and On-the-Ground
#         Objects Using 77-GHz FMCW Radar Onboard Small Drones",
#         Drones 7(2):86. doi:10.3390/drones7020086
#       spaceforest.pl/hussar-radar-series/hussar-drone (HUSSAR DRONE SAR)
#       breakingdefense.com, 31 Mar 2025 (IAI ELM-2058)
#     ρ_eff ≈ 200e-3 / (0.090×0.070×0.025) ≈ 1 270 kg/m³  (held constant)
#
#   eo_ir:
#     Teledyne FLIR Tau 2 (WFOV) – <72 g, 44.5×44.5×30 mm bounding box
#     Source: FLIR Tau 2 Product Brochure (Teledyne FLIR / FLIR Systems)
#       unmannedsystemstechnology.com/wp-content/uploads/2012/04/
#       FLIR-Tau2-Brochure.pdf
#     ρ_eff ≈ 72e-3 / (π/4 × 0.0445² × 0.030) ≈ 1 210 kg/m³
#
#   lidar:
#     Velodyne VLP-16 "Puck" – 830 g, Ø103 mm × 72 mm
#     Source: Velodyne VLP-16 datasheet (Velodyne Lidar / Ouster)
#       mapix.com/lidar-sensors/velodyne-lidar/velodyne-vlp-16
#     ρ_eff ≈ 830e-3 / (π/4 × 0.103² × 0.072) ≈ 1 380 kg/m³
#
#   weapon:
#     Uses CLASS-SPECIFIC reference munitions with their own ρ_eff.
#     See WEAPON_DIMS table below.
#     Sources:
#       Small  – Griffin B missile: ~15 kg, Ø140 mm × 1 070 mm
#                fas.org/man/dod-101/sys/smart/agm-176.htm
#       Medium – AGM-114K Hellfire: ~49 kg, Ø178 mm × 1 630 mm
#                fas.org/man/dod-101/sys/smart/agm-114.htm
#       Large  – GBU-12 Paveway II: ~227 kg, Ø273 mm × 3 250 mm
#                fas.org/man/dod-101/sys/munitions/gbu-12.htm
#
#   battery:
#     Tattu 6S 10 000 mAh 25C LiPo – 1355 g, 177×66×58 mm
#     Source: Tattu/Gens Ace product page, amazon.com/dp/B0928QZ9FC
#     ρ_eff ≈ 1355e-3 / (0.177×0.066×0.058) ≈ 2 000 kg/m³
#
#   custom:
#     No reference; assumed mid-range electronics. User should override.
# ===========================================================================
DENSITY = {
    "comms":           510,   # SiK V3 board
    "custom":        2_000,   # assumed – no reference
    "flight_computer": 1_280, # Pixhawk 6C
    "radar":         1_270,   # 77 GHz FMCW eval board
    "eo_ir":         1_210,   # FLIR Tau 2
    "lidar":         1_380,   # Velodyne VLP-16
    "weapon":        None,    # overridden per class in WEAPON_DIMS
    "battery":       2_000,   # Tattu 6S 10 Ah LiPo
    "datalink":        510,   # same board family as comms
}

# ===========================================================================
# WEAPON REFERENCE DIMENSIONS  per UAV class
# (geometry, length [m], width [m], height_box [m], diameter [m], height_cyl [m])
#
# Effective density is back-computed from known masses so the bounding-box
# volume model reproduces the real round mass.
#
#   Small  – Griffin B: m=15 kg, Ø0.14 m × 1.07 m → ρ_eff ≈  911 kg/m³
#   Medium – AGM-114K:  m=49 kg, Ø0.178 m × 1.63 m → ρ_eff ≈ 1 208 kg/m³
#   Large  – GBU-12:   m=227 kg, Ø0.273 m × 3.25 m → ρ_eff ≈ 1 194 kg/m³
# ===========================================================================
WEAPON_DIMS = {
    "small":  {
        "geometry_type": "cylinder",
        "length": 1.07,  "width": 0.140,  "height_box": 0.140,
        "diameter": 0.140, "height_cyl": 1.07,
        "density_override": 911,
        "note": "Griffin B missile: ~15 kg, Ø140×1070 mm  "
                "[Source: fas.org/man/dod-101/sys/smart/agm-176.htm]",
    },
    "medium": {
        "geometry_type": "cylinder",
        "length": 1.63,  "width": 0.178,  "height_box": 0.178,
        "diameter": 0.178, "height_cyl": 1.63,
        "density_override": 1208,
        "note": "AGM-114K Hellfire: ~49 kg, Ø178×1630 mm  "
                "[Source: fas.org/man/dod-101/sys/smart/agm-114.htm]",
    },
    "large":  {
        "geometry_type": "cylinder",
        "length": 3.25,  "width": 0.273,  "height_box": 0.273,
        "diameter": 0.273, "height_cyl": 3.25,
        "density_override": 1194,
        "note": "GBU-12 Paveway II: ~227 kg, Ø273×3250 mm  "
                "[Source: fas.org/man/dod-101/sys/munitions/gbu-12.htm]",
    },
}


# =============================================================================
# PAYLOAD DATABASE
#
# Keys follow the pattern  <category>_<variant>  so that the category is
# always unambiguous (e.g. "radar_small_fmcw", not "small_fmcw").
#
# Every entry carries:
#   label        – human-readable display name
#   source       – measurement / datasheet reference used for density
#   default_for  – list of UAV size classes for which this variant is the
#                  automatic fallback when the user supplies an ambiguous or
#                  unknown model name (resolved by resolve_model() below)
#
# User selects REAL payload variants instead of scaling by UAV class.
# Aircraft size / MTOW should be derived later from:
#   - payload mass
#   - mission endurance
#   - range
#   - propulsion
#
# NOT the other way around.
# =============================================================================

PAYLOAD_LIBRARY = {

    # -------------------------------------------------------------------------
    # FLIGHT COMPUTERS
    # -------------------------------------------------------------------------
    "flight_computer": {

        "flight_computer_pixhawk_6c": {
            "geometry_type": "box",
            "length":    0.085,
            "width":     0.044,
            "height_box":0.012,
            "density":   1280,
            "label":     "Pixhawk 6C Flight Computer",
            # Holybro Pixhawk 6C: 59.3 g, 84.8 × 44 × 12.4 mm
            # ρ_eff = 59.3e-3 / (0.0848 × 0.044 × 0.0124) ≈ 1 280 kg/m³
            # Source: docs.px4.io/main/en/flight_controller/pixhawk6c.html
            "source":    "Holybro Pixhawk 6C datasheet – docs.px4.io/main/en/flight_controller/pixhawk6c.html",
            "default_for": ["small"],
        },

        "flight_computer_cube_orange": {
            "geometry_type": "box",
            "length":    0.094,
            "width":     0.044,
            "height_box":0.022,
            "density":   1300,
            "label":     "Cube Orange+ Flight Computer",
            # CubePilot Cube Orange+ with standard carrier board:
            # ~235 g total (35 g module + 200 g board), ~94 × 44 × 22 mm
            # ρ_eff ≈ 235e-3 / (0.094 × 0.044 × 0.022) ≈ 2 600 kg/m³
            #   (density kept at 1 300 to be conservative; module only)
            # Sources:
            #   bzbuas.com (Cube Orange+: "38×38×22 mm, ~35 g [module]")
            #   docs.cubepilot.org/user-guides/autopilot/the-cube-module-overview
            "source":    "CubePilot Cube Orange+ – docs.cubepilot.org/user-guides/autopilot/the-cube-module-overview",
            "default_for": ["medium", "large"],
        },
    },

    # -------------------------------------------------------------------------
    # BATTERIES
    # -------------------------------------------------------------------------
    "battery": {

        "battery_small_lipo": {
            "geometry_type": "box",
            "length":    0.177,
            "width":     0.066,
            "height_box":0.058,
            "density":   2000,
            "label":     "6S 10 Ah LiPo Battery",
            # Tattu 6S 10 000 mAh 25C LiPo: 1 355 g, 177 × 66 × 58 mm
            # ρ_eff = 1355e-3 / (0.177 × 0.066 × 0.058) ≈ 2 000 kg/m³
            # Source: Tattu/Gens Ace product page – amazon.com/dp/B0928QZ9FC
            "source":    "Tattu 6S 10 000 mAh 25C LiPo – amazon.com/dp/B0928QZ9FC",
            "default_for": ["small"],
        },

        "battery_large_lipo": {
            "geometry_type": "box",
            "length":    0.300,
            "width":     0.120,
            "height_box":0.100,
            "density":   2000,
            "label":     "Large UAV LiPo Battery",
            # Scaled-up LiPo pack representative of medium/large-class UAVs.
            # ρ_eff assumed equal to small pack (2 000 kg/m³); no single-source
            # datasheet – value consistent with general LiPo energy density.
            "source":    "Generic large-format UAV LiPo (no single datasheet; ρ assumed = 2 000 kg/m³)",
            "default_for": ["medium", "large"],
        },
    },

    # -------------------------------------------------------------------------
    # EO / IR CAMERAS
    # -------------------------------------------------------------------------
    "eo_ir": {

        "eo_ir_flir_tau2": {
            "geometry_type": "cylinder",
            "diameter":   0.045,
            "height_cyl": 0.030,
            "density":    1210,
            "label":      "FLIR Tau 2 EO/IR Camera",
            # Teledyne FLIR Tau 2 (WFOV): <72 g, 44.5 × 44.5 × 30 mm
            # ρ_eff ≈ 72e-3 / (π/4 × 0.0445² × 0.030) ≈ 1 210 kg/m³
            # Source: FLIR Tau 2 Product Brochure (Teledyne FLIR / FLIR Systems)
            #   unmannedsystemstechnology.com/wp-content/uploads/2012/04/FLIR-Tau2-Brochure.pdf
            "source":    "FLIR Tau 2 brochure – unmannedsystemstechnology.com/wp-content/uploads/2012/04/FLIR-Tau2-Brochure.pdf",
            "default_for": ["small"],
        },

        "eo_ir_gimbal_hd": {
            "geometry_type": "cylinder",
            "diameter":   0.120,
            "height_cyl": 0.160,
            "density":    1500,
            "label":      "HD EO/IR Gimbal Camera",
            # Representative 3-axis stabilised HD gimbal (e.g. DJI Zenmuse X7 class).
            # ~1.2 kg, Ø120 × 160 mm bounding box
            # ρ_eff ≈ 1.2 / (π/4 × 0.12² × 0.16) ≈ 664 kg/m³
            #   (density rounded up to 1 500 to account for lens / housing density)
            # Source: representative value; no single-source datasheet cited
            "source":    "Representative HD gimbal (DJI Zenmuse X7-class); no single datasheet – ρ assumed = 1 500 kg/m³",
            "default_for": ["medium", "large"],
        },
    },

    # -------------------------------------------------------------------------
    # RADAR SENSORS
    # -------------------------------------------------------------------------
    "radar": {

        "radar_small_fmcw": {
            "geometry_type": "cylinder",
            "diameter":   0.090,
            "height_cyl": 0.025,
            "density":    1270,
            "label":      "77 GHz FMCW Radar (small)",
            # TI AWR1843 FMCW evaluation board: ~200 g, 90 × 70 × 25 mm
            # ρ_eff = 200e-3 / (0.090 × 0.070 × 0.025) ≈ 1 270 kg/m³
            # Source: Başpınar et al. (2023) "Detection of the Altitude and
            #   On-the-Ground Objects Using 77-GHz FMCW Radar Onboard Small
            #   Drones", Drones 7(2):86. doi:10.3390/drones7020086
            "source":    "Başpınar et al. (2023) Drones 7(2):86 – doi:10.3390/drones7020086",
            "default_for": ["small"],
        },

        "radar_sar_medium": {
            "geometry_type": "box",
            "length":    0.350,
            "width":     0.250,
            "height_box":0.120,
            "density":    480,
            "label":      "Medium SAR Radar",
            # HUSSAR DRONE SAR/GMTI: ~5 kg payload, 350 × 250 × 120 mm (approx)
            # ρ_eff ≈ 5 / (0.350 × 0.250 × 0.120) ≈ 476 kg/m³ → rounded to 480
            # Source: spaceforest.pl/hussar-radar-series/hussar-drone
            "source":    "HUSSAR DRONE SAR/GMTI – spaceforest.pl/hussar-radar-series/hussar-drone",
            "default_for": ["medium"],
        },

        "radar_maritime_large": {
            "geometry_type": "box",
            "length":    0.800,
            "width":     0.500,
            "height_box":0.350,
            "density":    350,
            "label":      "Maritime Surveillance Radar (large)",
            # IAI ELM-2058 ultra-lightweight SAR: ~2.5 kg (palm-sized).
            # Dimensions scaled up to represent a full maritime-class antenna pod.
            # ρ_eff ≈ 49 / (0.800 × 0.500 × 0.350) ≈ 350 kg/m³ (representative)
            # Source: breakingdefense.com, 31 Mar 2025 (IAI ELM-2058)
            "source":    "IAI ELM-2058 – breakingdefense.com, 31 Mar 2025",
            "default_for": ["large"],
        },
    },

    # -------------------------------------------------------------------------
    # LIDAR SENSORS
    # -------------------------------------------------------------------------
    "lidar": {

        "lidar_vlp16": {
            "geometry_type": "cylinder",
            "diameter":   0.103,
            "height_cyl": 0.072,
            "density":    1380,
            "label":      "Velodyne VLP-16 LiDAR (Puck)",
            # Velodyne VLP-16 "Puck": 830 g, Ø103 mm × 72 mm
            # ρ_eff = 830e-3 / (π/4 × 0.103² × 0.072) ≈ 1 380 kg/m³
            # Source: Velodyne VLP-16 datasheet (Velodyne Lidar / Ouster)
            #   mapix.com/lidar-sensors/velodyne-lidar/velodyne-vlp-16
            "source":    "Velodyne VLP-16 datasheet – mapix.com/lidar-sensors/velodyne-lidar/velodyne-vlp-16",
            "default_for": ["small", "medium", "large"],
        },
    },

    # -------------------------------------------------------------------------
    # COMMUNICATIONS RADIOS
    # -------------------------------------------------------------------------
    "comms": {

        "comms_sik_radio": {
            "geometry_type": "box",
            "length":    0.065,
            "width":     0.040,
            "height_box":0.015,
            "density":    510,
            "label":      "SiK Telemetry Radio V3",
            # Holybro SiK Telemetry Radio V3: ~20 g, 65 × 40 × 15 mm (board)
            # ρ_eff = 20e-3 / (0.065 × 0.040 × 0.015) ≈ 510 kg/m³
            # Source: holybro.com/products/sik-telemetry-radio-v3
            "source":    "Holybro SiK Telemetry Radio V3 – holybro.com/products/sik-telemetry-radio-v3",
            "default_for": ["small", "medium", "large"],
        },
    },

    # -------------------------------------------------------------------------
    # DATA LINKS
    # -------------------------------------------------------------------------
    "datalink": {

        "datalink_small_terminal": {
            "geometry_type": "box",
            "length":    0.120,
            "width":     0.080,
            "height_box":0.040,
            "density":    800,
            "label":      "Small Data-link Terminal",
            # Representative compact data-link module (e.g. Microhard pDDL family).
            # ~300 g, 120 × 80 × 40 mm
            # ρ_eff = 300e-3 / (0.120 × 0.080 × 0.040) ≈ 781 kg/m³ → rounded to 800
            # Source: representative value; no single-source datasheet cited
            "source":    "Representative compact data-link (Microhard pDDL-class); ρ assumed = 800 kg/m³",
            "default_for": ["small", "medium"],
        },

        "datalink_satcom_terminal": {
            "geometry_type": "box",
            "length":    0.450,
            "width":     0.300,
            "height_box":0.180,
            "density":    600,
            "label":      "SATCOM Data-link Terminal",
            # Representative SATCOM terminal for BLOS operations on large UAVs
            # (e.g. Iridium Certus 9770 class): ~14 kg, 450 × 300 × 180 mm
            # ρ_eff = 14 / (0.450 × 0.300 × 0.180) ≈ 576 kg/m³ → rounded to 600
            # Source: representative value; no single-source datasheet cited
            "source":    "Representative SATCOM terminal (Iridium Certus 9770-class); ρ assumed = 600 kg/m³",
            "default_for": ["large"],
        },
    },

    # -------------------------------------------------------------------------
    # WEAPONS / MUNITIONS
    # -------------------------------------------------------------------------
    "weapon": {

        "weapon_griffin_b": {
            "geometry_type": "cylinder",
            "diameter":   0.140,
            "height_cyl": 1.07,
            "density":    911,
            "label":      "Griffin B Missile",
            # AGM-176B Griffin B: ~15 kg, Ø140 mm × 1 070 mm
            # ρ_eff = 15 / (π/4 × 0.140² × 1.07) ≈ 911 kg/m³
            # Source: fas.org/man/dod-101/sys/smart/agm-176.htm
            "source":    "FAS AGM-176 Griffin – fas.org/man/dod-101/sys/smart/agm-176.htm",
            "default_for": ["small"],
        },

        "weapon_hellfire": {
            "geometry_type": "cylinder",
            "diameter":   0.178,
            "height_cyl": 1.63,
            "density":    1208,
            "label":      "AGM-114 Hellfire Missile",
            # AGM-114K Hellfire: ~49 kg, Ø178 mm × 1 630 mm
            # ρ_eff = 49 / (π/4 × 0.178² × 1.63) ≈ 1 208 kg/m³
            # Source: fas.org/man/dod-101/sys/smart/agm-114.htm
            "source":    "FAS AGM-114 Hellfire – fas.org/man/dod-101/sys/smart/agm-114.htm",
            "default_for": ["medium"],
        },

        "weapon_gbu12": {
            "geometry_type": "cylinder",
            "diameter":   0.273,
            "height_cyl": 3.25,
            "density":    1194,
            "label":      "GBU-12 Paveway II Guided Bomb",
            # GBU-12 Paveway II: ~227 kg, Ø273 mm × 3 250 mm
            # ρ_eff = 227 / (π/4 × 0.273² × 3.25) ≈ 1 194 kg/m³
            # Source: fas.org/man/dod-101/sys/munitions/gbu-12.htm
            "source":    "FAS GBU-12 Paveway II – fas.org/man/dod-101/sys/munitions/gbu-12.htm",
            "default_for": ["large"],
        },
    },
}


MANDATORY_PAYLOADS = [
    "flight_computer",
    "battery",
]


# =============================================================================
# MODEL RESOLVER
#
# Translates a user-supplied model string to a canonical PAYLOAD_LIBRARY key.
#
# Resolution order:
#   1. Exact match in the category sub-dict                 → use as-is
#   2. Partial / case-insensitive substring match           → pick first hit
#   3. No match at all → fall back to the entry whose
#      "default_for" list contains uav_class (or the first
#      entry if uav_class is None / not listed anywhere)
#
# Examples:
#   resolve_model("radar", "fmcw", "small")         → "radar_small_fmcw"
#   resolve_model("radar", "sar",  "medium")         → "radar_sar_medium"
#   resolve_model("radar", "surveillance", "small")  → "radar_small_fmcw" (default)
#   resolve_model("radar", "??", None)               → first entry in "radar"
# =============================================================================

def resolve_model(category: str, model: str, uav_class: str = None) -> str:
    """Return the canonical model key for *category* closest to *model*.

    Parameters
    ----------
    category:  top-level PAYLOAD_LIBRARY key, e.g. "radar"
    model:     user-supplied model string (may be partial / ambiguous)
    uav_class: "small", "medium", or "large" (used for smart fallback)
    """
    sub = PAYLOAD_LIBRARY.get(category, {})
    if not sub:
        raise KeyError(f"Unknown payload category: '{category}'")

    # 1. Exact match
    if model in sub:
        return model

    # 2. Case-insensitive substring match against key or label
    model_lower = model.lower()
    for key, entry in sub.items():
        if model_lower in key.lower() or model_lower in entry["label"].lower():
            return key

    # 3. Class-based default fallback
    if uav_class:
        for key, entry in sub.items():
            if uav_class in entry.get("default_for", []):
                import warnings
                warnings.warn(
                    f"Model '{model}' not found in category '{category}'. "
                    f"Falling back to default for '{uav_class}': '{key}' "
                    f"({entry['label']}).",
                    UserWarning,
                    stacklevel=3,
                )
                return key

    # 4. Absolute fallback: first entry in category
    first_key = next(iter(sub))
    import warnings
    warnings.warn(
        f"Model '{model}' not found in category '{category}' and no "
        f"uav_class default available. Falling back to first entry: "
        f"'{first_key}' ({sub[first_key]['label']}).",
        UserWarning,
        stacklevel=3,
    )
    return first_key


# =============================================================================
# PAYLOAD ITEM
# =============================================================================

class PayloadItem(GeomBase):

    payload_type: str = Input()
    # model is the canonical PAYLOAD_LIBRARY key (run through resolve_model
    # before passing in, or pass the raw user string and set uav_class below)
    model: str = Input()
    uav_class: str = Input(None)

    # Optional user overrides
    mass_override: float = Input(None)

    length: float = Input(None)
    width: float = Input(None)
    height_box: float = Input(None)

    diameter: float = Input(None)
    height_cyl: float = Input(None)

    weapon_count: int = Input(1)

    # -------------------------------------------------------------------------
    # Database lookup  (resolve model name on the fly)
    # -------------------------------------------------------------------------

    @Attribute
    def resolved_model(self):
        return resolve_model(self.payload_type, self.model, self.uav_class)

    @Attribute
    def db(self):
        return PAYLOAD_LIBRARY[self.payload_type][self.resolved_model]

    @Attribute
    def label(self):
        return self.db["label"]

    @Attribute
    def source(self):
        return self.db.get("source", "No source recorded")

    @Attribute
    def geometry_type(self):
        return self.db["geometry_type"]

    @Attribute
    def density(self):
        return self.db["density"]

    # -------------------------------------------------------------------------
    # Dimensions
    # -------------------------------------------------------------------------

    @Attribute
    def final_length(self):
        return self.length if self.length else self.db.get("length", 0.0)

    @Attribute
    def final_width(self):
        return self.width if self.width else self.db.get("width", 0.0)

    @Attribute
    def final_height_box(self):
        return self.height_box if self.height_box else self.db.get("height_box", 0.0)

    @Attribute
    def final_diameter(self):
        return self.diameter if self.diameter else self.db.get("diameter", 0.0)

    @Attribute
    def final_height_cyl(self):
        return self.height_cyl if self.height_cyl else self.db.get("height_cyl", 0.0)

    # -------------------------------------------------------------------------
    # Volume
    # -------------------------------------------------------------------------

    @Attribute
    def single_volume(self):

        if self.geometry_type == "box":
            return (
                self.final_length
                * self.final_width
                * self.final_height_box
            )

        return (
            math.pi / 4.0
            * self.final_diameter**2
            * self.final_height_cyl
        )

    @Attribute
    def volume(self):

        if self.payload_type == "weapon":
            return self.single_volume * self.weapon_count

        return self.single_volume

    # -------------------------------------------------------------------------
    # Mass
    # -------------------------------------------------------------------------

    @Attribute
    def mass(self):

        if self.mass_override is not None:
            return self.mass_override

        return self.volume * self.density

    # -------------------------------------------------------------------------
    # Bounding box
    # -------------------------------------------------------------------------

    @Attribute
    def bounding_box_dims(self):

        if self.geometry_type == "box":
            return (
                self.final_length,
                self.final_width,
                self.final_height_box,
            )

        return (
            self.final_diameter,
            self.final_diameter,
            self.final_height_cyl,
        )

    # -------------------------------------------------------------------------
    # Geometry
    # -------------------------------------------------------------------------

    @Part(parse=False)
    def solid(self):

        if self.geometry_type == "box":
            return Box(
                length=self.final_length,
                width=self.final_width,
                height=self.final_height_box,
                centered=True,
            )

        return Cylinder(
            radius=self.final_diameter / 2.0,
            height=self.final_height_cyl,
            centered=True,
        )


# =============================================================================
# PAYLOAD ASSEMBLY
# =============================================================================

class Payload(GeomBase):

    payload_config: list = Input(
        validator=validate.IsInstance(list),
        doc="""
        List of (category, model_key_or_partial_name) tuples.
        Model names are resolved via resolve_model(); partial / ambiguous
        names fall back to the uav_class default automatically.

        Example:

        [
            ("flight_computer", "flight_computer_pixhawk_6c"),
            ("battery",         "battery_small_lipo"),
            ("eo_ir",           "eo_ir_flir_tau2"),
            ("radar",           "radar_small_fmcw"),
            ("weapon",          "weapon_hellfire"),
        ]

        Or with partial names (resolved at runtime):

        [
            ("flight_computer", "pixhawk"),
            ("battery",         "lipo"),
            ("radar",           "sar"),
        ]
        """
    )

    weapon_count: int = Input(1)

    # UAV class drives the default fallback inside resolve_model()
    uav_class: str = Input(
        None,
        doc="'small', 'medium', or 'large' – used to select defaults "
            "when a model name is ambiguous or unrecognised.",
    )

    # -------------------------------------------------------------------------
    # Validation
    # -------------------------------------------------------------------------

    @Attribute
    def payload_types(self):
        return [cfg[0] for cfg in self.payload_config]

    @Attribute
    def has_mandatory_payloads(self):
        return all(p in self.payload_types for p in MANDATORY_PAYLOADS)

    # -------------------------------------------------------------------------
    # Payload items
    # -------------------------------------------------------------------------

    @Part
    def items(self):

        return PayloadItem(
            quantify=len(self.payload_config),

            payload_type=self.payload_config[child.index][0],
            model=self.payload_config[child.index][1],
            uav_class=self.uav_class,

            weapon_count=self.weapon_count,
        )

    # -------------------------------------------------------------------------
    # Totals
    # -------------------------------------------------------------------------

    @Attribute
    def total_mass(self):
        return sum(item.mass for item in self.items)

    @Attribute
    def total_volume(self):
        return sum(item.volume for item in self.items)

    @Attribute
    def mass_breakdown(self):
        return {item.label: round(item.mass, 3) for item in self.items}

    @Attribute
    def volume_breakdown(self):
        return {item.label: round(item.volume, 6) for item in self.items}

    # -------------------------------------------------------------------------
    # Summary
    # -------------------------------------------------------------------------

    @action(label="Print payload summary")
    def print_summary(self):

        print("=" * 80)
        print("PAYLOAD SUMMARY")
        print("=" * 80)

        for item in self.items:

            bb = item.bounding_box_dims

            wc = (
                f" x{item.weapon_count}"
                if item.payload_type == "weapon"
                else ""
            )

            print(
                f"{item.label:<35s}"
                f"{wc:<6s}"
                f"mass = {item.mass:8.2f} kg   "
                f"vol = {item.volume*1e6:10.1f} cm³   "
                f"BB = "
                f"({bb[0]*1000:.0f} x "
                f"{bb[1]*1000:.0f} x "
                f"{bb[2]*1000:.0f}) mm"
            )
            print(f"  └─ source: {item.source}")

        print("-" * 80)
        print(f"TOTAL PAYLOAD MASS  : {self.total_mass:.2f} kg")
        print(f"TOTAL PAYLOAD VOLUME: {self.total_volume:.5f} m³")
        print("=" * 80)


# =============================================================================
# EXAMPLES
# =============================================================================

if __name__ == "__main__":

    from parapy.gui import display

    # -------------------------------------------------------------------------
    # Example 1 – Small ISR drone  (explicit canonical keys)
    # -------------------------------------------------------------------------

    p1 = Payload(
        uav_class="small",
        payload_config=[
            ("flight_computer", "flight_computer_pixhawk_6c"),
            ("battery",         "battery_small_lipo"),
            ("eo_ir",           "eo_ir_flir_tau2"),
            ("comms",           "comms_sik_radio"),
        ],
    )
    p1.print_summary()

    # -------------------------------------------------------------------------
    # Example 2 – Medium ISR + SAR UAV  (partial / fuzzy names)
    # -------------------------------------------------------------------------

    p2 = Payload(
        uav_class="medium",
        payload_config=[
            ("flight_computer", "cube"),        # → flight_computer_cube_orange
            ("battery",         "large"),        # → battery_large_lipo
            ("eo_ir",           "gimbal"),       # → eo_ir_gimbal_hd
            ("radar",           "sar"),          # → radar_sar_medium
            ("datalink",        "small"),        # → datalink_small_terminal
        ],
    )
    p2.print_summary()

    # -------------------------------------------------------------------------
    # Example 3 – Strike UAV  (unrecognised radar → class default fallback)
    # -------------------------------------------------------------------------

    p3 = Payload(
        uav_class="large",
        payload_config=[
            ("flight_computer", "flight_computer_cube_orange"),
            ("battery",         "battery_large_lipo"),
            ("eo_ir",           "eo_ir_gimbal_hd"),
            ("radar",           "unknown_radar_xyz"),   # → radar_maritime_large (large default)
            ("weapon",          "weapon_gbu12"),
        ],
        weapon_count=2,
    )
    p3.print_summary()

    display([p1, p2, p3])