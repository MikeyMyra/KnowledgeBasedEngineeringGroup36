# app.py

from typing import Optional
from parapy.webgui import mui, layout, viewer, html
from parapy.webgui.core import Component, NodeType, State, alert, get_asset_url
from parapy.webgui.core.websocket.dispatchers import update

from parapy.webgui.app_bar import AppBar

# If your Drone lives in another module, adjust this import:
from Pythonfiles.Drone import Drone


class DroneApp(Component):
    """Single-design Drone concept app.

    Flow:
    - No Drone -> show mission dialog automatically.
    - User fills mission & payload intent -> "Create concept" -> Drone() is instantiated.
    - Once Drone exists:
        - Left: 3D viewer
        - Right: summary + buttons for dialogs, plots, exports.
    """

    # The active Drone model; None until user creates a concept
    drone: Optional[Drone] = State(None)  # type: ignore[assignment]

    # Mission / payload state BEFORE a Drone exists (option B)
    mission_cruise_speed: float = State(100.0)
    mission_altitude: float = State(2000.0)
    mission_range: float = State(500.0)
    mission_endurance: float = State(1.0)
    payload_role: str = State("Mapping")
    weapon_count: int = State(0)

    # UI state for dialogs
    mission_dialog_open: bool = State(True)
    fine_tune_dialog_open: bool = State(False)

    #Overrides need to be None
    uav_class_override_ui: str = State("")
    mission_objective_override_ui: str = State("")

    #Diagram state
    diagrams_open: bool = State(False)
    diagrams_data: dict = State(default_factory=dict)

    # loading bar
    creating_concept: bool = State(False)

    def open_fine_tune_dialog(self, *args):
        if self.drone is not None:
            self.uav_class_override_ui = self.drone.uav_class_override or ""
            self.mission_objective_override_ui = self.drone.mission_objective_override or ""
            self.fine_tune_dialog_open = True

    def on_change_uav_class_override(self, evt, *args):
        value = evt.target.value.strip()
        self.uav_class_override_ui = value
        # write through to model: empty string -> None
        if self.drone is not None:
            self.drone.uav_class_override = value or None

    def on_change_mission_objective_override(self, evt, *args):
        value = evt.target.value.strip()
        self.mission_objective_override_ui = value
        if self.drone is not None:
            self.drone.mission_objective_override = value or None

    def format_float(self, value, ndigits=1, suffix=""):
        if value is None:
            return f"– {suffix}".rstrip()
        try:
            return f"{round(value, ndigits)} {suffix}".rstrip()
        except TypeError:
            return f"{value} {suffix}".rstrip()

    def render_create_concept_progress_dialog(self) -> NodeType:
        return mui.Dialog(
            open=self.creating_concept,
            maxWidth="xs",
            fullWidth=True,
        )[
            mui.DialogTitle["Creating concept"],
            mui.DialogContent[
                layout.Box(orientation='vertical', gap='0.75em', style={'minWidth': 280})[
                    mui.LinearProgress(variant='indeterminate'),  # bar, no fixed %
                    mui.Typography(color="text.secondary")[
                        "Sizing concept and generating baseline geometry..."
                    ],
                ]
            ],
        ]

    def render(self) -> NodeType:
        theme = {
            'palette': {
                'primary': {'main': '#1565c0'},
                'secondary': {'main': '#e2bc53ff'},
                'background': {
                    'default': '#f4f6fb',
                    'paper': '#ffffff',
                },
            }
        }

        return mui.ThemeProvider(theme=theme)[
            mui.CssBaseline(),
                # IMPORTANT: full-height root box
            mui.Box(sx={
                'height': '100%',  # fill viewport provided by WebGUI
                'display': 'flex',
                'flexDirection': 'row',
                'padding': '1em',
                'gap': '1em',
                'overflow': 'hidden',
            })[
                # Left: viewer
                mui.Paper(sx={
                    'flex': '2 1 auto',
                    'height': '100%',
                    'padding': '0.5em',
                    'display': 'flex',
                    'flexDirection': 'column',
                    'overflow': 'hidden',
                })[
                    mui.Typography(variant="h6", sx={'mb': '0.5em'})["3D View"],
                        # This box must be flex + height, so viewer gets space
                    mui.Box(sx={
                        'flex': '1 1 auto',
                        'minHeight': 0,  # allow flexbox to shrink properly
                        'overflow': 'hidden',
                    })[
                        self.render_viewer(),
                    ],
                ],

                    # Right: summary & actions
                mui.Paper(sx={
                    'flex': '1 1 auto',
                    'height': '100%',
                    'padding': '1em',
                    'display': 'flex',
                    'flexDirection': 'column',
                    'overflow': 'hidden',
                })[
                    mui.Box(sx={
                        'flex': '1 1 auto',
                        'minHeight': 0,
                        'overflow': 'auto',
                        'display': 'flex',
                        'flexDirection': 'column',
                        'gap': '0.5em',
                    })[
                        self.render_summary(),
                    ],
                    self.render_actions(),
                ],
            ],

                # Dialogs
            self.render_mission_dialog(),
            self.render_fine_tune_dialog(),
            self.render_diagrams_dialog(),
            self.render_create_concept_progress_dialog(),
        ]

    def render_viewer(self) -> NodeType:
        if self.drone is None:
            return mui.Box(sx={
                'flex': '1 1 auto',
                'display': 'flex',
                'alignItems': 'center',
                'justifyContent': 'center',
                'border': '1px dashed #bbb',
            })[
                mui.Typography(color="text.secondary")[
                    "No concept yet. Click 'Configure mission' to create a drone concept."
                ]
            ]

        # The viewer itself: do NOT wrap in extra flex; let parent Box control size
        return viewer.Viewer(
            objects=[self.drone.aircraft],  # or just self.drone if it’s a GeomBase
            style={'width': '100%', 'height': '100%'}
        )

    def render_summary(self) -> NodeType:
        if self.drone is None:
            return mui.Box[
                mui.Typography(variant="h6")["Mission summary"],
                mui.Typography(color="text.secondary", sx={'mt': '0.5em'})[
                    "Provide mission parameters to see a summary and performance data."
                ],
            ]

        d = self.drone

        # Determine engine type and which loading to show
        engine_type = getattr(d, "engine_type", None)
        # engine_type is an Attribute -> no (), may be a string or something richer
        engine_type_str = str(engine_type).lower() if engine_type is not None else ""

        is_prop = "p" in engine_type_str

        if is_prop:
            loading_label = "W/P"
            loading_value = d.power_loading  # Attribute
        else:
            loading_label = "T/W"
            loading_value = d.thrust_loading  # Attribute

        return mui.Box(sx={'display': 'flex', 'flexDirection': 'column', 'gap': '0.5em'})[
            mui.Typography(variant="h6")["Mission & Performance"],
            mui.Typography[f"Cruise speed: {d.cruise_speed} m/s"],
            mui.Typography[f"Mission altitude: {d.mission_altitude} m"],
            mui.Typography[f"Range: {d.mission_range} km, Endurance: {d.mission_endurance} h"],
            mui.Typography[f"Payload role: {d.payload_role}, weapons: {d.weapon_count}"],

            mui.Divider(sx={'my': '0.5em'}),

            mui.Typography(variant="subtitle1")["Weights"],
            mui.Typography[f"MTOW: {self.format_float(d.MTOW, 1, 'kg')}"],
            mui.Typography[f"Empty weight: {self.format_float(d.empty_weight, 1, 'kg')}"],
            mui.Typography[f"Fuel weight: {self.format_float(d.fuel_weight, 1, 'kg')}"],

            mui.Divider(sx={'my': '0.5em'}),

            mui.Typography(variant="subtitle1")["Wing loading & power/thrust"],
            mui.Typography[f"W/S: {self.format_float(d.wing_loading, 1, 'N/m²')}"],
            mui.Typography[f"{loading_label}: {self.format_float(loading_value, 3)}"],

            mui.Divider(sx={'my': '0.5em'}),

            mui.Typography(variant="subtitle1")["Stability"],
            mui.Typography[f"Static margin: {self.format_float(d.static_margin, 3)}"],
            mui.Typography[f"Status: {d.stability_status}"],
        ]

    def render_actions(self) -> NodeType:
        return mui.Box(sx={
            'display': 'flex',
            'flexDirection': 'column',
            'gap': '0.5em',
        })[
            mui.Button(variant="contained", color="primary",
                       onClick=self.open_mission_dialog)[
                "Configure mission"
            ],
            mui.Button(variant="outlined", color="primary",
                       disabled=self.drone is None,
                       onClick=self.open_fine_tune_dialog)[
                "Fine-tune geometry"
            ],
            mui.Button(variant="outlined", color="secondary",
                       disabled=self.drone is None,
                       onClick=self.handle_show_diagrams)[
                "Show diagrams (W/P–W/S, polars, V–n)"
            ],
            mui.Button(variant="outlined", color="inherit",
                       disabled=self.drone is None,
                       onClick=self.handle_export_step)[
                "Export STEP"
            ],
            mui.Button(variant="outlined", color="inherit",
                       disabled=self.drone is None,
                       onClick=self.handle_export_pdf)[
                "Export PDF report"
            ],
        ]

    # -------------------------------------------------------------------------
    # Mission dialog (pre-Drone)
    # -------------------------------------------------------------------------
    def render_mission_dialog(self) -> NodeType:
        return mui.Dialog(
            open=self.mission_dialog_open,
            onClose=self.close_mission_dialog,
            scroll='paper',  # allow scrolling inside paper
            fullWidth=True,
            maxWidth="sm",
            sx={
                '& .MuiPaper-root': {
                    'mt': '3em',  # margin-top relative to viewport
                }
            },
        )[
            mui.DialogTitle["Configure mission"],
            mui.DialogContent[
                mui.Box(sx={
                    'display': 'flex',
                    'flexDirection': 'column',
                    'gap': '1em',
                    'minWidth': '400px',
                    'minHeight': '450px',
                    'mt': '0.5em',
                })[
                    mui.TextField(
                        label="Cruise speed [m/s]",
                        type="number",
                        value=self.mission_cruise_speed,
                        onChange=self.on_change_mission_cruise_speed,
                    ),
                    mui.TextField(
                        label="Mission altitude [m]",
                        type="number",
                        value=self.mission_altitude,
                        onChange=self.on_change_mission_altitude,
                    ),
                    mui.TextField(
                        label="Mission range [km]",
                        type="number",
                        value=self.mission_range,
                        onChange=self.on_change_mission_range,
                    ),
                    mui.TextField(
                        label="Mission endurance [h]",
                        type="number",
                        value=self.mission_endurance,
                        onChange=self.on_change_mission_endurance,
                    ),
                    mui.TextField(
                        label="Payload role",
                        helperText='e.g. "ISR", "Strike", "Mapping", "COMMS relay", "Patrol"',
                        value=self.payload_role,
                        onChange=self.on_change_payload_role,
                    ),
                    mui.TextField(
                        label="Weapon count",
                        type="number",
                        value=self.weapon_count,
                        onChange=self.on_change_weapon_count,
                    ),
                ]
            ],
            mui.DialogActions[
                mui.Button(onClick=self.close_mission_dialog)["Cancel"],
                mui.Button(
                    variant="contained",
                    color="primary",
                    onClick=self.handle_create_concept,
                    disabled=self.creating_concept,
                )["Create concept"],
            ]
        ]

    # Mission dialog field event handlers
    def on_change_mission_cruise_speed(self, evt, *args):
        try:
            self.mission_cruise_speed = float(evt.target.value or 0.0)
        except ValueError:
            pass

    def on_change_mission_altitude(self, evt, *args):
        try:
            self.mission_altitude = float(evt.target.value or 0.0)
        except ValueError:
            pass

    def on_change_mission_range(self, evt, *args):
        try:
            self.mission_range = float(evt.target.value or 0.0)
        except ValueError:
            pass

    def on_change_mission_endurance(self, evt, *args):
        try:
            self.mission_endurance = float(evt.target.value or 0.0)
        except ValueError:
            pass

    def on_change_payload_role(self, evt, *args):
        self.payload_role = evt.target.value

    def on_change_weapon_count(self, evt, *args):
        try:
            self.weapon_count = int(evt.target.value or 0)
        except ValueError:
            pass

    def open_mission_dialog(self, *args):
        self.mission_dialog_open = True

    def close_mission_dialog(self, *args):
        # Only allow closing if a drone exists; otherwise user is stuck with no concept
        if self.drone is not None:
            self.mission_dialog_open = False

    def handle_create_concept(self, *args):
        """Instantiate Drone using mission & payload intent."""
        # show progress dialog and lock button
        self.creating_concept = True
        update()  # push UI so progress bar appears immediately

        try:
            drone = Drone(
                cruise_speed=self.mission_cruise_speed,
                mission_altitude=self.mission_altitude,
                mission_range=self.mission_range,
                mission_endurance=self.mission_endurance,
                payload_role=self.payload_role,
                weapon_count=self.weapon_count,
            )
        except Exception as exc:
            print(f"Error creating Drone: {exc}")
        else:
            self.drone = drone
            self.mission_dialog_open = False
        finally:
            self.creating_concept = False
    # -------------------------------------------------------------------------
    # Fine-tuning dialog (post-Drone; uses SlotFields)
    # -------------------------------------------------------------------------
    def render_fine_tune_dialog(self) -> NodeType:
        if self.drone is None:
            return None

        return mui.Dialog(
            open=self.fine_tune_dialog_open,
            onClose=self.close_fine_tune_dialog,
            maxWidth="md",
            fullWidth=True,
            scroll='paper',  # scrolling inside dialog paper
        )[
            mui.DialogTitle["Fine-tune geometry & systems"],
            mui.DialogContent[
                mui.Box(sx={
                    'display': 'grid',
                    'gridTemplateColumns': 'repeat(2, minmax(0, 1fr))',
                    'gap': '1em',
                    'mt': '0.5em',
                })[
                    # Column 1: classification & objective
                    mui.Box[
                        mui.Typography(variant="subtitle1")["Classification overrides"],

                            # Comment / helper text above field
                        mui.Typography(variant="body2", color="text.secondary")[
                            "UAV class override (small/medium/large)"
                        ],
                        mui.TextField(
                            value=self.uav_class_override_ui,
                            onChange=self.on_change_uav_class_override,
                        ),

                        mui.Typography(variant="body2", color="text.secondary", sx={'mt': 0.5})[
                            "Mission objective override (High Speed/High Endurance/Low cost)"
                        ],
                        mui.TextField(
                            value=self.mission_objective_override_ui,
                            onChange=self.on_change_mission_objective_override,
                        ),
                    ],

                        # Column 2: fuselage
                    mui.Box[
                        mui.Typography(variant="subtitle1", sx={'mb': 1.5})["Fuselage shaping"],
                        layout.SlotFloatField(
                            self.drone, 'fuselage_cylinder_start',
                            label='Fuselage cylinder start [% of length]'
                        ),
                        layout.SlotFloatField(
                            self.drone, 'fuselage_cylinder_end',
                            label='Fuselage cylinder end [% of length]'
                        ),
                        layout.SlotFloatField(
                            self.drone, 'payload_nose_clearance',
                            label='Payload nose clearance [m]'
                        ),
                    ],

                        # Column 3: fuel system
                    mui.Box[
                        mui.Typography(variant="subtitle1", sx={'mb': 1.5})["Fuel system"],
                        mui.Typography(variant="body2", color="text.secondary", sx={'mb': 1.5})[
                            'Fuel type (avgas_100ll, jet_a, jp8, lipo_battery or "auto")'
                        ],
                        layout.SlotStringField(
                            self.drone, 'fuel_type'
                        ),

                        mui.Typography(variant="body2", color="text.secondary", sx={'mb': 1.5})[
                            'Fuel tank aspect ratio [-]'
                        ],
                        layout.SlotFloatField(
                            self.drone, 'fuel_tank_aspect_ratio'
                        ),
                    ],

                        # Column 4: wing planform
                    mui.Box[
                        mui.Typography(variant="subtitle1", sx={'mb': 1.5})["Wing planform"],
                        layout.SlotFloatField(
                            self.drone, 'wing_taper_ratio',
                            label='Wing taper ratio λ = c_tip / c_root'
                        ),
                    ],
                ]
            ],
            mui.DialogActions[
                mui.Button(onClick=self.close_fine_tune_dialog)["Close"],
            ]
        ]

    def open_fine_tune_dialog(self, *args):
        if self.drone is not None:
            self.fine_tune_dialog_open = True

    def close_fine_tune_dialog(self, *args):
        self.fine_tune_dialog_open = False

    # -------------------------------------------------------------------------
    # Handlers for plots and exports
    # -------------------------------------------------------------------------
    def render_diagrams_dialog(self) -> NodeType:
        wp_ws_png = self.diagrams_data.get("wp_ws_png")
        cl_png = self.diagrams_data.get("cl_png")
        vn_png = self.diagrams_data.get("vn_png")

        return mui.Dialog(
            open=self.diagrams_open,
            onClose=self.handle_close_diagrams,
            maxWidth='lg',
            fullWidth=True,
        )[
            mui.DialogTitle["Performance diagrams (PNG)"],
            mui.DialogContent(dividers=True)[
                mui.Box(
                    sx={
                        'display': 'flex',
                        'flexWrap': 'wrap',  # allow wrapping to next row
                        'gap': '1em',
                        'width': '80vw',
                        'maxHeight': '70vh',
                        'overflowY': 'auto',
                    }
                )[
                    # WP/WS PNG
                    mui.Box(sx={'flex': '1 1 calc(50% - 1em)'})[
                        (
                            html.img(
                                src=get_asset_url(wp_ws_png, hash=True),
                                style={
                                    'width': '100%',  # fit container width
                                    'height': 'auto',  # keep aspect ratio
                                    'objectFit': 'contain',
                                },
                            )
                            if wp_ws_png
                            else mui.Typography["No WP/WS diagram PNG"]
                        )
                    ],

                    # Cl–α PNG
                    mui.Box(sx={'flex': '1 1 calc(50% - 1em)'})[
                        (
                            html.img(
                                src=get_asset_url(cl_png, hash=True),
                                style={
                                    'width': '100%',
                                    'height': 'auto',
                                    'objectFit': 'contain',
                                },
                            )
                            if cl_png
                            else mui.Typography["No Cl–α diagram PNG"]
                        )
                    ],

                    # V–n PNG
                    mui.Box(sx={'flex': '1 1 calc(50% - 1em)'})[
                        (
                            html.img(
                                src=get_asset_url(vn_png, hash=True),
                                style={
                                    'width': '100%',
                                    'height': 'auto',
                                    'objectFit': 'contain',
                                },
                            )
                            if vn_png
                            else mui.Typography["No V–n diagram PNG"]
                        )
                    ],
                ],
                mui.DialogActions[
                    mui.Button(onClick=self.handle_close_diagrams)["Close"],
                ]
            ]
        ]

    def handle_close_diagrams(self, evt, *args) -> None:
        self.diagrams_open = False

    # -------------------------------------------------------------------------
    # Event handler that fills diagrams_data and opens dialog
    # -------------------------------------------------------------------------
    def handle_show_diagrams(self, *args):
        if self.drone is None:
            alert("No drone concept available.")
            return

        try:
            self.diagrams_data = {
                "wp_ws_png": self.drone.WP_WS_diagram(),
                "cl_png":    self.drone.plot_wing_cl_alpha(),
                "vn_png":    self.drone.vn_diagram(),
            }
            self.diagrams_open = True
        except Exception as exc:
            alert(f"Error running plot actions: {exc}")

    def handle_export_step(self, *args):
        if self.drone is None:
            alert("No drone concept available to export.")
            return
        try:
            self.drone.export_stp_file()
            alert("STEP file exported successfully.")
        except Exception as exc:
            alert(f"Error exporting STEP: {exc}")

    def handle_export_pdf(self, *args):
        if self.drone is None:
            alert("No drone concept available to export.")
            return
        try:
            self.drone.export_pdf_report()
            alert("PDF report exported successfully.")
        except Exception as exc:
            alert(f"Error exporting PDF: {exc}")


# -------------------------------------------------------------------------
# Entrypoint for local development
# -------------------------------------------------------------------------
if __name__ == "__main__":
    from parapy.webgui.core import display
    display(DroneApp, reload=True)
