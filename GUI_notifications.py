# notifications_component.py
from parapy.webgui import mui
from parapy.webgui.core import Component, NodeType
from parapy.webgui import layout

from GUI_NotiStore import NOTIFICATIONS  # import the global store


class Notifications(Component):
    def render(self) -> NodeType:
        # Using NOTIFICATIONS slots here makes this component reactive to changes
        return mui.Snackbar(
            open=NOTIFICATIONS.open,
            autoHideDuration=4000,
            onClose=self.on_close,
            anchorOrigin={'vertical': 'bottom', 'horizontal': 'center'},
        )[
            mui.Alert(
                severity=NOTIFICATIONS.severity,
                variant='filled',
                onClose=self.on_close,
            )[NOTIFICATIONS.message]
        ]

    def on_close(self, *args):
        NOTIFICATIONS.open = False
