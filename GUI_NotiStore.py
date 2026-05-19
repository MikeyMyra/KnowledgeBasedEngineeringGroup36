# notifications_store.py
from parapy.core import Base, Input


class NotificationStore(Base):
    message: str = Input("")
    severity: str = Input("info")   # 'error', 'warning', 'info', 'success'
    open: bool = Input(False)


NOTIFICATIONS = NotificationStore()


def notify_error(msg: str):
    """Open an error snackbar with the given message."""
    NOTIFICATIONS.message = msg
    NOTIFICATIONS.severity = "error"
    NOTIFICATIONS.open = True
