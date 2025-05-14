from abc import ABCMeta, abstractmethod
from PySide6.QtWidgets import QWidget
from .data_model import MonitorData

class MonitorPanelMeta(ABCMeta, type(QWidget)):
    """Metaclass combining ABCMeta and QWidget's metaclass."""
    pass
    """Combine QWidget metaclass with ABCMeta."""
    pass

class MonitorPanel(QWidget, metaclass=MonitorPanelMeta):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setup_ui()
        self.connect_signals()

    @abstractmethod
    def setup_ui(self): pass

    @abstractmethod
    def connect_signals(self): pass

    @abstractmethod
    def update_data(self, data: MonitorData): pass