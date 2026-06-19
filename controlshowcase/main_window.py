"""MainWindow — layout and wiring (the glue).

The window has three zones, mirroring the brief:

  * a **control panel** (left dock) — tabs to pick the mode and tune parameters;
  * the **cylinder view** (top of the central area) — the animated actuator;
  * the **plot** (below it) — setpoint vs. actual position over time.

Phase 0: the shell. The simulator, the QTimer loop, and the signal wiring between
the panel, the plant, and the views are added from Phase 1 onward.
"""

from __future__ import annotations

from PySide6.QtCore import Qt
from PySide6.QtGui import QAction, QKeySequence
from PySide6.QtWidgets import (
    QDockWidget,
    QMainWindow,
    QMessageBox,
    QSplitter,
    QWidget,
)

from . import __version__
from .ui.control_panel import ControlPanel
from .ui.cylinder_view import CylinderView
from .ui.plot_view import PlotView


class MainWindow(QMainWindow):
    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.setWindowTitle("ControlShowcase")
        self.resize(1100, 720)

        # Central area: cylinder on top, plot below, in a vertical splitter.
        self.cylinder_view = CylinderView(self)
        self.plot_view = PlotView(self)

        splitter = QSplitter(Qt.Orientation.Vertical, self)
        splitter.addWidget(self.cylinder_view)
        splitter.addWidget(self.plot_view)
        splitter.setStretchFactor(0, 0)
        splitter.setStretchFactor(1, 1)
        self.setCentralWidget(splitter)

        # Left dock: the tabbed control panel.
        self.control_panel = ControlPanel(self)
        self._control_dock = QDockWidget("Control", self)
        self._control_dock.setWidget(self.control_panel)
        self._control_dock.setAllowedAreas(
            Qt.DockWidgetArea.LeftDockWidgetArea | Qt.DockWidgetArea.RightDockWidgetArea
        )
        self.addDockWidget(Qt.DockWidgetArea.LeftDockWidgetArea, self._control_dock)
        self.resizeDocks([self._control_dock], [340], Qt.Orientation.Horizontal)

        self._build_menus()
        self.statusBar().showMessage("Ready — Phase 0 scaffold.")

    def _build_menus(self) -> None:
        menu = self.menuBar()

        # --- File -------------------------------------------------------
        file_menu = menu.addMenu("&File")
        quit_action = QAction("E&xit", self)
        quit_action.setShortcut(QKeySequence.StandardKey.Quit)
        quit_action.triggered.connect(self.close)
        file_menu.addAction(quit_action)

        # --- View -------------------------------------------------------
        view_menu = menu.addMenu("&View")
        view_menu.addAction(self._control_dock.toggleViewAction())

        # --- Help -------------------------------------------------------
        help_menu = menu.addMenu("&Help")
        about_action = QAction("&About", self)
        about_action.triggered.connect(self._on_about)
        help_menu.addAction(about_action)

    # --- Slots ----------------------------------------------------------
    def _on_about(self) -> None:
        QMessageBox.about(
            self,
            "About ControlShowcase",
            f"<b>ControlShowcase</b> v{__version__}<br>"
            "A teaching simulator for closed-loop position control of a "
            "hydraulic cylinder.<br>Built with PySide6 + pyqtgraph.",
        )
