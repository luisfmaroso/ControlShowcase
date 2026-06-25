"""Shared colours for the UI layer.

Kept in one place so the plot, the cylinder animation, and the control panel all
draw from the same palette — the same dark scheme as the sibling CsvPlotter.
"""

from __future__ import annotations

# Window / plot chrome (matches main.py's Fusion palette).
BACKGROUND = "#1e2228"
PANEL = "#262b33"
FOREGROUND = "#c8d0d8"  # axes, ticks, labels, default text
MUTED = "#7a828c"
ACCENT = "#3b6ea8"
GRID_ALPHA = 0.3

# Signal colours, reused for both the plot traces and the cylinder markers.
COLOR_SETPOINT = "#e8703a"  # orange — the target the controller chases
COLOR_POSITION = "#4f9dde"  # blue — the cylinder's actual position
COLOR_PWM = "#5fbf6f"       # green — the valve command
COLOR_ERROR = "#d65f8a"     # pink — setpoint minus position

# Cylinder animation.
COLOR_BARREL = "#3a4049"
COLOR_PISTON = "#8a929c"
COLOR_ROD = "#c8d0d8"
COLOR_LOAD = "#c2603a"  # the weight hanging on the rod
