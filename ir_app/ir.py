#!/usr/bin/env python3
"""Entry point for the refactored IR diagnostics application."""

import sys
from pathlib import Path

import rospy
from PyQt5.QtWidgets import QApplication

if __package__:
    from .ui import MainWindow
else:
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    from ui import MainWindow


def main():
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    app.aboutToQuit.connect(lambda: rospy.signal_shutdown("GUI closed"))
    return app.exec_()


if __name__ == "__main__":
    sys.exit(main())
