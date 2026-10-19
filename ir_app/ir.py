#!/usr/bin/env python3
"""Entry point for the refactored IR diagnostics application."""

import sys

import rospy
from PyQt5.QtWidgets import QApplication

from .ui import MainWindow


def main():
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    app.aboutToQuit.connect(lambda: rospy.signal_shutdown("GUI closed"))
    return app.exec_()


if __name__ == "__main__":
    sys.exit(main())
