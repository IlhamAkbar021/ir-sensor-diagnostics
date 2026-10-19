#!/usr/bin/env python
import sys
import rospy
import statistics
import socket
import time
from math import atan2, degrees, tan, radians
import matplotlib

matplotlib.use("Qt5Agg")
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qt5agg import NavigationToolbar2QT as NavigationToolbar
from matplotlib.figure import Figure
from PyQt5.QtWidgets import (
    QApplication,
    QMainWindow,
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QComboBox,
    QPlainTextEdit,
    QLabel,
    QGroupBox,
    QPushButton,
    QProgressBar,
    QGridLayout,
    QSpinBox,
    QMessageBox,
    QCheckBox,
    QDialog,
    QFileDialog,
    QStyle,
)
from PyQt5.QtCore import pyqtSignal, QObject, QTimer, Qt
from PyQt5.QtGui import QFont, QPainter, QColor
from aeolus_robio_msgs.msg import Analog4Sensor
from .ir_processing import (
    FOV_DEGREES as FOV,
    SENSOR_GEOMETRY,
    TFMINI_LOST_SIGNAL,
    WARMUP_SAMPLES,
    in_pass_range,
    pass_range as get_pass_range,
)

# === UI size settings: edit these values to resize the application ===
MAIN_WINDOW_WIDTH = 820
MAIN_WINDOW_HEIGHT = 700
PLOT_MIN_WIDTH = 850
PLOT_MIN_HEIGHT = 600
PLOT_SCREEN_WIDTH_RATIO = 0.85
PLOT_SCREEN_HEIGHT_RATIO = 0.90
PLOT_PARENT_WIDTH_RATIO = 0.85
PLOT_PARENT_HEIGHT_RATIO = 0.90
LIVE_SWITCH_WIDTH = 118
LIVE_SWITCH_HEIGHT = 30
PLOT_BUTTON_SIZE = 38


class ROSNode(QObject):
    """Handles ROS communications safely with Qt."""

    data_updated = pyqtSignal(object)

    def __init__(self):
        super().__init__()
        rospy.init_node("ir_gui_monitor", anonymous=True, disable_signals=True)
        self.sub = rospy.Subscriber(
            "/robot/center/ir/sensor", Analog4Sensor, self.callback, queue_size=1
        )

    def callback(self, msg):
        self.data_updated.emit(msg)


class ToggleSwitch(QWidget):
    """A minimal, instant toggle switch to guarantee zero lag and high performance."""

    stateChanged = pyqtSignal(bool)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFixedSize(50, 26)
        self._is_checked = False
        self._position = 3

    def mouseReleaseEvent(self, event):
        if event.button() == Qt.LeftButton:
            self._is_checked = not self._is_checked
            if self._is_checked:
                self._position = 26
            else:
                self._position = 3
            self.update()
            self.stateChanged.emit(self._is_checked)

    def isChecked(self):
        return self._is_checked

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        if self._is_checked:
            p.setBrush(QColor("#5cb85c"))
        else:
            p.setBrush(QColor("#337ab7"))
        p.setPen(Qt.NoPen)
        p.drawRoundedRect(0, 0, 50, 26, 13, 13)
        p.setBrush(QColor("#ffffff"))
        p.drawEllipse(self._position, 3, 20, 20)
        p.end()


class SensorPlotDialog(QDialog):
    def __init__(self, terminals, parent=None):
        super().__init__(parent)
        self.terminals = terminals
        self.live_mode = True
        self.setWindowTitle("TFmini Sampling Plots - IR1 and IR2")
        self.setMinimumSize(PLOT_MIN_WIDTH, PLOT_MIN_HEIGHT)
        layout = QVBoxLayout(self)
        self.toolbar = NavigationToolbar(terminals[0].plot_canvas, self)
        layout.addWidget(self.toolbar)
        plots_layout = QHBoxLayout()
        for plot_index, terminal in enumerate(terminals):
            terminal.plot_host.layout().removeWidget(terminal.plot_canvas)
            terminal.plot_canvas.setParent(self)
            terminal.plot_canvas.setVisible(True)
            terminal.plot_canvas.axes.set_title(
                terminal.plot_title(),
                fontsize=13,
                fontfamily="DejaVu Sans",
                fontweight="normal",
                color="#263238",
                pad=12,
            )
            terminal.plot_canvas.axes.set_xlabel(
                "Samples", fontsize=13, fontfamily="DejaVu Sans", color="#37474f"
            )
            if plot_index == 0:
                terminal.plot_canvas.axes.set_ylabel(
                    "Distance (cm)",
                    fontsize=13,
                    fontfamily="DejaVu Sans",
                    color="#37474f",
                )
            else:
                terminal.plot_canvas.axes.set_ylabel("")
            terminal.plot_canvas.axes.tick_params(
                axis="both", labelsize=12, colors="#263238"
            )
            terminal.plot_canvas.figure.subplots_adjust(
                left=0.17 if plot_index == 0 else 0.10,
                right=0.96,
                bottom=0.15,
                top=0.85,
            )
            plots_layout.addWidget(terminal.plot_canvas, stretch=1)
        layout.addLayout(plots_layout, stretch=1)
        export = QPushButton("Export")
        export.clicked.connect(self.export_plots)
        self.live_toggle = QPushButton("LIVE")
        self.live_toggle.setCheckable(True)
        self.live_toggle.setChecked(True)
        self.live_toggle.setFixedSize(LIVE_SWITCH_WIDTH, LIVE_SWITCH_HEIGHT)
        self.live_toggle.setStyleSheet("""
            QPushButton {
                color: #495057; background: #e9ecef;
                border: 1px solid #adb5bd; border-radius: 15px;
                padding: 3px 12px; font-weight: bold;
            }
            QPushButton:checked {
                color: white; background: #5cb85c;
                border: 1px solid #4cae4c;
            }
            QPushButton:hover { border: 1px solid #337ab7; }
        """)
        self.live_toggle.clicked.connect(self.toggle_live_view)
        reset = QPushButton("Reset View")
        reset.setToolTip("Clear the visible plot buffer")
        reset.clicked.connect(self.reset_view)
        controls = QHBoxLayout()
        controls.addWidget(self.live_toggle)
        controls.addWidget(reset)
        controls.addStretch()
        controls.addWidget(export)
        layout.addLayout(controls)

    def toggle_live_view(self, enabled):
        self.live_mode = enabled
        self.live_toggle.setText("LIVE VIEW" if enabled else "RECORDING")

    def set_recording_state(self, recording):
        """Make the plot state unambiguous while sampling is active."""
        if recording:
            self.live_mode = False
            self.live_toggle.setChecked(False)
            self.live_toggle.setText("RECORDING")
            self.live_toggle.setToolTip("Recording in progress; live view is paused")
            self.live_toggle.setEnabled(False)
            self.live_toggle.setStyleSheet(
                "QPushButton { color: #7a4b00; background: #fff3cd; "
                "border: 1px solid #e0b400; border-radius: 15px; "
                "padding: 3px 10px; font-weight: bold; }"
            )
        else:
            self.live_mode = False
            self.live_toggle.setChecked(False)
            self.live_toggle.setEnabled(True)
            self.live_toggle.setText("RECORDING")
            self.live_toggle.setToolTip("Click to switch to live view")
            self.live_toggle.setStyleSheet("""
                QPushButton { color: #495057; background: #e9ecef;
                    border: 1px solid #adb5bd; border-radius: 15px;
                    padding: 3px 12px; font-weight: bold; }
                QPushButton:checked { color: white; background: #5cb85c;
                    border: 1px solid #4cae4c; }
                QPushButton:hover { border: 1px solid #337ab7; }
            """)

    def reset_view(self):
        if self.parent().is_sampling:
            return
        for terminal in self.terminals:
            terminal.reset_plot_data()

    def export_plots(self):
        robot_name = socket.gethostname().replace("-docker", "").replace("_docker", "")
        environment = self.terminals[0].current_env.replace(" ", "-")
        condition = self.terminals[0].current_condition.replace(" ", "-")
        sample_count = self.terminals[0].calc_samples_count
        default_name = (
            f"{robot_name}_{environment}_{condition}_" f"{sample_count}-samples.png"
        )
        filename, _ = QFileDialog.getSaveFileName(
            self,
            "Export IR Plots",
            default_name,
            "PNG image (*.png);;PDF document (*.pdf);;SVG image (*.svg)",
        )
        if not filename:
            return
        root, dot, ext = filename.rpartition(".")
        root = root if dot else filename
        ext = ext if dot else "png"
        for terminal in self.terminals:
            terminal.plot_canvas.figure.savefig(
                f"{root}_IR{terminal.combo.currentText()}.{ext}",
                dpi=220,
                bbox_inches="tight",
                facecolor="white",
            )

    def closeEvent(self, event):
        for terminal in self.terminals:
            terminal.plot_host.layout().removeWidget(terminal.plot_canvas)
            terminal.plot_canvas.setParent(terminal.plot_host)
            terminal.plot_host.layout().addWidget(terminal.plot_canvas)
            terminal.plot_canvas.setVisible(False)
        self.parent().plot_dialog = None
        super().closeEvent(event)


class TerminalWidget(QGroupBox):
    """A modular UI component containing a dropdown, log display, and structured statistics."""

    def __init__(self, title, default_sensor):
        super().__init__(title)
        self.latest_data = None
        self.last_processed_data = None
        self.samples = []
        self.target_samples = 100
        self.calc_samples_count = 100
        self.current_condition = "None"
        self.current_env = "Factory Jig"
        self.pass_zone = None
        self.pass_range = None
        self.last_log_time = 0.0
        self.last_live_data = None
        self.last_live_plot = 0.0
        layout = QVBoxLayout()
        header_layout = QHBoxLayout()
        sensor_label = QLabel("Select Sensor:")
        header_layout.addWidget(sensor_label)
        self.combo = QComboBox()
        self.combo.addItems(["None", "1", "2", "3", "4"])
        self.combo.setCurrentText(default_sensor)
        self.combo.currentTextChanged.connect(self.on_sensor_changed)
        header_layout.addWidget(self.combo)
        header_layout.addStretch()
        self.terminal = QPlainTextEdit()
        self.terminal.setReadOnly(True)
        self.terminal.setFont(QFont("Consolas", 10))
        self.terminal.setPlaceholderText("Live monitor inactive...")
        self.terminal.document().setMaximumBlockCount(250)
        self.stats_group = QGroupBox("Sampling Statistics")
        self.stats_group.setStyleSheet(
            "QGroupBox { font-weight: bold; font-size: 13px; margin-top: 10px; } QGroupBox::title { subcontrol-origin: margin; left: 7px; padding: 0 3px; }"
        )
        stats_layout = QVBoxLayout()
        stats_layout.setSpacing(8)
        self.progress_bar = QProgressBar()
        self.progress_bar.setValue(0)
        self.progress_bar.setTextVisible(False)
        stats_layout.addWidget(self.progress_bar)
        metrics_layout = QHBoxLayout()
        self.stats_grid = QGridLayout()
        self.stats_grid.setVerticalSpacing(5)
        self.stats_grid.setHorizontalSpacing(15)
        self.lbl_avg = QLabel("<b>Avg:</b> -- cm")
        self.lbl_std = QLabel("<b>STD:</b> --")
        self.lbl_min = QLabel("<b>Min:</b> -- cm")
        self.lbl_max = QLabel("<b>Max:</b> -- cm")
        self.lbl_fail_rate = QLabel("<b>Fail Rate:</b> -- / --")
        for lbl in [
            self.lbl_avg,
            self.lbl_std,
            self.lbl_min,
            self.lbl_max,
            self.lbl_fail_rate,
        ]:
            lbl.setStyleSheet("font-size: 13px; color: #222222;")
        self.stats_grid.addWidget(self.lbl_avg, 0, 0)
        self.stats_grid.addWidget(self.lbl_std, 0, 1)
        self.stats_grid.addWidget(self.lbl_min, 1, 0)
        self.stats_grid.addWidget(self.lbl_max, 1, 1)
        self.stats_grid.addWidget(self.lbl_fail_rate, 2, 0, 1, 2)
        metrics_layout.addLayout(self.stats_grid, stretch=3)
        self.lbl_result = QLabel("STANDBY")
        self.lbl_result.setAlignment(Qt.AlignCenter)
        self.lbl_result.setStyleSheet(
            self._get_badge_style("#6c757d", "#e9ecef", "#dee2e6")
        )
        metrics_layout.addWidget(self.lbl_result, stretch=2)
        stats_layout.addLayout(metrics_layout)
        self.lbl_explanation = QLabel("")
        self.lbl_explanation.setStyleSheet("font-size: 12px; margin-top: 6px;")
        self.lbl_explanation.setWordWrap(True)
        self.lbl_explanation.setAlignment(Qt.AlignLeft | Qt.AlignTop)
        self.lbl_explanation.setMinimumHeight(40)  # PREVENTS LAYOUT SHIFT DURING FAIL
        stats_layout.addWidget(self.lbl_explanation)
        self.plot_canvas = FigureCanvas(Figure(figsize=(5, 2.2), dpi=90))
        self.plot_canvas.axes = self.plot_canvas.figure.add_subplot(111)
        self.plot_canvas.axes.set_xlim(0, 750)
        self.plot_canvas.axes.set_ylim(45, 80)
        self.plot_canvas.axes.grid(True, linestyle="--", alpha=0.5)
        (self.plot_line,) = self.plot_canvas.axes.plot(
            [],
            [],
            linestyle="none",
            marker=".",
            color="#1565c0",
            markersize=6.0,
            markeredgewidth=0.35,
            markeredgecolor="#0d47a1",
            alpha=0.9,
            label="TFmini samples",
        )
        self.plot_avg = self.plot_canvas.axes.axhline(
            0, color="#263238", linestyle="--", linewidth=1.2, label="Average"
        )
        self.plot_min = self.plot_canvas.axes.axhline(
            0, color="#d32f2f", linestyle=":", linewidth=1.2, label="Minimum"
        )
        self.plot_max = self.plot_canvas.axes.axhline(
            0, color="#ef6c00", linestyle=":", linewidth=1.2, label="Maximum"
        )
        (self.plot_min_point,) = self.plot_canvas.axes.plot(
            [],
            [],
            "o",
            markersize=9,
            markerfacecolor="#d32f2f",
            markeredgecolor="white",
            markeredgewidth=1.5,
            label="Minimum point",
        )
        (self.plot_max_point,) = self.plot_canvas.axes.plot(
            [],
            [],
            "o",
            markersize=9,
            markerfacecolor="#ef6c00",
            markeredgecolor="white",
            markeredgewidth=1.5,
            label="Maximum point",
        )
        for marker in (self.plot_avg, self.plot_min, self.plot_max):
            marker.set_visible(False)
        self.min_label = self.plot_canvas.axes.annotate(
            "",
            xy=(0, 0),
            xytext=(8, -18),
            textcoords="offset points",
            color="#b71c1c",
            fontsize=11,
            fontweight="bold",
            bbox=dict(boxstyle="round,pad=0.25", fc="#ffebee", ec="#d32f2f"),
        )
        self.max_label = self.plot_canvas.axes.annotate(
            "",
            xy=(0, 0),
            xytext=(8, 8),
            textcoords="offset points",
            color="#e65100",
            fontsize=11,
            fontweight="bold",
            bbox=dict(boxstyle="round,pad=0.25", fc="#fff3e0", ec="#ef6c00"),
        )
        self.min_label.set_visible(False)
        self.max_label.set_visible(False)
        self.avg_label = self.plot_canvas.axes.annotate(
            "",
            xy=(0.98, 0.05),
            xycoords="axes fraction",
            ha="right",
            va="bottom",
            color="#263238",
            fontsize=10,
            bbox=dict(boxstyle="round,pad=0.25", fc="#eceff1", ec="#607d8b"),
        )
        self.avg_label.set_visible(False)
        self.plot_host = QWidget()
        plot_layout = QHBoxLayout(self.plot_host)
        plot_layout.setContentsMargins(0, 0, 0, 0)
        self.plot_button = QPushButton()
        self.plot_button.setIcon(
            QApplication.style().standardIcon(QStyle.SP_TitleBarMaxButton)
        )
        self.plot_button.setToolTip("Open IR plots")
        self.plot_button.setFixedSize(30, 26)
        self.plot_button.clicked.connect(self.open_plot)
        plot_layout.addWidget(self.plot_button)
        self.plot_host.setVisible(False)
        self.stats_group.setLayout(stats_layout)
        layout.addLayout(header_layout)
        layout.addWidget(self.terminal)
        layout.addWidget(self.stats_group)
        self.setLayout(layout)

    def open_plot(self):
        window = self.window()
        if hasattr(window, "show_plot_dialog"):
            window.show_plot_dialog()

    def update_plot_data(self, redraw=True):
        window = self.window()
        if not getattr(window, "plot_dialog", None):
            return
        values = self.samples[-self.calc_samples_count :]
        self.plot_line.set_data(range(len(values)), values)
        self.plot_canvas.axes.set_xlim(0, max(750, len(values)))
        valid = [v for v in values if v > 0 and v != TFMINI_LOST_SIGNAL]
        if valid:
            avg_value = statistics.mean(valid)
            std_value = statistics.stdev(valid) if len(valid) > 1 else 0.0
            min_value = min(valid)
            max_value = max(valid)
            self.plot_avg.set_ydata([avg_value, avg_value])
            self.plot_min.set_ydata([min_value, min_value])
            self.plot_max.set_ydata([max_value, max_value])
            min_index = values.index(min_value)
            max_index = values.index(max_value)
            self.plot_min_point.set_data([min_index], [min_value])
            self.plot_max_point.set_data([max_index], [max_value])
            self.min_label.xy = (min_index, min_value)
            self.max_label.xy = (max_index, max_value)
            point_count = max(1, len(values) - 1)
            min_left = min_index < point_count * 0.15
            min_right = min_index > point_count * 0.78
            max_left = max_index < point_count * 0.15
            max_right = max_index > point_count * 0.78
            self.min_label.set_position(
                ((8, -18) if min_left else (-8, -18) if min_right else (8, -18))
            )
            self.max_label.set_position(
                ((8, 8) if max_left else (-8, 8) if max_right else (8, 8))
            )
            self.min_label.set_ha("left" if min_left or not min_right else "right")
            self.max_label.set_ha("left" if max_left or not max_right else "right")
            self.min_label.set_text(f"MIN {min_value:.2f} cm")
            self.max_label.set_text(f"MAX {max_value:.2f} cm")
            self.min_label.set_visible(True)
            self.max_label.set_visible(True)
            self.avg_label.set_text(f"AVG {avg_value:.2f} | STD {std_value:.2f}")
            self.avg_label.set_visible(True)
            for marker in (self.plot_avg, self.plot_min, self.plot_max):
                marker.set_visible(True)
            min_b, max_b = get_pass_range(self.current_env, self.current_condition)
            data_low = min(valid)
            data_high = max(valid)
            low = min(data_low, min_b)
            high = max(data_high, max_b)
            margin = max(1.0, (high - low) * 0.08)
            self.plot_canvas.axes.set_ylim(max(0, low - margin), high + margin)
        if redraw:
            self.plot_canvas.draw_idle()

    def update_realtime_plot(self, condition, env):
        """Append the newest monitor value and refresh the open plot lightly."""
        window = self.window()
        if (
            window.plot_dialog is None
            or not window.plot_dialog.live_mode
            or window.is_sampling
        ):
            return False
        if self.latest_data is None or self.latest_data is self.last_live_data:
            return False
        data = self._get_sensor_calculations()
        if data is None:
            return False
        self.last_live_data = self.latest_data
        self.current_condition = condition
        self.current_env = env
        if not window.is_sampling:
            self.samples.append(data[1])
            self.calc_samples_count = 750
            if len(self.samples) > self.calc_samples_count:
                del self.samples[: -self.calc_samples_count]
        now = time.monotonic()
        if now - self.last_live_plot < 0.10:
            return False
        self.last_live_plot = now
        active_range = get_pass_range(env, condition)
        if self.pass_zone is None or self.pass_range != active_range:
            if self.pass_zone is not None:
                self.pass_zone.remove()
            min_b, max_b = active_range
            self.pass_zone = self.plot_canvas.axes.axhspan(
                min_b,
                max_b,
                color="#4caf50",
                alpha=0.22,
                label=f"Pass range ({min_b:g}-{max_b:g} cm)",
            )
            self.pass_range = active_range
        self.plot_canvas.axes.set_title(
            self.plot_title(),
            fontsize=13,
            fontfamily="DejaVu Sans",
            fontweight="normal",
            color="#263238",
            pad=12,
        )
        if self.samples:
            self.update_plot_data(redraw=False)
        return True

    def reset_plot_data(self):
        """Clear the visible buffer when the test setup changes."""
        self.samples.clear()
        self.last_live_data = None
        self.last_processed_data = None
        self.pass_range = None
        self.plot_line.set_data([], [])
        self.plot_min_point.set_data([], [])
        self.plot_max_point.set_data([], [])
        for item in (
            self.plot_avg,
            self.plot_min,
            self.plot_max,
            self.min_label,
            self.max_label,
            self.avg_label,
        ):
            item.set_visible(False)
        if self.pass_zone is not None:
            self.pass_zone.remove()
            self.pass_zone = None
        self.plot_canvas.draw_idle()

    def highlight_metric(self, condition):
        """Highlight only the statistic used by the selected condition."""
        normal = "font-size: 13px; color: #222222;"
        for label in (self.lbl_min, self.lbl_max):
            label.setStyleSheet(normal)
        focus = self.lbl_max if condition == "Flat Surface" else self.lbl_min
        focus.setStyleSheet("font-size: 13px; font-weight: bold; color: #c58a00;")

    def refresh_pass_zone(self, condition, env):
        bounds = get_pass_range(env, condition)
        if self.pass_zone is not None:
            self.pass_zone.remove()
        self.pass_zone = self.plot_canvas.axes.axhspan(
            *bounds,
            color="#4caf50",
            alpha=0.22,
            label=f"Pass range ({bounds[0]:g}-{bounds[1]:g} cm)",
        )
        self.pass_range = bounds
        self.current_condition, self.current_env = condition, env
        self.plot_canvas.axes.set_ylim(max(0, bounds[0] - 3), bounds[1] + 3)
        self.plot_canvas.axes.set_title(
            self.plot_title(),
            fontsize=13,
            fontfamily="DejaVu Sans",
            color="#263238",
            pad=12,
        )
        self.plot_canvas.draw_idle()

    def _get_badge_style(self, text_color, bg_color, border_color):
        """Helper to dynamically generate industrial-looking result badges matching the button font."""
        return f"""
            QLabel {{
                color: {text_color}; 
                font-size: 14px; 
                font-weight: bold; 
                background-color: {bg_color}; 
                border: 2px solid {border_color}; 
                border-radius: 4px; 
                padding: 4px;
            }}
        """

    def plot_title(self):
        """Use one consistent title format for every environment/condition."""
        title = f"Sensor {self.combo.currentText()}"
        if self.current_condition != "None":
            bounds = get_pass_range(self.current_env, self.current_condition)
            title += f"  |  {self.current_condition}  |  {bounds[0]:g}-{bounds[1]:g} cm"
        return title

    def on_sensor_changed(self, new_text):
        if new_text == "None":
            self.terminal.clear()
        self.plot_canvas.axes.set_title(
            self.plot_title(),
            fontsize=13,
            fontfamily="DejaVu Sans",
            fontweight="normal",
            color="#263238",
            pad=12,
        )
        self.plot_canvas.axes.set_xlabel(
            "Samples", fontsize=13, fontfamily="DejaVu Sans"
        )
        if "Primary" in self.title():
            self.plot_canvas.axes.set_ylabel(
                "Distance (cm)", fontsize=13, fontfamily="DejaVu Sans"
            )
        else:
            self.plot_canvas.axes.set_ylabel("")
        self.plot_canvas.axes.tick_params(axis="both", labelsize=12)
        self.plot_canvas.draw_idle()

    def prepare_sampling(self, target_samples, calc_samples, condition, env):
        """Resets variables and configures custom targets for data sampling."""
        self.samples.clear()
        self.last_processed_data = None
        self.target_samples = target_samples
        self.calc_samples_count = calc_samples
        self.current_condition = condition
        self.current_env = env
        self.progress_bar.setMaximum(target_samples)
        self.progress_bar.setValue(0)
        for lbl in [self.lbl_avg, self.lbl_std, self.lbl_min, self.lbl_max]:
            lbl.setStyleSheet("font-size: 13px; color: #222222;")
        self.lbl_avg.setText("<b>Avg:</b> -- cm")
        self.lbl_std.setText("<b>STD:</b> --")
        self.lbl_min.setText("<b>Min:</b> -- cm")
        self.lbl_max.setText("<b>Max:</b> -- cm")
        self.lbl_fail_rate.setText("<b>Fail Rate:</b> -- / --")
        self.highlight_metric(condition)
        self.lbl_explanation.setText("")
        self.stats_group.setTitle("Sampling Statistics")
        self.lbl_result.setText("LOADING...")
        self.lbl_result.setStyleSheet(
            self._get_badge_style("#0c5460", "#d1ecf1", "#bee5eb")
        )
        self.plot_line.set_data([], [])
        self.plot_min_point.set_data([], [])
        self.plot_max_point.set_data([], [])
        self.min_label.set_visible(False)
        self.max_label.set_visible(False)
        self.avg_label.set_visible(False)
        for marker in (self.plot_avg, self.plot_min, self.plot_max):
            marker.set_visible(False)
        if self.pass_zone is not None:
            self.pass_zone.remove()
            self.pass_zone = None
        self.pass_range = None
        min_b, max_b = get_pass_range(env, condition)
        self.pass_zone = self.plot_canvas.axes.axhspan(
            min_b,
            max_b,
            color="#4caf50",
            alpha=0.22,
            label=f"Pass range ({min_b:g}-{max_b:g} cm)",
        )
        self.pass_range = (min_b, max_b)
        self.plot_canvas.axes.set_ylim(max(0, min_b - 3), max_b + 3)
        self.plot_canvas.axes.set_title(
            self.plot_title(),
            fontsize=13,
            fontfamily="DejaVu Sans",
            fontweight="normal",
            color="#263238",
            pad=12,
        )
        if "Primary" in self.title():
            self.plot_canvas.axes.set_ylabel(
                "Distance (cm)", fontsize=13, fontfamily="DejaVu Sans"
            )
        else:
            self.plot_canvas.axes.set_ylabel("")
        self.plot_canvas.axes.set_xlabel(
            "Samples", fontsize=13, fontfamily="DejaVu Sans"
        )
        self.plot_canvas.axes.tick_params(axis="both", labelsize=12)
        self.plot_canvas.draw_idle()

    def cancel_sampling(self):
        """Cleans up the UI safely if the user cancels data collection early."""
        self.lbl_result.setText("CANCELLED")
        # Use a warning yellow/orange color scheme for the cancelled badge
        self.lbl_result.setStyleSheet(
            self._get_badge_style("#856404", "#fff3cd", "#ffe8a1")
        )
        self.progress_bar.setValue(0)
        self.lbl_explanation.setText(
            "<span style='color: #856404; font-weight: bold;'>Data collection cancelled by operator.</span>"
        )

    def _get_sensor_calculations(self):
        sensor_key = self.combo.currentText()
        if sensor_key == "None" or not self.latest_data:
            return None
        sensor_attr = f"sensor_{sensor_key}"
        if not hasattr(self.latest_data, sensor_attr):
            return None
        distance = getattr(self.latest_data, sensor_attr)
        geometry = SENSOR_GEOMETRY[sensor_key]
        angle_rad = atan2(geometry["y"], distance + geometry["x"])
        angle_deg = degrees(angle_rad) + geometry["angle_offset"]
        angle_deg = (angle_deg + 180) % 360 - 180
        beam = 2 * (distance / 100.0) * tan(radians(FOV / 2))
        return sensor_key, distance, angle_deg, beam

    def process_realtime_data(self):
        # Limit text-widget work; sensor sampling remains independent at 13 ms.
        now = time.monotonic()
        if now - self.last_log_time < 0.25:
            return
        self.last_log_time = now
        data = self._get_sensor_calculations()
        if data is None:
            return
        sensor_key, distance, angle_deg, beam = data
        log_msg = f"[IR_{sensor_key}] Dist: {distance:.2f}cm | Angle: {angle_deg:.2f}° | Beam: {beam:.2f}cm"
        self.terminal.appendPlainText(log_msg)
        scrollbar = self.terminal.verticalScrollBar()
        scrollbar.setValue(scrollbar.maximum())

    def process_sampling_data(self):
        """Collects samples cleanly without lagging the UI."""
        if self.latest_data is self.last_processed_data:
            return False
        data = self._get_sensor_calculations()
        if data is None:
            self.progress_bar.setValue(self.target_samples)
            return True
        self.last_processed_data = self.latest_data
        _, distance, _, _ = data
        self.samples.append(distance)
        current_len = len(self.samples)
        # Plotting is intentionally deferred until collection completes. This
        # keeps the 13 ms acquisition loop responsive and CPU-light.
        # The sensor still samples at full speed; only the expensive plot
        # repaint is throttled to keep CPU usage low.
        if current_len % 50 == 0 or current_len >= self.target_samples:
            self.progress_bar.setValue(current_len)
        if current_len >= self.target_samples:
            self.calculate_statistics()
            if getattr(self.window(), "plot_dialog", None):
                self.update_plot_data()
            return True
        return False

    def calculate_statistics(self):
        """Computes statistics on the active sliced samples and runs condition checks."""
        if not self.samples:
            return
        active_samples = self.samples[-self.calc_samples_count :]
        avg_val = statistics.mean(active_samples)
        std_val = statistics.stdev(active_samples) if len(active_samples) > 1 else 0.0
        min_val = min(active_samples)
        max_val = max(active_samples)
        min_b, max_b = get_pass_range(self.current_env, self.current_condition)
        fail_count = sum(
            1 for value in active_samples if not in_pass_range(value, (min_b, max_b))
        )
        self.lbl_avg.setText(f"<b>Avg:</b> {avg_val:.2f} cm")
        self.lbl_std.setText(f"<b>STD:</b> {std_val:.2f}")
        self.lbl_min.setText(f"<b>Min:</b> {min_val:.2f} cm")
        self.lbl_max.setText(f"<b>Max:</b> {max_val:.2f} cm")
        self.lbl_fail_rate.setText(
            f"<b>Fail Rate:</b> {fail_count} / {len(active_samples)}"
        )
        self.lbl_fail_rate.setStyleSheet(
            "font-size: 13px; color: #d9534f; font-weight: bold;"
            if fail_count
            else "font-size: 13px; color: #5cb85c; font-weight: bold;"
        )
        self.stats_group.setTitle(
            f"Sampling Statistics ({len(active_samples)} analyzed)"
        )
        # Reset colors before evaluating
        for lbl in [self.lbl_avg, self.lbl_std, self.lbl_min, self.lbl_max]:
            lbl.setStyleSheet("font-size: 13px; color: #222222;")
        self.lbl_explanation.setText("")
        passed = None
        explanation = ""
        if self.current_condition == "Flat Surface":
            min_b, max_b = get_pass_range(self.current_env, self.current_condition)
            passed = in_pass_range(max_val, (min_b, max_b))
            if passed:
                self.lbl_max.setStyleSheet(
                    "font-size: 13px; color: #2e7d32; font-weight: bold;"
                )
            else:
                self.lbl_max.setStyleSheet(
                    "font-size: 13px; color: #c62828; font-weight: bold;"
                )
                explanation = f"Fail reason: Max value {max_val:.2f} outside range [{min_b}, {max_b}]"
        # CHANGED TO JUST "Cliff"
        elif self.current_condition == "Cliff":
            min_b, max_b = get_pass_range(self.current_env, self.current_condition)
            passed = in_pass_range(min_val, (min_b, max_b))
            if passed:
                self.lbl_min.setStyleSheet(
                    "font-size: 13px; color: #2e7d32; font-weight: bold;"
                )
            else:
                self.lbl_min.setStyleSheet(
                    "font-size: 13px; color: #c62828; font-weight: bold;"
                )
                explanation = f"Fail reason: Min value {min_val:.2f} outside range [{min_b}, {max_b}]"
        if passed is True:
            self.lbl_result.setText("PASS")
            self.lbl_result.setStyleSheet(
                self._get_badge_style("#155724", "#d4edda", "#c3e6cb")
            )
        elif passed is False:
            self.lbl_result.setText("FAIL")
            self.lbl_result.setStyleSheet(
                self._get_badge_style("#721c24", "#f8d7da", "#f5c6cb")
            )
            bold_fail_reason = (
                f"<span style='color: #d9534f; font-weight: bold;'>{explanation}</span>"
            )
            normal_reminder = "<br><span style='color: #222222; font-weight: normal;'>Please verify that your selected condition matches the actual physical setup.</span>"
            self.lbl_explanation.setText(bold_fail_reason + normal_reminder)
        else:
            self.lbl_result.setText("DONE")
            self.lbl_result.setStyleSheet(
                self._get_badge_style("#383d41", "#e2e3e5", "#d6d8db")
            )


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("IR Sensor Diagnostics")
        self.resize(MAIN_WINDOW_WIDTH, MAIN_WINDOW_HEIGHT)
        self.is_monitoring = False
        self.is_sampling = False
        self.nav_pose_confirmed = False
        self.plot_dialog = None
        # --- Main Layout ---
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)
        main_layout.setContentsMargins(15, 15, 15, 15)
        # --- Top Controls Section ---
        control_layout = QHBoxLayout()
        control_layout.setContentsMargins(5, 5, 5, 15)
        control_layout.setSpacing(6)
        # 1. Left Side: Live Telemetry Control
        toggle_layout = QHBoxLayout()
        toggle_layout.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        self.monitor_toggle = ToggleSwitch()
        self.monitor_toggle.stateChanged.connect(self.toggle_monitor)
        self.toggle_label = QLabel("Real-Time Monitor (Paused)")
        self.toggle_label.setStyleSheet(
            "font-size: 14px; font-weight: bold; color: #333333;"
        )
        toggle_layout.addWidget(self.monitor_toggle)
        toggle_layout.addWidget(self.toggle_label)
        control_layout.addLayout(toggle_layout)
        # Pushes elements to the center
        control_layout.addStretch()
        # 2. Middle Up: Directly placing Environment, Condition, and Sample Size (No outer wrapper)
        center_params_layout = QHBoxLayout()
        center_params_layout.setSpacing(8)
        # Style applied to internal boxes (Environment, Condition, Sample Size)
        inner_group_style = """
            QGroupBox { 
                font-weight: bold; font-size: 13px; color: #444444;
                border: 1px solid #cccccc; border-radius: 4px; 
                margin-top: 10px; background-color: #ffffff;
            }
            QGroupBox::title { subcontrol-origin: margin; left: 8px; padding: 0 3px; }
        """
        # 2a. Environment Group Box
        self.env_group = QGroupBox("Environment")
        self.env_group.setStyleSheet(inner_group_style)
        env_layout = QVBoxLayout()
        env_layout.setContentsMargins(10, 15, 10, 10)
        self.env_combo = QComboBox()
        self.env_combo.addItems(
            ["Factory Jig (W/ Case)", "Factory Jig (W/O Case)", "Wood Platform"]
        )
        # Use the no-case factory jig as the safest default setup.
        self.env_combo.setCurrentIndex(1)
        self.env_combo.currentTextChanged.connect(self.on_environment_changed)
        self.env_combo.setStyleSheet(
            "padding: 5px; font-weight: bold; background-color: white; border: 1px solid #bbbbbb;"
        )
        env_layout.addWidget(self.env_combo)
        self.env_group.setLayout(env_layout)
        center_params_layout.addWidget(self.env_group)
        # 2b. Condition Group Box
        self.condition_group = QGroupBox("Condition")
        self.condition_group.setStyleSheet(inner_group_style)
        cb_layout = QVBoxLayout()
        cb_layout.setContentsMargins(10, 15, 10, 10)
        cb_layout.setSpacing(5)
        self.cb_flat = QCheckBox("Flat Surface")
        self.cb_cliff = QCheckBox("Cliff")
        self.cb_flat.setStyleSheet("font-weight: bold; color: #222222;")
        self.cb_cliff.setStyleSheet("font-weight: bold; color: #222222;")
        # Set Flat Surface as the default
        self.cb_flat.setChecked(True)
        self.cb_flat.toggled.connect(self.on_cb_flat_toggled)
        self.cb_cliff.toggled.connect(self.on_cb_cliff_toggled)
        cb_layout.addWidget(self.cb_flat)
        cb_layout.addWidget(self.cb_cliff)
        self.condition_group.setLayout(cb_layout)
        center_params_layout.addWidget(self.condition_group)
        # 2c. Sample Size Group Box
        self.sample_group = QGroupBox("Sample Size")
        self.sample_group.setStyleSheet(inner_group_style)
        sample_layout = QVBoxLayout()
        sample_layout.setContentsMargins(10, 15, 10, 10)
        self.total_spin = QSpinBox()
        self.total_spin.setRange(1, 100000)
        self.total_spin.setValue(750)
        self.total_spin.setStyleSheet(
            "padding: 5px; font-weight: bold; min-width: 70px; border: 1px solid #bbbbbb;"
        )
        sample_layout.addWidget(self.total_spin)
        self.sample_group.setLayout(sample_layout)
        center_params_layout.addWidget(self.sample_group)
        # Add the clean center setup to the main top layout
        control_layout.addLayout(center_params_layout)
        # Pushes Start Button cleanly to the right
        control_layout.addStretch()
        # 3. Right Side: Start Data Collection Button
        btn_layout = QHBoxLayout()
        btn_layout.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        self.sample_btn = QPushButton("Start Collection")
        self.sample_btn.setMinimumHeight(45)
        self.sample_btn.setFixedWidth(138)
        self.set_button_style(self.sample_btn, "green")
        self.sample_btn.clicked.connect(self.toggle_sampling)
        btn_layout.addWidget(self.sample_btn)
        control_layout.addLayout(btn_layout)
        self.plot_button = QPushButton()
        self.plot_button.setIcon(
            QApplication.style().standardIcon(QStyle.SP_TitleBarMaxButton)
        )
        self.plot_button.setToolTip("Open live or completed IR plots")
        self.plot_button.setAccessibleName("Open combined IR plots")
        self.plot_button.setFixedSize(PLOT_BUTTON_SIZE, PLOT_BUTTON_SIZE)
        self.plot_button.clicked.connect(self.show_plot_dialog)
        control_layout.addWidget(self.plot_button, alignment=Qt.AlignVCenter)
        main_layout.addLayout(control_layout)
        # --- Terminals Layout ---
        terminals_layout = QHBoxLayout()
        self.term1 = TerminalWidget("Primary Monitor", "1")
        self.term2 = TerminalWidget("Secondary Monitor", "2")
        terminals_layout.addWidget(self.term1)
        terminals_layout.addWidget(self.term2)
        main_layout.addLayout(terminals_layout)
        self.reset_visual_data()
        # --- Global Application Styling ---
        self.setStyleSheet("""
            QMainWindow { background-color: #f4f6f8; }
            QLabel { color: #333333; }
            QComboBox, QSpinBox { 
                background-color: #ffffff; color: #333333; 
                border-radius: 3px;
            }
            QPlainTextEdit {
                background-color: #fdfdfd; color: #111111; 
                border: 1px solid #bbbbbb; padding: 5px;
                border-radius: 3px;
            }
            QProgressBar {
                border: 1px solid #aaaaaa;
                border-radius: 4px;
                text-align: center;
                background-color: #e9ecef;
                font-weight: bold;
                color: #111111;
            }
            QProgressBar::chunk {
                background-color: #ff9800;
                width: 10px;
            }
        """)
        # --- ROS Integration ---
        self.ros_node = ROSNode()
        self.ros_node.data_updated.connect(self.on_ros_data)
        # --- SEPARATE UPDATE TIMERS ---
        self.sampling_timer = QTimer()
        self.sampling_timer.timeout.connect(self.update_sampling)
        # Fast collection cadence; plotting remains throttled and disabled
        # while the plot dialog is closed, so this does not make the UI heavy.
        self.sampling_timer.start(13)
        self.monitor_timer = QTimer()
        self.monitor_timer.timeout.connect(self.update_monitor)
        self.monitor_timer.start(120)  # Real-time monitor update rate

    def on_cb_flat_toggled(self, checked):
        if checked:
            if self.cb_cliff.isChecked():
                self.cb_cliff.setChecked(False)
            self.reset_visual_data()
        else:
            # If the user tries to uncheck this, but the other one isn't checked, force it back on
            if not self.cb_cliff.isChecked():
                self.cb_flat.setChecked(True)

    def on_cb_cliff_toggled(self, checked):
        if checked:
            if self.cb_flat.isChecked():
                self.cb_flat.setChecked(False)
            self.reset_visual_data()
        else:
            # If the user tries to uncheck this, but the other one isn't checked, force it back on
            if not self.cb_flat.isChecked():
                self.cb_cliff.setChecked(True)

    def toggle_monitor(self):
        self.is_monitoring = self.monitor_toggle.isChecked()
        if self.is_monitoring:
            self.toggle_label.setText("Real-Time Monitor (Live)")
        else:
            self.toggle_label.setText("Real-Time Monitor (Paused)")

    def on_environment_changed(self, _value):
        self.nav_pose_confirmed = False
        self.term1.current_env = _value
        self.term2.current_env = _value
        self.reset_visual_data()

    def reset_visual_data(self):
        if not self.is_sampling:
            condition = "Flat Surface" if self.cb_flat.isChecked() else "Cliff"
            env = self.env_combo.currentText()
            self.term1.current_condition = condition
            self.term2.current_condition = condition
            self.term1.current_env = env
            self.term2.current_env = env
            self.term1.reset_plot_data()
            self.term2.reset_plot_data()
            self.term1.highlight_metric(condition)
            self.term2.highlight_metric(condition)
            self.term1.refresh_pass_zone(condition, env)
            self.term2.refresh_pass_zone(condition, env)

    def toggle_sampling(self):
        """Starts or aborts the data collection."""
        if not self.is_sampling:
            if self.total_spin.value() < 1:
                QMessageBox.warning(
                    self,
                    "Invalid Sample Size",
                    "You must select at least 1 sample to calculate.",
                )
                self.total_spin.setValue(100)
                return
            if not self.nav_pose_confirmed:
                pose_check = QMessageBox(self)
                pose_check.setWindowTitle("Robot Setup Verification")
                pose_check.setMinimumWidth(460)
                pose_check.setIcon(QMessageBox.Warning)
                pose_check.setText("<b>Robot pose verification required</b>")
                pose_check.setInformativeText(
                    "Before starting the measurement, place the robot in "
                    "<b><span style='color:#c62828;'>NAV POSE</span></b>.\n\n"
                    "Yes: the robot is ready and collection may start.\n"
                    "No: return to the setup and cancel collection."
                )
                remember = QCheckBox(
                    "Remember this confirmation for the current session"
                )
                pose_check.setCheckBox(remember)
                pose_check.setStandardButtons(QMessageBox.Yes | QMessageBox.No)
                pose_check.setDefaultButton(QMessageBox.No)
                if pose_check.exec_() != QMessageBox.Yes:
                    return
                if remember.isChecked():
                    self.nav_pose_confirmed = True
            self.is_sampling = True
            if self.plot_dialog is not None:
                self.plot_dialog.set_recording_state(True)
            # Determine selected condition based on checkboxes
            if self.cb_flat.isChecked():
                selected_condition = "Flat Surface"
            else:
                # CHANGED TO JUST "Cliff"
                selected_condition = "Cliff"
            selected_env = self.env_combo.currentText()
            calc_samples = self.total_spin.value()
            target_samples = calc_samples + WARMUP_SAMPLES
            self.term1.prepare_sampling(
                target_samples, calc_samples, selected_condition, selected_env
            )
            self.term2.prepare_sampling(
                target_samples, calc_samples, selected_condition, selected_env
            )
            # Make it clear to the user that they can click this again to cancel
            self.sample_btn.setText("Cancel Collection")
            # Disable the 3 individual group boxes
            self.env_group.setEnabled(False)
            self.condition_group.setEnabled(False)
            self.sample_group.setEnabled(False)
            # Use blue to indicate a stopping action is available
            self.set_button_style(self.sample_btn, "blue")
        else:
            self.is_sampling = False
            if self.plot_dialog is not None:
                self.plot_dialog.set_recording_state(False)
            self.sample_btn.setText("Start Collection")
            # Re-enable the 3 individual group boxes
            self.env_group.setEnabled(True)
            self.condition_group.setEnabled(True)
            self.sample_group.setEnabled(True)
            self.set_button_style(self.sample_btn, "green")
            # --- NEW: Check if this was a manual cancellation ---
            # If the progress bar hasn't reached the target, the operator stopped it early
            if self.term1.progress_bar.value() < self.term1.target_samples:
                self.term1.cancel_sampling()
            if self.term2.progress_bar.value() < self.term2.target_samples:
                self.term2.cancel_sampling()

    def set_button_style(self, button, color):
        base_style = "color: white; font-size: 12px; font-weight: bold; border-radius: 4px; padding: 7px 8px;"
        if color == "green":
            button.setStyleSheet(
                f"QPushButton {{ background-color: #5cb85c; border: 1px solid #4cae4c; {base_style} }} QPushButton:hover {{ background-color: #449d44; }}"
            )
        elif color == "blue":
            button.setStyleSheet(
                f"QPushButton {{ background-color: #333333; border: 1px solid #222222; {base_style} }} QPushButton:hover {{ background-color: #111111; }}"
            )

    def show_plot_dialog(self):
        if self.plot_dialog is not None:
            self.plot_dialog.raise_()
            self.plot_dialog.activateWindow()
            return
        self.plot_dialog = SensorPlotDialog([self.term1, self.term2], self)
        screen = QApplication.primaryScreen().availableGeometry()
        dialog_width = min(
            int(screen.width() * PLOT_SCREEN_WIDTH_RATIO),
            max(PLOT_MIN_WIDTH, int(self.width() * PLOT_PARENT_WIDTH_RATIO)),
        )
        dialog_height = min(
            int(screen.height() * PLOT_SCREEN_HEIGHT_RATIO),
            max(PLOT_MIN_HEIGHT, int(self.height() * PLOT_PARENT_HEIGHT_RATIO)),
        )
        self.plot_dialog.resize(dialog_width, dialog_height)
        # Populate both canvases once when opened; hidden canvases are never
        # repainted during sampling.
        geometry = self.plot_dialog.frameGeometry()
        geometry.moveCenter(self.geometry().center())
        self.plot_dialog.move(geometry.topLeft())
        self.plot_dialog.show()
        self.plot_dialog.set_recording_state(self.is_sampling)
        # Render once after the dialog is visible instead of blocking its
        # construction with two immediate Matplotlib repaints.
        self.term1.update_plot_data(redraw=False)
        self.term2.update_plot_data(redraw=False)
        QTimer.singleShot(0, self._draw_open_plots)

    def _draw_open_plots(self):
        if self.plot_dialog is not None:
            self.term1.plot_canvas.draw_idle()
            self.term2.plot_canvas.draw_idle()

    def on_ros_data(self, msg):
        self.term1.latest_data = msg
        self.term2.latest_data = msg

    def update_sampling(self):
        if rospy.is_shutdown():
            QApplication.quit()
        if self.is_sampling:
            done1 = self.term1.process_sampling_data()
            done2 = self.term2.process_sampling_data()
            if done1 and done2:
                # The maximize-style completed plot is available only after
                # both sensors finish the requested collection.
                self.plot_button.setVisible(True)
                if self.plot_dialog is not None:
                    self.plot_dialog.set_recording_state(False)
                self.toggle_sampling()

    def update_monitor(self):
        if rospy.is_shutdown():
            QApplication.quit()
        if self.is_monitoring:
            condition = "Flat Surface" if self.cb_flat.isChecked() else "Cliff"
            env = self.env_combo.currentText()
            self.term1.process_realtime_data()
            self.term2.process_realtime_data()
            if self.plot_dialog is not None:
                changed1 = self.term1.update_realtime_plot(condition, env)
                changed2 = self.term2.update_realtime_plot(condition, env)
                if changed1 or changed2:
                    QTimer.singleShot(0, self._draw_open_plots)


# === Main Execution ===
if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    app.aboutToQuit.connect(lambda: rospy.signal_shutdown("GUI closed"))
    try:
        sys.exit(app.exec_())
    except KeyboardInterrupt:
        pass
