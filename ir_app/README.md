# TFmini IR Sensor Diagnostics Tool

A ROS and PyQt5 desktop application for monitoring, sampling, and validating
TFmini distance sensors on the robot.

The tool provides live telemetry, configurable sampling, pass/fail statistics,
realtime plots, recording plots, and PNG/PDF/SVG export.

## Features

- Monitor IR sensors 1–4 through the ROS sensor topic.
- Display raw distance, angle, and beam-width data.
- Collect configurable sample sets with a 13 ms acquisition cycle.
- Calculate average, standard deviation, minimum, maximum, and fail rate.
- Validate Flat Surface and Cliff conditions against configured ranges.
- Show IR1 and IR2 in a combined plot window.
- Switch between `LIVE` and `RECORDING` plot modes.
- Reset the plot when the environment or condition changes.
- Export plot results with environment, condition, and sample information.
- Throttle plot redraws to reduce CPU usage.

## Project structure

```text
ir_app/
├── ir.py              # Application entry point
├── ui.py              # PyQt5 interface, ROS monitor, sampling, and plots
├── ir_processing.py   # Sensor geometry, ranges, and validation helpers
└── README.md          # Project documentation
```

## Requirements

- Python 3
- ROS with `rospy`
- PyQt5
- Matplotlib
- `aeolus_robio_msgs` ROS message package
- TFmini topic: `/robot/center/ir/sensor`

## Run

Run from the workspace root after sourcing the ROS environment:

```bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash
python3 ir_app/ir.py
```

## Main workflow

1. Select the environment.
2. Select `Flat Surface` or `Cliff`.
3. Select the sample count.
4. Confirm that the robot is in NAV POSE when prompted.
5. Click `Start Collection`.
6. Review the statistics and recording plot after sampling finishes.
7. Use `LIVE VIEW` when realtime visualization is required.

During recording, the plot is paused so the acquisition loop remains light.
Raw sensor data continues to be collected at the configured sampling rate.

## UI sizing

Main window and plot sizing values are near the top of `ui.py`:

```python
MAIN_WINDOW_WIDTH = 820
MAIN_WINDOW_HEIGHT = 700
PLOT_MIN_WIDTH = 850
PLOT_MIN_HEIGHT = 600
```

Adjust these values when the application needs to fit a different display.

## Git notes

The workspace `.gitignore` excludes Python cache files, virtual environments,
logs, ROS bag files, and generated media.

```bash
git add ir_app .gitignore
git commit -m "Add TFmini IR sensor diagnostics tool"
git push -u origin main
```
