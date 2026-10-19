# IR diagnostics application

Run from the workspace root:

```bash
python3 -m ir_app.ir
```

## Modules

- `ir.py`: small application entry point.
- `ui.py`: PyQt5 windows, plots, controls, ROS event handling, and UI sizes.
- `ir_processing.py`: sensor configuration, pass ranges, and pure validation.

The original `../ir.py` is preserved. Use the package entry point while the
refactor is being verified against the robot setup.
