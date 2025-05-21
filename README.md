# Robot Simulation and Analysis Project

This repository contains Python scripts and modules for simulating and analyzing robot behavior, including belt velocity calculations, trajectory visualization, and parameter optimization.

## Project Structure

```
├── belt_velocity.py
├── Full robot calculation
│   ├── constants.py
│   ├── court_visualization.py
│   ├── image.png
│   ├── main.py
│   ├── parameter_optimization.py
│   ├── physics.py
│   ├── __pycache__
│   │   ├── constants.cpython-310.pyc
│   │   ├── court_visualization.cpython-310.pyc
│   │   ├── parameter_optimization.cpython-310.pyc
│   │   ├── physics.cpython-310.pyc
│   │   ├── trajectory_visualization.cpython-310.pyc
│   │   └── ui_components.cpython-310.pyc
│   ├── README.md
│   ├── trajectory_visualization.py
│   └── ui_components.py
├── linearRegressionData.py
└── README.md
```

## File Descriptions

### Root Directory
- **`belt_velocity.py`**: Script for calculating belt velocity in the robot system.
- **`linearRegressionData.py`**: Script for performing linear regression on robot-related data.
- **`README.md`**: This file.

### `Full robot calculation` Directory
- **`constants.py`**: Contains constants used across the simulation (e.g., gravitational acceleration, robot dimensions).
- **`court_visualization.py`**: Script for visualizing the robot's environment or "court."
- **`image.png`**: Example output or reference image for visualization.
- **`main.py`**: Entry point for running the full robot simulation.
- **`parameter_optimization.py`**: Script for optimizing robot parameters (e.g., speed, angle).
- **`physics.py`**: Implements physics calculations for the robot (e.g., motion, collisions).
- **`trajectory_visualization.py`**: Script for plotting the robot's trajectory.
- **`ui_components.py`**: Contains UI components for interactive simulations (if applicable).

## Usage

1. **Run the main simulation**:
   ```bash
   cd Full\ robot\ calculation
   python3 main.py
   ```

2. **Calculate belt velocity**:
   ```bash
   python3 belt_velocity.py
   ```

3. **Perform linear regression**:
   ```bash
   python3 linearRegressionData.py
   ```

## Dependencies
- Python 3.10+
- Libraries:
  - `numpy`
  - `matplotlib`
  - `scipy` (for optimization)
  - `pandas` (for data handling)

## Contributing
Pull requests are welcome. For major changes, please open an issue first to discuss the proposed changes.

## License
[MIT](https://choosealicense.com/licenses/mit/)

