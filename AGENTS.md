# Repository Guidelines

This workspace contains ROS2 Humble packages for PX4 offboard and NMPC control plus Python-based NMPC code generation.

## Project Structure & Module Organization
- ROS2 packages live under `src/` (e.g. `src/px4_offboard_control`, `src/px4_nmpc`, `src/px4_payload_nmpc`).
- Python NMPC and model generators are in `px4/`, `px4_payload/`, and `multi_payload/` (do not move generated C code out of their `c_generated_code/` folders).
- `build/`, `install/`, and `log/` are colcon artifacts and must not be committed.

## Build, Test, and Development Commands
- Build selected packages: `colcon build --packages-select px4_offboard_control px4_nmpc px4_payload_nmpc`.
- Source workspace before running: `source install/setup.bash`.
- Regenerate NMPC solvers when models change, e.g. `python3 px4_payload/px4_payload_nmpc.py` (similar scripts exist in `px4/` and `multi_payload/`).
- Run nodes via launch files, e.g. `ros2 launch px4_offboard_control offboard_control.launch.py`.
- When tests are added, prefer `colcon test` from the workspace root.

## Coding Style & Naming Conventions
- C++: 4-space indentation, brace on new line (match existing files), prefer `PascalCase` for classes, `snake_case_` for member variables, and descriptive ROS2 node/parameter names.
- Python: follow PEP8 (4 spaces, `snake_case` for functions and variables); keep scripts executable with `python3`.
- Topics, frames, and parameters should use clear, lowercase `snake_case` (e.g. `/mavros/setpoint_position/local`, `control_frequency`).

## Testing Guidelines
- Add C++ tests using `ament_cmake_gtest` under each package’s `test/` directory; name tests after the component under test (e.g. `test_nmpc_controller.cpp`).
- Add Python tests with `pytest` where appropriate and document how to run them.
- For behavior changes, validate in PX4 SITL + Gazebo following the package READMEs (offboard control, payload NMPC, etc.).

## Commit & Pull Request Guidelines
- Use clear, imperative commit messages; prefix with the main package when helpful (e.g. `px4_payload_nmpc: fix offboard delay`).
- Keep changes focused; update launch files, configs, and README snippets when behavior or interfaces change.
- For PRs, describe the motivation, main changes, test or simulation steps (commands you ran), and include logs or screenshots for flight behavior changes.

## Agent-Specific Instructions
- Keep edits minimal and localized; preserve existing code style and ROS2/PX4 patterns.
- Do not introduce new external dependencies without strong justification and clear build instructions.
