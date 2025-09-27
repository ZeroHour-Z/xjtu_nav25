rm_localization_bringup
=======================

Centralized bringup package for rm_localization. All launch files and common parameter YAMLs live here so other backend packages (fast_lio, faster_lio_ros2, point_lio, fast_lio_localization_ros2) do not need git changes.

Usage
-----

- Fast-LIO mapping (default):
  `ros2 launch rm_localization_bringup localization_bringup.launch.py backend:=fast_lio`

- FASTER-LIO mapping:
  `ros2 launch rm_localization_bringup localization_bringup.launch.py backend:=faster_lio`

- Point-LIO mapping:
  `ros2 launch rm_localization_bringup localization_bringup.launch.py backend:=point_lio`

- Override params file, enable global localization helpers and RViz:
  `ros2 launch rm_localization_bringup localization_bringup.launch.py backend:=point_lio point_lio_params:=/path/to.yaml run_global_localization:=true rviz:=true`

Parameter YAMLs are under `config/`. Edit them here to keep all robot-specific settings in one place.

