#!/usr/bin/env python3
# Centralized bringup for rm_localization backends and optional localization helpers

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Core args
    backend_arg = DeclareLaunchArgument(
        "backend",
        default_value="fast_lio",
        description="Backend to run: fast_lio | faster_lio | point_lio",
    )
    use_sim_time_arg = DeclareLaunchArgument("use_sim_time", default_value="false")
    run_rviz_arg = DeclareLaunchArgument("rviz", default_value="true")

    # Param files per backend (defaults to bringup/config)
    fast_lio_params_arg = DeclareLaunchArgument(
        "fast_lio_params",
        default_value=PathJoinSubstitution(
            [
                FindPackageShare("rm_localization_bringup"),
                "config",
                "fast_lio_mid360.yaml",
            ]
        ),
        description="YAML for fast_lio node",
    )
    faster_lio_params_arg = DeclareLaunchArgument(
        "faster_lio_params",
        default_value=PathJoinSubstitution(
            [
                FindPackageShare("rm_localization_bringup"),
                "config",
                "faster_lio_ros2.yaml",
            ]
        ),
        description="YAML for faster_lio_ros2 node",
    )
    point_lio_ros2_params_arg = DeclareLaunchArgument(
        "point_lio_ros2_params",
        default_value=PathJoinSubstitution(
            [
                FindPackageShare("rm_localization_bringup"),
                "config",
                "point_lio_ros2_mid360.yaml",
            ]
        ),
        description="YAML for point_lio_ros2 node",
    )

    # Optional global localization helpers
    run_global_arg = DeclareLaunchArgument(
        "run_global_localization", default_value="false"
    )
    map_pcd_arg = DeclareLaunchArgument(
        "map_pcd",
        default_value=PathJoinSubstitution(
            [FindPackageShare("rm_localization_bringup"), "PCD", "blue", "map.pcd"]
        ),
    )

    # Common configurations
    backend = LaunchConfiguration("backend")
    use_sim_time = LaunchConfiguration("use_sim_time")
    rviz = LaunchConfiguration("rviz")
    fast_lio_params = LaunchConfiguration("fast_lio_params")
    faster_lio_params = LaunchConfiguration("faster_lio_params")
    point_lio_ros2_params = LaunchConfiguration("point_lio_ros2_params")
    run_global = LaunchConfiguration("run_global_localization")
    map_pcd = LaunchConfiguration("map_pcd")

    # Backend nodes
    from launch.substitutions import PythonExpression

    fast_lio_node = Node(
        package="fast_lio",
        executable="fastlio_mapping",
        name="fastlio_mapping",
        output="screen",
        parameters=[fast_lio_params, {"use_sim_time": use_sim_time}],
        condition=IfCondition(PythonExpression(["'", backend, "' == 'fast_lio'"])),
    )

    faster_lio_node = Node(
        package="faster_lio_ros2",
        executable="run_mapping_online",
        name="laser_mapping",
        output="screen",
        parameters=[faster_lio_params, {"use_sim_time": use_sim_time}],
        condition=IfCondition(PythonExpression(["'", backend, "' == 'faster_lio'"])),
    )

    point_lio_ros2_node = Node(
        package="point_lio_ros2",
        executable="pointlio_mapping",
        name="pointlio_mapping",
        output="screen",
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
        parameters=[point_lio_ros2_params, {"use_sim_time": use_sim_time}],
        condition=IfCondition(
            PythonExpression(["'", backend, "' == 'point_lio'"])
        ),
    )

    # Optional: global localization helpers + TFs (using fast_lio_localization_ros2)
    pcd_pub = Node(
        package="fast_lio_localization_ros2",
        executable="pcd_publisher",
        name="map_publisher",
        output="screen",
        parameters=[
            {
                "map": map_pcd,
                "frame_id": "map3d",
                "rate": 1.0,
                "use_sim_time": use_sim_time,
            }
        ],
        condition=IfCondition(run_global),
    )

    global_loc = Node(
        package="fast_lio_localization_ros2",
        executable="global_localization",
        name="global_localization",
        output="screen",
        parameters=[
            {
                "map2odom_completed": False,
                "region": 0,
                "use_sim_time": use_sim_time,
                "map_frame": "map3d",
                "odom_frame": "camera_init",
                "base_link_frame": "base_link",
            }
        ],
        condition=IfCondition(run_global),
    )

    transform_fusion = Node(
        package="fast_lio_localization_ros2",
        executable="transform_fusion",
        name="transform_fusion",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "map_frame": "map3d",
                "odom_frame": "camera_init",
                "base_link_frame": "base_link",
            }
        ],
        condition=IfCondition(run_global),
    )

    # Static TFs commonly used in existing launch files
    tf_body2base = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_body2base_link",
        arguments=[
            "0.0",
            "0.12",
            "-0.28",
            "1.5707963267948966",
            "0.27",
            "0",
            "body",
            "base_link",
        ],
        condition=IfCondition(run_global),
    )

    tf_map3dto2d = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_map3dto2d",
        arguments=["0", "0", "0.24", "-1.5707963267948966", "0", "0", "map", "map3d"],
        condition=IfCondition(run_global),
    )

    tf_base_link2realsense = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_base_link2realsense",
        arguments=["0.3", "0", "0.28", "0", "0", "0", "base_link", "camera_link"],
        condition=IfCondition(run_global),
    )

    # RViz (backend-specific default configs are in backend pkgs; use any available)
    # For simplicity, use point_lio_ros2 rviz if present.
    from ament_index_python.packages import get_package_share_directory
    import os

    try:
        rviz_cfg = os.path.join(
            get_package_share_directory("point_lio_ros2"), "rviz_cfg", "localize.rviz"
        )
    except Exception:
        rviz_cfg = ""
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_cfg] if rviz_cfg else [],
    )

    return LaunchDescription(
        [
            backend_arg,
            use_sim_time_arg,
            run_rviz_arg,
            fast_lio_params_arg,
            faster_lio_params_arg,
            point_lio_ros2_params_arg,
            run_global_arg,
            map_pcd_arg,
            fast_lio_node,
            faster_lio_node,
            point_lio_ros2_node,
            GroupAction([rviz_node], condition=IfCondition(rviz)),
            pcd_pub,
            global_loc,
            transform_fusion,
            tf_body2base,
            tf_map3dto2d,
            tf_base_link2realsense,
        ]
    )
