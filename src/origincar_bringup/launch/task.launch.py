#!/usr/bin/env python3
"""Terminal 1: start TaskMaster only.

The vehicle stack, perception, speech and PC bridge must run in the other
two terminals started by start_competition_3term.sh.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    mission_launch = os.path.join(
        get_package_share_directory('origincar_bringup'),
        'launch',
        'mission.launch.py',
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(mission_launch),
            launch_arguments={
                'start_task': 'true',
                'start_pc_bridge': 'false',
                'start_broadcast': 'false',
            }.items(),
        ),
    ])
