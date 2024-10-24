#******************************************************************************
#	                   ROS2 Pubilhser Example
#	                      Launch File
#          Copyright (C) 2021 Walter Fetter Lages <w.fetter@ieee.org>
#
#        This program is free software: you can redistribute it and/or modify
#        it under the terms of the GNU General Public License as published by
#        the Free Software Foundation, either version 3 of the License, or
#        (at your option) any later version.
#
#        This program is distributed in the hope that it will be useful, but
#        WITHOUT ANY WARRANTY; without even the implied warranty of
#        MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
#        Geneal Public License for more details.
#
#        You should have received a copy of the GNU General Public License
#        along with this program.  If not, see
#        <http://www.gnu.org/licenses/>.
#        
#*******************************************************************************

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            name='turtle',
            package='turtlesim',
            executable='turtlesim_node',
            output='screen'
        ),
        Node(
            name='turtle_cmd',
            package='turtle_command',
            executable='turtle_command',
            remappings=[('/turtle/cmd','/turtle1/cmd_vel')],
            output='screen'
        )
    ])
