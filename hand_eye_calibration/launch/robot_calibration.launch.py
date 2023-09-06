# Copyright 2023 Trossen Robotics
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the copyright holder nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
    TextSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
import yaml


from easy_handeye2.common_launch import arg_calibration_type, arg_tracking_base_frame, arg_tracking_marker_frame, arg_robot_base_frame, \
    arg_robot_effector_frame


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def launch_setup(context, *args, **kwargs):

    calibration_launch_arg = LaunchConfiguration('calibrate')
    evaluation_launch_arg = LaunchConfiguration('evaluate')
    publish_launch_arg = LaunchConfiguration('publish')
    camera1_name = LaunchConfiguration('cam1_name').perform(context)
    camera2_name = LaunchConfiguration('cam2_name').perform(context)
    camera3_name = LaunchConfiguration('cam3_name').perform(context)

    calib_name_launch_arg = LaunchConfiguration('calib_name')

    depthai_prefix = get_package_share_directory("depthai_ros_driver")
    interbotix_prefix = get_package_share_directory("interbotix_xscobot_moveit")
    easy_handeye_prefix = get_package_share_directory("easy_handeye2")
    calibration_prefix = get_package_share_directory("hand_eye_calibration")

    interbotix_moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(interbotix_prefix, 'launch',
                         'xscobot_moveit.launch.py')
        ),
        launch_arguments={"robot_model": "dx400",
                          "hardware_type": "fake"}.items())

    spatial_rgbd1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(depthai_prefix, 'launch',
                         'rgbd_pcl.launch.py')
        ),
        launch_arguments={"name": camera1_name,
                        #   "parent_frame": "dx400/ee_gripper_link",
                          "params_file": PathJoinSubstitution([calibration_prefix, 'config', 'camera_config.yaml']),
                          "cam_pos_z": str(0.1)}.items())

    spatial_rgbd2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(depthai_prefix, 'launch',
                         'rgbd_pcl.launch.py')
        ),
        launch_arguments={"name": camera2_name,
                        #   "parent_frame": "dx400/ee_gripper_link",
                          "params_file": PathJoinSubstitution([calibration_prefix, 'config', 'camera_config.yaml']),
                          "cam_pos_z": str(-0.1)
                          }.items())

    spatial_rgbd3 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(depthai_prefix, 'launch',
                         'rgbd_pcl.launch.py')
        ),
        launch_arguments={"name": camera3_name,
                        #   "parent_frame": "dx400/ee_gripper_link",
                          "params_file": PathJoinSubstitution([calibration_prefix, 'config', 'camera_config.yaml']),
                          "cam_pos_z": str(0.4)
                          }.items())

    apriltag_detection = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(calibration_prefix, 'launch',
                         'apriltag_detector.launch.py')
        ),
        launch_arguments={"camera_color_topic": "/"+camera1_name+"/rgb/image_raw",
                          "camera_info_topic": "/"+camera1_name+"/rgb/camera_info",
                          "camera_frame": "oak_rgb_camera_optical_frame"
                          }.items())

    easy_handeye_calibration = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(easy_handeye_prefix, 'launch',
                         'calibrate.launch.py')
        ),
        launch_arguments={"calibration_type": "eye_on_base",
                          "name": calib_name_launch_arg.perform(context),
                          "robot_base_frame": "dx400/base_link",
                          "robot_effector_frame": "dx400/ee_gripper_link",
                          "tracking_base_frame": "oak_rgb_camera_optical_frame",
                          "tracking_marker_frame": "tag_5"}.items(),
        condition=LaunchConfigurationEquals(
            launch_configuration_name='calibrate',
            expected_value='true'
            ))

    easy_handeye_evaluation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(easy_handeye_prefix, 'launch',
                         'evaluate.launch.py')
        ),
        launch_arguments={"name": calib_name_launch_arg.
                          perform(context)}.items(),
        condition=LaunchConfigurationEquals(
            launch_configuration_name='evaluate',
            expected_value='true'
            ))

    easy_handeye_publish = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(easy_handeye_prefix, 'launch',
                         'publish.launch.py')
        ),
        launch_arguments={"name": calib_name_launch_arg.
                          perform(context)}.items(),
        condition=LaunchConfigurationEquals(
            launch_configuration_name='publish',
            expected_value='true'
            ))



    return [
       interbotix_moveit,
       spatial_rgbd1,
    #    spatial_rgbd2,
    #    spatial_rgbd3,
       apriltag_detection,
       easy_handeye_calibration,
    #    easy_handeye_evaluation,
    #    easy_handeye_publish,

    ]


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            'calibrate',
            default_value='false',
            choices=('true', 'false'),
            description="Launches Hand-Eye Calibration",
        ),
        DeclareLaunchArgument(
            'evaluate',
            default_value='false',
            choices=('true', 'false'),
            description="Launches Hand-Eye Calibration Evaluation",
        ),
        DeclareLaunchArgument(
            'publish',
            default_value='false',
            choices=('true', 'false'),
            description="Publishes Hand-Eye Calibration values",
        ),
        DeclareLaunchArgument(
            'calib_name',
            default_value='eob_cam1',
            description="Name of the calibration values to save",
        ),
        DeclareLaunchArgument(
            'cam1_name',
            default_value='oak1',
            description="Name of camera 1",
        ),
        DeclareLaunchArgument(
            'cam2_name',
            default_value='oak2',
            description="Name of camera 2",
        ),
        DeclareLaunchArgument(
            'cam3_name',
            default_value='oak3',
            description="Name of camera 3",
        )
    ]

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
