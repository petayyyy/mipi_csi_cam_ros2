"""
MIPI CSI camera launch for Raspberry Pi 5 and RK3588/RK3588S boards.

Usage:
  # Raspberry Pi 5 through libcamerasrc:
  ros2 launch mipi_csi_cam_ros2 camera.launch.py source_type:=libcamera sensor:=imx219 width:=1280 height:=960 fps:=30 camera_id:=1

  # RK3588/RK3588S through rkisp1 V4L2 capture:
  ros2 launch mipi_csi_cam_ros2 camera.launch.py source_type:=v4l2 sensor:=ov5647 width:=640 height:=480 fps:=25 camera_id:=1

  # With resize (e.g. publish 640x480 from a 1280x960 capture):
  ros2 launch mipi_csi_cam_ros2 camera.launch.py sensor:=imx219 width:=1280 height:=960 \\
      resize_w:=640 resize_h:=480 fps:=30 camera_id:=1

  # Multiple cameras (specify libcamera camera-name):
  ros2 launch mipi_csi_cam_ros2 camera.launch.py sensor:=imx219 camera_id:=0 \\
      camera_name:="/base/axi/pcie@1000120000/rp1/i2c@88000/imx219@10"

Publishes:
  /camera_{id}/image_raw    — sensor_msgs/Image
  /camera_{id}/camera_info  — sensor_msgs/CameraInfo (latched)
  TF: base_link → camera_optical_{id}
"""

import os
import sys
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    PythonExpression,
    TextSubstitution,
    EqualsSubstitution,
)
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('mipi_csi_cam_ros2')
    config_dir = os.path.join(pkg_share, 'config')

    # Load camera TFs from params.yaml
    camera_tfs = {}
    params_path = os.path.join(config_dir, 'params.yaml')
    print(f'[mipi_csi_cam_ros2] Loading camera TF from: {params_path}', file=sys.stderr)
    if os.path.exists(params_path):
        try:
            with open(params_path, 'r') as f:
                cfg = yaml.safe_load(f) or {}
            for i in range(3):
                key = f'camera_tf_{i}'
                src = cfg.get(key, {})
                src = src.get('ros__parameters', src)
                camera_tfs[str(i)] = {
                    'parent_frame': str(src.get('parent_frame', 'base_link')),
                    'xyz': [str(x) for x in src.get('xyz', [0.0, 0.0, 0.0])],
                    'rpy': [str(r) for r in src.get('rpy', [0.0, 0.0, 0.0])],
                }
            print(f'[mipi_csi_cam_ros2] Loaded camera_tfs: {camera_tfs}', file=sys.stderr)
        except Exception as e:
            print(f'[mipi_csi_cam_ros2] Error loading params.yaml: {e}', file=sys.stderr)

    ld_items = [
        DeclareLaunchArgument('source_type', default_value='libcamera',
                              description='Capture backend: libcamera | v4l2'),
        DeclareLaunchArgument('sensor',      default_value='imx219',
                              description='Sensor name: imx219 | ov5647 | imx708'),
        DeclareLaunchArgument('width',       default_value='1280'),
        DeclareLaunchArgument('height',      default_value='960'),
        DeclareLaunchArgument('fps',         default_value='30'),
        DeclareLaunchArgument('camera',      default_value='1',
                              description='Alias for camera_id'),
        DeclareLaunchArgument('camera_id',   default_value=LaunchConfiguration('camera')),
        DeclareLaunchArgument('resize_w',    default_value='0',
                              description='Output width (0 = same as capture)'),
        DeclareLaunchArgument('resize_h',    default_value='0',
                              description='Output height (0 = same as capture)'),
        DeclareLaunchArgument('video_device', default_value='/dev/video11',
                              description='V4L2 capture device for source_type=v4l2'),
        DeclareLaunchArgument('format', default_value='NV12',
                              description='V4L2 caps pixel format for source_type=v4l2'),
        DeclareLaunchArgument('io_mode', default_value='dmabuf',
                              description='V4L2 io-mode: auto | dmabuf | mmap'),
        DeclareLaunchArgument('v4l2_use_framerate_caps', default_value='false',
                              description='Append framerate caps to the V4L2 pipeline'),
        DeclareLaunchArgument('qos_reliable', default_value='false',
                              description='QoS reliability (default: false=BEST_EFFORT, no back-pressure)'),
        DeclareLaunchArgument('camera_name', default_value='',
                              description='libcamera camera-name property (empty = auto)'),
    ]

    source_type  = LaunchConfiguration('source_type')
    sensor       = LaunchConfiguration('sensor')
    width        = LaunchConfiguration('width')
    height       = LaunchConfiguration('height')
    fps          = LaunchConfiguration('fps')
    camera_id    = LaunchConfiguration('camera_id')
    resize_w     = LaunchConfiguration('resize_w')
    resize_h     = LaunchConfiguration('resize_h')
    video_device = LaunchConfiguration('video_device')
    pixel_format = LaunchConfiguration('format')
    io_mode      = LaunchConfiguration('io_mode')
    v4l2_use_framerate_caps = LaunchConfiguration('v4l2_use_framerate_caps')
    qos_reliable = LaunchConfiguration('qos_reliable')
    camera_name  = LaunchConfiguration('camera_name')

    # /camera_{id}/image_raw
    topic = PythonExpression(["'/camera_' + str('", camera_id, "') + '/image_raw'"])

    # frame_id: camera_optical_{id}
    frame_id = PythonExpression(["'camera_optical_' + str('", camera_id, "')"])

    # calibration file path: {config_dir}/{sensor}_{width}x{height}.yaml
    calibration_file = PythonExpression([
        "'", config_dir, "/' + str('", sensor,
        "') + '_' + str('", width,
        "') + 'x' + str('", height, "') + '.yaml'",
    ])

    ld_items.append(
        Node(
            package='mipi_csi_cam_ros2',
            executable='mipi_csi_cam_ros2_node',
            name=PythonExpression(["'camera_' + str('", camera_id, "')"]),
            parameters=[{
                'topic':            topic,
                'source_type':      source_type,
                'sensor':           sensor,
                'fps':              fps,
                'width':            width,
                'height':           height,
                'resize_w':         resize_w,
                'resize_h':         resize_h,
                'video_device':     video_device,
                'format':           pixel_format,
                'io_mode':          io_mode,
                'v4l2_use_framerate_caps': v4l2_use_framerate_caps,
                'qos_reliable':     qos_reliable,
                'calibration_file': calibration_file,
                'frame_id':         frame_id,
                'camera_name':      camera_name,
            }],
            output='screen',
        )
    )

    # Static TF publisher (one per camera_id)
    for cam_id, tf_cfg in camera_tfs.items():
        ld_items.append(
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name=[TextSubstitution(text='camera_tf_'),
                      TextSubstitution(text=cam_id)],
                condition=IfCondition(
                    EqualsSubstitution(camera_id, cam_id)
                ),
                arguments=[
                    tf_cfg['xyz'][0], tf_cfg['xyz'][1], tf_cfg['xyz'][2],
                    tf_cfg['rpy'][0], tf_cfg['rpy'][1], tf_cfg['rpy'][2],
                    tf_cfg['parent_frame'],
                    [TextSubstitution(text='camera_optical_'),
                     TextSubstitution(text=cam_id)],
                ],
                output='screen',
            )
        )

    return LaunchDescription(ld_items)
