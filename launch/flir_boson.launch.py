from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Launch the FLIR Boson USB camera driver.

    To load a camera calibration, pass camera_info_url on the command line.
    Common patterns:

      # A calibration shipped with this package (for quick testing only,
      # intrinsics are NOT guaranteed to match your physical camera):
      camera_info_url:=package://flir_boson_usb/example_calibrations/Boson640.yaml

      # Your own calibration file on disk:
      camera_info_url:=file:///home/user/calibrations/my_boson.yaml

      # A calibration in another ROS package:
      camera_info_url:=package://my_robot_calibrations/boson_front.yaml

    Leave camera_info_url empty (the default) to publish uncalibrated
    CameraInfo. Downstream nodes that require calibration will need it
    set explicitly.
    """

    namespace_arg = DeclareLaunchArgument(
        'namespace', default_value='flir_boson',
        description='The ROS namespace for the camera node'
    )
    frame_id_arg = DeclareLaunchArgument(
        'frame_id', default_value='boson_camera',
        description='The TF frame ID stamped on each Image header'
    )
    dev_arg = DeclareLaunchArgument(
        'dev', default_value='/dev/video0',
        description='The Linux video device path for the camera'
    )
    frame_rate_arg = DeclareLaunchArgument(
        'frame_rate', default_value='30.0',
        description='V4L2 polling rate in Hz. Any positive value is accepted; '
                    'invalid values are clamped to 1.0 Hz. Typical Boson '
                    'hardware rates are 9, 30, or 60.'
    )
    video_mode_arg = DeclareLaunchArgument(
        'video_mode', default_value='YUV',
        description='YUV: hardware-AGC mono8/bgr8 from camera DSP (low CPU). '
                    'RAW16: raw 16-bit thermal counts, published as mono16. '
                    'RAW16_AGC: host-side percentile AGC, published as mono8.'
    )
    zoom_enable_arg = DeclareLaunchArgument(
        'zoom_enable', default_value='False',
        description='Digital 2x upscale (320x256 to 640x512). Only honored on '
                    'Boson_320 in RAW16_AGC mode.'
    )
    publish_color_arg = DeclareLaunchArgument(
        'publish_color', default_value='False',
        description='Publish 3-channel BGR colorized output instead of mono8 '
                    '(YUV mode only)'
    )
    raw16_agc_low_pct_arg = DeclareLaunchArgument(
        'raw16_agc_low_pct', default_value='1.0',
        description='Bottom-tail clip percentage for RAW16_AGC (e.g. 1.0 '
                    'discards the darkest 1% of pixels). Valid range: [0, 50).'
    )
    raw16_agc_high_pct_arg = DeclareLaunchArgument(
        'raw16_agc_high_pct', default_value='1.0',
        description='Top-tail clip percentage for RAW16_AGC (e.g. 1.0 '
                    'discards the brightest 1% of pixels). Valid range: [0, 50).'
    )
    sensor_type_arg = DeclareLaunchArgument(
        'sensor_type', default_value='Boson_640',
        description='Physical sensor array size. Boson_320 or Boson_640.'
    )
    camera_info_url_arg = DeclareLaunchArgument(
        'camera_info_url', default_value='',
        description='Camera calibration file URL (file:// or package://). '
                    'Empty (default) publishes uncalibrated CameraInfo. See '
                    'header docstring for examples.'
    )

    boson_camera_node = Node(
        package='flir_boson_usb',
        executable='boson_camera_node',
        name='flir_boson_usb_node',
        namespace=LaunchConfiguration('namespace'),
        output='screen',
        parameters=[{
            'frame_id': LaunchConfiguration('frame_id'),
            'dev': LaunchConfiguration('dev'),
            'frame_rate': LaunchConfiguration('frame_rate'),
            'video_mode': LaunchConfiguration('video_mode'),
            'zoom_enable': LaunchConfiguration('zoom_enable'),
            'publish_color': LaunchConfiguration('publish_color'),
            'raw16_agc_low_pct': LaunchConfiguration('raw16_agc_low_pct'),
            'raw16_agc_high_pct': LaunchConfiguration('raw16_agc_high_pct'),
            'sensor_type': LaunchConfiguration('sensor_type'),
            'camera_info_url': LaunchConfiguration('camera_info_url'),
        }]
    )

    return LaunchDescription([
        namespace_arg,
        frame_id_arg,
        dev_arg,
        frame_rate_arg,
        video_mode_arg,
        zoom_enable_arg,
        publish_color_arg,
        raw16_agc_low_pct_arg,
        raw16_agc_high_pct_arg,
        sensor_type_arg,
        camera_info_url_arg,
        boson_camera_node,
    ])