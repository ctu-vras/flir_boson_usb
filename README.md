# FLIR Boson USB ROS Driver (`flir_boson_usb`)

A ROS 1 and ROS 2 USB camera driver for the FLIR Boson thermal camera utilizing V4L2 and OpenCV.

## Prerequisites

Your user must belong to the `video` group to access USB video devices.

```bash
# Grant your user access to USB Video devices
sudo usermod -aG video $USER

# You may need to log out and log back in for the group change to take effect.
```

## Building

### ROS 2

```bash
cd ~/ws
rosdep install --from-paths src --ignore-src -r
colcon build --packages-select flir_boson_usb --symlink-install
source install/setup.bash
```

### ROS 1

```bash
cd ~/ws
rosdep install --from-paths src --ignore-src -r
catkin build flir_boson_usb  # or catkin_make --only-pkg-with-deps flir_boson_usb
source devel/setup.bash
```

## Device Verification

Verify that the system detects the UVC-compliant FLIR Boson hardware:
```bash
lsusb | grep FLIR
v4l2-ctl --list-devices
```

It should return something like the following:
```bash
Boson: FLIR Video (usb-0000:00:14.0-7.3):
	/dev/video4 <-- This one should be used
	/dev/video5
	/dev/media2
```

## Launching the Camera

Use the provided launch file. Arguments can be overridden per-camera:

```bash
# ROS 2
ros2 launch flir_boson_usb flir_boson.launch.xml dev:=/dev/video4 video_mode:=YUV frame_rate:=30.0

# ROS 1
roslaunch flir_boson_usb flir_boson.launch dev:=/dev/video4 video_mode:=YUV frame_rate:=30.0
```

### Video modes

The driver supports two `video_mode` values, each producing a different output stream:

- **`YUV`** (default) — Camera-side DSP handles Automatic Gain Control (AGC) for a contrast-mapped 8-bit image (`mono8` or `bgr8` when `publish_color:=True`). Lowest CPU footprint, ideal for single-board computers or multi camera setup.

- **`RAW16`** — Direct 16-bit bolometer thermal counts (`mono16`). No host processing; ideal for radiometric work or custom feature detection. Some packages might not accept `mono16` directly so be mindful of that.

### Notes on frame rate and performance

- **Hardware rate vs. driver polling.** The `frame_rate` argument controls the software polling loop via a non-blocking poll() architecture. Setting it above hardware limits (e.g., polling at 60Hz on a 9Hz camera) safely drops back to the physical rate with no CPU penalty.

- **Automatic telemetry handling.** If binary telemetry metadata rows are active via the FLIR GUI, the driver reads the full buffer, handles telemetry isolation, and automatically crops the published image back to nominal array sizes (640×512 or 320×256) so standard `CameraInfo` calibrations continue to function perfectly.

### Launch arguments and node parameters

The provided launch files expose all driver parameters as launch args. So most of these arguments also right away describe the node parameters.

| Argument | Description | Default |
| :--- | :--- | :--- |
| `namespace` | ROS namespace for the camera node. | `flir_boson` |
| `frame_id` | Frame used in `header.frame_id`. | `boson_camera` |
| `dev` | The linux file descriptor location for the camera (e.g. `/dev/video4`). | `/dev/video0` |
| `sensor_type` | Physical sensor array size. `Boson_320` or `Boson_640`. The driver cross-checks this against the V4L2-negotiated width and refuses to start on a mismatch. | `Boson_640` |
| `frame_rate` | Frame rate of the camera. Only 9.0/30.0/60.0 supported.| `30.0` |
| `video_mode` | Camera image format. `YUV`: camera-side AGC (`mono8`/`bgr8`, low CPU). `RAW16`: raw 16-bit thermal counts (`mono16`, no camera processing).| `YUV` |
| `zoom_enable` | Digital 2× upscale (`320×256` → `640×512`) of the published image. Only available on `Boson320` cameras. | `False` |
| `publish_color` | In `YUV` mode, publishes a `bgr8` colorized image instead of `mono8`. Use this if a color palette (e.g., Rainbow) is enabled via the FLIR GUI. In `RAW16` mode, this enables the heatmap image. | `False` |
| `raw16_agc_low_pct` | Bottom-tail clip percentage for `RAW16` driver-side AGC (e.g. `1.0` discards the darkest 1% of pixels before linear stretch). Valid range `[0, 50)`; invalid values revert to `1.0`. | `1.0` |
| `raw16_agc_high_pct` | Top-tail clip percentage for `RAW16` driver-side AGC (e.g. `1.0` discards the brightest 1% of pixels before linear stretch). Valid range `[0, 50)`; invalid values revert to `1.0`. | `1.0` |
| `camera_info_url` | Camera calibration file URL (`file://` or `package://`). Empty publishes uncalibrated `CameraInfo`. See [Calibration](#calibration) below. | `""` |
| `point_x` (dynamic param)| X coord of the temperature probe point. | `319` |
| `point_y` (dynamic param) | Y coord of the temperature probe point. | `255` |
| `max_temp_limit` (dynamic param) | Maximum temperature for constant AGC. | `50` |
| `min_temp_limit` (dynamic param) | Minimum temperature for constant AGC. | `20` |
| `norm_margin` (dynamic param) | Constant AGC margin (in 8-bit image units). Non-zero values means the given number of lowest and highest values will not be used in the linear stretch. | `20.0` |

### Tuning the RAW16 AGC percentiles

Defaults of `1.0 / 1.0` (discard 1% from each tail) work well for most scenes. Adjust if:

- The scene contains a small but very hot object (lamp, exhaust, sun) that's compressing the rest of the image — *raise* `raw16_agc_high_pct` (try `2.0` or `5.0`).
- The image looks washed-out or low-contrast — *lower* both percentages toward `0.5`.
- A few dead/stuck pixels are dominating the range — keep low percentages, even `0.1` is usually enough to discard isolated outliers thanks to the histogram-based clipping.

Setting both to `0.0` is equivalent to a pure min/max stretch.

### Dynamically reconfigurable parameters

`point_x`, `point_y`, `max_temp_limit`, `min_temp_limit` and `norm_margin` are dynamic parameters that can be tuned
while the node is running:

```bash
# ROS 2
ros2 param set /flir_boson/flir_boson_usb_node point_x 320

# ROS 1 (dynamic_reconfigure)
rosrun rqt_reconfigure dynparam set /flir_boson/flir_boson_usb_node point_x 320
```

## Published topics

- **`/<namespace>/image_raw`** (`sensor_msgs/msg/Image`) — The primary video stream. Encoding depends on `video_mode`:
  - `YUV` mode: `mono8`, or `bgr8` if `publish_color:=True`
  - `RAW16` mode: `mono16` (raw 16-bit thermal counts)

- **`/<namespace>/camera_info`** (`sensor_msgs/msg/CameraInfo`) — Calibration matrices and metadata, populated from `camera_info_url` if provided.

Both topics are advertised with `rclcpp::SensorDataQoS` (best-effort). `image_transport` additionally exposes `image_raw/compressed`, `image_raw/compressedDepth`, and `image_raw/theora` topics if the corresponding plugins are installed.

## Calibration

This driver publishes empty `CameraInfo` by default. To load a calibration, set `camera_info_url`:

```bash
# ROS 2
ros2 launch flir_boson_usb flir_boson.launch.xml camera_info_url:=file:///home/user/my_boson.yaml

# ROS 1
roslaunch flir_boson_usb flir_boson.launch camera_info_url:=file:///home/user/my_boson.yaml
```

Note: Files inside `example_calibrations/` act purely as format reference models. To create target matrices for your physical lens setup, use the [`camera_calibration`](https://docs.ros.org/en/jazzy/p/camera_calibration/doc/tutorial_mono.html) tools.

`camera_info_url` accepts both `file://` URLs (absolute path on disk) and `package://` URLs (path relative to a ROS package share directory).

## Customizing the RAW16 pipeline

To implement custom radiometric filters, lookup tables, or neural network inference arrays directly on raw thermal data, process the `RAW16` (`mono16`) topic stream externally within an independent node.

## Troubleshooting

- **`ERROR: Invalid Video Device`** — Verify user permissions in the `video` group (`groups` should list it). Ensure correct hardware links via `ls /dev/video*`.

- **`Hardware mismatch! Configured for Boson_640 but V4L2 negotiated width 320`** — Wrong `sensor_type` for the physical camera. Set it to match the actual sensor.

- **`Driver reports YUV bytesperline=X but width=Y`** — The driver is reporting strided YUV buffers, which this node does not currently handle. File an issue with the output of `v4l2-ctl -d <dev> --get-fmt-video` attached.

- **`ros2 topic echo` or `rqt_image_view` shows nothing.** Most likely a QoS mismatch: the driver publishes best-effort, but many tools default to reliable. Either tell the subscriber to use best-effort (`ros2 topic echo /flir_boson/image_raw --qos-reliability best_effort`) or run `rqt_image_view` which negotiates QoS automatically.

- **`RAW16` AGC output looks washed out or saturated.** Tune `raw16_agc_low_pct` and `raw16_agc_high_pct` — see [Tuning the RAW16_AGC percentiles](#tuning-the-raw16-agc-percentiles).

## Credits & lineage

This package is a substantial rewrite of two earlier open-source projects which is
built for both ROS generations (the ROS 2 rewrite was contributed by the port
maintainers, the ROS 1 nodelet was restored on top of it). Credit to the original authors:

1. **[FLIR Systems / BosonUSB](https://github.com/FLIR/BosonUSB)** — Foundational V4L2 C++ interactions and 16-bit to 8-bit AGC conversions.
2. **[AutonomouStuff / flir_boson_usb](https://github.com/astuff/flir_boson_usb)** — Original ROS 1 wrapper, nodelet architecture, and RAW16 image processing filters.
2. **[GITAI / flir_boson_usb](https://github.com/GITAI/flir_boson_usb)** — Extension of the ROS 1 driver with more 8-bit outputs.

Both the original codebases and this port are released under the MIT License.