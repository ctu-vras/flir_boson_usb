# FLIR Boson USB ROS 2 Driver (`flir_boson_usb`)

A ROS 2 (Humble) USB camera driver for the FLIR Boson thermal camera utilizing V4L2 and OpenCV.

The node is written as an `rclcpp_components` composable node to enable Intra-Process Communication (IPC) pipelines. Topics are advertised using `rclcpp::SensorDataQoS` (best-effort, shallow depth) to optimize high-rate video delivery by dropping stale frames.

## Prerequisites

Your user must belong to the `video` group to access USB video devices.

```bash
# Navigate to your workspace root
cd ~/ros2_ws

# Install all dependencies (including v4l-utils and ROS packages)
rosdep install --from-paths src --ignore-src -r -y

# Grant your user access to USB Video devices
sudo usermod -aG video $USER
```

> You may need to log out and log back in for the group change to take effect.

## Building

```bash
cd ~/ros2_ws
colcon build --packages-select flir_boson_usb --symlink-install
source install/setup.bash
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

Use the provided Python launch file. Arguments can be overridden per-camera:

```bash
ros2 launch flir_boson_usb flir_boson.launch.py \
    dev:=/dev/video0 \
    video_mode:=YUV \
    frame_rate:=30.0
```

### Video modes

The driver supports three `video_mode` values, each producing a different output stream:

- **`YUV`** (default) — Camera-side DSP handles Automatic Gain Control (AGC) for a contrast-mapped 8-bit image (`mono8` or `bgr8` when `publish_color:=True`). Lowest CPU footprint, ideal for single-board computers or multi camera setup.

- **`RAW16`** — Direct 16-bit bolometer thermal counts (`mono16`). No host processing; ideal for radiometric work or custom feature detection. Some packages might not accept `mono16` directly so be mindful of that.

- **`RAW16_AGC`** —Direct 16-bit source with host-side percentile-clipping AGC mapped to `mono8`. Tunable via `raw16_agc_low_pct` / `raw16_agc_high_pct`. More robust to outlier pixels and isolated hot spots than the camera-internal AGC, at the cost of CPU time per frame.

### Notes on frame rate and performance

- **Hardware rate vs. driver polling.** The `frame_rate` argument controls the software polling loop via a non-blocking poll() architecture. Setting it above hardware limits (e.g., polling at 60Hz on a 9Hz camera) safely drops back to the physical rate with no CPU penalty.

- **Automatic telemetry handling.** If binary telemetry metadata rows are active via the FLIR GUI, the driver reads the full buffer, handles telemetry isolation, and automatically crops the published image back to nominal array sizes (640×512 or 320×256) so standard `CameraInfo` calibrations continue to function perfectly.

### Launch arguments

| Argument | Description | Default |
| :--- | :--- | :--- |
| `namespace` | ROS namespace for the camera node. | `flir_boson` |
| `frame_id` | TF frame ID stamped on each `Image` header. | `boson_camera` |
| `dev` | Linux video device path (e.g. `/dev/video4`). | `/dev/video0` |
| `frame_rate` | V4L2 dequeue timer rate in Hz. Settles to physical limit if parameter exceeds hardware rate. Typical hardware rates are 9, 30, or 60. | `30.0` |
| `video_mode` | `YUV`: camera-side AGC (`mono8`/`bgr8`, low CPU). `RAW16`: raw 16-bit thermal counts (`mono16`, no host processing). `RAW16_AGC`: host-side percentile AGC (`mono8`, tunable). | `YUV` |
| `publish_color` | Publishes a `bgr8` colorized image instead of `mono8`. Use this if a color palette (e.g., Rainbow) is enabled via the FLIR GUI. Only supported in `YUV` mode; ignored in `RAW16`/`RAW16_AGC`. | `False` |
| `raw16_agc_low_pct` | Bottom-tail clip percentage for `RAW16_AGC` (e.g. `1.0` discards the darkest 1% of pixels before linear stretch). Valid range `[0, 50)`; invalid values revert to `1.0`. | `1.0` |
| `raw16_agc_high_pct` | Top-tail clip percentage for `RAW16_AGC` (e.g. `1.0` discards the brightest 1% of pixels before linear stretch). Valid range `[0, 50)`; invalid values revert to `1.0`. | `1.0` |
| `zoom_enable` | Digital 2× upscale (`320×256` → `640×512`) of the published image. Only honored on `sensor_type:=Boson_320` in `RAW16_AGC` mode; ignored elsewhere. | `False` |
| `sensor_type` | Physical sensor array size. `Boson_320` or `Boson_640`. The driver cross-checks this against the V4L2-negotiated width and refuses to start on a mismatch. | `Boson_640` |
| `camera_info_url` | Camera calibration file URL (`file://` or `package://`). Empty publishes uncalibrated `CameraInfo`. See [Calibration](#calibration) below. | `""` |

### Tuning the RAW16_AGC percentiles

Defaults of `1.0 / 1.0` (discard 1% from each tail) work well for most scenes. Adjust if:

- The scene contains a small but very hot object (lamp, exhaust, sun) that's compressing the rest of the image — *raise* `raw16_agc_high_pct` (try `2.0` or `5.0`).
- The image looks washed-out or low-contrast — *lower* both percentages toward `0.5`.
- A few dead/stuck pixels are dominating the range — keep low percentages, even `0.1` is usually enough to discard isolated outliers thanks to the histogram-based clipping.

Setting both to `0.0` is equivalent to a pure min/max stretch.

## Published topics

- **`/<namespace>/image_raw`** (`sensor_msgs/msg/Image`) — The primary video stream. Encoding depends on `video_mode`:
  - `YUV` mode: `mono8`, or `bgr8` if `publish_color:=True`
  - `RAW16` mode: `mono16` (raw 16-bit thermal counts)
  - `RAW16_AGC` mode: `mono8`

- **`/<namespace>/camera_info`** (`sensor_msgs/msg/CameraInfo`) — Calibration matrices and metadata, populated from `camera_info_url` if provided.

Both topics are advertised with `rclcpp::SensorDataQoS` (best-effort). `image_transport` additionally exposes `image_raw/compressed`, `image_raw/compressedDepth`, and `image_raw/theora` topics if the corresponding plugins are installed.

## Calibration

This driver publishes empty `CameraInfo` by default. To load a calibration, set `camera_info_url`:

```bash
ros2 launch flir_boson_usb flir_boson.launch.py \
    camera_info_url:=file:///home/user/my_boson.yaml
```

Note: Files inside `example_calibrations/` act purely as format reference models. To create target matrices for your physical lens setup, use the ROS 2 [`camera_calibration`](https://docs.ros.org/en/jazzy/p/camera_calibration/doc/tutorial_mono.html) tools.

`camera_info_url` accepts both `file://` URLs (absolute path on disk) and `package://` URLs (path relative to a ROS package share directory).

## Customizing the RAW16 pipeline

To implement custom radiometric filters, lookup tables, or neural network inference arrays directly on raw thermal data, hook into the `RAW16_AGC` execution path inside `BosonCamera::captureAndPublish()` where the uncompressed raw 16-bit `cv::Mat` is passed to the `agc()` processing loop. Alternatively, capture the unprocessed `RAW16` (`mono16`) topic stream externally within an independent node.

## Troubleshooting

- **`ERROR: Invalid Video Device`** — Verify user permissions in the `video` group (`groups` should list it). Ensure correct hardware links via `ls /dev/video*`.

- **`Hardware mismatch! Configured for Boson_640 but V4L2 negotiated width 320`** — Wrong `sensor_type` for the physical camera. Set it to match the actual sensor.

- **`Driver reports YUV bytesperline=X but width=Y`** — The driver is reporting strided YUV buffers, which this node does not currently handle. File an issue with the output of `v4l2-ctl -d <dev> --get-fmt-video` attached.

- **`ros2 topic echo` or `rqt_image_view` shows nothing.** Most likely a QoS mismatch: the driver publishes best-effort, but many tools default to reliable. Either tell the subscriber to use best-effort (`ros2 topic echo /flir_boson/image_raw --qos-reliability best_effort`) or run `rqt_image_view` which negotiates QoS automatically.

- **`RAW16_AGC` output looks washed out or saturated.** Tune `raw16_agc_low_pct` and `raw16_agc_high_pct` — see [Tuning the RAW16_AGC percentiles](#tuning-the-raw16_agc-percentiles).

## Credits & lineage

This package is a ROS 2 Humble port and substantial rewrite of two earlier open-source projects. Credit to the original authors:

1. **[FLIR Systems / BosonUSB](https://github.com/FLIR/BosonUSB)** — Foundational V4L2 C++ interactions and 16-bit to 8-bit AGC conversions.
2. **[AutonomouStuff / flir_boson_usb](https://github.com/astuff/flir_boson_usb)** — Original ROS 1 wrapper, nodelet architecture, and RAW16 image processing filters.

Both original codebases and this ROS 2 port are released under the MIT License.