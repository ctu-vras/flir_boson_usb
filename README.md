<!--
SPDX-License-Identifier: MIT
SPDX-FileCopyrightText: 2018 FLIR Systems, INC
SPDX-FileCopyrightText: 2018-2019 AutonomouStuff, LLC
SPDX-FileCopyrightText: Czech Technical University in Prague
-->

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

- **`YUV`** (default) — Camera-side DSP handles Automatic Gain Control (AGC) for a contrast-mapped 8-bit image (`mono8` or `bgr8` when `yuv_color:=True`). Lowest CPU footprint, ideal for single-board computers or multi camera setup. Only `image_raw` is published in this mode.

- **`RAW16`** — Direct 16-bit bolometer thermal counts (`mono16`) on `image_raw`, plus the display, heatmap and temperature outputs described in [Published topics](#published-topics). Some packages might not accept `mono16` directly so be mindful of that.

### Notes on frame rate and performance

- **Hardware rate vs. driver polling.** The `frame_rate` argument controls the software polling loop via a non-blocking poll() architecture. Setting it above hardware limits (e.g., polling at 60Hz on a 9Hz camera) safely drops back to the physical rate with no CPU penalty.

- **Automatic telemetry handling.** If binary telemetry metadata rows are active via the FLIR GUI, the driver reads the full buffer, handles telemetry isolation, and automatically crops the published image back to nominal array sizes (640×512 or 320×256) so standard `CameraInfo` calibrations continue to function perfectly.

### Launch arguments and node parameters

The provided launch files expose all driver parameters as launch args. So most of these arguments also right away describe the node parameters. The checked `Dynamic` column marks the parameters that can be changed while the node is running; see [Dynamically reconfigurable parameters](#dynamically-reconfigurable-parameters).

| Argument | Description | Default | Dynamic |
| :--- | :--- | :--- | :---: |
| `namespace` | ROS namespace for the camera node. | `flir_boson` | |
| `frame_id` | Frame used in `header.frame_id`. | `boson_camera` | |
| `camera_info_url` | Camera calibration file URL (`file://` or `package://`). Empty publishes uncalibrated `CameraInfo`. See [Calibration](#calibration) below. | `""` | |
| `dev` | The linux file descriptor location for the camera (e.g. `/dev/video4`). | `/dev/video0` | |
| `sensor_type` | Physical sensor array size. `Boson_320` or `Boson_640`. The driver cross-checks this against the V4L2-negotiated width and refuses to start on a mismatch. Also used as camera name if it does not have a serial number. | `Boson_640` | |
| `frame_rate` | Frame rate of the camera. Only 9.0/30.0/60.0 supported.| `30.0` | |
| `video_mode` | Camera image format. `YUV`: camera-side AGC (`mono8`/`bgr8`, low CPU). `RAW16`: raw 16-bit thermal counts (`mono16`, no camera processing).| `YUV` | |
| `zoom_enable` | Digital 2× upscale (`320×256` → `640×512`) of the published image. Only available on `Boson320` cameras. | `False` | |
| `yuv_color` | In `YUV` mode, publishes `image_raw` as `bgr8` instead of `mono8`. Use this if a color palette (e.g., Rainbow) is enabled via the FLIR GUI. | `False` | |
| `agc_mode` | Content of `image_visual`. `none`: the topic is not created at all. `fixed_range`: stretch the `agc_fixed_min_temp`/`agc_fixed_max_temp` interval. `auto_range`: stretch the percentiles of the current frame given by `agc_auto_low_pct`/`agc_auto_high_pct`. | `auto_range` | |
| `agc_auto_low_pct` | Bottom-tail clip percentage of the `auto_range` AGC (e.g. `1.0` discards the darkest 1% of pixels before linear stretch). Only honoured in `agc_mode:=auto_range`. Valid range `[0, 50)`; invalid values revert to `1.0`. | `1.0` | ✔ |
| `agc_auto_high_pct` | Top-tail clip percentage of the `auto_range` AGC (e.g. `1.0` discards the brightest 1% of pixels before linear stretch). Only honoured in `agc_mode:=auto_range`. Valid range `[0, 50)`; invalid values revert to `1.0`. | `1.0` | ✔ |
| `agc_fixed_min_temp` | Lower bound of the `fixed_range` stretch, in degrees Celsius. Has to be lower than `agc_fixed_max_temp`. | `20` | ✔ |
| `agc_fixed_max_temp` | Upper bound of the `fixed_range` stretch, in degrees Celsius. Has to be greater than `agc_fixed_min_temp`. | `50` | ✔ |
| `agc_norm` | Renormalise `image_visual` to the bounds observed on the current frame, on top of whichever `agc_mode` produced them. | `False` | ✔ |
| `agc_norm_margin` | How much wider the observed bounds are taken when `agc_norm` is on. Non-zero values mean the given number of lowest and highest grey levels of the active stretch are not used in the linear stretch. | `20.0` | ✔ |
| `heatmap_mode` | Content of `image_heatmap`. `none`: the topic is not created at all. `visual`: colourise `image_visual`. Requires an `agc_mode` other than `none`. | `none` | |
| `overlay_mode` | What is stamped on `image_heatmap`. `none`: nothing. `min_max_ptr`: the minimum, maximum and probe reading text and the probe marker. `min_max`: the minimum and maximum text only. `ptr`: the probe text and marker only. Printed in the unit of `temp_mode`. See [The heatmap overlay](#the-heatmap-overlay). | `min_max_ptr` | ✔ |
| `temp_ptr_x` | X coord of the temperature probe point, i.e. the point whose reading is published on `ptr_temp` and stamped as `Ptr`. It has to lie inside the published image. | `319` | ✔ |
| `temp_ptr_y` | Y coord of the temperature probe point, i.e. the point whose reading is published on `ptr_temp` and stamped as `Ptr`. It has to lie inside the published image. | `255` | ✔ |
| `colormap` | Palette `image_heatmap` is painted with. `autumn`, `bone`, `jet`, `winter`, `rainbow`, `ocean`, `summer`, `spring`, `cool`, `hsv`, `pink` and `hot` are always available; `parula` needs OpenCV 4.0 or newer, `magma`, `inferno`, `plasma`, `viridis` and `cividis` need 4.4, and `twilight`, `twilight_shifted`, `turbo` and `deepgreen` need 4.5. Case is ignored. A palette the built OpenCV does not have is not offered, and an unknown value keeps the palette used so far (at startup it stops the node). | `jet` | ✔ |
| `temp_mode` | Unit of `image_temp`. `none`: the topic is not created at all. `c`, `k`, `f`: absolute degrees Celsius, Kelvin and Fahrenheit as `32FC1`. `centi_c`, `centi_k`, `centi_f`: the same three units in hundredths, as `16SC1`, `16UC1` and `16SC1`. Only offered for radiometric cameras; see [Temperature units](#temperature-units). | `none` | |
| `radiometric` | Whether the camera can map the raw counts to absolute temperatures, which is what enables `image_temp`, `min_temp`, `max_temp` and `ptr_temp`. `auto`: match the device identification against `radiometric_patterns`. `true`/`false`: override the detection. | `auto` | |
| `radiometric_patterns` | Regular expressions tested against the device identification when `radiometric` is `auto`. An invalid pattern is reported and skipped. | `["[Rr]adiometric"]` | |

Invalid combinations are rejected rather than silently corrected: an unknown `agc_mode`, `heatmap_mode`, `overlay_mode`,
`temp_mode`, `colormap` or `radiometric` value stops the node at startup, and a `temp_ptr_x`/`temp_ptr_y`/temperature-limit
change that would produce an invalid combination is refused by the parameter callback, which keeps the previous values.
A `colormap` or `overlay_mode` change to a name that is not offered is reported in the node log, and the heatmap
keeps painting with the palette and the stamped content it used so far.

### Tuning the AGC

The stretch that turns 16-bit counts into the `image_visual` grey levels is controlled by `agc_mode`. The parameters of each of the two stretches carry its name: `agc_auto_*` belong to `auto_range`, `agc_fixed_*` to `fixed_range`, and `agc_norm`/`agc_norm_margin` apply to whichever of them produced the bounds.

- `auto_range` (default) stretches the percentiles of the current frame. Defaults of `1.0 / 1.0` (discard 1% from each tail) work well for most scenes. Adjust if:
  - The scene contains a small but very hot object (lamp, exhaust, sun) that's compressing the rest of the image — *raise* `agc_auto_high_pct` (try `2.0` or `5.0`).
  - The image looks washed-out or low-contrast — *lower* both percentages toward `0.5`.
  - A few dead/stuck pixels are dominating the range — keep low percentages, even `0.1` is usually enough to discard isolated outliers thanks to the histogram-based clipping.

  Setting both to `0.0` is equivalent to a pure min/max stretch.
- `fixed_range` stretches the `agc_fixed_min_temp`/`agc_fixed_max_temp` interval instead, which keeps the grey levels of a given temperature stable between frames.
- `none` does not produce `image_visual` at all (and therefore no `image_heatmap` either).

`agc_norm` is a second stage on top of whichever `agc_mode` produced the bounds: it re-stretches the result to the bounds observed on the current frame, widened by `agc_norm_margin`.

### Temperature units

`temp_mode` decides both the unit and the image encoding of `image_temp`. A radiometric Boson reports the scene
temperature as hundredths of Kelvin, so every unit is only a rescaling of the raw counts:

| `temp_mode` | `image_temp` encoding | Range the encoding can carry |
| :--- | :--- | :--- |
| `c` | `32FC1` | degrees Celsius, no practical limit |
| `k` | `32FC1` | Kelvin, no practical limit |
| `f` | `32FC1` | degrees Fahrenheit, no practical limit |
| `centi_c` | `16SC1` | -327.68 to 327.67 deg C |
| `centi_k` | `16UC1` | -273.15 to 382.20 deg C |
| `centi_f` | `16SC1` | -199.82 to 164.26 deg C |

The 16-bit units carry half the bytes of the 32-bit ones, which matters on a 60 Hz `640×512` stream, at the price of
a resolution of a hundredth of a degree and of the range in the last column. `centi_k` is a lossless copy of the raw
counts (the sensor already reports hundredths of Kelvin), so it is the cheapest way to move the full-range data to
another node. A pixel whose temperature does not fit into the encoding is saturated to the corner of its range
rather than wrapped around, which is visible as a patch of the extreme value on very hot or very cold scenes; pick
one of the `32FC1` units when that can happen.

The unit is a startup parameter, like the other presets: it decides whether `image_temp` exists at all and what
encoding it carries, and both are fixed when the publishers are created.

### The heatmap overlay

`overlay_mode` chooses what is stamped on `image_heatmap`. `Max` and `Min` are the bounds of the stretch the visible
pixels were made from, `Ptr` is the reading at `temp_ptr_x`/`temp_ptr_y`, and the probe marker sits at the same point:

| `overlay_mode` | stamped |
| :--- | :--- |
| `none` | nothing |
| `min_max_ptr` | the `Max`, `Min` and `Ptr` text and the probe marker |
| `min_max` | the `Max` and `Min` text only, i.e. no probe marker |
| `ptr` | the `Ptr` text and the probe marker |

The bounds describe the picture and are stamped whatever the probe point is; the probe text and its marker need a
probe inside the published image and are left out when `temp_ptr_x`/`temp_ptr_y` is outside of it.

Every reading that is stamped is printed in the unit of `temp_mode`, with the unit written next to the number:

| camera and `temp_mode` | stamped text |
| :--- | :--- |
| radiometric, `c` / `k` / `f` | `Max: 26.85 deg C` (also `deg K`, `deg F`), with two decimals |
| radiometric, `centi_c` / `centi_k` / `centi_f` | `Max: 2685 cdeg C` — hundredths of the degree, the integers the 16-bit image carries |
| non-radiometric camera, or `temp_mode:=none` | `Max: 30000 counts` — the raw 16-bit thermal counts of the frame |

A camera without radiometry cannot be turned into absolute temperatures, so the overlay prints the counts it really
has instead of a temperature that would be made up. In the hundredths units the printed number is the value the
corresponding pixel of `image_temp` holds, i.e. saturated to the range of the encoding rather than the temperature
that does not fit into it.

### Dynamically reconfigurable parameters

`temp_ptr_x`, `temp_ptr_y`, `agc_fixed_max_temp`, `agc_fixed_min_temp`, `agc_norm`, `agc_norm_margin`,
`agc_auto_low_pct`, `agc_auto_high_pct`, `colormap` and `overlay_mode` are dynamic parameters that can be tuned
while the node is running:

```bash
# ROS 2
ros2 param set /flir_boson/flir_boson_usb_node temp_ptr_x 320
ros2 param set /flir_boson/flir_boson_usb_node colormap turbo
ros2 param set /flir_boson/flir_boson_usb_node overlay_mode none

# ROS 1 (dynamic_reconfigure)
rosrun rqt_reconfigure dynparam set /flir_boson/flir_boson_usb_node temp_ptr_x 320
```

## Published topics

Every image of a frame shares the `header.stamp` and `header.frame_id` of `image_raw`, so the streams can be matched pixel-for-pixel.

A topic either carries data or it does not exist at all — the driver does not advertise a publisher for an output that
the current parameter preset disables. The condition under which each of the derived topics exists is given with it below.

- **`/<namespace>/image_raw`** (`sensor_msgs/msg/Image`) — The primary video stream, always published. Encoding depends on `video_mode`:
  - `YUV` mode: `mono8`, or `bgr8` if `yuv_color:=True`
  - `RAW16` mode: `mono16` (raw 16-bit thermal counts)

- **`/<namespace>/camera_info`** (`sensor_msgs/msg/CameraInfo`) — Calibration matrices and metadata, published together with `image_raw` and populated from `camera_info_url` if provided.

- **`/<namespace>/image_visual`** (`sensor_msgs/msg/Image`, `mono8`) — The contrast-mapped display image. Exists when `video_mode` is `RAW16` and `agc_mode` is not `none`.

- **`/<namespace>/image_heatmap`** (`sensor_msgs/msg/Image`, `bgr8`) — `image_visual` colourised with the `colormap` palette, with the `overlay_mode` readings and the probe marker stamped on it. Exists when `image_visual` exists and `heatmap_mode` is not `none`. See [The heatmap overlay](#the-heatmap-overlay).

- **`/<namespace>/image_temp`** (`sensor_msgs/msg/Image`) — The absolute temperature of every pixel, in the unit and the encoding given by `temp_mode` (`32FC1` for `c`/`k`/`f`, `16SC1` or `16UC1` for the `centi_` units). Exists when the camera is radiometric (`radiometric`) and `temp_mode` is not `none`. See [Temperature units](#temperature-units).

- **`/<namespace>/min_temp`**, **`/<namespace>/max_temp`**, **`/<namespace>/ptr_temp`** (`flir_boson_usb/msg/Temperature`, degrees Celsius) — The bounds of the stretch the visible pixels were made from, and the temperature at `temp_ptr_x`/`temp_ptr_y`. They exist exactly when `image_temp` exists.

All image topics are advertised with `rclcpp::SensorDataQoS` (best-effort). `image_transport` additionally exposes `image_raw/compressed`, `image_raw/compressedDepth`, and `image_raw/theora` topics if the corresponding plugins are installed.

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

The URL can also contain the substitution `${NAME}` which expands to the camera name. If this driver succeeds detecting a serial number, the name will be set to that. If not, it sets the camera name to either `Boson_320` or `Boson_640`.

## Customizing the RAW16 pipeline

To implement custom radiometric filters, lookup tables, or neural network inference arrays directly on raw thermal data, process the `RAW16` (`mono16`) topic stream externally within an independent node.

## Troubleshooting

- **`ERROR: Invalid Video Device`** — Verify user permissions in the `video` group (`groups` should list it). Ensure correct hardware links via `ls /dev/video*`.

- **`Hardware mismatch! Configured for Boson_640 but V4L2 negotiated width 320`** — Wrong `sensor_type` for the physical camera. Set it to match the actual sensor.

- **`Driver reports YUV bytesperline=X but width=Y`** — The driver is reporting strided YUV buffers, which this node does not currently handle. File an issue with the output of `v4l2-ctl -d <dev> --get-fmt-video` attached.

- **`ros2 topic echo` or `rqt_image_view` shows nothing.** Most likely a QoS mismatch: the driver publishes best-effort, but many tools default to reliable. Either tell the subscriber to use best-effort (`ros2 topic echo /flir_boson/image_raw --qos-reliability best_effort`) or run `rqt_image_view` which negotiates QoS automatically.

- **A topic you expect is missing from `ros2 topic list`.** The parameter that enables it is off — `image_visual` needs `video_mode:=RAW16` and an `agc_mode` other than `none`, `image_heatmap` additionally needs `heatmap_mode:=visual`, and `image_temp` with the three temperature topics needs a radiometric camera and a `temp_mode` other than `none`. This is intentional; see [Published topics](#published-topics).

- **AGC output looks washed out or saturated.** Tune `agc_auto_low_pct` and `agc_auto_high_pct`, or switch to `agc_mode:=fixed_range` — see [Tuning the AGC](#tuning-the-agc).

## Credits & lineage

This package is a substantial rewrite of two earlier open-source projects which is
built for both ROS generations (the ROS 2 rewrite was contributed by the port
maintainers, the ROS 1 nodelet was restored on top of it). Credit to the original authors:

1. **[FLIR Systems / BosonUSB](https://github.com/FLIR/BosonUSB)** — Foundational V4L2 C++ interactions and 16-bit to 8-bit AGC conversions.
2. **[AutonomouStuff / flir_boson_usb](https://github.com/astuff/flir_boson_usb)** — Original ROS 1 wrapper, nodelet architecture, and RAW16 image processing filters.
2. **[GITAI / flir_boson_usb](https://github.com/GITAI/flir_boson_usb)** — Extension of the ROS 1 driver with more 8-bit outputs.

Both the original codebases and this port are released under the MIT License.