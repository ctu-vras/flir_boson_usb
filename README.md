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

The driver works best with Boson SDK. However, this library is not freely redistributable.
This project can automatically download and compile it, but it needs your explicit consent.
Set CMake variable `USE_BOSON_SDK` to `ON` to enable this functionality.

## Building

### ROS 2

```bash
cd ~/ws
rosdep install --from-paths src --ignore-src -r
colcon build --packages-select flir_boson_usb --symlink-install --cmake-args -DUSE_BOSON_SDK=ON
source install/setup.bash
```

### ROS 1

```bash
cd ~/ws
rosdep install --from-paths src --ignore-src -r
catkin config  --cmake-args -DUSE_BOSON_SDK=ON
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

- **Publisher queue depth.** Every publisher advertises `queue_size` frames (the `keep_last` history depth, 1 by default). With the default, a subscriber that does not keep up simply misses the stale frame instead of the driver buffering a backlog and growing the latency; raise it when a slow consumer must not miss frames.

- **Automatic telemetry handling.** If binary telemetry metadata rows are active via the FLIR GUI, the driver reads the full buffer, handles telemetry isolation, and automatically crops the published image back to nominal array sizes (640×512 or 320×256) so standard `CameraInfo` calibrations continue to function perfectly.

### Launch arguments and node parameters

The provided launch files expose all driver parameters as launch args. So most of these arguments also right away describe the node parameters. The checked `Dynamic` column marks the parameters that can be changed while the node is running; see [Dynamically reconfigurable parameters](#dynamically-reconfigurable-parameters).

| Argument | Description | Default | Dynamic |
| :--- | :--- | :--- | :---: |
| `namespace` | ROS namespace for the camera node. | `flir_boson` | |
| `frame_id` | Frame used in `header.frame_id`. | `boson_camera` | |
| `camera_info_url` | Camera calibration file URL (`file://` or `package://`). Empty publishes uncalibrated `CameraInfo`. See [Calibration](#calibration) below. | `""` | |
| `dev` | The linux file descriptor location for the camera (e.g. `/dev/video4`). | `/dev/video0` | |
| `control_dev` | The linux file descriptor location for the camera's control channel (e.g. `/dev/ttyACM0`). This gives access to camera control and detailed status. | `/dev/video0` | |
| `sensor_type` | Physical sensor array size. `Boson_320` or `Boson_640`. The driver cross-checks this against the V4L2-negotiated width and refuses to start on a mismatch. Also used as camera name if it does not have a serial number. | `Boson_640` | |
| `frame_rate` | Frame rate of the camera. Only 9.0/30.0/60.0 supported.| `30.0` | |
| `queue_size` | Size of the publisher queues (the `keep_last` history depth) of every topic the node publishes. See [Notes on frame rate and performance](#notes-on-frame-rate-and-performance). | `1` | |
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
| `temp_ptr_x` | X coord of the temperature probe point, i.e. the point whose reading is published on `ptr_temp` and stamped as `Ptr`. It has to lie inside the published image. | `319` (`159` for a 320 sensor) | ✔ |
| `temp_ptr_y` | Y coord of the temperature probe point, i.e. the point whose reading is published on `ptr_temp` and stamped as `Ptr`. It has to lie inside the published image. | `255` (`127` for a 320 sensor) | ✔ |
| `colormap` | Palette `image_heatmap` is painted with. `autumn`, `bone`, `jet`, `winter`, `rainbow`, `ocean`, `summer`, `spring`, `cool`, `hsv`, `pink` and `hot` are always available; `parula` needs OpenCV 4.0 or newer, `magma`, `inferno`, `plasma`, `viridis` and `cividis` need 4.4, and `twilight`, `twilight_shifted`, `turbo` and `deepgreen` need 4.5. Case is ignored. A palette the built OpenCV does not have is not offered, and an unknown value keeps the palette used so far (at startup it stops the node). | `jet` | ✔ |
| `gain` | Gain mode the camera runs with, `high` or `low`. The driver does not switch the gain (do that in the FLIR GUI); the parameter only says which gain the counts were made with, because the two gains map the same temperature to different counts. | `high` | |
| `temp_mode` | Unit of `image_temp`. `none`: the topic is not created at all. `counts`: the raw counts, compensated for the gain they were made with, as `32FC1`. `c`, `k`, `f`: absolute degrees Celsius, Kelvin and Fahrenheit, also `32FC1`. Offered on every camera; on a non-radiometric one the absolute units are only a rescaled guess; see [Temperature units](#temperature-units). The launch files pass `none`, so that a stream nobody subscribes to is not made. | `counts` | |
| `radiometric` | Whether the camera can map the raw counts to absolute temperatures. `auto`: query the device (fallback to `false`). `true`/`false`: override the detection. The driver reports the outcome in the log; the temperature outputs follow `temp_mode`, not this parameter. | `auto` | |
| `ffc_mode` | Flat field correction mode the camera is switched to at startup, through the control channel. `-1`: leave the camera alone. `0`: manual, `1`: auto, `2`: external, `3`: shutter test. | `-1` | |

Invalid combinations are rejected rather than silently corrected: an unknown `agc_mode`, `heatmap_mode`, `overlay_mode`,
`temp_mode`, `gain`, `colormap` or `radiometric` value stops the node at startup, and a `temp_ptr_x`/`temp_ptr_y`/temperature-limit
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

`temp_mode` decides the unit of `image_temp`; all its units are carried in `32FC1`. Every unit is a single affine
rescaling of the raw counts, and what the counts stand for depends on the `gain` the camera runs with: on high gain
the counts already are hundredths of Kelvin, on low gain the sensor amplifies twice as much and therefore counts
twice as fast. With `m` the gain multiplier (`1` on high gain, `2` on low gain) and `c` the count value of a pixel:

| `temp_mode` | the pixel holds |
| :--- | :--- |
| `counts` | `c * m` -- the counts brought to the high-gain scale, so a given scene gives the same value whatever gain produced it |
| `c` | `c * m / 100 - 273.15` deg C |
| `k` | `c * m / 100` K |
| `f` | `c * m / 100 * 1.8 - 459.67` deg F |

`counts` is the default and is meaningful on any camera: a non-radiometric Boson produces counts too, they only do
not stand for a calibrated temperature. The absolute units are offered on every camera as well, but on a camera
whose counts are not calibrated they are a rescaled guess rather than a measurement. The map is only physically
correct for a camera running in its temperature-linear mode, which is what radiometric Bosons use.

The `gain` parameter has to say what the camera really uses -- the driver never switches the gain itself, it only
rescales the counts by the multiplier of the given gain. A camera that switches its gain by itself therefore
publishes wrong temperatures in the frames between the switch and the parameter change; reading the gain from the
frame telemetry is planned for a later stage.

The unit is a startup parameter, like the other presets: it decides whether `image_temp` exists at all, and both it
and the encoding are fixed when the publishers are created.

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

| `temp_mode` | stamped text |
| :--- | :--- |
| `c` / `k` / `f` | `Max: 26.85 deg C` (also `deg K`, `deg F`), with two decimals |
| `counts` | `Max: 30000 counts` — the gain-compensated counts of the frame, printed as integers because the counts unit has no fractional part either |

Every stamped number is the value the corresponding pixel of `image_temp` holds: the overlay and the temperature
image are computed from the same affine map of [`temp_mode`](#temperature-units) and `gain`, so they cannot
disagree.

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

- **`/<namespace>/image_temp`** (`sensor_msgs/msg/Image`, `32FC1`) — Every pixel in the unit given by `temp_mode`, made by the radiometric filter the driver runs inside itself. Exists when `video_mode` is `RAW16` and `temp_mode` is not `none`. See [Temperature units](#temperature-units) and [Filter chains](#filter-chains).

- **`/<namespace>/min_temp`**, **`/<namespace>/max_temp`**, **`/<namespace>/ptr_temp`** (`flir_boson_usb/msg/Temperature`, degrees Celsius) — The bounds of the stretch the visible pixels were made from, and the temperature at `temp_ptr_x`/`temp_ptr_y`. They exist exactly when `image_temp` exists.

All image topics are advertised with `rclcpp::SystemDefaultsQoS` (reliable) and a queue of `queue_size` frames. Be aware of possible problems if you subscribe via reliable subscribers and the link does not have enough bandwidth.

## Filter chains

The RAW16 post-processing can also be expressed as a chain of `sensor_filters` plugins, so that the stages of
the pipeline become individually selectable and replaceable instead of being hard-wired into the node. This
section records how the driver's processing maps onto filters, which is the part that needs thinking about: a
filter takes one message and returns one message, and the driver's processing does not fit that shape
everywhere.

The filters are pluginlib plugins of the `filters` category, loaded by the `flir_boson_usb/<Name>_<MessageType>`
name shown in the table, and they are registered only in the ROS 2 build.
[doc/filter_chains.md](doc/filter_chains.md) holds the full design, including what is still missing before the
chain path can replace the code built into the node.

Everything in this section that concerns the filter itself -- its name, parameters, input and output encodings --
is covered by `test_radiometric_filter`, and the way the driver sets the chain up is covered by
`test_radiometric_filter_chain`. A chain that somebody else set up was checked by hand rather than by a test:
`examples/filters_example.launch.xml` was run against `sensor_filters` 2.0.0 on ROS 2 Jazzy with a synthetic
`image_raw`, and the driver half of that launch has never been run because no camera was attached.

**Status.** `image_temp` is made by `RadiometricFilter`, which the driver loads into a filter chain of its own
when it creates the temperature publishers. `image_visual`, `image_heatmap` and the three temperature readings are
still made by the processing stage in `pipeline.cpp`; the stages that should replace them are described below but
not implemented.

### The stages as filters

| Filter | In | Out | Parameters |
| :--- | :--- | :--- | :--- |
| `flir_boson_usb/RadiometricFilter` | `mono16` counts | the `32FC1` encoding of `output_unit` | `gain`, `output_unit` |
| `flir_boson_usb/AgcFilter_Image` (planned) | `mono16` counts | `mono8` | `mode`, `auto_low_pct`, `auto_high_pct`, `fixed_min_temp`, `fixed_max_temp`, `renormalize`, `norm_margin` |
| `flir_boson_usb/ColormapFilter_Image` (planned) | `mono8` | `bgr8` | `colormap` |
| `flir_boson_usb/HeatmapFilter_Image` (planned) | `mono16` counts | `bgr8` | `colormap`, `overlay_mode`, `probe_x`, `probe_y`, `temp_mode`, `gain` |

### The chain inside the driver

The driver does not wait for someone else to set the chain up: when `temp_mode` is not `none`, it creates a chain
under the parameter prefix `image_temp`, puts a single `RadiometricFilter` named `radiometry` into it and pushes
every `image_raw` through it to `image_temp`. The filter therefore has parameters of its own on the driver node,
which the driver fills in from its own parameters when it creates the publishers:

| Parameter of the driver node | Follows |
| :--- | :--- |
| `image_temp.filter1.params.gain` | `gain` |
| `image_temp.filter1.params.output_unit` | `temp_mode` |

They are set once at startup and are not meant to be set separately; changing `temp_mode` or `gain` means
restarting the node, like the other presets. The name and type of the filter (`image_temp.filter1.name` and
`.type`) are declared by the node as read-only parameters and are not worth touching -- replacing the filter there
means rebuilding the node's list of publishers, which is what `createFilterChain()` and `addFiltersToNodeOptions()`
do together.

### A chain of someone else

The very same filter can be run in a chain that is not part of the driver, to convert an `image_raw` that comes
from somewhere else or to chain further stages onto it. A filter reads its parameters from its own namespace
inside the chain, so on a chain node named `/boson_temperature` (the executable is
`sensor_filters/image_filter_chain`, subscribing to `input` and publishing `output`) the conversion filter is
configured by

```yaml
/boson_temperature:
  ros__parameters:
    image_filter_chain:
      filter1:
        name: radiometric
        type: flir_boson_usb/RadiometricFilter
        params:
          gain: high            # high | low
          output_unit: k        # counts | c | k | f
```

Run the driver with `temp_mode:=none` alongside such a chain, or the topic would carry the driver's conversion and
the chain's; `examples/filters_example.launch.xml` shows the whole thing.

The filter assumes it is fed counts of a camera in its temperature-linear mode: whoever writes the chain already
knows what is in front of it. A non-radiometric Boson can use the filter too, with `output_unit` left at `counts`,
to get a gain-compensated count image.

The unit can also be changed while the chain runs:

```text
ros2 param set /boson_temperature image_filter_chain.filter1.params.output_unit f
```

The filter refuses a value it does not know, in which case the call fails and the filter keeps converting with
the unit it had.

The layout above was checked against `sensor_filters` 2.0.0, including the parameters the filter has. Two
things about it are easy to get wrong: a parameter file has to key the chain node by its **fully qualified** name,
so a namespaced chain node needs the namespace in the key or it starts with no filters and no warning; and the
`input` and `output` remappings have to be absolute when the chain node runs in a namespace. Both are handled in
`examples/filters_example.launch.xml`.

The unit of the output belongs to the filter rather than to a separate unit-conversion filter because the
rescaling from counts to a unit is a single arithmetic operation; splitting it into counts to degrees and
degrees to the unit would rescale the whole image twice for no gain. `output_unit` accepts the same names as
[temp_mode](#temperature-units), minus `none` -- a temperature image always has a unit. `gain` is a parameter of
the filter rather than of the counts, because the counts themselves say nothing about the gain that made them.

### Where the rest of the driver's processing went

The stages above are the only ones that fit the one-in-one-out shape. Everything else the driver derives from
a frame went somewhere else, and this is the part to review if you are considering extending the chain:

| Driver mechanism | Where it is in the chain model | Why |
| :--- | :--- | :--- |
| `radiometric` auto-detection over the control channel | stays in the driver, which is the only thing that talks to the camera | The filters do not ask. A chain of a non-radiometric camera simply does not contain the filters that need absolute temperatures, which is how the "the topic either carries data or does not exist" rule is expressed on the chain path. Since the temperature image is offered for non-radiometric counts too, the driver only reports the detected state in the log today. |
| `gain` of the camera | the `gain` parameter of the filter, pushed in by the driver | The filter is the only consumer of the gain, and the driver is the only thing that could learn the real gain from the camera or from the frame telemetry. Once telemetry carries the gain, the driver will set the filter's parameter from it instead of from the static parameter. |
| `min_temp`, `max_temp`, `ptr_temp` | a node subscribing to the temperature image (planned), not a filter | They are scalars, and a filter has one output, which is already the temperature image. They still come from the processing stage today. |
| `temp_ptr_x`, `temp_ptr_y` | the heatmap filter **and** the readings node | The overlay and the readings need the same probe position and a filter cannot be handed anything but its input image, so the parameter is necessarily duplicated. Keep the two in sync. |
| `camera_info` | needed by none of these filters | The counts-to-temperature map is affine and constant, so there is nothing to read from the calibration. |
| `zoom_enable` | out of the chain | It changes the image size, which silently invalidates `CameraInfo`; a resize filter would have to come with a calibration-scaling filter. |
| `ros2 param set` of the AGC, heatmap, overlay and probe parameters | the filter's own parameter, applied by its `reconfigureCB()` | `filters::FilterChain` -- which is what a `sensor_filters` chain, and the driver's own chain, run on -- forwards every parameter whose name starts with a filter's prefix to that filter, as an *on-set* callback, i.e. before the new value reaches the parameter storage. A filter therefore has to declare the parameter modifiable and take the new value from the callback argument; `getParam()` would still return the old one. The radiometric filter does exactly that, and changing its unit on a running chain is covered by `test_radiometric_filter_chain`. The planned filters have to do the same. |

The planned `HeatmapFilter_Image` is deliberately a single filter covering both the palette and the overlay
text, rather than a palette filter followed by an overlay filter. The overlay prints the minimum, maximum and
probe temperatures of the frame it annotates, and those have to be computed from the raw counts -- by the time
a palette filter has run, the counts are gone. So the filter that draws the overlay has to sit on the counts
and produce the colour image itself.

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

To implement custom radiometric filters, lookup tables, or neural network inference arrays directly on raw thermal data, process the `RAW16` (`mono16`) topic stream externally within an independent node. If what you want to chain together are stages of this kind, [Filter chains](#filter-chains) covers doing it as `filters` plugins instead, and the conversion filter of this package is one example of such a plugin.

## Troubleshooting

- **`ERROR: Invalid Video Device`** — Verify user permissions in the `video` group (`groups` should list it). Ensure correct hardware links via `ls /dev/video*`.

- **`Hardware mismatch! Configured for Boson_640 but V4L2 negotiated width 320`** — Wrong `sensor_type` for the physical camera. Set it to match the actual sensor.

- **`Driver reports YUV bytesperline=X but width=Y`** — The driver is reporting strided YUV buffers, which this node does not currently handle. File an issue with the output of `v4l2-ctl -d <dev> --get-fmt-video` attached.

- **A topic you expect is missing from `ros2 topic list`.** The parameter that enables it is off — `image_visual` needs `video_mode:=RAW16` and an `agc_mode` other than `none`, `image_heatmap` additionally needs `heatmap_mode:=visual`, and `image_temp` with the three temperature topics needs `video_mode:=RAW16` and a `temp_mode` other than `none`. This is intentional; see [Published topics](#published-topics).

- **AGC output looks washed out or saturated.** Tune `agc_auto_low_pct` and `agc_auto_high_pct`, or switch to `agc_mode:=fixed_range` — see [Tuning the AGC](#tuning-the-agc).

## Credits & lineage

This package is a substantial rewrite of two earlier open-source projects which is
built for both ROS generations (the ROS 2 rewrite was contributed by the port
maintainers, the ROS 1 nodelet was restored on top of it). Credit to the original authors:

1. **[FLIR Systems / BosonUSB](https://github.com/FLIR/BosonUSB)** — Foundational V4L2 C++ interactions and 16-bit to 8-bit AGC conversions.
2. **[AutonomouStuff / flir_boson_usb](https://github.com/astuff/flir_boson_usb)** — Original ROS 1 wrapper, nodelet architecture, and RAW16 image processing filters.
2. **[GITAI / flir_boson_usb](https://github.com/GITAI/flir_boson_usb)** — Extension of the ROS 1 driver with more 8-bit outputs.

Both the original codebases and this port are released under the MIT License.