<!--
SPDX-License-Identifier: MIT
SPDX-FileCopyrightText: Czech Technical University in Prague
-->

# Producing the RAW16 outputs with sensor filter chains

This document describes the design behind the `filters::FilterBase` plugins of this package and the state of the
migration. It is written for someone who has to decide whether the chain path can replace the code built into the
node, and for someone who has to add the next filter.

The short version: the radiometric conversion is a filter today, and the node no longer does it itself -- it loads
the filter into a chain of its own and publishes what the chain returns on `image_temp`. The other outputs are
still produced by the processing stage; they are described here but not implemented as filters yet.
The stage is being taken aprat filter by filter, and what is left of it once all of them exist is an open question.

## 1. What the node does

In `RAW16` mode the node publishes:

| topic | type | created when |
| :--- | :--- | :--- |
| `image_raw` (+ `camera_info`) | `sensor_msgs/Image` `mono16` counts | always |
| `image_visual` | `mono8` | `agc_mode != none` |
| `image_heatmap` | `bgr8` | `agc_mode != none` and `heatmap_mode == visual` |
| `image_temp` | `32FC1` | `temp_mode != none` |
| `max_temp`, `min_temp`, `ptr_temp` | `flir_boson_usb/Temperature` | as `image_temp` |

The pixel work that is left in `include/flir_boson_usb/pipeline.hpp` and `src/pipeline.cpp` -- a library with no
ROS dependency -- is `buildHistogram()`, `percentileBounds()`, `renormalize()`, `stretchTo8Bit()`, `drawOverlay()`,
`formatOverlayValue()`, the name and enum tables, and the stateful `Pipeline` class that turns one frame of counts
into `image_visual`, `image_heatmap` and the three readings in a single pass. The temperature image is not among
its outputs any more: it is produced by `src/filters/radiometric_filter.cpp`, which shares the name tables and the
affine map with the stage instead of duplicating them.

Two properties of that code decide the shape of the design:

- **The radiometric conversion is a constant affine map**, `tempAffine(gain_mode, temp_mode)`: high-gain counts are
  hundredths of Kelvin, low-gain counts tick twice as fast, and every unit is a scale and an offset on top of that.
  There are no planar-target tables and no emissivity model, so a filter needs no second input and no access to the
  camera to do it. This is why the radiometric filter was the first one written and why it is small.
- **Which gain the counts were made with is not written in the pixels**, so the filter has to be told; see the
  `gain` parameter in §3.
- **The readings are computed from the frame and consumed together with it.** `Pipeline::process()` returns the
  minimum, maximum and probe counts alongside the images, and `drawOverlay()` draws exactly those numbers. A
  filter cannot return them; see §4.

## 2. The filter contract this package targets

A filter is a `filters::FilterBase<sensor_msgs::msg::Image>` with `bool configure()` and
`bool update(const sensor_msgs::msg::Image& in, sensor_msgs::msg::Image& out)`. The message type comes from that
base class rather than from a template parameter of the filter: a filter of this package converts images, a chain
hands it nothing else, and there is nothing to parameterise. A filter is therefore an ordinary pluginlib class --
declared in `radiometric_filter.xml`, exported with `pluginlib_export_plugin_description_file(filters ...)`,
implemented in a source file next to the export -- and it is loaded by the name
`flir_boson_usb/<Name>_<MessageType>`. That is the whole of what a filter is, and everything in §3 and §4 follows
from it. `test/test_radiometric_filter.cpp` exercises this contract, and
`test/test_radiometric_filter_chain.cpp` configures a `filters::FilterChain` the way the driver does and converts a
frame through the filter the chain loaded from the installed plugin description.

The filters of this package derive from `cras::FilterBase<sensor_msgs::msg::Image>` of `cras_cpp_common` rather
than from `filters::FilterBase` directly. It is still the base class a chain instantiates -- `cras::FilterBase`
derives from it and adds the two things a filter of this size wants: `this->getParam(name, default)`, which
declares the parameter if it is not declared yet and returns the value instead of writing it through a
reference, and `this->declareParameter(name, default)`. Since the derived `configure()` hides the inherited
one, it has to chain up to `cras::FilterBase<...>::configure()` first -- that call is what creates the parameter
helper every `getParam()` goes through.

The pixels are converted with `cv_bridge`: `cv_bridge::toCvShare()` for the incoming message (it only wraps the
buffer, which is what we want, because the conversion itself merely reads it) and
`cv_bridge::CvImage(header, encoding, image).toImageMsg()` for the outgoing one. The one thing `cv_bridge` does
not check is that the message is as long as its own `step` times its `height`, so a filter that would read past
the buffer has to check that itself before calling it.

What a chain looks like is the business of `sensor_filters`, and the filter library deliberately does not depend on
it, so no test here starts a chain node. `examples/filters_example.launch.xml` nevertheless shows how to set up
such chain.

- the filter list is configured as `image_filter_chain.filter<N>.{name,type,params}` with `N` starting at 1, and a
  chain node inside a namespace reads that list from a parameter file only under its **fully qualified** node name
  -- a key without the namespace leaves the chain running with no filters and without complaining;
- the chain node is `sensor_filters/image_filter_chain`, it subscribes to `input` and publishes `output`, and both
  have to be remapped to absolute names when the node is started in a namespace, because a remap target without a
  leading slash is resolved inside the node's own namespace;
- the options of the chain itself (`image_transport`, `input_queue_size`, `output_queue_size`, `publication_type`,
  `subscription_type`, `is_lazy`) are declared on the node's **root** parameter interface, so two chains of this
  kind cannot share one node.

## 3. Where each output goes

| node output | in the chain design | status |
| :--- | :--- | :--- |
| `image_temp` | `flir_boson_usb/RadiometricFilter`, run by the driver itself | **implemented, in use** |
| `image_visual` | an AGC filter wrapping `buildHistogram`/`percentileBounds`/`renormalize`/`stretchTo8Bit`, parameters `mode`, `auto_low_pct`, `auto_high_pct`, `fixed_min_degC`, `fixed_max_degC`, `renormalize`, `norm_margin` | not implemented |
| `image_heatmap` | a palette filter (`colormap`) and an overlay filter (`overlay_mode`, `probe_x`, `probe_y`) -- but see §4, they cannot be separated | not implemented |
| `min_temp`, `max_temp`, `ptr_temp` | a node subscribing to the temperature image, parameters `probe_x`, `probe_y`, `unit` | not implemented |
| `camera_info` | needed by none of these filters; the conversion is affine and the readings take `frame_id` from the image header | n/a |
| `zoom_enable` | out of scope; a resolution change also invalidates `camera_info`, so it would need a resize filter *and* a rescaled `CameraInfo` | not implemented |

`RadiometricFilter` takes two parameters: `output_unit` (a unit name from the temperature units table,
deciding both the unit and the output encoding) and `gain` (`high` or `low`, the gain the counts were made with,
which decides the scale of the map). It has no `radiometric` parameter: whoever puts the filter into a chain knows
whether the camera in front of it measures temperatures, and what the filter could do with that knowledge is
either pick another map or refuse to run, neither of which is its business. A non-radiometric camera is no reason
to leave the filter out either -- with `output_unit` set to `counts` it produces the gain-compensated count image,
which is worth having on its own. The detection the driver does over the control channel therefore stays in the
driver, where it currently only ends up in the log.

### The chain inside the driver

`BosonCamera::createPublishers()` builds the chain when `image_temp` is to be published: `createFilterChain()`
takes a parameter prefix (`image_temp`) and a list of default filters (here one filter named `radiometry` of type
`flir_boson_usb/RadiometricFilter`), declares `<prefix>.filter<N>.{name,type}` and configures a
`filters::FilterChain` under that prefix. The driver then pushes `gain` and `temp_mode` into the filter as
`image_temp.filter1.params.{gain,output_unit}`; both are modifiable, so a user can set them directly too, but the
driver only does it once at startup and nothing keeps the two sets in sync afterwards. Two details of
`filters::FilterChain` are worth knowing before touching that code:

- the chain reads the list of filters out of the **parameter overrides** of the node, so a chain written in code
  rather than in a parameter file has to have `<prefix>.filter<N>.name` pushed into the overrides before
  `configure()` is called, which is what `addFiltersToNodeOptions()` does for the component and the node
  executable. Without it the chain silently ends up empty, and `createFilterChain()` reports the wrong number of
  filters;
- an in-process chain needs a prefix of its own on the parameter interface and nothing else -- unlike a
  `sensor_filters` chain node, which needs a node of its own because of its root options (§2). Several in-process
  chains can therefore share the driver node, each under its own prefix, e.g. `image_visual.filter1...`.

## 4. Why the readings are not filters

A filter has one image in and one image out. That is the entire interface, and it is what makes filters freely
composable -- so the design keeps it rather than bending it.

The consequence is worth stating precisely, because it is the one place where the filter model costs something:

- **The scalar readings cannot be filters at all.** A filter has nowhere to hand a `sensor_msgs/Temperature` to.
  They therefore belong to a node of their own that subscribes to the temperature image. That node duplicates the
  probe position parameter with the overlay, which is a consistency hazard we accept and document rather than hide.
- **The overlay cannot be a separate filter from the palette.** `drawOverlay()` today draws the minimum, maximum
  and probe values that `Pipeline` computed from the same frame of *counts*. By the time a palette filter has run,
  the counts are gone, and a filter placed after it cannot recover them. Keeping the interface means the overlay
  must recompute its numbers from the pixels it is given, which in turn means it must sit on the `mono16` branch
  and produce the colour image itself. So palette and overlay merge into one composite filter. That is a real loss
  of composability, and the honest options are to accept it or to extend the filter interface with a second
  auxiliary input upstream.

## 5. What blocks feature parity

- **Runtime reconfiguration.** The node honours `ros2 param set` for the AGC, heatmap, overlay, colormap and probe
  parameters, and the filters that will implement those stages have to honour it too. The mechanism exists and
  works: `filters::FilterChain`, which is what a `sensor_filters` chain and the driver's own chain run on,
  registers an *on-set* callback on the node and forwards to each filter every parameter whose name starts with
  that filter's prefix, refusing the whole call if any filter reports failure. Two consequences for a filter: the
  parameter must not be declared read-only (`cras::FilterBase` declares it modifiable unless told otherwise), and
  the new value has to be read from the argument of `reconfigureCB()` because the parameter interface still holds
  the old one at that point. The radiometric filter does both, and setting its `output_unit` and `gain` on a chain
  configured the way the driver configures it is covered by `test_radiometric_filter_chain` -- including that an
  unknown value is refused and the filter keeps what it had. What is still open is only the work of writing
  `reconfigureCB()` in the filters that do not exist yet.
- **The driver sets the filter parameters once.** `createPublishers()` pushes `gain` and `temp_mode` into the
  filter when the publishers are created, which fits the fact that both are startup presets -- `image_temp` exists
  or it does not, and the encoding is decided with it. The two filter parameters are nevertheless modifiable, so
  setting them by hand desynchronises the image from the driver's own parameters, which still decide the overlay
  unit and the readings. Either they should be declared read-only, or the driver should push them again from its
  parameter callback.
- **The gain is only known from the outside.** Nothing in the pixels says which gain made them, so the number the
  whole conversion rests on is a parameter the user has to keep correct. The frame telemetry the camera can be
  configured to send does carry it; reading it would let the driver set the filter's `gain` per frame instead of
  once at startup.

## 6. Migration

1. Add filters alongside the built-in path; nothing in the node changes and every topic keeps working. *(the
   radiometric filter)*
2. Compare the two paths frame-for-frame in a test that pushes one synthetic frame through `Pipeline::process()`
   and through the filter. *(for the radiometric conversion -- `test_radiometric_filter` checks the pixels against
   the affine map of the unit table in every unit and both gains, and against the bounds and the probe reading the
   stage reports for the same frame)*
3. Switch the node over to the filter for that output, and delete the code the filter replaces. *(done for
   `image_temp`: `captureAndPublish()` hands the raw message to the chain and publishes what comes back, and the
   stage no longer has a temperature image among its outputs)*
4. Go through the remaining outputs the same way, one filter at a time, keeping each switch behind a parity test.
5. Then delete the fan-out from `BosonCamera::captureAndPublish()` and keep only the free functions the filters
   wrap. How much of `Pipeline` is left when all the stages exist is an open question; it may well be nothing, and
   that is acceptable.

## 7. Building and testing

The filter is built only in the ROS 2 branch of `CMakeLists.txt`; the `catkin` branch has no filter library and no
plugin export of its own. It links `cras_cpp_common::cras_filter_base` (and `cras_cpp_common::cras_type_utils`,
because the templated `getParam()` instantiates `cras::typeName<T>()` in the translation unit that calls it) and
`cv_bridge`, and the package does not depend on `sensor_filters` at all -- the chain test drives
`filters::FilterChain` directly instead of starting a chain node. The two filter suites read their parameters from
a node, so they initialise rclcpp and bring their own `main()`; `gtest_main` is then simply never pulled out of
the library it lives in.
`test_radiometric_filter` converts frames and links the filter library, because the filter is an ordinary class
whose definition lives in that library. `test_radiometric_filter_chain` goes through a chain and therefore
deliberately does **not** link it: a class that is both linked into the executable and loaded from a library makes
class_loader report a namespace collision, and the linked copy would not be the copy a chain loads anyway.

`ctest` in the build directory runs the linters and the three test suites. The chain test loads the filter through
pluginlib, so the prefix the package is installed to has to be on `AMENT_PREFIX_PATH` for it to find the plugin
description; `colcon test` puts it there, and when it is not, the test adds the prefix it was compiled with.
Without either of them, the three tests that go through the chain fail and say which classes the loader did find.

Not covered by any test: the ROS 1 build -- no ROS 1 installation was available here, and the `catkin` branch of
`CMakeLists.txt` has not been adapted to the filter-based `image_temp` either. The chain of
`../examples/filters_example.launch.xml` was run by hand, as §2 says, and the driver half of it has never been run
because no camera was attached.
