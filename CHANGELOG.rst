.. SPDX-License-Identifier: MIT
.. SPDX-FileCopyrightText: 2018 FLIR Systems, INC
.. SPDX-FileCopyrightText: 2018-2019 AutonomouStuff, LLC
.. SPDX-FileCopyrightText: Czech Technical University in Prague

^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package flir_boson_usb
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. 3.0.0 (2026-10-01)
.. ------------------
.. Breaking release: the AGC, normalisation, heatmap and radiometric processing moved out of the
.. node into a ROS-independent ``flir_boson_usb::Pipeline`` stage, and a topic now either carries
.. data or does not exist at all.
..
.. * Topics
..
..   * ``image8`` renamed to ``image_visual`` (``mono8``).
..   * ``image8_norm`` removed; ``agc_norm`` re-stretches ``image_visual`` in place instead of producing a second stream.
..   * ``image_temp`` is now the absolute temperature of every pixel instead of a ``bgr8`` overlay of the heatmap. The unit is chosen by ``temp_mode``: ``c``, ``k`` and ``f`` are degrees Celsius, Kelvin and Fahrenheit as ``32FC1``, and ``centi_c``, ``centi_k`` and ``centi_f`` are the same units in hundredths as 16-bit integers.
..   * ``min_temp``, ``max_temp`` and ``ptr_temp`` are only published for radiometric cameras, and report the bounds of the stretch the visible pixels were made from.
..   * ``image_visual``, ``image_heatmap``, ``image_temp`` and the three temperature topics are no longer advertised when the corresponding parameter disables them.
..   * All images of one frame share the ``header.stamp`` and ``header.frame_id`` of ``image_raw``.
..
.. * Parameters
..
..   * ``publish_color`` removed. Its YUV half is now ``yuv_color`` (``image_raw`` as ``bgr8``), its heatmap half is now ``heatmap_mode``.
..   * ``raw16_agc_low_pct`` / ``raw16_agc_high_pct`` renamed to ``agc_low_pct`` / ``agc_high_pct``; the AGC bounds are now computed in 16-bit counts instead of 8-bit grey levels.
..   * ``norm_margin`` renamed to ``agc_norm_margin`` and only honoured together with the new ``agc_norm``.
..   * Added ``agc_mode`` (``none``/``fixed_range``/``auto_range``), ``heatmap_mode``, ``overlay_mode``, ``temp_mode`` (``none``/``c``/``k``/``f``/``centi_c``/``centi_k``/``centi_f``), ``radiometric`` and ``radiometric_patterns``.
..   * Added ``colormap``, which chooses the palette ``image_heatmap`` is painted with; the palette used to be hard-coded to JET. It is a dynamic parameter, so the palette can be switched while the node is running, and the palettes the built OpenCV does not have are not offered.
..   * ``overlay_mode`` is a dynamic parameter too, so the stamped content can be switched while the node is running.
..   * Invalid parameter combinations are refused at parameter-set time instead of silently corrected, and the probe point is validated against the real published size (so a 640-sized probe on a Boson\_320 is rejected at startup).
..
.. * Heatmap overlay
..
..   * The stamped readings are printed in the unit of ``temp_mode`` and carry its unit (``26.85 deg C``, ``2685 cdeg C``) instead of always degrees Celsius. A non-radiometric camera, for which no absolute temperature exists, prints the raw 16-bit counts instead (``30000 counts``).
..   * The text is drawn white on a black outline, so it stays readable whatever colour the palette picks for the pixels under it.
..
.. * Launch files
..
..   * The ``rectify`` block rectifies ``image_raw``, ``image_visual``, ``image_heatmap`` and ``image_temp`` according to the enabled modes instead of the removed ``image8`` / ``image8_norm`` topics.
..   * ``point_x`` and ``point_y`` are now actually passed to the node.
..   * ``flir_boson_320.launch.xml`` includes the ROS 2 launch file that exists.

2.0.0 (2026-05-28)
------------------
* ROS 2 Humble support (https://github.com/akhilj95/flir_boson_usb2)

1.2.1 (2019-07-01)
------------------
* Merge pull request `#3 <https://github.com/astuff/flir_boson_usb/issues/3>`_ from valgur/patch-1
* Fix minor issues detected by catkin_lint
* Fixing installation of nodelet_plugins.xml.
* Contributors: Joe Driscoll, Joshua Whitley, Martin Valgur, Sam Rustan

1.2.0 (2019-01-24)
------------------
* Merge pull request `#2 <https://github.com/astuff/flir_boson_usb/issues/2>`_ from astuff/feat/add_camera_info_manager
  Feat/add camera info manager
* Defaulting to 60 fps everywhere.
* Swapping boost shared pointers for std shared pointers.
* More expressive error messages.
* Adding frame_rate parameter.
* Adding frame_id param and actually making it do something useful.
* Adding launch file with rectification.
* Cleaning up example file and launch file. Added namespace to launch.
* Adding example Boson_640.yaml calibration file.
* Adding camera_info_manager for image rectification.
* Contributors: Joshua Whitley, Rinda Gunjala

1.1.2 (2019-01-23)
------------------
* Found bug in device path parameter.
* Contributors: Joshua Whitley

1.1.1 (2019-01-21)
------------------
* Temporarily removing camera_info_manager.
* Contributors: Joshua Whitley

1.1.0 (2019-01-21)
------------------
* Modified install of launch folder.
* Added roslint and cleaned up based on suggestions.
* Converted driver to BosonCamera nodelet.
* Contributors: Joshua Whitley

1.0.0 (2018-12-13)
------------------
* Adding LICENSE.
* Adding status badge to README.
* Adding launch file.
* Initial commit.
* Contributors: Joshua Whitley
