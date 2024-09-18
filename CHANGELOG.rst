^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package camera_aravis
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

4.1.7 (2024-09-18)
-------------------
* MotionCam3D compatibility with firmware 1.13.3

4.1.6 (2023-10-16)
-------------------
* MotionCam3D - expose possibility to work beyond working range from launchfile

4.1.5 (2023-10-13)
-------------------
* document setting camera MTU through GenICam and on OS side
* remove MTU from rosparams (wasn't working anyway, breaking change)
* remove MTU from dynamic reconfigure (might cause corrupted data, breaking change)
* MotionCam3D
 * expose MTU setting through GenICam from launchfile (also example)
 * expose setting white balance preset from launchfile

4.1.4 (2023-08-16)
-------------------
* Always preffer multipart mode over chunked

4.1.3 (2023-08-10)
-------------------
* Adapt ROI at runtime to received data
* Photoneo MotionCam3D ColorCamera component (raw RGB) launchfile support
 * With intrinsics (camera_info) and extrinsics (tf transform)
* Document fix for network layer buffer timeouts
 * Fixes ARV_BUFFER_STATUS_TIMEOUT warnings
 * this is separate from wrong application layer logic fix in 4.1.1

4.1.2 (2023-08-04)
-------------------
* Photoneo MotionCam3D Scanner mode support
* Photoneo MotionCam3D TextureSource support
* expose Photoneo MotionCam3D parameters in launchfile
* optimize Photoneo YCoCg(-R) pixel format for SIMD autovectorization

4.1.1 (2023-07-28)
-------------------
* Fix exposure time incorrectly set for MotionCam from dynamic reconfigure
* Add simple internal buffer processing time benchmark
* Decouple application buffer processing from aravis
 * Fixes ARV_BUFFER_STATUS_TIMEOUT warnings

4.1.0 (2023-07-25)
------------------
* Launchfile and sample calibration for Photoneo MotionCam M+ (multipart)
* Rework frame_id support
 * Breaking changes
  * Removed frame_id from dynamic reconfigure
  * Removed optical frame tf publishing
* Internal FloatToUint pixel format (depth map conversion)
* Internal custom Photoneo YCoCg 4:2:0 subsampling pixel format
* Mechanism for overriding pixel format used internally
* Automatically disable/enable multipart components
* Multipart data support
 * Breaking changes: multisource separator is now `;` and multipart `,`

4.0.4 (2022-12-23)
------------------
* Update package maintainer
* Refactor node params (`#21 <https://github.com/FraunhoferIOSB/camera_aravis/issues/21>`_)
  * Refactor node params
  * Rename extended_camera_info\_ -> pub_ext_camera_info\_
  * Move stream parameters to the top of onInit()
* fix: only reset PTP clock when in "Faulty" or "Disabled" state (`#23 <https://github.com/FraunhoferIOSB/camera_aravis/issues/23>`_)
* Update industrial_ci default branch to main
* Contributors: Dominik Kleiser, Peter Mortimer, Ruf, Boitumelo

4.0.3 (2022-07-08)
------------------
* Refactor image conversion (`#20 <https://github.com/FraunhoferIOSB/camera_aravis/issues/20>`_)
* Use plain file names for includes (`#17 <https://github.com/FraunhoferIOSB/camera_aravis/issues/17>`_)
* Add verbose flag for feature detection (default = false) (`#19 <https://github.com/FraunhoferIOSB/camera_aravis/issues/19>`_)
* Assume num_streams\_ = 1 if DeviceStreamChannelCount and GevStreamChannelCount unavailable (`#18 <https://github.com/FraunhoferIOSB/camera_aravis/issues/18>`_)
* Add Line0 to Line5 to TriggerSource Enum
* Fix: nodelet namespace
* Fix: onInit deadlock
* Contributors: Dominik Kleiser, Boitumelo Ruf, Thomas Emter, Peter Mortimer, tas, Geoff McIver

4.0.2 (2022-05-04)
------------------
* Add optional ExtendedCameraInfo message to publish additional camera acquisition parameters
* Fix: Set reasonable height and width when not given in the CameraInfo
* Contributors: Peter Mortimer

4.0.1 (2022-03-25)
------------------
* Add ROS getter/setter services for camera features
* Add support for multistream encoding conversion
* Fix: Pass on the correct encoding for the additional streams of multisource cameras
* Fix: Continuously check the spawning\_ flag
* Fix: Check spawning\_ flag only once during spawnStream
* Contributors: Peter Mortimer, Thomas Emter, Dominik Kleiser

4.0.0 (2021-10-27)
------------------
* Major refactoring
* Add support for ROS Noetic and aravis-0.6
* Fix several bugs (see git history)
* Add new features:

  * Support for multisource cameras
  * Zero-copy transport with ROS nodelets
  * Camera time synchronization
  * Example launch files

* Update package author and maintainer
* Contributors: Dominik Klein, Floris van Breugel, Gaël Écorchard, Thomas Emter, Peter Mortimer, Dominik Kleiser
