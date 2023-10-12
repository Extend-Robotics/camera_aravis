# camera_aravis

Actively maintained repository for the ROS1 camara_aravis driver. It is open source under the LGPL (like Aravis itself).

The [Aravis](http://live.gnome.org/Aravis) library is a glib/gobject based library for video acquisition using Genicam cameras. It currently implements the gigabit ethernet and USB3 protocols used by industrial cameras.

The camera_aravis driver has long history of multiple forks and now abandoned GitHub repositories. This repository is based on https://github.com/florisvb/camera_aravis.git, which in turn was forked from a deleted github repo (https://github.com/CaeruleusAqua/camera_aravis), which was itself forked from https://github.com/ssafarik/camera_aravis.

------------------------

Tested with Aravis version 0.8.27, requires >= 0.8.25 for multipart data handling.

The basic command to run camera_aravis:

	$ rosrun camera_aravis cam_aravis

To run it in a given namespace:

	$ ROS_NAMESPACE=cam1 rosrun camera_aravis cam_aravis

------------------------
## Continuous Integration

| Service    | Noetic  | Master |
| ---------- | ------- | ------ |
| GitHub     | [![build](https://github.com/FraunhoferIOSB/camera_aravis/actions/workflows/industrial_ci_action.yml/badge.svg?branch=noetic-devel)](https://github.com/FraunhoferIOSB/camera_aravis/actions/workflows/industrial_ci_action.yml/badge.svg?branch=noetic-devel)    | [![build](https://github.com/FraunhoferIOSB/camera_aravis/actions/workflows/industrial_ci_action.yml/badge.svg?branch=master)](https://github.com/FraunhoferIOSB/camera_aravis/actions/workflows/industrial_ci_action.yml/badge.svg?branch=master) |
| ROS Build Farm | [![build](https://build.ros.org/job/Ndev__camera_aravis__ubuntu_focal_amd64/6/badge/icon?style=plastic&subject=build)](https://build.ros.org/job/Ndev__camera_aravis__ubuntu_focal_amd64/6/)   | N/A |

## Configuration

This ROS node publishes messages image_raw and camera_info for a specified camera.  It supports
a variety of camera features via the ROS reconfigure_gui, including the following:
* ExposureAuto         (string: Off, Once, Continuous)
* GainAuto             (string: Off, Once, Continuous)
* ExposureTimeAbs      (float)
* Gain                 (float)
* AcquisitionMode      (string: Continuous, SingleFrame, MultiFrame)
* AcquisitionFrameRate (float)
* TriggerMode          (string: Off, On)
* TriggerSource        (string: Any, Software, Line0, Line1, Line2)
* softwaretriggerrate  (float)
* frame_id             (string)
* FocusPos             (integer)
* usb_mode             (string: SYNC, ASYNC, DEFAULT) { aravis >= v0.8.17 }

Note that the above are also the ROS parameter names of their respective feature.  You may
set initial values for the camera by setting ROS parameters in the camera's namespace.

In addition to the above features, this driver now supports (almost) every feature of every camera,
you just have to know how the feature is specified; each GenICam-based camera contains
an XML file onboard, and by viewing this file you can determine which ROS parameters to set
for camera_aravis to write to the camera.  You can use arv-tool-0.2 to see the feature list
and the XML file (e.g. "arv-tool-0.2 --name=Basler-21285878 features")

Note that for this special feature access, the ROS parameter type must match the feature type.
For example, a Basler ac640 has a boolean feature called "GammaEnable", an integer feature
called "BlackLevelRaw", and a string enum feature called "PixelFormat" that takes values
(Mono8, Mono12, Mono12Packed, YUV422Packed, etc).  The ROS params that you set for these
must be, respectively, a bool, an integer and a string.  Also note that boolean features must
be specified as ROS params false/true, not as integer 0/1.

	$ rosparam set /camera_aravis/GammaEnable false
	$ rosparam set /camera_aravis/BlackLevelRaw 5
	$ rosparam set /camera_aravis/PixelFormat Mono12
	$ rosrun camera_aravis cam_aravis


------------------------
camera_aravis supports multiple cameras, each of which may be specified on the
command-line, or via parameter.  Runs one camera per node.

To specify which camera to open, via the command-line:

	$ rosrun camera_aravis cam_aravis _guid:=Basler-21237813


To specify which camera to open, via a parameter:

	$ rosparam set /camera_aravis/guid Basler-21237813
	$ rosrun camera_aravis cam_aravis

-------------------------

camera_aravis supports multisource cameras and multipart data

use `channel_name`, `pixel_format`, `camera_info_url`, `frame_id` to specify multisource/multipart camera
- `;` seperates multi-source channels
- `,` separates multipart parts
- even if you don't specify `camera_info_urls` keep the correct structure, e.g. `","`
- specify `frame_id` for each source/part, e.g. `"camera_optical_frame,camera_optical_frame"`
- for multipart scenario the order of components should match order the device sends them in
  - typically ordered by `ComponentIDValue`
- for example multisource see `multisource_camera_aravis.launch`
- for example multipart see `photoneo_motioncam.launch`

------------------------

Parameter `pixel_format_internal` overrides pixel format used internally in `camera_aravis`.

You may use it to implement device quirks like custom device pixel formats that are sent under different
GenICam/GiGE-Vision pixel format.

The `pixel_format_internal` works also with multisource and multipart devices
- `;` seperates multi-source channels
- `,` separates multipart components
- you may specify partial overrides
  - e.g. `"PhotoneoYCoCg420,"` means
    - ovveride first component with `PhotoneoYCoCg420` pixel format
    - keep the second component as is
- for example multipart pixel format overriding see `photoneo_motioncam.launch`

--------------------------

It supports the dynamic_reconfigure protocol, and once the node is running, you may adjust
its parameters by running the following and then manipulating the GUI:

	$ rosrun dynamic_reconfigure reconfigure_gui


------------------------
There is an additional nice feature related to timestamps that unifies ROS time with camera time.
We want a stable timestamp on the images that the camera delivers, giving a nice smooth time
delta from frame to frame.  If we were to use the ROS clock on the PC, by the time we get the
image packets from the camera a variable amount of time has passed on the PC's clock due to
variable network and system delays.  The camera's onboard clock is stable but it doesn't match
with the ROS clock on the PC, and furthermore since it comes from a different piece of hardware,
the two clock's rates are slightly different.

The solution is to start with a base of ROS time, and to accumulate the dt's from the camera clock.
To accomodate the difference in clock rates, a PID controller gently pulls the result toward
ROS time.

-------------------------

## Troubleshooting

### MTU

For GigE Vision cameras in high bandwidth scenarios you may need to increase MTU for more fps

On OS side you may do it with `ip`

```bash
# identify your network interface name
ip link list
# set jumbo frames 9000 mtu for network interface 'enp60s0'
sudo ip link set dev enp60s0 mtu 9000
```

On device side you may do it with GenICam:
- `GevSCPSPacketSize` or `DeviceStreamChannelPacketSize`

For example see [`photoneo_motioncam.launch`](https://github.com/Extend-Robotics/camera_aravis/blob/extend/launch/photoneo_motioncam.launch)

Notes:
- if you have switch between CPU and camera
  - that can do the fragmentation
- you may be able to bump only camera MTU
  - and get increased performance
- this is handy when camera MTU is bottleneck for FPS but CPU can't increase MTU

### `ARV_BUFFER_STATUS_TIMEOUT`

If you see a lot of errors like

```bash
[ WARN] [1691651985.401198104]: (range_optical_frame (and possibly subframes)) Frame error: ARV_BUFFER_STATUS_TIMEOUT
```

Check if you are not getting errors on network layer

```bash
# check if errors are growing
netstat -us | grep errors
```

If yes try increasing network buffer sizes

```bash
# read current
sudo sysctl net.core.rmem_max
sudo sysctl net.core.rmem_default
```

The unit is bytes (kernel [docs](https://kernel.org/doc/Documentation/sysctl/net.txt))


```bash
# set new, e.g. try 10x more than your original
sudo sysctl -w net.core.rmem_max=NEW_VALUE
sudo sysctl -w net.core.rmem_default=NEW_VALUE
```

Recheck if still getting errors.

Make final change permanent in `/etc/sysctl.conf`

```bash
tail /etc/sysctl.conf
# for what other values do
#kernel.sysrq=438

###################################################################

# Increase network buffer sizes
# 10 times my original max ~= 16 Mb

net.core.rmem_max = NEW_VALUE
net.core.rmem_default = NEW_VALUE
```





