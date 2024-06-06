# asus_xtion_docker

# to compile

mkdir build \
cd build \
cmake .. \
make \
./OpenNI2_OpenCV


# from openni2 doc to use mutliple cameras - NOT DONE YET

Opens a device.  This can either open a device chosen arbitrarily from all devices
on the system, or open a specific device selected by passing this function the device URI.

To open any device, simply pass the constant@ref ANY_DEVICE to this function.  If multiple
devices are connected to the system, then one of them will be opened.  This procedure is most
useful when it is known that exactly one device is (or can be) connected to the system.  In that case,
requesting a list of all devices and iterating through it would be a waste of effort.

If multiple devices are (or may be) connected to a system, then a URI will be required to select
a specific device to open.  There are two ways to obtain a URI: from a DeviceConnected event, or
by calling @ref OpenNI::enumerateDevices().

In the case of a DeviceConnected event, the @ref OpenNI::Listener will be provided with a DeviceInfo object
as an argument to its @ref OpenNI::Listener::onDeviceConnected "onDeviceConnected()" function.  
The DeviceInfo.getUri() function can then be used to obtain the URI.

If the application is not using event handlers, then it can also call the static function
@ref OpenNI::enumerateDevices().  This will return an array of @ref DeviceInfo objects, one for each device
currently available to the system.  The application can then iterate through this list and
select the desired device.  The URI is again obtained via the @ref DeviceInfo::getUri() function.