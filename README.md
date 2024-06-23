
# asus_xtion_docker

  

This repo contains the code to save and view RGB-D Data from an ASUS Xtion Sensor.

  

## Tips and Tricks to get the legacy sensors working

  

- All 3 Xtion from SIPG Lab have been updated to latest firmware to support USB 3.0.

- Newer operating systems have moved away from `ehci_hcd`USB Controller and are now using the `xhci_hcd`. And the combination of this newer controller with the old microphones in the ASUS can be a problem in some combination of hardware+OS. 
- The easiest way to get rid of the problem is just by blacklisting the driver responsible to for USB-Audio, in Ubuntu it is usually: `snd_usb_audio`:
	- `echo "blacklist snd_usb_audio" >> /etc/modprobe.d/blacklist.conf`
- Use `dmesg -w` to check is there is any USB Error.
- The expected output is solely:
```bash
    [165499.136372] usb 3-2: New USB device strings: Mfr=5, Product=4, SerialNumber=0
    [165499.136377] usb 3-2: Product: PrimeSense Device
    [165499.136380] usb 3-2: Manufacturer: PrimeSense
    [165575.144797] usb 2-1: new SuperSpeed USB device number 74 using xhci_hcd
    [165575.169645] usb 2-1: New USB device found, idVendor=05e3, idProduct=0626, bcdDevice= 6.56
```

### Multiple Cameras in one Computer
The USB bandwidth used  by an ASUS Xtion is in the order of Gigabits, so having more than one sensor connected to the same USB Controller will cause problems and latency when using the cameras, if it even becomes possible. So use `lsubs -t` and try to plug the ASUS into different USB Buses. 

Use smaller resolutions for both RGB or Depth, in order to be sure that the USB controller is not saturated.

### Using USB extensions 
Use active USB extension in order to preserve the most the signal between the sensor and the computer. The ASUS are not meant to be used with a much bigger cable than the one it comes with.

  

# Requirements
```bash
sudo apt install libopenni2-dev libpcl-dev
```
OpenNi2 can be downloaded from here https://structure.io/openni/. And in the CMakeList.txt file point the OPENNI2_DIR to the folder with the lib files.

# Building

```bash
mkdir build \
cd build \
cmake .. \
make \
./OpenNI2_OpenCV
```
