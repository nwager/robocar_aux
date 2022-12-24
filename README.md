# robocar_aux

Runs on a Raspberry Pi Pico and communicates with the Robocar Jetson Nano via USB serial.

## Dependencies

### CMake

My Jetson Nano flashed with CMake 3.10 which is too old for this project. I upgraded to the latest CMake by following [this answer](https://askubuntu.com/a/1205458). I've listed the commands below:

```bash
sudo apt-get update
sudo apt-get install apt-transport-https ca-certificates gnupg software-properties-common wget
# add updated repository
wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc 2>/dev/null | sudo apt-key add -
sudo apt-add-repository 'deb https://apt.kitware.com/ubuntu/ bionic main'
# install
sudo apt-get update
sudo apt-get install cmake
```

### Pico SDK

Clone the Raspberry Pi Pico SDK repo and set the corresponding environment variables (can source them on shell startup for convenience):

```bash
git clone https://github.com/raspberrypi/pico-sdk.git --recurse-submodules
export PICO_SDK_PATH=path/to/pico-sdk
```

You also need to install the dependencies for the Pico toolchain:

```bash
sudo apt install cmake gcc-arm-none-eabi libnewlib-arm-none-eabi libstdc++-arm-none-eabi-newlib
```

Additionally, you can install [picotool](https://github.com/raspberrypi/picotool) to upload the executable without having to unplug the Pico. After installing picotool, you can either run it with `sudo` or add a udev rule ([udev rule source](https://gist.github.com/tjvr/3c406bddfe9ae0a3860a3a5e6b381a93)):
```
# /etc/udev/rules.d/99-pico.rules

# Make an RP2040 in BOOTSEL mode writable by all users, so you can `picotool`
# without `sudo`. 
SUBSYSTEM=="usb", ATTRS{idVendor}=="2e8a", ATTRS{idProduct}=="0003", MODE="0666"

# Symlink an RP2040 running MicroPython from /dev/pico.
#
# Then you can `mpr connect $(realpath /dev/pico)`.
SUBSYSTEM=="tty", ATTRS{idVendor}=="2e8a", ATTRS{idProduct}=="0005", SYMLINK+="pico"
```

### FreeRTOS SMP

Clone the FreeRTOS-Kernel repo, checkout the `smp` branch, and set the corresponding environment variable:
```bash
git clone https://github.com/FreeRTOS/FreeRTOS-Kernel.git
cd path/to/FreeRTOS-Kernel
git checkout smp
export PICO_FREERTOS_PATH=path/to/FreeRTOS-Kernel
```

