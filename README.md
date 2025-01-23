# NuttX for Pygmy

This repository contains the board support package for the [Pygmy flight computer][pygmy] on [NuttX][nuttx].

The Pygmy is an RP2040 base flight computer with several peripherals:
- Micro-SD card
- RN2903 radio transceiver
- L86G GPS
- LSM6DSO32 IMU
- LIS2MDL magnetometer
- MS5607 barometric pressure sensor
- 32KBit EEPROM
- Buzzer
- LEDs

This board support package provides support for all of these peripherals and configuration options for them.

## Usage

Once you have installed all NuttX dependencies and set up the NuttX build environment according to the [NuttX
guide][nx-install], you can clone this repository inside your `nuttx-space`.

**Note:** you should also clone the 2.0.0 version of the [Pico SDK][pico-sdk] in your `nuttx-space` too.

Your folder structure should look like this:

```console
$ ls
nuttx
nuttx-apps
pygmy-nx
pico-sdk
```

Then you can set the `PICO_SDK_PATH` build environment variable:

```console
$ export PICO_SDK_PATH="absolute/path/to/pico-sdk"
```

And then from within the `nuttx` folder, you can configure and build one of the Pygmy configurations. In this case, the
`usbnsh` configuration.

```
$ cd nuttx
$ make distclean
$ ./tools/configure.sh ../pygmy-nx/configs/usbnsh
$ make
```

You should now have a UF2 that you can upload to the Pygmy!

[pygmy]: https://github.com/linguini1/pygmy
[nx-install]: https://nuttx.apache.org/docs/latest/quickstart/install.html
[nuttx]: https://nuttx.apache.org/
