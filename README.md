# CH32-V003-based Stylophone

## Features
* 20-tone keyboard and octave-shift button
* Easily customizable, digitally synthesized sound
* USB bootloader from [rv003usb](https://github.com/cnlohr/rv003usb)
* Easy and cheap to build
* Powered by cnlohr's [ch32fun](https://github.com/cnlohr/ch32fun) environment

Note: Current schematic has an amplifier that doesn't work well.
Despite being usable, it is rather quiet and inefficient, so I highly recommend designing your own (either with two mosfets and a driver like TC428 or a 20kHz low-pass plus a linear audio amplifier (like MD8002).

## First time use
To build the firmware, you should firstly follow the instructions at [ch32fun wiki](https://github.com/cnlohr/ch32fun/wiki/Installation) to install the toolchain.
You will also need to install `cmake` (alternatively, you can try to somehow integrate the project into the make ecosystem of ch32fun).

Prebuilt binaries for the bootloader, bootloader configuration utility and minichlink are available in `bin` folder for convenience.
You can build them from source, but you'll need to apply the patch in `bin/rv003usb.patch` to the [rv003usb](https://github.com/cnlohr/rv003usb) repository at commit `4674694672937776d9196526dcf2af6adc5523bc` (feel free to use a newer version, but you'll have to transfer the changes manually).
The patch contains hardware-specific configuration.

To flash and configure the bootloader, firstly connect the WCH-linkE to the stylophone. (Note for people who got their boards from LSTME: the programming header is located directly above the microcontroller. The pinout goes from left to right as follows: 3v3, GND, SWDIO)
Then, run the following command:
Linux:
```shell
./bin/minichlink -a -w bin/configurebootloader.bin flash -w bin/bootloader.bin bootloader -b
```

Windows:
```shell
.\bin\minichlink.exe -a -w bin\configurebootloader.bin flash -w bin\bootloader.bin bootloader -b
```

After this is done, unplug the stylophone and the programmer, then proceed to the next section.

## Build and flash
To enter the programming mode, connect the stylus and press F (the ninth key including semitones) on the stylophone keyboard.
While this is pressed, connect the stylophone to your computer with a USB cable (do not connect a WCH-LinkE).
You should now see a new USB device (on Linux, you should see it with `lsusb`. On Windows, it'll appear in device manager).

To build and flash the project, use the following commands (both Windows and Linux):
```shell
mkdir build
cd build
cmake ..
cmake --build . --target flash
```
This builds the project and flashes it using the USB bootloader, then immediately starts running the uploaded code.
