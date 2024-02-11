# Arduino code

## Building

You'll need to comment out the SERCOM5 in your samd variants:

    `.../.platformio/packages/framework-arduino-samd-sparkfun/variants/SparkFun_SAMD_Mini/variant.cpp`

(currently lines 217 - 220)

Otherwise, the code should just pull from various github libraries (see .ini file).

## Connecting from ROS

Provides interface with low-level devices: sensors, motors, etc.

ROS interface is provided via a Serial on pins 0 and 1 -- **not** the main USB connector, which is used for sending debugging info.

On a ROS device connected via USB, run

```rosrun rosserial_python serial_node.py /dev/ttyxxxx```

where the port is typically `/dev/ttyACM0` or `/dev/ttyUSB0` -- you may need to list out the ports to find it.

More info about rosserial can be found at http://wiki.ros.org/rosserial_arduino/Tutorials/

## List of SERCOMs

* SERCOM0: Jetson via UART on D0 and D1.
* SERCOM1: Standard SPI on D10-13. Nothing connected, yet. Might be disabled, since I'm using the TCs connected to those pins.
* SERCOM2: GPS on D3 and D4. `See gps-ROS.cpp`.
* SERCOM3: Standard I2C on D20 and D21. So far just battery monitor.
* SERCOM4: TFmini on pins A1 and A2 (D15 and D16, I think). See `tfmini-ROS.cpp`.
* SERCOM5: TFmini on pins D6 and D7. Note that you have to remove the default SERCOM5 in Arduino to make this work. See `tfmini-ROS.cpp` for deets.

## List of Custom Timers

* TCC0:4 on D2 for the ESC. 50Hz. **N.B. PA14 and PA15 are on the same MUX register!** Be careful not to clobber. 
* TC3:1 on D5 for the LED. 10Hz. **N.B. PA14 and PA15 are on the same MUX register!** Be careful not to clobber. 
* TCC2 on D11 for sound. Variable freq. Set up in LED, since they share a source clock.
