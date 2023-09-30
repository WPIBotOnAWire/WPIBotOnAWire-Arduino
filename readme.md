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
