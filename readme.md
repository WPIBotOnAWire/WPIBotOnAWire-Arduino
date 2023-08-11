# Arduino code

Provides interface with low-level devices: sensors, motors, etc.

ROS interface is provided via a Serial on pins 0 and 1 -- **not** the main USB connector, which is used for sending debugging info.

On a ROS device connected via USB, run

```rosrun rosserial_python serial_node.py /dev/ttyxxxx```

where the port is typically `/dev/ttyACM0` or `/dev/ttyUSB0` -- you may need to list out the ports to find it.

More info about rosserial can be found at http://wiki.ros.org/rosserial_arduino/Tutorials/
