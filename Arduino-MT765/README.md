# Challenger MT765 Arduino

This sketch runs on an Arduino Uno for the MT765 control box. It listens for ROS commands via `rosserial` and sets PWM outputs for throttle, transmission and steering.

## Requirements

* Arduino IDE 1.8+ or `arduino-cli` >= 0.33
* Board: **Arduino/Genuino Uno** (`arduino:avr:uno`)

## Compile and Upload

```bash
arduino-cli compile --fqbn arduino:avr:uno Arduino-MT765.ino
arduino-cli upload -p /dev/ttyACM0 --fqbn arduino:avr:uno Arduino-MT765.ino
```

After uploading, start the serial bridge:

```bash
rosrun rosserial_python serial_node.py /dev/ttyACM0 _baud:=57600
```

## ROS Topics

The firmware subscribes to these topics for command input:

* `/throttle/put` – throttle setting (`std_msgs/UInt8`)
* `/transmission/put` – forward/neutral/reverse (`std_msgs/UInt8`)
* `/steering/put` – steering angle (`std_msgs/UInt8`)

It drives the actuators directly without publishing feedback topics.
