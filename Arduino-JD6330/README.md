# John Deere 6330 Arduino

This directory contains the sketch used on an Arduino board for controlling the JD6330 tractor. It communicates with ROS over `rosserial`.

## Requirements

* Arduino IDE 1.8 or newer (or `arduino-cli` >= 0.33)
* Board: **Arduino/Genuino Uno** (`arduino:avr:uno`)

## Building with `arduino-cli`

```bash
arduino-cli compile --fqbn arduino:avr:uno Arduino-JD6330.ino
```

## Uploading

Connect the board (usually `/dev/ttyACM0`) and run:

```bash
arduino-cli upload -p /dev/ttyACM0 --fqbn arduino:avr:uno Arduino-JD6330.ino
```

Once uploaded, bring up the ROS serial bridge:

```bash
rosrun rosserial_python serial_node.py /dev/ttyACM0 _baud:=57600
```

## ROS Topics

This microcontroller subscribes to the following command topics:

* `/steering/put` – desired steering angle (`std_msgs/UInt8`)
* `/transmission/shuttle/put` – shuttle control (`std_msgs/UInt8`)
* `/throttle/put` – throttle command (`std_msgs/UInt8`)
* `/hitch/put` – 3‑pt hitch position (`std_msgs/UInt8`)
* `/ignition/put` – ignition enable (`std_msgs/Bool`)
* `/passthrough/put` – manual/auto switch (`std_msgs/Bool`)

The board publishes status on matching `/get` topics such as `/steering/get` and `/throttle/get`.
