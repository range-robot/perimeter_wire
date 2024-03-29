

# Perimeter Wire

This repository contains the hardware and software for a perimeter wire sensor designed for autonomous navigation. The project was originally developed by Meadow Robotics and is now released under the GNU General Public License (GPL) following the discontinuation of Meadow Robotics.

This repository is not under active development. However commercial support is available. You can contact me at [@mistoll](https://github.com/mistoll).

The system comprises a generator and a sensor. The generator produces a software-defined signal and operates within a voltage range of 9 to 48V.
The sensor is equipped with three sensor coils for three axes, with the option to also mount Hall sensors. The sensor signal undergoes amplification and is then processed entirely in digital form.

The driver is built on ROS (Robot Operating System) but is also adaptable to other systems.

## License

This project is licensed under the GNU General Public License (GPL) - see the [LICENSE.md](LICENSE.md) file for details.


## Alternative products

 * https://www.goetting-agv.com/components/inductive
 * http://wiki.ardumower.de/index.php?title=Perimeter_wire
 * https://www.robotshop.com/de/de/robotshop-perimeter-wire-generator-sensor-soldering-kit.html


## Theory

Magnetic field strength for z coil:
k: konstant
x: distance orthogonal to wire
d: distance above ground (to wire)

B=k * x / (d^2 + x^2)
https://www.wolframalpha.com/input/?i=sin(arctan(x%2Fd))+*+cos(arctan(x%2Fd))%2Fd

We can use the difference of the absolute signal of two coild to get a 0-crossing signal
For d = 0.4 and coil distance 0.05 we get:
https://www.wolframalpha.com/input/?i=%7Csin%28arctan%28x%2F0.4%29%29+*+cos%28arctan%28x%2F0.4%29%29%2F0.4%7C+-+%7Csin%28arctan%28%28x-0.05%29%2F0.4%29%29+*+cos%28arctan%28%28x-0.05%29%2F0.4%29%29%2F0.4%7C


## Orientation

The hardware is designed to measure a vector in a cartesian coordinate system. However the coil for x is inverted.

Looking at the magnetic field of a loop. We define z > 0 to be inside the loop and when above the loop y > 0 to be forward direction. In that case x > 0 means rotation to the right.

When the pcb is mounted flat on the robot and upper pcb is facing in drive direction orientation is '-xyz'.  When it's upside down (in case) and facing in drive direction orientation is '-x-y-z'. And when its mounted on the front of the robot, with the case / pcb facing in drive direction orientation is '-z-yx'.

## Debugging

* Compile using --cmake-args -DCMAKE_BUILD_TYPE=Debug
* Run openocd
* Connect using vs code (native-debug)

```
"type": "gdb",
"request": "attach",
"name": "Attach to gdbserver",
"executable": "//home/michael/golfcart/workspace/devel/.private/perimeter_wire_generator_firmware/lib/perimeter_wire_generator_firmware/firmware-ATSAMD21E17D.elf",
"gdbpath": "gdb-multiarch",
"target": ":3333",
"remote": true,
"cwd": "${workspaceRoot}/firmware/generator",
"valuesFormatting": "parseText",
```

## Program and Test

See prod/Testing.md