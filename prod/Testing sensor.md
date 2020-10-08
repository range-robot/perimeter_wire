
# Program and Test

## sensor pcb

Testing a sensor pcb requires

* completely assembled sensor pcb
* PC with linux (set up for build and testing)
* ATMEL ICE Programmer with cable
* USB cable
* PW generator
* multimeter

### Programming

1. Connect ATMEL ICE to pc and sensor pcb (SAM port)
2. Connect USB cable to pc and sensor pcb
3. Run
   > rosrun perimeter_wire_bootloader program.sh
4. Run
   > rosrun perimeter_wire_sensor_firmware program.sh

   If you get error "Device unsupported" bootloader is not running.
   Then disconnect the USB and run the command again within 5 seconds.

### Testing

Disconnect all cables.

1. Set potentiometers (3x) to 100kOhm +- 2kOhm (center position, Pins 1-2. R27: left, bottom, R25/R26: bottom, right)
2. Setup generator wire
   Short connector, 200Ohm, 12V
   Code: 0x1b62 (7010)
   Divider: 4
   Mode. 7
   Loop: Rectangle 800mm x 1400mm, 0,5mm² (generator on long edge)
3. Connect Sensor to pc and start
   > rosrun perimeter_wire_sensor_driver console -p /dev/ttyACM0 -DS -d 4 -c 7010 -r 0 -F 10 -f 32
4. Position the sensor 60mm above the wire (use spacer!), middle of longe edge, away from generator.
   Stay away from sensor (at least 300mm).
   Check measurements. The position of the inductor must be aligned with the loop.

   | Aligned  | \|X\|     | \|Y\|     | \|Z\|     |
   |----------|-----------|-----------|-----------|
   | L1       | < 0.5     | > 5.0     | 1.0 - 4.0 |
   | L2       | > 5.0     | < 0.5     | 1.0 - 4.0 |
   | L3       | 1.0 - 4.0 | > 5.0     | < 0.5     |

   Write this table to the test protocoll (using real values)
