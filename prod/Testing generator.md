
# Program and Test

## generator pcb

Testing a sensor pcb requires
* completely assembled generator assembly
* PC with linux (set up for build and testing)
* ATMEL ICE Programmer with cable
* USB cable


### Test and Program

1. Setup power supply current limit to 0.1A. Voltage 1V. Connect to generator power.
   Raise voltage slowly to 12V. Current should stay below 0.01A.
2. Connect ATMEL ICE to pc and generator (SAM port, J2)
3. Connect USB cable to pc and generato
4. On testing pc open terminal and run
   > rosrun perimeter_wire_bootloader program.sh

   Disconnect ATMEL ICE

5. Run
   > rosrun perimeter_wire_generator_firmware program.sh

   If you get error "Device unsupported" bootloader is not running.
   Then disconnect the USB and run the command again within 5 seconds.
6. Run
   > rosrun perimeter_wire_generator_driver console -p /dev/ttyACM0 -m 7 -a -b -c 5555 -d 4

   LEDs should light up green. Some output at console.
7. Connect wire between CH1 and short
   Power supply should show current of 0.05A - 0.07A.
   Remove wire
8. Connect wire between CH2 and short
   Power supply should show current of 0.05A - 0.07A.
   Remove wire
9. Program default mode
   > rosrun perimeter_wire_generator_driver console -p /dev/ttyACM0 -m 7 -a -b -c 7010 -d 4 -s
10. Disconnect all cables, then reattach usb cable.
    After 10sec, LEDs should light up green
    > rosrun perimeter_wire_generator_driver console -p /dev/ttyACM0

    Output must contain:
    ```
    Channel A:
    Mode:   0x7
    Divider:        4
    Code:   0x1b62
    Channel B:
    Mode:   0x7
    Divider:        4
    Code:   0x1b62
    ```