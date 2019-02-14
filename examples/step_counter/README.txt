
Usage of step_counter
===========================

Usage
---------------------------

Select options in below.

- [CXD56xx Configuration]
    [I2C0] <= Y
    [Sensor Control Unit] <= Y
- [Memory manager] <= Y
    [Memory Utilities]
      [Memory manager] <= Y
      [Message] <= Y
- [Drivers]
    [Sensor Drivers] <= Y
      [Bosch BMI160 Sensor support] <= Y
        [SCU Sequencer] <= Y
- [Sensing]
    [Sensing manager] <= Y
    [Step counter] <= Y
- [ASMP] <= Y
- [Examples]
    [Step counter sensor example] <= Y

Or use stepcounter default configuration

$ ./tools/config.py examples/step_counter

Build and install
--------------------------

Type 'make' to build SDK.
Install 'nuttx.spk' to system.

Execute
--------------------------

Type 'step_counter' on nsh.
nsh>step_counter

The following parameters are displayed until stopped.

Display parameters:
 tempo, stride, speed, distance, time stamp, step, move-type

Stop is the press of the space key.
After stopping example will end.

