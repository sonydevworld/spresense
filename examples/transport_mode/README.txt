
Usage of transport_mode
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
      [Bosch BMP280 Barometic Pressure Sensor] <= Y
        [SCU Sequencer] <= Y
      [Asahi AK09911/AK09912 Compass Sensor] <= Y
        [SCU Sequencer] <= Y

- [Sensing]
    [Sensing manager] <= Y
      [Sensing manager power control enable] <= Y
    [Barometer] <= Y
    [Transport mode] <= Y

- [ASMP] <= Y
- [Examples]
    [Transport mode sensor example] <= Y

Or use transport_mode default configuration

$ ./tools/config.py examples/transport_mode
^^^^^^^^^^^^^
Build and install
--------------------------

Type 'make' to build SDK.
Install 'nuttx.spk' to system.

Execute
--------------------------

Type 'transport_mode' on nsh.
nsh>transport_mode

TRAM has three states:

- MS or Motion Sensing state
- CMD or Continuous Motion Detection state
- TMI or Transportation Mode Inference state

When you keep moving, the state chages MS -> CMD -> TMI,
and mode inference is performed in the TMI state.
While in the TMI state, the infered mode is put to console log like this.

Mode: Walking

This is an example for using the Transportation Mode Inference Engine(TRAM)
library. TRAM can infer 13 modes how user is transporting by using three
kinds of sensors.

  The modes are:

  - Staying
  - Walking
  - Running
  - Ascending stairs
  - Descending stairs
  - Going up on escalator
  - Going down on escalator
  - Going up in elevator
  - Going down in elevator
  - Getting on train
  - Getting on bus
  - Getting in car
  - Riding bicycle

