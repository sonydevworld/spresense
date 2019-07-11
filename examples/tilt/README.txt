examples/tilt
^^^^^^^^^^^^^^

  This is simple example of detect tilt from sensor device using Sensor
  Control Unit. Available as nsh built-in command.

  Supported sensors are:

  - Bosch BMI160 sensor
  - Kionix KX022/KX122 acceleration sensor

  This application depends on these devices and SCU configuration, but each
  sensors can't be used on the same time.

  Configuration Pre-requisites:

    CONFIG_BMI160    - BMI160
    CONFIG_KX022     - KX022
    CONFIG_CXD56_SCU - CXD56xx Sensor Control Unit

  Example Configuration:

    CONFIG_EXAMPLES_TILT - Enable tilt example
    CONFIG_EXAMPLES_TILT_PROGNAME - Program name.
    CONFIG_EXAMPLES_TILT_PRIORITY - Example priority. Default: 100
    CONFIG_EXAMPLES_TILT_STACKSIZE - Example stack size. Default: 2048

  Operation:

    Run this application from nsh. If you tilt the board, it detects
    the motion and displays the results.
