examples/tilt
^^^^^^^^^^^^^^

  This is simple example of detect tilt from sensor device using Sensor
  Control Unit. Available as nsh built-in command.

  Supported sensors are:

  - Bosch BMI160 sensor

  Configuration Pre-requisites:

    CONFIG_BMI160    - BMI160
    CONFIG_CXD56_SCU - CXD56xx Sensor Control Unit

  Example Configuration:

    CONFIG_EXAMPLES_TILT - Enable tilt example
    CONFIG_EXAMPLES_TILT_PROGNAME - Program name.
    CONFIG_EXAMPLES_TILT_PRIORITY - Example priority. Default: 100
    CONFIG_EXAMPLES_TILT_STACKSIZE - Example stack size. Default: 2048

  Operation:

    Program will be stop when hit space to console.
    If you can't stop this example, keep pressing the space until finished.
    Or enable CONFIG_SCHED_WAITPID to run as foreground task.
