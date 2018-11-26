examples/pwm
^^^^^^^^^^^^

  This is a simple example of PWM function.
  Available as nsh built-in command.
  This example can control PWM by select PWM device and the pulse duty ratio,
  the pulse frequency, the duration.

  Configuration in this example:

    CONFIG_EXAMPLES_PWM - Enable pulse width modulation (PWM) example
    CONFIG_CXD56_PWM    - Enable PWM function
    CONFIG_CXD56_PWM0   - Enable PWM channel 0
    CONFIG_CXD56_PWM1   - Enable PWM channel 1
    CONFIG_CXD56_PWM2   - Enable PWM channel 2
    CONFIG_CXD56_PWM3   - Enable PWM channel 3

  Build(In <spresense>/sdk):
    $ ./tools/config.py --kernel release
    $ ./tools/config.py board/spresense examples/pwm
    $ make buildkernel
    $ make

  Flash(In <spresense>/sdk):
    $ ./tools/flash.sh nuttx.spk

  Operation:
    * Launch "pwm" for output PWM with current settings.
      $ pwm

    * Launch "pwm -f" for output PWM with frequency setting.
      $ pwm -f 2000

    * Launch "pwm -d" for output PWM with duty ratio setting.
      $ pwm -d 30

    * Launch "pwm -t" for output PWM with duration setting.
      $ pwm -t 5

    * Launch "pwm -p" for output PWM with PWM channel setting.
      $ pwm -p /dev/pwm0

