examples/adc
^^^^^^^^^^^^

  This is simple example of take AD conversion data using Sensor Control
  Unit. Available as nsh built-in command.
  This example obtains and shows AD conversion data in every 1 second.
  It will end when the read count is reached to CONFIG_EXAMPLES_ADC_READCOUNT.

  This application depends on its ADC and SCU configuration.

  Configuration Pre-requisites:

    CONFIG_CXD56_ADC - CXD56xx ADC
    CONFIG_CXD56_SCU - CXD56xx Sensor Control Unit

  Example Configuration:

    CONFIG_EXAMPLES_ADC           - Enable this example
    CONFIG_EXAMPLES_ADC_PROGNAME  - Program name.
    CONFIG_EXAMPLES_ADC_PRIORITY  - Example priority. Default: 100
    CONFIG_EXAMPLES_ADC_STACKSIZE - Example stack size. Default: 2048
    CONFIG_EXAMPLES_ADC_DEVPATH   - ADC device path. Default: /dev/lpadc0
    CONFIG_EXAMPLES_ADC_READCOUNT - Number of reads. Default: 10
    CONFIG_EXAMPLES_ADC_BUFSIZE   - Buffer size of ADC example.

  Operation:

    No console operation is needed. You can see the average, maximum and
    minimum value of AD conversion result.
