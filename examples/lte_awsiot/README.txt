examples/lte_awsiot
^^^^^^^^^^^^^^^^^^^

  This application is a sample that connect to the LTE network
  and subscribe/publish to the AWS IoT server.
  Original sample file is :
    externals/awsiot/aws-iot-device-sdk-embedded-C/samples/linux/subscribe_publish_sample/subscribe_publish_sample.c

  Supported LTE modem is ALT1250.

  Build kernel and SDK:

  This application can be used by lte_awsiot default config.

  $ ./tools/config.py examples/lte_awsiot
  $ make

  Execute under nsh:

  Type 'lte_awsiot' on nsh like this.
  nsh> lte_awsiot

