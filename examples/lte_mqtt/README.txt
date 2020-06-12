examples/lte_mqtt
^^^^^^^^^^^^^^^^^

  This application is a sample that connect to the LTE network
  and subscribe to the MQTT server.
  Original sample file is :
    externals/mqtt/paho.mqtt.embedded-c/MQTTClient-C/samples/linux/stdoutsub.c

  Supported LTE modem is ALT1250.

  Get MQTT submodule:

  $ git submodule init
  $ git submodule update

  Build kernel and SDK:

  This application can be used by lte_mqtt default config.

  $ ./tools/config.py examples/lte_mqtt
  $ make

  Execute under nsh:

  Type 'lte_mqtt <options>' on nsh like this.
  nsh> lte_mqtt /test --host iot.eclipse.org --clientid test

