examples/bluetooth_spp
^^^^^^^^^^^^^^^^^^^^^^

  This is a simple example of Bluetooth SPP data transfer.
  Available as nsh built-in command.
  This example turn on bluetooth and try to connect other device via SPP
  and Receive Rx data.

  Configuration in this example:

    CONFIG_EXAMPLES_BLUETOOTH_SPP - Enable this example

  Build(In <spresense>/sdk):
    $ ./tools/config.py examples/bt_spp
    $ make buildkernel
    $ make

  Flash(In <spresense>/sdk):
    $ ./tools/flash.sh nuttx.spk

  Operation:
    1. Launch "bt_spp" application by NuttShell
      $ bt_spp

    2 Connect "SONY_BT_SPP_SAMPLE" by Android or PC

    3. Send SPP data by Android or PC with terminal application

    4. bt_spp application will output
              "onSppReceiveData [BT] Receive Data data[0] = 0xXX, Length = NN"

    5. Loop back data will come in Android or PC terminal

