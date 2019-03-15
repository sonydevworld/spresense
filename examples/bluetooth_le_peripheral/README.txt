examples/bluetooth_le_peripheral
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

  This is a simple example of Bluetooth SPP data transfer.
  Available as nsh built-in command.
  This example turn on bluetooth and try to connect other device via SPP
  and Receive Rx data.

  Configuration in this example:

    CONFIG_EXAMPLES_BLUETOOTH_LE_PERIPHERAL - Enable this example

  Build(In <spresense>/sdk):
    $ ./tools/config.py examples/ble_peripheral
    $ make buildkernel
    $ make

  Flash(In <spresense>/sdk):
    $ ./tools/flash.sh nuttx.spk

  Operation:
    1. Launch "ble_peripheral" application by NuttShell
      $ ble_peripheral

    2 Connect "SONY_BLE" by PC or Andrlid (ex. "nRF Connect" as Android application)

    3. Write characteristic data by Android or PC application

    4. bt_spp application will output
              "onWrite [BLE] data[0] = 0x12, Length = 2"

    5. After 2 seconds ble_peripheral will notify "0x06 0x14" to characteristic.
