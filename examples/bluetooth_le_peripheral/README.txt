examples/bluetooth_le_peripheral
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

  This is a simple example of Bluetooth le peripheral role
  Available as nsh built-in command.
  This example turn on bluetooth and wait for BLE central to connect
  and execute data communication.

  Configuration in this example:

    CONFIG_EXAMPLES_BLUETOOTH_LE_PERIPHERAL - Enable this example

  Build(In <spresense>/sdk):
    $ ./tools/config.py examples/ble_central device/nrf52
    $ make

  Flash(In <spresense>/sdk):
    $ ./tools/flash.sh nuttx.spk

  Operation:
    1. Launch "ble_peripheral" application by NuttShell
      $ ble_peripheral

  Configuration in this example:

    CONFIG_EXAMPLES_BLUETOOTH_LE_PERIPHERAL - Enable this example

  Build(In <spresense>/sdk):
    $ ./tools/config.py examples/ble_peripheral device/nrf52
    $ make

  Flash(In <spresense>/sdk):
    $ ./tools/flash.sh nuttx.spk

  Operation:
    1. Launch "ble_peripheral" application by NuttShell
      $ ble_peripheral

    2. Then, the following BLE procedures are executed automatically.
       - Wait for BLE central to connect.
       - Wait for BLE central to write characteristic 10 times.
         (e.g. <spresense>/examples/bluetooth_le_central/ application executes this.)
       - Send notification to BLE central 10 times every one second.
       - Wait for BLE central to disconnect.

