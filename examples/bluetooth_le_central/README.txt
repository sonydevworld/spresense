examples/bluetooth_le_central
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

  This is a simple example of Bluetooth le central role
  Available as nsh built-in command.
  This example turn on bluetooth and try to connect peripheral device
  and discover database on such device

  Configuration in this example:

    CONFIG_EXAMPLES_BLUETOOTH_LE_CENTRAL - Enable this example

  Build(In <spresense>/sdk):
    $ ./tools/config.py examples/ble_central
    $ make buildkernel
    $ make

  Flash(In <spresense>/sdk):
    $ ./tools/flash.sh nuttx.spk

  Operation:
    1. Launch "ble_central" application by NuttShell
      $ ble_central

    2. It tries to scan and connect peripheral device named "SONY-CENTRAL"(e.g. "nRF Connect" as Android application to simulate the peripheral device)

    3. After connected to the peripheral device, "ble_central" application starts to discover the database on the peripheral device for services and characters

    4. ble_central application will output "on_db_discovery [BLE]" for result
