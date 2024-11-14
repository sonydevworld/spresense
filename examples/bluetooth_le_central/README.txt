examples/bluetooth_le_central
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

  This is a simple example of Bluetooth le central role
  Available as nsh built-in command.
  This example turn on bluetooth and try to connect peripheral device
  and discover database on such device

  Configuration in this example:

    CONFIG_EXAMPLES_BLUETOOTH_LE_CENTRAL - Enable this example

  Build(In <spresense>/sdk):
    $ ./tools/config.py examples/ble_central device/nrf52
    $ make

  Flash(In <spresense>/sdk):
    $ ./tools/flash.sh nuttx.spk

  Operation:
    1. Launch "ble_central" application by NuttShell
      $ ble_central

      If there is only one argument, you are able to specify a device name.
      $ ble_central <Device name>
      <Device name> is a string.

      If there are two arguments, you are able to specify service and characteristic UUIDs.
      $ ble_central <Service UUID> <Characteristic UUID>
      <UUID> is a hexadecimal string without "0x".
       e.g.) 1812, 12345678-90AB-CDEF-1234-567890ABCDEF

    2. Then, the following BLE procedures are executed automatically.
       - scan and connect to peripheral device named "SONY-PERIPHERAL"
         (e.g. <spresense>/examples/bluetooth_le_peripheral/ application.)
       - Discover the database on connected peripheral device,
         and store the data that has the property "notify,read,write".
       - read/write descriptors
       - read/write characteristics 10 times
       - receive notification 10 times
       - Disconnect from connected BLE peripheral device.

