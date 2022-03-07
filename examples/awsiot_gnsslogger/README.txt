examples/awsiot_gnsslogger
^^^^^^^^^^^^^^^^^^^^^^^^^^

  This sample code is an example application of GNSS logger with publishing
  location to AWS IoTCore.

  [Board environment]

    This sample assumes to work with below parts.

    - Spresense Main Board
    - Connectivity board, "IDY iS110B Wi-Fi Add-on board" or "Spresense LTE Board".

  [Overview of this application]

    This application is a GNSS logger and measured location information is
    published to AWS IoT. The connectivity is not cared in this application.
    Therefore, network daemon like "lte_daemon" or "gs2200m" is needed to set up
    before executing this application.

  [Save needed files into Spresense SPI-flash]

    When you make a "Thing" on AWS IoT Core console, you can get a device
    certification file, a device private key file and root CA of AWS.

    These 3 files are needed to connect the AWS.

    Assuming that each file name is as follows:
      Device certification file : cert.pem
      Device private key file : privkey.pem
      AWS root CA : rootCA.pem

    Install those files as following command.

      $ cd sdk
      $ ./tools/flash.sh -c /dev/ttyUSB0 -w [path to cert.pem] [path to privkey.pem] [path to rootCA.pem]

    You also need to use the name of the thing that you decided when you created
    the "thing" as the "Client ID" in your app.  Furthermore, you need to pass
    the Endpoint of the AWS connection to the app.

    In order to specify these settings to the app, you need to write an ini
    file and save it on SPI-Flash with the file name /mnt/spif/aws_iot.ini.

    A sample of aws_iot.ini file is stored in
    examples/awsiot_gnsslogger/spif/aws_iot.ini.
    Please follow the comments inside and rewrite your settings accordingly.

    After rewriting the aws_iot.ini file to suit your configuration, save the
    file on SPI-Flash.

    $ cd sdk
    $ ./tools/flash.sh -c /dev/ttyUSB0 -w ../examples/awsiot_gnsslogger/spif/aws_iot.ini


  [How to build]

    Configure for this example by using default config. This config is using
    Wi-Fi board.

      $ cd sdk
      $ ./tools/config.py examples/wifi_awsiot_gnsslogger

    After finishing configuration, type make to start build in sdk/ directory.

      $ make

    nuttx.spk is created in sdk/ directory if it succeeded.
    Then flash it into the device.
    Below example is a case of ubuntu CLI command to flash.
    In below example, USB connection with Spresense main board is /dev/ttyUSB0.

      $ ./tools/flash.sh -c /dev/ttyUSB0 nuttx.spk
      >>> Install files ...
      install -b 115200
      Install nuttx.spk
      |0%-----------------------------50%------------------------------100%|
      ######################################################################

      xxxxx bytes loaded.
      Package validation is OK.
      Saving package to "nuttx"
      updater# sync
      updater# Restarting the board ...
      reboot

  [How to work]

      At first, connect to a Wi-Fi AP with internet connection.
      For example, if there is a AP with "wifi_net" as SSID and "abcdefg" as
      the password, type as below to connect the board to Wi-Fi.

        nsh> gs2200m wifi_net abcdefg &

      It takes a few seconds.
      After successful of Wi-Fi connection, you can check your own IP address 
      with the following command.

        nsh> ifconfig
        eth0    Link encap:Ethernet HWaddr 3c:95:09:00:84:2a at UP
                inet addr:192.168.11.1 DRaddr:192.168.11.1 Mask:255.255.255.0

      Then, execute this application.

        nsh> awsiot_gnsslogger
        == Configuration ==
            Host URL        = [host url which is set in ini file is shown]
            Host PORT       = 8883
            Root CA Path    = /mnt/spif/RootCA1.pem
            Cert Path       = /mnt/spif/pub1crt.pem
            PrivKEY Path    = /mnt/spif/pub1key.pem
            MQTT Client ID  = TestPublisher1
            MQTT CMD TO     = 20000 (ms)
            MQTT SSL TO     = 5000 (ms)
            MQTT KEEP ALIVE = 600 (sec)
        Time out signal wait.
        Location is NOT fixed. 0 satellites is captured.
        Location is NOT fixed. 3 satellites is captured.
        Location is NOT fixed. 3 satellites is captured.
        Location is NOT fixed. 3 satellites is captured.
        ....

      To capture GNSS satellites, take the device outside or by a window.

      After fix the measured location, the application will set UTC time and
      date which is measured by GNSS as a system time on the board. Then it
      connects to AWS IoT server.
      And after established the connectionm the location latitude and longitude
      is published as a topic on "data/device/gps" with below message.

        Location is fixed. 4 satellites is captured.
            Publish data to data/device/gps topic as <<{ "device_loc": { "lat": xxx.xxxxx, "lng": xxx.xxxxxx } }>>

      If you subscribe the topic "data/device/gps" in AWS IoTCore, you can see
      the published data.

