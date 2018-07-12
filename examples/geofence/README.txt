
examples/geofence
^^^^^^^^^^^^^^^^^^

This sample shows the use of geofencing.
After positioning the device position, set the region based on that position.
When you move the device, the state transition of gofence is displayed.

configuration:

[System Type]
  CXD56xx Peripheral Support -->
    [*] GNSS device
    [*]   Geofence Support

[Application Configuration]
  Examples -->
    [*] Geofence transitions example

Output example:

nsh>geofence
geofence [3:100]
nsh> Hello, GEOFENCE SAMPLE!!
Open /dev/gnss
## No Positioning Data
UTC time : Hour:0, minute:0, sec:4
## No Positioning Data
UTC time : Hour:0, minute:0, sec:5
## No Positioning Data
UTC time : Hour:0, minute:0, sec:6
## No Positioning Data
UTC time : Hour:0, minute:0, sec:7
## No Positioning Data
UTC time : Hour:0, minute:0, sec:8

~~~~
## No Positioning Data
UTC time : Hour:8, minute:23, sec:25
## POSITION: LAT 35431825, LNG 139370870
Open /dev/geofence
Add Region
  ID0:LAN 35431825, LNG 139370870, RAD 50
  ID1:LAN 35432825, LNG 139370870, RAD 50
  ID2:LAN 35430825, LNG 139370870, RAD 50
  ID3:LAN 35431825, LNG 139371870, RAD 50
  ID4:LAN 35431825, LNG 139369870, RAD 50
Used ID : 0x0000001f
Start Geofencing ...
[GEO] Updated region:5
      ID:0, Statu:ENTER
      ID:1, Statu:EXIT
      ID:2, Statu:EXIT
      ID:3, Statu:EXIT
      ID:4, Statu:EXIT
[GNSS] POS: LAT 35431825, LNG 139370870
[GEO] Updated region:1
      ID:0, Statu:EXIT
[GNSS] POS: LAT 35432237, LNG 139370435
[GEO] Updated region:1
      ID:1, Statu:ENTER
[GNSS] POS: LAT 35432644, LNG 139370428
[GEO] Updated region:1
      ID:1, Statu:DWELL
[GNSS] POS: LAT 35432681, LNG 139370437
[GEO] Updated region:1
      ID:1, Statu:EXIT
[GNSS] POS: LAT 35432672, LNG 139370235
[GEO] Updated region:1
      ID:1, Statu:ENTER
[GNSS] POS: LAT 35432639, LNG 139370499
[GEO] Updated region:1
      ID:1, Statu:EXIT
[GNSS] POS: LAT 35433066, LNG 139371531
[GEO] Updated region:1
      ID:1, Statu:ENTER
[GNSS] POS: LAT 35432852, LNG 139371352
[GEO] Updated region:1
      ID:1, Statu:EXIT
[GNSS] POS: LAT 35432374, LNG 139370534
[GEO] Updated region:1
      ID:4, Statu:ENTER
[GNSS] POS: LAT 35432134, LNG 139370169
[GEO] Updated region:1
      ID:4, Statu:EXIT
[GNSS] POS: LAT 35432246, LNG 139370406
End of GEOFENCE Sample:0


Notes:
This sample will terminate if 10 states change is geofence transition detected.
If the device does not move, state change does not occur and the sample may not be terminated.

