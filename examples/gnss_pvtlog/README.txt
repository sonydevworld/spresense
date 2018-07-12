examplex/gnss_pvtlog
^^^^^^^^^^^^^^^^^^^

This sample shows the use of PVTLOG.
Execute the following 1, 2, 3 by specifying argument.
1, 'w', 'W' or no argument.
After positioning the device position, Pvt log stored in GPS receiver.
It is notified when the number of logs exceeds the set threshold.
Read the cached log using the read().
The notified log is saved in the file.
The path to save the file can be changed by configuration.
2, 'r', 'R'.
Read the saved file and output it to the console.
3, 'd', 'D'.
Delete the saved file.
and 'a', 'A'
Execute the above 1 to 3 at once.

configuration:

[System Type]
  CXD56xx Peripheral Support -->
    [*] GNSS device

[Application Configuration]
  Examples -->
    [*] GNSS PVTLOG example
    (/mnt/spif/PVTLOG) path to save the log file

Output example:

nsh> gnss_pvtlog w
gnss_pvtlog [5:100]
Hello, GNSS_PVTLOG!!
gnss_pvtlog_write()
Set GNSS signal 
Set PVTLOG signal 
Delete Log 
Start Log 
Start Positioning 
No Positioning...
No Positioning...

....

 Y=2017, M= 3, d=24 h= 4, m=52, s= 1 m=796, Lat 35:25:9109 , Lon 139:22:1334, Log No:1 
 Y=2017, M= 3, d=24 h= 4, m=52, s= 2 m=  0, Lat 35:25:9109 , Lon 139:22:1334, Log No:1 
 Y=2017, M= 3, d=24 h= 4, m=52, s= 3 m=  0, Lat 35:25:9125 , Lon 139:22:1324, Log No:2 
 Y=2017, M= 3, d=24 h= 4, m=52, s= 4 m=  0, Lat 35:25:9109 , Lon 139:22:1302, Log No:3 

....

 Y=2017, M= 3, d=24 h= 5, m= 0, s= 8 m=  0, Lat 35:25:8903 , Lon 139:22:1163, Log No:61 
 Y=2017, M= 3, d=24 h= 5, m= 0, s= 9 m=  0, Lat 35:25:8904 , Lon 139:22:1162, Log No:62 
 Y=2017, M= 3, d=24 h= 5, m= 0, s=10 m=  0, Lat 35:25:8902 , Lon 139:22:1157, Log No:63 
Stop Log 
/mnt/spif/file6.dat write OK
[END] GNSS_PVTLOG Sample 
...

nsh> gnss_pvtlog r
gnss_pvtlog_read()
/mnt/spif/file1.dat read OK(85 line)
 Y=2017, M= 3, d=24 h= 4, m=52, s= 1 m= 28, Lat 35:25:9109 , Lon 139:22:1334, Log No:1 
 Y=2017, M= 3, d=24 h= 4, m=52, s= 3 m=  0, Lat 35:25:9125 , Lon 139:22:1324, Log No:2 
 Y=2017, M= 3, d=24 h= 4, m=52, s= 4 m=  0, Lat 35:25:9109 , Lon 139:22:1302, Log No:3 
 Y=2017, M= 3, d=24 h= 4, m=52, s= 5 m= 21, Lat 35:25:9119 , Lon 139:22:1298, Log No:4 
 Y=2017, M= 3, d=24 h= 4, m=52, s= 6 m=  0, Lat 35:25:9116 , Lon 139:22:1296, Log No:5 

....

 Y=2017, M= 3, d=24 h= 5, m= 0, s= 5 m=  0, Lat 35:25:8911 , Lon 139:22:1166, Log No:58 
 Y=2017, M= 3, d=24 h= 5, m= 0, s= 6 m=  0, Lat 35:25:8909 , Lon 139:22:1165, Log No:59 
 Y=2017, M= 3, d=24 h= 5, m= 0, s= 7 m=  0, Lat 35:25:8906 , Lon 139:22:1165, Log No:60 
 Y=2017, M= 3, d=24 h= 5, m= 0, s= 8 m=  0, Lat 35:25:8903 , Lon 139:22:1163, Log No:61 
 Y=2017, M= 3, d=24 h= 5, m= 0, s= 9 m=  0, Lat 35:25:8904 , Lon 139:22:1162, Log No:62 
 Y=2017, M= 3, d=24 h= 5, m= 0, s=10 m=  0, Lat 35:25:8902 , Lon 139:22:1157, Log No:63 
done.

nsh> gnss_pvtlog d
gnss_pvtlog_delete()
/mnt/spif/file1.dat delete ok
/mnt/spif/file2.dat delete ok
/mnt/spif/file3.dat delete ok
/mnt/spif/file4.dat delete ok
/mnt/spif/file5.dat delete ok
/mnt/spif/file6.dat delete ok

