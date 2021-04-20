This configuration contains required options to use vtun vpn application.
It provides "vtun" NSH command to connect VTUN server.

Usage

 Host side:
   $ sudo vtund -s -f <VTUN server conf file> -n
 Spresense side:
   nsh> vtun -f <VTUN client conf file> vpn <Server IP address> -n &
   nsh> ifconfig tun0 <Local IP address such as 192.168.12.11>
   nsh> ifup tun0

Limitation

  This feature can be work for GS2200m wifi and WIZnet W5x00 ethernet board.

[Source path]
externals/vtun
