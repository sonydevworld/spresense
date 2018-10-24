__copyright__ = [
 'Copyright (C) 2018 Sony Semiconductor Solutions Corp.']
__license__ = 'GPL v2'
import time, sys, os, struct, glob, fnmatch, errno, telnetlib, argparse, shutil, subprocess, re, xmodem
import_serial_module = True
SDK_RELEASE = False
if SDK_RELEASE:
    PRINT_RAW_COMMAND = False
    REBOOT_AT_END = True
else:
    PRINT_RAW_COMMAND = True
    REBOOT_AT_END = True
try:
    import serial
except:
    import_serial_module = False

PROTOCOL_SERIAL = 0
PROTOCOL_TELNET = 1

class ConfigArgs:
    PROTOCOL_TYPE = None
    SERIAL_PORT = 'COM1'
    SERVER_PORT = 4569
    SERVER_IP = 'localhost'
    EOL = bytes([10])
    WAIT_RESET = True
    AUTO_RESET = False
    DTR_RESET = False
    XMODEM_BAUD = 0
    NO_SET_BOOTABLE = False
    PACKAGE_NAME = []
    FILE_NAME = []
    ERASE_NAME = []
    PKGSYS_NAME = []
    PKGAPP_NAME = []


ROM_MSG = [
 b'Welcome to nash']

class ConfigArgsLoader:

    def __init__(self):
        self.parser = argparse.ArgumentParser(formatter_class=argparse.RawTextHelpFormatter)
        self.parser.add_argument('package_name', help='the name of the package to install', nargs='*')
        self.parser.add_argument('-f', '--file', dest='file_name', help='save file', action='append')
        self.parser.add_argument('-e', '--erase', dest='erase_name', help='erase file', action='append')
        self.parser.add_argument('-S', '--sys', dest='pkgsys_name', help='the name of the system package to install', action='append')
        self.parser.add_argument('-A', '--app', dest='pkgapp_name', help='the name of the application package to install', action='append')
        self.parser.add_argument('-a', '--auto-reset', dest='auto_reset', action='store_true', default=None, help='try to auto reset develop board if possible')
        self.parser.add_argument('-d', '--dtr-reset', dest='dtr_reset', action='store_true', default=None, help='try to auto reset develop board if possible')
        self.parser.add_argument('-n', '--no-set-bootable', dest='no_set_bootable', action='store_true', default=None, help='not to set bootable')
        group = self.parser.add_argument_group()
        group.add_argument('-i', '--server-ip', dest='server_ip', help='the ip address connected to the telnet server')
        group.add_argument('-p', '--server-port', dest='server_port', type=int, help='the port connected to the telnet server')
        group = self.parser.add_argument_group()
        group.add_argument('-c', '--serial-port', dest='serial_port', help='the serial port')
        group.add_argument('-b', '--xmodem-baudrate', dest='xmodem_baud', help='Use the faster baudrate in xmodem')
        mutually_group = self.parser.add_mutually_exclusive_group()
        mutually_group.add_argument('-t', '--telnet-protocol', dest='telnet_protocol', action='store_true', default=None, help='use the telnet protocol for binary transmission')
        mutually_group.add_argument('-s', '--serial-protocol', dest='serial_protocol', action='store_true', default=None, help='use the serial port for binary transmission, default options')
        mutually_group2 = self.parser.add_mutually_exclusive_group()
        mutually_group2.add_argument('-F', '--force-wait-reset', dest='wait_reset', action='store_true', default=None, help='force wait for pressing RESET button')
        mutually_group2.add_argument('-N', '--no-wait-reset', dest='wait_reset', action='store_false', default=None, help='if possible, skip to wait for pressing RESET button')

    def update_config(self):
        args = self.parser.parse_args()
        ConfigArgs.PACKAGE_NAME = args.package_name
        ConfigArgs.FILE_NAME = args.file_name
        ConfigArgs.ERASE_NAME = args.erase_name
        ConfigArgs.PKGSYS_NAME = args.pkgsys_name
        ConfigArgs.PKGAPP_NAME = args.pkgapp_name
        if args.serial_protocol == True:
            ConfigArgs.PROTOCOL_TYPE = PROTOCOL_SERIAL
        else:
            if args.telnet_protocol == True:
                ConfigArgs.PROTOCOL_TYPE = PROTOCOL_TELNET
        if ConfigArgs.PROTOCOL_TYPE == None:
            proto = os.environ.get('SPRITZER_PROTOCOL')
            if proto is not None:
                if 's' in proto:
                    ConfigArgs.PROTOCOL_TYPE = PROTOCOL_SERIAL
                else:
                    if 't' in proto:
                        ConfigArgs.PROTOCOL_TYPE = PROTOCOL_TELNET
                if ConfigArgs.PROTOCOL_TYPE == None:
                    ConfigArgs.PROTOCOL_TYPE = PROTOCOL_SERIAL
        if ConfigArgs.PROTOCOL_TYPE == PROTOCOL_SERIAL:
            if args.serial_port is not None:
                ConfigArgs.SERIAL_PORT = args.serial_port
            else:
                port = os.environ.get('SPRITZER_PORT')
                if port is not None:
                    ConfigArgs.SERIAL_PORT = port
                else:
                    print('SPRITZER_PORT is not set, Use ' + ConfigArgs.SERIAL_PORT + '.')
        else:
            ConfigArgs.PROTOCOL_TYPE = PROTOCOL_TELNET
            if args.server_port is not None:
                ConfigArgs.SERVER_PORT = args.server_port
            else:
                port = os.environ.get('SPRITZER_TELNETSRV_PORT')
                if port is not None:
                    ConfigArgs.SERVER_PORT = port
                else:
                    print('SPRITZER_TELNETSRV_PORT is not set, Use ' + str(ConfigArgs.SERVER_PORT) + '.')
                if args.server_ip is not None:
                    ConfigArgs.SERVER_IP = args.server_ip
                else:
                    ip = os.environ.get('SPRITZER_TELNETSRV_IP')
                    if ip is not None:
                        ConfigArgs.SERVER_IP = ip
                    else:
                        print('SPRITZER_TELNETSRV_IP is not set, Use ' + ConfigArgs.SERVER_IP + '.')
                    if args.xmodem_baud is not None:
                        ConfigArgs.XMODEM_BAUD = args.xmodem_baud
                    if args.auto_reset is not None:
                        ConfigArgs.AUTO_RESET = args.auto_reset
                    if args.dtr_reset is not None:
                        ConfigArgs.DTR_RESET = args.dtr_reset
                    if args.no_set_bootable is not None:
                        ConfigArgs.NO_SET_BOOTABLE = args.no_set_bootable
                    if args.wait_reset is not None:
                        ConfigArgs.WAIT_RESET = args.wait_reset


class TelnetDev:

    def __init__(self):
        srv_ipaddr = ConfigArgs.SERVER_IP
        srv_port = ConfigArgs.SERVER_PORT
        self.recvbuf = b''
        try:
            self.telnet = telnetlib.Telnet(host=srv_ipaddr, port=srv_port, timeout=10)
            self.telnet.write(b'\xff')
        except Exception as e:
            print('Cannot connect to the server %s:%d' % (srv_ipaddr, srv_port))
            sys.exit(e.args[0])

    def readline(self, size=None):
        res = b''
        ch = b''
        while ch != ConfigArgs.EOL:
            ch = self.getc_raw(1, timeout=0.1)
            if ch == b'':
                return res
            res += ch

        return res

    def getc_raw(self, size, timeout=1):
        res = b''
        tm = time.monotonic()
        while size > 0:
            while self.recvbuf == b'':
                self.recvbuf = self.telnet.read_eager()
                if self.recvbuf == b'':
                    if time.monotonic() - tm > timeout:
                        return res
                    time.sleep(0.1)

            res += self.recvbuf[0:1]
            self.recvbuf = self.recvbuf[1:]
            size -= 1

        return res

    def write(self, buffer):
        self.telnet.write(buffer)

    def discard_inputs(self, timeout=1.0):
        while 1:
            ch = self.getc_raw(1, timeout=timeout)
            if ch == b'':
                break

    def getc(self, size, timeout=1):
        c = self.getc_raw(size, timeout)
        if PRINT_RAW_COMMAND:
            print('.', end='')
        sys.stdout.flush()
        return c

    def putc(self, buffer, timeout=1):
        self.telnet.write(buffer)

    def reboot(self):
        pass


class SerialDev:

    def __init__(self):
        if import_serial_module is False:
            print("Cannot import serial module, maybe it's not install yet.")
            print('\n', end='')
            print('Please install python-setuptool by Cygwin installer.')
            print('After that use easy_intall command to install serial module')
            print('    $ cd tool/')
            print('    $ python3 -m easy_install pyserial-2.7.tar.gz')
            quit()
        else:
            port = ConfigArgs.SERIAL_PORT
        try:
            self.serial = serial.Serial(port, baudrate=115200, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS, timeout=0.1)
        except Exception as e:
            print('Cannot open port : ' + port)
            sys.exit(e.args[0])

    def readline(self, size=None):
        return self.serial.readline(size)

    def write(self, buffer):
        self.serial.write(buffer)
        self.serial.flush()

    def discard_inputs(self, timeout=1.0):
        time.sleep(timeout)
        self.serial.flushInput()

    def getc(self, size, timeout=1):
        self.serial.timeout = timeout
        c = self.serial.read(size)
        self.serial.timeout = 0.1
        if PRINT_RAW_COMMAND:
            print('.', end='')
        sys.stdout.flush()
        return c

    def putc(self, buffer, timeout=1):
        self.serial.timeout = timeout
        self.serial.write(buffer)
        self.serial.flush()
        self.serial.timeout = 0.1

    def putc_win(self, buffer, timeout=1):
        self.serial.write(buffer)
        while self.serial.out_waiting == 0:
            break

    def setBaudrate(self, baudrate):
        self.serial.baudrate = baudrate

    def reboot(self):
        self.serial.setDTR(False)
        self.serial.setDTR(True)
        self.serial.setDTR(False)


class FlashWriter:

    def __init__(self, protocol_sel=PROTOCOL_SERIAL):
        if protocol_sel == PROTOCOL_TELNET:
            self.serial = TelnetDev()
        else:
            self.serial = SerialDev()

    def cancel_autoboot(self):
        boot_msg = ''
        self.serial.reboot()
        while boot_msg == '':
            rx = self.serial.readline().strip()
            self.serial.write(b'r')
            for msg in ROM_MSG:
                if msg in rx:
                    boot_msg = msg
                    break

        while 1:
            rx = self.serial.readline().decode(errors='replace').strip()
            if 'updater' in rx:
                self.serial.write(b'\n')
                self.serial.discard_inputs()
                return boot_msg.decode(errors='ignore')

    def recv(self):
        rx = self.serial.readline()
        if PRINT_RAW_COMMAND and rx.decode(errors='replace').strip() != '':
            print(rx.decode(errors='replace'), end='')
        return rx

    def wait(self, string):
        while 1:
            rx = self.recv()
            if string.encode() in rx:
                time.sleep(0.1)
                break

    def wait_for_prompt(self):
        prompt_pat = re.compile(b'updater')
        while 1:
            rx = self.recv()
            if prompt_pat.search(rx):
                time.sleep(0.1)
                break

    def send(self, string):
        self.serial.write(str(string).encode() + b'\n')
        rx = self.serial.readline()
        if PRINT_RAW_COMMAND:
            print(rx.decode(errors='replace'), end='')

    def read_output(self, prompt_text):
        output = []
        while 1:
            rx = self.serial.readline()
            if prompt_text.encode() in rx:
                time.sleep(0.1)
                break
            if rx != '':
                output.append(rx.decode(errors='ignore').rstrip())

        return output

    def install_files(self, files, command):
        if ConfigArgs.XMODEM_BAUD:
            command += ' -b ' + ConfigArgs.XMODEM_BAUD
        if os.name == 'nt':
            modem = xmodem.XMODEM(self.serial.getc, self.serial.putc_win, 'xmodem1k')
        else:
            modem = xmodem.XMODEM(self.serial.getc, self.serial.putc, 'xmodem1k')
        for file in files:
            with open(file, 'rb') as (bin):
                self.send(command)
                print('Install ' + file)
                self.wait('Waiting for XMODEM (CRC or 1K) transfer. Ctrl-X to cancel.')
                if ConfigArgs.XMODEM_BAUD:
                    self.serial.setBaudrate(ConfigArgs.XMODEM_BAUD)
                    self.serial.discard_inputs()
                modem.send(bin)
                if ConfigArgs.XMODEM_BAUD:
                    self.serial.setBaudrate(115200)
            self.wait_for_prompt()

    def save_files(self, files):
        if ConfigArgs.XMODEM_BAUD:
            command = 'save_file -b ' + ConfigArgs.XMODEM_BAUD + ' -x '
        else:
            command = 'save_file -x '
        if os.name == 'nt':
            modem = xmodem.XMODEM(self.serial.getc, self.serial.putc_win, 'xmodem1k')
        else:
            modem = xmodem.XMODEM(self.serial.getc, self.serial.putc, 'xmodem1k')
        for file in files:
            with open(file, 'rb') as (bin):
                self.send(command + os.path.basename(file))
                print('Save ' + file)
                self.wait('Waiting for XMODEM (CRC or 1K) transfer. Ctrl-X to cancel.')
                if ConfigArgs.XMODEM_BAUD:
                    self.serial.setBaudrate(ConfigArgs.XMODEM_BAUD)
                    self.serial.discard_inputs()
                modem.send(bin)
                if ConfigArgs.XMODEM_BAUD:
                    self.serial.setBaudrate(115200)
                self.wait_for_prompt()
                self.send('chmod d+rw ' + os.path.basename(file))
                self.wait_for_prompt()

    def delete_files(self, files):
        self.send('ls')
        lines = self.read_output('updater')
        for line in lines:
            binfile = line.strip().split(' ')[0]
            for file in files:
                if fnmatch.fnmatch(binfile, file):
                    self.delete_binary(binfile)

    def delete_binary(self, bin_name):
        self.send('rm ' + bin_name)
        self.wait_for_prompt()


def main():
    try:
        config_loader = ConfigArgsLoader()
        config_loader.update_config()
    except:
        return errno.EINVAL

    writer = FlashWriter(ConfigArgs.PROTOCOL_TYPE)
    do_wait_reset = True
    if ConfigArgs.AUTO_RESET and subprocess.call('cd ' + sys.path[0] + '; ./reset_board.sh', shell=True) == 0:
        print('auto reset board sucess!!')
        do_wait_reset = False
        bootrom_msg = writer.cancel_autoboot()
    if ConfigArgs.DTR_RESET:
        do_wait_reset = False
        bootrom_msg = writer.cancel_autoboot()
    if ConfigArgs.WAIT_RESET == False and do_wait_reset == True:
        rx = writer.recv()
        time.sleep(1)
        for i in range(3):
            writer.send('')
            rx = writer.recv()
            if ('updater').encode() in rx:
                do_wait_reset = False
                break
            time.sleep(1)

    if do_wait_reset:
        print('Please press RESET button on target board')
        sys.stdout.flush()
        bootrom_msg = writer.cancel_autoboot()
    if ConfigArgs.ERASE_NAME:
        print('>>> Remove exisiting files ...')
        writer.delete_files(ConfigArgs.ERASE_NAME)
    if ConfigArgs.PACKAGE_NAME or ConfigArgs.PKGSYS_NAME or ConfigArgs.PKGAPP_NAME:
        print('>>> Install files ...')
    if ConfigArgs.PACKAGE_NAME:
        writer.install_files(ConfigArgs.PACKAGE_NAME, 'install')
    if ConfigArgs.PKGSYS_NAME:
        writer.install_files(ConfigArgs.PKGSYS_NAME, 'install -k sys.key')
    if ConfigArgs.PKGAPP_NAME:
        writer.install_files(ConfigArgs.PKGAPP_NAME, 'install -k app.key')
    if ConfigArgs.FILE_NAME:
        print('>>> Save files ...')
        writer.save_files(ConfigArgs.FILE_NAME)
    if not ConfigArgs.NO_SET_BOOTABLE:
        print('>>> Save Configuration to FlashROM ...')
        writer.send('set bootable M0P')
        writer.wait_for_prompt()
    writer.send('sync')
    writer.wait_for_prompt()
    if REBOOT_AT_END:
        print('Restarting the board ...')
        writer.send('reboot')
    return 0


if __name__ == '__main__':
    pass
try:
    sys.exit(main())
except KeyboardInterrupt:
    print('Canceled by keyboard interrupt.')
