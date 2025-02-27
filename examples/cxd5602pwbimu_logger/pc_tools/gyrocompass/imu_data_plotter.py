#!/usr/bin/python
# -*- coding: utf-8 -*-

import argparse
import math
import sys
import os
import platform
import csv
import numpy as np
import glob
import struct

import matplotlib.pyplot as plt

################################################################

def float16_hex_to_double(hex_value):
    # Convert the hexadecimal representation to binary data
    binary_data = bytes.fromhex(hex_value)
    # Interpret the binary data as a single precision (32-bit floating point number)
    float32 = struct.unpack('>f', binary_data)[0] # Big-endian
    # Convert to double precision
    double_value = float(float32)
    return double_value

def read_imu_file_hex(infile):

    f = open(infile)
    csv_reader = csv.reader(f)

    gx_list = []
    gy_list = []
    gz_list = []
    ax_list = []
    ay_list = []
    az_list = []
    standerd_gravity = 9.80665
    count = 0
    for row in csv_reader:

        try:
            gx_list.append(float16_hex_to_double(row[2])) # [rad/s]
            gy_list.append(float16_hex_to_double(row[3])) # [rad/s]
            gz_list.append(float16_hex_to_double(row[4])) # [rad/s]
            ax_list.append(float16_hex_to_double(row[5])) # [m/s/s]
            ay_list.append(float16_hex_to_double(row[6])) # [m/s/s]
            az_list.append(float16_hex_to_double(row[7])) # [m/s/s]
        except:
            print("Invalid data: ", row)
        count += 1
    f.close()

    plt.suptitle(infile)
    plt.subplot(2,1,1)
    plt.plot(range(len(gx_list)), np.degrees(np.array(gx_list)), color="blue",  label="gx")
    plt.plot(range(len(gy_list)), np.degrees(np.array(gy_list)), color="red",   label="gy")
    plt.plot(range(len(gz_list)), np.degrees(np.array(gz_list)), color="green", label="gz")
    plt.grid(True)
    plt.legend(loc='upper right')
    plt.xlabel("sample")
    plt.ylabel("gyr [dps]")

    plt.subplot(2,1,2)
    plt.plot(range(len(ax_list)), np.array(ax_list)/standerd_gravity, color="blue",   label="ax")
    plt.plot(range(len(ay_list)), np.array(ay_list)/standerd_gravity, color="red",    label="ay")
    plt.plot(range(len(az_list)), np.array(az_list)/standerd_gravity, color="green",  label="az")
    plt.grid(True)
    plt.legend(loc='upper right')
    plt.xlabel("sample")
    plt.ylabel("acc [G]")

    out_file = infile.rsplit(".",1)[0] + ".png"
    plt.savefig(out_file)
    # plt.show()
    plt.close()

    print("Plot:", infile, "=>", out_file)


################################################################

if __name__ == "__main__":

    if len(sys.argv) != 2:
      print("Usage $ python3 " + sys.argv[0] + " <data directory>")
      exit()
    elif not os.path.isdir(sys.argv[1]):
      print(sys.argv[1] + " is not directory")
      exit()

    print("Plot IMU log data...")

    infiles = glob.glob(sys.argv[1] + os.sep + "*.txt")
    for infile in infiles:
        read_imu_file_hex(infile)

