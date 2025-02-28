#!/usr/bin/python
# -*- coding: utf-8 -*-

import argparse
import math
import sys
import os
import platform
import copy
import csv
import numpy as np
import glob
import struct
import matplotlib.pyplot as plt

EARTH_RATE = math.radians(360.0/86164.098903691) # [rad/s]
RAD2DEG = 180/np.pi
RAD2DPH = RAD2DEG*3600

################################################################

def float16_hex_to_double(hex_value):
    # Convert the hexadecimal representation to binary data
    binary_data = bytes.fromhex(hex_value)
    # Interpret the binary data as a single precision (32-bit floating point number)
    float32 = struct.unpack('>f', binary_data)[0] # Big-endian
    # Convert to double precision
    double_value = float(float32)
    return double_value

def read_imu_file_hex(input_file):

    f = open(input_file)
    csv_reader = csv.reader(f)

    gx_list = []
    gy_list = []
    gz_list = []
    ax_list = []
    ay_list = []
    az_list = []
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
    f.close()

    ave_gx = sum(gx_list) / len(gx_list)
    ave_gy = sum(gy_list) / len(gy_list)
    ave_gz = sum(gz_list) / len(gz_list)
    ave_ax = sum(ax_list) / len(ax_list)
    ave_ay = sum(ay_list) / len(ay_list)
    ave_az = sum(az_list) / len(az_list)

    ave_acc = np.array([ave_ax, ave_ay, ave_az])
    ave_gyr = np.array([ave_gx, ave_gy, ave_gz])

    return ave_acc, ave_gyr


def read_imu_files(input_files):
    
    ave_acc_list = []
    ave_gyr_list = []
    for input_file in input_files:
        (ave_acc, ave_gyr) = read_imu_file_hex(input_file)
        ave_acc_list.append(ave_acc)
        ave_gyr_list.append(ave_gyr)
    
    return ave_acc_list, ave_gyr_list

def acc_to_euler(init_a, Rz):
    # A = R*a: Global <= Local
    # R = RxRyRz: Rz => Ry => Rx
    
    norm_a =  np.linalg.norm(init_a) # if there is gravity, norm_a shold be 1.0 [G]
    ax = init_a[0] / norm_a # [G]
    ay = init_a[1] / norm_a
    az = init_a[2] / norm_a
    
    Rx =  math.atan2(ay, az)
    Ry = -math.asin(ax)
    
    return Rx, Ry, Rz

def correct_bias(param, imu):
    imu = np.array(imu)
    bias = np.array(param)
    imu_corrected = imu - bias
    return imu_corrected


def CircleFitting(ave_gyr_list):
    temp = np.array(ave_gyr_list).T
    x = temp[0]
    y = temp[1]
    valid = False

    sumx  = sum(x)
    sumy  = sum(y)
    sumx2 = sum([ix ** 2 for ix in x])
    sumy2 = sum([iy ** 2 for iy in y])
    sumxy = sum([ix * iy for (ix,iy) in zip(x,y)])

    F = np.array([[sumx2,sumxy,sumx],
                  [sumxy,sumy2,sumy],
                  [sumx,sumy,len(x)]])

    G = np.array([[-sum([ix ** 3 + ix*iy **2 for (ix,iy) in zip(x,y)])],
                  [-sum([ix ** 2 *iy + iy **3 for (ix,iy) in zip(x,y)])],
                  [-sum([ix ** 2 + iy **2 for (ix,iy) in zip(x,y)])]])

    if np.linalg.det(F) > 0.0:
        T = np.linalg.inv(F).dot(G)

        cxe = float(T[0][0] / -2)
        cye = float(T[1][0] / -2)
        sqdiff = cxe**2+cye**2-T[2][0]
        
        if sqdiff > 0.0:
            re = math.sqrt(sqdiff)
            valid = True
        else:
            re = copy.deepcopy(EARTH_RATE)
    else:
        cxe = 0.0
        cye = 0.0
        re = copy.deepcopy(EARTH_RATE)

    cze = np.mean(np.array(ave_gyr_list).T[2])
    return np.array([cxe,cye,cze,re,valid])

def calc_direction(ave_acc_list, ave_gyr_list):
    # ave_acc_list NOT used in this version

    #--- estimate bias ---#
    result_circle = CircleFitting(ave_gyr_list)
    gyro_bias = result_circle[:3]
    valid = result_circle[4]
    direction_list = []

    if valid == False:
        print("**** Error !!  Invalid imu data.  Maybe too noisy or too narrow attitude coverage.****")
        sys.exit()

    #--- correct imu data ---#
    for ave_gyr in ave_gyr_list:
        #--- correcte bias ---#
        ave_gyr_corrected = correct_bias(gyro_bias, ave_gyr)

        #--- estimate Rz ---#
        (gx, gy, gz) = ave_gyr_corrected

        Rz = math.atan2(-gx,gy) # 0 as y-axis = north. plus as clock-wise
        obs_direction = math.degrees(Rz)
        if obs_direction < 0.0:
            obs_direction += 360.0

        direction_list.append(obs_direction)

    return direction_list

def get_direction_word(direction):
    DIRECTION_UNIT = 11.25 # deg. 32 div
    DIRECTION_UNIT_2 = 2*DIRECTION_UNIT 
    WORD_TABLE = ["北", "北北東", "北東", "東北東", "東", "東南東", "南東", "南南東", "南", "南南西", "南西", "西南西", "西", "西北西", "北西", "北北西"]

    word_addr = int( (direction + DIRECTION_UNIT) / DIRECTION_UNIT_2 )

    word = WORD_TABLE[word_addr]

    return word   

def open_result_file(infiles, input_folder):
    #--- define delimitor ---#
    '''
    pf = platform.system()
    if pf == "Windows":
        print("Platform: Windows")
    elif pf == "Darwin":
        print("Platform: MacOS")
    elif pf == "Linux":
        print('Platform: Linux')
    '''

    resultfilename = input_folder + os.sep + 'gyrocompass_result.csv'

    #--- open result file ---#
    fresult = open(resultfilename, 'w')
    header = '入力ファイル, 方位値 [deg], 方角\n'
    fresult.write(header)

    return fresult, resultfilename

def write_result(infiles, direction_list, input_folder):
    fresult, fname = open_result_file(infiles, input_folder)

    print("\nCalculating compass bearing....")

    for (input_file, obs_direction) in zip(infiles, direction_list):
        direction_word = get_direction_word(obs_direction)
        outstr = input_file + "," + str(obs_direction) + "," + str(direction_word) + "\n"
        print("     " + outstr, end="")
        fresult.write(outstr)

    fresult.close()

    print("\nWrite this result in " + fname)


################################################################

if __name__ == "__main__":
    if len(sys.argv) != 2:
      print("Usage $ python3 " + sys.argv[0] + " <data directory>")
      exit()
    elif not os.path.isdir(sys.argv[1]):
      print(sys.argv[1] + " is not directory")
      exit()

    infiles = glob.glob(sys.argv[1] + os.sep + "*.txt")
    print("Reading files: ")
    for i in range(len(infiles)):
        print("  " + infiles[i])
    if len(infiles) < 3:
        print("**** Error !!   Too few data. At least 3 datas required. ****")
        sys.exit()

    (ave_acc_list, ave_gyr_list) = read_imu_files(infiles)

    direction_list = calc_direction(ave_acc_list, ave_gyr_list)
 
    write_result(infiles, direction_list, sys.argv[1])

