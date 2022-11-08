#!/usr/bin/python
# coding: utf8 

import sys
import traceback
import os
sys.path.append(os.path.dirname(os.path.realpath(__file__)) + "/..")
import os.path
import select
import time
import math
import struct
from time import sleep
from ctypes import *
import wave
import serial



# Get arguments on the command line:
#   - COM port
#   - COM baudrate
#   - Output WAV file for mic #1
#   - Output WAV file for mic #2
#   - Output WAV file for mic #3
#   - Output WAV file for mic #4

if (len(sys.argv) != 7):
    print("Error: invalid number of arguments!")
    sys.exit(-1)

try: 
    com_port = sys.argv[1]
    print("COM port: " + com_port)
except:
    print("Error: invalid COM port!")
    sys.exit(-1)


try: 
    com_baudrate = int(sys.argv[2])
    print("Baudrate: " + str(com_baudrate))
except:
    print("Error: invalid COM baudrate!")
    sys.exit(-1)

try: 
    output_file1 = sys.argv[3]+".wav"
    print("Dumping mic #1 in: " + output_file1)
except:
    print("Error: invalid output file!")
    sys.exit(-1)

try: 
    output_file2 = sys.argv[4]+".wav"
    print("Dumping mic #2 in: " + output_file2)
except:
    print("Error: invalid output file!")
    sys.exit(-1)

try: 
    output_file3 = sys.argv[5]+".wav"
    print("Dumping mic #3 in: " + output_file3)
except:
    print("Error: invalid output file!")
    sys.exit(-1)

try: 
    output_file4 = sys.argv[6]+".wav"
    print("Dumping mic #4 in: " + output_file4)
except:
    print("Error: invalid output file!")
    sys.exit(-1)


if __name__ == "__main__":

    f_out1 = wave.open(output_file1, 'w')
    f_out1.setparams((1, 2, 16000, 0, 'NONE', 'not compressed'))

    f_out2 = wave.open(output_file2, 'w')
    f_out2.setparams((1, 2, 16000, 0, 'NONE', 'not compressed'))

    f_out3 = wave.open(output_file3, 'w')
    f_out3.setparams((1, 2, 16000, 0, 'NONE', 'not compressed'))

    f_out4 = wave.open(output_file4, 'w')
    f_out4.setparams((1, 2, 16000, 0, 'NONE', 'not compressed'))

    with serial.Serial(com_port, com_baudrate, timeout=1) as ser:
        
        sync = False
        last_sync_value = 255
        data = []
        nframe=0
        nbyte=0
        while True:
            data += ser.read(64)
            print("Serial: ")
            print("".join("{:02x} ".format(x) for x in data))
            
            out = False
            while not out and len(data) >= 4:
                if not sync:
                    if len(data) < 4:
                        data = []
                    elif data[0] == ord("a") and data[1] == ord("b") and data[2] == ord("c") and data[3] == ord("d"):
                        if len(data) <= 4:
                            out = True
                        else:
	                        sync = True
	                        print("Sync! " + str(last_sync_value))
	                        last_sync_value = data[4]
	                        data = data[5:]
                    else:
                        data = data[1:]
                else:
                    if len(data) < 5:
                        out = True
                    elif data[0] == ord("a") and data[1] == ord("b") and data[2] == ord("c") and data[3] == ord("d"):
                        if data[4] != (last_sync_value + 1) % 256:
                            print("Invalid sync value (" + str(data[4]) + " / " + str(last_sync_value) + ")!")
                            exit(-1)
                        last_sync_value = (last_sync_value + 1) % 256
                        print("Check Sync! " + str(last_sync_value))
                        nframe = nframe + 1
                        print("nframe: " + str(nframe))
                        print("nbyte: " + str(nbyte))
                        nbyte = 0
                        data = data[5:]                    
                    elif len(data) >= 2:
                            #print("Data: ")
                            c = bytearray()
                            #print("".join("{:02x} ".format(x) for x in data[0:2]))
                            c.append(data[0])
                            c.append(data[1])
                            nbyte = nbyte + 2
                            if nbyte <= 32 :
                                f_out1.writeframesraw(c)
                            elif nbyte <= 64 :
                                f_out2.writeframesraw(c)
                            elif nbyte <= 96 :
                                f_out3.writeframesraw(c)
                            elif nbyte <= 128 :
                                f_out4.writeframesraw(c)
                            # thow away other bytes for now
                            data = data[2:]

    print("Done!")
    
