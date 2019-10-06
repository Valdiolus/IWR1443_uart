import time
import serial
from time import sleep
import numpy as np
import cv2
from sys import argv
from array import array
import struct

np.set_printoptions(linewidth=np.inf)
INIT_FILE = "iwr1443_init_log_only_range"

#RS232Tx/RX is for config port
#AR_mss_logger is for data port

RPI = 1

def find_sublist(sub, bigger):
    if not bigger:
        return -1
    if not sub:
        return 0
    first, rest = sub[0], sub[1:]
    pos = 0
    try:
        while True:
            pos = bigger.index(first, pos) + 1
            if not rest or bigger[pos:pos+len(rest)] == rest:
                return pos
    except ValueError:
        return -1

def TTY_INIT(tty, speed):
    ser = serial.Serial()
    ser.port = tty #"/dev/ttyUSB0" #ttyACM0"
    ser.baudrate = speed
    ser.NewLine = "\n"
    ser.open()
    return ser

def Radar_INIT(serial):
    log = open(INIT_FILE, "r")
    while True:
        line = log.readline()
        lenght = len(line)
        if(lenght>9):
            if line[10] is not '%' and line[0:10] == 'mmwDemo:/>':
                serial.write(line[10:lenght].encode("utf-8")+b'\n')
                t1 = time.perf_counter()
                while True:
                    res = serial.readline()
                    print(res)
                    if res == b'Done\n':
                        break
                    sleep(0.05)
                    #break
                    if time.perf_counter() > t1 + 0.3:  # wait 1 sec
                        serial.write(line[10:lenght].encode("utf-8") + b'\n') #repeat sending
                        t1 = time.perf_counter()

        if line[0] == 'q':
            print("END OF FILE")
            break
    log.close()

def Radar_START(serial):
    sleep(0.1)
    serial.write('sensorStart'.encode("utf-8") + b'\n')
    #while True:
    #    res = serial.readline()
    #    print(res)
    #    if res == b'Done\n':
    #        break
    #    sleep(0.05)
    print("RADAR STARTED")

def Radar_STOP(serial):
    sleep(0.1)
    serial.write('sensorStop'.encode("utf-8") + b'\n')
    #while True:
    #    res = serial.readline()
    #    print(res)
    #    if res == b'Done\n':
    #        break
    #    sleep(0.05)
    print("RADAR STOPPED")

def From_str_to_32(str, len, Nfr, buff):
    Nfr_int = 0
    cmpl_buff = 0
    if Nfr > 0:
        if Nfr+len<32:
            buff[Nfr:Nfr+len] = str[0:len]
            Nfr = Nfr + len
            #print("PAR:", list(buff[Nfr:Nfr+len]))
            return Nfr, buff, cmpl_buff
        else:
            buff[Nfr:] = str[0:32-Nfr]
            print("FIN:", list(buff))
            Nfr_int = 32-Nfr
            cmpl_buff = 1
            return Nfr, buff, cmpl_buff

    for i in range(Nfr_int, len, 32):
            if len - i - 32 < 0:
                buff[0:len - i] = str[i:len]
                Nfr = len - i
                #print("PER:", list(buff[0:len - i]))
            else:
                print("ALL:", str[i:i + 32])
                Nfr = 0
                cmpl_buff = 1
                return Nfr, buff, cmpl_buff
    return Nfr, buff, cmpl_buff

def Radar_window(frame_int):
    #frame[,,:] = (24, 27, 68)
    cv2.line(frame_int, (320, 0), (320, 470), (0, 0, 255), 1) #(24, 27, 68)
    cv2.line(frame_int, (0, 470), (640, 470), (0, 0, 255), 1)  # (24, 27, 68)
    #cv2.line(frame, (zone_int[4], zone_int[5]), (zone_int[6], zone_int[7]), (0, 255, 0), 2)
    # show the output frame
    #frame_int = cv2.resize(frame_int, None, fx=1.5, fy=1.5, interpolation=cv2.INTER_CUBIC)  # x2 image
    cv2.imshow("Frame", frame_int)

def Radar_add_points(frame_int, buffer):
    cv2.circle(frame_int,(320 + buffer[3]*20, 470 - buffer[4]*20), 2, (0,255,0))
    return frame_int

def Radar_READ_DATA(serial):
    t1 = time.perf_counter()
    MW = [2, 1, 4, 3, 6, 5, 8, 7]
    HEAD_NEED = [28, 40, 44, 46]#object numb, payload len, detected obj num, xyz format
    MAIN_HEAD = 48
    DATA_HEAD = list(np.zeros(4))
    DATA_MAIN = np.zeros((6), dtype='int16')
    pb = 0
    hb = 0 #number of beginning of the header in the packet
    byte_current = 0
    object_num = 0
    payload_len = 0
    det_obj_num = 0
    xyz_format = 0
    data_stage = 0
    frame = np.zeros((480, 640, 3), np.uint8)
    while time.perf_counter() < t1+6000: #wait 5 sec
        res = list(serial.readline())
        lenght = len(res)
        #print("Current:", lenght)
        #print("FRM:", res)
        hb = find_sublist(MW, res) - 1 #find place of array
        pb = 0
        while pb < lenght:
            if pb == hb and hb >= 0:
                #Nbytes_await = res[hb + 12]
                byte_current = 0  # set to beginning of the header - new packet
                data_stage = 0  # new packet with new header
                Radar_window(frame) #show radar data on the screen
                frame = np.zeros((480, 640, 3), np.uint8)

            if data_stage < 4:
                if byte_current == HEAD_NEED[data_stage]:
                    DATA_HEAD[data_stage] = res[pb]
                    data_stage += 1
                    if data_stage == 4:
                        print("Payload lenght:", DATA_HEAD)
            else:
                if byte_current < 44 + DATA_HEAD[1] and data_stage > 4: #header + payload
                    if byte_current % 2 > 0: #with remainder - *256 (High byte)
                        DATA_MAIN[data_stage - 5] = (DATA_MAIN[data_stage - 5] + res[pb] * 256)
                        data_stage += 1
                    else:
                        DATA_MAIN[data_stage - 5] = res[pb]

                    if data_stage > 10:# end of part of the payload
                        data_stage = 5
                        DATA_MAIN[3] /=  (256)
                        DATA_MAIN[4] /=  (256)
                        print("New Payload:", DATA_MAIN)
                        frame = Radar_add_points(frame, DATA_MAIN)
                else:
                    data_stage += 1

            pb += 1 # check new byte of current packet
            byte_current += 1 # new byte of new header + payload

        if (cv2.waitKey(1) & 0xFF) == ord("q"):
            break
            # Data from last line
    print("Time is passed")

def Main():

    ser = TTY_INIT("/dev/ttyUSB0", 115200)

    Radar_INIT(ser)
    Radar_START(ser)
    
    if RPI == 1:
        ser = TTY_INIT("/dev/ttyAMA0", 921600)
    else:
        ser = TTY_INIT("/dev/ttyUSB0", 921600)

    Radar_READ_DATA(ser)

    ser = TTY_INIT("/dev/ttyUSB0", 115200)

    Radar_STOP(ser)

Main()
quit()

#add to the end of config file if only dynamic points needed
#mmwDemo:/>clutterRemoval 1
#Done
