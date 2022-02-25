"""
Seeed Groove IMU 9DOF ICM20600 & AK09918 
"""
import sys
import os
import time
import machine
import math

#help(machine)

#sda_pin=machine.Pin(0) #Alternative SDA RP2
#scl_pin=machine.Pin(1) #Alternative SCL RP2
#i2c = machine.SoftI2C(sda=sda_pin, scl=scl_pin)

i2c = machine.I2C(0) # default hardware I2C on GP8 and GP9

def discover_devices():
    devices = i2c.scan()
    if len(devices) == 0:
        print("No i2c device found")
    else:
        print(len(devices),"devices found")
        for dev in devices:
            print("Address: " ,(dev, hex(dev)))
        print('\n')

discover_devices()

print("Machine Id: ", machine.unique_id())

class Icm20600:
    ICM20600_ADDR = 0x69

    #register too mach for me
    XG_OFFS_TC_H = 0x04
    XG_OFFS_TC_L = 0x05
    YG_OFFS_TC_H = 0x07
    YG_OFFS_TC_L = 0x08
    ZG_OFFS_TC_H = 0x0A
    ZG_OFFS_TC_L = 0x0B

    SELF_TEST_X_ACCEL = 0x0D
    SELF_TEST_Y_ACCEL = 0x0E
    SELF_TEST_Z_ACCEL = 0x0F

    XG_OFFS_USRH = 0x13
    XG_OFFS_USRL = 0x14
    YG_OFFS_USRH = 0x15
    YG_OFFS_USRL = 0x16
    ZG_OFFS_USRH = 0x17
    ZG_OFFS_USRL = 0x18

    SMPLRT_DIV = 0x19

    CONFIG = 0x1A
    GYRO_CONFIG = 0x1B
    ACCEL_CONFIG = 0x1C
    ACCEL_CONFIG2 = 0x1D
    LP_MODE_CFG = 0x1E

    ACCEL_WOM_X_THR = 0x20
    ACCEL_WOM_Y_THR = 0x21
    ACCEL_WOM_Z_THR = 0x22

    FIFO_EN = 0x23

    FSYNC_INT = 0x36
    INT_PIN_CFG = 0x37
    INT_ENABLE = 0x38
    FIFO_WM_INT_STATUS = 0x39
    INT_STATUS = 0x3A

    ACCEL_XOUT_H = 0x3B
    ACCEL_XOUT_L = 0x3C
    ACCEL_YOUT_H = 0x3D
    ACCEL_YOUT_L = 0x3E
    ACCEL_ZOUT_H = 0x3F
    ACCEL_ZOUT_L = 0x40

    TEMP_OUT_H = 0x41
    TEMP_OUT_L = 0x42

    GYRO_XOUT_H = 0x43
    GYRO_XOUT_L = 0x44
    GYRO_YOUT_H = 0x45
    GYRO_YOUT_L = 0x46
    GYRO_ZOUT_H = 0x47
    GYRO_ZOUT_L = 0x48

    SELF_TEST_X_GYRO = 0x50
    SELF_TEST_Y_GYRO = 0x51
    SELF_TEST_Z_GYRO = 0x52

    FIFO_WM_TH1 = 0x60
    FIFO_WM_TH2 = 0x61

    SIGNAL_PATH_RESET = 0x68
    ACCEL_INTEL_CTRL = 0x69
    USER_CTRL = 0x6A

    PWR_MGMT_1 = 0x6B
    PWR_MGMT_2 = 0x6C

    I2C_IF = 0x70

    FIFO_COUNTH = 0x72
    FIFO_COUNTL = 0x73
    FIFO_R_W = 0x74

    WHO_AM_I = 0x75

    XA_OFFSET_H = 0x77
    XA_OFFSET_L = 0x78
    YA_OFFSET_H = 0x7A
    YA_OFFSET_L = 0x7B
    ZA_OFFSET_H = 0x7D
    ZA_OFFSET_L = 0x7E

    Gyro  = [0,0,0]
    Accel = [0,0,0]
    Mag   = [0,0,0]
    pitch = 0.0
    roll  = 0.0
    yaw   = 0.0
    pu8data=[0,0,0,0,0,0,0,0]
    U8tempX=[0,0,0,0,0,0,0,0,0]
    U8tempY=[0,0,0,0,0,0,0,0,0]
    U8tempZ=[0,0,0,0,0,0,0,0,0]
 
    def __init__(self, i2c, address=ICM20600_ADDR):
        self._address = address
        self._bus = i2c
        if self.reg_read(self.WHO_AM_I) != 0x11:
            print("ICM20600 not recognised:", self.reg_read(self.WHO_AM_I))
            return -1
        print("initialize")
        self.reg_write(self.CONFIG,0x00)
        self.reg_write(self.FIFO_EN,0x00)
        time.sleep(1)

        #need def power mode setting
        pwr1 = 0x00
        pwr1 = self._read_byte(self.PWR_MGMT_1)&0x8f
        gyr = self.reg_read(self.LP_MODE_CFG)&0x7f
        self.reg_write(self.PWR_MGMT_1,pwr1|0x00)
        self.reg_write(self.PWR_MGMT_2,0x00)
        self.reg_write(self.LP_MODE_CFG,gyr)
        #need gyro scale,output rate,average setting
        #2000DPS bw176 average 1
        #DPS
        gyr_c = self.reg_read(self.GYRO_CONFIG)&0xe7
        self.reg_write(self.GYRO_CONFIG,gyr_c|0x18)
        #rate
        conf = self.reg_read(self.CONFIG)&0xf8
        self.reg_write(self.CONFIG,conf|0x01)
        #averge
        lp_c = self.reg_read(self.LP_MODE_CFG)&0x8f
        self.reg_write(self.LP_MODE_CFG,lp_c|0x00)
        #need accel config setting
        #16G 1k420 average4 acc_scale 16000
        ac_c = self.reg_read(self.ACCEL_CONFIG)&0xE7
    #        print(ac_c)
        self.reg_write(self.ACCEL_CONFIG,0x18|ac_c)
        ac_c2 = self.reg_read(self.ACCEL_CONFIG2)&0xf0
        self.reg_write(self.ACCEL_CONFIG2,0x07|ac_c2)
        ac_c2 = self.reg_read(self.ACCEL_CONFIG2)&0xcf
        self.reg_write(self.ACCEL_CONFIG2,ac_c2)
        print("ICM-20600 OK\n")
 
    def s8(self,value):
        return -(value&0b10000000)|(value&0b01111111)

        
    def reg_write(self,addr,data):
        #print("write ",int(self._address),":",hex(addr),"-",data)
        i2c.writeto_mem(int(self._address),int(addr),bytes([int(data)]))
        time.sleep(0.0001)
        
    def reg_read(self,addr):
        #print("read: ", int(self._address),":",hex(addr))
        rec = self._bus.readfrom_mem(int(self._address),int(addr),1)
        return rec[0]

    def _read_byte(self,cmd):
        #print("read: ", int(self._address),int(cmd),1)
        rec=self._bus.readfrom_mem(int(self._address),int(cmd),1)
        return rec[0]
    
    def _read_block(self, reg, length=1):
        rec=self._bus.readfrom_mem(int(self._address),int(reg),length)
        return rec

    def _read_u16(self,cmd):
        LSB = self._bus.readfrom_mem(int(self._address),int(cmd),1)
        MSB = self._bus.readfrom_mem(int(self._address),int(cmd)+1,1)
        return (MSB[0] << 8) + LSB[0]

    def _write_byte(self,cmd,val):
        self._bus.writeto_mem(int(self._address),int(cmd),bytes([int(val)]))
        time.sleep(0.0001)

    def getAccel(self):
        raw_accel=[0.0,0.0,0.0]
        raw_accel[0] = self.s8(self.reg_read(self.ACCEL_XOUT_H))*16/0xff*2
        raw_accel[1] = self.s8(self.reg_read(self.ACCEL_YOUT_H))*16/0xff*2
        raw_accel[2] = self.s8(self.reg_read(self.ACCEL_ZOUT_H))*16/0xff*2
        return raw_accel

    def getGyro(self):
        raw_gyro = [0,0,0]
        raw_gyro[0] = self.s8(self.reg_read(self.GYRO_XOUT_H))/0xff*2*2000
        raw_gyro[1] = self.s8(self.reg_read(self.GYRO_YOUT_H)+1)/0xff*2*2000
        raw_gyro[2] = self.s8(self.reg_read(self.GYRO_ZOUT_H))/0xff*2*2000
        return raw_gyro

class Ak09918:
    AK09918_ADDR = 0x0C
    WIA1 = 0x00
    WIA2 = 0x01
    RAV1 = 0x02
    RSV2 = 0x03
    ST1 = 0x10
    HXL = 0x11
    HXH = 0x12
    HYL = 0x13
    HYH = 0x14
    HZL = 0x15
    HZH = 0x16
    TMPS = 0x17
    ST2 = 0x18
    CNTL1 = 0x30
    CNTL2 = 0x31
    CNTL3 = 0x32
    TS1 = 0x33
    TS2 = 0x34
    CMM = 0x00 # Power down
    CMM0 = 0x01  # Measurement mode Single
    CMM1 = 0x02  # Measurement mode 10 Hz
    CMM2 = 0x04  # Measurement mode 20 Hz
    CMM3 = 0x06  # Measurement mode 50 Hz
    CMM4 = 0x08  # Measurement mode 100 Hz
    DRDY = 0x01  # data ready
    DOR  = 0x02  # data overrun
    
    def __init__(self, i2c, addr = AK09918_ADDR):
        self._address = addr
        self._i2c = i2c
        self.reg_write(self.CNTL3,0x01)
        self.reg_read(self.ST1)
        self.reg_write(self.CNTL2,self.CMM)
        self.reg_write(self.CNTL2,self.CMM0)

        self.reg_read(self.ST1)
        self.reg_read(self.CNTL2)
        
    def reg_write(self,reg,data):
        self._i2c.writeto_mem(int(self._address),int(reg),bytes([int(data)]))

    def reg_read(self,addr):
        reg = self._i2c.readfrom_mem(int(self._address),int(addr),1)
        return reg[0]

    def get_mag_axis(self):
        self.reg_write(self.CNTL3,0x01)
        self.reg_write(self.CNTL2,self.CMM0)
        time.sleep(0.01)
        while not self.reg_read(self.ST1) & self.DRDY:
          print(self.reg_read(self.ST1))
          pass
        mag_x_axis = bytearray(2)
        mag_y_axis = bytearray(2)
        mag_z_axis = bytearray(2)
        mag_x_axis[0] = self.reg_read(self.HXL)
        mag_x_axis[1] = self.reg_read(self.HXH)
        mag_y_axis[0] = self.reg_read(self.HYL)
        mag_y_axis[1] = self.reg_read(self.HYH)
        mag_z_axis[0] = self.reg_read(self.HZL)
        mag_z_axis[1] = self.reg_read(self.HZH)
        x = int.from_bytes(mag_x_axis,'little',True)
        y = int.from_bytes(mag_y_axis,'little',True)
        z = int.from_bytes(mag_z_axis,'little',True)
        if x > 32767: x -= 65536 
        if y > 32767: y -= 65536 
        if z > 32767: z -= 65536 
        self.reg_read(self.reg_read(self.ST2))
        return [x,y,z]

    def get_heading(self):
        self.xyz = self.get_mag_axis()
        #self.normvals = self.normalize(self.mag_axis)
        self.compass_heading = int(math.atan2(self.xyz[1], self.xyz[0]) * 180.0 / math.pi)
        self.compass_heading += 180
        return self.compass_heading
    
icm = Icm20600(i2c)
ak = Ak09918(i2c)

#ax = ak.get_mag_axis()
#print(ax)
print(ak.get_heading(),'°')

while True:
    #print(icm.getAccel())
    #print(icm.getGyro())
    print(ak.get_mag_axis())
    print(ak.get_heading(),'°')
    time.sleep(1)
    