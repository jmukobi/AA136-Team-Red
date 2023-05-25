# type: ignore
"""
CircuitPython driver for Feather M4 board.
"""
# Common CircuitPython Libs
import board#, microcontroller
import busio, time, sys
#from storage import mount,umount,VfsFat
#from analogio import AnalogIn
import digitalio, sdcardio, pwmio, tasko

# Hardware Specific Libs
import adafruit_rfm9x # Radio
#import bmx160 # IMU
import neopixel # RGB LED
#import bq25883 # USB Charger
#import adm1176 # Power Monitor

# Common CircuitPython Libs
from os import listdir,stat,statvfs,mkdir,chdir
#from bitflags import bitFlag,multiBitFlag,multiByte
from micropython import const

#Sensor Imports
#import board
from adafruit_lsm6ds.lsm6dsox import LSM6DSOX as LSM6DS
from adafruit_lis3mdl import LIS3MDL
from adafruit_motorkit import MotorKit

#Magnetorquer Imports
#import board
#from adafruit_motorkit import MotorKit

#Reaction Wheel Imports
#import board
import pwmio
#import digitalio

"""
# NVM register numbers
_BOOTCNT  = const(0)
_VBUSRST  = const(6)
_STATECNT = const(7)
_TOUTS    = const(9)
_GSRSP    = const(10)
_ICHRG    = const(11)
_FLAG     = const(16)
"""
SEND_BUFF=bytearray(252)

print("ayo")

class Satellite:
    # General NVM counters
    #c_boot      = multiBitFlag(register=_BOOTCNT, lowest_bit=0,num_bits=8)
    #c_vbusrst   = multiBitFlag(register=_VBUSRST, lowest_bit=0,num_bits=8)
    #c_state_err = multiBitFlag(register=_STATECNT,lowest_bit=0,num_bits=8)
    #c_gs_resp   = multiBitFlag(register=_GSRSP,   lowest_bit=0,num_bits=8)
    #c_ichrg     = multiBitFlag(register=_ICHRG,   lowest_bit=0,num_bits=8)

    # Define NVM flags
    #f_lowbatt  = bitFlag(register=_FLAG,bit=0)
    #f_solar    = bitFlag(register=_FLAG,bit=1)
    #f_gpson    = bitFlag(register=_FLAG,bit=2)
    #f_lowbtout = bitFlag(register=_FLAG,bit=3)
    #f_gpsfix   = bitFlag(register=_FLAG,bit=4)
    #f_shtdwn   = bitFlag(register=_FLAG,bit=5)

    def __init__(self):
        """
        Big init routine as the whole board is brought up.
        """
        self.BOOTTIME= const(time.time())
        self.data_cache={}
        #self.filenumbers={}
        #self.vlowbatt=6.0
        #self.send_buff = memoryview(SEND_BUFF)
        self.debug=True
        #self.micro=microcontroller
        """self.hardware = {
                       'IMU':    False,
                       'Radio1': False,
                       'Radio2': False,
                       'SDcard': False,
                       'GPS':    False,
                       'WDT':    False,
                       'USB':    False,
                       'PWR':    False}"""
        # Define burn wires:
        #self._relayA = digitalio.DigitalInOut(board.RELAY_A)
        #self._relayA.switch_to_output(drive_mode=digitalio.DriveMode.OPEN_DRAIN)
        #self._resetReg = digitalio.DigitalInOut(board.VBUS_RST)
        #self._resetReg.switch_to_output(drive_mode=digitalio.DriveMode.OPEN_DRAIN)

        # Define battery voltage
        #self._vbatt = AnalogIn(board.BATTERY)

        # Define MPPT charge current measurement
        #self._ichrg = AnalogIn(board.L1PROG)
        #self._chrg = digitalio.DigitalInOut(board.CHRG)
        #self._chrg.switch_to_input()

        # Define SPI,I2C,UART
        self.i2c = board.I2C()  # uses board.SCL and board.SDA
        self.accel_gyro = LSM6DS(self.i2c)
        self.mag = LIS3MDL(self.i2c)
        #self.spi   = board.SPI()
        self.uart  = busio.UART(board.TX,board.RX)

        self.cs = digitalio.DigitalInOut(board.D5)
        self.reset = digitalio.DigitalInOut(board.D6)
        self.spi = busio.SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)

        # Define Magnetorquer Pins
        self.motorwing = MotorKit(i2c=self.i2c)

        #digitalio.DigitalInOut(board.A3).deinit()
        #pwmio.PWMOut(board.A2).deinit()
        #Define Motor Pins
        #self.pwm1 = pwmio.PWMOut(board.A2, frequency=5000, duty_cycle=1)
        #self.dir1 = digitalio.DigitalInOut(board.A3)
        #self.dir1.direction = digitalio.Direction.OUTPUT

        #self.pwm2 = pwmio.PWMOut(board.A4, frequency=5000, duty_cycle=1)
        self.dir2 = digitalio.DigitalInOut(board.A5)
        self.dir2.direction = digitalio.Direction.OUTPUT

        self.pwm3 = pwmio.PWMOut(board.D10, frequency=5000, duty_cycle=1)
        self.dir3 = digitalio.DigitalInOut(board.D9)
        self.dir3.direction = digitalio.Direction.OUTPUT

        # Define GPS
        #self.en_gps = digitalio.DigitalInOut(board.EN_GPS)
        #self.en_gps.switch_to_output()

        # Define filesystem stuff
        self.logfile="/log.txt"

        """
        # Define radio
        _rf_cs1 = digitalio.DigitalInOut(board.RF1_CS)
        _rf_rst1 = digitalio.DigitalInOut(board.RF1_RST)
        self.enable_rf = digitalio.DigitalInOut(board.EN_RF)
        self.radio1_DIO0=digitalio.DigitalInOut(board.RF1_IO0)
        # self.enable_rf.switch_to_output(value=False) # if U21
        self.enable_rf.switch_to_output(value=True) # if U7
        _rf_cs1.switch_to_output(value=True)
        _rf_rst1.switch_to_output(value=True)
        self.radio1_DIO0.switch_to_input()
        """
        """
        # Initialize SD card (always init SD before anything else on spi bus)
        try:
            # Baud rate depends on the card, 4MHz should be safe
            _sd = sdcardio.SDCard(self.spi, board.SD_CS, baudrate=4000000)
            _vfs = VfsFat(_sd)
            mount(_vfs, "/sd")
            self.fs=_vfs
            sys.path.append("/sd")
            self.hardware['SDcard'] = True
            self.logfile="/sd/log.txt"
        except Exception as e:
            if self.debug: print('[ERROR][SD Card]',e)
        """
        # Initialize Neopixel
        try:
            self.neopixel = neopixel.NeoPixel(board.A1, 1, brightness=0.5)
            self.neopixel[0] = (255,255,0)
            #self.hardware['Neopixel'] = True
        except Exception as e:
            if self.debug: print('[WARNING][Neopixel]',e)

        # Initialize magnetorquers
        self.mag_throttle = [0, 0, 0]
        self.motorwing.motor1.throttle = self.mag_throttle[0]
        self.motorwing.motor2.throttle = self.mag_throttle[1]
        self.motorwing.motor3.throttle = self.mag_throttle[2]

        """
        # Initialize USB charger
        try:
            self.usb = bq25883.BQ25883(self.i2c1)
            self.usb.charging = False
            self.usb.wdt = False
            self.usb.led=False
            self.usb.charging_current=8 #400mA
            self.usb_charging=False
            self.hardware['USB'] = True
        except Exception as e:
            if self.debug: print('[ERROR][USB Charger]',e)

        # Initialize Power Monitor
        try:
            self.pwr = adm1176.ADM1176(self.i2c1)
            self.pwr.sense_resistor = 1
            self.hardware['PWR'] = True
        except Exception as e:
            if self.debug: print('[ERROR][Power Monitor]',e)

        # Initialize IMU
        try:
            self.IMU = bmx160.BMX160_I2C(self.i2c1)
            self.hardware['IMU'] = True
        except Exception as e:
            if self.debug: print('[ERROR][IMU]',e)

        # # Initialize GPS
        # try:
        #     self.gps = GPS(self.uart,debug=False) # still powered off!
        #     self.gps.timeout_handler=self.timeout_handler
        #     self.hardware['GPS'] = True
        # except Exception as e:
        #     if self.debug: print('[ERROR][GPS]',e)
        """
        # Initialize radio #1 - UHF
        try:
            # Configure the radio featherwing
            self.rfm9x = adafruit_rfm9x.RFM9x(self.spi, self.cs, self.reset, 433)
            self.rfm9x.spreading_factor = 9
            self.rfm9x.coding_rate = 5
            self.rfm9x.bandwidth = 500000
            # Configure the radio settings
            self.rfm9x.tx_power = 23
            self.rfm9x.enable_crc = True
        except Exception as e:
            if self.debug: print('[ERROR][RADIO]',e)

        # set PyCubed power mode
        #self.power_mode = 'normal'

    '''
    def reinit(self,dev):
        dev=dev.lower()
        if   dev=='gps':
            self.gps.__init__(self.uart,debug=False)
        elif dev=='pwr':
            self.pwr.__init__(self.i2c1)
        elif dev=='usb':
            self.usb.__init__(self.i2c1)
        elif dev=='imu':
            self.IMU.__init__(self.i2c1)
        else:
            print('Invalid Device? ->',dev)'''

    @property
    def acceleration(self):
        #if self.hardware['IMU']:
        return self.accel_gyro # m/s^2

    @property
    def magnetic(self):
        return self.IMU.mag # uT

    @property
    def gyro(self):
        #if self.hardware['IMU']:
        return self.IMU.gyro # deg/s

    @property
    def temperature(self):
        #if self.hardware['IMU']:
        return self.IMU.temperature # Celsius

    @property
    def RGB(self):
        return self.neopixel[0]
    @RGB.setter
    def RGB(self,value):
        try:
            self.neopixel[0] = value
        except Exception as e:
            print('[WARNING]',e)

    @property
    def magnetorquer(self):
        return self.mag_throttle
    @magnetorquer.setter
    def magnetorquer(self, mag_throttle):
        throttle = throttle/2 #Max magnetorquer voltage = 6V, supply voltage = 12V
        #throttle is a length 3 array for magnetorquer voltages of the form [x,y,z]
        self.motorwing.motor1.throttle = mag_throttle[0]
        self.mag_throttle[0] = mag_throttle[0]
        self.motorwing.motor2.throttle = mag_throttle[1]
        self.mag_throttle[1] = mag_throttle[1]
        self.motorwing.motor3.throttle = mag_throttle[2]
        self.mag_throttle[2] = mag_throttle[2]
        #kit.motor4.throttle = throttle


    '''@property
    def charge_batteries(self):
        if self.hardware['USB']:
            return self.usb_charging
    @charge_batteries.setter
    def charge_batteries(self,value):
        if self.hardware['USB']:
            self.usb_charging=value
            self.usb.led=value
            self.usb.charging=value

    @property
    def battery_voltage(self):
        _vbat=0
        for _ in range(50):
            _vbat+=self._vbatt.value * 3.3 / 65536
        _voltage = (_vbat/50)*(316+110)/110 # 316/110 voltage divider
        return _voltage # volts

    @property
    def system_voltage(self):
        if self.hardware['PWR']:
            try:
                return self.pwr.read()[0] # volts
            except Exception as e:
                print('[WARNING][PWR Monitor]',e)
        else:
            print('[WARNING] Power monitor not initialized')

    @property
    def current_draw(self):
        """
        current draw from batteries
        NOT accurate if powered via USB
        """
        if self.hardware['PWR']:
            idraw=0
            try:
                for _ in range(50): # average 50 readings
                    idraw+=self.pwr.read()[1]
                return (idraw/50)*1000 # mA
            except Exception as e:
                print('[WARNING][PWR Monitor]',e)
        else:
            print('[WARNING] Power monitor not initialized')

    def charge_current(self):
        """
        LTC4121 solar charging IC with charge current monitoring
        See Programming the Charge Current section
        """
        _charge = 0
        if self.solar_charging:
            _charge = self._ichrg.value * 3.3 / 65536
            _charge = ((_charge*988)/3010)*1000
        return _charge # mA

    @property
    def solar_charging(self):
        return not self._chrg.value

    @property
    def reset_vbus(self):
        # unmount SD card to avoid errors
        if self.hardware['SDcard']:
            try:
                umount('/sd')
                self.spi.deinit()
                time.sleep(3)
            except Exception as e:
                print('vbus reset error?', e)
                pass
        self._resetReg.drive_mode=digitalio.DriveMode.PUSH_PULL
        self._resetReg.value=1'''

    '''
    def log(self, msg):
        if self.hardware['SDcard']:
            with open(self.logfile, "a+") as f:
                t=int(time.monotonic())
                f.write('{}, {}\n'.format(t,msg))
    
    def print_file(self,filedir=None,binary=False):
        if filedir==None:
            return
        print('\n--- Printing File: {} ---'.format(filedir))
        if binary:
            with open(filedir, "rb") as file:
                print(file.read())
                print('')
        else:
            with open(filedir, "r") as file:
                for line in file:
                    print(line.strip())

    def timeout_handler(self):
        print('Incrementing timeout register')
        if (self.micro.nvm[_TOUTS] + 1) >= 255:
            self.micro.nvm[_TOUTS]=0
            # soft reset
            self.micro.on_next_reset(self.micro.RunMode.NORMAL)
            self.micro.reset()
        else:
            self.micro.nvm[_TOUTS] += 1

    def powermode(self,mode):
        """
        Configure the hardware for minimum or normal power consumption
        Add custom modes for mission-specific control
        """
        if 'min' in mode:
            self.RGB = (0,0,0)
            self.neopixel.brightness=0
            if self.hardware['Radio1']:
                self.radio1.sleep()
            if self.hardware['Radio2']:
                self.radio2.sleep()
            self.enable_rf.value = False
            if self.hardware['IMU']:
                self.IMU.gyro_powermode  = 0x14 # suspend mode
                self.IMU.accel_powermode = 0x10 # suspend mode
                self.IMU.mag_powermode   = 0x18 # suspend mode
            if self.hardware['PWR']:
                self.pwr.config('V_ONCE,I_ONCE')
            if self.hardware['GPS']:
                self.en_gps.value = False
            self.power_mode = 'minimum'

        elif 'norm' in mode:
            self.enable_rf.value = True
            if self.hardware['IMU']:
                self.reinit('IMU')
            if self.hardware['PWR']:
                self.pwr.config('V_CONT,I_CONT')
            if self.hardware['GPS']:
                self.en_gps.value = True
            self.power_mode = 'normal'
            # don't forget to reconfigure radios, gps, etc...

    def new_file(self,substring,binary=False):
        '''
        #substring something like '/data/DATA_'
        #directory is created on the SD!
        #int padded with zeros will be appended to the last found file
    '''
        if self.hardware['SDcard']:
            ff=''
            n=0
            _folder=substring[:substring.rfind('/')+1]
            _file=substring[substring.rfind('/')+1:]
            print('Creating new file in directory: /sd{} with file prefix: {}'.format(_folder,_file))
            try: chdir('/sd'+_folder)
            except OSError:
                print('Directory {} not found. Creating...'.format(_folder))
                try: mkdir('/sd'+_folder)
                except Exception as e:
                    print(e)
                    return None
            for i in range(0xFFFF):
                ff='/sd{}{}{:05}.txt'.format(_folder,_file,(n+i)%0xFFFF)
                try:
                    if n is not None:
                        stat(ff)
                except:
                    n=(n+i)%0xFFFF
                    # print('file number is',n)
                    break
            print('creating file...',ff)
            if binary: b='ab'
            else: b='a'
            with open(ff,b) as f:
                f.tell()
            chdir('/')
            return ff

    def burn(self,burn_num,dutycycle=0,freq=1000,duration=1):
        """
        Operate burn wire circuits. Wont do anything unless the a nichrome burn wire
        has been installed.

        IMPORTANT: See "Burn Wire Info & Usage" of https://pycubed.org/resources
        before attempting to use this function!

        burn_num:  (string) which burn wire circuit to operate, must be either '1' or '2'
        dutycycle: (float) duty cycle percent, must be 0.0 to 100
        freq:      (float) frequency in Hz of the PWM pulse, default is 1000 Hz
        duration:  (float) duration in seconds the burn wire should be on
        """
        # convert duty cycle % into 16-bit fractional up time
        dtycycl=int((dutycycle/100)*(0xFFFF))
        print('----- BURN WIRE CONFIGURATION -----')
        print('\tFrequency of: {}Hz\n\tDuty cycle of: {}% (int:{})\n\tDuration of {}sec'.format(freq,(100*dtycycl/0xFFFF),dtycycl,duration))
        # create our PWM object for the respective pin
        # not active since duty_cycle is set to 0 (for now)
        if '1' in burn_num:
            burnwire = pwmio.PWMOut(board.BURN1, frequency=freq, duty_cycle=0)
        elif '2' in burn_num:
            burnwire = pwmio.PWMOut(board.BURN2, frequency=freq, duty_cycle=0)
        else:
            return False
        # Configure the relay control pin & open relay
        self._relayA.drive_mode=digitalio.DriveMode.PUSH_PULL
        self._relayA.value = 1
        self.RGB=(255,0,0)
        # Pause to ensure relay is open
        time.sleep(0.5)
        # Set the duty cycle over 0%
        # This starts the burn!
        burnwire.duty_cycle=dtycycl
        time.sleep(duration)
        # Clean up
        self._relayA.value = 0
        burnwire.duty_cycle=0
        self.RGB=(0,0,0)
        burnwire.deinit()
        self._relayA.drive_mode=digitalio.DriveMode.OPEN_DRAIN
        return True
    '''


cubesat = Satellite()
print(cubesat)