#TODO implement spool measurement (or setter from nfc) and save it to the file

import time, sys

from smbus2 import SMBusWrapper
from smbus2 import SMBus
#from misc.singleton import Singleton

tare = 12165
spool_weight = 0    # 215
scale = 975


#@Singleton
class I2CBus:
    def __init__(self, i2c_address):
        self.address = i2c_address
        self.flag = 1

    def measure_weight1(self):
        self._write(self.address, cmd=int(0xAF),data = [])
        time.sleep(2.0)                     #time for scale to measure
        result_str = ""
        for x in self._read(self.address):
            if x != 255:    # ignore ansii empty characters
                result_str += (chr(x))
        return (0.001*float(result_str)+699.69)
        #return (result_str)

    def measure_weight2(self):
        self._write(self.address, cmd=int(0xCF),data = [])
        time.sleep(0.5)
        result_str = ""
        for x in self._read(self.address):
            if x != 255:    # ignore ansii empty characters
                result_str += (chr(x))
        return (-0.0011*float(result_str) -72.898)
        #return(result_str)

    def turn_the_lights_on(self, R, G, B):
        self._control_light(0, R, G, B)

    def turn_the_lights_off(self):
        self._control_light(1, 255, 255, 255)

    def fade(self, R, G, B):
        self._control_light(3, R, G, B)

    def brighten(self, R, G, B):
        self._control_light(2, R, G, B)

    def breath(self, R, G, B):
        self._control_light(4, R, G, B)

    def flash(self, R, G, B):
        self._control_light(5 , R, G, B)

    def set_left_LED(self,nLED,R,G,B):
        self._write(self.address, cmd=int(0x0C), data=[nLED, R, G, B])

    def set_right_LED(self,nLED,R,G,B):
        self._write(self.address, cmd=int(0xC0), data=[nLED, R, G, B])

    def config_left_LED(self,nLED,R,G,B):
        self._write(self.address, cmd=int(0x0A), data=[nLED, R, G, B])
        
    def config_right_LED(self,nLED,R,G,B):
        self._write(self.address,cmd=int(0xA0), data=[nLED, R, G, B])

    def accept_LED_config(self):
        self._write(self.address,cmd =int(0xAA),data=[])

    def light_up_to_UP(self,nLED,R,G,B,panel='LR'):
        if panel == 'L':
            self._write(self.address, cmd=int(0xD0), data=[nLED, R, G, B])
        elif panel == 'R':
            self._write(self.address, cmd=int(0xE0), data=[nLED, R, G, B])
        else:
            self._write(self.address, cmd=int(0xB0), data=[nLED, R, G, B])

    def light_up_to_DOWN(self,nLED,R,G,B,panel='LR'):
        if panel == 'L':
            self._write(self.address, cmd=int(0x0D), data=[nLED, R, G, B])
        elif panel == 'R':
            self._write(self.address, cmd=int(0x0E), data=[nLED, R, G, B])
        else:
            self._write(self.address, cmd=int(0x0B), data=[nLED, R, G, B])

    def _control_light(self, mode, R, G, B):
        self._write(self.address, cmd=int(0xCC), data=[mode, R, G, B])
    
    def _power_OFF(self):
        self._write(self.address,cmd = int(0xFF),data=[])

    def _cancel_power_OFF(self):
        self._write(self.address,cmd = int(0xF0),data=[])
        time.sleep(0.1)
    def _read(self, address):
            b = self._errorHandleRead(address, 0, 16)
            while(b[1]):
                 b = self._errorHandleRead(address, 0, 16)
                 time.sleep(0.3)
            time.sleep(0.3)
            return(b[0])
                
    def _errorHandleRead(self, address, cmd, data=[]):
        try:
            errorflag = 0
            bus = SMBus(1)
            readout = bus.read_i2c_block_data(address, 0, 16)
            bus.close()
            time.sleep(0.1)
        except IOError:
            print "IOError"
            errorflag = 1
            time.sleep(0.2)
            bus = SMBus(1)
            bus.close()
            time.sleep(0.2)
            readout = 0;
        return (readout,errorflag)

    def _write(self, address, cmd, data=[]):
        while(self._errorHandleWrite(address,cmd,data)):           # checking if there was a connection error
            time.sleep(0.1)
            self._errorHandleWrite(address,cmd,data)            # sending message one again to fill broaken message and end message propearly
            time.sleep(0.1)

    def _errorHandleWrite(self, address, cmd, data=[]):
        try:
            errorflag = 0
            bus = SMBus(1)
            bus.write_i2c_block_data(address, cmd, data)
            bus.close()
            time.sleep(0.1)
        except IOError:
            print "IOError"
            errorflag = 1
            time.sleep(0.2)
            bus = SMBus(1)
            bus.close()
            time.sleep(0.2)
        return errorflag


if __name__ == '__main__':
    bus = I2CBus(8)
    bus.turn_the_lights_off()
    while 0:
        bus.turn_the_lights_on(0,0,255)
        for x in range(0,22):
           bus.set_right_LED(x,0,0,0)
        for y in range(0,22):
           bus.set_left_LED(y,0,0,0)
        #bus.turn_the_lights_off()
    while 1:
        ans = str(raw_input(">>"))
        if ans == 'q':
            break
        elif ans == 'm1':
                print bus.measure_weight1()
        elif ans == 'm2':
                print bus.measure_weight2()
        elif ans == 'Y':
            bus._power_OFF()
        elif ans == 'N':
            bus._cancel_power_OFF()
        elif ans == 'B':
            bus.breath(120,80,90);
        else:
            bus._control_light(int(ans), 255, 0, 0)