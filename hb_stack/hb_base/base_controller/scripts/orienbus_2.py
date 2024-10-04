##############################################################

import minimalmodbus
import hid

_WRITE_REGISTER = 125
_FEEDBACK_SPEED_REG_LOWER = 207
_FEEDBACK_SPEED_REG_UPPER = 206

_WRITE_REGISTER_SPEED = 1157
_FWD_DEC = 10#56
_REV_DEC = 18#24

_MIN_RPM = 80
_MAX_RPM = 4000

speed = 0

class ModBus(object):

    """
        ModBus class for talking to instruments (slaves).
        Uses the minimalmodbus python library.
    Args:
        * port (str): The serial port name, for example ``/dev/ttyUSB0`` (Linux),
          ``/dev/tty.usbserial`` (OS X) or ``COM4`` (Windows).
        * slaveaddress (int): Slave address in the range 1 to 247 (use decimal numbers,
          not hex). Address 0 is for broadcast, and 248-255 are reserved.
    """

    def __init__(self, _port, _slave_address):

        self._port = _port
        self._slave_address = _slave_address

        self.instrument = minimalmodbus.Instrument(self._port, self._slave_address)
        self.instrument.serial.baudrate = 115200
        self.instrument.serial.bytesize = 8
        self.instrument.serial.parity   = minimalmodbus.serial.PARITY_EVEN
        self.instrument.serial.stopbits = 1
        self.instrument.serial.timeout  = 0.05
        self.instrument.mode = minimalmodbus.MODE_RTU
        self.instrument.clear_buffers_before_each_transaction = True

        print("Successfully Connected to Slave Address {} ...".format(self._slave_address))

    def writeSpeed(self, speed):
        try:
            if (speed >= _MIN_RPM and speed <= _MAX_RPM):
                print("forward lah")
                self.instrument.write_register(_WRITE_REGISTER_SPEED, speed)#,0,6,False)
                self.instrument.write_register(_WRITE_REGISTER, _FWD_DEC)#,0,6,False) # run motor forward with default acceleration
                

            elif (speed <= -_MIN_RPM and speed >= -_MAX_RPM):
                print("reverse lah")
                self.instrument.write_register(_WRITE_REGISTER_SPEED, -speed)#,0,6,False)
                self.instrument.write_register(_WRITE_REGISTER, _REV_DEC)#,0,6,False) # run motor backward with default acceleration
                #self.instrument.write_register(_WRITE_REGISTER, 24,0,6,False) # run motor forward with default acceleration
                #print("reverse lah")
		

            else:
                print("stop lah")
                self.instrument.write_register(_WRITE_REGISTER, 34)#,0,6,False)
                #self.instrument.write_register(_WRITE_REGISTER, _REV_DEC,0,6,False) # stop motor with default deceleartion
		
        except:
            pass
            #print("nope")
    def readSpeed(self):

        global speed
        try:
            speed = self.instrument.read_register(_FEEDBACK_SPEED_REG_LOWER) - self.instrument.read_register(_FEEDBACK_SPEED_REG_UPPER)
        except:
            print("Failed to read from instrument")

        

        return speed

class OrienBus(object):


    def __init__(self, _port):
        self._port = _port

        print("Connecting to port {} ...".format(self._port))

    def initialize(self, _slave_address):
        return ModBus(self._port, _slave_address)
	
