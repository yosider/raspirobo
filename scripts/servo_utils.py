from time import sleep
import smbus
import math

class Servos(object):
    """Control surbo motors through PCA9685."""
    def __init__(self):
        self.bus = smbus.SMBus(1)
        self.address_pca9685 = 0x40

        self._resetPCA9685()
        self._setPCA9685Freq(50)

        self.NEUTRAL = 276
        self.PLUS = self.NEUTRAL + 100
        self.MINUS = self.NEUTRAL - 100


    def _resetPCA9685(self):
        self.bus.write_byte_data(self.address_pca9685, 0x00, 0x00)

    def _setPCA9685Freq(self, freq):
        freq = 0.9*freq # from Arduino library...
        prescaleval = 25000000.0    # 25MHz
        prescaleval /= 4096.0       # 12-bit
        prescaleval /= float(freq)
        prescaleval -= 1.0
        prescale = int(math.floor(prescaleval + 0.5))
        oldmode = self.bus.read_byte_data(self.address_pca9685, 0x00)
        newmode = (oldmode & 0x7F) | 0x10             # sleepmode
        self.bus.write_byte_data(self.address_pca9685, 0x00, newmode) # to sleepmode
        self.bus.write_byte_data(self.address_pca9685, 0xFE, prescale) # prescaler
        self.bus.write_byte_data(self.address_pca9685, 0x00, oldmode)
        sleep(0.005)
        self.bus.write_byte_data(self.address_pca9685, 0x00, oldmode | 0xa1)


    def setPCA9685Duty2(self, channel, off1, off2):
        channelpos = 0x6 + 4*channel
        on1 = on2 = 0
        # x&0xFF = 1~8th bit of x
        # x>>8 = 8 bit shift of x (0 if [0,255], 1 if [256,511], ...)
        data = [on1&0xFF, on1>>8, off1&0xFF, off1>>8,
                on2&0xFF, on2>>8, off2&0xFF, off2>>8]
        self.bus.write_i2c_block_data(self.address_pca9685, channelpos, data)


    def setPCA9685Duty6(self, channel, off1, off2, off3, off4, off5, off6):
        channelpos = 0x6 + 4*channel
        on1 = on2 = on3 = on4 = on5 = on6 = 0
        data = [on1&0xFF, on1>>8, off1&0xFF, off1>>8,
                on2&0xFF, on2>>8, off2&0xFF, off2>>8,
                on3&0xFF, on3>>8, off3&0xFF, off3>>8,
                on4&0xFF, on4>>8, off4&0xFF, off4>>8,
                on5&0xFF, on5>>8, off5&0xFF, off5>>8,
                on6&0xFF, on6>>8, off6&0xFF, off6>>8]
        self.bus.write_i2c_block_data(self.address_pca9685, channelpos, data)
