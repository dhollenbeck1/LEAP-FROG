from smbus import SMBus
import time

i2cbus = SMBus(1)

while True:
    
    i2cbus.write_byte(0x70,0x51)
    i2cbus.write_byte(0x6f,0x51)
    time.sleep(0.12)
    val1 = i2cbus.read_word_data(0x70,0xE1)
    val2 = i2cbus.read_word_data(0x6f,0xE1)
    rng1 = (val1>>8) & 0xff | (val1 & 0xff)
    rng2 = (val2>>8) & 0xff | (val2 & 0xff)
    print("rangfinder 1 %s cm" % rng1)
    print("rangfinder 2 %s cm" % rng2)
