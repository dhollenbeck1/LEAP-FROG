from smbus import SMBus
import time

i2cbus = SMBus(1)
rng1_old = 700
rng2_old = 700
Ts = 0.2
alph = 2
rng2 = 700

while True:
    i2cbus.write_byte(0x70,0x51)
    i2cbus.write_byte(0x6f,0x51)
    time.sleep(Ts)
    val1 = i2cbus.read_word_data(0x70,0xE1)
    val2 = i2cbus.read_word_data(0x6f,0xE1)
    rng1_u = (val1>>8) & (0xff | (val1 & 0xff))
    rng2_u = (val2>>8) & (0xff | (val2 & 0xff))
    rng1 = rng1_old - alph*Ts*(rng1_old-rng1_u)
    rng1_old = rng1
    rng2 = rng2_old - alph*Ts*(rng2_old-rng2_u)
    rng2_old = rng2
    print("raw: %s %s, filt: %s %s" % \
          (round(rng1_u,0),round(rng2_u,0),round(rng1,0),round(rng2,0)))
    