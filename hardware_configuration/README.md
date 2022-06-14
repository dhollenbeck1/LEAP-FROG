# Running Multiple Rangfinders Maxboxtic MB1202 IC2 Configuration with Raspberry Pi 4B
## Assign different i2c address to diffferent Rangfinders in adruino

## Write Python script to read rangfinder address
### Turn on i2c on raspberry pi
run
```
sudo raspi-config
```
Select I2C section and enable it, then reboot.
The rangfinder pin connection as follows
```
Pin(1) 3V3 to sensor VIN
Pin(9) GND to sensor GND
Pin(5) SCL to sensor SCL
Pin(3) SDA to sensor SDA
```
![Raspberry Pi Pin Distribution](https://cdn.sparkfun.com/assets/learn_tutorials/1/5/9/5/GPIO.png)

### Pre-install packages for i2c tools and PMBus.
```
sudo apt-get install i2c-tools
pip3 install smbus2
```
### Check whether i2c would be able to detect multiple rangfinders, it should be at different address
run the following command to check which i2c port are currently occupied.
```
ls /dev/*i2c*
```
then running the following command with specific i2c port number, usually should be 1
```
i2cdetect -y 1
```
If you were assigned different i2c address correctly, then you should be able to see multiple rangefinders with unique i2c address
### One time setup for building data buffer of reading i2c rangefinder value
The default I2C-Address of the sensor is 0x70. To perform a range measurement you must send the "Take Range Reading” command byte 0x51.
```
i2cset -y 1 <rangefinder address> 0x51
i2cget -y 1 <rangefinder address> 0xE1 w # read, which in rangefinder is 225 8bit reading/writing
```
For example, in our LEAPFROG project, we have
``` 
i2cset -y 1 0x70 0x51 
i2cget -y 1 0x70 0xE1 w
```
Secondary rangefinder setup
```
i2cset -y 1 0x6F 0x51 
i2cget -y 1 0x6F 0xE1 w
```
keeping implementing above commands if you have multiple rangfinders, make sure change the i2c address

If everything looks good, please run python script to get the rangefinder data in real-time.
```
python3 i2creading.py
```

