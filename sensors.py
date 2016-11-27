#!/usr/bin/python
import paho.mqtt.client  as mqtt
import paho.mqtt.publish as publish
import time,json,ssl
import smbus
import time from ctypes
import c_short


#import wiringpi2 as wpi
#from tentacle_pi.TSL2561 import TSL2561
#import nfc
import sys, thread



def on_connect(mqttc, obj, flags, rc):
    if rc == 0:
        print("Connected with result code "+str(rc))
        # client.subscribe("$SYS/#")
    else :
        print("Error connecting to AWS IoT service! (Error code " + str(rc) + ": " + RESULT_CODES[rc] + ")")
        client.disconnect()

def on_message(client, userdata, msg):
    print(msg.topic+" "+str(msg.payload))

#Connect to AWS IoT
client = mqtt.Client(client_id="raspberry", protocol=mqtt.MQTTv311)
#print client
client.on_connect = on_connect
client.on_message = on_message
#time.sleep(3)
client.tls_set("certs/root-CA.crt", certfile="certs/certificate.pem.crt", keyfile="certs/private.pem.key", tls_version=ssl.PROTOCOL_SSLv23, ciphers=None)
client.tls_insecure_set(True)
client.connect("a17uoxmen1dmho.iot.us-east-1.amazonaws.com", 8883, 60)
client.loop_start()
# msg = {'data': 1}
# client.publish('sensorTopic', json.dumps(msg))

# GPIO pin setup
#wpi.wiringPiSetup()
#wpi.pinMode(2, 0) # PIR sensor on wpi GPIO2
#wpi.pinMode(3, 0) # mat pressure sensor on wpi GPIO3
#wpi.pinMode(21, 0) # sound sensor GATE on wpi GPIO21

# I2C TSL2561 setup
#tsl = TSL2561(0x39,"/dev/i2c-1")
#tsl.enable_autogain()
#tsl.set_time(0x00)

# nfc functions
#def connected(tag):
#    global nfcid
#    nfcid = str(tag)[12:]
#    print "read successful"
#    time.sleep(3)
#    return False

def reader():
    while True:
        # we do a read every 5s
        timeout = lambda: time.time() - started > 5
        started = time.time()
        #device = nfc.ContactlessFrontend('tty:S2:pn532')
        #device.connect(rdwr={'on-connect': connected}, terminate=timeout)
        #device.close()
        

#nfcid = 0
# thread = threading.Thread(target=reader)
# thread.start()
# end nfc setup

# count = 0
#mlux = mpir = mmat = msound = mvolume = 0


DEVICE = 0x77 # Default device I2C address
  
#bus = smbus.SMBus(0)  # Rev 1 Pi uses 0
bus = smbus.SMBus(1) # Rev 2 Pi uses 1
  
def convertToString(data):
  # Simple function to convert binary data into
  # a string
  return str((data[1] + (256 * data[0])) / 1.2)
 
def getShort(data, index):
  # return two bytes from data as a signed 16-bit value
  return c_short((data[index]<< 8) + data[index + 1]).value
 
def getUshort(data, index):
  # return two bytes from data as an unsigned 16-bit value
  return (data[index]<< 8) + data[index+1]
 
def readBmp180Id(addr=DEVICE):
  # Register Address
  REG_ID     = 0xD0
 
  (chip_id, chip_version) = bus.read_i2c_block_data(addr, REG_ID, 2)
  return (chip_id, chip_version)
   
def readBmp180(addr=DEVICE):
  # Register Addresses
  REG_CALIB  = 0xAA
  REG_MEAS   = 0xF4
  REG_MSB    = 0xF6
  REG_LSB    = 0xF7
  # Control Register Address
  CRV_TEMP   = 0x2E
  CRV_PRES   = 0x34
  # Oversample setting
  OVERSAMPLE = 3    # 0 - 3
   
  # Read calibration data
  # Read calibration data from EEPROM
  cal = bus.read_i2c_block_data(addr, REG_CALIB, 22)
 
  # Convert byte data to word values
  AC1 = getShort(cal, 0)
  AC2 = getShort(cal, 2)
  AC3 = getShort(cal, 4)
  AC4 = getUshort(cal, 6)
  AC5 = getUshort(cal, 8)
  AC6 = getUshort(cal, 10)
  B1  = getShort(cal, 12)
  B2  = getShort(cal, 14)
  MB  = getShort(cal, 16)
  MC  = getShort(cal, 18)
  MD  = getShort(cal, 20)
 
  # Read temperature
  bus.write_byte_data(addr, REG_MEAS, CRV_TEMP)
  time.sleep(0.005)
  (msb, lsb) = bus.read_i2c_block_data(addr, REG_MSB, 2)
  UT = (msb << 8) + lsb
 
  # Read pressure
  bus.write_byte_data(addr, REG_MEAS, CRV_PRES + (OVERSAMPLE << 6))
  time.sleep(0.04)
  (msb, lsb, xsb) = bus.read_i2c_block_data(addr, REG_MSB, 3)
  UP = ((msb << 16) + (lsb << 8) + xsb) >> (8 - OVERSAMPLE)
 
  # Refine temperature
  X1 = ((UT - AC6) * AC5) >> 15
  X2 = (MC << 11) / (X1 + MD)
  B5 = X1 + X2 temperature = (B5 + 8) >> 4
 
  # Refine pressure
  B6  = B5 - 4000
  B62 = B6 * B6 >> 12
  X1  = (B2 * B62) >> 11
  X2  = AC2 * B6 >> 11
  X3  = X1 + X2
  B3  = (((AC1 * 4 + X3) << OVERSAMPLE) + 2) >> 2
 
  X1 = AC3 * B6 >> 13
  X2 = (B1 * B62) >> 16
  X3 = ((X1 + X2) + 2) >> 2
  B4 = (AC4 * (X3 + 32768)) >> 15
  B7 = (UP - B3) * (50000 >> OVERSAMPLE)
 
  P = (B7 * 2) / B4
 
  X1 = (P >> 8) * (P >> 8)
  X1 = (X1 * 3038) >> 16
  X2 = (-7357 * P) >> 16
  pressure = P + ((X1 + X2 + 3791) >> 4)
 
  return (temperature/10.0,pressure/ 100.0)
 
def main():
  print
   
  (temperature,pressure)=readBmp180()
  print "Temperature : ", temperature, "C"
  print "Pressure    : ", pressure, "mbar"
   
if __name__=="__main__":
   main()




def func(name, delay):
    var = 1
    while True:
        time.sleep(3)
        ts = int(time.time())
        mpress = pressure
        mpir = var
        mmat = var
        mtemp = temperature
        mvolume = var
        mom = var
        dad = var + 1
        var = var + 1
        msg = {'timestamp': ts, 'pressure': mpress, 'pir': mpir, 'mat': mmat, 'temperature': mtemp, 'volume': mvolume, 'mom': mom, 'dad': dad}
        print name
        print json.dumps(msg)
        client.publish('sensorTopic', json.dumps(msg))


try:
    thread.start_new_thread(func("Thread1", 2))
    thread.start_new_thread(func("Thread2", 4))
except:
    print "Error: unable to start thread"

while 1:
    pass

