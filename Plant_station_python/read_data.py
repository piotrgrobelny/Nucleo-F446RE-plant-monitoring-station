
import time

import serial.tools.list_ports

#automatically connect to port with STM
ports = serial.tools.list_ports.comports()
for port, desc, hwid in sorted(ports):
    if "STM" in desc:
        serial_port = str(port)
ser = serial.Serial(serial_port, 115200)
time.sleep(2)

air_humidity = []
temperature = []
light_intensity = []
soil_humidity = []

#saving data from sensors
def save_data():
    data = read_string()
    air_humidity.append(data[0])
    temperature.append(data[1])
    light_intensity.append(data[2])
    soil_humidity.append(data[3])
    return data

#read data from serial
def read_string():
    air_humidity = 0
    temperature = 0
    light_intensity = 0
    soil_humidity = 0
    for x in range(0, 4):
        b = ser.readline()    #read serial
        string_n = b.decode()   #decode to unicode
        string = string_n.rstrip()  #strip from /n
        if string:
            sensor = string[0] #decode letter that tell from which sensor is given value
            value = float(string[1:len(string)-1])
            if sensor == "H":
                air_humidity = value
            elif sensor == "T":
                temperature = value
            elif sensor == "L":
                light_intensity = value
            elif sensor == "S":
                soil_humidity = value

    return air_humidity, temperature, light_intensity, soil_humidity

if __name__ == "__main__":
    while(1):
        save_data()