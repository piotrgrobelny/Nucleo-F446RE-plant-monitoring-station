# Plant monitoring station
Plant monitoring station based on arduino UNO/Nucleo-F446RE. Station collects data about temperature, humidity or light intensity from various sensors and display them in python app.

## How does it work? 
Station is working on Arduino UNO or Nucleo-F446RE. There are three sensors which are connected to main board.
* temperature and air humidity sensor with digital output
* light intensity sensor with analog output
* soil humidity sensor with analog output 

Main board Nucleo/arduino is collecting data from those sensors and transfering them to PC by serial USB. Format of sended data in both cases is the same: string with label of sensor and read value.

### Nucleo
To run program on Nucleo-F446RE you need to push hex file which is inside `Plant_station_nucleo` directory to nucleo board.
You can do it by STM32 ST-LINK utility program. If you have other nucleo board you can use `main.c` in the same directory as a help to build hex file. Uart is configured with a 115200 baud rate. 
 
### Arduino 
To run program on Arduino Uno just open the project from `Plant_station_arduino` dir, add `dht11.zip` library from `Libraries_arduino` dir and upload code to your board.
 
#### Sensor connections 
Nucleo: 
* Temperature and humidity sensor -> D7
* Soil humidity sensor -> A0
* Light intensity -> A1

Arduino:
* Temperature and humidity sensor -> D3
* Soil humidity sensor -> A1
* Light intensity -> A0

#### USB serial configuration
Port is configured to automatically recognize STM boards. If you want to use arduino or you can't connect with STM you need to change manually COM port inside `read_data.py` file. `ser = serial.Serial(serial_port, 115200)` just change serial_port to your COM port.
## Python application
If your serial output looks similar to this: 
`T24 L8 H45 S34`(T - temperature, L - light, H - air humidity, S - soil humidity) you can just run `Plant station.exe` file inside `Plant_station_python` dir.

#### Code 
* `read_data.py` - Python file responsible for reading and saving data from USB serial
* `gui.py` - Gui is designed in QtDesigner program, you can find template in `gui templates`.
* `main.py` - Init read functions and gui to display data on screen     

