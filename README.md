# plant-monitoring-station
Monitoring station based on arduino/stm32. It's consist of: temperature and humidity sensor, light intensity sensor and LCD display 

## Nukleo
Żeby uruchomić program na Nukleo, należy stworzyć nowy projekt (ja pisałam w System Workbench for STM32) i wykorzystać wrzucony plik jako plik main w swoim projekcie. Nie są potrzebne żadne dodatkowe biblioteki.

Aby móc przesyłać floaty przez UART, trzeba wejść w Properties -> Properties -> C/C++ Build -> Settings -> MCU GCC Linker -> Miscellaneous -> Linker flags i dodać tam flagę -u _printf_float.

Po skompilowaniu, należy wgrać plik hex na płytkę, używając np. STM32 ST-Link Utility. Powinien się znajdować w folderze Debug.

UART jest skonfigurowany na przesyłanie z baud rate 115200. Trzba pamiętać, żeby to zmienić np. w Putty.
