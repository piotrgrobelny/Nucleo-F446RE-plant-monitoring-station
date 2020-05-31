#include <dht11.h>
 
dht11 DHT11;
 
#define DHT11PIN 2    //przypisanie pinu 2 Arduino jako odczyt z sensora
int light_pin = A0;
int soil_pin = A1;
int light = 0;
int soil = 0;
void setup()
{
  Serial.begin(115200);                    //inicjalizacja monitora szeregowego
  //Serial.println("Program testowy DHT11"); 
  Serial.println();
}
 
void loop()
{
  int chk = DHT11.read(DHT11PIN);         //sprawdzenie stanu sensora, a następnie wyświetlenie komunikatu na monitorze szeregowym
  light = analogRead(light_pin);
  soil = analogRead(soil_pin);
  //Serial.print("Wilgotnosc (%): ");              //wyświetlenie wartości wilgotności
  //Serial.print((float)DHT11.humidity, 2);
  //Serial.print("   Temperatura (C): ");           //wyświetlenie temperatury
  //Serial.println((float)DHT11.temperature, 2);
  //Serial.print("   Naświetlenie (%): ");           //wyświetlenie temperatury
  //Serial.println(val);
  Serial.print("H"); //air humidity 
  Serial.println((float)DHT11.humidity, 2);
  Serial.print("T"); //temperature
  Serial.println((float)DHT11.temperature, 2);
  Serial.print("L"); //light intensity 
  Serial.println(light);
  Serial.print("S");//soil humidity 
  Serial.println(soil);
 
  delay(1000);                                  //opóźnienie między kolejnymi odczytami - 1 s
}
