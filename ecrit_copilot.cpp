#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>

#define DHTPIN 11     // Pin où le pin DATA du DHT22 est connecté
#define DHTTYPE DHT22 // Définir le type de DHT
#define DHT_POWER_SUPPLY_PIN 10 // Pin pour alimenter le DHT22

#define RESET_SIGFOX_MODULE 2 // Pin pour réinitialiser le module Sigfox
#define MICROPROCESOR_VOLTAGE_CAN_MEASUREMENT A0 
#define MICROPROCESOR_VOLTAGE_CAN_PULL_UP_PIN A1

DHT dht(DHTPIN, DHTTYPE);

void setup() {
  Serial.begin(9600);
  pinMode(DHT_POWER_SUPPLY_PIN, OUTPUT);
  pinMode(RESET_SIGFOX_MODULE, OUTPUT);
  digitalWrite(DHT_POWER_SUPPLY_PIN, LOW);
  dht.begin();
}

void loop() {
  lectureDHT();
  delay(5000);
}

void lectureDHT() {
  digitalWrite(DHT_POWER_SUPPLY_PIN, HIGH);
  delay(2000);

  float humidity = dht.readHumidity();
  float temperature = dht.readTemperature();

  if (isnan(humidity) || isnan(temperature)) {
    Serial.println("Échec de lecture du capteur DHT22 !");
  } else {
    Serial.print("Humidité: ");
    Serial.print(humidity);
    Serial.print(" %\t");
    Serial.print("Température: ");
    Serial.print(temperature);
    Serial.println(" *C");

    unsigned int voltage = mesureTension();
    envoyerTrame(voltage, temperature, humidity);
  }

  digitalWrite(DHT_POWER_SUPPLY_PIN, LOW);
}

unsigned int mesureTension() {
  digitalWrite(MICROPROCESOR_VOLTAGE_CAN_PULL_UP_PIN, HIGH);
  delay(10);
  int sensorValue = analogRead(MICROPROCESOR_VOLTAGE_CAN_MEASUREMENT);
  digitalWrite(MICROPROCESOR_VOLTAGE_CAN_PULL_UP_PIN, LOW);

  float voltage = sensorValue * (1.1 / 1023.0) * ((47.0 + 10.0) / 10.0);
  return (unsigned int)(voltage * 1000); // Convertir en millivolts
}

void envoyerTrame(unsigned int voltage, float temperature, float humidity) {
  unsigned int temp = (unsigned int)(temperature * 100);
  unsigned int hum = (unsigned int)(humidity * 100);

  char trame[13];
  sprintf(trame, "AT$SF=%04X%04X%04X", voltage, temp, hum);

  Serial.println(trame);
  delay(100);
  Serial.println("AT$P=2"); // Mettre le module en veille
  delay(10000); // Attendre 10 secondes avant la prochaine mesure
}