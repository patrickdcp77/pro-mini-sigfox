#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>

// Déclaration de la variable de débogage
boolean DEBUG_MODE = 1; // =1 pour activer le débogage, =0 pour désactiver

#define DHTPIN 11     // Pin où le pin DATA du DHT22 est connecté
#define DHTTYPE DHT22 // Définir le type de DHT
#define DHT_POWER_SUPPLY_PIN 10 // Pin pour alimenter le DHT22

#define MICROPROCESOR_VOLTAGE_CAN_MEASUREMENT A0 
#define MICROPROCESOR_VOLTAGE_CAN_PULL_UP_PIN A1

DHT dht(DHTPIN, DHTTYPE);

// Déclaration des fonctions
void lectureDHT(float &temperature, float &humidity);
unsigned int mesureTension();
String creerTrameSigfox(unsigned int vcc, float temperature, float humidity);

void setup() {
  // Initialiser la communication série
  Serial.begin(9600);
  
  // Configurer les pins
  pinMode(DHT_POWER_SUPPLY_PIN, OUTPUT);
  pinMode(MICROPROCESOR_VOLTAGE_CAN_PULL_UP_PIN, OUTPUT);
  digitalWrite(DHT_POWER_SUPPLY_PIN, LOW); // Éteindre le capteur au début
  
  // Initialiser le capteur DHT
  dht.begin();
}

void loop() {
  float temperature, humidity;
  lectureDHT(temperature, humidity);
  unsigned int vcc = mesureTension();
  String trame = creerTrameSigfox(vcc, temperature, humidity);
  delay(5000);
}

void lectureDHT(float &temperature, float &humidity) {
  // Allumer le capteur
  digitalWrite(DHT_POWER_SUPPLY_PIN, HIGH);
  
  // Attendre un peu pour que le capteur soit prêt
  delay(2000);
  
  // Lire les données du capteur
  humidity = dht.readHumidity();
  temperature = dht.readTemperature();
  
  // Vérifier si la lecture a échoué
  if (isnan(humidity) || isnan(temperature)) {
    if (DEBUG_MODE) {
      Serial.println("Échec de lecture du capteur DHT22 !");
    }
  } else {
    // Afficher les valeurs lues
    if (DEBUG_MODE) {
      Serial.print("Humidité: ");
      Serial.print(humidity);
      Serial.print(" %\t");
      Serial.print("Température: ");
      Serial.print(temperature);
      Serial.println(" *C");
    }
  }
  
  // Éteindre le capteur
  digitalWrite(DHT_POWER_SUPPLY_PIN, LOW);
}

unsigned int mesureTension() {
  digitalWrite(MICROPROCESOR_VOLTAGE_CAN_PULL_UP_PIN, HIGH);
  delay(10);
  int sensorValue = analogRead(MICROPROCESOR_VOLTAGE_CAN_MEASUREMENT);
  digitalWrite(MICROPROCESOR_VOLTAGE_CAN_PULL_UP_PIN, LOW);

  // Calculer la tension mesurée en millivolts
  float measuredVoltage = sensorValue * (1.1 / 1023.0) * ((47.0 + 10.0) / 10.0);
  unsigned int voltage_mV = (unsigned int)(measuredVoltage * 1000); // Convertir en millivolts

  // Utiliser une règle de trois pour calculer VCC
  // Le rapport est 4910 / 1084
  float ratio = 4910.0 / 1084.0;
  float vcc = voltage_mV * ratio;
  unsigned int vcc_mV = (unsigned int)(vcc); // Convertir en millivolts

  // Afficher la tension mesurée et VCC
  if (DEBUG_MODE) {
    Serial.print("Tension mesurée: ");
    Serial.print(voltage_mV);
    Serial.println(" mV");
    Serial.print("VCC déduit: ");
    Serial.print(vcc_mV);
    Serial.println(" mV");
  }

  return vcc_mV;
}

String creerTrameSigfox(unsigned int vcc, float temperature, float humidity) {
  char trame[13];
  unsigned int temp = (unsigned int)(temperature * 100);
  unsigned int hum = (unsigned int)(humidity * 100);
  sprintf(trame, "%04X%04X%04X", vcc, temp, hum);
  
  // Afficher la trame Sigfox
  if (DEBUG_MODE) {
    Serial.print("Trame Sigfox: ");
    Serial.println(trame);
  }
  
  return String(trame);
}