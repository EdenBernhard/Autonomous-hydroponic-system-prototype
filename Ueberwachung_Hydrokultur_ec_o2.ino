#include <Arduino.h>
#include <Stepper.h>
#include <PubSubClient.h>
#include <WiFi.h>
#include <DFRobot_ESP_EC.h>

#define EC_PIN 34
#define DO_PIN 35

//Variablen für die Berechnung des Sauerstoffgehalts in der Nährlösung 
#define VREF 5000    //VREF (mv)
#define ADC_RES 4095 //ADC Resolution
#define READ_TEMP 25 //Wassertemperatur

#define CAL1_V 2665 //Justierungswert
#define CAL1_T 25   //Temperatur

float voltage,ecValue,temperature = 25; // Variablen für die Berechnung des EC-Werts
int anzVoltage = 0;
DFRobot_ESP_EC ec; //Objekt der Bibliothek erstellen

//Variablen für die Berechnung des Sauerstoffgehalts
int ADC_Raw;
int ADC_Voltage;
int DO;
int o2Value;
int anzADC;

int zeit;

//Array mit festgelegten Werten zur Berechnung des Sauertsoffgehalts
const int DO_Table[41] = {
    14460, 14220, 13820, 13440, 13090, 12740, 12420, 12110, 11810, 11530,
    11260, 11010, 10770, 10530, 10300, 10080, 9860, 9660, 9460, 9270,
    9080, 8900, 8730, 8570, 8410, 8250, 8110, 7960, 7820, 7690,
    7560, 7430, 7300, 7180, 7070, 6950, 6840, 6730, 6630, 6530, 6410 };

//Variablen für WiFi-Verbindung
const char* ssid = "FRITZ!Box Fon WLAN 7320";
const char* wifi_password = "%1Wadehadedudeda#$2023";

//Variablen für MQTT-Verbindung
const char* mqtt_server = "raspberrypi";
const char* mqtt_username = "eden";
const char* mqtt_password = "hydrokultur";
const char* conductivity_topic = "home/hydrokultur/leitfähigkeit";
const char* oxygen_topic = "home/hydrokultur/sauerstoff";
const char* clientID = "client_hydrokultur_ec_o2";

//Erstellen von WiFi und MQTT Objekten
WiFiClient wifiClient;
PubSubClient client(mqtt_server, 1883, wifiClient); 

const int stepsPerRevolution = 400; //Anzahl der Schritte pro Umdrehung

//Erstellen von Stepper-Objekten für Ansteuerung der Schrittmotoren 
Stepper myStepper_Duenger(stepsPerRevolution, 27, 26, 25, 33);

void setup() {
  Serial.begin(115200); //Baudrate für die serielle Kommunikation auf 9600

  //GPIO-Pin als Dateneingang festlegen
  pinMode (EC_PIN, INPUT); 
  pinMode (DO_PIN, INPUT);

  connect_WiFi(); //stellt Verbindung zum WiFi her

  myStepper_Duenger.setSpeed(30); //Drehgeschwindigkeit der Schrittmotoren festlegen 

  zeit = millis(); //Aktuelle Systemzeit in Variable speichern
}

void loop() {
  Serial.println(analogRead(EC_PIN));
  voltage = voltage+map(analogRead(EC_PIN)+100.0, 0, 4095, 0, 5000);
  anzVoltage = anzVoltage+1;

  ADC_Raw = ADC_Raw+analogRead(DO_PIN);
  anzADC = anzADC+1;

  if((zeit+10000)<=millis()){ // 30 Min. warten (18E5)

    connect_MQTT(); //stellt Verbindung zum Broker her
   
    berechne_EC(); //berechnet den Leitfähigkeitswert

    ADC_Voltage = int(VREF) * (ADC_Raw/anzADC) / ADC_RES;

    berechne_O2(ADC_Voltage); //berechnet den Sauerstoffgehalt

    sende_EC(); //sendet den Leitfähigkeitswert
    sende_O2(); //sendet den Sauerstoffwert

    steuere_Dosierpumpe(); //steuert die Dosierpumpe

    if(steuere_Dosierpumpe() == true){
      zeit = millis()-5000;  // Sorgt für ein Durchlaufen der Verzweigung nach nur 10 Min. (12E5)
    }else{
      zeit = millis();
    }

    schaltePumpeAus();

    client.disconnect(); //trennt die Verbindung zum Broker

    voltage = 0;
    anzVoltage = 0;
    ADC_Raw = 0;
    anzADC = 0;
  }
}

void connect_WiFi(){
  Serial.print("Connecting to ");
  Serial.println(ssid);

  // Connect to the WiFi
  WiFi.begin(ssid, wifi_password);

  // Wait until the connection has been confirmed before continuing
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
}

void connect_MQTT(){
  if (client.connect(clientID, mqtt_username, mqtt_password)) {
    Serial.println("Connected to MQTT Broker!");
  }
  else {
    Serial.println("Connection to MQTT Broker failed...");
  }
}

void berechne_EC(){
	ecValue = ec.readEC((voltage/anzVoltage), temperature); // Berechne EC-Wert aus Spannung und Tempertaur 
}

void berechne_O2(int voltage_mv){   
    o2Value = (voltage_mv * DO_Table[READ_TEMP] / CAL1_V);
}

void sende_EC(){
  if (client.publish(conductivity_topic, String(ecValue).c_str())) {
     Serial.println("Leitfähigkeitswert sent!");
  }else {
    Serial.println("Leitfähigkeitswert failed to send. Reconnecting to MQTT Broker and trying again");
    client.connect(clientID, mqtt_username, mqtt_password);
    delay(10);
    client.publish(conductivity_topic, String(ecValue).c_str());
  }
}

void sende_O2(){
  if (client.publish(oxygen_topic, String(o2Value).c_str())) {
     Serial.println("Sauerstoffwert sent!");
  }else {
    Serial.println("Sauerstoffwert failed to send. Reconnecting to MQTT Broker and trying again");
    client.connect(clientID, mqtt_username, mqtt_password);
    delay(10);
    client.publish(oxygen_topic, String(o2Value).c_str());
  }
}

bool steuere_Dosierpumpe(){
  if(ecValue > 0.5){
      Serial.println("Düngerzugabe!!");
      myStepper_Duenger.step(stepsPerRevolution);
      return true;
  }
  return false; 
}

void schaltePumpeAus(){
  //Duenger
  digitalWrite(27, LOW);
  digitalWrite(26, LOW);
  digitalWrite(25, LOW);
  digitalWrite(33, LOW);
}