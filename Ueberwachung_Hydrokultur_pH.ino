#include <Arduino.h>
#include <Stepper.h>
#include <PubSubClient.h>
#include <WiFi.h>

#define PH 34

//Variablen für WiFi-Verbindung
const char* ssid = "FRITZ!Box Fon WLAN 7320";
const char* wifi_password = "%1Wadehadedudeda#$2023";

//Variablen für MQTT-Verbindung
const char* mqtt_server = "raspberrypi";
const char* mqtt_username = "eden";
const char* mqtt_password = "hydrokultur";
const char* pH_topic = "home/hydrokultur/pH_Wert";
const char* clientID = "client_hydrokultur_pH";

//Erstellen von WiFi und MQTT Objekten
WiFiClient wifiClient;
PubSubClient client(mqtt_server, 1883, wifiClient); 

const int stepsPerRevolution = 400; //Anzahl der Schritte pro Umdrehung

//Erstellen von Stepper-Objekten für Ansteuerung der Schrittmotoren 
Stepper myStepper_PH_Hoch(stepsPerRevolution, 13, 12, 14, 27);
Stepper myStepper_PH_Runter(stepsPerRevolution, 26, 25, 33, 32);

//Deklarierung globaler Variablen
float pH_Value;
float pH_EndValue;
int zeit;
int anzPH_Werte;

void setup() {
  Serial.begin(9600); //Baudrate für die serielle Kommunikation auf 9600
  pinMode (PH, INPUT); //GPIO-Pin als Dateneingang festlegen

  connect_WiFi(); //stellt Verbindung zum WiFi her

  //Drehgeschwindigkeit der Schrittmotoren festlegen 
  myStepper_PH_Hoch.setSpeed(30);
  myStepper_PH_Runter.setSpeed(30);

  zeit = millis(); //Aktuelle Systemzeit in Variable speichern
}

void loop() {
  pH_Value = pH_Value+analogRead(PH);

  anzPH_Werte = anzPH_Werte+1; 
  
  if((zeit+10000)<=millis()){ //30 Min. warten (18E5)

    connect_MQTT(); //stellt Verbindung zum Broker her 
   
    berechne_PH(); //berechnet den pH-Wert Broker
    sende_PH(); //sende den pH-Wert an den 

    steuere_Dosierpumpe(); //Ansteuerung der Dosierpumpen

    if(steuere_Dosierpumpe() == true){
      zeit = millis()-5000;  //Sorgt für ein Durchlaufen der Verzweigung nach nur 10 Min. (12E5)
    }else{
      zeit = millis();
    }

    schaltePumpeAus();

    client.disconnect(); //trennt die Verbindung 
  
    pH_Value = 0; 
    anzPH_Werte = 0;
  }
}

void connect_WiFi(){
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, wifi_password); //Internetverbindung herstellen

  while (WiFi.status() != WL_CONNECTED) { //Wartet solange bis die Verbindung hergestellt wurde
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

void berechne_PH(){
    float pH_Durchschnitt = pH_Value/anzPH_Werte;
    pH_EndValue = (pH_Durchschnitt-2372.8)/-105.97;
}

void sende_PH(){
 if (client.publish(pH_topic, String(pH_EndValue).c_str())) {
    Serial.println("pH-Wert sent!");
    }
    else {
      Serial.println("PH-Wert failed to send. Reconnecting to MQTT Broker and trying again");
      client.connect(clientID, mqtt_username, mqtt_password);
      delay(10); //Warten damit client.publish nicht zeitgleich mit client.connect ausgeführt wird 
      client.publish(pH_topic, String(pH_EndValue).c_str());
    }
}

boolean steuere_Dosierpumpe(){
  if(pH_EndValue < 6){
      Serial.println("PH-Hoch");
      myStepper_PH_Hoch.step(stepsPerRevolution);
      return true;
  }else if(pH_EndValue > 8){
      Serial.println("PH-Runter");
      myStepper_PH_Runter.step(stepsPerRevolution);
      return true;
  }
  return false; 
}

void schaltePumpeAus(){
  //pH_Hoch
  digitalWrite(13, LOW);
  digitalWrite(12, LOW);
  digitalWrite(14, LOW);
  digitalWrite(27, LOW);
  //pH_Runter
  digitalWrite(26, LOW);
  digitalWrite(25, LOW);
  digitalWrite(33, LOW);
  digitalWrite(32, LOW);
}

