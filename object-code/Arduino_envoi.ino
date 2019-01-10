#include <SPI.h>
#include <mrf24j.h>
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
#include "structureGPS.h"
#include <MQ135.h>

//=======================================================================================================================================================
//PARTIE INITIALISATION DES VARIABLES
//=======================================================================================================================================================

//Initialisation des variables GPS
HardwareSerial mySerial = Serial3;
Adafruit_GPS GPS(&mySerial);

boolean usingInterrupt = false;
void useInterrupt(boolean); 
uint32_t timer = millis();

coord gpsfinal;


//Initialisation des variables de CO2
int airquality = 0;

//Initialisation des pin xbee
const int pin_reset = 49;
const int pin_cs = 53; // default CS pin on ATmega8/168/328
const int pin_interrupt = 10; // default interrupt pin on ATmega8/168/328

Mrf24j mrf(pin_reset, pin_cs, pin_interrupt);

long last_time;
long last_time2;
long tx_interval = 2000;

//=======================================================================================================================================================
//PARTIE SETUP
//=======================================================================================================================================================
void setup() {
  Serial.begin(115200);
  
  //Section pour les xbee
  mrf.reset();
  mrf.init();
  mrf.set_pan(0xcafe);
  // This is _our_ address
  mrf.address16_write(0x4204); 
  attachInterrupt(0, interrupt_routine, CHANGE); // interrupt 0 equivalent to pin 2(INT0) on ATmega8/168/328
  last_time = millis();
  interrupts();
  
  //Initialisation de la communication avec le module GPS
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  useInterrupt(true);

  
  //Délais de sécurité avant de commencer
  Serial.print("Veuillez patienter pendant l'initialisation du programme ");
  delay(500);
  Serial.print(".");
  delay(500);
  Serial.print(".");
  delay(500);
  Serial.println(".");
  mySerial.println(PMTK_Q_RELEASE);

}

//=======================================================================================================================================================
//PARTIE INTERRUPTION 
//=======================================================================================================================================================

//Interruption pour les modules Xbee
void interrupt_routine() {
    mrf.interrupt_handler(); // mrf24 object interrupt routine
}
void handle_rx() {
}

void handle_tx() {
    if (mrf.get_txinfo()->tx_ok) {
        Serial.println("Transmission OK, got ack");
    } else {
        Serial.print("La transmission à echouée apres : ");Serial.print(mrf.get_txinfo()->retries);Serial.println(" essais\n");
    }
}

//Interruption pour la partie GPS
SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
}

void useInterrupt(boolean v) {
  if (v) {
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
  } else {
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
  }
}

//=======================================================================================================================================================
//PARTIE FONCTIONS
//=======================================================================================================================================================

//Fonctions qui récupere les coords GPS
void recupGPS(){
   if (GPS.newNMEAreceived()) {
    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
  }

  // if millis() or timer wraps around, we'll just reset it
  if (timer > millis())  timer = millis();

  if (millis() - timer > 2000) { 
    timer = millis(); // reset the timer
    
    Serial.print("Fix du module GPS: "); 
    Serial.println((int)GPS.fix);
    if (GPS.fix) {
     gpsfinal.fix = GPS.fix;
     gpsfinal.latitude = GPS.latitudeDegrees;
     gpsfinal.longitude = GPS.longitudeDegrees;
    }
  }
} 

//Fonction qui affiche les coord GPS dans le term
void affichageGPS(coord gpsaff){
  if(gpsaff.fix == 1){
   Serial.print("Coordonnees (works with Google Maps): ");
   Serial.print(gpsaff.latitude, 4);
   Serial.print(", "); 
   Serial.println(gpsaff.longitude, 4);
  }
  else{
   Serial.println("Probleme avec l'obtention du fix");
   Serial.print("Coordonnees (works with Google Maps): ");
   Serial.print(gpsaff.latitude, 4);
   Serial.print(", "); 
   Serial.println(gpsaff.longitude, 4);
  }
   
}

//Fonctions qui récupère les données CO2
int CO2() {
  int sensorValue;
  sensorValue = analogRead(A1);
  return sensorValue;
}

//Fonction qui affiche les données CO2
void afficheCO2(int Airqua){
  Serial.print("Air Quality :");
  Serial.print(Airqua);
  Serial.print("*PPM");
  Serial.println(); 
}

//Fonction qui envoie les données des capteurs a l'arduno connecté au beaglebone
void Send(int qualiteAir, coord gpsfinal){
    const char* msgChar;
    String msgString;
    
   msgString += FloatToString(gpsfinal.latitude); 
      
    msgString = String(qualiteAir) + "/" + FloatToString(gpsfinal.latitude) + "/" + 
      FloatToString(gpsfinal.longitude) + "/" + String(gpsfinal.fix);
    Serial.print("msgSTring : ");
    Serial.println(msgString);
      
    msgChar = msgString.c_str();
    mrf.send16(0x4205, (char *)msgChar);
    
    Serial.println("envoi ...");
    Serial.print("Message envoye : ");
    Serial.println((char *)msgChar);
}

//Fonction qui convertie un Float en String
 String FloatToString(float val){
  char aCharTab [10];
  String motFinal = "";
  
  dtostrf(val, 9, 4, aCharTab);
  motFinal += aCharTab;
  motFinal.trim();
  
  return motFinal;
}

//=======================================================================================================================================================
//PARTIE MAIN
//=======================================================================================================================================================


void loop() {
  
   //Récupération et affichage des infos CO2 et GPS
   Serial.println("=====================================");
   airquality = CO2();
   afficheCO2(airquality);
   recupGPS();
   affichageGPS(gpsfinal);
   
    //Flags des interruptions pour la communication Xbee
    mrf.check_flags(&handle_rx, &handle_tx);
    unsigned long current_time = millis();
    if (current_time - last_time > tx_interval) {
        last_time = current_time;
        
        //Envoi des informations a l'arduino suivant
        Send(airquality, gpsfinal);     
  }
  delay(500);
}


