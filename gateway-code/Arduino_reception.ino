#include <mrf24j.h>
#include <SPI.h>


//=======================================================================================================================================================
//PARTIE VARIABLES D'ENVIRONNEMENT
//=======================================================================================================================================================
const int pin_reset = 48;
const int pin_cs = 46; // default CS pin on ATmega8/168/328
const int pin_interrupt = 11; // default interrupt pin on ATmega8/168/328
int essai = 0;
String msgCO2;

HardwareSerial mySerial = Serial3;

Mrf24j mrf(pin_reset, pin_cs, pin_interrupt);

//=======================================================================================================================================================
//PARTIE SETUP
//=======================================================================================================================================================
void setup() {
  Serial.begin(115200);
  mySerial.begin(115200);
  
  mrf.reset();
  mrf.init();
  
  mrf.set_pan(0xcafe);
  // This is _our_ address
  mrf.address16_write(0x4205);
  
  attachInterrupt(0, interrupt_routine, CHANGE); // interrupt 0 equivalent to pin 2(INT0) on ATmega8/168/328
  interrupts();
}


//=======================================================================================================================================================
//INTERRUPTIONS
//=======================================================================================================================================================

void interrupt_routine() {
  mrf.interrupt_handler(); // mrf24 object interrupt routine
}

void handle_rx() {
    essai = essai +1;
    Serial.println("====================================");
    Serial.print("Essai numero : ");
    Serial.println(essai);
    
    Serial.print("\r\nMESSAGE RECU PAR LA LIAISON : ");
    for (int i = 0; i < mrf.rx_datalength(); i++) {
        Serial.write(mrf.get_rxinfo()->rx_data[i]);
        mySerial.write(mrf.get_rxinfo()->rx_data[i]);
       // msgCO2 = msgCO2 + mrf.get_rxinfo()->rx_data[i];
    }
    mySerial.println();
    Serial.println();
    Serial.println("FIN DE L'ENVOI A BEN");
    
    
    Serial.print("\r\nLQI/RSSI=");
    Serial.print(mrf.get_rxinfo()->lqi, DEC);
    Serial.print("/");
    Serial.println(mrf.get_rxinfo()->rssi, DEC);
}

void handle_tx() {
    // code to transmit, nothing to do
}

//=======================================================================================================================================================
//MAIN
//=======================================================================================================================================================
void loop() {
    mrf.check_flags(&handle_rx, &handle_tx);
}

