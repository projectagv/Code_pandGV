#include <Wire.h>
#include "VL53L0X.h"
#include "TimerFreeTone.h"

/* 
   De MS1 MS2 en MS3 pins zijn om hlave stappen etc in te stellen, die gebruiken we dus niet want 1 stap is goed genoeg
   Er moet een condensator van 47 microFarad tussen de Vmot(VMotor, motorvoeding), en de bijhorende GND
   De 1A en 1B moeten op dezelfde coil zitten
   Met de dir kies je de richting, en door de step pin even hoog te zetten wordt er één stap gezet
   Reset en sleep moeten aan elkaar doorverbonden zijn
   De enable pin moet aan begin high gemaakt worden, zodat de stappenmotor geen rare dingen gaat doen
   Rechtsom draaien op de stappenmotor drivers is het limiet van stroom hoger leggen, linksom lager
*/

// startknop = A2
// volgAan = A0
// noodstop = A1

// Achterzijkant = Linksvoorpin arduino
// Boomsensor = Middenvoor arduino
// Zijkantvoor = zijkantvoor arduino
// Middenvoor = sensorBoom arduino
// Linksvoor = zijakntachter arduino
// rechtsvoor = rechtsvoor arduino

#define stapPinL 11      // Linker wiel
#define richtingPinL 12 
#define enablePinL 13

#define stapPinR 8          // verkeerd om Rechter wiel
#define richtingPinR 9      // verkeerd om
#define enablePinR 10       // verkeerd om

#define sensor0Pin 7    // Linksvoor
#define sensor1Pin 6    // Middenvoor
#define sensor2Pin 4    // Rechtsvoor
#define sensor3Pin 5    // Zijkantvoor
#define sensor4Pin 3    // Boom
#define sensor5Pin 2    // Zijkantachter

#define muziekpin1 0
#define muziekpin2 1

#define aanUitPin 13
#define volgAanUitPin 14
#define noodstop 15

#define sensor0I2C 10
#define sensor1I2C 11
#define sensor2I2C 12
#define sensor3I2C 13
#define sensor4I2C 14
#define sensor5I2C 15

#define kalibreren 0
#define tellen 1
#define volgen 2

// X is de lange kant, Y de korte
#define stappenHeleMatX 1330
#define stappenHeleMatY 660
#define aantalStappenPerRotatie 200
#define maxRPM 30
#define standaardRPM 10

#define maxAfwijkingZij 10
#define bochtAfwijkingZij 65
#define middenTotMiddenWielen 157
#define voorTotMidden 120

#define draaiCirkel middenTotMiddenWielen*PI
#define wielDiameter 75
#define afstandPerRotatie wielDiameter*PI
#define afstandPerStap afstandPerRotatie/200
#define afgelegdeHoekPerRotatie draaiCirkel/afstandPerRotatie //niet nuttig, tussenresultaat
#define aantalRotatiesPerGrade afgelegdeHoekPerRotatie/360
#define aantalStappenPerGrade aantalRotatiesPerGrade * aantalStappenPerRotatie

#define bijstuurTimerMax 1000 // de timer die overschreden moet worden om opnieuw bij te sturen
#define snelheidAfwijkingConstante 2  // het aantal rpm verandering dat we willen
#define volgAfstand 80  // de afstand tussen agv en volgpersoon
#define boomafstand 150// de afstand tussen agv en boom voor tellen
#define beepboom 1 //beep voor boom
#define beeppersoon 2 //beep voor persoon
#define beepvolgaan 3 //beep voor volgmodus aan
#define beepvolguit 4 //beep voor volgmodus uit
#define beepPersoon2 5
#define victoryBeep 6

#define bochtRPM standaardRPM/2
#define afstandsFactorZijkant 0.97357
#define zijkantAfstandOptimum 60
#define medewerkerAfstand 150
#define lengteAfwijkingSensor 16
#define volgAfwijking 20

// Frequentie van muzieknoten
#define NOTE_G5 784
#define NOTE_A5 880
#define NOTE_B5 988

#define startKnop A2
#define volgKnop A0
#define noodstopKnop A1
// startknop = A2
// volgAan = A0
// noodstop = A1

// Muziek defines, periode is de periode van een 4maat in ms, 
#define periode     857
#define heel      periode
#define half      periode / 2
#define kwart     periode / 4
#define achtste     periode / 8
#define zestiende     periode / 16

#define sensorTimeBudget 40000
#define sensorTimeout 200
#define timerFactor 20


VL53L0X sensorLinksVoor;
VL53L0X sensorMiddenVoor;
VL53L0X sensorRechtsVoor;
VL53L0X sensorZijkantVoor;
VL53L0X sensorBoom;
VL53L0X sensorZijkantAchter;

volatile float timerFlag = 0;
float stapLTimer = 1;
float stapRTimer = 1;
float LRPM = standaardRPM;
float RRPM = standaardRPM;
float bijstuurTimer = 1;

unsigned long aantalStappenL = 0;
unsigned long aantalStappenR = 0;
int boomTeller = 0;
bool boomBezig = false;
bool openingBezig = false;
int8_t openingTeller = 0;

uint8_t bochtStatus = 0;
unsigned long beginBochtStappen = 0;

int8_t switchVar = 1;
uint8_t switchTellen = 0;

volatile float afstandLinksVoor = 0;
volatile float afstandMiddenVoor = 0;
volatile float afstandRechtsVoor = 0;
volatile float afstandZijkantVoor = 0;
volatile float afstandBoom = 0;
volatile float afstandZijkantAchter = 0;

volatile int8_t sensorTellerVoor = 0;
volatile int8_t sensorTellerZij = 0;
volatile unsigned long sensorVoorTemp = 0;
volatile unsigned long sensorZijTemp = 0;

uint16_t huidigeHoek = 0;
uint32_t stopTimer = 0;
volatile bool begonnen = false;
bool volgModus = false;
bool kalibrerenKlaar = true;
bool richting = false;

bool bochtDingen(uint16_t, int16_t);
void bijsturen(uint16_t, uint16_t, uint16_t);
void dobeep(int8_t);
bool leesKnop(int);
void volgenBijsturen(int,int,int);


void setup() {

	uint8_t sensorPinnen[] = {sensor0Pin, sensor1Pin, sensor2Pin, sensor3Pin, sensor4Pin, sensor5Pin};
	uint8_t sensorI2C[] = {sensor0I2C, sensor1I2C, sensor2I2C, sensor3I2C, sensor4I2C, sensor5I2C};

	for (uint8_t i=0; i<8; i++){
		pinMode(sensorPinnen[i], OUTPUT);
		digitalWrite(sensorPinnen[i], LOW);
	}
  
	Wire.begin();
  
	// Instellen sensoren
	pinMode(sensorPinnen[0], INPUT);
	delay(10);
	sensorLinksVoor.init();
	delay(10);
	sensorLinksVoor.setTimeout(200);
	delay(10);
	sensorLinksVoor.setAddress(sensorI2C[0]);
	delay(10);
	sensorLinksVoor.setMeasurementTimingBudget(sensorTimeBudget); //(20 ms, default is 33 ms)
	delay(10);

	pinMode(sensorPinnen[1], INPUT);
	delay(10);
	sensorMiddenVoor.init(true);
	delay(10);
	sensorMiddenVoor.setTimeout(200);
	delay(10);
	sensorMiddenVoor.setAddress(sensorI2C[1]);
	delay(10);
	sensorMiddenVoor.setMeasurementTimingBudget(sensorTimeBudget); //(20 ms, default is 33 ms)
	delay(10);

	pinMode(sensorPinnen[2], INPUT);
	delay(10);
	sensorRechtsVoor.init(true);
	delay(10);
	sensorRechtsVoor.setTimeout(200);
	delay(10);
	sensorRechtsVoor.setAddress(sensorI2C[2]);
	delay(10);
	sensorRechtsVoor.setMeasurementTimingBudget(sensorTimeBudget); //(20 ms, default is 33 ms)
	delay(10);

	pinMode(sensorPinnen[3], INPUT);
	delay(10);
	sensorZijkantVoor.init(true);
	delay(10);
	sensorZijkantVoor.setTimeout(200);
	delay(10);
	sensorZijkantVoor.setAddress(sensorI2C[3]);
	delay(10);
	sensorZijkantVoor.setMeasurementTimingBudget(sensorTimeBudget); //(20 ms, default is 33 ms)
	delay(10);

	pinMode(sensorPinnen[4], INPUT);
	delay(10);
	sensorBoom.init(true);
	delay(10);
	sensorBoom.setTimeout(200);
	delay(10);
	sensorBoom.setAddress(sensorI2C[4]);
	delay(10);
	sensorBoom.setMeasurementTimingBudget(sensorTimeBudget); //(20 ms, default is 33 ms)
	delay(10);

	pinMode(sensorPinnen[5], INPUT);
	delay(10);
	sensorZijkantAchter.init(true);
	delay(10);
	sensorZijkantAchter.setTimeout(200);
	delay(10);
	sensorZijkantAchter.setAddress(sensorI2C[5]);
	delay(10);
	sensorZijkantAchter.setMeasurementTimingBudget(sensorTimeBudget); //(20 ms, default is 33 ms)
	delay(10);

	// Instellen stappenmotorPinnen
	pinMode(stapPinL, OUTPUT);
	pinMode(stapPinR, OUTPUT); 
	pinMode(richtingPinL, OUTPUT);
	pinMode(richtingPinR, OUTPUT);
	pinMode(enablePinL, OUTPUT);
	pinMode(enablePinR, OUTPUT);
	digitalWrite(enablePinL, HIGH);
	digitalWrite(enablePinR, HIGH);
	digitalWrite(enablePinL, LOW);
	digitalWrite(enablePinR, LOW);

	// Instellen timerInterrupt op 1 ms
	// Timer2 want Timer0 is nodig voor de delay functies, en Timer1 is nodig voor servo's (niet gebruikt in dit project)
	TCCR2A = 0;// set entire TCCR0A register to 0
	TCCR2B = 0;// same for TCCR0B
	TCNT2  = 0;//initialize counter value to 0
	// set compare match register for 1khz increments
	OCR2A = (250/timerFactor - 1);// = (16*10^6) / (1000*64) - 1 (must be <256) 249 voor 1 ms, 49 voor 1/5 ms
    //  = (16*10^6)kloktijd / ((1000)frequentie in HZ, * (64)prescaler) - 1
	// turn on CTC mode
	TCCR2A |= (1 << WGM21);
	// Set CS01 and CS00 bits for 64 prescaler
	TCCR2B |= (1 << CS21) | (1 << CS20);   
	// enable timer compare interrupt
	TIMSK2 |= (1 << OCIE2A);

	sei();
  
	sensorLinksVoor.startSensor();
	sensorZijkantVoor.startSensor();

	while(1){
		if(leesKnop(startKnop)){
			switchVar = 0;
			break;
		}
		if(leesKnop(volgKnop)){
			switchVar = 1;
			break;
		}
		updateSensoren();
	}
	
	digitalWrite(enablePinL, LOW);
	digitalWrite(enablePinR, LOW);
	
	while(!leesKnop(startKnop) || !leesKnop(volgKnop)){
		updateSensoren();
	}
}

void loop() {
	switch(switchVar){
		case 0: //  Rijden, bomen tellen, stoppen voor medewerker, nog een keer indrukken om rijrichting om te draaien, kan niet stoppen voor medewerker bij het achteruit rijden
			// motoren aansturen
			if (RRPM == 0){ // Reset timer bij 0 rpm
				stapRTimer = 1;
			}

			if (stapRTimer > (1000/((RRPM/60)*200))){
				if (richting){
					digitalWrite(richtingPinR, HIGH);
				}
				else{
					digitalWrite(richtingPinR, LOW);
				}
				digitalWrite(stapPinR, HIGH);
				stapRTimer = 1;
				delayMicroseconds(200);
				digitalWrite(stapPinR, LOW);
				aantalStappenR++;
			}
			  
			if (LRPM == 0){ // zelfde
				stapLTimer = 1;
			}
			  
			if (stapLTimer > (1000/((LRPM/60)*200))){ // verkeerd om, daarom low
				if (richting){
					digitalWrite(richtingPinL, LOW);
				}
				else{
					digitalWrite(richtingPinL, HIGH);
				}
				digitalWrite(stapPinL, HIGH);
				stapLTimer = 1;
				delayMicroseconds(200);
				digitalWrite(stapPinL, LOW);
				aantalStappenL++;
			}
			// motoren aansturen
			
			// knoppen checken
			if(leesKnop(startKnop)){
				richting = !richting;
				while(leesKnop(startKnop)){
					updateSensoren();
				}
			}
			
			if(leesKnop(volgKnop)){
				switchVar = 1;
				while(leesKnop(volgKnop)){
					updateSensoren();
				}
			}
			
			if(leesKnop(noodstopKnop)){
				
				while(leesKnop(noodstopKnop)){
					updateSensoren();
				}
				while(!leesKnop(noodstopKnop)){
					updateSensoren();
				}
				while(leesKnop(noodstopKnop)){
					updateSensoren();
				}
			}
			// knoppen checken
			
			// boomdetectie
			if (afstandBoom < 200 && boomBezig == false){ // Als je een boom ziet die nog niet gezien is
				boomTeller++;
				dobeep(beepboom);
			}
        
			if (afstandBoom > 200){ // Detecteer einde van een boom
				boomBezig = false;
			}
			// boomdetectie
			
			
			if ((afstandLinksVoor < medewerkerAfstand || afstandMiddenVoor < medewerkerAfstand || afstandRechtsVoor < medewerkerAfstand || stopTimer != 0) && bochtStatus == 0){  // Evil pseudocode of death, please replace
			// Als 1 van de 3 sensoren voor een medewerker detecteerd binnen de minimale afstand OF we al aan het wachten waren (nodig voor reset) EN we ook niet al in een bocht zijn
				LRPM = 0;
				RRPM = 0; // stop
            
				if (stopTimer == 0){ // start timer als dit de eerste keer is
					stopTimer = 1;
					dobeep(beeppersoon); // Beep naar de persoon
				}
				
				if (stopTimer > 5000 && stopTimer < 6000){ // als we al 5 seconden wachten, beep urgenter
					dobeep(beepPersoon2);
				}
				
				if (stopTimer > 15000 && stopTimer < 16000){
					//////////////////////////////////////////////////////////////////////////////////////////////////
					RRPM = 0;
					LRPM = 0;
                
					for (uint16_t i=0; i<boomTeller; i++){ // Aantal getelde bomen beeps
						dobeep(beepboom);
						delay(1000);
					}
					delay(2000);
					dobeep(victoryBeep);
					stopTimer = 0;
                }
                
				if (afstandLinksVoor > medewerkerAfstand && afstandMiddenVoor > medewerkerAfstand && afstandRechtsVoor > medewerkerAfstand){ // Als de mederwerken niet meer gedetecteerd wordt, reset waardes
					stopTimer = 0;
					LRPM = standaardRPM;
					RRPM = standaardRPM;
				}
			}
		break;
		
		case 1: //  
		
			// knoppen checken
			if(leesKnop(startKnop)){
				switchVar = 0;
				while(leesKnop(startKnop)){
					updateSensoren();
				}
			}
			
			if(leesKnop(volgKnop)){
				switchVar = 1;
				while(leesKnop(volgKnop)){
					updateSensoren();
				}
				
				for(byte i=0; i<90; i++){ // draai 90 graden elke keer dat knop is ingedrukt
					bochtDingen(aantalStappenR - beginBochtStappen, 90);
				}
			}
			
			if(leesKnop(noodstopKnop)){
				
				while(leesKnop(noodstopKnop)){
					updateSensoren();
				}
				while(!leesKnop(noodstopKnop)){
					updateSensoren();
				}
				while(leesKnop(noodstopKnop)){
					updateSensoren();
				}
			}
			// knoppen checken
			
		break;
	}
}

void dobeep(int8_t beepkeuze){ // FIXED
  switch (beepkeuze){
    case beeppersoon:
     // beep beep
      TimerFreeTone(muziekpin1, NOTE_B5, kwart);
      delay(kwart);
      TimerFreeTone(muziekpin1, NOTE_B5, kwart);
    break;

    /*
    case beepboom:
      TimerFreeTone(muziekpin1, NOTE_B5, half);
      delay(half);
    break;
    */
    
    case beepvolgaan:
      // beep beeeeeeeeeeeepppp
      TimerFreeTone(muziekpin1, NOTE_B5, kwart);
      delay(kwart);
      TimerFreeTone(muziekpin1, NOTE_B5, heel);
    break;
    
    case beepvolguit:
      // beeeeeeeeeeeeeeeeppp beep
      TimerFreeTone(muziekpin1, NOTE_B5, heel);
      delay(kwart);
      TimerFreeTone(muziekpin1, NOTE_B5, kwart);
    break;
    
    case beepPersoon2:
      // beep beep beep beep beep beep beep beep
      for(uint8_t i=0; i< 8; i++){
        TimerFreeTone(muziekpin1, NOTE_B5, kwart);
        delay(kwart);
      }
    break;
    
    case victoryBeep:
      // Final fantasy tune
      TimerFreeTone(muziekpin1, NOTE_B5, zestiende);;
      delay(zestiende);
      TimerFreeTone(muziekpin1, NOTE_B5, zestiende);
      delay(zestiende);
      TimerFreeTone(muziekpin1, NOTE_B5, zestiende);
      delay(zestiende);
      TimerFreeTone(muziekpin1, NOTE_B5, half);
      TimerFreeTone(muziekpin1, NOTE_G5, half);
      TimerFreeTone(muziekpin1, NOTE_A5, half);
      TimerFreeTone(muziekpin1, NOTE_B5, achtste);
      delay(kwart);
      TimerFreeTone(muziekpin1, NOTE_A5, achtste);
      TimerFreeTone(muziekpin1, NOTE_B5, heel);
    break;
  }
}

void updateSensoren(){
  switch(sensorTellerVoor){
    case 0:
      if(sensorLinksVoor.checkSensorData(sensorVoorTemp) == 0){
        afstandLinksVoor = sensorLinksVoor.leesSensor();
        sensorTellerVoor = 1;
        sensorVoorTemp = sensorMiddenVoor.startSensor();
      }
      else if (sensorVoorTemp > sensorTimeout){
        sensorLinksVoor.leesSensor();
        sensorTellerVoor = 1;
        sensorVoorTemp = sensorMiddenVoor.startSensor();
      }
    break;
    
    case 1:
      if(sensorMiddenVoor.checkSensorData(sensorVoorTemp) == 0){
        afstandMiddenVoor = sensorMiddenVoor.leesSensor();
        sensorTellerVoor = 2;
        sensorVoorTemp = sensorRechtsVoor.startSensor();
      }
      else if (sensorVoorTemp > sensorTimeout){
        sensorMiddenVoor.leesSensor();
        sensorTellerVoor = 2;
        sensorVoorTemp = sensorRechtsVoor.startSensor();
      }
    break;

    case 2:
      if(sensorRechtsVoor.checkSensorData(sensorVoorTemp) == 0){
        afstandRechtsVoor = sensorRechtsVoor.leesSensor();
        sensorTellerVoor = 0;
        sensorVoorTemp = sensorLinksVoor.startSensor();
      }
      else if (sensorVoorTemp > sensorTimeout){
        sensorRechtsVoor.leesSensor();
        sensorTellerVoor = 0;
        sensorVoorTemp = sensorLinksVoor.startSensor();
      }
    break;
  }

  switch(sensorTellerZij){
    case 0:
      if(sensorZijkantVoor.checkSensorData(sensorZijTemp) == 0){
        afstandZijkantVoor = sensorZijkantVoor.leesSensor();
        sensorTellerZij = 1;
        sensorZijTemp = sensorBoom.startSensor();
      }
      else if (sensorZijTemp > sensorTimeout){
        sensorZijkantVoor.leesSensor();
        sensorTellerZij = 1;
        sensorZijTemp = sensorBoom.startSensor();
      }
    break;
    
    case 1:
      if(sensorBoom.checkSensorData(sensorZijTemp) == 0){
        afstandBoom = sensorBoom.leesSensor();
        sensorTellerZij = 2;
        sensorZijTemp = sensorZijkantAchter.startSensor();
      }
      else if (sensorZijTemp > sensorTimeout){
        sensorBoom.leesSensor();
        sensorTellerZij = 2;
        sensorZijTemp = sensorZijkantAchter.startSensor();
      }
    break;

    case 2:
      if(sensorZijkantAchter.checkSensorData(sensorZijTemp) == 0){
        afstandZijkantAchter = sensorZijkantAchter.leesSensor() - 15;
        sensorTellerZij = 0;
        sensorZijTemp = sensorZijkantVoor.startSensor();
      }
      else if (sensorZijTemp > sensorTimeout){
        sensorZijkantAchter.leesSensor();
        sensorTellerZij = 0;
        sensorZijTemp = sensorZijkantVoor.startSensor();
      }
    break;
  }
}

bool bochtDingen(uint16_t aantalStappenGedaan = 0, int16_t hoek = 90){
  
  if (aantalStappenGedaan >= sqrt(pow(hoek*aantalStappenPerGrade, 2))){
    return true;
  }
  
  if (hoek > -100 && hoek > 0){
    digitalWrite(richtingPinL, LOW); // verkeerd om, daarom low
    digitalWrite(stapPinL, HIGH);
    delayMicroseconds(500);
    digitalWrite(stapPinL, LOW);
    delayMicroseconds(9500);
  }
  else if (hoek < 100 && hoek < 0){
    digitalWrite(richtingPinR, HIGH);
    digitalWrite(stapPinR, HIGH);
    delayMicroseconds(500);
    digitalWrite(stapPinR, LOW);
    delayMicroseconds(9500);
  }
  else if (hoek == 180){
    digitalWrite(richtingPinR, HIGH);
    digitalWrite(richtingPinL, HIGH); // verkeerd om, daarom high
    digitalWrite(stapPinR, HIGH);
    digitalWrite(stapPinL, HIGH);
    delayMicroseconds(500);
    digitalWrite(stapPinR, LOW);
    digitalWrite(stapPinL, LOW);
    delayMicroseconds(9500);
  }
  
  return false;
}

bool leesKnop(uint8_t knopPin) {
  for(int8_t i=0; i<8; i++){
    if (digitalRead(knopPin) != 0){
      return(false);
    }
  }
  return(true);
}

ISR(TIMER2_COMPA_vect){
  timerFlag++;
}
