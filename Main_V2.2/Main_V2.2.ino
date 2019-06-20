#include <Wire.h>
#include "VL53L0X.h"

// dingen comment do stuff
// BRAAAAAAAAAAAAAAAAAAAAAAAAAANCCCCCCCCCCCCCCCCCCCCCCCHHHHHHHHHHHHHHHHHHHHHHHHHHHHH

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

#define stapPinL 11			// Linker wiel
#define richtingPinL 12 
#define enablePinL 13

#define stapPinR 8          // verkeerd om Rechter wiel
#define richtingPinR 9      // verkeerd om
#define enablePinR 10       // verkeerd om

#define sensor0Pin 7		// Linksvoor
#define sensor1Pin 6		// Middenvoor
#define sensor2Pin 4		// Rechtsvoor
#define sensor3Pin 5		// Zijkantvoor
#define sensor4Pin 3		// Boom
#define sensor5Pin 2		// Zijkantachter
#define sensor6Pin 16
#define sensor7Pin 17

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
#define sensor6I2C 16
#define sensor7I2C 17

#define kalibreren 0
#define tellen 1
#define volgen 2

// X is de lange kant, Y de korte
#define stappenHeleMatX 1330
#define stappenHeleMatY 660
#define aantalStappenPerRotatie 200
#define maxRPM 40
#define standaardRPM 30

#define maxAfwijkingZij 10
#define bochtAfwijkingZij 400
#define middenTotMiddenWielen 157
#define voorTotMidden 120

#define draaiCirkel middenTotMiddenWielen*PI
#define wielDiameter 75
#define afstandPerRotatie wielDiameter*PI
#define afstandPerStap afstandPerRotatie/200
#define afgelegdeHoekPerRotatie draaiCirkel/afstandPerRotatie //niet nuttig, tussenresultaat
#define aantalRotatiesPerGrade afgelegdeHoekPerRotatie/360
#define aantalStappenPerGrade aantalRotatiesPerGrade * aantalStappenPerRotatie

#define bijstuurTimerMax 400 // de timer die overschreden moet worden om opnieuw bij te sturen
#define snelheidAfwijkingConstante 2  // het aantal rpm verandering dat we willen per afwijking in mm aan de zijkant
#define volgAfstand 80  // de afstand tussen agv en volgpersoon
#define boomafstand // de afstand tussen agv en boom voor tellen
#define beepboom 1 //beep voor boom
#define beeppersoon 2 //beep voor persoon
#define beepvolgaan 3 //beep voor volgmodus aan
#define beepvolguit 4 //beep voor volgmodus uit
#define beepPersoon2 5
#define victoryBeep 6

#define bochtRPM 20
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
#define noodstop A1
// startknop = A2
// volgAan = A0
// noodstop = A1

// Muziek defines, periode is de periode van een 4maat in ms, 
#define periode 		857
#define heel			periode
#define half			periode / 2
#define kwart			periode / 4
#define achtste			periode / 8
#define zestiende 		periode / 16


VL53L0X sensorLinksVoor;
VL53L0X sensorMiddenVoor;
VL53L0X sensorRechtsVoor;
VL53L0X sensorZijkantVoor;
VL53L0X sensorBoom;
VL53L0X sensorZijkantAchter;

/*
VL53L0X sensor6;
VL53L0X sensor7;
*/


volatile float timerFlag = 0;
float stapLTimer = 1;
float stapRTimer = 1;
float LRPM = 0;
float RRPM = 0;
int bijstuurTimer = 1;

unsigned long aantalStappenL = 0;
unsigned long aantalStappenR = 0;
int knopvolgen = 0;
int knopaan = 0;
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

bool bochtDingen(uint16_t, int16_t);
void bijsturen(uint16_t, uint16_t, uint16_t);
void dobeep(int8_t);
bool leesKnop(int);
void volgenBijsturen(int,int,int);


void setup() {

  uint8_t sensorPinnen[] = {sensor0Pin, sensor1Pin, sensor2Pin, sensor3Pin, sensor4Pin, sensor5Pin, sensor6Pin, sensor7Pin};
  uint8_t sensorI2C[] = {sensor0I2C, sensor1I2C, sensor2I2C, sensor3I2C, sensor4I2C, sensor5I2C, sensor6I2C, sensor7I2C};

  for (uint8_t i=0; i<8; i++){
    pinMode(sensorPinnen[i], OUTPUT);
    digitalWrite(sensorPinnen[i], LOW);
  }
  
  Wire.begin();

  dobeep(beeppersoon);

  // Instellen sensoren
  pinMode(sensorPinnen[0], INPUT);
  delay(10);
  sensorLinksVoor.init();
  delay(10);
  sensorLinksVoor.setTimeout(200);
  delay(10);
  sensorLinksVoor.setAddress(sensorI2C[0]);
  delay(10);
  sensorLinksVoor.setMeasurementTimingBudget(35000); //(20 ms, default is 33 ms)
  delay(10);

  pinMode(sensorPinnen[1], INPUT);
  delay(10);
  sensorMiddenVoor.init(true);
  delay(10);
  sensorMiddenVoor.setTimeout(200);
  delay(10);
  sensorMiddenVoor.setAddress(sensorI2C[1]);
  delay(10);
  sensorMiddenVoor.setMeasurementTimingBudget(35000); //(20 ms, default is 33 ms)
  delay(10);

  pinMode(sensorPinnen[2], INPUT);
  delay(10);
  sensorRechtsVoor.init(true);
  delay(10);
  sensorRechtsVoor.setTimeout(200);
  delay(10);
  sensorRechtsVoor.setAddress(sensorI2C[2]);
  delay(10);
  sensorRechtsVoor.setMeasurementTimingBudget(35000); //(20 ms, default is 33 ms)
  delay(10);

  pinMode(sensorPinnen[3], INPUT);
  delay(10);
  sensorZijkantVoor.init(true);
  delay(10);
  sensorZijkantVoor.setTimeout(200);
  delay(10);
  sensorZijkantVoor.setAddress(sensorI2C[3]);
  delay(10);
  sensorZijkantVoor.setMeasurementTimingBudget(35000); //(20 ms, default is 33 ms)
  delay(10);

  pinMode(sensorPinnen[4], INPUT);
  delay(10);
  sensorBoom.init(true);
  delay(10);
  sensorBoom.setTimeout(200);
  delay(10);
  sensorBoom.setAddress(sensorI2C[4]);
  delay(10);
  sensorBoom.setMeasurementTimingBudget(35000); //(20 ms, default is 33 ms)
  delay(10);

  pinMode(sensorPinnen[5], INPUT);
  delay(10);
  sensorZijkantAchter.init(true);
  delay(10);
  sensorZijkantAchter.setTimeout(200);
  delay(10);
  sensorZijkantAchter.setAddress(sensorI2C[5]);
  delay(10);
  sensorZijkantAchter.setMeasurementTimingBudget(35000); //(20 ms, default is 33 ms)
  delay(10);

/*
  pinMode(sensorPinnen[6], INPUT);
  delay(10);
  sensor6.init(true);
  delay(10);
  sensor6.setTimeout(200);
  delay(10);
  sensor6.setAddress(sensorI2C[6]);
  delay(10);
  sensor6.setMeasurementTimingBudget(40000); //(20 ms, default is 33 ms)
  delay(10);

  pinMode(sensorPinnen[7], INPUT);
  delay(10);
  sensor7.init(true);
  delay(10);
  sensor7.setTimeout(200);
  delay(10);
  sensor7.setAddress(sensorI2C[7]);
  delay(10);
  sensor7.setMeasurementTimingBudget(40000); //(20 ms, default is 33 ms)
  delay(10);
*/

  // Instellen stappenmotorPinnen
  pinMode(stapPinL, OUTPUT);
  pinMode(stapPinR, OUTPUT); 
  pinMode(richtingPinL, OUTPUT);
  pinMode(richtingPinR, OUTPUT);
  pinMode(enablePinL, OUTPUT);
  pinMode(enablePinR, OUTPUT);
  digitalWrite(enablePinL, HIGH);
  digitalWrite(enablePinR, HIGH);
  delay(2000);
  digitalWrite(enablePinL, LOW);
  digitalWrite(enablePinR, LOW);

  // Instellen timerInterrupt op 1 ms
  TCCR0A = 0;// set entire TCCR0A register to 0
  TCCR0B = 0;// same for TCCR0B
  TCNT0  = 0;//initialize counter value to 0
  // set compare match register for 1khz increments
  OCR0A = 62;// = (16*10^6) / (1000*64) - 1 (must be <256) 249 voor 1 ms
  // turn on CTC mode
  TCCR0A |= (1 << WGM01);
  // Set CS01 and CS00 bits for 64 prescaler
  TCCR0B |= (1 << CS01) | (1 << CS00);   
  // enable timer compare interrupt
  TIMSK0 |= (1 << OCIE0A);

  sei();
  
  sensorLinksVoor.startSensor();
  sensorZijkantVoor.startSensor();

  while(leesKnop(startKnop)){
    updateSensoren();
  }
}

void loop() {
	if (!leesKnop(noodstop)){ // Noodstop ingedrukt
		delay(20);
		if(!leesKnop(noodstop)){ // Double check
			while(!leesKnop(noodstop)){	// Zolang de knop ingedrukt is
			}
			while(leesKnop(noodstop)){ // Als de knop losgelaten is, wachten tot hij weer ingedrukt is
			}
			while(!leesKnop(noodstop)){ // wachten tot de knop opnieuw losgelaten is
			}
		}
	}
 
	if (!leesKnop(volgKnop)){ // Volgknop ingedrukt
		delay(20);
		if(!leesKnop(volgKnop)){ // Double check
		
			while(!leesKnop(volgKnop)){	// Zolang de knop ingedrukt is
			}
			
			volgModus = !volgModus;		// toggle de waarde van volgModus
			if (volgModus){
				dobeep(beepvolgaan);	// doe de dingen voor de bijhorende nieuwe waarde van volgmodus
				switchVar = volgen;
			}
			else{
				dobeep(beepvolguit);
				switchVar = tellen;
			}
			begonnen = false;
		}
	}
	
	if (RRPM > maxRPM){ // als de max is overschreden, zet terug naar max
		RRPM = maxRPM;
	}
	if (LRPM > maxRPM){
		LRPM = maxRPM;
	}
	if (RRPM < 0){ // Zelfde met min
		RRPM = 0;
	}
	if (LRPM < 0){
		LRPM = 0;
	}
	
	if (timerFlag){ //Als er minstens 1/4 ms voorbij is gegaan, update alle timers, en zet de variabele terug naar 0
		if (bijstuurTimer){
			bijstuurTimer += timerFlag/4;
			if (bijstuurTimer > bijstuurTimerMax){
				bijstuurTimer = 0;
			}
		}
		if (stapLTimer){
			stapLTimer += timerFlag/4;
		}
		if (stapRTimer){
			stapRTimer += timerFlag/4;
		}
		if (stopTimer){
			stopTimer += timerFlag/4;
		}
		timerFlag = 0;
		// !!!!!!!!!!!!!!!!!! Double check of alle timers hier in staan, anders werken ze obviously niet !!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	}
	
	
  // motoren aansturen
  if (RRPM == 0){	// Reset timer bij 0 rpm
	  stapRTimer = 1;
  }

  if (stapRTimer > (1000/((RRPM/60)*200))){ // verkeerd om, daarom low
    digitalWrite(richtingPinR, LOW);
    digitalWrite(stapPinR, HIGH);
    stapRTimer = 1;
    delayMicroseconds(00);
    digitalWrite(stapPinR, LOW);
    aantalStappenR++;
  }
  
  if (LRPM == 0){	// zelfde
	  stapLTimer = 1;
  }
  
  if (stapLTimer > (1000/((LRPM/60)*200))){
    digitalWrite(richtingPinL, HIGH);
    digitalWrite(stapPinL, HIGH);
    stapLTimer = 1;
    delayMicroseconds(200);
    digitalWrite(stapPinL, LOW);
    aantalStappenL++;
  }
  // motoren aansturen

  updateSensoren(); // Functie die de samenwerking van sensoren verzorgt en data refreshed waar mogelijk
  
  
  switch (switchVar){ // Switch case tussen kalibreren, tellen en volgen
    /*
    case kalibreren:
      uint16_t afwijking = (sqrt(pow(afstandZijkantVoor - afstandZijkantAchter, 2)));
      if (afwijking > maxAfwijkingZij && afwijking < bochtAfwijkingZij && bijstuurTimer == 0){ // Bijsturen als de sensorwaardes niet binnen de mogelijke afwijking zijn, maar het geen bocht is EN er niet bijgestuurd is in de laatste x ms (is dit wel nodig???) 
        bijsturen(afstandZijkantVoor, afstandZijkantAchter, afwijking);
      }
      
      if (medewerker){  // Evil pseudocode of death, please replace
         LRPM = 0;
         RRPM = 0; // stop
        if (LRPM && RRPM == 0) {
          timerFlag = 1; // start timer
          dobeep(beeppersoon); // do medewerker beep
          if (timer > x){
                // als timer > x, doe andere dingen die we nog niet gedefinieerd hebben, uit gecomment want compiler
          }
        }
       
    
      }
      if (bocht || bochtStatus != 0){ // Als bocht of bocht al begonnen is bochtlogica herkennning moet nog
        if (bochtStatus == 0){
          bochtStatus = 1;
          beginBochtStappen = aantalStappenR;
        }
        
        if (bochtStatus == 1){
          LRPM = bochtRPM;
          RRPM = bochtRPM;
          if (aantalStappenR > (beginBochtStappen + voorTotMidden*afstandPerStap)){
            bochtStatus = 2;
          }
        }
        
        if (bochtStatus == 2){
          beginBochtStappen = aantalStappenR;
          bochtStatus = 3;
        }
        
        if (bochtStatus == 3){
          bool temp = bochtDingen(aantalStappenR - beginBochtStappen, 90);
          if (temp){
            bochtStatus = 0;
          }
        }
      }
      
    break;
    */


    case tellen:
		if (!begonnen){	// Reset de RPM waardes bij het ingaan van deze case
		  RRPM = standaardRPM;
		  LRPM = standaardRPM;
		}
    
		switch(switchTellen){ // switch case tussen delen van de route, 
			case 0: // stuk naar rechts + bocht
			case 2:	// stuk naar links + bocht
			case 4:	// stuk naar rechts + bocht, einde rondje
		  
				if (afstandBoom < 200 && boomBezig == false){ // Als je een boom ziet die nog niet gezien is
					boomTeller++;
					dobeep(beepboom);
				}
				
				if (afstandBoom > 200){	// Detecteer einde van een boom
					boomBezig == false;
				}
	   
				if (afstandZijkantAchter < 250){ // bij het zien van de omkeping achter
					begonnen = true;
				}
			
				uint16_t afwijking = (sqrt(pow(afstandZijkantVoor - afstandZijkantAchter, 2))); // verschil tussen voor en achter absoluut
				
				if (afwijking > maxAfwijkingZij && afwijking < bochtAfwijkingZij && bijstuurTimer == 0 && bochtStatus == 0);{ // Bijsturen als de sensorwaardes niet binnen de mogelijke afwijking zijn, maar het geen bocht is EN er niet bijgestuurd is in de laatste x ms
					bijsturen(afstandZijkantVoor, afstandZijkantAchter, afwijking); // Bijstuurfunctie, gebruikt afstanden en verschil, past de snelheden aan
				}
				
				/*
				if (afstandZijkantVoor * afstandsFactorZijkant > zijkantAfstandOptimum + maxAfwijkingZij && bijstuurTimer == 0){ // Stuur bovendien 4 RPM bij afhankelijk van hoeveer de voorste sensor van de omkeping
					RRPM += 4;
					LRPM -= 4;
				}
				
				else if (afstandZijkantVoor * afstandsFactorZijkant < zijkantAfstandOptimum - maxAfwijkingZij && bijstuurTimer == 0){
					RRPM -= 4;
					LRPM += 4;
				}
				*/
				
				
				if ((afstandLinksVoor < medewerkerAfstand || afstandMiddenVoor < medewerkerAfstand || afstandRechtsVoor < medewerkerAfstand || stopTimer != 0) && bochtStatus == 0){  // Evil pseudocode of death, please replace
				// Als 1 van de 3 sensoren voor een medewerker detecteerd binnen de minimale afstand OF we al aan het wachten waren (nodig voor reset)
					LRPM = 0;
					RRPM = 0; // stop
					
					if (stopTimer == 0){ // start timer als dit de eerste keer is
						stopTimer = 1;
						dobeep(beeppersoon); // Beep naar de persoon
					}
					if (stopTimer > 5000){ // als we al 5 seconden wachten, beep urgenter
						dobeep(beepPersoon2);
					}
					// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
					if (stopTimer > 15000){ // Als we al 15 seconden wachten, assume bug, assume muur, werkt nu niet, en kan misschien beter iets compleet anders zijn
						// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
						stopTimer = 0;
						LRPM = standaardRPM;
						RRPM = standaardRPM;
						// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
					}
					// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
					
					if (afstandLinksVoor > medewerkerAfstand && afstandMiddenVoor > medewerkerAfstand && afstandRechtsVoor > medewerkerAfstand){ // Als de mederwerken niet meer gedetecteerd wordt, reset waardes
						stopTimer = 0;
						LRPM = standaardRPM;
						RRPM = standaardRPM;
					}
				}
			
				if ((afstandZijkantVoor > afstandZijkantAchter && afwijking > bochtAfwijkingZij) || bochtStatus != 0){ // Als bocht of bocht al begonnen is
					if (bochtStatus == 0){ 	// Als dit de eerste keer is, bocht gedetecteerd dus initaliseer
						bochtStatus = 1;
						beginBochtStappen = aantalStappenR;
						LRPM = bochtRPM;
						RRPM = bochtRPM;
					}
				
					if (bochtStatus == 1){	// Rij door tot de voorwaartse afstand overbrugt is, is stukje sensorafwijking omdat hij onder een hoek staat, en doorrijden tot midden van wielen
						if (aantalStappenR > (beginBochtStappen + (voorTotMidden + lengteAfwijkingSensor)*afstandPerStap)){
							bochtStatus = 2; //  volgende stap als de afstand overbrugt is
						}
					}
				
					if (bochtStatus == 2){ // Sla het huidige aantal stappen genomen door de rechter stappenmotor
						beginBochtStappen = aantalStappenR;
						bochtStatus = 3;
					}
				
					if (bochtStatus == 3){ // Zet bochtstappen tot aan de hoekverdraaiing, dat is einde van deze case
						bool temp = bochtDingen(aantalStappenR - beginBochtStappen, 90);
						aantalStappenR++;
						if (temp){ // einde bocht, reset waardes
							bochtStatus = 0;
							if (switchTellen == 4){ // Als het het einde van de complete ronde van de AGV is
								//Einde ronde, doe dingen ofzo, idk
								RRPM = 0;
								LRPM = 0;
								
								for (uint16_t i=0; i<boomTeller; i++){ // Aantal getelde bomen beeps
								// Beeps moeten waarschijnlijk vervangen worden ///////////////////////////////////////////////////////////////////////////////////////////
								//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
									digitalWrite(muziekpin1, HIGH);
									delay(half);
									digitalWrite(muziekpin1, LOW);
									delay(half);
								}
								//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
								
								delay(2000);
								
								dobeep(victoryBeep);
								
								while(true){
								} // Infinite loop voor einde
							}
							else{ // Zo niet, +1 voor volgende stap en ga verder naar volgend stuk
								switchTellen++;
								begonnen = false;
							}
						}
					}
				}
			break;
	  

			case 1: // Stuk omhoog, 2 openingen, dan bocht
			case 3: // stuk naar beneden, 1 opening, dan bocht
			
				// ALS MEDEWERKER -- fixed
				if ((afstandMiddenVoor < medewerkerAfstand || stopTimer != 0) && bochtStatus == 0){  // Evil pseudocode of death, please replace
				// Als 1 van de 3 sensoren voor een medewerker detecteerd binnen de minimale afstand OF we al aan het wachten waren (nodig voor reset)
					LRPM = 0;
					RRPM = 0; // stop
					
					if (stopTimer == 0){ // start timer als dit de eerste keer is
						stopTimer = 1;
						//dobeep(beeppersoon); // Beep naar de persoon
					}
					if (stopTimer > 5000){ // als we al 5 seconden wachten, beep urgenter
						//dobeep(beepPersoon2);
					}
					// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
					if (stopTimer > 15000){ // Als we al 15 seconden wachten, assume bug, assume muur, werkt nu niet, en kan misschien beter iets compleet anders zijn
						// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
						stopTimer = 0;
						LRPM = standaardRPM;
						RRPM = standaardRPM;
						// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
					}
					// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
					
					if (afstandLinksVoor > medewerkerAfstand && afstandMiddenVoor > medewerkerAfstand && afstandRechtsVoor > medewerkerAfstand){ // Als de mederwerken niet meer gedetecteerd wordt, reset waardes
						stopTimer = 0;
						LRPM = standaardRPM;
						RRPM = standaardRPM;
					}
				}
		
		
				int8_t openingen;
				if (switchTellen == 1){ // Detectie of het case 1 of 3 is, want ze hebben een andere hoeveelheid lege stukken die ze moeten overbruggen
					openingen = 2;
				}
				if (switchTellen == 3){
					openingen = 1;
				}
				LRPM = bochtRPM;
				RRPM = bochtRPM;
				
				uint16_t tempAfwijking = (sqrt(pow(afstandZijkantVoor - afstandZijkantAchter, 2)));
				
				if (tempAfwijking < bochtAfwijkingZij){ // Als we niet langs een opening aan het rijden zijn
					openingBezig = false;
				}
		
				if ((afstandZijkantVoor > afstandZijkantAchter && tempAfwijking > bochtAfwijkingZij && openingBezig == false) || bochtStatus != 0){ // De eerste keer afstand voor groter is dan achter, en het een bochtafstand is OF we al in een bocht zijn
					openingTeller++;
					openingBezig = true;
					if (openingTeller > openingen){ // Als het de bochtopening is doe bochtdingen, zelfde als boven
						if (bochtStatus == 0){ 	// Als dit de eerste keer is, bocht gedetecteerd dus initaliseer
						bochtStatus = 1;
						beginBochtStappen = aantalStappenR;
						LRPM = bochtRPM;
						RRPM = bochtRPM;
						}
				
						if (bochtStatus == 1){	// Rij door tot de voorwaartse afstand overbrugt is, is stukje sensorafwijking omdat hij onder een hoek staat, en doorrijden tot midden van wielen
							if (aantalStappenR > (beginBochtStappen + (voorTotMidden + lengteAfwijkingSensor)*afstandPerStap)){
								bochtStatus = 2; //  volgende stap als de afstand overbrugt is
							}
						}
					
						if (bochtStatus == 2){ // Sla het huidige aantal stappen genomen door de rechter stappenmotor
							beginBochtStappen = aantalStappenR;
							bochtStatus = 3;
						}
					
						if (bochtStatus == 3){ // Zet bochtstappen tot aan de hoekverdraaiing, dat is einde van deze case
							bool temp = bochtDingen(aantalStappenR - beginBochtStappen, 90);
							aantalStappenR++;
							if (temp){ // einde bocht, reset waardes en ga verder naar volgend stuk
								bochtStatus = 0;
								openingTeller = 0;
								switchTellen++;
								begonnen = false;
							}
						}
					}
				}
			break;
		}
    break; // Einde case tellen
    
    case volgen:
		if (!begonnen){ // als dit de eerste keer in de case is, initialiseer
			if(afstandMiddenVoor < volgAfstand + maxAfwijkingZij*6){ // wacht tot medewerken op afstand is 
			// HIER DINGEN VERANDERD< W@AS EEN WHILE N+LOOP< NU EEN IF!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
				begonnen = true;
				RRPM = standaardRPM;
				LRPM = standaardRPM;
			}
		}
		else if (bijstuurTimer == 0){ // Zo niet, als het tijd is om bij te sturen, doe dat
			bijstuurTimer = 1; // Reset timer
			volgenBijsturen(afstandLinksVoor, afstandMiddenVoor, afstandRechtsVoor); // Stuur bij afhankelijk van afstand en deel waar de mederwerker zich bevindt
		}
    break; // Einde case volgen
  }
}

void dobeep(int8_t beepkeuze){
	switch (beepkeuze){
		case beeppersoon:
	   // beep beep
		  digitalWrite(muziekpin1, HIGH);
		  delay(kwart);
		  digitalWrite(muziekpin1, LOW);
		  delay(kwart);
		  digitalWrite(muziekpin1, HIGH);
		  delay(kwart);
		  digitalWrite(muziekpin1, LOW);
		break;
		
		case beepvolgaan:
			// beep beeeeeeeeeeeepppp
		  digitalWrite(muziekpin1, HIGH);
		  delay(kwart);
		  digitalWrite(muziekpin1, LOW);
		  delay(kwart);
		  digitalWrite(muziekpin1, HIGH);
		  delay(heel);
		  digitalWrite(muziekpin1, LOW);
		break;
		
		case beepvolguit:
			// beeeeeeeeeeeeeeeeppp beep
		  digitalWrite(muziekpin1, HIGH);
		  delay(heel);
		  digitalWrite(muziekpin1, LOW);
		  delay(kwart);
		  digitalWrite(muziekpin1, HIGH);
		  delay(kwart);
		  digitalWrite(muziekpin1, LOW);
		break;
		
		case beepPersoon2:
			// beep beep beep beep beep beep beep beep
		  for(uint8_t i=0; i< 8; i++){
			  digitalWrite(muziekpin1, HIGH);
			  delay(kwart);
			  digitalWrite(muziekpin1, LOW);
			  delay(kwart);
			}
		break;
		
		case victoryBeep:
			// Final fantasy tune
			digitalWrite(muziekpin1, HIGH);
			delay(zestiende);
			digitalWrite(muziekpin1, LOW);
			delay(zestiende);
			digitalWrite(muziekpin1, HIGH);
			delay(zestiende);
			digitalWrite(muziekpin1, LOW);
			delay(zestiende);
			digitalWrite(muziekpin1, HIGH);
			delay(zestiende);
			digitalWrite(muziekpin1, LOW);
			delay(zestiende);
			digitalWrite(muziekpin1, HIGH);
			delay(half);
			digitalWrite(muziekpin1, LOW);
			digitalWrite(muziekpin1, HIGH);
			delay(half);
			digitalWrite(muziekpin1, LOW);
			digitalWrite(muziekpin1, HIGH);
			delay(half);
			digitalWrite(muziekpin1, LOW);
			digitalWrite(muziekpin1, HIGH);
			delay(achtste);
			digitalWrite(muziekpin1, LOW);
			delay(kwart);
			digitalWrite(muziekpin1, HIGH);
			delay(achtste);
			digitalWrite(muziekpin1, LOW);
			digitalWrite(muziekpin1, HIGH);
			delay(heel);
			digitalWrite(muziekpin1, LOW);
		break;
	}
}

/* Uit gecomment omdat ze niet werken met timer interrupts wegens vermoedelijk de tone functie
void dobeep(int8_t beepkeuze){ // do beep WERKT NIET MET TIMER INTERRUPT ATM!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	switch (beepkeuze){
		case beeppersoon:
	   // beep beep
		  tone(muziekpin1, NOTE_B5, kwart);
		  delay(kwart);
		  noTone(muziekpin1);
		  delay(kwart);
		  tone(muziekpin1, NOTE_B5, kwart);
		  delay(kwart);
		  noTone(muziekpin1);
		break;
		
		case beepvolgaan:
			// beep beeeeeeeeeeeepppp
		  tone(muziekpin1, NOTE_B5, kwart);
		  delay(kwart);
		  noTone(muziekpin1);
		  delay(kwart);
		  tone(muziekpin1, NOTE_B5, heel);
		  delay(heel);
		  noTone(muziekpin1);
		break;
		
		case beepvolguit:
			// beeeeeeeeeeeeeeeeppp beep
		  tone(muziekpin1, NOTE_B5, heel);
		  delay(heel);
		  noTone(muziekpin1);
		  delay(kwart);
		  tone(muziekpin1, NOTE_B5, kwart);
		  delay(kwart);
		  noTone(muziekpin1);
		break;
		
		case beepPersoon2:
			// beep beep beep beep beep beep beep beep beep beep beep beep beep beep beep beep
		  for(uint8_t i=0; i< 8; i++){
			  tone(muziekpin1, NOTE_B5, kwart);
			  delay(kwart);
			  noTone(muziekpin1);
			  delay(kwart);
			}
		break;
		
		case victoryBeep:
			// Final fantasy tune
			tone(muziekpin1, NOTE_B5, zestiende);
			delay(zestiende);
			noTone(muziekpin1);
			delay(zestiende);
			tone(muziekpin1, NOTE_B5, zestiende);
			delay(zestiende);
			noTone(muziekpin1);
			delay(zestiende);
			tone(muziekpin1, NOTE_B5, zestiende);
			delay(zestiende);
			noTone(muziekpin1);
			delay(zestiende);
			tone(muziekpin1, NOTE_B5, half);
			delay(half);
			noTone(muziekpin1);
			tone(muziekpin1, NOTE_G5, half);
			delay(half);
			noTone(muziekpin1);
			tone(muziekpin1, NOTE_A5, half);
			delay(half);
			noTone(muziekpin1);
			tone(muziekpin1, NOTE_B5, achtste);
			delay(achtste);
			noTone(muziekpin1);
			delay(kwart);
			tone(muziekpin1, NOTE_A5, achtste);
			delay(achtste);
			noTone(muziekpin1);
			tone(muziekpin1, NOTE_B5, heel);
			delay(heel);
			noTone(muziekpin1);
		break;
	}
}
*/

void bijsturen(uint16_t voor, uint16_t achter, uint16_t afwijking){
  //uint16_t RPMVerandering = (afwijking-maxAfwijkingZij)*snelheidAfwijkingConstante;
  uint16_t RPMVerandering = snelheidAfwijkingConstante;
  if (voor > achter){
    RRPM += RPMVerandering;
    LRPM -= RPMVerandering;
  }
  else{
    RRPM -= RPMVerandering;
    LRPM += RPMVerandering;
  }
  bijstuurTimer = 1;
}

void volgenBijsturen(float links, float rechts, float midden){ //Nog een keer checken, volgsyntax herschreven aan de hand van de 6 vershillende mogelijkheden

	uint16_t LM = sqrt(pow(links-midden, 2));		// Verschil Links en Midden
	uint16_t MR = sqrt(pow(midden-rechts, 2));	// Verschil Midden en Rechts
	uint16_t LR = sqrt(pow(links-rechts, 2));		// Verschil Links en Rechts
  
	if(MR + volgAfwijking < LM && MR + volgAfwijking < LR && links < rechts){
		LRPM -= 2*snelheidAfwijkingConstante;
		RRPM += 2*snelheidAfwijkingConstante;
		if (links + maxAfwijkingZij < volgAfstand){
			LRPM -= snelheidAfwijkingConstante;
			RRPM -= snelheidAfwijkingConstante;
		}
		if (links > volgAfstand + maxAfwijkingZij){
			LRPM += snelheidAfwijkingConstante;
			RRPM += snelheidAfwijkingConstante;
		}
	}
	
	else if(LM + volgAfwijking < LR && LM + volgAfwijking < MR && links < rechts){
		LRPM -= snelheidAfwijkingConstante;
		RRPM += snelheidAfwijkingConstante;
		if ((links + midden)/2 + maxAfwijkingZij < volgAfstand){
			LRPM -= snelheidAfwijkingConstante;
			RRPM -= snelheidAfwijkingConstante;
		}
		if ((links + midden)/2 > volgAfstand + maxAfwijkingZij){
			LRPM += snelheidAfwijkingConstante;
			RRPM += snelheidAfwijkingConstante;
		}
	}
	
	else if(MR + volgAfwijking < LM && MR + volgAfwijking < LR && rechts < links){
		LRPM += snelheidAfwijkingConstante;
		RRPM -= snelheidAfwijkingConstante;
		if ((rechts + midden)/2 + maxAfwijkingZij < volgAfstand){
			LRPM -= snelheidAfwijkingConstante;
			RRPM -= snelheidAfwijkingConstante;
		}
		if ((rechts + midden)/2 > volgAfstand + maxAfwijkingZij){
			LRPM += snelheidAfwijkingConstante;
			RRPM += snelheidAfwijkingConstante;
		}
	}
	
	else if(LM + volgAfwijking < LR && LM + volgAfwijking < MR && rechts < links){
		LRPM += 2*snelheidAfwijkingConstante;
		RRPM -= 2*snelheidAfwijkingConstante;
		if (links + maxAfwijkingZij < volgAfstand){
			LRPM -= snelheidAfwijkingConstante;
			RRPM -= snelheidAfwijkingConstante;
		}
		if (links > volgAfstand + maxAfwijkingZij){
			LRPM += snelheidAfwijkingConstante;
			RRPM += snelheidAfwijkingConstante;
		}
	}
	
	else if(LR + volgAfwijking < LM && LR + volgAfwijking < MR){
		if (midden + maxAfwijkingZij < volgAfstand){
			LRPM -= snelheidAfwijkingConstante;
			RRPM -= snelheidAfwijkingConstante;
		}
		if (midden > volgAfstand + maxAfwijkingZij){
			LRPM += snelheidAfwijkingConstante;
			RRPM += snelheidAfwijkingConstante;
		}
	}
	
	else{
		if (midden + maxAfwijkingZij < volgAfstand){
			LRPM -= snelheidAfwijkingConstante;
			RRPM -= snelheidAfwijkingConstante;
		}
		if (midden > volgAfstand + maxAfwijkingZij){
			LRPM += snelheidAfwijkingConstante;
			RRPM += snelheidAfwijkingConstante;
		}
	}
	
	bijstuurTimer = 1;
}

bool bochtDingen(uint16_t aantalStappenGedaan = 0, int16_t hoek = 90){
  
  if (aantalStappenGedaan >= sqrt(pow(hoek*aantalStappenPerGrade, 2))){
    return true;
  }
  
  if (hoek < 100 && hoek > 0){
    digitalWrite(richtingPinL, HIGH);
    digitalWrite(stapPinL, HIGH);
    delayMicroseconds(500);
    digitalWrite(stapPinL, LOW);
    delayMicroseconds(4500);
  }
  else if (hoek > -100 && hoek < 0){
    digitalWrite(richtingPinR, LOW); // verkeerd om, daarom low
    digitalWrite(stapPinR, HIGH);
    delayMicroseconds(500);
    digitalWrite(stapPinR, LOW);
    delayMicroseconds(4500);
  }
  else if (hoek == 180){
    digitalWrite(richtingPinR, LOW); // verkeerd om, daarom low
    digitalWrite(richtingPinL, LOW);
    digitalWrite(stapPinR, HIGH);
    digitalWrite(stapPinL, HIGH);
    delayMicroseconds(500);
    digitalWrite(stapPinR, LOW);
    digitalWrite(stapPinL, LOW);
    delayMicroseconds(4500);
  }
  
  return false;
}

void updateSensoren(){
  switch(sensorTellerVoor){
    case 0:
      if(sensorLinksVoor.checkSensorData(sensorVoorTemp) == 0){
        afstandLinksVoor = sensorLinksVoor.leesSensor();
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
    break;

    case 2:
      if(sensorRechtsVoor.checkSensorData(sensorVoorTemp) == 0){
        afstandRechtsVoor = sensorRechtsVoor.leesSensor();
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
    break;
    
    case 1:
      if(sensorBoom.checkSensorData(sensorZijTemp) == 0){
        afstandBoom = sensorBoom.leesSensor();
        sensorTellerZij = 2;
        sensorZijTemp = sensorZijkantAchter.startSensor();
      }
    break;

    case 2:
      if(sensorZijkantAchter.checkSensorData(sensorZijTemp) == 0){
        afstandZijkantAchter = sensorZijkantAchter.leesSensor();
        sensorTellerZij = 0;
        sensorZijTemp = sensorZijkantVoor.startSensor();
      }
    break;
  }
}

bool leesKnop(uint8_t knopPin) {
  for(int8_t i=0; i<8; i++){
    if (digitalRead(knopPin) != 0){
      return(false);
    }
  }
  return(true);
}

ISR(TIMER0_COMPA_vect){
  timerFlag++;
}