/*
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
	
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
	
    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */
/*
 * Basé sur :
 * https://github.com/khancyr/droneID_FR
 * https://github.com/khancyr/TTGO_T_BEAM
 * https://github.com/dev-fred/GPS_Tracker_ESP8266
 * https://github.com/aster94/GoProControl
 * 
 * Adapté pour fonctionner avec un ESP8266 Wemos D1 mini
 */

 
// ======= Paramètres balise esp32 ** à modifier ** ============= //
/**
  * Le nom du point d'acces wifi CHANGEZ LE par ce que vous voulez !!!
  */
const char ssid[] = "ILLEGAL_DRONE_AP";
/**
  * CHANGEZ l'ID du drone par celui que Alphatango vous a fourni (Trigramme + Modèle + numéro série) !
  */
//                        "000000000000000000000000000000"  // 30 caractères
const char drone_id[31] = "ILLEGAL_DRONE_APPELEZ_POLICE17"; // si l'id est inférieur à 30 caractères, le compléter avec des "0" au début

// =========== Includes ======================= //
#include <ESP8266WiFi.h>
#include <SoftwareSerial.h>   
#include "droneID_FR.h"

#include <TinyGPS++.h> 

#include <GoProControl.h>
#include "Secrets.h"

#define GPS_BAUD_RATE 9600  // Valeur par défaut
#define GPS_RX_PIN 5       // Brancher le fil Tx du GPS
#define GPS_TX_PIN 4       // Brancher le fil Rx du GPS

// On déclare les pins des Leds
#define GPS_LED_PIN 0 //D3
#define GOPRO_LED_PIN 16 //D0

// On déclare les pins des boutons
#define Gopro_OnOff_Button 13 //D7
#define Gopro_Mode_Button 14 //D5
#define Gopro_Trigger_PushButton 12 //D6

// On déclare la Led interne a l'esp -> conservé pour utilisation future
#define INTERNAL_LED_PIN 2  //internal blue LED D4



extern "C" {
#include "user_interface.h"
  int wifi_send_pkt_freedom(uint8 *buf, int len, bool sys_seq);
}

// ========================================================== //

SoftwareSerial softSerial(GPS_RX_PIN, GPS_TX_PIN);
TinyGPSPlus gps;

droneIDFR drone_idfr;

GoProControl gp(GOPRO_SSID, GOPRO_PASS, HERO3); //renseigner les SSID et PASS dans le fichier Secrets.h et renseigner le type de gopro. se référer à la librairie d'aster94 cité plus haut. HERO3 dans mon cas.

// beacon frame definition
static constexpr uint16_t MAX_BEACON_SIZE = 40 + 32 + droneIDFR::FRAME_PAYLOAD_LEN_MAX; // default beaconPacket size + max ssid size + max drone id frame size

// beacon frame definition
uint8_t beaconPacket[MAX_BEACON_SIZE] = {
  /*  0 - 3  */ 0x80, 0x00, 0x00, 0x00, // Type/Subtype: managment beacon frame
  /*  4 - 9  */ 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, // Destination: broadcast
  /* 10 - 15 */ 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, // Source
  /* 16 - 21 */ 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, // Source

  // Fixed parameters
  /* 22 - 23 */ 0x00, 0x00, // Fragment & sequence number (will be done by the SDK)
  /* 24 - 31 */ 0x83, 0x51, 0xf7, 0x8f, 0x0f, 0x00, 0x00, 0x00, // Timestamp
  /* 32 - 33 */ 0xe8, 0x03, // Interval: 0x64, 0x00 => every 100ms - 0xe8, 0x03 => every 1s
  /* 34 - 35 */ 0x21, 0x04, // capabilities Tnformation

  // Tagged parameters

  // SSID parameters
  /* 36 - 38 */ 0x03, 0x01, 0x06, // DS Parameter set, current channel 6 (= 0x06), // TODO: manually set it
  /* 39 - 40 */ 0x00, 0x20,       // 39-40: SSID parameter set, 0x20:maxlength:content
};


// Vérification ssid max 30 
static_assert((sizeof(ssid)/sizeof(*ssid))<=32, "AP SSID should be less than 32 letters");
// Vérification drone_id max 30 
static_assert((sizeof(drone_id)/sizeof(*drone_id))<=31, "Drone ID should be less that 30 letters !");  // 30 lettres + null termination

uint8_t program = 0; // 1: test beacon without GPS
uint8_t camtype = 0; // 1: camera xiaomi?
char buff[5][256];
uint64_t gpsSec = 0;
uint64_t beaconSec = 0;
int nb_sat = 0;

// Définintion des variables de contrôle de la gopro
int PosButtonOn = 0; //0: Gopro off ; 1: Gopro on
int PosButtonMode = 0; //0: mode photo ; 1: mode video
int PrevPosButtonMode = 0;
int GoProLatency = 2000; //Latence de la gopro en milli secondes-> temps de repos entre deux commandes à la gopro
bool GoProOn = false; 
bool TriggerState = false;
uint64_t TimerGoPro = 0; 

// Filtre anti-rebond (debouncer)
#define DEBOUNCE_TIME 250
volatile uint64_t DebounceTimer = 0;

// Fonction interruption trigger gopro avec filtre antirebond
ICACHE_RAM_ATTR void trigger() {
  if ( millis() - DEBOUNCE_TIME  >= DebounceTimer ) {
    DebounceTimer = millis();
    Serial.println("camera triggered");
    TriggerState = true ; //True: le bouton de début/fin video ou prise de photo a été déclenché
  }
}

// utilisation
void setup() {
  // start serial
  Serial.begin(115200);
  Serial.println();

  //Leds et boutons
  pinMode(INTERNAL_LED_PIN, OUTPUT);
  digitalWrite(INTERNAL_LED_PIN, HIGH);
  pinMode(GPS_LED_PIN, OUTPUT);
  digitalWrite(GPS_LED_PIN, LOW);
  pinMode(GOPRO_LED_PIN, OUTPUT);
  digitalWrite(GOPRO_LED_PIN, LOW);
  pinMode(Gopro_Mode_Button, INPUT_PULLUP);
  pinMode(Gopro_OnOff_Button, INPUT_PULLUP);
  pinMode(Gopro_Trigger_PushButton, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(Gopro_Trigger_PushButton), trigger, RISING);
  
  // start WiFi
  WiFi.mode(WIFI_OFF);
  delay(100);
  WiFi.mode(WIFI_AP_STA); //Permets de mettre l'ESP en mode Acces Point & Station
  
  // set default AP settings
  WiFi.softAP(ssid, nullptr, 6, false, 0); // ssid, pwd, channel, hidden, max_cnx, 
  WiFi.setOutputPower(20.5); // max 20.5dBm
  
  softap_config current_config;
  wifi_softap_get_config(&current_config);

  current_config.beacon_interval = 1000;
  wifi_softap_set_config(&current_config);
  
  Serial.println();
  Serial.println("Started \\o/");
  Serial.println();

  softSerial.begin(GPS_BAUD_RATE);
 
  drone_idfr.set_drone_id(drone_id); 

  delay(5000);
}

/**
 * Début du code principal. C'est une boucle infinie.
 */
void loop()
{
    static uint64_t gpsMap = 0;

        
    switch (program) {
    case 0:
        // Ici on lit les données qui arrivent du GPS et on les passe à la librairie TinyGPS++ pour les traiter
        while (softSerial.available())
            gps.encode(softSerial.read());
        // On traite le cas où le GPS a un problème
        if (millis() > 5000 && gps.charsProcessed() < 10) {
            Serial.println("No GPS detected");
            digitalWrite(GPS_LED_PIN, LOW);
            return;
        }
        // On traite le cas si la position GPS n'est pas valide
        if (!gps.location.isValid()) {
            if (millis() - gpsMap > 1000) {
                Serial.print("Waiting... SAT=");  Serial.println(gps.satellites.value());
                digitalWrite(GPS_LED_PIN, HIGH);
                delay(100);
                digitalWrite(GPS_LED_PIN, LOW);
                gpsMap = millis();
            }
            return;
        }
		
        else if (gps.location.isUpdated() && gps.altitude.isUpdated() && gps.course.isUpdated() && gps.speed.isUpdated()){
            // On traite le cas où la position GPS est valide.
            // On renseigne le point de démarrage quand la précision est satisfaisante
            if ( gps.satellites.value() > nb_sat ) {//+ de satellites
                nb_sat = gps.satellites.value();
                Serial.print("Waiting... SAT=");  Serial.println(gps.satellites.value());
                digitalWrite(GPS_LED_PIN, HIGH);
                delay(100);
                digitalWrite(GPS_LED_PIN, LOW);
            }
            if (!drone_idfr.has_home_set() && gps.satellites.value() > 5 && gps.hdop.hdop() < 2.0) {
                Serial.println("Setting Home Position");
                digitalWrite(GPS_LED_PIN, HIGH);
                delay(100);
                digitalWrite(GPS_LED_PIN, LOW);
                drone_idfr.set_home_position(gps.location.lat(), gps.location.lng(), gps.altitude.meters());
            }

            // On actualise les données GPS de la librairie d'identification drone.
            drone_idfr.set_current_position(gps.location.lat(), gps.location.lng(), gps.altitude.meters());
            drone_idfr.set_heading(gps.course.deg());
            drone_idfr.set_ground_speed(gps.speed.mps());
            digitalWrite(GPS_LED_PIN, HIGH);
            /*
            float gps_time_elapsed = (float(millis() - gpsMap) / 1000.0f); 
            gpsMap = millis();
        
            Serial.println(gps_time_elapsed,3);
            */
            
            // Ici on affiche les données GPS pour sur le port Serie, toutes les secondes.
            // Décommenter ce block pour voir les informations détaillées sur le port série
            /*if (millis() - gpsMap > 1000) {         
                Serial.print("LAT=");  Serial.print(gps.location.lat(), 6); Serial.print(" LONG="); Serial.print(gps.location.lng(), 6);
                Serial.print(" ALT=");  Serial.print(gps.altitude.meters());  Serial.print(" SAT=");  Serial.print(gps.satellites.value());
                Serial.print(" DOP=");  Serial.print(gps.hdop.hdop());  Serial.print(" DISTfromLAST_POS_Sent=");  Serial.println(drone_idfr.get_distance_from_last_position_sent());
                gpsMap = millis();
            }*/
        }
        break;
    case 1:
        // Envoi données GPS de test
        drone_idfr.set_current_position(43.1234, -0.1234, 223);
        drone_idfr.set_heading(234);
        drone_idfr.set_ground_speed(45);

        drone_idfr.set_home_position(43.1234, -0.1234, 100);
        
    }


    switch (camtype) {
    case 0:
		// On lit les valeurs des boutons
		PosButtonOn = digitalRead(Gopro_OnOff_Button);
		PosButtonMode = digitalRead(Gopro_Mode_Button);
		
		//GESTION DE L'ALLUMAGE ET DE L'ARRET DE LA GOPRO
		
		// On connecte et démarre la gopro si le bouton est sur ON, et que la gopro n'est ni connectée, ni allumée
    if (PosButtonOn == 1 && !GoProOn && !gp.isConnected()) {
			Serial.println("tentative de connection à la gopro");
      digitalWrite(GPS_LED_PIN, LOW);
      digitalWrite(GOPRO_LED_PIN, HIGH);
      delay(100);
      digitalWrite(GOPRO_LED_PIN, LOW);
      delay(100);
      digitalWrite(GOPRO_LED_PIN, HIGH);
      delay(100);
      digitalWrite(GOPRO_LED_PIN, LOW);
      delay(100);
      digitalWrite(GOPRO_LED_PIN, HIGH);
      delay(100);
      digitalWrite(GOPRO_LED_PIN, LOW);
      
			gp.begin(); //Connecte la gopro. !!!LE WIFI DE LA GOPRO DOIT ETRE ALLUME, SINON ON NE FAIT QU'ESSAYER DE SE CONNECTER A LA GOPRO SANS PASSER EN MODE BALISE!!!
			Serial.print("Connected to gopro: ");
			Serial.println(gp.isConnected() == true ? "Yes" : "No");
			
			//démarre la gopro
			if (gp.isConnected()){
				gp.turnOn();
				GoProOn = true;
				delay(5000);
				
				if(PosButtonMode == 0 && GoProOn){
					gp.setMode(PHOTO_MODE);
          PrevPosButtonMode = 0;
          digitalWrite(GOPRO_LED_PIN, HIGH);
					delay(1000);
				}
				if(PosButtonMode == 1 && GoProOn){
					gp.setMode(VIDEO_MODE);
          PrevPosButtonMode = 1;
          digitalWrite(GOPRO_LED_PIN, HIGH);
					delay(1000);
				}
			}
		}
		
		// On traite le cas si on veut éteindre la gopro ou désactiver la commande de la gopro
		if (PosButtonOn == 0 && GoProOn) {
			//on Traite le cas si la gopro est en train d'enregistrer
			if (gp.isRecording()){
				Serial.println("Gopro en train d'enregistrer. Arrêt de gopro enclenché, arrêt enregistrement");
				gp.stopShoot();
				delay(1000);
			}
			//on éteint la gopro
			gp.turnOff();
			GoProOn = false;
			//on déconnecte la gopro
			gp.end();
      digitalWrite(GOPRO_LED_PIN, LOW);
		}
		
		//GESTION DU CHANGEMENT DE MODE PHOTO/VIDEO
		    
    if (GoProOn && PosButtonMode != PrevPosButtonMode && (millis()-TimerGoPro) >= GoProLatency) {
			if (PrevPosButtonMode == 0) {
				gp.setMode(VIDEO_MODE);
				PrevPosButtonMode = 1;
				TimerGoPro = millis();
			}
			else if (PrevPosButtonMode == 1) {
				if (gp.isRecording()){
					gp.stopShoot();
					TimerGoPro = millis();
				}
				else if (!gp.isRecording()){
				  gp.setMode(PHOTO_MODE);
				  PrevPosButtonMode = 0;
				  TimerGoPro = millis();
				}
			}
		}
		
		//GESTION DU DEMARRAGE/ARRET EREGISTREMENT
    if (GoProOn && TriggerState && (millis()-TimerGoPro) >= GoProLatency) {
			if (PosButtonMode == 0 && PrevPosButtonMode == 0) {
				TriggerState = false;
				gp.shoot();
				TimerGoPro = millis();
        Serial.println("prise photo");
			}
			else if (PosButtonMode == 1 && PrevPosButtonMode == 1) {
				if (gp.isRecording()){
					TriggerState = false;
					gp.stopShoot();
					TimerGoPro = millis();
          Serial.println("fin video");
				}
				
				else if (!gp.isRecording()){
					TriggerState = false;
					gp.shoot();
					TimerGoPro = millis();
          Serial.println("debut video");
				}
			}
		}
    
		break;
    case 1:
      //code a créer pour une autre caméra ?
    break;
    }
	

    /**
     * On regarde s'il est temps d'envoyer la trame d'identification drone : 
     *  - soit toutes les 3s,
     *  - soit si le drone s'est déplacé de 30m,
     *  - uniquement si la position Home est déjà définie,
     *  - et dans le cas où les données GPS sont nouvelles.
     */            
     if (drone_idfr.has_home_set() && drone_idfr.time_to_send()) {
        float time_elapsed = (float(millis() - beaconSec) / 1000); 
        beaconSec = millis();
        
        Serial.print(time_elapsed,1); Serial.print("s Send beacon: "); Serial.print(drone_idfr.has_pass_distance() ? "Distance" : "Time");
        Serial.print(" with ");  Serial.print(drone_idfr.get_distance_from_last_position_sent()); Serial.print("m Speed="); Serial.println(drone_idfr.get_ground_speed_kmh()); 
        
        /**
         * On commence par renseigner le ssid du wifi dans la trame
         */
        // write new SSID into beacon frame
        const size_t ssid_size = (sizeof(ssid)/sizeof(*ssid)) - 1; // remove trailling null termination
        beaconPacket[40] = ssid_size;  // set size
        memcpy(&beaconPacket[41], ssid, ssid_size); // set ssid
        const uint8_t header_size = 41 + ssid_size;  //TODO: remove 41 for a marker
        /**
         * On génère la trame wifi avec l'identification
         */
        const uint8_t to_send = drone_idfr.generate_beacon_frame(beaconPacket, header_size);  // override the null termination
        // Décommenter ce block pour voir la trame entière sur le port série
        /* Serial.println("beaconPacket : ");
        for (auto i=0; i<sizeof(beaconPacket);i++) {
            Serial.print(beaconPacket[i], HEX);
            Serial.print(" ");
        }
        Serial.println(" ");
        */
        
        /**
         * On envoie la trame
         */
        wifi_send_pkt_freedom(beaconPacket, to_send, 0);
		 
        /**
         * On reset la condition d'envoi
         */
        drone_idfr.set_last_send();
    }

}
