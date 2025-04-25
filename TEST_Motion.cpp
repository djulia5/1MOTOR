#include <Arduino.h>
#include "driver/twai.h"
#include "xiaomi_cybergear_driver.h"
#include <math.h>

// Définitions des broches
#define RX_PIN D7
#define TX_PIN D6
#define BUTTON_PIN D5  // Broche du bouton poussoir

// IDs CAN
static const uint8_t MOTOR_ID_0 = 0x02;  // Moteur 0 0x7F
static const uint8_t MASTER_ID = 0xFD;


const float SPEED_MOTION = 1.0f;  // Vitesse cible pour le mode motion (augmentée pour test)
const float KP_VALUE = 1.0f;   // Gain proportionnel
const float KD_VALUE = 1.0f;    // Gain dérivé
const float TRQ_MOTION = 1.0f;  // Couple cible pour le mode motion (augmenté pour test)
//const float KI_SPEED = 0.008f;  // Gain intégral pour la boucle de vitesse interne


// Instances des drivers CyberGear
XiaomiCyberGearDriver motor0(MOTOR_ID_0, MASTER_ID);


// Variables pour la gestion du bouton
int etatBouton = 0;         // État actuel du bouton
int dernierEtatBouton = HIGH;  // Dernier état du bouton
unsigned long lastButtonPress = 0;  // Pour le débouncing

// Vérifier et réinitialiser le bus CAN si nécessaire
void check_and_reset_can_bus() {
    twai_status_info_t status;
    twai_get_status_info(&status);
    if (status.state != TWAI_STATE_RUNNING) {
        Serial.println("Bus CAN pas dans l’état RUNNING, réinitialisation...");
        twai_stop();
        delay(100);
        twai_start();
        delay(100);
        motor0.init_twai(RX_PIN, TX_PIN, true);
    }
}

// Définir la position actuelle comme zéro
void set_mechanical_position_to_zero(uint8_t motor_id, uint8_t master_id) {
    check_and_reset_can_bus();

    twai_message_t message;
    message.extd = 1;
    message.identifier = (6UL << 24) | ((uint32_t)master_id << 8) | motor_id;
    message.data_length_code = 8;
    message.data[0] = 1;
    for (uint8_t i = 1; i < 8; i++) {
        message.data[i] = 0;
    }

    esp_err_t ret = twai_transmit(&message, pdMS_TO_TICKS(1000));
    if (ret != ESP_OK) {
        Serial.print("Erreur lors de l’envoi de CMD_SET_MECH_POSITION_TO_ZERO : ");
        Serial.println(esp_err_to_name(ret));
    }
}

void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10);

    // Configuration du bouton sur D5
    pinMode(BUTTON_PIN, INPUT_PULLUP);  // Logique inversée : HIGH = non pressé, LOW = pressé

    // **Séquence d’initialisation**
    // 1. Initialisation du bus CAN
    Serial.println("Initialisation du bus CAN...");
    if (motor0.init_twai(RX_PIN, TX_PIN, true) != ESP_OK) {
        Serial.println("Erreur d’initialisation du bus CAN !");
        while (1) delay(1000);
    }
 

    // 2. Initialisation des moteurs
    Serial.println("Initialisation des moteurs...");
    
    motor0.init_motor(MODE_MOTION);
    motor0.enable_motor();
    delay(1000);
   
    // 3. Définir la position actuelle comme zéro
   Serial.println("Définition de la position zéro...");
   set_mechanical_position_to_zero(MOTOR_ID_0, MASTER_ID);
   delay(1000);

  // 5. Mouvement initial pour tester
 Serial.println("Mouvement initial vers 0 radians...");
 motor0.set_motion_control(0.0f, SPEED_MOTION, TRQ_MOTION, KP_VALUE, KD_VALUE);
}

void loop() {
    // Lire l’état du bouton
    etatBouton = digitalRead(BUTTON_PIN);

    // Détecter un appui (logique inversée : HIGH → LOW)
    if (etatBouton == LOW && dernierEtatBouton == HIGH) {
        unsigned long currentTime = millis();
        if (currentTime - lastButtonPress >= 200) {  // Débouncing
            lastButtonPress = currentTime;
 
            motor0.set_motion_control(-0.5f, SPEED_MOTION, TRQ_MOTION, KP_VALUE, KD_VALUE);
            delay(1000);
            motor0.set_motion_control(0.0f, SPEED_MOTION, TRQ_MOTION, KP_VALUE, KD_VALUE);
            delay(1000);
            motor0.set_motion_control(0.5f, SPEED_MOTION, TRQ_MOTION, KP_VALUE, KD_VALUE);
            delay(1000);
            motor0.set_motion_control(0.0f, SPEED_MOTION, TRQ_MOTION, KP_VALUE, KD_VALUE);
            delay(1000);


        }


    }
    dernierEtatBouton = etatBouton;

    delay(10);  // Réduire la charge CPU
}
