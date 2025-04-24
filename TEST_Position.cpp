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

// Paramètres ajustables
//const float MAX_SPEED = 6.0f;  // Vitesse max en rad/s
//const float KP_VALUE = 10.0f;  // Vitesse max
//const float TRQ = 4.0f;  // Vitesse max en rad/s
//const float CURR = 10.0f;  // Vitesse max en rad/s
//const float DELAY = 200;  // Vitesse max en rad/s
//const float KI_SPEED = 0.008;  // Gain intégral pour la boucle de vitesse interne
//const float KI_VALUE = 0.1f;   // Gain intégral (petit pour stabilité)
//const float KD_VALUE = 0.5f;   // Gain dérivé (amortit oscillations)


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
    delay(1000);

    // 2. Initialisation des moteurs
    Serial.println("Initialisation des moteurs...");
    
    motor0.init_motor(MODE_POSITION);
    motor0.set_position_kp(10.0f);
    motor0.set_limit_speed(7.0f);
    motor0.set_limit_current(10.0f);
    motor0.set_limit_torque(5.0f);
    //motor0.set_speed_ki(0.008f);
    motor0.enable_motor();

    // 3. Définir la position actuelle comme zéro
    Serial.println("Définition de la position zéro...");
    set_mechanical_position_to_zero(MOTOR_ID_0, MASTER_ID);
    delay(1000);

    // Étape 10 : Envoyer une consigne de position (par exemple, 1.0 rad)
    Serial.println("Envoi d’une consigne de position (1.0 rad)...");
    motor0.set_position_ref(0.1);
    delay(2000);

    // Étape 11 : Ramener le moteur à la nouvelle position zéro (0.0 rad)
    Serial.println("Retour à la nouvelle position zéro (0.0 rad)...");
    motor0.set_position_ref(0.0f);
    delay(2000);
}

void loop() {
    // Lire l’état du bouton
    etatBouton = digitalRead(BUTTON_PIN);

    // Détecter un appui (logique inversée : HIGH → LOW)
    if (etatBouton == LOW && dernierEtatBouton == HIGH) {
        unsigned long currentTime = millis();
        if (currentTime - lastButtonPress >= 200) {  // Débouncing
            lastButtonPress = currentTime;
            motor0.set_position_ref(0.0);
            delay(2000);
            motor0.set_position_ref(0.8);
            delay(2000);
            motor0.set_position_ref(0.0);
            delay(2000);
            motor0.set_position_ref(0.8);
            delay(2000);
            motor0.set_position_ref(0.0);
            delay(2000);


        }


    }
    dernierEtatBouton = etatBouton;

    delay(10);  // Réduire la charge CPU
}
