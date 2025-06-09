#include <Arduino.h>
#include <Wire.h>
#include <ESP32Servo.h>
#include "rgb_lcd.h"
#include <BluetoothSerial.h>
#include "Vl53l0x.h"

#define PIN_SG90 27

#define PIN_MoteurGauche 25
#define PIN_MoteurDroit 4
#define PIN_SensMoteurGauche 26
#define PIN_SensMoteurDroit 23

#define PIN_CNY70_ExGauche 34 // 39
#define PIN_CNY70_Gauche 35   // 34
#define PIN_CNY70_Milieu 33   // 35
#define PIN_CNY70_Droit 39    // 32
#define PIN_CNY70_ExDroit 32  // 33

#define PIN_BP1 12
#define PIN_BP2 14
#define PIN_Pot 36
#define PIN_FDC 13

void lecture_capteur(void);
int potentiometre(void);
void CompteLigne(void);
void Droite(void);
void Gauche(void);
void RouleMaPoule(void);
int DemarageAutomatique(void);
void Asserv(void *);
void IHM(void *);
void TOFServo(void *);

rgb_lcd lcd;
BluetoothSerial SerialBT;
Servo sg90;
Vl53l0x TOF;

int val_TOF = 0;
int tab_capteur[5] = {0, 0, 0, 0, 0};            // Déclaration des tableaux pour les valeurs lues des CNY70
int tab_min[5] = {4096, 4096, 4096, 4096, 4096}; // Déclaration des tableaux pour les valeurs MIN des CNY70
int tab_max[5] = {0, 0, 0, 0, 0};                // Déclaration des tableaux pour les valeurs MAX des CNY70
int tab_capteur_norm[5] = {0, 0, 0, 0, 0};       // Déclaration des tableaux pour les valeurs normalis�es des CNY70

int frequence = 20000;
int canal0 = 0;
int canal3 = 3;
int canal2 = 2;
int resolution = 10;

int etat_course = 0;
int pos_ligne = 0;
int PWMGauche = 0;
int PWMDroit = 0;

int v_base = 800; // V MAX = 1024
int pos, ancienne_pos = 0, cmd;

int val_BP1 = 0, val_BP2 = 0;
int val_pot;
float KP = 7.8, KpGlo = 7.8;
float KD = 11, KdGlo = 11;
int menu = 0, menu1 = 0, fdc = 0, Lig = 0, i = 0, Lid = 0, Clig = 0, Clid = 0, add = 0, Dem = 0, E1 = 1, E2 = 0, E3 = 0, E4 = 0, E5 = 0, E6 = 0, E7 = 0, E8 = 0, E9 = 0, E10 = 0, nor = 0;
char caractere = 0;
int old_val_ExG, old_val_ExD;
int Ligne = 0;
int prout = 0;
int etatTof = 0;
int ratrape = 0;
float tab_bidon[100];

TaskHandle_t Asservissement;
TaskHandle_t IHMM;

void setup()
{
    Wire.setClock(400000);
    Serial.begin(115200);
    lcd.begin(16, 2);      // définition du format de l'écran
    lcd.setRGB(100, 0, 0); // définition du rapport de couleur pour le rétroéclairage
    SerialBT.begin("WALL-E");

    // Setup CNY70 //
    pinMode(PIN_CNY70_ExGauche, INPUT);
    pinMode(PIN_CNY70_Gauche, INPUT);
    pinMode(PIN_CNY70_Milieu, INPUT);
    pinMode(PIN_CNY70_Droit, INPUT);
    pinMode(PIN_CNY70_ExDroit, INPUT);
    /////////////////

    // Setup PWM //
    ledcSetup(canal2, frequence, resolution);
    // ledcSetup(canal3,frequence,resolution);
    ledcAttachPin(PIN_MoteurGauche, canal2);
    ledcAttachPin(PIN_MoteurDroit, canal3);
    pinMode(PIN_SensMoteurDroit, OUTPUT);
    pinMode(PIN_SensMoteurGauche, OUTPUT);
    digitalWrite(PIN_SensMoteurDroit, HIGH);
    digitalWrite(PIN_SensMoteurGauche, LOW);
    ledcWrite(canal2, 0);
    ledcWrite(canal3, 0);
    ///////////////

    pinMode(PIN_BP1, INPUT_PULLDOWN);
    pinMode(PIN_BP2, INPUT_PULLDOWN);
    pinMode(PIN_Pot, INPUT);
    pinMode(PIN_FDC, INPUT);

    ledcWrite(canal2, 0);
    ledcWrite(canal3, 0);
    VL53L0X_Error status = VL53L0X_ERROR_NONE;
    status = TOF.begin(I2C_DEFAULT_ADDR, true);
    if (VL53L0X_ERROR_NONE != status)
    {
        Serial.println("start vl53l0x mesurement failed!");
        TOF.printPalError(status);
        while (1)
            ;
    }
    TOF.continuousRangingInit();
    if (VL53L0X_ERROR_NONE != status)
    {
        Serial.println("start vl53l0x mesurement failed!");
        TOF.printPalError(status);
        while (1)
            ;
    }
    xTaskCreate(Asserv, "Asserv", 16384, NULL, 15, &Asservissement);
    xTaskCreate(IHM, "IHM", 8192, NULL, 12, &IHMM);
    xTaskNotifyGive(IHMM);
}

void loop()
{
    vTaskDelay(pdMS_TO_TICKS(1000000000));
}

void lecture_capteur(void)
{
    tab_capteur[0] = analogRead(PIN_CNY70_ExGauche);
    tab_capteur[1] = analogRead(PIN_CNY70_Gauche);
    tab_capteur[2] = analogRead(PIN_CNY70_Milieu);
    tab_capteur[3] = analogRead(PIN_CNY70_Droit);
    tab_capteur[4] = analogRead(PIN_CNY70_ExDroit);
    // Normalisation des capteurs grâce à la relation du cours /////////////
    for (int i = 0; i < 5; i++)
    {
        tab_capteur_norm[i] = 100.0 * ((float)tab_capteur[i] - (float)tab_min[i]) / ((float)tab_max[i] - (float)tab_min[i]); // Valeur entre 0 et 100
    }
    pos = (tab_capteur_norm[1] - tab_capteur_norm[3]); // Valeur Gauche - Droit
    add = tab_capteur_norm[1] + tab_capteur_norm[3];   // Gauche + Droit
                                                       // if(add>160&&pos>0)pos = 100;                                //saturation de l'erreurAdd commentMore actions
                                                       // if(add>160&&pos<0)pos = -100;
}
int potentiometre(void)
{
    int ValeurADC_Pot;
    int Pot;
    ValeurADC_Pot = analogRead(PIN_Pot);  // Lit la valeur analogique sur le bit 0
    Pot = ((float)ValeurADC_Pot / 40.95); // Calcule la valeur du potentiomètre entre 0 et 100
    return Pot;
}

void CompteLigne(void)
{
    switch (Clid)
    {
    case 1:
        if (tab_capteur_norm[4] < 15)
            Lid++, Clid = 0;
        break;
    case 0:
        if (tab_capteur_norm[4] > 85)
            Clid = 1; // compteur de ligne à droite
        break;
    }
    switch (Clig)
    {
    case 1:
        if (tab_capteur_norm[0] < 15)
            Lig++, Clig = 0;
        break;
    case 0:
        if (tab_capteur_norm[0] > 85)
            Clig = 1; // compteur de ligne à gauche
        break;
    }
}

void Droite(void)
{
    PWMGauche = 500;
    PWMDroit = 500;
    digitalWrite(PIN_SensMoteurDroit, LOW);
    digitalWrite(PIN_SensMoteurGauche, LOW);
}

void Gauche(void)
{
    PWMGauche = 500;
    PWMDroit = 500;
    digitalWrite(PIN_SensMoteurDroit, HIGH);
    digitalWrite(PIN_SensMoteurGauche, HIGH);
}

void RouleMaPoule(void)
{

    digitalWrite(PIN_SensMoteurDroit, HIGH);
    digitalWrite(PIN_SensMoteurGauche, LOW);
    switch (ratrape)
    {
    case 0:
        cmd = KP * pos + KD * (pos - ancienne_pos); // Calcul de la valeur de l'ordre de correction avec Kp et Kd
        ancienne_pos = pos;                         // La valeur de 'ancienne pos' devient la valeur de la 'pos' actuelle, utile pour le calcul de d�riv�e
        PWMGauche = v_base + cmd;                   // Calcul Vitesse Moteur Gauche
        PWMDroit = v_base - cmd;                    // Calcul Vitesse Moteur Droit
        if (tab_capteur_norm[2] > 90 && tab_capteur_norm[3] < 30 && Lid >= 1)
            ratrape = 2;
        if (tab_capteur_norm[2] > 90 && tab_capteur_norm[1] < 30 && Lid >= 1)
            ratrape = 1;
        break;
    case 1:
        PWMDroit = 600, PWMGauche = 0;
        if (tab_capteur_norm[2] < 40)
            ratrape = 0;
        break;
    case 2:
        PWMDroit = 0, PWMGauche = 600;
        if (tab_capteur_norm[2] < 40)
            ratrape = 0;
        break;
    }
}

int DemarageAutomatique(void)
{
    int etat = 0, temp = 0;
    Dem = 0;
    temp = millis();
    while (etat != 5)
    {
        tab_capteur[1] = analogRead(PIN_CNY70_Gauche);
        tab_capteur[2] = analogRead(PIN_CNY70_Milieu);
        tab_capteur[3] = analogRead(PIN_CNY70_Droit);
        tab_capteur[4] = analogRead(PIN_CNY70_ExDroit);
        // Étalonnage des capteurs pour récupérer les valeurs Min et Max entre 0 et 1023
        for (i = 0; i < 5; i++)
        { // On répéte 5 fois, et i prends les valeurs 0, 1, 2, 3 et 4
            if (tab_capteur[i] > tab_max[i])
            {                                // Si la valeur du CNY70, à l'indice i, est supérieure à son ancienne valeur max
                tab_max[i] = tab_capteur[i]; //      Alors la valeur du CNY70, à l'indice i, devient la nouvelle valeur max du capteur
            }
            if (tab_capteur[i] < tab_min[i])
            {                                // Si la valeur du CNY70, à l'indice i, est inférieure à son ancienne valeur min
                tab_min[i] = tab_capteur[i]; //      Alors la valeur du CNY70, à l'indice i, devient la nouvelle valeur min du capteur
            }
        }
        lcd.setCursor(0, 0);
        lcd.printf("AutoScale");
        lcd.setCursor(0, 1);
        lcd.printf("Fait: En cours");
        Droite();
        lecture_capteur();
        ledcWrite(canal2, PWMGauche);
        ledcWrite(canal3, PWMDroit);
        if ((millis() - temp) >= 2800)
        {
            etat = 5;
        }
    }
    lcd.clear();
    return 1;
}
void Asserv(void *)
{
    VL53L0X_RangingMeasurementData_t rangingMeasurementData;
    bool Finish = false;
    int TOF_Val = 1000, temp = 0;
    while (1)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        while (!Finish)
        {
            if (millis() - temp >= 20)
            {
                TOF.performContinuousRangingMeasurement(&rangingMeasurementData);
                TOF_Val = rangingMeasurementData.RangeMilliMeter;
                temp = millis();
            }
            lecture_capteur();
            fdc = digitalRead(PIN_FDC);
            if ((fdc != 0) || ((Lig >= 1) && (E10 == 1)))
                Finish = true;
            switch (nor)
            {
            case 0:
                v_base = 500, KP = 4, KD = 15;
                break;
            case 1:
                v_base = 800, KP = 7.8, KD = 11;
                break;
            }
            switch (etat_course)
            {
            case 0:
                if (val_BP1 == 1 && val_BP2 == 1)
                {
                    etat_course = 1;
                }
                if (SerialBT.available()) // vérification de l'appui d'une touche
                {
                    caractere = SerialBT.read(); // enregistrement de la touche presser
                    Serial.printf("C=%c", caractere);
                    if (caractere == 'D')
                        etat_course = 1; // prêt à démarer
                }
                PWMGauche = 0;
                PWMDroit = 0;
                break;
            case 1: // Suivi de ligne
                switch (pos_ligne)
                {
                case 0: // Le robot est centré sur la ligne
                    if (ratrape == 0)
                        CompteLigne();
                    RouleMaPoule();
                    if (0 || (ratrape == 0 && tab_capteur_norm[0] > 80 && tab_capteur_norm[1] > 80 && tab_capteur_norm[2] > 80 && tab_capteur_norm[3] > 80 && tab_capteur_norm[4] > 80))
                    {
                        PWMGauche = 1023;
                        PWMDroit = 1023;
                    }
                    if (Lig == 2 && E1 == 1)
                        pos_ligne = 2;
                    if (Lig == 1 && E2 == 1)
                        nor = 0;
                    if (Lig == 2 && E2 == 1)
                        pos_ligne = 3, Lid = 0;
                    if (Lid == 1 && E3 == 1)
                        nor = 0;
                    if (Lid == 2 && E3 == 1)
                        pos_ligne = 4, Lid = 0;
                    if (Lig == 5 && E4 == 1)
                        nor = 0;
                    if (Lid == 3 && E4 == 1)
                        pos_ligne = 5, Lig = 0;
                    if (Lid == 1 && E5 == 1)
                        nor = 0;
                    if (Lid == 2 && E5 == 1)
                        pos_ligne = 6, Lig = 0;
                    if (Lid == 1 && E6 == 1)
                        nor = 0;
                    if (Lid == 2 && E6 == 1)
                        pos_ligne = 7, Lig = 0;
                    if (Lid == 1 && E7 == 1)
                        pos_ligne = 8, Lig = 0;
                    if (Lid == 1 && E8 == 1)
                        pos_ligne = 9, Lid = 0;
                    if (Lid == 1 && E9 == 1)
                        pos_ligne = 10, Lid = 0;
                    if (TOF_Val <= 300 && 1)
                        etat_course = 3;
                    break;
                case 1:
                    PWMGauche = 0;
                    PWMDroit = 0;
                    break;
                case 2:
                    Gauche();
                    CompteLigne();
                    if (tab_capteur_norm[2] < 20 && Lig == 3)
                    {
                        Lig = 0;
                        Lid = 0;
                        E1 = 0;
                        E2 = 1;
                        Clid = 0;
                        Clig = 0;
                        pos_ligne = 0;
                        nor = 1;
                    }
                    break;
                case 3:
                    Gauche();
                    CompteLigne();
                    if (tab_capteur_norm[2] < 10 && Lid == 1)
                    {
                        E2 = 0;
                        E3 = 1;
                        Lid = 0;
                        Lig = 0;
                        Clid = 0;
                        Clig = 0;
                        pos_ligne = 0;
                        nor = 1;
                    }
                    break;
                case 4:
                    Droite();
                    CompteLigne();
                    if (tab_capteur_norm[2] < 10 && Lid == 1)
                    {
                        E3 = 0;
                        E4 = 1;
                        Lid = 0;
                        Lig = 0;
                        Clid = 0;
                        Clig = 0;
                        nor = 1;
                        pos_ligne = 0;
                    }
                    break;
                case 5:
                    Droite();
                    CompteLigne();
                    if (tab_capteur_norm[2] < 10 && Lig == 1)
                    {
                        E4 = 0;
                        E5 = 1;
                        Lig = 0;
                        Lid = 0;
                        Clid = 0;
                        Clig = 0;
                        nor = 1;
                        pos_ligne = 0;
                    }
                    break;
                case 6:
                    Gauche();
                    CompteLigne();
                    if (Lig >= 1 && tab_capteur_norm[2] < 10&&tab_capteur_norm[4]>80)
                    {
                        E5 = 0;
                        E6 = 1;
                        Lid = 0;
                        Lig = 0;
                        Clig = 0;
                        Clid = 0;
                        nor = 1;
                        pos_ligne = 0;
                    }
                    break;
                case 7:
                    Gauche();
                    CompteLigne();
                    if (Lig == 1 && tab_capteur_norm[2] < 10)
                    {
                        E6 = 0;
                        E7 = 1;
                        Lig = 0;
                        Lid = 1;
                        Clig = 0;
                        Clid = 0;
                        pos_ligne = 0;
                    }
                    break;
                case 8:
                    Gauche();
                    CompteLigne();
                    if (Lig >= 1 && tab_capteur_norm[2] < 10&&tab_capteur_norm[4]>80)
                    {
                        E7 = 0;
                        E8 = 1;
                        Lig = 0;
                        Lid = 0;
                        Clig = 0;
                        Clid = 0;
                        pos_ligne = 0;
                    }
                    break;
                case 9:
                    Gauche();
                    CompteLigne();
                    if (Lid == 1 && tab_capteur_norm[2] < 10)
                    {
                        E8 = 0;
                        E9 = 1;
                        Lid = 0;
                        Lig = 0;
                        Clig = 0;
                        Clid = 0;
                        pos_ligne = 0;
                    }
                    break;
                case 10:
                    Gauche();
                    CompteLigne();
                    if (Lid == 1 && tab_capteur_norm[2] < 10)
                    {
                        E9 = 0;
                        E10 = 1;
                        Lid = 0;
                        Lig = 0;
                        Clig = 0;
                        Clid = 0;
                        pos_ligne = 0;
                    }
                    break;
                default:
                    pos_ligne = 0;
                    break;
                }
                break;
            case 2:
                PWMGauche = 00;
                PWMDroit = 00;
                if (val_BP2 == 1)
                    etat_course = 0, Lid = 0, Lig = 0;
                if (SerialBT.available()) // vérification de l'appui d'une touche
                {
                    caractere = SerialBT.read(); // enregistrement de la touche presser
                    Serial.printf("C=%c", caractere);
                    if (caractere == 'R')
                        etat_course = 0; // prêt à démarer
                }
                break;
            case 3:
                PWMGauche = 0;
                PWMDroit = 0;
                CompteLigne();
                if (TOF_Val >= 400)
                    pos_ligne = 0, etat_course = 1;

                break;
            default:
                PWMGauche = 0; // arret par defaut
                PWMDroit = 0;
                etat_course = 0;
                E1 = 1, E2 = 0, E3 = 0, E4 = 0, E5 = 0, E6 = 0, E7 = 0;
                break;
            }
            if (PWMGauche > 818)
                PWMGauche = 818; // saturation des moteurs
            if (PWMDroit > 818)
                PWMDroit = 818;
            if (PWMGauche < 0)
                PWMGauche = 0;
            if (PWMDroit < 0)
                PWMDroit = 0;
            ledcWrite(canal2, PWMGauche);
            ledcWrite(canal3, PWMDroit);
            Serial.printf(" etat_course=%1d\n", etat_course);
            vTaskDelay(pdMS_TO_TICKS(5));
        }
        PWMDroit = 0;
        PWMGauche = 0;
        ledcWrite(canal2, PWMGauche);
        ledcWrite(canal3, PWMDroit);
        Finish = false;
        xTaskNotifyGive(IHMM);
    }
}
void IHM(void *)
{
    bool finish = false;
    while (1)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        while (!finish)
        {
            PWMDroit = 0;
            PWMGauche = 0;
            ledcWrite(canal2, PWMGauche);
            ledcWrite(canal3, PWMDroit);
            menu = val_pot / 10;
            val_BP1 = digitalRead(PIN_BP1);
            Serial.printf("BP1=%1d", val_BP1);
            val_BP2 = digitalRead(PIN_BP2);
            Serial.printf("BP2=%1d", val_BP2);
            val_pot = potentiometre();
            Serial.printf("potentiomètre=%3d", val_pot);
            fdc = digitalRead(PIN_FDC);
            Serial.printf("FDC=%1d\n", fdc);
            lecture_capteur();
            if (menu != menu1)
            {
                menu1 = menu;
                lcd.clear();
            }

            switch (menu)
            {
            case 0:
                lcd.setCursor(0, 0);
                lcd.printf("MenuModif");
                lcd.setCursor(0, 1);
                lcd.printf("Etalonage");
                // Lecture des valeurs analogiques des 5 CNY70 /////////////////////////
                tab_capteur[0] = analogRead(PIN_CNY70_ExGauche);
                tab_capteur[1] = analogRead(PIN_CNY70_Gauche);
                tab_capteur[2] = analogRead(PIN_CNY70_Milieu);
                tab_capteur[3] = analogRead(PIN_CNY70_Droit);
                tab_capteur[4] = analogRead(PIN_CNY70_ExDroit);
                // Étalonnage des capteurs pour récupérer les valeurs Min et Max entre 0 et 1023
                for (i = 0; i < 5; i++)
                { // On répéte 5 fois, et i prends les valeurs 0, 1, 2, 3 et 4
                    if (tab_capteur[i] > tab_max[i])
                    {                                // Si la valeur du CNY70, à l'indice i, est supérieure à son ancienne valeur max
                        tab_max[i] = tab_capteur[i]; //      Alors la valeur du CNY70, à l'indice i, devient la nouvelle valeur max du capteur
                    }
                    if (tab_capteur[i] < tab_min[i])
                    {                                // Si la valeur du CNY70, à l'indice i, est inférieure à son ancienne valeur min
                        tab_min[i] = tab_capteur[i]; //      Alors la valeur du CNY70, à l'indice i, devient la nouvelle valeur min du capteur
                    }
                }
                if (val_BP1 == 1 && val_BP2 == 0)
                {
                    for (i = 0; i < 5; i++)
                    {                      // On répéte 5 fois, et i prends les valeurs 0, 1, 2, 3 et 4
                        tab_max[i] = 0;    //      Alors la valeur du CNY70, à l'indice i, devient la nouvelle valeur max du capteur
                        tab_min[i] = 4096; //      Alors la valeur du CNY70, à l'indice i, devient la nouvelle valeur min du capteur
                        lcd.setCursor(0, 1);
                        lcd.printf("Reset");
                        Lig = 0;
                        Lid = 0;
                        E1 = 1, E2 = 0, E3 = 0, E4 = 0, E5 = 0;
                    }
                }
                break;
            case 1:
                lcd.setCursor(0, 0);
                lcd.printf("KP:%1.2f", KP);
                lcd.setCursor(0, 1);
                lcd.printf("Pot:%d", val_pot);
                if (val_BP2 == 1)
                {
                    while (val_BP1 == 0)
                    {
                        val_BP1 = digitalRead(PIN_BP1);
                        val_BP2 = digitalRead(PIN_BP2);
                        lcd.setCursor(0, 0);
                        lcd.printf("KP:%1.2f", KP);
                        lcd.setCursor(0, 1);
                        lcd.printf("Pot:%d", val_pot);
                        val_pot = potentiometre();
                        KP = (float)val_pot / 10.0;
                    }
                    delay(1000);
                    val_BP1 = 0;
                    val_BP2 = 0;
                }
                break;
            case 2:
                lcd.setCursor(0, 0);
                lcd.printf("KD:%2.2f", KD);
                lcd.setCursor(0, 1);
                lcd.printf("Pot:%d", val_pot);
                if (val_BP2 == 1)
                {
                    while (val_BP1 == 0)
                    {
                        val_BP1 = digitalRead(PIN_BP1);
                        val_BP2 = digitalRead(PIN_BP2);
                        lcd.setCursor(0, 0);
                        lcd.printf("KD:%2.2f", KD);
                        lcd.setCursor(0, 1);
                        lcd.printf("Pot:%d", val_pot);
                        val_pot = potentiometre();
                        KD = (float)val_pot;
                    }
                    delay(1000);
                    val_BP1 = 0;
                    val_BP2 = 0;
                }
                break;
            case 3:
                lcd.setCursor(0, 0);
                lcd.printf("VIT:%4d", v_base);
                lcd.setCursor(0, 1);
                lcd.printf("Pot:%d", val_pot);
                if (val_BP2 == 1)
                {
                    while (val_BP1 == 0)
                    {
                        val_BP1 = digitalRead(PIN_BP1);
                        val_BP2 = digitalRead(PIN_BP2);
                        lcd.setCursor(0, 0);
                        lcd.printf("VIT:%4d", v_base);
                        lcd.setCursor(0, 1);
                        lcd.printf("Pot:%d", val_pot);
                        val_pot = potentiometre();
                        v_base = (float)val_pot * 10.0;
                    }

                    delay(1000);
                    val_BP1 = 0;
                    val_BP2 = 0;
                }
                break;
            case 4:
                lcd.setCursor(0, 0);
                lcd.printf("Autoscale");
                lcd.setCursor(0, 1);
                lcd.printf("Fait: %d", Dem);
                if (val_BP1 == 1 && Dem == 0)
                {
                    Dem = DemarageAutomatique();
                }
                break;
            case 5:
                lcd.setCursor(0, 0);
                lcd.printf("etat:%1d Lid:%2d", etat_course, Lid);
                lcd.setCursor(0, 1);
                lcd.printf("pos:%3dLig:%2d", pos, Lig);
                break;
            case 6:
                lcd.setCursor(0, 0);
                lcd.printf("CedN:%3dCidN:%3d", tab_capteur_norm[4], tab_capteur_norm[3]);
                lcd.setCursor(0, 1);
                lcd.printf("CegN:%3dCigN:%3d", tab_capteur_norm[0], tab_capteur_norm[1]);
                break;
            case 7:
                lcd.setCursor(0, 0);
                lcd.printf("Ced:%4dCid:%4d", tab_capteur[4], tab_capteur[3]);
                lcd.setCursor(0, 1);
                lcd.printf("Ceg:%4dCig:%4d", tab_capteur[0], tab_capteur[1]);
                Serial.printf("Ced:%4d Cid:%4d Ceg:%4d Cig:%4d", tab_capteur[4], tab_capteur[3], tab_capteur[0], tab_capteur[1]);
                break;
            case 8:
                lcd.setCursor(0, 0);
                lcd.printf("PWG:%4dPWD:%4d", PWMGauche, PWMDroit);
                lcd.setCursor(0, 1);
                lcd.printf("Err:%3dAdd:%3d", pos, add);
                break;
            }
            if (val_BP1 == 1 && val_BP2 == 1)
                finish = true;
        }
        finish = false;
        lcd.clear();
        etatTof = 1;
        xTaskNotifyGive(Asservissement);
    }
}