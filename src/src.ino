/**
 * @file      src.ino
 * @author    Chayanon Pitak (Chayanon_Pitak@cmu.ac.th)
 */

#include "utilities.h"
#include "MQTT_credentials.h"
#include "MQTT_cert.h"
#include <Arduino.h>
#include <SoftwareSerial.h>
#include <TinyGsmClient.h>
#include <time.h>
#include <TinyGPSPlus.h>

#define TINY_GSM_RX_BUFFER 1024 // Set RX buffer to 1Kb

// Define the serial console for debug prints, if needed
#define TINY_GSM_DEBUG TinyGSMSerialMonitor

// See all AT commands, if wanted
// #define DUMP_AT_COMMANDS

#ifdef DUMP_AT_COMMANDS
#include <StreamDebugger.h>
StreamDebugger debugger(SerialAT, SerialMon);
TinyGsm modem(debugger);
#else
TinyGsm modem(SerialAT);
#endif

TinyGPSPlus gps;
SoftwareSerial gpsss(GPS_RX, GPS_TX);

static void writeRGB(int r, int g, int b)
{
    ledcWrite(0, 255-r);
    ledcWrite(1, 255-g);
    ledcWrite(2, 255-b);
}

static void printTime()
{
    Serial.print("[");
    Serial.print(millis()/1000.0);
    Serial.print("] \t");
}

// This custom version of delay() ensures that the gps object
// is being "fed".
static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (gpsss.available())
      gps.encode(gpsss.read());
  } while (millis() - start < ms);
}

// Read analog pin and return the median value of the last n readings.
static unsigned short analogDownsamplingReadMedian(uint8_t pin, unsigned int samplingRate = 5)
{
    unsigned short* results = (unsigned short*)malloc(samplingRate * sizeof(short));
    for (unsigned int i = 0; i < samplingRate; i++)
    {
        results[i] = analogRead(pin);
    }
    for (unsigned int i = 0; i < samplingRate; i++)
    {
        for (unsigned int j = i + 1; j < samplingRate; j++)
        {
            if (results[i] > results[j])
            {
                unsigned short temp = results[i];
                results[i] = results[j];
                results[j] = temp;
            }
        }
    }
    unsigned short result = results[samplingRate / 2] + (samplingRate % 2 == 0 ? results[samplingRate / 2 - 1] : 0);
    free(results);
    return result;
}

// Read analog pin pair and return array of the downsampled pair.
static unsigned short* readPair(short pin0, short pin1) 
{
    unsigned short* result = (unsigned short*)malloc(2 * sizeof(short));
    result[0] = analogDownsamplingReadMedian(pin0);
    result[1] = analogDownsamplingReadMedian(pin1);
    return result;
}

void setup()
{
    Serial.begin(115200);

    // Set status light up / Status LED check
    ledcAttachPin(STATUS_LED_RED, 0);
    ledcAttachPin(STATUS_LED_GREEN, 1);
    ledcAttachPin(STATUS_LED_BLUE, 2);
    ledcSetup(0, 12000, 8);
    ledcSetup(1, 12000, 8);
    ledcSetup(2, 12000, 8);
    pinMode(LED_IN1, OUTPUT);
    pinMode(LED_OUT1, OUTPUT);
    pinMode(LED_IN2, OUTPUT);
    pinMode(LED_OUT2, OUTPUT);
    digitalWrite(LED_IN1, HIGH);
    digitalWrite(LED_OUT1, HIGH);
    digitalWrite(LED_IN2, HIGH);
    digitalWrite(LED_OUT2, HIGH);
    writeRGB(0, 0, 0);
    delay(500);
    writeRGB(255, 0, 0);
    delay(500);
    writeRGB(0, 255, 0);
    delay(500);
    writeRGB(0, 0, 255);
    delay(500);
    writeRGB(0, 0, 0);

    /* STATUS LIGHT
    * YELLOW    Setting modem & SIM card up
    * RED       ERROR
    * GREEN     SIM card & Network online
    */

    // Set modem up
    writeRGB(255, 255, 0);  // Yellow
    SerialAT.begin(115200, SERIAL_8N1, MODEM_RX_PIN, MODEM_TX_PIN);
    pinMode(BOARD_POWERON_PIN, OUTPUT);
    digitalWrite(BOARD_POWERON_PIN, HIGH);

    pinMode(MODEM_RESET_PIN, OUTPUT);
    digitalWrite(MODEM_RESET_PIN, !MODEM_RESET_LEVEL); delay(100);
    digitalWrite(MODEM_RESET_PIN, MODEM_RESET_LEVEL); delay(2600);
    digitalWrite(MODEM_RESET_PIN, !MODEM_RESET_LEVEL);

    pinMode(BOARD_PWRKEY_PIN, OUTPUT);
    digitalWrite(BOARD_PWRKEY_PIN, LOW);
    delay(100);
    digitalWrite(BOARD_PWRKEY_PIN, HIGH);
    delay(100);
    digitalWrite(BOARD_PWRKEY_PIN, LOW);

    // Check if the modem is online
    printTime();
    Serial.println("Starting modem ");

    int retry = 0;
    while (!modem.testAT(1000)) {
        Serial.print(".");
        if (retry++ > 10) {
            digitalWrite(BOARD_PWRKEY_PIN, LOW);
            delay(100);
            digitalWrite(BOARD_PWRKEY_PIN, HIGH);
            delay(1000);
            digitalWrite(BOARD_PWRKEY_PIN, LOW);
            retry = 0;
        }
    }
    Serial.println();

    // Check if SIM card is online
    SimStatus sim = SIM_ERROR;
    while (sim != SIM_READY) {
        sim = modem.getSimStatus();
        switch (sim) {
        case SIM_READY:
            printTime();
            Serial.println("SIM card online");
            break;
        case SIM_LOCKED:
            printTime();
            Serial.println("The SIM card is locked. Please unlock the SIM card first.");
            // const char *SIMCARD_PIN_CODE = "123456";
            // modem.simUnlock(SIMCARD_PIN_CODE);
            writeRGB(255, 0, 0);    // Red
            break;
        default:
            break;
        }
        delay(1000);
    }

    // Check network registration status and network signal status
    int16_t sq ;
    printTime();
    Serial.print("Wait for the modem to register with the network.");
    RegStatus status = REG_NO_RESULT;
    while (status == REG_NO_RESULT || status == REG_SEARCHING || status == REG_UNREGISTERED) {
        status = modem.getRegistrationStatus();
        switch (status) {
        case REG_UNREGISTERED:
        case REG_SEARCHING:
            sq = modem.getSignalQuality();
            printTime();
            Serial.printf("Signal Quality:%d", millis() / 1000, sq);
            delay(1000);
            break;
        case REG_DENIED:
            printTime();
            Serial.println("Network registration was rejected, please check if the APN is correct");
            writeRGB(255, 0, 0);    // Red
            return ;
        case REG_OK_HOME:
            printTime();
            Serial.println("Online registration successful");
            break;
        case REG_OK_ROAMING:
            printTime();
            Serial.println("Network registration successful, currently in roaming mode");
            break;
        default:
            printTime();
            Serial.printf("Registration Status:%d\n", status);
            delay(1000);
            break;
        }
    }
    Serial.println();
    printTime();
    Serial.printf("Registration Status:%d\n", status);
    Serial.println();

    String ueInfo;
    if (modem.getSystemInformation(ueInfo)) {
        printTime();
        Serial.print("Inquiring UE system information:");
        Serial.println(ueInfo);
    }

    if (!modem.enableNetwork()) {
        printTime();
        Serial.println("Enable network failed!");
        writeRGB(255, 0, 0);    // Red
        return;
    }

    delay(1000);

    /* STATUS LIGHT
    * YELLOW    Setting NTP up
    * RED       ERROR
    * GREEN     NTP online
    */

    // Set NTP up
    writeRGB(255, 255, 0);  // Yellow
    configTime(0, 0, NTP_SERVER);
    struct tm timeinfo;
    if(!getLocalTime(&timeinfo))
    {
        printTime();
        Serial.println("Failed to obtain time");
        writeRGB(255, 0, 0);    // Red
        return;
    }
    printTime();
    Serial.print("Time obtained: ");
    Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
    writeRGB(0, 255, 0);    // Green
    delay(1000);


    /* STATUS LIGHT
    * YELLOW    Setting MQTT up
    * RED       ERROR
    * GREEN     MQTT online
    */

    // Set MQTT up

    writeRGB(255, 255, 0);  // Yellow
    modem.mqtt_begin(true, true);   // SSL, SNI
    modem.mqtt_set_certificate(MQTT_CERT);

    printTime();
    Serial.print("Connecting to ");
    Serial.print(MQTT_BROKER);

    bool mqtt_status = modem.mqtt_connect(0, MQTT_BROKER, MQTT_BROKER_PORT, MQTT_CLIENT_ID, MQTT_BROKER_USERNAME, MQTT_BROKER_PASSWORD);
    if (!mqtt_status || !modem.mqtt_connected()) {
        printTime();
        Serial.print("Failed to connect to ");
        Serial.println(MQTT_BROKER);
        writeRGB(255, 0, 0);    // Red
        return;
    }

    printTime();
    Serial.println("Connected to MQTT broker");
    writeRGB(0, 255, 0);    // Green
    delay(1000);


    /* STATUS LIGHT
    * YELLOW    Setting GPS up, waiting for first GPS data
    * GREEN     First GPS data received
    */

    // Set GPS up
    writeRGB(255, 255, 0);  // Yellow
    gpsss.begin(GPS_BAUD);
    
    while (true)
    {
        gps.encode(gpsss.read());
        if (gps.location.isValid())
        {
            printTime();
            Serial.print("First GPS data received: ");
            Serial.print(gps.location.lat(), 6);
            Serial.print(", ");
            Serial.println(gps.location.lng(), 6);
            writeRGB(0, 255, 0);    // Green
            break;
        }
        else 
        {
            printTime();
            Serial.println("No GPS data received yet");
            smartDelay(5000);
        }
    }

    delay(1000);

    // TODO: GET STATIONS LOCATION

    /* STATUS LIGHT
    * YELLOW    Set other PINs up
    * GREEN     PINs ready
    */

    // Set other PINs up
    writeRGB(255, 255, 0);  // Yellow
    pinMode(IN1, INPUT);
    pinMode(OUT1, OUTPUT);
    pinMode(IN2, INPUT);
    pinMode(OUT2, OUTPUT);
    pinMode(POT_IN1, INPUT);
    pinMode(POT_OUT1, OUTPUT);
    pinMode(POT_IN2, INPUT);
    pinMode(POT_OUT2, OUTPUT);

    printTime();
    Serial.println("Pin setup completed");
    writeRGB(0, 255, 0);    // Green

    smartDelay(1000);

    /* STATUS LIGHT
    * YELLOW    Creating tasks
    * GREEN     Tasks ready
    */

    // Create tasks
    printTime();
    Serial.println("Creating tasks");
    writeRGB(0, 255, 0);    // Yellow
    xTaskCreatePinnedToCore(countPeople, "task_1", 10000, NULL, 3, NULL, 0);
    xTaskCreatePinnedToCore(checkStation, "task_2", 10000, NULL, 2, NULL, 1);
    xTaskCreatePinnedToCore(checkSerial, "task_3", 10000, NULL, 1, NULL, 1);
    writeRGB(255, 0, 0);    // Green

    // End of setup
    printTime();
    Serial.println("Setup completed");
    Serial.println("=============================================");
    Serial.println(" Send 'd' to toggle debug mode (default off)");
    Serial.println(" Send 'r' to reset counters");
    Serial.println("=============================================");
}

void loop() {}

bool debug = false;

// State machine states
enum state
{
    IDLE = 0,
    SEMI_OUT_1 = -1,
    SEMI_OUT_2 = -2,
    SEMI_OUT_3 = -3,
    OUT = -4,
    SEMI_IN_1 = 1,
    SEMI_IN_2 = 2,
    SEMI_IN_3 = 3,
    IN = 4,
    SPECIAL = 6
};
state currentState1 = IDLE;
state currentState2 = IDLE;

state setState(state currentState, unsigned short* distances, unsigned short* bounds)
{
    bool inside = distances[0] > bounds[0];
    bool outside = distances[1] > bounds[1];
    switch (currentState)
    {
        case IDLE: // 0 0
            if (!inside && outside) return SEMI_IN_1;
            if (inside && !outside) return SEMI_OUT_1;
            if (inside && outside) return SPECIAL;
            break;
        // IN
        case SEMI_IN_1: // 0 1
            if (inside && outside) return SEMI_IN_2;
            if (inside && !outside) return SEMI_IN_3;
            if (inside && !outside) return SEMI_IN_3;
            if (!inside && !outside) return IDLE;
            break;
        case SEMI_IN_2: // 1 1
            if (inside && !outside) return SEMI_IN_3;
            if (!inside && !outside) return IN;
            if (!inside && outside) return SEMI_IN_1;
            break;
        case SEMI_IN_3: // 1 0
            if (!inside && !outside) return IN;
            if (inside && outside) return SEMI_IN_2;
            if (!inside && outside) return SEMI_IN_1;
            if (!inside && outside) return SEMI_IN_1;
            break;
        // OUT
        case SEMI_OUT_1: // 1 0
            if (inside && outside) return SEMI_OUT_2;
            if (!inside && outside) return SEMI_OUT_3;
            if (!inside && outside) return SEMI_OUT_3;
            if (!inside && !outside) return IDLE;
            break;
        case SEMI_OUT_2: // 1 1
            if (!inside && outside) return SEMI_OUT_3;
            if (!inside && !outside) return OUT;
            if (inside && !outside) return SEMI_OUT_1;
            break;
        case SEMI_OUT_3: // 0 1
            if (!inside && !outside) return OUT;
            if (inside && outside) return SEMI_OUT_2;
            if (inside && !outside) return SEMI_OUT_1;
            if (inside && !outside) return SEMI_OUT_1;
            break;
        // SPECIAL
        case SPECIAL: // 1 1
            if (!inside && outside) return SEMI_OUT_3;
            if (inside && !outside) return SEMI_IN_3;
            if (!inside && !outside) return IDLE;
            break;
    }
    return currentState;
}

// Occupancy counter
short people = 0;
short in = 0;
short out = 0;

void resetCounters()
{
    currentState1 = IDLE;
    currentState2 = IDLE;
    people = 0;
    in = 0;
    out = 0;
    if (debug) 
    {
        struct tm timeinfo;
        getLocalTime(&timeinfo);
        printTime();
        Serial.print("Resettted at ");
        Serial.print(&timeinfo, "%A, %B %d %Y %H:%M:%S");
        Serial.print(", people: ");
    }
}

void setOccupancy(state* currentState)
{
    switch (*currentState)
    {
        case OUT:
            if (people > 0) 
            {
                people--;
                out++;
            }
            if (debug) 
            {
                printTime();
                Serial.print("Exited, people: ");
                Serial.println(people);
            }
            *currentState = IDLE;
            break;
        case IN:
            people++;
            in++;
            if (debug) 
            {
                printTime();
                Serial.print("Entered, people: ");
                Serial.println(people);
            }
            *currentState = IDLE;
            break;
    }
}

// Debugging
void plotSensor(char label, unsigned short* distances, unsigned short* bounds, state* currentState)
{
    Serial.print(">INSIDE_");
    Serial.print(label);
    Serial.print(":");
    Serial.println(distances[0]);
    Serial.print(">OUTSIDE_");
    Serial.print(label);
    Serial.print(":");
    Serial.println(distances[1]);

    Serial.print(">POT_INSIDE_");
    Serial.print(label);
    Serial.print(":");
    Serial.println(bounds[0]);
    Serial.print(">POT_OUTSIDE_");
    Serial.print(label);
    Serial.print(":");
    Serial.println(bounds[1]);

    Serial.print(">BOOL_INSIDE_");
    Serial.print(label);
    Serial.print(":");
    Serial.println(distances[0] > bounds[0]);
    Serial.print(">BOOL_OUTSIDE_");
    Serial.print(label);
    Serial.print(":");
    Serial.println(distances[1] > bounds[1]);

    Serial.print(">STATE_");
    Serial.print(label);
    Serial.print(":");
    Serial.println(*currentState);
}

// Tasks
void countPeople(void* parameter) 
{
    while (true) {
        unsigned short* distances1 = readPair(IN1, OUT1);
        unsigned short* bounds1 = readPair(POT_IN1, POT_OUT1);

        unsigned short* distances2 = readPair(IN2, OUT2);
        unsigned short* bounds2 = readPair(POT_IN2, POT_OUT2);

        currentState1 = setState(currentState1, distances1, bounds1);
        currentState2 = setState(currentState2, distances2, bounds2);

        if (debug) 
        {
            plotSensor('1', distances1, bounds1, &currentState1);
            plotSensor('2', distances2, bounds2, &currentState2);
            Serial.print(">PEOPLE:");
            Serial.println(people);
            Serial.print(">IN:");
            Serial.println(in);
            Serial.print(">OUT:");
            Serial.println(out);
        }
        
        setOccupancy(&currentState1);
        setOccupancy(&currentState2);

        free(distances1);
        free(bounds1);
        free(distances2);
        free(bounds2);
    }
}

void checkStation(void* parameter) 
{ 
    while (true) 
    {
        
    }
}

void checkSerial(void* parameter) 
{
    while (true) 
    {
        if (Serial.available() > 0) {
            char c = Serial.read();
            if (c == 'd') {
                debug = !debug;
                if (debug) 
                {
                    printTime();
                    Serial.println("Debug mode on");
                }
                else 
                {
                    printTime();
                    Serial.println("Debug mode off");
                }
            }
            else if (c == 'r') resetCounters();
        }
    }
}
