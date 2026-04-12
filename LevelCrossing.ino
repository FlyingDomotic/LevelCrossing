#define CODE_VERSION "V26.4.12-1"

#define VERSION_FRANCAISE                                           // Messages in French (comment to have English Message)
#define ILS_CLOSED LOW                                              // State read when ILS is closed
#define LED_OFF LOW                                                 // State to send to turn LED off
#define LED_ON HIGH                                                 // State to send to turn LED off
#define DISPLAY_ILS_TIME 100                                        // Display ILS state every xxx ms
#define DISPLAY_KEYBOARD_INPUT                                      // Display each character read on keyboard (do not define if not needed)
#define MP3_PIN 13                                                  // MP3 sound module pin (comment it not used)
#define DCC_PIN 2                                                   // DCC decoder pin (comment it not used)

/* This code manages a level crossing for model railroad.

Level crossing has one barrier on each side, in front of car road side. It may also have  opposite side, which will close after car side, giving a delay.

Settings are send to Arduino through its serial (USB) link. This link is also used to send feedback and messages to user.

Settings are saved in Arduino's EEPROM to be available after (re)start.

# Principle:
    A pair of detectors (ILS, infrared, contact, hall effect detector...) is put each side of level crossing to manage. When a train is detected, we start blinking LEDs, sound ring if MP3 player installed, wait a bit before closing first set of barriers (those not on car road side), wait a bit before closing second set of barriers (in case of a 4 barriers level crossing), and wait for train leaving zone to reopen at the same time all barriers, stop blinking, stop sound if used.

    Train entering is done when entry detectors are activated for a given time (to avoid false detection).

    Train leaving is done when leaving detectors are not activated anymore for a given time.

    To be realistic, entering detector should be put something far of level crossing, to let time for level crossing to blink/sound/close barriers, while leaving detector should be close to level crossing, to quickly reopen road when train is gone.

    Usually, a track is one way only, but you may want to protect a level crossing in both directions. In this case, use a second pair of detectors in the opposite directions.

    Please note that track is considered to be free if a train is seen in reverse direction as reverse direction is only here to avoid closing barriers when loco hits enter ILS after leave.

    If you wish to protect reverse direction on a track, just put a couple of ILS in reverse direction

    You may protect up to 5 directions:
        - 5 one way tracks,
        - 1 two ways tracks and 3 one way track,
        - 2 two ways track, and one one way track

# Hardware Arduino Nano:
    - 2 servomotors:
        - to manage barriers on car side of the road (will close first)
    - 2 optional servo motors:
        - to manage barriers on opposite side of the road (will close after the car side if present)
    - 4 LEDS:
        - 2 on each side of level crossing, will blink alternatively
    - 1 IR detector/ILS/switch/hall effect sensor per track and traffic direction:
        - to detect train entering into protected zone
    - 1 IR detector/ILS/switch/hall effect sensor per track and traffic direction:
        - to detect train leaving into protected zone
    - 1 optional DY-SV17F sound module:
        - to ring when train is detected
    - 1 optional DCC decoder:
        - to remotely control level crossing and manual/automatic mode. Note that DCC only command is possible. In this case, you may supress detectors if train detection is done outside.

# Setting parameters:
    - define open and close angles for the 2 barriers on car side/full side
    - define delay to add after a degree rotation (set servo speed)
    - define delay to wait after detecting train before closing barriers
    - if 2 barriers on each road side (else let the 4 angles to zero):
        - define open and close angles for the 2 barriers on opposite side
        - define delay to wait after closing those on car side (ms)
    - define LEDs blink time (ms)
    - define time to wait with IR detection before considering it as used (ms)
    - define time to wait without IR detection before considering it as unused (ms)
    - if sound is used:
        - set volume
        - set sound index
    - if DCC decoder is used:
        - set manual mode address
        - set open/close address

# Direction state management:
    In normal direction: when detect train entering (on enter ILS), wait for first exit, then wait leaveTrainDelay after last exit (on leave ILS)
        - if enter ILS just closed and state is waitingForTrain -> set state to waitingForFirstExit and close barriers
        - if leave ILS just closed ans state is waitingForFirstExit -> set state to waitingForLastExit
        - if leave ILS is opened and state is waitingForLastExit -> arm exit timer
        - when exit timer expires -> set state to waitingForTrain, if all directions are in waitingForTrain or waitForLastReverseExit state then open barriers
    In  reverse direction: when detect train entering in reverse way (on leave ILS), wait leaveTrainDelay after last exit (on enter ILS)
        - if leave ILS just closed and state is waitingForTrain -> set state to waitForLastReverseExit
        - if enter ILS just opened and state is waitForLastReverseExit -> arm exit timer
        - same exit timer expiration as normal direction

// >>> commandComment.inc
# Commands:
    - B1CA0-180 : Barrier 1 close angle
    - B1OA0-180 : Barrier 1 open angle
    - B2CA0-180 : Barrier 2 close angle
    - B2OA0-180 : Barrier 2 open angle
    - B3CA0-180 : Barrier 3 close angle
    - B3OA0-180 : Barrier 3 open angle
    - B4CA0-180 : Barrier 4 close angle
    - B4OA0-180 : Barrier 4 open angle
    - WBC0-9999 : Wait before close (ms)
    - DFED0-99 : Delay for each degree (ms)
    - DBSC0-9999 : Delay before second close (ms)
    - LBT0-9999 : LED blink time (ms)
    - DT0-9999 : Debounce time (ms)
    - LTD0-9999 : Leave train delay (ms)
    - DCC0-1 : Use DCC
    - MADA0-2044 : Manual mode DCC address
    - OBDA0-2044 : Open barriers DCC address
    - CBDA0-2044 : Close barrier DCC address
    - CS0-99 : Closing sound
    - TS1-99 : Test sound
    - SV0-30 : Sound Volume
    - SI0-999 : Sound Increment (ms)
    - R : Run
    - S : Stop
    - IS : ILS State
    - O : Open barriers
    - C : Close barriers
    - D : Toggle Debug
    - INIT : Global Initialization
    - DV : Display Variables
// <<<

Author : Flying Domotic, April 2025, for FabLab
License: GNU GENERAL PUBLIC LICENSE - Version 3, 29 June 2007

*/

//      ******************
//      **** Includes ****
//      ******************

#include "Arduino.h"
#include "EEPROM.h"
#include "Servo.h"
#ifdef MP3_PIN
    #include "mp3_player_module_wire.h"                             // MP3 player (one wire connection)
#endif
#ifdef DCC_PIN
    #include "NmraDcc.h"                                            // DCC message decoder (https://github.com/mrrwa/NmraDcc.git)
#endif

//      **************************************
//      **** Constants, structs and enums ****
//      **************************************

// Command and text
// >>> commandCommands.inc
// Commands
#ifdef VERSION_FRANCAISE
    #define BARRIER1_CLOSE_ANGLE_COMMAND "AFB1"
    #define BARRIER1_OPEN_ANGLE_COMMAND "AOB1"
    #define BARRIER2_CLOSE_ANGLE_COMMAND "AFB2"
    #define BARRIER2_OPEN_ANGLE_COMMAND "AOB2"
    #define BARRIER3_CLOSE_ANGLE_COMMAND "AFB3"
    #define BARRIER3_OPEN_ANGLE_COMMAND "AOB3"
    #define BARRIER4_CLOSE_ANGLE_COMMAND "AFB4"
    #define BARRIER4_OPEN_ANGLE_COMMAND "AOB4"
    #define WAIT_BEFORE_CLOSE_COMMAND "AAF"
    #define DELAY_FOR_EACH_DEGREE_COMMAND "AACD"
    #define DELAY_BEFORE_SECOND_CLOSE_COMMAND "AASF"
    #define LED_BLINK_TIME_COMMAND "DCL"
    #define DEBOUNCE_TIME_COMMAND "DAR"
    #define LEAVE_TRAIN_DELAY_COMMAND "TST"
    #ifdef DCC_PIN
        #define USE_DCC_COMMAND "DCC"
        #define MANUAL_MODE_DCC_ADDRESS_COMMAND "ADMM"
        #define OPEN_BARRIERS_DCC_ADDRESS_COMMAND "ADOB"
        #define CLOSE_BARRIERS_DCC_ADDRESS_COMMAND "ADFB"
    #endif
    #ifdef MP3_PIN
        #define CLOSE_SOUND_COMMAND "SF"
        #define PLAY_SOUND_COMMAND "TS"
        #define SOUND_VOLUME_COMMAND "V"
        #define SOUND_INCREMENT_DURATION_COMMAND "IS"
    #endif
    #define START_PROCESS_COMMAND "M"
    #define STOP_PROCESS_COMMAND "A"
    #define DISPLAY_ILS_STATE_COMMAND "E"
    #define OPEN_BARRIERS_COMMAND "O"
    #define CLOSE_BARRIERS_COMMAND "F"
    #define TOGGLE_DEBUG_COMMAND "D"
    #define INIT_COMMAND "INIT"
    #define DISPLAY_VARIABLES_COMMAND "AV"
#else
    #define BARRIER1_CLOSE_ANGLE_COMMAND "B1CA"
    #define BARRIER1_OPEN_ANGLE_COMMAND "B1OA"
    #define BARRIER2_CLOSE_ANGLE_COMMAND "B2CA"
    #define BARRIER2_OPEN_ANGLE_COMMAND "B2OA"
    #define BARRIER3_CLOSE_ANGLE_COMMAND "B3CA"
    #define BARRIER3_OPEN_ANGLE_COMMAND "B3OA"
    #define BARRIER4_CLOSE_ANGLE_COMMAND "B4CA"
    #define BARRIER4_OPEN_ANGLE_COMMAND "B4OA"
    #define WAIT_BEFORE_CLOSE_COMMAND "WBC"
    #define DELAY_FOR_EACH_DEGREE_COMMAND "DFED"
    #define DELAY_BEFORE_SECOND_CLOSE_COMMAND "DBSC"
    #define LED_BLINK_TIME_COMMAND "LBT"
    #define DEBOUNCE_TIME_COMMAND "DT"
    #define LEAVE_TRAIN_DELAY_COMMAND "LTD"
    #ifdef DCC_PIN
        #define USE_DCC_COMMAND "DCC"
        #define MANUAL_MODE_DCC_ADDRESS_COMMAND "MADA"
        #define OPEN_BARRIERS_DCC_ADDRESS_COMMAND "OBDA"
        #define CLOSE_BARRIERS_DCC_ADDRESS_COMMAND "CBDA"
    #endif
    #ifdef MP3_PIN
        #define CLOSE_SOUND_COMMAND "CS"
        #define PLAY_SOUND_COMMAND "TS"
        #define SOUND_VOLUME_COMMAND "SV"
        #define SOUND_INCREMENT_DURATION_COMMAND "SI"
    #endif
    #define START_PROCESS_COMMAND "R"
    #define STOP_PROCESS_COMMAND "S"
    #define DISPLAY_ILS_STATE_COMMAND "IS"
    #define OPEN_BARRIERS_COMMAND "O"
    #define CLOSE_BARRIERS_COMMAND "C"
    #define TOGGLE_DEBUG_COMMAND "D"
    #define INIT_COMMAND "INIT"
    #define DISPLAY_VARIABLES_COMMAND "DV"
#endif
// <<<

//  EEPROM

#define MAGIC_NUMBER 59                                             // EEPROM magic byte
#define EEPROM_VERSION 1                                            // EEPROM version

// EEPROM data (current version)
struct eepromData_s {
    uint8_t magicNumber;                                            // Magic number
    uint8_t version;                                                // Structure version
    uint8_t barrier1CloseAngle;                                     // Barrier 1 servo closed angle
    uint8_t barrier1OpenAngle;                                      // Barrier 1 servo opened angle
    uint8_t barrier2CloseAngle;                                     // Barrier 2 servo closed angle
    uint8_t barrier2OpenAngle;                                      // Barrier 2 servo closed angle
    uint8_t barrier3CloseAngle;                                     // Barrier 3 servo closed angle
    uint8_t barrier3OpenAngle;                                      // Barrier 3 servo closed angle
    uint8_t barrier4CloseAngle;                                     // Barrier 4 servo closed angle
    uint8_t barrier4OpenAngle;                                      // Barrier 4 servo closed angle
    uint16_t waitBeforeClose;                                       // Delay before closing first barrier
    uint16_t delayBeforeSecondClose;                                // Delay after first before second barrier
    uint16_t ledBlinkTime;                                          // LED blinking delay
    uint16_t debounceTime;                                          // ILS debounce time
    uint16_t leaveTrainDelay;                                       // Delay after last leave pulse to consider train out
    uint16_t delayForEachDegree;                                    // Delay between each degree of servo rotation
    bool useDCC;                                                    // Use DCC decoder
    uint8_t manualModeDccAddress;                                   // Manual mode  DCC address (0 = unused)
    uint8_t openBarriersDccAddress;                                 // Open barriers DCC address (0 = unused)
    uint8_t closeBarriersDccAddress;                                // Close barriers DCC address (0 = unused)
    bool isModeAuto;                                                // Is auto mode active (else manual)?
    bool isActive;                                                  // When active flag is true, relays are triggered by ILS
    bool inDebug;                                                   // Print debug message when true
    uint8_t closeSound;                                             // Close sound index
    uint8_t soundVolume;                                            // Sound volume
    uint16_t soundIncrementDuration;                                // Sound increment
};

// ILS
#define ILS_COUNT 10                                                // Count of declared ILS
#define ILS_PINS {4, 5, 7, 8, A0, A1, A2, A3, A4, A5}               // PINs wshere ILS are connected

struct ilsData_s {                                                  // ILS data for one contact
    uint8_t pinNumber;                                              // Pin number to scan
    bool isEnterIls;                                                // True if enter ILS, leave else
    uint8_t directionIndex;                                         // Direction index
    bool isClosed;                                                  // ILS is currently closed
    bool lastWasClosed;                                             // ILS was closed at preivous loop
    unsigned long lastChangeTime;                                   // Last time pin state changed
};

// Servos
#define SERVO_COUNT 4                                               // Max count of servos
#define SERVO_PINS {3, 6, 9, 10}                                    // Pins where servos are connected

struct servoData_s {                                                // Data for one servo
    Servo servo;                                                    // Servo class
    uint8_t pinNumber;                                              // Map servos to Arduino PIN number
    uint8_t currentPosition;                                        // Current servo position
    uint8_t targetPosition;                                         // Servo target position
    uint8_t servoOpenAngle;                                         // Angle to open barrier
    uint8_t servoCloseAngle;                                        // Angle to close barrier
};

// LEDS
#define LED_COUNT 2                                                 // Count of LEDS

struct ledData_s {                                                  // Data for one LED
    uint8_t pinNumber;                                              // Map LED to Arduino PIN number
    bool isActive;                                                  // True if LEd active
};

// Direction state machine
#define DIRECTION_COUNT (ILS_COUNT / 2)                             // Count of directions

enum directionStateEnum {
    waitingForTrain = 0,                                            // Waiting for train entering
    waitingForFirstExit,                                            // Waiting for first element exit
    waitingForLastExit,                                             // Waiting for last element exit
    waitForLastReverseExit                                          // Wait for last exit in reverse way (exit ILS before enter ILS)
};

struct directionData_s {                                            // Data for one direction
    unsigned long exitTimer;                                        // Exit timer for this direction
    directionStateEnum state;                                       // State machine for direction
};

// Serial comands
#define BUFFER_LENGHT 50                                            // Serial input buffer length

//      **************
//      **** Data ****
//      **************

// Sound
#ifdef MP3_PIN
    Mp3PlayerModuleWire mp3Player(MP3_PIN);                         // MP3 module one-line protocol
    unsigned long lastSoundChangeTime = 0;                          // Last time we changed sound volume
    unsigned long soundChangeDuration = 0;                          // Duration of one sound change
    int8_t soundIncrement = 0;                                      // Sound increment (negative to decrement)
    uint8_t soundVolume = 0;                                        // Current sound volume
#endif

// ILS
ilsData_s ilsData[ILS_COUNT];                                       // ILS data table
bool displayIls = false;                                            // When set, continously display ILS state
unsigned long lastIlsDisplay = 0;                                   // Last time we displayed ILS state

// Direction
directionData_s directionData[DIRECTION_COUNT];                     // Direction data

// Servo
servoData_s servoData[SERVO_COUNT];                                 // Servo data
unsigned long lastServoTime = 0;                                    // Last time we updated servos
unsigned long barrierGroup1Timer = 0;                               // Timer before closing barrier group 1
unsigned long barrierGroup2Timer = 0;                               // Timer before closing barrier group 2

// LEDS
ledData_s ledData[LED_COUNT];                                       // LED data
unsigned long ledLastBlinkTime = 0;                                 // Last time leds changed (0 = leds inactive)

#ifdef DCC_PIN
    // DCC decoder
    NmraDcc dccInterface(512);                                      // DCC class, EEPROM starts @ 512
    bool dccActive = false;                                         // DCC active flag
#endif

// Serial commands
uint8_t bufferLen = 0;                                              // Used chars in input buffer
char inputBuffer[BUFFER_LENGHT];                                    // Serial input buffer
uint16_t commandValue;                                              // Value extracted from command

// EEPROM
eepromData_s data;                                                  // Data stored to/read from EEPROM

// Mode
bool isOpened = true;                                               // Is level crossing opened

//      *********************************************
//      **** Routines and functions  declaration ****
//      *********************************************

void printIlsName(uint8_t index);                                   // Print ILS name on screen
void printIlsState(void);                                           // Print ILS state on screen
void displayStatus(void);                                           // Display current status
void loadSettings(void);                                            // Load settings from EEPROM
void dispatchSettings(void);                                        // Dispatch settings into internal structures
void saveSettings(void);                                            // Save settings to EEPROM (only if modified)
void initSettings(void);                                            // Init settings to default values
void resetInputBuffer(void);                                        // Reset serial input buffer
void workWithSerial(void);                                          // Work with Serial input
bool isCommand(char* inputBuffer, char* commandToCheck);            // Check command without value
bool isCommandValue(char* inputBuffer, char* commandToCheck, uint16_t minValue, uint16_t maxValue); // Check command with value
#ifdef MP3_PIN
    void mp3Setup();                                                // Setup sound chip
    void mp3Loop(void);                                             // Loop for MP3
    void changeVolume();                                            // Change current volume
    void playSound(uint8_t index);                                  // Play sound index
    void stopPlayingSound(void);                                    // Stop playing sound
#endif
#ifdef DCC_PIN
    void dccSetup(void);                                            // DCC setup
    void dccLoop(void);                                             // DCC loop
    void notifyDccAccTurnoutOutput(uint16_t addr, uint8_t direction, uint8_t outputPower); // Called when a DCC Turnout Packet is received
    void traceDccTurnout(uint16_t addr, uint8_t direction, uint8_t outputPower); // Trace DCC message

#endif
void displayIlsState(void);                                         // Display all ILS state
void printHelp(void);                                               // Print help message
void toggleDebug(void);                                             // Toggle  debug flag
void reinitAll(void);                                               // Reinitialize all settings
void executeCommand(void);                                          // Execute command read on serial input (a-z and 0-9)
void displayVariables(void);                                        // Display all variables (debug)
void startLed(void);                                                // Start LEDs blinking
void stopLed(void);                                                 // Stop LEDs blinking
void startProcess(void);                                            // Start process
void stopProcess(void);                                             // Stop process
void openBarriers(void);                                            // Open barriers
void closeBarriers(void);                                           // Close barriers
void closeBarriersGroup1(void);                                     // Open barriers group #1
void closeBarriersGroup2(void);                                     // Open barriers group #2
void startSound(void);                                              // Start close sound
void stopSound(void);                                               // Stop sound
void setup(void);                                                   // Setup
void ilsSetup(void);                                                // ILS
void ledSetup(void);                                                // LEDS
void servoSetup(void);                                              // Servos
void loop(void);                                                    // Main loop
void ilsLoop(void);                                                 // ILS
void ledLoop(void);                                                 // LEDS
void servoLoop(void);                                               // Servos
void serialLoop(void);                                              // Serial command loop

//      *************************************
//      **** Routines and functions code ****
//      *************************************

// Display current status
void displayStatus(void) {
    #ifdef VERSION_FRANCAISE
        Serial.println(isOpened ? F("Ouvert") : F("Fermé"));
        if (data.inDebug) Serial.print(F(", déverminage"));
        if (data.isModeAuto) {
            Serial.println(data.isActive ? F(", en marche") : F(", à l'arrêt"));
        } else {
            Serial.println(F(", mode manuel"));
        }
    #else
    Serial.println(isOpened ? F("Opened") : F("Closed"));
    if (data.inDebug) Serial.print(F(", debug"));
        if (data.isModeAuto) {
            Serial.println(data.isActive ? F(", running") : F(", stopped"));
        } else {
            Serial.println(F(", manuel mode"));
        }
    #endif
}

// Load settings from EEPROM
void loadSettings(void) {
    initSettings();                                                 // Init data structure
    if (EEPROM.read(0) != MAGIC_NUMBER) {                           // Is first byte equal to magic number?
        #ifdef VERSION_FRANCAISE
            Serial.print(F("Magic est "));
        #else
            Serial.print(F("Magic is "));
        #endif
        Serial.print(EEPROM.read(0));
        #ifdef VERSION_FRANCAISE
            Serial.print(F(", pas "));
        #else
            Serial.print(F(", not "));
        #endif
        Serial.print(MAGIC_NUMBER);
        Serial.println(F("!"));
        return;
    }

    uint8_t version = EEPROM.read(1);                               // Get version
    if (version == 1) {
        EEPROM.get(0, data);                                        // Load EEPROM V1 structure
        dispatchSettings();                                         // Dispatch settings into internal structures
    } else {
        #ifdef VERSION_FRANCAISE
            Serial.print(F("Version est "));
        #else
            Serial.print(F("Version is "));
        #endif
        Serial.print(version);
        #ifdef VERSION_FRANCAISE
            Serial.print(F(", pas "));
        #else
            Serial.print(F(", not "));
        #endif
        Serial.print(EEPROM_VERSION);
        Serial.println(F("!"));
        return;
    }
}

// Dispatch settings into internal structures
void dispatchSettings(void){
    servoData[0].servoCloseAngle = data.barrier1CloseAngle;
    servoData[0].servoOpenAngle = data.barrier1OpenAngle;
    servoData[1].servoCloseAngle = data.barrier2CloseAngle;
    servoData[1].servoOpenAngle = data.barrier2OpenAngle;
    servoData[2].servoCloseAngle = data.barrier3CloseAngle;
    servoData[2].servoOpenAngle = data.barrier3OpenAngle;
    servoData[3].servoCloseAngle = data.barrier4CloseAngle;
    servoData[4].servoOpenAngle = data.barrier4OpenAngle;
}

// Save settings to EEPROM (only if modified)
void saveSettings(void) {
    data.magicNumber = MAGIC_NUMBER;                                // Force magic number
    data.version = EEPROM_VERSION;                                  // ... and version
    eepromData_s savedData;                                         // Current EEPROM data
    EEPROM.get(0, savedData);                                       // Get saved data
    if (memcmp(&data, &savedData, sizeof(data))) {                  // Compare full buffers
        EEPROM.put(0, data);                                        // Store structure
    }
}

// Init settings to default values
void initSettings(void) {
    data.isActive = false;                                          // When active flag is true, relays are triggered by ILS
    data.inDebug = false;                                           // Print debug message when true
    data.isModeAuto = false;                                        // Is auto mode active (else manual)?
    data.barrier1OpenAngle = 90;                                    // Barrier 1 servo opened angle (dispatchSettings() after change)
    data.barrier2CloseAngle = 90;                                   // Barrier 2 servo closed angle (dispatchSettings() after change)
    data.barrier2OpenAngle = 90;                                    // Barrier 2 servo closed angle (dispatchSettings() after change)
    data.barrier3CloseAngle = 90;                                   // Barrier 3 servo closed angle (dispatchSettings() after change)
    data.barrier3OpenAngle = 90;                                    // Barrier 3 servo closed angle (dispatchSettings() after change)
    data.barrier4CloseAngle = 90;                                   // Barrier 4 servo closed angle (dispatchSettings() after change)
    data.barrier4OpenAngle = 90;                                    // Barrier 4 servo closed angle (dispatchSettings() after change)
    data.waitBeforeClose = 1000;                                    // Delay before closing first barrier
    data.delayBeforeSecondClose = 1000;                             // Delay after first before second barrier
    data.ledBlinkTime = 500;                                        // LED blinking delay
    data.debounceTime = 50;                                         // ILS debounce time
    data.leaveTrainDelay = 1000;                                    // Delay after last leave pulse to consider train out
    data.useDCC = false;                                            // Use DCC decoder
    data.manualModeDccAddress = 0;                                  // Manual mode  DCC address (0 = unused)
    data.openBarriersDccAddress = 0;                                // Open barriers DCC address (0 = unused)
    data.closeBarriersDccAddress = 0;                               // Close barriers DCC address (0 = unused)
    data.delayForEachDegree = 0;                                    // Delay between each degree of servo rotation
    data.closeSound = 1;                                            // Close sound index
    data.soundVolume = 15;                                          // Sound volume
    data.soundIncrementDuration = 0;                                // Sound increment
}

// Reset serial input buffer
void resetInputBuffer(void) {
    memset(inputBuffer, 0, sizeof(inputBuffer));
    bufferLen = 0;
}

// Work with Serial input
void workWithSerial(void) {
    // Reset display ILS flag
    if (displayIls) {
        Serial.println();
        displayIls = false;
    }
    // Read serial input
    while (Serial.available()) {
        // Read one character
        char c = Serial.read();
        if (c == 13) {                                              // Is this a return?
            Serial.print(c);
            executeCommand();
            resetInputBuffer();
        } else if (c) {                                             // Is this not null?
            // Keep only "A" to "Z", "a" to "z" and "0" to "9"
            if ((c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9')) {
                if (bufferLen >= BUFFER_LENGHT - 1) {
                    #ifdef VERSION_FRANCAISE
                        Serial.println(F("Buffer plein - Reset!"));
                    #else
                        Serial.println(F("Buffer full - Reset!"));
                    #endif
                    resetInputBuffer();
                }
                inputBuffer[bufferLen++] = c;
            } else if (c == 8) {                                    // Is this <backspace> character?
                if (bufferLen) {                                    // Is buffer not empty?
                    bufferLen--;                                    // Reduce buffer len
                    inputBuffer[bufferLen] = 0;                     // Remove charcater
                }
            }
        }
    }
    #ifdef DISPLAY_KEYBOARD_INPUT
        Serial.print(F("\r"));
        Serial.print(inputBuffer);
        Serial.print(F("  "));
        Serial.print(F("\r"));
        Serial.print(inputBuffer);
    #endif
}

// Check command without value
bool isCommand(char* inputBuffer, char* commandToCheck) {
    return !strcasecmp(inputBuffer, commandToCheck);
}

// Check command with value
bool isCommandValue(char* inputBuffer, char* commandToCheck, uint16_t minValue, uint16_t maxValue) {
    if (strncmp(inputBuffer, commandToCheck, strlen(commandToCheck))) {
        return false;
    }
    commandValue = atoi(&inputBuffer[strlen(commandToCheck)]);      // Convert given value
    if (commandValue < minValue || commandValue > maxValue) {       // Check limits
        Serial.print(commandValue);
        #ifdef VERSION_FRANCAISE
            Serial.print(F(" hors limites "));
        #else
            Serial.print(F(" out of limits "));
        #endif
        Serial.print(minValue);
        Serial.print(F("-"));
        Serial.print(maxValue);
        #ifdef VERSION_FRANCAISE
            Serial.print(F(" pour la commande "));
        #else
            Serial.print(F(" for command "));
        #endif
        Serial.println(commandToCheck);
        return false;
    }
    return true;
}

// Display all ILS state
void displayIlsState(void) {
    displayIls = true;
}

// Print help message
void printHelp(void) {
    // >>> commandHelp.inc
    // Help message
    #ifdef VERSION_FRANCAISE
        Serial.print(F(BARRIER1_CLOSE_ANGLE_COMMAND)); Serial.print(F("0-180 : Angle fermeture barrière 1 (°) => ")); Serial.println(data.barrier1CloseAngle);
        Serial.print(F(BARRIER1_OPEN_ANGLE_COMMAND)); Serial.print(F("0-180 : Angle ouverture barrière 1 (°) => ")); Serial.println(data.barrier1OpenAngle);
        Serial.print(F(BARRIER2_CLOSE_ANGLE_COMMAND)); Serial.print(F("0-180 : Angle fermeture barrière 2 (°) => ")); Serial.println(data.barrier2CloseAngle);
        Serial.print(F(BARRIER2_OPEN_ANGLE_COMMAND)); Serial.print(F("0-180 : Angle ouverture barrière 2 (°) => ")); Serial.println(data.barrier2OpenAngle);
        Serial.print(F(BARRIER3_CLOSE_ANGLE_COMMAND)); Serial.print(F("0-180 : Angle fermeture barrière 3 (°) => ")); Serial.println(data.barrier3CloseAngle);
        Serial.print(F(BARRIER3_OPEN_ANGLE_COMMAND)); Serial.print(F("0-180 : Angle ouverture barrière 3 (°) => ")); Serial.println(data.barrier3OpenAngle);
        Serial.print(F(BARRIER4_CLOSE_ANGLE_COMMAND)); Serial.print(F("0-180 : Angle fermeture barrière 4 (°) => ")); Serial.println(data.barrier4CloseAngle);
        Serial.print(F(BARRIER4_OPEN_ANGLE_COMMAND)); Serial.print(F("0-180 : Angle ouverture barrière 4 (°) => ")); Serial.println(data.barrier4OpenAngle);
        Serial.print(F(WAIT_BEFORE_CLOSE_COMMAND)); Serial.print(F("0-9999 : Attente avant fermeture (ms) => ")); Serial.println(data.waitBeforeClose);
        Serial.print(F(DELAY_FOR_EACH_DEGREE_COMMAND)); Serial.print(F("0-99 : Attente après chaque degré (ms) => ")); Serial.println(data.delayForEachDegree);
        Serial.print(F(DELAY_BEFORE_SECOND_CLOSE_COMMAND)); Serial.print(F("0-9999 : Attente avant seconde fermeture (ms) => ")); Serial.println(data.delayBeforeSecondClose);
        Serial.print(F(LED_BLINK_TIME_COMMAND)); Serial.print(F("0-9999 : Durée clignotement LED (ms) => ")); Serial.println(data.ledBlinkTime);
        Serial.print(F(DEBOUNCE_TIME_COMMAND)); Serial.print(F("0-9999 : Délai anti-rebond (ms) => ")); Serial.println(data.debounceTime);
        Serial.print(F(LEAVE_TRAIN_DELAY_COMMAND)); Serial.print(F("0-9999 : Temps sortie train (ms) => ")); Serial.println(data.leaveTrainDelay);
        #ifdef DCC_PIN
            Serial.print(F(USE_DCC_COMMAND)); Serial.print(F("0-1 : Utiliser DCC => ")); Serial.println(data.useDCC);
            Serial.print(F(MANUAL_MODE_DCC_ADDRESS_COMMAND)); Serial.print(F("0-2044 : Adresse DCC mode manuel => ")); Serial.println(data.manualModeDccAddress);
            Serial.print(F(OPEN_BARRIERS_DCC_ADDRESS_COMMAND)); Serial.print(F("0-2044 : Adresse DCC ouverture barrières => ")); Serial.println(data.openBarriersDccAddress);
            Serial.print(F(CLOSE_BARRIERS_DCC_ADDRESS_COMMAND)); Serial.print(F("0-2044 : Adresse DCC fermeture barrières => ")); Serial.println(data.closeBarriersDccAddress);
        #endif
        #ifdef MP3_PIN
            Serial.print(F(CLOSE_SOUND_COMMAND)); Serial.print(F("0-99 : Son fermeture => ")); Serial.println(data.closeSound);
            Serial.print(F(PLAY_SOUND_COMMAND)); Serial.print(F("1-99 : Test son")); Serial.println();
            Serial.print(F(SOUND_VOLUME_COMMAND)); Serial.print(F("0-30 : Volume son => ")); Serial.println(data.soundVolume);
            Serial.print(F(SOUND_INCREMENT_DURATION_COMMAND)); Serial.print(F("0-999 : Incrément son (ms) => ")); Serial.println(data.soundIncrementDuration);
        #endif
        Serial.print(F(START_PROCESS_COMMAND)); Serial.print(F(" : Marche")); Serial.println();
        Serial.print(F(STOP_PROCESS_COMMAND)); Serial.print(F(" : Arrêt")); Serial.println();
        Serial.print(F(DISPLAY_ILS_STATE_COMMAND)); Serial.print(F(" : Etat ILS")); Serial.println();
        Serial.print(F(OPEN_BARRIERS_COMMAND)); Serial.print(F(" : Ouverture barrières")); Serial.println();
        Serial.print(F(CLOSE_BARRIERS_COMMAND)); Serial.print(F(" : Fermeture barrières")); Serial.println();
        Serial.print(F(TOGGLE_DEBUG_COMMAND)); Serial.print(F(" : Bascule déverminage")); Serial.println();
        Serial.print(F(INIT_COMMAND)); Serial.print(F(" : Initialisation globale")); Serial.println();
        Serial.print(F(DISPLAY_VARIABLES_COMMAND)); Serial.print(F(" : Afficher variables")); Serial.println();
    #else
        Serial.print(F(BARRIER1_CLOSE_ANGLE_COMMAND)); Serial.print(F("0-180 : Barrier 1 close angle => ")); Serial.println(data.barrier1CloseAngle);
        Serial.print(F(BARRIER1_OPEN_ANGLE_COMMAND)); Serial.print(F("0-180 : Barrier 1 open angle => ")); Serial.println(data.barrier1OpenAngle);
        Serial.print(F(BARRIER2_CLOSE_ANGLE_COMMAND)); Serial.print(F("0-180 : Barrier 2 close angle => ")); Serial.println(data.barrier2CloseAngle);
        Serial.print(F(BARRIER2_OPEN_ANGLE_COMMAND)); Serial.print(F("0-180 : Barrier 2 open angle => ")); Serial.println(data.barrier2OpenAngle);
        Serial.print(F(BARRIER3_CLOSE_ANGLE_COMMAND)); Serial.print(F("0-180 : Barrier 3 close angle => ")); Serial.println(data.barrier3CloseAngle);
        Serial.print(F(BARRIER3_OPEN_ANGLE_COMMAND)); Serial.print(F("0-180 : Barrier 3 open angle => ")); Serial.println(data.barrier3OpenAngle);
        Serial.print(F(BARRIER4_CLOSE_ANGLE_COMMAND)); Serial.print(F("0-180 : Barrier 4 close angle => ")); Serial.println(data.barrier4CloseAngle);
        Serial.print(F(BARRIER4_OPEN_ANGLE_COMMAND)); Serial.print(F("0-180 : Barrier 4 open angle => ")); Serial.println(data.barrier4OpenAngle);
        Serial.print(F(WAIT_BEFORE_CLOSE_COMMAND)); Serial.print(F("0-9999 : Wait before close (ms) => ")); Serial.println(data.waitBeforeClose);
        Serial.print(F(DELAY_FOR_EACH_DEGREE_COMMAND)); Serial.print(F("0-99 : Delay for each degree (ms) => ")); Serial.println(data.delayForEachDegree);
        Serial.print(F(DELAY_BEFORE_SECOND_CLOSE_COMMAND)); Serial.print(F("0-9999 : Delay before second close (ms) => ")); Serial.println(data.delayBeforeSecondClose);
        Serial.print(F(LED_BLINK_TIME_COMMAND)); Serial.print(F("0-9999 : LED blink time (ms) => ")); Serial.println(data.ledBlinkTime);
        Serial.print(F(DEBOUNCE_TIME_COMMAND)); Serial.print(F("0-9999 : Debounce time (ms) => ")); Serial.println(data.debounceTime);
        Serial.print(F(LEAVE_TRAIN_DELAY_COMMAND)); Serial.print(F("0-9999 : Leave train delay (ms) => ")); Serial.println(data.leaveTrainDelay);
        #ifdef DCC_PIN
            Serial.print(F(USE_DCC_COMMAND)); Serial.print(F("0-1 : Use DCC => ")); Serial.println(data.useDCC);
            Serial.print(F(MANUAL_MODE_DCC_ADDRESS_COMMAND)); Serial.print(F("0-2044 : Manual mode DCC address => ")); Serial.println(data.manualModeDccAddress);
            Serial.print(F(OPEN_BARRIERS_DCC_ADDRESS_COMMAND)); Serial.print(F("0-2044 : Open barriers DCC address => ")); Serial.println(data.openBarriersDccAddress);
            Serial.print(F(CLOSE_BARRIERS_DCC_ADDRESS_COMMAND)); Serial.print(F("0-2044 : Close barrier DCC address => ")); Serial.println(data.closeBarriersDccAddress);
        #endif
        #ifdef MP3_PIN
            Serial.print(F(CLOSE_SOUND_COMMAND)); Serial.print(F("0-99 : Closing sound => ")); Serial.println(data.closeSound);
            Serial.print(F(PLAY_SOUND_COMMAND)); Serial.print(F("1-99 : Test sound")); Serial.println();
            Serial.print(F(SOUND_VOLUME_COMMAND)); Serial.print(F("0-30 : Sound Volume => ")); Serial.println(data.soundVolume);
            Serial.print(F(SOUND_INCREMENT_DURATION_COMMAND)); Serial.print(F("0-999 : Sound Increment (ms) => ")); Serial.println(data.soundIncrementDuration);
        #endif
        Serial.print(F(START_PROCESS_COMMAND)); Serial.print(F(" : Run")); Serial.println();
        Serial.print(F(STOP_PROCESS_COMMAND)); Serial.print(F(" : Stop")); Serial.println();
        Serial.print(F(DISPLAY_ILS_STATE_COMMAND)); Serial.print(F(" : ILS State")); Serial.println();
        Serial.print(F(OPEN_BARRIERS_COMMAND)); Serial.print(F(" : Open barriers")); Serial.println();
        Serial.print(F(CLOSE_BARRIERS_COMMAND)); Serial.print(F(" : Close barriers")); Serial.println();
        Serial.print(F(TOGGLE_DEBUG_COMMAND)); Serial.print(F(" : Toggle Debug")); Serial.println();
        Serial.print(F(INIT_COMMAND)); Serial.print(F(" : Global Initialization")); Serial.println();
        Serial.print(F(DISPLAY_VARIABLES_COMMAND)); Serial.print(F(" : Display Variables")); Serial.println();
    #endif
    // <<<
}

// Toggle  debug flag
void toggleDebug(void){
    data.inDebug = !data.inDebug;
    saveSettings();
}

// Start process
void startProcess(void) {
    data.isActive = true;
    saveSettings();
}

// Stop process
void stopProcess(void) {
    data.isActive = false;
    saveSettings();
}

// Open barrier
void openBarriers(void) {
    isOpened = true;                                                // Set flag to open
    for (uint8_t i=0; i<SERVO_COUNT; i++) {
        servoData[i].targetPosition = servoData[i].servoOpenAngle;  // Open all barriers
    }
    barrierGroup1Timer = 0;                                         // Clear barrier group timers
    barrierGroup2Timer = 0;
    stopSound();
    stopLed();
}

// Close barriers
void closeBarriers() {
    isOpened = false;                                               // Set flag to close
    startSound();
    startLed();
    barrierGroup1Timer = millis();
}

// Close barriers group #1
void closeBarriersGroup1(void){
    // Set servo target position for servo 1 & 2
    servoData[0].targetPosition = servoData[0].servoCloseAngle;
    servoData[1].targetPosition = servoData[1].servoCloseAngle;
    barrierGroup1Timer = 0;                                         // Clear group 1 timer
    barrierGroup2Timer = millis();                                  // Set group 2 timer
}

// Close barriers group #2
void closeBarriersGroup2(void){
    // Set servo target position for servo 3 & 4
    servoData[2].targetPosition = servoData[2].servoCloseAngle;
    servoData[3].targetPosition = servoData[3].servoCloseAngle;
    barrierGroup2Timer = 0;                                         // Clear group 2 timer
}

// Start close sound
void startSound(void) {
    #ifdef MP3_PIN
        playSound(data.closeSound);
    #endif
}

// Stop close sound
void stopSound(void) {
    #ifdef MP3_PIN
        stopPlayingSound();
    #endif
}

// Reinitialize all settings
void reinitAll(void) {
    initSettings();
    saveSettings();
}

// Execute command read on serial input (a-z and 0-9)
void executeCommand(void) {
    #ifndef DISPLAY_KEYBOARD_INPUT
        Serial.println(F(""));
        #ifdef VERSION_FRANCAISE
            Serial.print(F("Reçu : "));
        #else
            Serial.print(F("Received: "));
        #endif
    #endif
    Serial.println(inputBuffer);
    // >>> commandTest.inc
    if (isCommandValue(inputBuffer, (char*) BARRIER1_CLOSE_ANGLE_COMMAND, 0, 180)) {
        data.barrier1CloseAngle = commandValue;
        saveSettings();
        dispatchSettings();
    } else if (isCommandValue(inputBuffer, (char*) BARRIER1_OPEN_ANGLE_COMMAND, 0, 180)) {
        data.barrier1OpenAngle = commandValue;
        saveSettings();
        dispatchSettings();
    } else if (isCommandValue(inputBuffer, (char*) BARRIER2_CLOSE_ANGLE_COMMAND, 0, 180)) {
        data.barrier2CloseAngle = commandValue;
        saveSettings();
        dispatchSettings();
    } else if (isCommandValue(inputBuffer, (char*) BARRIER2_OPEN_ANGLE_COMMAND, 0, 180)) {
        data.barrier2OpenAngle = commandValue;
        saveSettings();
        dispatchSettings();
    } else if (isCommandValue(inputBuffer, (char*) BARRIER3_CLOSE_ANGLE_COMMAND, 0, 180)) {
        data.barrier3CloseAngle = commandValue;
        saveSettings();
        dispatchSettings();
    } else if (isCommandValue(inputBuffer, (char*) BARRIER3_OPEN_ANGLE_COMMAND, 0, 180)) {
        data.barrier3OpenAngle = commandValue;
        saveSettings();
        dispatchSettings();
    } else if (isCommandValue(inputBuffer, (char*) BARRIER4_CLOSE_ANGLE_COMMAND, 0, 180)) {
        data.barrier4CloseAngle = commandValue;
        saveSettings();
        dispatchSettings();
    } else if (isCommandValue(inputBuffer, (char*) BARRIER4_OPEN_ANGLE_COMMAND, 0, 180)) {
        data.barrier4OpenAngle = commandValue;
        saveSettings();
        dispatchSettings();
    } else if (isCommandValue(inputBuffer, (char*) WAIT_BEFORE_CLOSE_COMMAND, 0, 9999)) {
        data.waitBeforeClose = commandValue;
        saveSettings();
    } else if (isCommandValue(inputBuffer, (char*) DELAY_FOR_EACH_DEGREE_COMMAND, 0, 99)) {
        data.delayForEachDegree = commandValue;
        saveSettings();
    } else if (isCommandValue(inputBuffer, (char*) DELAY_BEFORE_SECOND_CLOSE_COMMAND, 0, 9999)) {
        data.delayBeforeSecondClose = commandValue;
        saveSettings();
    } else if (isCommandValue(inputBuffer, (char*) LED_BLINK_TIME_COMMAND, 0, 9999)) {
        data.ledBlinkTime = commandValue;
        saveSettings();
    } else if (isCommandValue(inputBuffer, (char*) DEBOUNCE_TIME_COMMAND, 0, 9999)) {
        data.debounceTime = commandValue;
        saveSettings();
    } else if (isCommandValue(inputBuffer, (char*) LEAVE_TRAIN_DELAY_COMMAND, 0, 9999)) {
        data.leaveTrainDelay = commandValue;
        saveSettings();
    #ifdef DCC_PIN
        } else if (isCommandValue(inputBuffer, (char*) USE_DCC_COMMAND, 0, 1)) {
            data.useDCC = commandValue;
            saveSettings();
            dccSetup();
        } else if (isCommandValue(inputBuffer, (char*) MANUAL_MODE_DCC_ADDRESS_COMMAND, 0, 2044)) {
            data.manualModeDccAddress = commandValue;
            saveSettings();
        } else if (isCommandValue(inputBuffer, (char*) OPEN_BARRIERS_DCC_ADDRESS_COMMAND, 0, 2044)) {
            data.openBarriersDccAddress = commandValue;
            saveSettings();
        } else if (isCommandValue(inputBuffer, (char*) CLOSE_BARRIERS_DCC_ADDRESS_COMMAND, 0, 2044)) {
            data.closeBarriersDccAddress = commandValue;
            saveSettings();
    #endif
    #ifdef MP3_PIN
        } else if (isCommandValue(inputBuffer, (char*) CLOSE_SOUND_COMMAND, 0, 99)) {
            data.closeSound = commandValue;
            saveSettings();
        } else if (isCommandValue(inputBuffer, (char*) PLAY_SOUND_COMMAND, 1, 99)) {
            playSound(commandValue);
        } else if (isCommandValue(inputBuffer, (char*) SOUND_VOLUME_COMMAND, 0, 30)) {
            data.soundVolume = commandValue;
            saveSettings();
        } else if (isCommandValue(inputBuffer, (char*) SOUND_INCREMENT_DURATION_COMMAND, 0, 999)) {
            data.soundIncrementDuration = commandValue;
            saveSettings();
    #endif
    } else if (isCommand(inputBuffer, (char*) START_PROCESS_COMMAND)) {
        startProcess();
    } else if (isCommand(inputBuffer, (char*) STOP_PROCESS_COMMAND)) {
        stopProcess();
    } else if (isCommand(inputBuffer, (char*) DISPLAY_ILS_STATE_COMMAND)) {
        displayIlsState();
    } else if (isCommand(inputBuffer, (char*) OPEN_BARRIERS_COMMAND)) {
        openBarriers();
    } else if (isCommand(inputBuffer, (char*) CLOSE_BARRIERS_COMMAND)) {
        closeBarriers();
    } else if (isCommand(inputBuffer, (char*) TOGGLE_DEBUG_COMMAND)) {
        toggleDebug();
    } else if (isCommand(inputBuffer, (char*) INIT_COMMAND)) {
        init();
    } else if (isCommand(inputBuffer, (char*) DISPLAY_VARIABLES_COMMAND)) {
        displayVariables();
    // <<<
    } else {
        if (inputBuffer[0]) {
            printHelp();
        }
    }
    displayStatus();
}

// Display all variables (debug)
void displayVariables(void) {
    #ifdef VERSION_FRANCAISE
        Serial.println(F("ILS: EEEESSSS"));
    #else
        Serial.println(F("ILS: EEEELLLL"));
    #endif
    Serial.println(F("ILS: 12341234"));
    Serial.print(F(  "     "));
    printIlsState();
    Serial.println();
    Serial.print("Directions:");
    for (uint8_t i=0; i<DIRECTION_COUNT; i++) {
        Serial.print(F(" "));
        Serial.print(directionData[i].state);
    }
    Serial.println();
    Serial.print(F("Servos:"));
    for (uint8_t i=0; i<SERVO_COUNT; i++) {
        Serial.print(F(" "));
        Serial.print(servoData[i].currentPosition);
        Serial.print(F("/"));
        Serial.print(servoData[i].targetPosition);
    }
}

// Print ILS name
void printIlsName(uint8_t index){
    #ifdef VERSION_FRANCAISE
        Serial.print(ilsData[index].isEnterIls ? F("E") : F("S"));
    #else
        Serial.print(ilsData[index].isEnterIls ? F("E") : F("L"));
    #endif
    Serial.print(ilsData[index].directionIndex);
}

// Print ILS state
void printIlsState(void ){
    for (uint8_t i=0; i<ILS_COUNT; i++) {
        if (ilsData[i].isClosed) {
            Serial.print(F("X "));
        } else {
            Serial.print(F("- "));
        }
    }
}

// ILS setup
void ilsSetup(void){
    uint8_t ilsPins[] = ILS_PINS;                                   // Load ILS pins
    for (uint8_t i=0; i<ILS_COUNT; i++) {                           // For all ILS
        ilsData[i].pinNumber = ilsPins[i];                          // Load pin index
        ilsData[i].lastChangeTime = millis();                       // Save change time
        if (ilsData[i].pinNumber) {                                 // If pin index defined, read current value and save it
            pinMode(ilsData[i].pinNumber, INPUT);                   // Set mode to INPUT
            ilsData[i].isClosed = digitalRead(ilsData[i].pinNumber) == ILS_CLOSED; // Set current ILS state
            ilsData[i].lastWasClosed = ilsData[i].isClosed;         // Set previous ILS state
        }
        ilsData[i].isEnterIls = (i & 1) == 0;                       // Is enter ILS is odd number
        ilsData[i].directionIndex = (i >> 1);                       // Direction index is i/2
    }
    for (uint8_t i=0; i<DIRECTION_COUNT; i++) {                     // For all directions
        directionData[i].state = waitingForTrain;                   // Waiting for train
        directionData[i].exitTimer = 0;                             // Clear exit timer
    }
}

// ILS loop
void ilsLoop(void){
    unsigned long now = millis();

    // Scan all ILS
    for (uint8_t i=0; i<ILS_COUNT; i++) {
        // Check for pinNumber defined
        if (ilsData[i].pinNumber) {
            // Read current state
            bool currentState = (digitalRead(ilsData[i].pinNumber) == ILS_CLOSED);
            if (ilsData[i].lastWasClosed != currentState) {         // Does state change since last call?
                ilsData[i].lastWasClosed = currentState;            // Save state
                ilsData[i].lastChangeTime = now;                    // Save last change time
            }
            if (ilsData[i].isClosed != ilsData[i].lastWasClosed) {  //State changed?
                if ((now - ilsData[i].lastChangeTime) > data.debounceTime && data.isActive) { // Is pin stable for debounce time and mode active
                    ilsData[i].isClosed = ilsData[i].lastWasClosed; // Load state with last stable one
                    if (data.inDebug) {
                        #ifdef VERSION_FRANCAISE
                            Serial.print(ilsData[i].isClosed?F("Fermeture"):F("Ouverture"));
                        #else
                            Serial.print(ilsData[i].isClosed?F("Closing"):F("Opening"));
                        #endif
                        Serial.print(F(" ILS "));
                        printIlsName(i);
                        #ifdef VERSION_FRANCAISE
                            Serial.print(" à ");
                        #else
                            Serial.print(" at ");
                        #endif
                        Serial.println(millis());
                    }
                }
                bool isEnterIls = ilsData[i].isEnterIls;
                uint8_t directionIndex = ilsData[i].directionIndex;
                // if enter ILS just closed and state is waitingForTrain -> set state to waitingForFirstExit and close barriers
                if (isEnterIls && ilsData[i].isClosed
                        && directionData[directionIndex].state == waitingForTrain) {
                    directionData[directionIndex].state = waitingForFirstExit;
                    // Are we in auto mode?
                    if (data.isModeAuto){
                        closeBarriers();
                    }
                }
                // if leave ILS just closed ans state is waitingForFirstExit -> set state to waitingForLastExit
                else if (!isEnterIls && ilsData[i].isClosed
                        && directionData[directionIndex].state == waitingForFirstExit) {
                    directionData[directionIndex].state = waitingForLastExit;
                }
                // if leave ILS is opened and state is waitingForLastExit -> arm exit timer
                else if (!isEnterIls && !ilsData[i].isClosed
                        && directionData[directionIndex].state == waitingForLastExit) {
                    directionData[directionIndex].exitTimer = millis();
                }
                // if leave ILS just closed and state is waitingForTrain -> set state to waitForLastReverseExit
                else if (!isEnterIls && ilsData[i].isClosed
                        && directionData[directionIndex].state == waitingForTrain) {
                    directionData[directionIndex].state = waitForLastReverseExit;
                }
                // if enter ILS just opened and state is waitForLastReverseExit -> arm exit timer
                else if (isEnterIls && !ilsData[i].isClosed
                        && directionData[directionIndex].state == waitForLastReverseExit) {
                    directionData[directionIndex].exitTimer = millis();
                }
            }
        }
    }

    // Scan all exit timers for expiration
    bool oneTimerExpired = false;                                   // At least one timer expired flag

    for (uint8_t i=0; i<DIRECTION_COUNT; i++) {
        if (directionData[i].exitTimer && ((millis() - directionData[i].exitTimer) > data.leaveTrainDelay)) {
            oneTimerExpired = true;
            directionData[i].exitTimer = 0;                         // Clear exit timer
        }
    }

    // Do we expire at east one exit timer?
    if (oneTimerExpired) {
        bool oneTrainSeen = false;
        // Check for all direction state for train
        for (uint8_t i=0; i<DIRECTION_COUNT; i++) {
            // Note that track is considered to be free if a train is seen in reverse direction
            //  as reverse direction is only here to avoid closing barriers when loco hits enter ILS after leave
            // If you wish to protect reverse direction on a track, just put a couple of ILS in reverse direction
            if (directionData[i].state != waitingForTrain && directionData[i].state != waitForLastReverseExit) {
                oneTrainSeen = true;
            }
        }
        // No train seen?
        if (!oneTrainSeen) {
            // Are we in auto mode?
            if (data.isModeAuto){
                openBarriers();                                     // Open barriers, last train in normal direction seen
            }
        }
    }

    // Display ILS state if needed
    if (displayIls) {
        if ((millis() - lastIlsDisplay) > DISPLAY_ILS_TIME) {
            Serial.print (F("\rILS : "));
            printIlsState();
            lastIlsDisplay = millis();
        }
    }
}

// LEDS setup
void ledSetup(void){
    for (uint8_t i=0; i<LED_COUNT; i++){
        digitalWrite(ledData[i].pinNumber, ledData[i].isActive? LED_ON : LED_OFF);
        pinMode(ledData[i].pinNumber, OUTPUT);
    }
}

// LEDS loop
void ledLoop(void){
    // Are LEDS active and last blink > delay?
    if (ledLastBlinkTime && ((millis() - ledLastBlinkTime) > data.ledBlinkTime)) {
        // For all LEDS
        for (uint8_t i=0; i<LED_COUNT; i++){
            ledData[i].isActive = !ledData[i].isActive;             // Revert LED state
            digitalWrite(ledData[i].pinNumber, ledData[i].isActive? LED_ON : LED_OFF);  // Set LED accordingly
        }
        ledLastBlinkTime = millis();                                // Save last blink time
    }
}

// Start LEDs blinking
void startLed(void){
    // For all LEDS
    for (uint8_t i=0; i<LED_COUNT; i++){
        ledData[i].isActive = (i & 1);                              // Set LED initial state depending on odd/even index
        digitalWrite(ledData[i].pinNumber, ledData[i].isActive? LED_ON : LED_OFF);  // Set LED accordingly
        }
    ledLastBlinkTime = millis();                                    // Save last blink time
}

// Stop LEDs blinking
void stopLed(void){
    // For all LEDS
    for (uint8_t i=0; i<LED_COUNT; i++){
        ledData[i].isActive = false;                                // Set LED off
        digitalWrite(ledData[i].pinNumber, ledData[i].isActive? LED_ON : LED_OFF);  // Set LED accordingly
    }
    ledLastBlinkTime = 0;                                           // Deactivate LEDS
}

// Servos setup
void servoSetup(void){
    // For all servos
    for (uint8_t i=0; i<SERVO_COUNT; i++){
        servoData[i].servo.attach(servoData[i].pinNumber);          // Attach pin to servo
        servoData[i].targetPosition = servoData[i].servoOpenAngle;  // Open barrier
        servoData[i].currentPosition = servoData[i].servoOpenAngle; // Current barrier position
        servoData[i].servo.write(servoData[i].targetPosition);      // Move immediatly to open

    }
}

// Servos loop
void servoLoop(void){
    // Does delay since last servo update expire?
    if ((millis() - lastServoTime) > data.delayForEachDegree) {
        // For all servos, turn by +/- one degree to reach target position
        int8_t increment = 1;                                       // By default, turn right
        for (uint8_t i=0; i<SERVO_COUNT; i++) {
            if (servoData[i].currentPosition != servoData[i].targetPosition) {// Only if servo not at target position
                if (servoData[i].currentPosition > servoData[i].targetPosition) { // Servo should turn left
                    increment = -1;
                }
                servoData[i].currentPosition += increment;          // Update current position
                servoData[i].servo.write(servoData[i].targetPosition);           // Turn servo
            }
        }
        lastServoTime = millis();                                   // Update last servo change time
    }

    // Check timers
    if (barrierGroup1Timer && (millis() - barrierGroup1Timer) > data.waitBeforeClose) {
        closeBarriersGroup1();
    }
    if (barrierGroup2Timer && (millis() - barrierGroup2Timer) > data.delayBeforeSecondClose) {
        closeBarriersGroup2();
    }
}

// DCC interface

#ifdef DCC_PIN
    // Called when a DCC Turnout Packet is received
    void notifyDccAccTurnoutOutput(uint16_t addr, uint8_t direction, uint8_t outputPower) {
        if (data.manualModeDccAddress && data.manualModeDccAddress == addr) {
            traceDccTurnout(addr, direction, outputPower);
            data.manualModeDccAddress = (direction != 0);
        } 
        if (data.openBarriersDccAddress && data.openBarriersDccAddress == addr && direction == 1) {
            traceDccTurnout(addr, direction, outputPower);
            openBarriers();
        } 
        if (data.closeBarriersDccAddress && data.closeBarriersDccAddress == addr && direction == 1) {
            traceDccTurnout(addr, direction, outputPower);
            closeBarriers();           
        } 
    }

    // Trace DCC message
    void traceDccTurnout(uint16_t addr, uint8_t direction, uint8_t outputPower) {
        if (data.inDebug) {
            Serial.print(F("DCC Turnout addr "));
            Serial.print(addr) ;
            Serial.print(F(", direction "));
            Serial.print(direction) ;
            Serial.print(F(", power "));
            Serial.println(outputPower, HEX);

        }
    }
    // DCC setup
    void dccSetup(void) {
        // Is DCC already active?
        if (dccActive) {
            dccInterface.deInit(false);                             // Stop DCC
        }
        // Should we activate DCC?
        if (data.useDCC) {
            #ifdef digitalPinToInterrupt
                dccInterface.pin(DCC_PIN, 0);                       // DCC decoder pin
            #else
                dccInterface.pin(DCC_PIN, 1);                       // DCC decoder pin
            #endif
            dccInterface.init(MAN_ID_DIY, 10, CV29_ACCESSORY_DECODER | CV29_OUTPUT_ADDRESS_MODE, 0, false);
            dccActive = true;                                       // DCC is now active
        }
    }

    // DCC loop
    void dccLoop(void) {
        if (dccActive) {
            dccInterface.process();                                 // Run DCC loop
    
        }
    }
#endif

// Serial command loop
void SerialLoop(void) {
    // Scan serial for input
    if (Serial.available()) {
        workWithSerial();
    }
}
#ifdef MP3_PIN
    // Init sound chip
    void mp3Setup(void) {
        mp3Player.set_storage(mp3Player.STORAGE_FLASH);             // Use files in flash (should be downloaded before using USB)
        mp3Player.set_play_mode(mp3Player.PLAY_TRACK_REPEAT);       // Repeat track forever
    }

    // Play a sound
    void playSound(uint8_t index) {
        if (data.soundVolume && index) {                            // Only if volume and index set
            if (data.soundIncrementDuration && data.soundVolume > 1) {      // Sound increment and target volume > 1?
                soundVolume = 1;                                    // Start at volume 1
                soundIncrement = 1;                                 // Increase sound
                soundChangeDuration = data.soundIncrementDuration / data.soundVolume; // Wait time between increments
                lastSoundChangeTime = millis();                     // Set last volume change time

            } else {
                soundVolume = data.soundVolume;                     // Set directly target sound volume
                lastSoundChangeTime = 0;                            // Clear last volume set time
            }
            mp3Player.set_volume(soundVolume);                      // Set volume
            mp3Player.set_track_index(index);                       // Set track index to play
            mp3Player.play();                                       // Play track
        }
    }

    // Stop playing a sound
    void stopPlayingSound(void) {
        if (data.soundVolume) {                                     // Only if volume set
            if (data.soundIncrementDuration && data.soundVolume > 1) {      // Sound increment and target volume > 1?
                soundVolume -= 1;                                   // Start at volume - 1
                soundIncrement = -1;                                // Decrease sound
                soundChangeDuration = data.soundIncrementDuration / data.soundVolume; // Wait time between increments
                lastSoundChangeTime = millis();                     // Set last volume change time

            } else {
                soundVolume = 0;                                    // Set directly target sound volume
                lastSoundChangeTime = 0;                            // Clear last volume set time
            }
            mp3Player.set_volume(soundVolume);                      // Set volume
            if (!soundVolume) {                                     // Is volume set to zero?
                mp3Player.stop();                                   // Stop player
            }
        }

    }

    // Set sound volume giving increase/decrease
    void changeVolume() {
        soundVolume += soundIncrement;                              // Increment/decrement sound
        if (soundVolume < data.soundVolume && soundVolume > 0) {    // We're not yet at target sound
            lastSoundChangeTime = millis();                         // Set last volume change time
        } else {
            lastSoundChangeTime = 0;                                // Reset last volume change time
        }
        mp3Player.set_volume(soundVolume);                          // Set new volume
        if (!soundVolume) {                                         // Volume = 0?
            mp3Player.stop();                                       // Stop player
        }
    }

    void mp3Loop(void){
        // Are we in volume change with delay expired?
        if (lastSoundChangeTime && ((millis() - lastSoundChangeTime) >= soundChangeDuration)) {
            changeVolume();                                         // Increase/decrease volume
        }
    }
#endif

// Setup
void setup(void){
    Serial.begin(115200);
    Serial.println();
    #ifdef VERSION_FRANCAISE
        Serial.print(F("Passage à niveau "));
    #else
        Serial.print(F("Level crossing "));
    #endif
    Serial.print(CODE_VERSION);
    #ifdef VERSION_FRANCAISE
        Serial.println(F(" lancé ..."));
    #else
        Serial.println(F(" started..."));
    #endif
    resetInputBuffer();
    initSettings();
    loadSettings();
    ledSetup();
    servoSetup();
    ilsSetup();
    #ifdef MP3_PIN
        mp3Setup();
    #endif
    #ifdef DCC_PIN
        dccSetup();
    #endif
    displayStatus();
}

// Main loop
void loop(void){
    ilsLoop();
    ledLoop();
    servoLoop();

    #ifdef MP3_PIN
        mp3Loop();
    #endif

    #ifdef DCC_PIN
        dccSetup();
    #endif

    SerialLoop();
}