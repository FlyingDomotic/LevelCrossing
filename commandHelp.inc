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
    Serial.print(F(INERTIA_DEGREES_COUNT_COMMAND)); Serial.print(F("0-25 : Nombre de degrés inertie => ")); Serial.println(data.inertiaDegreesCount);
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
    Serial.print(F(DISPLAY_ILS_STATE_COMMAND)); Serial.print(F(" : État ILS")); Serial.println();
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
    Serial.print(F(INERTIA_DEGREES_COUNT_COMMAND)); Serial.print(F("0-25 : Inertia degrees count => ")); Serial.println(data.inertiaDegreesCount);
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
