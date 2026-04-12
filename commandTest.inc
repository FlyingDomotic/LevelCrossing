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
        dccChanged();
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
