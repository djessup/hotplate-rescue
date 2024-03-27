//
// Created by DJ on 26/03/2024.
//

#include "settings.h"
#include "EEPROM.h"
#include <new>
#include <Arduino.h>


/**
 * Construct w/ default EEPROM address
 */
HeaterSettings::HeaterSettings() {
    new(this) HeaterSettings(SETTINGS_DEFAULT_EEPROM_ADDR);
}

/**
 * Construct w/ specific EEPROM offset address
 * @param eepromAddr offset to save settings to in EEPROM
 */
HeaterSettings::HeaterSettings(int eepromAddr) {
    this->eepromAddr = eepromAddr;
    this->settings = this->load();
}

unsigned int HeaterSettings::getPreheatTarget() {
    return this->settings.preheatTarget;
}

void HeaterSettings::setPreheatTarget(unsigned int preheatTarget) {
    this->settings.preheatTarget = preheatTarget;
    this->isDirty = true;
}

unsigned int HeaterSettings::getReflowTarget() {
    return this->settings.reflowTarget;
}

void HeaterSettings::setReflowTarget(unsigned int reflowTarget) {
    this->settings.reflowTarget = reflowTarget;
    this->isDirty = true;
}

unsigned int HeaterSettings::getPreheatDuration() {
    return this->settings.preheatDuration;
}

void HeaterSettings::setPreheatDuration(unsigned int preheatDuration) {
    this->settings.preheatDuration = preheatDuration;
    this->isDirty = true;
}

unsigned int HeaterSettings::getReflowDuration() {
    return this->settings.reflowDuration;
}

void HeaterSettings::setReflowDuration(unsigned int reflowDuration) {
    this->settings.reflowDuration = reflowDuration;
    this->isDirty = true;
}

settings_t HeaterSettings::load() {
    settings_t storedSettings;
    EEPROM.get(this->eepromAddr, storedSettings);
    storedSettings = this->alignToLimits(storedSettings);
    if (storedSettings) {
        return storedSettings;
    } else {
        storedSettings = settings_t(); // use defaults
        return storedSettings;
    }
}

bool HeaterSettings::save(settings_t s) {
    if (isDirty) {
        return false;
    }
    EEPROM.put(this->eepromAddr, alignToLimits(s));
    return true;
}

settings_t HeaterSettings::alignToLimits(settings_t s) {

    // check min/max are within limits
    if (s.maxTemp > SETTINGS_MAX_MAX_TEMP) {
        s.maxTemp = SETTINGS_MAX_MAX_TEMP;
    } else if (s.minTemp < 0) { // I know it's unsigned, but strange things can happen in EEPROM...
        s.minTemp = 0;
    }
    // check min isn't higher than max
    if (s.minTemp > s.maxTemp) {
        s.minTemp = s.maxTemp;
    }
    // check timeout is within limits
    if (s.heaterTimeout > SETTINGS_MAX_HEATER_TIMEOUT) {
        s.heaterTimeout = SETTINGS_MAX_HEATER_TIMEOUT;
    }
    // check targets are within min/max
    if (s.preheatTarget > s.maxTemp) {
        s.preheatTarget = s.maxTemp;
    } else if (s.preheatTarget < s.minTemp) {
        s.preheatTarget = s.minTemp;
    }
    if (s.reflowTarget > s.maxTemp) {
        s.reflowTarget = s.maxTemp;
    } else if (s.reflowTarget < s.minTemp) {
        s.reflowTarget = s.minTemp;
    }
    // check durations are within timeout
    if (s.preheatDuration > SETTINGS_MAX_HEATER_TIMEOUT) {
        s.preheatTarget = SETTINGS_MAX_HEATER_TIMEOUT;
    }
    if (s.reflowDuration > SETTINGS_MAX_HEATER_TIMEOUT) {
        s.reflowDuration = SETTINGS_MAX_HEATER_TIMEOUT;
    }

    return s;
}

unsigned int HeaterSettings::getMinTemp() {
    return this->settings.minTemp;
}

void HeaterSettings::setMinTemp(unsigned int minTemp) {
    this->settings.minTemp = minTemp;
}

unsigned int HeaterSettings::getMaxTemp() {
    return this->settings.maxTemp;
}

void HeaterSettings::setMaxTemp(unsigned int maxTemp) {
    this->settings.maxTemp = maxTemp;
}

unsigned int HeaterSettings::getHeaterTimeout() {
    return this->settings.heaterTimeout;
}

void HeaterSettings::setHeaterTimeout(unsigned int heaterTimeout) {
    if (heaterTimeout > SETTINGS_MAX_HEATER_TIMEOUT)
        this->settings.heaterTimeout = heaterTimeout;
}


bool HeaterSettings::save() {
    if (this->isDirty) {
        return this->save(this->settings);
    }
    return false;
}
