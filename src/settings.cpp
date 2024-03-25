//
// Created by DJ on 26/03/2024.
//

#include "settings.h"
#include "EEPROM.h"
#include <new>


HeaterSettings currentSettings{
        .targetTemp = DEFAULT_TARGET_TEMP,
};

//
//void storeSettings() {
//    int eeAddress = EEPROM_SETTINGS_ADDR;
//    EEPROM.put(eeAddress, currentSettings);
//    saveSettings = false;
//    Serial.println("Settings saved.");
//}
//
//HeaterSettings loadSettings() {
//    int eeAddress = EEPROM_SETTINGS_ADDR;
//    HeaterSettings settings{};
//    EEPROM.get(eeAddress, settings);
//
//    if (isnan(settings.targetTemp) || settings.targetTemp < MIN_TEMP || settings.targetTemp > MAX_TEMP) {
//        settings.targetTemp = DEFAULT_TARGET_TEMP;
//    }
//    return settings;
//}

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

    if (this->isValid(storedSettings)) {
        return storedSettings;
    } else {
        storedSettings = settings_t(); // use defaults
        return storedSettings;
    }
}

bool HeaterSettings::save(settings_t s) {
    if (!isValid(s)) {
        return false;
    }
    EEPROM.put(this->eepromAddr, s);
    return true;
}

bool HeaterSettings::isValid(settings_t s) {
//    bool valid = true;
    return (s.preheatTarget < this->minTemp
         || s.preheatTarget > this->maxTemp
         || s.reflowTarget  < this->minTemp
         || s.reflowTarget  > this->maxTemp
         || s.preheatDuration > this->heaterTimeout
         || s.reflowDuration  > this->heaterTimeout)
}

unsigned int HeaterSettings::getMinTemp() {
    return minTemp;
}

void HeaterSettings::setMinTemp(unsigned int minTemp) {
    this->minTemp = minTemp;
}

unsigned int HeaterSettings::getMaxTemp() {
    return maxTemp;
}

void HeaterSettings::setMaxTemp(unsigned int maxTemp) {
    this->maxTemp = maxTemp;
}

unsigned int HeaterSettings::getHeaterTimeout() {
    return heaterTimeout;
}

void HeaterSettings::setHeaterTimeout(unsigned int heaterTimeout) {
    this->heaterTimeout = heaterTimeout;
}

HeaterSettings::HeaterSettings(int eepromAddr, unsigned int minTemp, unsigned int maxTemp, unsigned int timeout) {
    this->eepromAddr = eepromAddr;
    this->setMinTemp(minTemp);
    this->setMaxTemp(maxTemp);
    this->setHeaterTimeout(timeout);
    this->settings = this->load();
}

HeaterSettings::HeaterSettings() {
    new (this) HeaterSettings(SETTINGS_DEFAULT_EEPROM_ADDR,
                              SETTINGS_DEFAULT_MIN_TEMP,
                              SETTINGS_DEFAULT_MAX_TEMP,
                              SETTINGS_DEFAULT_MAX_TIMEOUT);

}

bool HeaterSettings::save() {
    if (this->isDirty) {
        return this->save(this->settings);
    }
    return false
}
