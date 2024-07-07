//
// Created by DJ on 26/03/2024.
//

#ifndef HOTPLATE_RESCUE_SETTINGS_H
#define HOTPLATE_RESCUE_SETTINGS_H

/**
 * Absolute upper limit of the "max temp" setting
 */
#define SETTINGS_MAX_MAX_TEMP 250

/**
 * Absolute upper limit of the "heater timeout" setting (in seconds)
 */
#define SETTINGS_MAX_HEATER_TIMEOUT (1 * 60)

/**
 * Default EEPROM offset to save settings at
 */
#define SETTINGS_DEFAULT_EEPROM_ADDR 0

/**
 * Heater settings (this is what actually gets saved to EEPROM)
 */
struct settings_t {
    /**
     * Settings struct version (reserved for future use)
     */
    unsigned int version = 1;

    /**
     * Checksum of the settings struct (used to verify the settings are not corrupted during EEPROM operations)
     */
    unsigned long checksum = 1;

    /**
     * Minimum temp the hotplate supports
     */
    unsigned int minTemp = 0;

    /**
     * Maxiumum temp the hotplate supports
     */
    unsigned int maxTemp = 200;

    /**
     * Number of seconds the hotplate can be left ON continuously before it will turn off automatically (safety feature)
     */
    unsigned int heaterTimeout = 30 * 60; // 30 minutes
    /**
     * Preheat phase target temp (degrees C)
     */
    unsigned int preheatTarget = 120;
    /**
     * Reflow phase target temp (degrees C)
     */
    unsigned int reflowTarget = 170;
    /**
     * Preheat phase duration (in seconds)
     */
    unsigned int preheatDuration = 7 * 60; // 7 minutes
    /**
     * Reflow phase duration (in seconds)
     */
    unsigned int reflowDuration = 3 * 60; // 3 minutes
};


class HeaterSettings {
public:
    HeaterSettings() : eepromAddr(SETTINGS_DEFAULT_EEPROM_ADDR), settingsLoaded(false) {}
    HeaterSettings(int eepromAddress) : eepromAddr(eepromAddress), settingsLoaded(false) {}

    bool loadSettings();

    void saveSettings();

    // Getters
    unsigned int getMinTemp();
    unsigned int getMaxTemp();
    unsigned int getHeaterTimeout();
    unsigned int getPreheatTarget();
    unsigned int getReflowTarget();
    unsigned int getPreheatDuration();
    unsigned int getReflowDuration();

    // Setters
    void setMinTemp(unsigned int value);
    void setMaxTemp(unsigned int value);
    void setHeaterTimeout(unsigned int value);
    void setPreheatTarget(unsigned int value);
    void setReflowTarget(unsigned int value);
    void setPreheatDuration(unsigned int value);
    void setReflowDuration(unsigned int value);

private:
    settings_t settings;
    int eepromAddr;
    bool settingsLoaded;

    void setDefaultValues();
    void ensureSettingsLoaded();

    // Returns CRC-16-CCITT checksum of the settings struct
    unsigned long calculateCRC();
    // Enforces limits on the settings struct to ensure values are within bounds
    static void enforceLimits(settings_t &s);
};


#endif //HOTPLATE_RESCUE_SETTINGS_H
