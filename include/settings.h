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
    unsigned int heaterTimeout = 30 * 60;
    /**
     * Preheat phase target temp (degrees C/F)
     */
    unsigned int preheatTarget = 120;
    /**
     * Reflow phase target temp (degrees C/F)
     */
    unsigned int reflowTarget = 170;
    /**
     * Preheat phase duration (in seconds)
     */
    unsigned int preheatDuration = 7 * 60;
    /**
     * Reflow phase duration (in seconds)
     */
    unsigned int reflowDuration = 3 * 60;
};

/**
 * Encapsulates the hotplate settings that get persisted to EEPROM
 * Handles reading/writing to EEPROM and checking inputs are valid
 */
class HeaterSettings {
public:
    HeaterSettings();
    explicit HeaterSettings(int eepromAddr);

    unsigned int getPreheatTarget();
    void setPreheatTarget(unsigned int preheatTarget);

    unsigned int getReflowTarget();
    void setReflowTarget(unsigned int reflowTarget);

    unsigned int getPreheatDuration();
    void setPreheatDuration(unsigned int preheatDuration);

    unsigned int getReflowDuration();
    void setReflowDuration(unsigned int reflowDuration);

    unsigned int getMinTemp();
    void setMinTemp(unsigned int minTemp);

    unsigned int getMaxTemp();
    void setMaxTemp(unsigned int maxTemp);

    unsigned int getHeaterTimeout();
    void setHeaterTimeout(unsigned int heaterTimeout);

    /**
     * Loads settings from EEPROM, aligning values with configured limits if they fall outside
     */
    settings_t load();

    bool save();

private:
    /**
     * Offset where settings are saved in EEPROM
     */
    int eepromAddr;

    /**
     * True if settings in RAM are different than in EEPROM (i.e. a save is due)
     */
    bool isDirty;

    /**
     * Active settings object
     */
    settings_t settings;

    /**
     * Persist settings to EEPROM
     */
    bool save(settings_t s);

    /**
     * Check if a settings object is valid per the min/max limits set and fix it if not
     * @returns the settings object, with any out-of-bounds values changed to be within bounds.
     */
    static settings_t alignToLimits(settings_t s);
};


#endif //HOTPLATE_RESCUE_SETTINGS_H
