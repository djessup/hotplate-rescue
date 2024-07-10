#ifndef HOTPLATE_RESCUE_METRIC_H
#define HOTPLATE_RESCUE_METRIC_H

#include <Arduino.h>
#include "main.h"

struct Metric {
    String name;
    enum ValueType {
        INT,
        LONG,
        DOUBLE,
        FLOAT,
        UINT,
        ULONG,
        BOOL,
        INVALID
    } type;
    union {
        int intValue;
        long longValue;
        double doubleValue;
        float floatValue;
        unsigned int uintValue;
        unsigned long ulongValue;
        bool boolValue;
    } value;

    // Constructor for int
    Metric(const String &n, int v) : name(n), type(INT) {
        value.intValue = v;
    }

    // Constructor for long
    Metric(const String &n, long v) : name(n), type(LONG) {
        value.longValue = v;
    }

    // Constructor for double
    Metric(const String &n, double v) : name(n), type(DOUBLE) {
        value.doubleValue = v;
    }

    // Constructor for float
    Metric(const String &n, float v) : name(n), type(FLOAT) {
        value.floatValue = v;
    }

    // Constructor for unsigned int
    Metric(const String &n, unsigned int v) : name(n), type(UINT) {
        value.uintValue = v;
    }

    // Constructor for unsigned long
    Metric(const String &n, unsigned long v) : name(n), type(ULONG) {
        value.ulongValue = v;
    }

    // Constructor for unsigned long
    Metric(const String &n, bool v) : name(n), type(BOOL) {
        value.boolValue = v;
    }
};


// Print a telemetry metric (teleplot format)
inline void _PM(Metric metric) { 
    #if SERIAL_TELEMETRY
    #define SerialD Serial
    SerialD.print(">"); 
    SerialD.print(metric.name);  
    SerialD.print(":"); 
     switch (metric.type) { 
        case Metric::INT: 
            Serial.println(metric.value.intValue); 
            break; 
        case Metric::LONG: 
            Serial.println(metric.value.longValue); 
            break; 
        case Metric::DOUBLE: 
            Serial.println(metric.value.doubleValue); 
            break; 
        case Metric::FLOAT: 
            Serial.println(metric.value.floatValue); 
            break; 
        case Metric::UINT: 
            Serial.println(metric.value.uintValue); 
            break; 
        case Metric::ULONG: 
            Serial.println(metric.value.ulongValue); 
            break; 
        default: 
            Serial.println("Invalid Type"); 
    } 
    #endif
}

#endif // HOTPLATE_RESCUE_METRIC_H