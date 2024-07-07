#include <max6675.h>
#include "../.pio/libdeps/nanoatmega328/LiquidCrystal_I2C/LiquidCrystal_I2C.h"
// ============================================= Profiles =============================================
// Reflow profile
String states[4] = {"Preheat      ","Soak   ","Ramp up","Reflow "};

int Times_profile[3][4] = { {60, 150, 60, 20}, 
                            {60, 150, 60, 20},
                            {60, 200, 60, 20}};
int Temps_profile[3][4] = { {150, 165, 235, 235}, 
                            {150, 165, 250, 250},
                            {150, 165, 230, 230}};
// ===========================================   Settings   ===========================================
const int timeDelay = 20;
const int overshoot = 20;
const int hotspotCompensation = 20;
const int hysteresis = 1;
// ===========================================   Variables   ===========================================
int setpoint;
int roomTemp;
int currentState;
float timeElapsed = 0;
float timeSinceLastHeat = 0;
int timeleft = 0;
int totaltimeleft = 0;
float timeHeating = 0;
float error;
float measuredTemp;
int profile = 1;
// ============================================   States   =============================================
enum reflowStates{
  HEATING,
  COOL,
  STARTUP
};
reflowStates reflowState = STARTUP;
// =============================================   Pins   =============================================
// ThermoCouple
const int thermo_gnd_pin = A4;
const int thermo_vcc_pin = A3;
const int thermo_so_pin  = A0;
const int thermo_cs_pin  = A1;
const int thermo_sck_pin = A2;
// Relay 
const int relay_pin = A5;
// Display
const int rw = 4;
const int ledp = 10;
const int ledm = 11;
const int rs = 3;
const int en = 5;
const int d4 = 6;
const int d5 = 7;
const int d6 = 8;
const int d7 = 9;
// Buttons
const int button = 13;
// =======================================   Library objects  =========================================
MAX6675 thermocouple(thermo_sck_pin, thermo_cs_pin, thermo_so_pin);  
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);
// ============================================= Setup =============================================
void setup() {
  Serial.begin(9600);
  Serial.println("i, setpoint, measured, control, integral, derivative");
   
  // Compensate for hot spots & Add up the times
  for (int j = 0; j < sizeof(Temps_profile)/sizeof(Temps_profile[0]); j++){
    for (int i = 0; i < sizeof(Temps_profile[0])/sizeof(Temps_profile[0][0]); i++){
      Temps_profile[j][i] += hotspotCompensation;
    }
    for (int i = 1; i < sizeof(Times_profile[0])/sizeof(Times_profile[0][0]); i++){
      Times_profile[j][i] += Times_profile[j][i-1];
    }
  }
  
  // Pins
  pinMode(relay_pin, OUTPUT);
  digitalWrite(relay_pin, LOW);
  
  pinMode(thermo_vcc_pin, OUTPUT); 
  pinMode(thermo_gnd_pin, OUTPUT); 
  digitalWrite(thermo_vcc_pin, HIGH);
  digitalWrite(thermo_gnd_pin, LOW);

  pinMode(rw, OUTPUT);
  pinMode(ledp, OUTPUT);
  pinMode(ledm, OUTPUT);
  digitalWrite(rw, LOW);
  digitalWrite(ledp, HIGH);
  digitalWrite(ledm, LOW);

  pinMode(button, INPUT_PULLUP);

  // LCD
  lcd.begin(16, 2);
  lcd.setCursor(0, 1);
  lcd.print((int)thermocouple.readCelsius());
  lcd.setCursor(4, 1);
  lcd.print((char)223);
  lcd.print("C");
}

// ============================================= Loop =============================================
void loop() {
  switch (reflowState) {
    case HEATING:{
      setpoint = Times_profile[profile][currentState];
      timeleft = Times_profile[profile][currentState] - timeElapsed;
      totaltimeleft = Times_profile[profile][3] - timeElapsed;
      // LCD
      lcd.setCursor(0,0);
      lcd.print(states[currentState]);
      lcd.setCursor(9,0);
      lcd.print("        ");
      lcd.setCursor(9,0);
      lcd.print(timeleft);
      lcd.setCursor(13,0);
      lcd.print(totaltimeleft);
      
      if (timeElapsed >= Times_profile[profile][currentState]){
        currentState++;
        if (currentState == 4){
          reflowState = COOL;
          break;
        }
      }

      if (digitalRead(button) == 1){
        delay(200);
        if (digitalRead(button) == 1){
          reflowState = STARTUP;
          lcd.setCursor(0,0);
          lcd.print("Reflow stopped  ");
          Serial.println("Reflow Stopped");
          delay(2000);
        }
      }
    }
    break;   
    case COOL:{
      setpoint = 0;
      // LCD
      lcd.setCursor(0,0);
      lcd.print("Cooling down    ");
      if(measuredTemp < 50){
        reflowState = STARTUP;
      }
    }
    break;   
    case STARTUP:{
      setpoint = 0;
      // LCD
      lcd.setCursor(0,0);
      lcd.print("Profile ");
      lcd.print(profile);
      lcd.print("     ");
      lcd.setCursor(11, 1);
      lcd.print("START");
      // Check button
      if (digitalRead(button) == 1){
        delay(1000);
        if (digitalRead(button) == 1){ // long press: start reflow
          lcd.setCursor(11, 1);
          lcd.print(" STOP");
          lcd.setCursor(0, 0);
          lcd.print("Starting reflow");
          reflowState = HEATING;
          timeElapsed = 0;
          currentState = 0;
          delay(1000);
          Serial.println("Starting reflow");
        }
        else{ // short press: switch profile
          profile++;
          if(profile > sizeof(Temps_profile)/sizeof(Temps_profile[0])){
            profile = 1;
          }
          lcd.setCursor(0,0);
          lcd.print("Profile ");
          lcd.print(profile);
          lcd.print("     ");
          Serial.print("Switched to profile: ");
          Serial.println(profile);
        }
      }
    }
    break;
  }
  
  // MEASURE CURRENT TEMPERATURE
  measuredTemp = thermocouple.readCelsius();
  lcd.setCursor(0, 1);
  lcd.print(measuredTemp);

  // CALCULATE ERROR
  error = setpoint - measuredTemp;

  // CONVERT TO BANGBANG CONTROL WITH HYSTERESIS
  if (error >= overshoot) {
    digitalWrite(relay_pin, HIGH);
    timeSinceLastHeat = 0;
  }
  else {
    timeSinceLastHeat++;
    digitalWrite(relay_pin, LOW);
  }

  if ((timeSinceLastHeat > 30) && (error > hysteresis)) {
    digitalWrite(relay_pin, HIGH);
    timeHeating ++;
    if (timeHeating > 10) {
      timeSinceLastHeat = 0;
      timeHeating = 0;
    }
  }
  
  // PRINT
  Serial.print(timeElapsed);
  Serial.print(", ");
  Serial.print(setpoint);
  Serial.print(", ");
  Serial.print(measuredTemp);
  Serial.print(", ");
  Serial.print(digitalRead(relay_pin) * 100);
  Serial.print(", ");
  Serial.println(error);

  // ADVANCE CLOCK
  timeElapsed++;
  if (reflowState == STARTUP){
    delay(100); // check button in startup
  }
  else{
    delay(1000);// 1s time step otherwise
  }
}
