#include "DHT.h"
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include <EEPROM.h>

#define LM35_SENSOR_CHANNEL 0
#define ADC_REF_VOLTAGE 5.0
#define BULB_PIN 6     // 6
#define BUTTON_OK 0     // 8
#define BUTTON_CANCEL 1 // 9
#define BUTTON_NEXT 2   // 10 PLUS
#define BUTTON_PREV 3   // 11 MINUS


LiquidCrystal_I2C lcd(0x3F, 16, 2);

enum Buttons {
  EV_OK,
  EV_CANCEL,
  EV_NEXT,
  EV_PREV,
  EV_NONE,
  EV_MAX_NUM
};

enum Menus {

  MENU_MAIN= 0,
  MENU_KP,
  MENU_KI,
  MENU_KD,
  MENU_Temperatura,
  MENU_TimpIncalzire,
  MENU_TimpMentinere,
  MENU_TimpRacire,
  MENU_Perturbatii,
  MENU_StatusCheck,
  MENU_MAX_NUM
};



// Adress
int addrs_kp = 100;
int addrs_ki = 108;
int addrs_kd = 116;
int addrs_tempSet = 124;
int addrs_tincalzire = 132;
int addrs_tmentinere = 140;
int addrs_tracire = 148;

// Values
int tempAdc;
double tempSet;
double tempActual

// Time Variables
unsigned long tincalzire;
unsigned long tmentinere;
unsigned long tracire;
unsigned long now = 0;
unsigned long previousMillis = 0;
unsigned long previousRandomMillis = 0;

String situatieProces = "";

//PID Values
double kp, ki, kd;
double dt, last_time = 0;
double proportional, derivative;
double integral, previous, output = 0;
double actual, error;
double setpoint;

// Booleans
bool isPerturb = false;
bool isFirstIncalzire = false;
bool isFirstMentinere = false;

Menus scroll_menu = MENU_MAIN;
Menus current_menu =  MENU_MAIN;

void state_machine(enum Menus menu, enum Buttons button);
Buttons GetButtons(void);
void print_menu(enum Menus menu);

typedef void (state_machine_handler_t)(void);

void print_menu(enum Menus menu){
  lcd.clear();
  switch(menu)
  {
    case MENU_Perturbatii:
      lcd.print(" Mod Perturbatii");
      lcd.setCursor(0,1);
      if(isPerturb){
        lcd.print(" - <<  ON  >> + ");
      }else{
        lcd.print(" - <<  OFF >> + ");
      }
      break;
    case MENU_MAIN:    
      tempActual = convertAdcToCelsius(actual);
      lcd.print("A:");
      lcd.print(tempActual);
      lcd.print("  S:");
      tempSet = read_memory(addrs_tempSet);
      lcd.print(tempSet);

  	  lcd.setCursor(0,1);
      lcd.print(" - << Main >> + ");
      break;
    case MENU_KP:
    	lcd.print("   KP = ");
      kp = read_memory(addrs_kp);
    	lcd.print(kp);

  	  lcd.setCursor(0,1);
      lcd.print(" - <<  KP  >> + ");
    	break;
    case MENU_KI:	
    	lcd.print("   KI = ");
      ki = read_memory(addrs_ki);
    	lcd.print(ki);

  	  lcd.setCursor(0,1);
      lcd.print(" - <<  KI  >> + ");
      break;
    case MENU_KD:	
    	lcd.print("   KD = ");
      kd = read_memory(addrs_kd);
    	lcd.print(kd);

  	  lcd.setCursor(0,1);
      lcd.print(" - <<  KD  >> + ");
      break;
    case MENU_Temperatura:
    	lcd.print("  TEMP = ");
      tempSet = read_memory(addrs_tempSet);
    	lcd.print(tempSet);

  	  lcd.setCursor(0,1);
      lcd.print(" - << Temp >> + ");
    	break;
    case MENU_TimpIncalzire:
    	lcd.print("T Incalz = ");
      tincalzire = read_memory(addrs_tincalzire);
    	lcd.print(MilliToSeconds(tincalzire),1);
    	lcd.print("s");

  	  lcd.setCursor(0,1);
      lcd.print(" - << Tinc >> + ");
    	break;
    case MENU_TimpMentinere:
    	lcd.print("T Mentin = ");
      tmentinere = read_memory(addrs_tmentinere);
    	lcd.print(MilliToSeconds(tmentinere),1);
    	lcd.print("s");

  	  lcd.setCursor(0,1);
      lcd.print(" - << Tmen >> + ");
    	break;
    case MENU_TimpRacire:
    	lcd.print("T Racire = ");
      tracire = read_memory(addrs_tracire);
    	lcd.print(MilliToSeconds(tracire),1);
    	lcd.print("s");
      
  	  lcd.setCursor(0,1);
      lcd.print(" - << Trac >> + ");
    	break;
    case MENU_StatusCheck:
      tempActual = convertAdcToCelsius(actual);
      lcd.print("A:");
      lcd.print(tempActual);
      lcd.print("  ");
      lcd.print(situatieProces);

      lcd.setCursor(0,1);
      lcd.print("Time: ");

      if(situatieProces == "Incalzire"){
        lcd.print(MilliToSeconds(double(now - previousMillis)),1);
        lcd.print("/");
        lcd.print(MilliToSeconds(tincalzire),1);
      }else if(situatieProces == "Mentinere"){
        lcd.print(MilliToSeconds(double(now - previousMillis- tincalzire)),1);
        lcd.print("/");
        lcd.print(MilliToSeconds(tmentinere),1);
      }else if(situatieProces == "Racire   "){
        lcd.print(MilliToSeconds(double(now - previousMillis- tincalzire- tmentinere)),1);
        lcd.print("/");
        lcd.print(MilliToSeconds(tracire),1);
      }else{
        lcd.print("");
      }
      break;
    default:
    	lcd.print("ERR DEFAULT");
   		break;
  }
  if(current_menu != MENU_MAIN && current_menu != MENU_Perturbatii && current_menu != MENU_StatusCheck)
  {
  	lcd.setCursor(0,1);
  	lcd.print("  Change Value  ");
  }
}


void start_perturb(void){
  if(isPerturb){
    isPerturb = false;
  }else{
    isPerturb = true;
  }
}
void enter_menu(void){
  current_menu = scroll_menu;
}
void go_home(void){
  scroll_menu = MENU_MAIN;
  current_menu = scroll_menu;
}
void go_next(void){
  scroll_menu = (Menus) ((int)scroll_menu + 1);
  scroll_menu = (Menus) ((int)scroll_menu % MENU_MAX_NUM);
}
void go_prev(void){
  if(scroll_menu == MENU_MAIN){
    scroll_menu = MENU_StatusCheck;
  }else{
    scroll_menu = (Menus) ((int)scroll_menu - 1);
    scroll_menu = (Menus) ((int)scroll_menu % MENU_MAX_NUM);
  }
}


void inc_kp(void){
  kp += 0.1;
  save_memory(addrs_kp, kp);
}
void dec_kp(void){
  kp -= 0.1;
  save_memory(addrs_kp, kp);
}
void inc_ki(void){
  ki += 0.01;
  save_memory(addrs_ki, ki);
}
void dec_ki(void){
  ki -= 0.01;
  save_memory(addrs_ki, ki);
}
void inc_kd(void){
  kd = kd + 0.01;
  save_memory(addrs_kd, kd);
}
void dec_kd(void){
  kd -= 0.01;
  save_memory(addrs_kd, kd);
}
void inc_tempSet(void){
  tempSet += 0.1;
  setpoint = convertCelsiusToAdc(tempSet);
  save_memory(addrs_tempSet, tempSet);
}
void dec_tempSet(void){
  tempSet -= 0.1;
  setpoint = convertCelsiusToAdc(tempSet);
  save_memory(addrs_tempSet, tempSet);
}
void inc_timp_incalzire(void){
  tincalzire = tincalzire + 500;
  save_memory(addrs_tincalzire, tincalzire);
}
void dec_timp_incalzire(void){
  tincalzire = tincalzire - 500;
  save_memory(addrs_tincalzire, tincalzire);
}
void inc_timp_mentinere(void){
  tmentinere = tmentinere + 500;
  save_memory(addrs_tmentinere, tmentinere);
}
void dec_timp_mentinere(void){
  tmentinere = tmentinere - 500;
  save_memory(addrs_tmentinere, tmentinere);
}
void inc_timp_racire(void){
  tracire = tracire + 500;
  save_memory(addrs_tracire, tracire);
}
void dec_timp_racire(void){
  tracire = tracire - 500;
  save_memory(addrs_tracire, tracire);
}

// Save memory to EEPROM
void save_memory(int address, double value){ 
  double storedValue;

  EEPROM.get(address, storedValue);
  if (value != storedValue) {
    EEPROM.put(address, value);
  }
}

// Read memory from EEPROM
double read_memory(int address){
  double storedValue;
  EEPROM.get(address, storedValue);
  return storedValue;
}

state_machine_handler_t* sm[MENU_MAX_NUM][EV_MAX_NUM] = 
{ //events: OK , CANCEL , NEXT, PREV
  {enter_menu, go_home, go_next, go_prev},                      // MENU_MAIN
  {go_home, go_home, inc_kp, dec_kp},                           // MENU_KP
  {go_home, go_home, inc_ki, dec_ki},                           // MENU_KI
  {go_home, go_home, inc_kd, dec_kd},                           // MENU_KD
  {go_home, go_home, inc_tempSet, dec_tempSet},                 // MENU_Temperatura
  {go_home, go_home, inc_timp_incalzire, dec_timp_incalzire},   // MENU_TimpIncalzire
  {go_home, go_home, inc_timp_mentinere, dec_timp_mentinere},   // MENU_TimpMentinere
  {go_home, go_home, inc_timp_racire, dec_timp_racire},         // MENU_TimpRacire
  {start_perturb, go_home, go_home, go_home},                   // MENU_Perturbatii
  {go_home, go_home, go_home, go_home},                         // MENU_StatusCheck
};

void state_machine(enum Menus menu, enum Buttons button){
  sm[menu][button]();
}

Buttons GetButtons(void){
  enum Buttons ret_val = EV_NONE;
  if ((PINB &= (1 << BUTTON_OK)))
  {
    ret_val = EV_OK;
  }
  else if ((PINB &= (1 << BUTTON_CANCEL)))
  {
    ret_val = EV_CANCEL;
  }
  else if ((PINB &= (1 << BUTTON_NEXT)))
  {
    ret_val = EV_NEXT;
  }
  else if ((PINB &= (1 << BUTTON_PREV)))
  {
    ret_val = EV_PREV;
  }
  return ret_val;
}

void init_adc() {
  // Reference Selection
  ADMUX |= (1 << REFS0); 
  // ADC Prescaler Selections 128
  ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
  // ADC Enable
  ADCSRA |= (1 << ADEN); 
}
double read_lm35_adc() {
  ADMUX &= 0xF0; 
  // ADC A0 pin selection
  ADMUX |= LM35_SENSOR_CHANNEL; 
  // start the ADC conversion
  ADCSRA |= (1 << ADSC);
  // wait for the conversion to complete
  while (ADCSRA & (1 << ADSC)) {}
  // get the ADC result and calculate the temperature
  int adc_value = ADC;
  // Analog value to Voltage value
  return (double)adc_value;
}
double convertAdcToCelsius(double adc_value){
  double voltage = adc_value * ADC_REF_VOLTAGE / 1024.0;
  // Voltage to Celsius
  double temperature = voltage * 100.0;
  return temperature;
}

int convertCelsiusToAdc(double temperature){
  int adc_value = temperature/ 100 * 1024 / ADC_REF_VOLTAGE;
  return adc_value;
}
void PID(double target){
 
  dt = (now - last_time)/1000;
  last_time = now;

  error = target - actual;

  proportional = error;
  integral += error * dt;
  derivative = (error - previous) / dt;
  previous = error;

  output = (kp * proportional) + (ki * integral) + (kd * derivative);
  if(output > 255){
    output = 255;
  }else if(output < 0){
    output = 0;
  }
}

double MilliToSeconds(double ms){
  return ms / 1000;
}

void setup()
{
  Serial.begin(9600);
  lcd.begin();
	lcd.backlight();
  init_adc();

  isFirstIncalzire = true;
  isFirstMentinere = false;

  kp = read_memory(addrs_kp);
  ki = read_memory(addrs_ki);
  kd = read_memory(addrs_kd);
  tincalzire = read_memory(addrs_tincalzire);
  tmentinere = read_memory(addrs_tmentinere);
  tracire = read_memory(addrs_tracire);
  tempSet = read_memory(addrs_tempSet);
  setpoint = convertCelsiusToAdc(tempSet);
  randomSeed(analogRead(0));


  save_memory(addrs_kp, 10);
  save_memory(addrs_ki, 0.40);
  save_memory(addrs_kd, 0.00);
  save_memory(addrs_tempSet, 50);
  save_memory(addrs_tincalzire, 40000);
  save_memory(addrs_tmentinere, 20000);
  save_memory(addrs_tracire, 10000);
  setpoint = convertCelsiusToAdc(50);
}



void loop()
{
  volatile Buttons event = GetButtons();

  if (event != EV_NONE) {
    state_machine(current_menu, event);
  }
  print_menu(scroll_menu);

  now = millis();

  actual = read_lm35_adc();  
  targetTemp = 0;

  // 20 and -1000 are just for error managmenet; usually is without
  tempIncreasePerMs = (setpoint + 20 - actual) / (tincalzire-1000) ;
  
  // If Perturbation is active
  if (isPerturb && situatieProces != "racire"){
    unsigned long timpDeAsteptare = random(2000, 4001);
    if(now - previousRandomMillis >= timpDeAsteptare){
      output = (double)random(256);
      Serial.print("Random Value:");
      Serial.println(output);
      analogWrite(BULB_PIN,output);
      delay(300);
      previousRandomMillis = now;;
    }
  }

  // 3 STAGES: INCALZIRE, MENTINERE, RACIRE

  // INCALZIRE
  if (isFirstIncalzire && (actual < setpoint || (now - previousMillis <= tincalzire))) {
    targetTemp = actual + tempIncreasePerMs * (now - previousMillis);

    if (actual >= setpoint) {
      targetTemp = setpoint;
    }

    Serial.print("Timer: ");
    Serial.print(now - previousMillis);
    Serial.print("/");
    Serial.println(tincalzire);

    PID(targetTemp);
    situatieProces = "Incalzire";

    // MENTINERE
  } else if (actual >= (setpoint-1.5) && (now - previousMillis > tincalzire && now - previousMillis <= tincalzire + tmentinere)) {
    PID(setpoint);
    situatieProces = "Mentinere";
    isFirstIncalzire = false;
    isFirstMentinere = true;
    Serial.print("Timer: ");
    Serial.print(now - previousMillis - tincalzire);
    Serial.print("/");
    Serial.println(tmentinere);
  
    // RACIRE
  } else if ((now - previousMillis > tincalzire + tmentinere) && (now - previousMillis) <= (tincalzire + tmentinere + tracire)) {
    output = 0;
    isFirstMentinere = false;
    situatieProces = "Racire   ";
    Serial.print("Timer: ");
    Serial.print(now - previousMillis - tincalzire - tmentinere);
    Serial.print("/");
    Serial.println(tracire);

    // RESTART VALUES
  } else if (!isFirstMentinere){
    output = 0;
    situatieProces = "";
    previousMillis = now;
    isFirstIncalzire = true;
  }
  
  analogWrite(BULB_PIN,output);
  
  // Serial Plotter
  Serial.print(setpoint);
  Serial.print(",");
  Serial.print(actual);
  Serial.print(",");
  Serial.print(output);
  Serial.print(",");
  Serial.println(situatieProces);
  delay(350);
}
