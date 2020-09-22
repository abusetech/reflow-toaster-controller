#define LCD_WIDTH 16
#define PWM_PERIOD 3000

#define THERM_SCK 11
#define THERM_SO 3
#define THERM_CS 12
/***************************************************************************************/
//You MUST change the code in the WDT interrupt  to turn off the relay if this changes!
#define RELAY_PIN 10
//
/***************************************************************************************/

//Profile Paramters
//Rate parameters in degrees / millisecond
#define PREHEAT_RATE  0.002
#define SOAK_RATE     0.001
#define RAMP_RATE     0.003
//Timelimits -- these are for handling failures (in milliseconds)
#define PREHEAT_TIME_LIMIT 120000
#define SOAK_TIME_LIMIT    90000
#define RAMP_TIME_LIMIT    60000


#define btnRIGHT  0
#define btnUP     1
#define btnDOWN   2
#define btnLEFT   3
#define btnSELECT 4
#define btnNONE   5

#include <LiquidCrystal.h>
#include <string.h>
#include <stdio.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>

#include "slow_pwm.h"
#include "max6675.h"
#include "PID.h"

LiquidCrystal lcd(8, 9, 4, 5, 6, 7);                 // select the pins used on the LCD panel
SlowPWM relay_output(PWM_PERIOD, RELAY_PIN);         // relay to be controlled
Max6675 thermometer(THERM_SCK, THERM_CS, THERM_SO);  // thermocouple sensor
PID pid(0.1, 0.2, 0.05, PWM_PERIOD);                // PID controller

enum process_states{
  READY = 0,
  PREHEAT,
  SOAK,
  RAMP_UP,
  REFLOW,
  COOLING
};

// define some values used by the panel and buttons
int lcd_key     = 0;
int adc_key_in  = 0;

//Reflow process variables
unsigned long start_time = 0;
unsigned long last_time = 0;
unsigned long now = 0;
unsigned long elapsed_time = 0;
unsigned long reflow_start = 0;
//True if a reflow process is currently running
boolean running = false;
//Current temperature and desired temperature
float temperature = 20.0;
float setpoint = 20.0;
//Current process state
int phase = 0;

char* process_msg[] = {"READY", "PREHEAT", "SOAK", "RAMP UP", "REFLOW", "COOLING"};

int read_LCD_buttons(){               // read the buttons

if (!digitalRead(A1)) return btnSELECT;
else if (!digitalRead(A0)) return btnDOWN;
else return btnNONE;

/*
    adc_key_in = analogRead(0);       // read the value from the sensor 

    if (adc_key_in > 1000) return btnNONE; 
    if (adc_key_in < 50)   return btnRIGHT;  
    if (adc_key_in < 250)  return btnUP; 
    if (adc_key_in < 450)  return btnDOWN; 
    if (adc_key_in < 650)  return btnLEFT; 
    if (adc_key_in < 850)  return btnSELECT;  

    return btnNONE;                // when all others fail, return this.
*/
}

ISR(WDT_vect){
  //WDT Expired, All pins become inputs, pullups disabled
  DDRB = 0;
  DDRC = 0;
  DDRD = 0;
  PORTB = 0;
  PORTC = 0;
  PORTD = 0;
  //Work-around for questionable bootloader code
  //System reset flag is not cleared by the booloader
  wdt_disable();
  //"reset" the MCU -- works around needing bootloader modification
  asm volatile("jmp 0");
  //This is only executed if the MCU does not get reset
  while(1){}
}

void setup(){
   wdt_enable(WDTO_500MS);            //Enable the watchdog timer
   WDTCSR |= (1 << WDIE);          //Enable watch dog interrupt
   wdt_reset();
   
   pinMode(A0,INPUT_PULLUP);
   pinMode(A1, INPUT_PULLUP);
   
   lcd.begin(16, 2);               // start the LCD library
   lcd.setCursor(0,0);             // set the LCD cursor position 
   lcd.print("INITIALIZING");      // print a simple message on the LCD
   relay_output.setDutyCycle(0);   // Skillet relay OFF
   Serial.begin(9600);
}
 

void update(float dt){  

  if (running == true){
    elapsed_time = millis() - start_time;
    float output = pid.compute(temperature, setpoint);
    relay_output.setDutyCycle(output);
  }else{
    relay_output.setDutyCycle(0);
  }
  
  //PREHEAT: 2C/ s up to 150C
  if (phase == PREHEAT){
      setpoint += PREHEAT_RATE * dt;
      if (temperature >= 150){
        phase = SOAK;
      }else if(setpoint >= 150){
        setpoint = 150;
      }
  }else if(phase == SOAK){
      //SOAK: Ramp to 200 over 60s
      //we only increment the setpoint here
      setpoint += SOAK_RATE * dt;
      if (setpoint > 200){
        phase = RAMP_UP;
      }
  }else if(phase == RAMP_UP){
      //REFLOW: 3C/s to 255C
      setpoint += RAMP_RATE * dt;
      if (setpoint > 255){setpoint = 255;}
      if (temperature >= 250){
        phase = REFLOW;
        reflow_start = millis();
      }  
  }else if(phase == REFLOW){
    //Maintain 250C for 30s
    setpoint = 250;
    if(millis() - reflow_start > 30000){
      phase = COOLING;
    }
  }else if(phase == COOLING){
    setpoint = 0;
    if (temperature < 45){
      phase = READY; 
      stop_process();
    }
  }
  //COOLING: 6C/s down to room temp
}

void start_process(){
  start_time = millis();
  last_time = start_time;
  setpoint = temperature;
  running = true;
  phase = 1;
  pid.init(temperature, 0);
}

void stop_process(){
  running = false;
  start_time = 0;
  elapsed_time = 0;
  relay_output.setDutyCycle(0);
}

void update_lcd(unsigned long elapsed_time, float temperature, float setpoint, float duty){
   //Get a nice representation of elapsed time (elapsed_time is in mS)
   unsigned long seconds_total = elapsed_time/1000;
   unsigned int seconds = seconds_total % 60;
   unsigned int minutes = (seconds_total - seconds) / 60;
   //This ensures the time cannot be longer than 5 characters
   if(seconds > 99){seconds = 99;}
   if(minutes > 99){minutes = 99;}
   //This ensures the temperature cannot be longer than 3 characters
   int temp_temp = temperature;
   if(temp_temp > 999){temp_temp = 999;}
   if(temp_temp < -99){temp_temp = -99;}
   //Setpoint limited to 3 characters
   int set_temp = setpoint;
   if(set_temp > 999){set_temp = 999;}
   if(set_temp < -99){set_temp = -99;}
   //duty limited to 3 charaters
   int duty_temp = duty;
   if (duty_temp > 999){duty_temp = 999;}
   if (duty_temp < -99){duty_temp = -99;}
   //Buffer for out strings (Extra space left in case there is a small overflow)
   //Only 17 charaters are actually needed
   char line_1[20];
   char line_2[20];
   //format the text to be printed to the LCD
   sprintf(line_1, "%-12s%3dC", process_msg[phase], temp_temp);
   sprintf(line_2, "%02u:%02u  %3d%% %3dC", minutes, seconds, duty_temp, set_temp);
   //Print the text to the LCD
   lcd.setCursor(0,0);
   lcd.print(line_1);
   lcd.setCursor(0,1);
   lcd.print(line_2);
}

void failure(char* s){
  digitalWrite(RELAY_PIN, LOW); 
  lcd.setCursor(0,0);
  lcd.print(s);
  lcd.print(":");
  lcd.print(phase);
  while(1){wdt_reset();}
}

void loop(){
   wdt_reset();
   
   //Get Temperature
   if(thermometer.read_sensor()){failure("READ ERR");}           //Sensor read error
   if(thermometer.is_open()){failure("OPEN TC");}                  //Thermocouple disconnected
   //Sanity checks
   if (temperature > 280){failure("OVERHEAT");}                      //Overtemperature
   if (setpoint > 270){failure("SET LIMIT");}                        //Set point limit
   if (elapsed_time > 500000){failure("TIME LIMIT");}                //Process time limit (500s)
   if (relay_output.getHighTime() && !running){failure("PWM FAIL");} //PWM on and no process running
   if (digitalRead(RELAY_PIN) && !running){failure("STUCK ON");}     //Relay on and no process running
  
   now = millis();
   float time_s = float(now - last_time);
   
   //thermometer.read_sensor();
   temperature = thermometer.get_temperature();
   Serial.println(temperature);
      
   /*
   if (digitalRead(RELAY_PIN)){
    temperature += 0.003 * time_s;
   }else{
     temperature -= 0.009 * time_s;
   }
   */
   
   
   update(time_s);
   last_time = now;
   
   int duty = (relay_output.getHighTime() * 100.0)/relay_output.getPeriod();
   update_lcd(elapsed_time, temperature, setpoint, duty);
   relay_output.update();

    
   lcd_key = read_LCD_buttons();
   switch (lcd_key){               // depending on which button was pushed, we perform an action

       case btnRIGHT:{             //  push button "RIGHT" and show the word on the screen
            break;
       }
       case btnLEFT:{
             break;
       }    
       case btnUP:{
             break;
       }
       case btnDOWN:{
             stop_process();
             break;
       }
       case btnSELECT:{
             start_process();
             break;
       }
       case btnNONE:{
             break;
       }
   }
}

