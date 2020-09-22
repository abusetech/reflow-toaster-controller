#ifndef MAX6675_CPP
#define MAX6675_CPP

#include "max6675.h"


Max6675::Max6675(int sck, int cs, int so){
  this->sck = sck;
  this->cs = cs;
  this->so = so;
  pinMode(cs, OUTPUT);
  pinMode(sck, OUTPUT);
  pinMode(so, INPUT);
  digitalWrite(cs, HIGH);
}

boolean Max6675::read_sensor(){
  //Updates the sensor object with the new status and temperature
  //Returns true on error
  
  //CS goes low
  //SCK goes high then low
  //read SO
  //Repeat
  //CS goes high
  
  digitalWrite(cs, LOW);
  delay(2);
  digitalWrite(cs, HIGH);
  delay(220);
  digitalWrite(cs, LOW);
  
  uint16_t temp = 0;
  for (int i = 15; i >= 0; i--){
    digitalWrite(sck, HIGH);
    delay(2);
    temp += digitalRead(so) << i;
    digitalWrite(sck,LOW);
    delay(2);
  }
  
  digitalWrite(cs, HIGH);
  
  Serial.println(temp);
  if (temp & MAX6675_ID_MASK){
    //ID bit should be 0
    return true;
  }else{
    bits = temp;
    return false;
  }
  
}

float Max6675::get_temperature(){
  //LSB is 0.25C, so shift by 2 to truncate
  return float((bits & MAX6675_TEMP_MASK)>>3)/4.0;
}

boolean Max6675::is_open(){
  return (bits & MAX6675_OPEN_MASK) > 0;
}


#endif
