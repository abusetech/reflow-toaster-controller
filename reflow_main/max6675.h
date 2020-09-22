#ifndef MAX6675_H
#define MAX6675_H

#include <Arduino.h>

#define MAX6675_TEMP_MASK 0b0111111111111000
#define MAX6675_OPEN_MASK 0x0004
#define MAX6675_ID_MASK 2
#define MAX6675_STATE_MASK 1

class Max6675{
    uint16_t bits = 0;
    int sck, cs, so;
  public:
    Max6675(int sck, int cs, int so);
    boolean read_sensor();
    boolean is_open();
    float get_temperature();
};


#endif

