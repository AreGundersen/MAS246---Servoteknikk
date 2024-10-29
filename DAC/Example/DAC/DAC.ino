#include <dac.h>

void setup()
{
dac_init();
}

void loop()
{
set_dac(500,1000);
delay(1500);
set_dac(1500,2000);
delay(1500);
}
