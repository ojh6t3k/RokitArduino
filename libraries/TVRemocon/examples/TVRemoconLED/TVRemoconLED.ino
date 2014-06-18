#include <IRremote.h>
#include <TVRemocon.h>

unsigned char data = 0;

void setup()
{
   TVRemocon.begin();
   pinMode(8, OUTPUT);
   pinMode(9, OUTPUT);
}

void loop()
{
  data = TVRemocon.receive();
  
  if(data == 0xD8) digitalWrite(8, HIGH);
  else if(data == 0xB8) digitalWrite(9, HIGH);
  else {digitalWrite(8,LOW); digitalWrite(9, LOW);}
 
}
