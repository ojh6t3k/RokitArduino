#include <IRremote.h>
#include <TVRemocon.h>

unsigned char data = 0;

void setup()
{
   Serial.begin(9600);
   TVRemocon.begin(); 
   Serial.println(" Press any key..");
}

void loop()
{
   data = TVRemocon.receive();
   if(data == KEY_U) Serial.println("Pressed Up key.");
   else if(data == KEY_D) Serial.println( "Pressed Down key.");
   else if(data == KEY_L) Serial.println( "Pressed Left key.");
   else if(data == KEY_R) Serial.println( "Pressed Right key.");
   else if(data == KEY_C) Serial.println( "Pressed Center key.");
   else if(data == KEY_F1) Serial.println( "Pressed F1 key.");
   else if(data == KEY_F2) Serial.println( "Pressed F2 key.");
   else if(data == KEY_F3) Serial.println( "Pressed F3 key.");
   else if(data == KEY_F4) Serial.println( "Pressed F4 key."); 
}
