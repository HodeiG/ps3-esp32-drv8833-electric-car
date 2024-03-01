#include <Ps3Controller.h>

void setup()
{
    Serial.begin(9600);
    Ps3.begin();

}

void loop()
{
  if (Ps3.isConnected()){
    Serial.println("Connected!");
  } else {
    String address = Ps3.getAddress();
    Serial.println(address);
  }

  delay(3000);
}
