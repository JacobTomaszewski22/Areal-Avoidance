//Initial Code to Control the Solenoid: Jacob Tomazewski
//Set Up Global Variables
  const int Solenoid_Pin = 13;   //The solenoid pin
  int Solenoid_State = LOW;       //set the state
  bool fly = false;

void setup() {
  // put your setup code here, to run once:
  pinMode(Solenoid_Pin, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:

  //Need to get a function to check the status if the area has been broken by the fly
  if(fly == true)
    {
      Solenoid_State = HIGH;
    }
  else
  {
    Solenoid_State = LOW;
  }

  digitalWrite(Solenoid_Pin, Solenoid_State);

}