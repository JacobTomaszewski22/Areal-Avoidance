//Initial Code to Control the Solenoid: Jacob Tomazewski
//code for the button control taken from https://docs.arduino.cc/built-in-examples/digital/Debounce



//Set Up Global Variables
  const int Solenoid_Pin = 22;   //The solenoid pin connected to pin 13
  int Solenoid_State = LOW;       //set the state
  bool fly = false;



void setup() {
  // put your setup code here, to run once:
  pinMode(Solenoid_Pin, OUTPUT);
  digitalWrite(Solenoid_Pin, LOW);

  pinMode(LED_BUILTIN, OUTPUT);

}

void loop() {
  // put your main code here, to run repeatedly:
    digitalWrite(Solenoid_Pin, LOW);
    digitalWrite(LED_BUILTIN, LOW);
    delay(10000);
    digitalWrite(Solenoid_Pin, HIGH);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(500);

}