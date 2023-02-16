//Initial Code to Control the Solenoid: Jacob Tomazewski
//code for the button control taken from https://docs.arduino.cc/built-in-examples/digital/Debounce

//Set Up Global Variables
  const int Solenoid_Pin = 13;   //The solenoid pin connected to pin 13
  int Solenoid_State = LOW;       //set the state
  bool fly = false;


  //The contant and global variables for the push button to control the solenoid
  //This will be replaced with the output of the mocap cameras
  const int Button_Pin = 2              //button connected to pin 2
  int button_state;
  int previous_button_state = LOW;
  unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
  unsigned long debounceDelay = 50;    // the debounce time; increase if the output flickers


void setup() {
  // put your setup code here, to run once:
  pinMode(Solenoid_Pin, OUTPUT);

  //Button pin set up
  pinMode(Button_Pin, INPUT)
}

void loop() {
  // put your main code here, to run repeatedly:

  //Need to get a function to check the status if the area has been broken by the fly 
  //currently it will be controlled with a push button's status
  if(digitalRead(Button_Pin))    
      {fly = true}
  else
      {fly = false;}

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