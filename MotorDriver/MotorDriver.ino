//Test for the Motor Driver
//Robert Krug, Todor Stoyanov 13/06/2014

int M1_IN1=24; //Motor input 1
int M1_IN2=25; //Motor input 2
int M1_SF=26;  // Motor status flag
int M1_FB=54;  //Analog input A0 for current sensing on Motor 1
int EN =27;    //Driver board enable pin
int M1_D2=6;   // PWM pin

int count = 0;

int cs_m1 = 0; //value of the current sensor for motor 1 
 
void setup() {
  pinMode(M1_IN1,OUTPUT);
  pinMode(M1_IN2,OUTPUT);
  pinMode(M1_SF,INPUT);
  pinMode(M1_FB,INPUT);
  pinMode(EN,OUTPUT);
  pinMode(M1_D2,OUTPUT);
  
  analogWriteResolution(12); //sets the resolution of the analogWrite(...) function to 12 bit, i.e., between 0 - 4095
  
  Serial.begin(9600); //open a serial connection
  
  digitalWrite(EN,HIGH); //Enable the driver board
}

void loop() 
{
  float control_value=200;
  
  //M1_IN1 and M1_IN2 set the direction
  if (count%2 == 0)
  {
    digitalWrite(M1_IN1,LOW);
    digitalWrite(M1_IN2,HIGH); 
  }
  else
  {
    digitalWrite(M1_IN1,HIGH);
    digitalWrite(M1_IN2,LOW);  
  }  
  
  analogWrite(M1_D2, map(control_value, 0, 1023, 0, 4095)); //map the control value in the PWM range
  Serial.print(" , 12-bit PWM value : ");
  Serial.print(map(control_value, 0, 1023, 0, 4095));
  
  delay(2000); //wait 2 seconds
  
  cs_m1 = analogRead(M1_FB);    // read the current sensor
  Serial.print(" , CS value : ");
  Serial.println(cs_m1);    
  
  count++;
}
