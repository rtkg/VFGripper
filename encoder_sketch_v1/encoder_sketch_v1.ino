#define SELECT_PIN 35
#define CLOCK_PIN 33
#define DATA_PIN 31

void setup()
{
  //setup our pins
  pinMode(DATA_PIN, INPUT);
  pinMode(CLOCK_PIN, OUTPUT);
  pinMode(SELECT_PIN, OUTPUT);

  //give some default values
  digitalWrite(CLOCK_PIN, HIGH);
  digitalWrite(SELECT_PIN, HIGH);

  Serial.begin(19200);
}

//variables to keep track of position
int reading = 0;
float angle = 0;

void loop()
{
    Serial.print("Reading: ");
   reading = readPosition();
   
   if (reading >= 0)
   {
      angle = (float)reading * 0.088;
//((float)reading / 1024.0) * 360.0;

      Serial.print("Reading: ");
      Serial.print(reading, DEC);
      Serial.print(" Angle: ");
      Serial.println(angle, 4); //DEC);
   }
   else
   {
      Serial.print("Error: ");
      Serial.println(reading);
   }
   
   delay(1000);
}

//read the current angular position
int readPosition()
{
  unsigned int position = 0;

  //shift in our data  
  digitalWrite(SELECT_PIN, LOW);
  delayMicroseconds(1);
  byte d1 = shiftIn(DATA_PIN, CLOCK_PIN,8);
  byte d2 = shiftIn(DATA_PIN, CLOCK_PIN,8);
  byte d3 = shiftIn(DATA_PIN, CLOCK_PIN,2);
  digitalWrite(SELECT_PIN, HIGH);

  //get our position variable
  position = d1;
  position = position << 8;
  position = position|d2;


  position = position >> 4;

  //check the offset compensation flag: 1 == started up
  if (!((d2 & B00001000) == B00001000))
    position = -1;

  //check the cordic overflow flag: 1 = error
  if (d2 & B00000100)
    position = -2;

  //check the linearity alarm: 1 = error
  if (d2 & B00000010)
    position = -3;

  //check the magnet range: 11 = error
  if ((d2 & B00000001) & (d3 & B10000000))
    position = -4;
    
  //add the checksum bit

  return position;
}

//read in a byte of data from the digital input of the board.
byte shiftIn(byte data_pin, byte clock_pin, int readBits)
{
  byte data = 0;

  for (int i=readBits-1; i>=0; i--)
  {
    digitalWrite(clock_pin, LOW);
    delayMicroseconds(1);
    digitalWrite(clock_pin, HIGH);
    delayMicroseconds(1);

    byte bit = digitalRead(data_pin);
    //Serial.println(bit, BIN);
    data = data|(bit << i);


  }

  //Serial.print("byte: ");
  //Serial.println(data, BIN);

  return data;
}

