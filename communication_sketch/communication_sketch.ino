#define POSITION_MODE 0
#define CURRENT_MODE 1
#define BELT_MODE 2
#define NO_MODE 3

void setup() {
  // start serial port at 9600 bps:
  Serial.begin(9600);
}

int mode = NO_MODE;
short target_val = 0;
  
void loop() {
  // put your main code here, to run repeatedly:
  processMessage();
  Serial.println(target_val, DEC);  
  Serial.println(mode, DEC);
  delay(100);
}

void processMessage() {

  byte b1,b2;
  if(Serial.available()) {
    char code[3];  
    for (int i=0; i<3; i++) {
        b1 = Serial.read();
        code[i] = b1;
    }
    //Serial.println(code);
    if(code[0] == 'P' && code[1] == 'O' && code[2] == 'S') {
       //position mode 
       mode = POSITION_MODE;
       b1 = Serial.read();
       b2 = Serial.read();
       target_val = b2;
       target_val = target_val << 8;
       target_val = target_val | b1;
       //Serial.println(b1, BIN);
       //Serial.println(b2, BIN);
    }
    if(code[0] == 'N' && code[1] == 'O' && code[2] == 'N') {
       mode = NO_MODE;
       target_val = 0;
    }
  } 
}


