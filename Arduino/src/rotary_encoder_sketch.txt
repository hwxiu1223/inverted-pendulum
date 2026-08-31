#include <Arduino.h>

const int pinA = 2;
const int pinB = 3;

volatile long enc_count = 0;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200); // up to 115200
  pinMode(pinA, INPUT_PULLUP);
  pinMode(pinB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(pinA), interrupt, CHANGE); //triggers whenever A changes
}

void loop() {
  // put your main code here, to run repeatedly:
  if (Serial.available() > 0) {
    String count_line = Serial.readStringUntil('\n');
    count_line.trim(); //removes whitespace

    if (count_line.startsWith("SET,")){
      long saved_count = count_line.substring(4).toInt();
      noInterrupts();
      enc_count = saved_count;
      interrupts();
      Serial.print("Encoder count set to: ");
      Serial.println(enc_count);

    }
  }


  static long prev_count = 0;

  if (prev_count != enc_count){
  Serial.print("Count: ");
  Serial.println(enc_count);
  prev_count = enc_count;
  }
}


void interrupt(){
  if (digitalRead(pinA) == digitalRead(pinB)){  
    enc_count++;
    }
  else{
    enc_count--;
    }
 }
