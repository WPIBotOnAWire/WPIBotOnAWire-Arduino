#include <Arduino.h>
#include <ESC.h>



//PWM Out, Min, Max, Arming
ESC myESC1 (8, 1000, 2000, 1500);
ESC myESC2 (9, 1000, 2000, 1500);


// Packet format:
// /--------------------------------------------------\
// | CODE (uint32) | LEN (uint32) | params (variable) |
// \--------------------------------------------------/

struct Packet {
  uint32_t code;
  uint32_t length;
  char* params;
};

void setup() {
  Serial1.begin(115200);
  Serial.begin(9600);

  Serial.begin(115200);
  Serial.setTimeout(10000);
  Serial.print("Arming Motors");
  myESC1.arm();
  myESC2.arm();
  delay(8000);
  Serial.println(" - Done");

  bool hardwareTest = true;
  if (hardwareTest) {
    Serial.print("Motor 1 @ 1400");
    myESC1.speed(1400);
    delay(1500);
    Serial.println(" - Stopped");
    myESC1.speed(1500);
    delay(2500);
    Serial.print("Motor 1 @ 1600");
    myESC1.speed(1600);
    delay(1500);
    Serial.println(" - Stopped");
    myESC1.speed(1500);
    delay(2500);

    Serial.print("Motor 2 @ 1400");
    myESC2.speed(1400);
    delay(1500);
    Serial.println(" - Stopped");
    myESC2.speed(1500);
    delay(2500);
    Serial.print("Motor 2 @ 1600");
    myESC2.speed(1600);
    delay(1500);
    Serial.println(" - Stopped");
    myESC2.speed(1500);

    Serial.println("Motor Check Complete!");
  }
}

void check_serial() {
  if(Serial1.available()) {
    Packet p;
    Serial1.readBytes((char*)&p.code, sizeof(uint32_t));
    Serial1.readBytes((char*)&p.length, sizeof(uint32_t));
    char buf[p.length + 1];
    buf[p.length] = '\0';
    p.params = buf;

    int n = 0;
    while(n < p.length) {
      n += Serial1.readBytes(p.params + n, p.length - n);
    }

    Serial.print("Code: ");
    Serial.println(p.code);
    Serial.print("Message: ");
    Serial.println(*((uint32_t*)p.params));

    int ESCSpeed = p.params;

    myESC1.speed(ESCSpeed);                                   
    myESC2.speed(ESCSpeed);
  }
}

void loop() {
  check_serial();
}
