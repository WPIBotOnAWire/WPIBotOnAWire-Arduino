#include <Arduino.h>

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
  }
}

void loop() {
  check_serial();
}