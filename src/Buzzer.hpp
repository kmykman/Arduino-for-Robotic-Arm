#ifndef __BUZZER__
#define __BUZZER__

class Buzzer {
  private:
    int pin;
  public:
    Buzzer(int pin): pin(pin) {
    }
    void beep() {
      tone(pin, 500, 500);
    }
    void beepError() {
      tone(pin, 100, 1000);
    }
    void beepShort() {
      tone(pin, 50, 100);
    }
};

#endif //__BUZZER__
