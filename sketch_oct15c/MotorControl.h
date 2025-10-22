#pragme once
#include <Arduino.h>
#include <Config.h>

class MotorControl {
  public:
    static void begin();
    static void moveForward(int pwm);
    static void moveBackward(int pwm);
    static void turnLeft(int leftSpeed, int rightSpeed);
    static void turnRight(int leftSpeed, int rightSpeed);
    static void stop();

  private:
    static void leftForward(int pwm);
    static void leftBackward(int pwm);
    static void rightForward(int pwm);
    static void rightBackward(int pwm);
}
