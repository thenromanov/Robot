#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <Robot.h>
#include <Wire.h>

Button::Button(int pin) : pin(pin) {
    pinMode(pin, INPUT_PULLUP);
}

bool Button::read() {
    return !digitalRead(pin);
}

Leds::Leds(int pin, int count) : pin(pin), count(count) {
    pixels = Adafruit_NeoPixel(count, pin, NEO_GRBW + NEO_KHZ800);
    pixels.begin();
    pixels.show();
}

void Leds::turn_on(int num=0, int r=255, int g=0, int b=0) {
    pixels.setPixelColor(num, pixels.Color(r, g, b));
    pixels.show();
}

void Leds::turn_off(int num=0) {
    pixels.setPixelColor(num, pixels.Color(0, 0, 0));
    pixels.show();
}

Motor::Motor(int pwm, int in1, int in2) : pwm(pwm), in1(in1), in2(in2) {
    pinMode(pwm, OUTPUT);
    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);
}

void Motor::start(int speed) {
    analogWrite(pwm, (abs(speed) > 255 ? 255 : abs(speed)));
    digitalWrite(in1, (speed >= 0));
    digitalWrite(in2, (speed < 0));
}

Gyro::Gyro(int sda, int scl) : sda(sda), scl(scl) {
    Wire.begin();
    Wire.setSDA(sda);
    Wire.setSCL(scl);
    mpu.initialize();
    mpu.dmpInitialize();
    mpu.CalibrateGyro(6);
    mpu.setDMPEnabled(true);
}

double Gyro::read() {
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        heading = ypr[0] - zero;
    }
    while (heading - mod < -PI) mod -= 2 * PI;
    while (heading - mod > PI) mod += 2 * PI;
    return heading;
}

void Gyro::set_zero() {
    zero = heading;
}

Cam_Block::Cam_Block() {}

Cam_Block::Cam_Block(int x, int y) : x(x), y(y) {}

Vect::Vect(int x, int y) : x(x), y(y) {}

Vect::Vect(const Cam_Block& a, const Cam_Block& b) {
    x = b.x - a.x;
    y = b.y - a.y;
}

double Vect::get_length() {
    return hypot(x, y);
}

int Vect::operator* (const Vect& b) const {
    return x * b.x + y * b.y;
}

int Vect::operator% (const Vect& b) const {
    return x * b.y - y * b.x;
}

double Vect::operator^ (const Vect& b) const {
    return atan2(x * b.y - y * b.x, x * b.x + y * b.y);
}