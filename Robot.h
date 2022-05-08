#ifndef Robot_h
#define Robot_h

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <MPU6050_6Axis_MotionApps20.h>

class Button {
	public:
		Button(int pin);

		bool read();

	private:
		int pin;
};


class Leds {
	public:
		Leds(int pin, int count);

		void turn_on(int num=0, int r=255, int g=0, int b=0);

		void turn_off(int num=0);

	private:
		int pin, count;
		Adafruit_NeoPixel pixels;
};

class Motor {
	public:
		Motor(int pwm, int in1, int in2);

		void start(int speed);

	private:
		int pwm, in1, in2;
};

class Gyro {
	public:
		Gyro(int sda, int scl);

		double read();

	private:
		double heading;
		int sda, scl;
		uint8_t fifoBuffer[64];
		Quaternion q;
		VectorFloat gravity;
		float ypr[3], mod = 0;
		MPU6050 mpu;
};

class Cam_Block {
	public:
		int x, y, w, h;
		uint64_t time;
		double angle, dist;
		bool found;	

		Cam_Block();

		Cam_Block(int x, int y);
};

class Vect {
	public:
		int x, y;

		Vect (int x, int y);

		Vect (const Cam_Block& a, const Cam_Block& b);

		double get_length();

		int operator* (const Vect& b) const;
		int operator% (const Vect& b) const;
		double operator^ (const Vect& b) const;
};

#endif