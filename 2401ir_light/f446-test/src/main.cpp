#include <Arduino.h>

int A = D7;
int B = D8;
int C = D9;

void setup() {
	pinMode(A, OUTPUT);
	pinMode(B, OUTPUT);
	pinMode(C, OUTPUT);

	//pwm周波数を設定
	analogWriteFrequency(10000);
}

void loop() {
	analogWrite(A, 127);
	analogWrite(B, 0);
	analogWrite(C, 0);
	delay(1);
	analogWrite(A, 0);
	analogWrite(B, 127);
	analogWrite(C, 0);
	delay(1);
	analogWrite(A, 0);
	analogWrite(B, 0);
	analogWrite(C, 127);
	delay(1);
	analogWrite(A, 0);
	analogWrite(B, 0);
	analogWrite(C, 0);
	delay(1);
}
