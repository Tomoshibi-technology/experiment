#include <Arduino.h>

#include "./control/control.h"

#include "./fled/fled.h"
#include <Adafruit_NeoPixel.h>
#define LED_PIN0 PA6
#define LED_NUM 300
Adafruit_NeoPixel neopixel0 = Adafruit_NeoPixel(LED_NUM, LED_PIN0, NEO_GRB + NEO_KHZ800);
FLED led0(&neopixel0, LED_NUM);

#include <HardwareSerial.h>
HardwareSerial PC(PA10, PA9); //UART1 RX, TX
//HardwareSerial ARM(PC7, PC6); //UART6 RX, TX
HardwareSerial POLE(PD2, PC12);

#define BCD(c) 5 * (5 * (5 * (5 * (5 * (5 * (5 * (c & 128) + (c & 64)) + (c & 32)) + (c & 16)) + (c & 8)) + (c & 4)) + (c & 2)) + (c & 1)

struct Note {
	int pitch;    // 音の高さ
	float duration; // ビートの長さ
};

void setup() {
	PC.begin(115200);
	PC.println("start");

	POLE.begin(1000000);
	
	// led0.init();
	// led0.set_color_rgb_all(50, 0, 0);
	// led0.show();

	//ーーーこれは必須ーーーーー
	control_init();
	//ーーーーーまじでーーーーー

	pinMode(PC11,INPUT); // 下のスライド
}

unsigned long loop_timer = 10000;
unsigned long sec_timer = 0;

uint32_t mycount = 0;

uint8_t kaeru0[9] = {0x0, 0x1, 0x2, 0x3, 0x2, 0x1, 0x0, 0xF, 0xF};
uint8_t kaeru1[56] = {0x0, 0x1, 0x2, 0x3, 0x2, 0x1, 0x0, 0xF,
											0x2, 0x3, 0x4, 0x5, 0x4, 0x3, 0x2, 0xF,
											0x0, 0xF, 0xF, 0x0, 0xF, 0xF, 0x0, 0xF, 0xF, 0x0, 0xF, 0xF, 
											0x0, 0xF, 0x0, 0xF, 0x1, 0xF, 0x1, 0xF,
											0x2, 0xF, 0x2, 0xF, 0x3, 0xF, 0x3, 0xF,
											0x2, 0xF, 0xF, 0x1, 0xF, 0xF, 0x0, 0xF, 0xF, 0xF, 0xF, 0xF
										};
uint8_t doremi[9] = {0x0, 0x1, 0x2, 0x3, 0x4, 0x5, 0x6, 0x7, 0xF};

float hogeee = 0.45;
Note kaeru2[42] = {
	{0x0,1},{0x1,1},{0x2,1},{0x3,1},{0x2,1},{0x1,1},{0x0,2},
	{0x2,1},{0x3,1},{0x4,1},{0x5,1},{0x4,1},{0x3,1},{0x2,2},
	{0x0,1},{0xF,1},{0x0,1},{0xF,1},{0x0,1},{0xF,1},{0x0,1},{0xF,1},
	{0x0,hogeee},{0xF,0.5-hogeee},{0x0,hogeee},{0xF,0.5-hogeee},
	{0x1,hogeee},{0xF,0.5-hogeee},{0x1,hogeee},{0xF,0.5-hogeee},
	{0x2,hogeee},{0xF,0.5-hogeee},{0x2,hogeee},{0xF,0.5-hogeee},
	{0x3,hogeee},{0xF,0.5-hogeee},{0x3,hogeee},{0xF,0.5-hogeee},
	{0x2,1},{0x1,1},{0x0,2},{0xF,2}
};


void loop() {
	//ーーーこれは必須ーーーーー
	control_loop();
	//ーーーーーまじでーーーーー

	//ーーーーーーーーーーループ計測ーーーーーーーーーー
	// PC.print(micros() - loop_timer);
	loop_timer = micros();
	if(sec_timer+1000 < millis()){
		sec_timer = millis();
		mycount++;
		mycount %= 250;
	}

	uint8_t send_data[12] = {
		0+mycount,1+mycount,2+mycount,3+mycount,4+mycount,5+mycount,
		6+mycount,7+mycount,8+mycount,9+mycount,10+mycount,11+mycount
	};

	for(int i=0; i<12;i++){
		send_data[i] = mycount;
	}
	
	//有線送信
	POLE.write(250);
	for(int i=0; i<12; i++){
		if(send_data[i] == 250) send_data[i] = 251;
		POLE.write(send_data[i]);
		// PC.print(send_data[i]);
	}
	

	// ーー無線の内容が見れます(別に消して大丈夫)ーー
	// if(twelite.read()){ //tweliteから受信成功したら1を返す
	// 	PC.print(micros() - loop_timer);
	// 	for(int i=0; i<4; i++){
	// 		PC.print("  :");
	// 		PC.print(twelite.receive_data[i]);
	// 	}
	// 	PC.println();
	// }
}
