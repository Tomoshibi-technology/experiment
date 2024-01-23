#include <Arduino.h>
#include <HardwareSerial.h>

#define VERSION 1.00

#include "./fled/fled.h"
#include <Adafruit_NeoPixel.h>
#define LED_A_PIN 32  // The data pin that the WS2812 strips are connected to.32
#define LED_B_PIN 14
#define LED_C_PIN 13
#define LED_D_PIN 33
#define LED_E_PIN 27
#define LED_NUM 60

Adafruit_NeoPixel rawpixel[5] = {
	Adafruit_NeoPixel(60, LED_A_PIN, NEO_GRB + NEO_KHZ800),
	Adafruit_NeoPixel(120, LED_B_PIN, NEO_GRB + NEO_KHZ800),
	Adafruit_NeoPixel(60, LED_C_PIN, NEO_GRB + NEO_KHZ800),
	Adafruit_NeoPixel(120, LED_D_PIN, NEO_GRB + NEO_KHZ800),
	Adafruit_NeoPixel(40, LED_E_PIN, NEO_GRB + NEO_KHZ800),
};

FLED led[6] = {
	FLED(&rawpixel[0], 0, 59),
	FLED(&rawpixel[1], 119, 60),
	FLED(&rawpixel[1], 0, 59),
	FLED(&rawpixel[2], 0, 59),
	FLED(&rawpixel[3], 119, 60),
	FLED(&rawpixel[3], 0, 59)
};

FLED circuit_led[4] = {
	FLED(&rawpixel[4], 0, 9), //LED_FROM, LED_TO
	FLED(&rawpixel[4], 10, 19),
	FLED(&rawpixel[4], 20, 29),
	FLED(&rawpixel[4], 30, 39),
};


#include "./ser_ctrl/ser_ctrl.h"
HardwareSerial ctrl(2);
CTRL ser_ctrl(&ctrl);  


#include "./oled/oled.h"
#include <U8g2lib.h>
#define SDA_PIN 21
#define SCL_PIN 22
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0,/* reset=*/ U8X8_PIN_NONE);
OLED oled(&u8g2);

#include "./speaker/speaker.h"
#define SPEAKER_PIN 4
SPEAKER speaker(SPEAKER_PIN);

#include "./power/power.h"
#define VOL_PIN 26 //これ、無線機能のピンとかぶってるから注意
#define VOL_MAX 12.6
#define VOL_MIN 11.1
POWER power(VOL_PIN, VOL_MAX, VOL_MIN);

#include "./dip/dip.h"
#define DIP_PIN0 25
#define DIP_PIN1 18
#define DIP_PIN2 19
#define DIP_PIN3 23
DIP dip(DIP_PIN0, DIP_PIN1, DIP_PIN2, DIP_PIN3);

#include "./button/button.h"
#define BUTTON_PIN0 34
#define BUTTON_PIN1 5
#define BUTTON_PIN2 15
BUTTON button(BUTTON_PIN0, BUTTON_PIN1, BUTTON_PIN2);

const String name[16] = {
	"Center",
	"Display",
	"ArmA",
	"ArmB",
	"LEDPole",
	"Robot5",
	"Robot6",
	"Robot7",
	"Robot8",
	"Robot9",
	"Robot10",
	"Robot11",
	"Robot12",
	"Robot13",
	"Robot14",
	"Robot15"
};

void led_test(float piyo);
void led_rainbow(float piyo);

#define BCD(c) 5 * (5 * (5 * (5 * (5 * (5 * (5 * (c & 128) + (c & 64)) + (c & 32)) + (c & 16)) + (c & 8)) + (c & 4)) + (c & 2)) + (c & 1)


void setup() {
	Serial.begin(115200);
	// ser_ctrl.init();
	ctrl.begin(230400);
	
	power.init();
	dip.init();
	button.init();
	
	oled.init();
	speaker.init();
	speaker.ring(C4);

	for(int i=0; i<5; i++)rawpixel[i].begin();
	for(int i=0; i<6; i++)led[i].init();
	for(int i=0; i<4; i++)circuit_led[i].init();

	for(int n=0; n<30; n++){
		for(int i=0; i<6; i++)led[i].clear();
		for(int i=0; i<4; i++)circuit_led[i].clear();
		for(int j=0; j<6; j++){
			led[j].set_color_hsv_all(150, 250, n);
			// led[j].set_width_hsv(24.5, n, 150, 250, 30); //set_width_hsv(float center, float width, int h, int s, int v)
		}
		for(int j=0; j<4; j++){
			circuit_led[j].set_color_hsv_all(150, 250, n);
			// circuit_led[j].set_width_hsv(4.5, n/2.5, 150, 250, 30);
		}
		for(int i=0; i<6; i++)led[i].show();
		for(int i=0; i<4; i++)circuit_led[i].show();
		if(n > 25){
			speaker.mute();
		}
	}

	oled.display_version(VERSION);
	oled.show();

	speaker.boot_music();

	delay(1000);

	pinMode(36, INPUT); //SENSOR_VP
	pinMode(39, INPUT); //SENSOR_VN
}


uint32_t hoge = 0;
float piyo = -30.0;

uint8_t pre_phase = 0;
uint32_t pf_time = 0;

uint32_t loop_time = 0;

uint32_t led_timer = 0;

uint32_t A_tim = 0;
uint32_t B_tim = 0;
uint32_t C_tim = 0;


void loop(){
	if(digitalRead(39)){
		for(int i=0; i<4; i++) circuit_led[i].clear();
		for(int i=0; i<6; i++) led[i].clear();
	
		for(int i=0; i<4; i++) circuit_led[i].set_color_rgb_all(100, 100, 100);
		for(int i=0; i<6; i++) led[i].set_color_rgb_all(100, 100, 100);

		for(int i=0; i<4; i++) circuit_led[i].show();
		for(int i=0; i<6; i++) led[i].show();
	}

	// Serial.print("loop_time: ");
	// Serial.print(micros()-loop_time);
	// Serial.print("  ");
	loop_time = micros();
	hoge++;
	piyo += 0.01;
	ser_ctrl.read();
	
	bool btn_val[3] = {0, 0, 0};
	button.read(btn_val);
	
	//ーーーーーーーーーー表示ーーーーーーーーーー
	// printf(" data0: %#X data1: %#X\n", ser_ctrl.data[0], ser_ctrl.data[1]);
	if(ctrl.available()){
		int hoge = ctrl.read();
		if(hoge!=250) printf(" data0: %d \n", hoge);
	}
	// oled.clear();
	// oled.display_title("Pole"+ String(dip.read_ID()) +" V" + String(VERSION));
	// oled.display_battary(power.voltage(), power.percentage());

	// char StrID[5];
	// sprintf(StrID, "%02X%02X ", ser_ctrl.data[1], ser_ctrl.data[0]);

	// oled.half_display_num(
	// 	"ID = "+String(StrID) + "  D2 = "+ String(ser_ctrl.data[2]),
	// 	"D3 = "+String(ser_ctrl.data[3]) + "  D4 = "+String(ser_ctrl.data[4])
	// );
	// oled.half_display_3button(btn_val);
	// oled.show();

}



