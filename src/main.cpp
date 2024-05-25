/*
  Blink

  Turns an LED on for one second, then off for one second, repeatedly.

  Most Arduinos have an on-board LED you can control. On the UNO, MEGA and ZERO
  it is attached to digital pin 13, on MKR1000 on pin 6. LED_BUILTIN is set to
  the correct LED pin independent of which board is used.
  If you want to know what pin the on-board LED is connected to on your Arduino
  model, check the Technical Specs of your board at:
  https://www.arduino.cc/en/Main/Products

  modified 8 May 2014
  by Scott Fitzgerald
  modified 2 Sep 2016
  by Arturo Guadalupi
  modified 8 Sep 2016
  by Colby Newman

  This example code is in the public domain.

  https://www.arduino.cc/en/Tutorial/BuiltInExamples/Blink
*/

#include <Arduino.h>

#include <SoftwareSerial.h>

SoftwareSerial rxSerial(10, 11); // RX, TX

// the setup function runs once when you press reset or power the board
void setup() {
  // initialize digital pin LED_BUILTIN as an output.
    pinMode(LED_BUILTIN, OUTPUT);
    rxSerial.begin(19200);
    Serial.begin(115200);
}

// CRSF protocol state machine
enum crsf_state {
	CRSF_STATE_WAIT_SYNC,
	CRSF_STATE_WAIT_LENGTH,
	CRSF_STATE_PAYLOAD,
	// CRFS_STATE_END
};

enum crsf_state crsf_state = CRSF_STATE_WAIT_SYNC;

typedef struct __attribute__((packed)) crsf_frame {
	uint8_t type;
	unsigned ch0: 11;
	unsigned ch1: 11;
	unsigned ch2: 11;
	unsigned ch3: 11;
	unsigned ch4: 11;
	unsigned ch5: 11;
	unsigned ch6: 11;
	unsigned ch7: 11;
	unsigned ch8: 11;
	unsigned ch9: 11;
	unsigned ch10: 11;
	unsigned ch11: 11;
	unsigned ch12: 11;
	unsigned ch13: 11;
	unsigned ch14: 11;
	unsigned ch15: 11;
	uint8_t crc;
} crsf_frame_t;

uint8_t crsf_crc8(const uint8_t *ptr, uint8_t len){
	static const uint8_t crsf_crc8tab[256] = {
		0x00, 0xD5, 0x7F, 0xAA, 0xFE, 0x2B, 0x81, 0x54, 0x29, 0xFC, 0x56, 0x83, 0xD7, 0x02, 0xA8, 0x7D,
		0x52, 0x87, 0x2D, 0xF8, 0xAC, 0x79, 0xD3, 0x06, 0x7B, 0xAE, 0x04, 0xD1, 0x85, 0x50, 0xFA, 0x2F,
		0xA4, 0x71, 0xDB, 0x0E, 0x5A, 0x8F, 0x25, 0xF0, 0x8D, 0x58, 0xF2, 0x27, 0x73, 0xA6, 0x0C, 0xD9,
		0xF6, 0x23, 0x89, 0x5C, 0x08, 0xDD, 0x77, 0xA2, 0xDF, 0x0A, 0xA0, 0x75, 0x21, 0xF4, 0x5E, 0x8B,
		0x9D, 0x48, 0xE2, 0x37, 0x63, 0xB6, 0x1C, 0xC9, 0xB4, 0x61, 0xCB, 0x1E, 0x4A, 0x9F, 0x35, 0xE0,
		0xCF, 0x1A, 0xB0, 0x65, 0x31, 0xE4, 0x4E, 0x9B, 0xE6, 0x33, 0x99, 0x4C, 0x18, 0xCD, 0x67, 0xB2,
		0x39, 0xEC, 0x46, 0x93, 0xC7, 0x12, 0xB8, 0x6D, 0x10, 0xC5, 0x6F, 0xBA, 0xEE, 0x3B, 0x91, 0x44,
		0x6B, 0xBE, 0x14, 0xC1, 0x95, 0x40, 0xEA, 0x3F, 0x42, 0x97, 0x3D, 0xE8, 0xBC, 0x69, 0xC3, 0x16,
		0xEF, 0x3A, 0x90, 0x45, 0x11, 0xC4, 0x6E, 0xBB, 0xC6, 0x13, 0xB9, 0x6C, 0x38, 0xED, 0x47, 0x92,
		0xBD, 0x68, 0xC2, 0x17, 0x43, 0x96, 0x3C, 0xE9, 0x94, 0x41, 0xEB, 0x3E, 0x6A, 0xBF, 0x15, 0xC0,
		0x4B, 0x9E, 0x34, 0xE1, 0xB5, 0x60, 0xCA, 0x1F, 0x62, 0xB7, 0x1D, 0xC8, 0x9C, 0x49, 0xE3, 0x36,
		0x19, 0xCC, 0x66, 0xB3, 0xE7, 0x32, 0x98, 0x4D, 0x30, 0xE5, 0x4F, 0x9A, 0xCE, 0x1B, 0xB1, 0x64,
		0x72, 0xA7, 0x0D, 0xD8, 0x8C, 0x59, 0xF3, 0x26, 0x5B, 0x8E, 0x24, 0xF1, 0xA5, 0x70, 0xDA, 0x0F,
		0x20, 0xF5, 0x5F, 0x8A, 0xDE, 0x0B, 0xA1, 0x74, 0x09, 0xDC, 0x76, 0xA3, 0xF7, 0x22, 0x88, 0x5D,
		0xD6, 0x03, 0xA9, 0x7C, 0x28, 0xFD, 0x57, 0x82, 0xFF, 0x2A, 0x80, 0x55, 0x01, 0xD4, 0x7E, 0xAB,
		0x84, 0x51, 0xFB, 0x2E, 0x7A, 0xAF, 0x05, 0xD0, 0xAD, 0x78, 0xD2, 0x07, 0x53, 0x86, 0x2C, 0xF9};

	uint8_t crc = 0;
	for (uint8_t i = 0; i < len; i++) {
		crc = crsf_crc8tab[crc ^ *ptr++];
	}
	return crc;
}

bool ledState = false;

volatile int channel[16] = {0};
volatile bool crsf_frame_ready = false;

void crsf_parse_payload(uint8_t *payload, uint8_t len) {
	if(len != 0x18) {
		// LOG_ERR("CRSF Frame Length: %d", len);
		return;
	}
	if (payload[0] != 0x16) {		// Type: RC Channels
		// LOG_ERR("CRSF Frame Type: 0x%02X", payload[0]);
		return;
	}

	crsf_frame_t *frame = (crsf_frame_t *)payload;

	channel[0] = frame->ch0;
	channel[1] = frame->ch1;
	channel[2] = frame->ch2;
	channel[3] = frame->ch3;
	channel[4] = frame->ch4;
	channel[5] = frame->ch5;
	channel[6] = frame->ch6;
	channel[7] = frame->ch7;
	channel[8] = frame->ch8;
	channel[9] = frame->ch9;
	channel[10] = frame->ch10;
	channel[11] = frame->ch11;
	channel[12] = frame->ch12;
	channel[13] = frame->ch13;
	channel[14] = frame->ch14;
	channel[15] = frame->ch15;

	crsf_frame_ready = true;

    if(channel[2] > 1500) {
        // toggle LED
        ledState = !ledState;
    }
        digitalWrite(LED_BUILTIN, ledState);
        Serial.println(channel[2]);
}

void rxParseByte(uint8_t c) {
    // static crsf_frame_t frame;
    // static uint8_t *ptr = (uint8_t *)&frame;
    static uint8_t len = 0;
    // static uint8_t crc = 0;
    static unsigned int crsf_payload_pos;
    static uint8_t crsf_payload[32];

    switch (crsf_state) {
    case CRSF_STATE_WAIT_SYNC:
        if (c == 0xC8) {
            crsf_state = CRSF_STATE_WAIT_LENGTH;
            // len = 0;
            // crc = 0;
        }
        break;
    case CRSF_STATE_WAIT_LENGTH:
        len = c;
        if(len > 32) {
            crsf_state = CRSF_STATE_WAIT_SYNC;
            break;
        }
        crsf_payload_pos = 0;
        crsf_state = CRSF_STATE_PAYLOAD;
        break;
    case CRSF_STATE_PAYLOAD:
        // *ptr++ = byte;
        // crc = crsf_crc8(&byte, 1);
        // if (--len == 0) {
        //     crsf_state = CRSF_STATE_WAIT_SYNC;
        //     if (crc == 0) {
        //         // valid frame
        //         Serial.write((uint8_t *)&frame, sizeof(frame));
        //     }
        // }
        crsf_payload[crsf_payload_pos++] = c;
        if (crsf_payload_pos == len) {
            crsf_state = CRSF_STATE_WAIT_SYNC;
            crsf_parse_payload(crsf_payload, len);
        }

        break;
    }
}

// the loop function runs over and over again forever
void loop() {
//   digitalWrite(LED_BUILTIN, HIGH);  // turn the LED on (HIGH is the voltage level)
//   delay(1000);                      // wait for a second
//   digitalWrite(LED_BUILTIN, LOW);   // turn the LED off by making the voltage LOW
//   delay(100);                      // wait for a second
    
    if (rxSerial.available()) {
        //  Serial.write(mySerial.read());
        // Read one char
        char c = rxSerial.read();
        // Parse byte
        rxParseByte(c);

        // // toggle LED
        // ledState = !ledState;
        // digitalWrite(LED_BUILTIN, ledState);
    }
    //   if (Serial.available()) {
    //  mySerial.write(Serial.read());
    //   }
}