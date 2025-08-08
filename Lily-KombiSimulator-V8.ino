#include "driver/twai.h"
#include "driver/gpio.h"
#include <FastLED.h>

// RS485 and CAN Boost power supply
#define ME2107_EN 16 

// CAN-interface
#define CAN_TX GPIO_NUM_27
#define CAN_RX GPIO_NUM_26
#define CAN_SPEED_MODE 23

// led
#define LED_PIN     4        // IO04 for onboard LED
#define NUM_LEDS    1        // Only one LED
#define BRIGHTNESS  20       // 0 - 255
#define LED_TYPE    WS2812
#define COLOR_ORDER GRB

CRGB leds[NUM_LEDS];

// various timers
static uint32_t t100ms_last = 0; 
static uint32_t t200ms_last = 0; 
static uint32_t t1000ms_last = 0; 
static uint32_t t5000ms_last = 0;

static uint8_t ign_counter = 0;
static uint8_t sp_counter = 0;
static uint8_t er_counter = 0;
static uint8_t sos_counter = 0;
static uint8_t tp_counter = 0;
static uint8_t cc_counter = 0;
static uint8_t dtd_counter = 0;

bool HighBeam = false;

byte value = 0x00;
byte onoff = 0x28;
byte ignition = 1;


int button_counter = 0;

int Fuel = 1000;
unsigned int RealRev = 70000; //engine speed in 0.1 rpm 
unsigned int RealSpeed = 0; //car speed in 0.1 km/h
const unsigned int MaxSpeed = 3000;

int KombiRev = 0; // engine speed as being sent to kombi (578 equals 6000 rpm)
int KombiSpeed = 0; // card speed as bein sent to kombi (6350 equals 100 km/h)

uint16_t Distance = 0; // we don't really work with the distance, could be a constant here

//speed-time-table (desired speed at x secs from start)
//time in seconds
//speed in km/h
static const uint16_t SpeedTime[4][2] = {
	{0, 0},
	{10000, 2700},
	{43000, 1000},
	{55000, 0}
};
int SpeedTime_counter = 0;
int SpeedTime_max = 5;
unsigned int DesiredSpeed = SpeedTime[0][0];

// we don't listen to frames, but if you would...
/*
void Twai_Receive_Message(twai_message_t &message) {
  //Serial.printf("ID: 0x%X\n", message.identifier);
  if (!(message.rtr)) {
   // if (message.identifier == 0x1A1) { // speed
     Serial.println("Received a frame...");
   // }
  }
}
*/

////////////////
//  Specials  //
////////////////

// dit zijn de specials die we tijdens het uitvoeren een keertje afspelen

static const uint32_t X[16][11] = { 
  { 6000, 0x39E, 8, 0x0C, 0x00, 0x00, 0x02, 0x4F, 0xE9, 0x07, 0xF2}, // set time and date
	{ 6500, 0x5C0, 8, 0x40, 0x0F, 0x00, 0x29, 0xFF, 0xFF, 0xFF, 0xFF}, // lv open
	{ 6600, 0x5C0, 8, 0x40, 0x0E, 0x00, 0x29, 0xFF, 0xFF, 0xFF, 0xFF}, // rv open
	{ 7000, 0x5C0, 8, 0x40, 0x11, 0x00, 0x29, 0xFF, 0xFF, 0xFF, 0xFF}, // ra open
	{ 7100, 0x5C0, 8, 0x40, 0x10, 0x00, 0x29, 0xFF, 0xFF, 0xFF, 0xFF}, // la open
	{ 8000, 0x5C0, 8, 0x40, 0x10, 0x00, 0x28, 0xFF, 0xFF, 0xFF, 0xFF}, // la dicht
	{ 8100, 0x5C0, 8, 0x40, 0x11, 0x00, 0x28, 0xFF, 0xFF, 0xFF, 0xFF}, // ra dicht
	{ 8200, 0x5C0, 8, 0x40, 0x0e, 0x00, 0x28, 0xFF, 0xFF, 0xFF, 0xFF},  // rv dicht
	{ 8300, 0x5C0, 8, 0x40, 0x0F, 0x00, 0x28, 0xFF, 0xFF, 0xFF, 0xFF}, // lv dicht
  { 9000, 0x1F6, 2, 0x91, 0xF2, 0, 0, 0, 0, 0, 0},  // indicator left
  { 9600, 0x1F6, 2, 0x91, 0xF1, 0, 0, 0, 0, 0, 0},
  {10600, 0x1F6, 2, 0x80, 0xF0, 0, 0, 0, 0, 0, 0},
  {12000, 0x1F6, 2, 0xA1, 0xF2, 0, 0, 0, 0, 0, 0},  // indicator right
  {12600, 0x1F6, 2, 0xA1, 0xF1, 0, 0, 0, 0, 0, 0},
  {13600, 0x1F6, 2, 0x80, 0xF0, 0, 0, 0, 0, 0, 0},
  {42000, 0x5C0, 8, 0x40, 0x27, 0x00, 0x29, 0xFF, 0xFF, 0xFF, 0xFF} // engine overheated
};
int x_counter = 0;
int x_max = 16; // or automatically int x_max = sizeof(X) / sizeof(X[0]); // calculate number of rows

// calculate CRC-8 MSB-first (poly=0x1D, init=0x00, final xor 0xB1)
// Thanks to chatgpt o4-mini-high for writing this function with just a dump as input
uint8_t calcCRC8(const uint8_t* data, size_t len, uint8_t xorout) {
  uint8_t poly = 0x1D;
  uint8_t init = 0x00; 
  uint8_t crc = init;
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t bit = 0; bit < 8; bit++) {
      if (crc & 0x80) {
        crc = (crc << 1) ^ poly;
      } else {
        crc <<= 1;
      }
    }
  }
  return crc ^ xorout;
}

uint32_t start = millis();

void setup() {
  Serial.begin(115200);
  delay(5000);
  Serial.println("Hier gaan we starten met de can-bus:");

  //CAN bus initialization  
  pinMode(ME2107_EN, OUTPUT);
  digitalWrite(ME2107_EN, HIGH);
  pinMode(CAN_SPEED_MODE, OUTPUT);   // High speed mode
  digitalWrite(CAN_SPEED_MODE, LOW);

  // Initialize configuration structures using macro initializers
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX, CAN_RX, TWAI_MODE_NORMAL); // TWAI_MODE_NORMAL, TWAI_MODE_NO_ACK or TWAI_MODE_LISTEN_ONLY
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  // Install TWAI driver
  if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
      Serial.println("Driver installed\n");
  } else {
      Serial.println("Failed to install driver\n");
      return;
  }

  // Start TWAI driver
  if (twai_start() == ESP_OK) {
      Serial.println("Driver started\n");
  } else {
      Serial.println("Failed to start driver\n");
      return;
  }

  // init leds
  FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS);
  FastLED.setBrightness(BRIGHTNESS);

  // led off
  leds[0] = CRGB::Black;
  FastLED.show();

}

void loop() {

  // Wait for the message to be received
  twai_message_t rmessage;
  if (twai_receive(&rmessage, pdMS_TO_TICKS(10)) == ESP_OK) {
    //printf("Message received\n");
    //printf("R");
  }
  /*
  // Process received message
  if (rmessage.extd) {
      printf("Message is in Extended Format\n");
  } else {
      printf("Message is in Standard Format\n");
  }
  printf("ID is %d\n", rmessage.identifier);
  if (!(rmessage.rtr)) {
      for (int i = 0; i < rmessage.data_length_code; i++) {
          printf("Data byte %d = %d\n", i, rmessage.data[i]);
      }
  }
*/

  uint32_t now = millis();

  //leds[0] = CRGB::Green;
  //FastLED.show();

  twai_message_t can_message = {};


  //////////////////
	//  100ms Frames //
	//////////////////

  if (now > t100ms_last + 100) {  // 100ms-timer (10Hz)
    t100ms_last = now;

	  // IGNITION 0x12F
    // sending keepalive-frame, original frequency is 10Hz, but a bit slower is probably also allowed
    // we called this a keepalive-frame because during initial testing, we found that the KOMBI only stays alive when sending a frame with this can-id
    // but most likely it is a frame sending the ignition-status of the car, and the KOMBI considers that info as important, and will turn off without it

    // prepare frame
		can_message.identifier = 0x12F; // can-id
		can_message.extd = 0;           // standaard 11‑bit
	  can_message.data_length_code = 8; 
		can_message.rtr = 0;            // Data vs RTR frame
		if (ignition == 0) {
			can_message.data[1] = 0x20 + ign_counter; 
			can_message.data[2] = 0x89;
			can_message.data[3] = 0x6D;
			can_message.data[7] = 0x80;
		} else {
			can_message.data[1] = 0x50 + ign_counter; 
			can_message.data[2] = 0x8A;
			can_message.data[3] = 0xDD;
			can_message.data[7] = 0x81;
		}
		can_message.data[4] = 0xD1;
    can_message.data[5] = 0x05;
		can_message.data[6] = 0x30;
		can_message.data[0] = 0; // crc-byte, but this one is not checked, so we won't waste time calculating this
    ign_counter = (ign_counter + 1) % 15;  // counter increase, repeating 15 steps, lowest is 0, highest is 14 (for some reason 15 is skipped)
    // send frame
    if (twai_transmit(&can_message, pdMS_TO_TICKS(10)) == ESP_OK) {
      //for (int i = 0; i < 8; i++) Serial.printf("%02X ", smessage.data[i]);
      //Serial.println("");
    } else {
      //Serial.println("Send failed.");
    }


    // REV COUNTER 0x0F3
    can_message.identifier = 0x0F3;
    can_message.extd = 0;
    can_message.data_length_code = 8;
    KombiRev = RealRev / 104;
    can_message.data[1] = ((KombiRev & 0x000F) << 4) + ign_counter; // use same counter as ignition
    can_message.data[2] = ((KombiRev >> 4) & 0xFF);
    can_message.data[3] = 0xC0;
    can_message.data[4] = 0xFA;
    can_message.data[5] = 0xCA;
    can_message.data[6] = 0xFF;
    can_message.data[7] = 0xFF;
    can_message.data[0] = calcCRC8(&can_message.data[1], 7, 0x8F);
    if (twai_transmit(&can_message, pdMS_TO_TICKS(10)) == ESP_OK) {
    }

    
	
    // NO STEERING ERROR 0x2A7
    // i did not examine the meaning of this id, but by sending this, the appearance of a steering error is avoided
    can_message.identifier = 0x2A7; // no steering error
    can_message.extd = 0;
    can_message.data_length_code = 5;
    can_message.data[1] = 0xF0 +ign_counter;
    can_message.data[2] = 0xFE;
    can_message.data[3] = 0xFF;
    can_message.data[4] = 0x14;
    can_message.data[0] = calcCRC8(&can_message.data[1], 4, 0x38);
    if (twai_transmit(&can_message, pdMS_TO_TICKS(10)) == ESP_OK) {
    }
   

    // TRIP COUNTER 0x2BB
    // this message tells the kombi the movement of the car. because this is used to determine the range, an error will occur when this frame is missing
    //prepare frame
    can_message.identifier = 0x2BB;
    can_message.extd = 0;
    can_message.data_length_code = 5;
    can_message.data[1] = 0xF0 + tp_counter;
    can_message.data[2] = lowByte(Distance); 
    can_message.data[3] = highByte(Distance);
    can_message.data[4] = 0xF2;
    can_message.data[0] = calcCRC8(&can_message.data[1], 4, 0x78);
    if (twai_transmit(&can_message, pdMS_TO_TICKS(10)) == ESP_OK) {
      //for (int i = 0; i < 5; i++) Serial.printf("%02X ", tp_message.data[i]);
      //Serial.println("");
    }
    if (tp_counter == 0) {  // repeat sequence 0, A, 5
      tp_counter = 0xA;
    } else if (tp_counter == 10) {
      tp_counter = 0x5;
    } else {
      tp_counter = 0x0;
    }


  } // end 100ms frames


  /////////////////
	//  200ms Frames //
	/////////////////

  if (now > t200ms_last + 200) {  // 200ms-timer (5Hz)
    t200ms_last = now;
 
    // SPEED 0x1A1
    //prepare frame
    can_message.identifier = 0x1A1; 
    can_message.extd = 0;
    can_message.data_length_code = 5;
    can_message.data[1] = 0xC0 + sp_counter;
    KombiSpeed = RealSpeed * 6.37;
    can_message.data[2] = lowByte(KombiSpeed); 
    can_message.data[3] = highByte(KombiSpeed);
    can_message.data[4] = 0x91;
    can_message.data[0] = 0x0;
    sp_counter = sp_counter + 2;  // a bit of a strange counter, repeating 0,2,4,6,8,10,12,14,1,3,5,7,9,11,13 (total 15 steps)
    if (sp_counter == 16) {sp_counter = 1;}
    if (sp_counter == 15) {sp_counter = 0;}
    // send frame
    if (twai_transmit(&can_message, pdMS_TO_TICKS(10)) == ESP_OK) {
      //for (int i = 0; i < 5; i++) Serial.printf("%02X ", sp_message.data[i]);
      //Serial.printf("%02X ", sp_message.data[2]);
      //Serial.printf("%02X\n", sp_message.data[3]);
      //Serial.printf("Counter: %d", sp_counter);
      //Serial.println("");
    }
    //if (Speed < 18000) {Speed = Speed + 150;}


    // NO RESTRAINT ERROR 0x0D7
    can_message.identifier = 0x0D7;
    can_message.extd = 0;
    can_message.data_length_code = 2;
    can_message.data[0] = er_counter;
    can_message.data[1] = 0xFF;
    // send frame
    if (twai_transmit(&can_message, pdMS_TO_TICKS(10)) == ESP_OK) {
    }
    er_counter++; 
    if (er_counter == 0xFF) {er_counter = 0x00;}


    // FUEL LEVEL SENSORS 0x349
    can_message.identifier = 0x349;
    can_message.extd = 0;
    can_message.data_length_code = 5;
    can_message.data[0] = lowByte(Fuel);
    can_message.data[1] = highByte(Fuel);
    can_message.data[2] = lowByte(Fuel);
    can_message.data[3] = highByte(Fuel);
    can_message.data[4] = 0x00;
    if (twai_transmit(&can_message, pdMS_TO_TICKS(10)) == ESP_OK) {
    }
    //if (Fuel < 10000) {Fuel = Fuel + 50;}




    // check if we want a different speed
    if (SpeedTime_counter < SpeedTime_max) {
      if (now - start > SpeedTime[SpeedTime_counter + 1][0]) {
        SpeedTime_counter++;
        DesiredSpeed = SpeedTime[SpeedTime_counter][1];
        Serial.printf("Desired speed changed to %d\n", DesiredSpeed);
      }
    }
    const int Acceleration = 40; // is ongeveer 20
    //if (RealSpeed < DesiredSpeed && RealSpeed > 0) {
    if (RealSpeed < DesiredSpeed ) {
      //Serial.println("De snelheid is te laag!");
      RealSpeed = RealSpeed + Acceleration - Acceleration * RealSpeed / MaxSpeed; // av = a0(1-v/vmax)
    } else if (RealSpeed > DesiredSpeed) { // slowing down
        unsigned int step = 50;
        unsigned int diff = RealSpeed - DesiredSpeed;
        if (step > diff) {
          step = diff;
        }
        RealSpeed -= step;
      //Serial.println("De snelheid is te hoog!");
    }
    //RealSpeed = 789;
    // now calculate engine speed (Rev) at given vehicle speed (Speed)
    //const int EndRatio = 27;
    // first, we divide the speed by the endratio and then we will multiply by the gearratio.
    //uint32_t x = RealSpeed / EndRatio;
    if (RealSpeed >= 2400) {        // Gear = 7 (809:47.7)
      RealRev = RealSpeed * 17.0; 
    } else if (RealSpeed >= 1970) { // Gear = 6 (1000:47.7)
      RealRev = RealSpeed * 21.0; 
    } else if (RealSpeed >= 1650) { // Gear = 5 (1221:47.7)
      RealRev = RealSpeed * 25.6; 
    } else if (RealSpeed >= 1230) { // Gear = 4 (1457:47.7)
      RealRev = RealSpeed * 30.5; 
    } else if (RealSpeed >= 790) {  // Gear = 3 (1950:47.7)
      RealRev = RealSpeed * 40.9; 
    } else if (RealSpeed >= 460) {  // Gear = 2 (3029:47.7)
      RealRev = RealSpeed * 63.5; 
    } else {                        // Gear = 1 (5250:47.7)
      RealRev = RealSpeed * 110;  
    }
    Serial.printf("RealSpeed %d\n", RealSpeed);

    //TEST CC&LIM 0x289
    unsigned int CCSpeed = DesiredSpeed / 10;
    can_message.identifier = 0x289;  // Cruise Control & Limiter
    can_message.extd = 0;
    can_message.data_length_code = 8;
    can_message.data[1] = 0xF0 + cc_counter;
    can_message.data[2] = 0x22; // 0xE0 up or down button pressed? no action-0xE0, press=0x26 followed by 0x22 while pressed
    can_message.data[3] = 0xFF; // fixed
    if (DesiredSpeed == 1000) {
      can_message.data[4] = 0xFE; // 0xFE=active cc and 0xFD when not active?
      can_message.data[5] = 0x80; //80=cc, 88=limiter?
    } else {
      can_message.data[4] = 0xFD; // 0xFE=active cc and 0xFD when not active?
      can_message.data[5] = 0x00; //80=cc, 88=limiter?
    }
    //can_message.data[6] = 0x0C + ((CCSpeed & 0x000F) << 4) ; // bits 0-3: cc=0xC, limiter=0xE, bits 7-4 LSB selected speed
    can_message.data[6] = 0x0E + ((CCSpeed & 0x000F) << 4) ; // bits 0-3: cc=0xC, limiter=0xE, bits 7-4 LSB selected speed
    can_message.data[7] = ((CCSpeed >> 4) & 0xFF); // bits 3-0 MSB selected speed, rest =0
    can_message.data[0] = calcCRC8(&can_message.data[1], 7, 0x77); // mandatory (frames with incorrect checksum will be ignored here)
    if (twai_transmit(&can_message, pdMS_TO_TICKS(10)) == ESP_OK) {
      //Serial.print("CC: ");
      //for (int i = 0; i < 8; i++) Serial.printf("%02X ", can_message.data[i]);
      //Serial.println("");
    }
    cc_counter = (cc_counter + 1) % 15; 

  } //end 200ms frames


  ////////////////////
	//  1000ms Frames //
	////////////////////

  if (now > t1000ms_last + 1000) {  // 1000ms-timer (1Hz)
    t1000ms_last = now;
  
    // DRIVE TRAIN DATA 0x3F9
    // the 'drive train 2 data', send something to convince the kombi the engine is really running,
    // i just picked a random frame from a dump
    can_message.identifier = 0x3F9;  // Drive Train Data 2
    can_message.extd = 0;
    can_message.data_length_code = 8;
    can_message.data[1] = 0xF0 + dtd_counter;
    can_message.data[2] = 0x82;
    can_message.data[3] = 0x0E;
    can_message.data[4] = 0x74;
    can_message.data[5] = 0x79;
    can_message.data[6] = 0x0A;
    can_message.data[7] = 0x82;
    can_message.data[0] = 0;
    if (twai_transmit(&can_message, pdMS_TO_TICKS(10)) == ESP_OK) {
      //Serial.print("DTD: ");
      //for (int i = 0; i < 8; i++) Serial.printf("%02X ", dtd_message.data[i]);
      //Serial.println("");
    }
    dtd_counter = (dtd_counter + 1) % 15;


    // SEAT BELT ON 0x297
    can_message.identifier = 0x297;
    can_message.extd = 0;
    can_message.data_length_code = 7;
    can_message.data[1] = 0xE0 + dtd_counter;
    can_message.data[2] = 0xF1; 
    can_message.data[3] = 0xE0; // unused
    can_message.data[4] = 0xF1; // seat belt left-rear (0xF0 or 0xF1) 
    can_message.data[5] = 0x91; // seat belt mid-read (0x90 or 0x91)
    can_message.data[6] = 0xE9; // seat belt right-rear (0xE8 or 0xE9)
    can_message.data[0] = calcCRC8(&can_message.data[1], 6, 0x21); 
    if (twai_transmit(&can_message, pdMS_TO_TICKS(10)) == ESP_OK) {
    }


    // HIGH BEAM 0x21A
    // we slowly flash the high beam indicator to indicate the program is still working
    can_message.identifier = 0x21A;
    can_message.extd = 0;
    can_message.data_length_code = 3;
    can_message.data[0] = 0x24 + HighBeam + HighBeam; // send out high beam off: 0x24 or high beam on: 0x26
    can_message.data[1] = 0x3E;
    can_message.data[2] = 0xF7;
    if (twai_transmit(&can_message, pdMS_TO_TICKS(10)) == ESP_OK) {
    }
    HighBeam = not HighBeam;
  
    // NO BRAKE ERROR 0x36E
    can_message.identifier = 0x36E; //did not examine this id, but not sending this results in 'Brake system. Drive moderately'
    can_message.extd = 0;
    can_message.data_length_code = 5;
    can_message.data[1] = 0xF0 + ign_counter;
    can_message.data[2] = 0xFE;
    can_message.data[3] = 0xFF;
    can_message.data[4] = 0x14;
    can_message.data[0] = calcCRC8(&can_message.data[1], 4, 0x7E);
    if (twai_transmit(&can_message, pdMS_TO_TICKS(10)) == ESP_OK) {
    }


  } // end 1000ms frames


  ////////////////////
	//  5000ms Frames //
	////////////////////

  if (now > t5000ms_last + 5000) {  // 5000ms-timer (0.2Hz)
    t5000ms_last = now;
  
    // NO SOS CALL SYSTEM ERROR
    can_message.identifier = 0x2C3;
    can_message.extd = 0;
    can_message.data_length_code = 8;
    can_message.data[0] = 0x00 + sos_counter;
    can_message.data[1] = 0x15;
    can_message.data[2] = 0x0F; 
    can_message.data[3] = 0x00;
    can_message.data[4] = 0xFF;
    can_message.data[5] = 0x70;
    can_message.data[6] = 0xFF;
    can_message.data[7] = 0xFF;
    if (twai_transmit(&can_message, pdMS_TO_TICKS(10)) == ESP_OK) {
    }
    sos_counter = (sos_counter + 1) % 15;

  } // end 5000ms frames


  if (x_counter < x_max) {
    // is het tijd voor een special?
    if (now - start >  X[x_counter][0]) {
      //prepare frame
      twai_message_t x_message = {};
      x_message.identifier = X[x_counter][1];
      x_message.extd = 0;
      x_message.data_length_code = X[x_counter][2];
      for (int i=0; i < X[x_counter][2]; i++) x_message.data[i] = X[x_counter][i+3];
      if (twai_transmit(&x_message, pdMS_TO_TICKS(10)) == ESP_OK) {
        //for (int i = 0; i < X[x_counter][2]; i++) Serial.printf("%02X ", x_message.data[i]);
        //Serial.printf("---%d\n",ti_sequence);
      }
      x_counter++;
    }
  }

}
