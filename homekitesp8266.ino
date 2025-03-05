/*
 * switch.ino
 *
 *  Created on: 2020-05-15
 *      Author: Mixiaoxiao (Wang Bin)
 *
 * HAP section 8.38 Switch
 * An accessory contains a switch.
 *
 * This example shows how to:
 * 1. define a switch accessory and its characteristics (in my_accessory.c).
 * 2. get the switch-event sent from iOS Home APP.
 * 3. report the switch value to HomeKit.
 *
 * You should:
 * 1. read and use the Example01_TemperatureSensor with detailed comments
 *    to know the basic concept and usage of this library before other examples。
 * 2. erase the full flash or call homekit_storage_reset() in setup()
 *    to remove the previous HomeKit pairing storage and
 *    enable the pairing with the new accessory of this new HomeKit example.
 */

#include <arduino_homekit_server.h>
#include "wifi_info.h"

#define LOG_D(fmt, ...)   printf_P(PSTR(fmt "\n") , ##__VA_ARGS__);

void setup() {
	Serial.begin(115200);
	wifi_connect(); // in wifi_info.h
	// homekit_storage_reset(); // to remove the previous HomeKit pairing storage when you first run this new HomeKit example
	my_homekit_setup();
}

void loop() {
	my_homekit_loop();
	delay(10);
}

//==============================
// HomeKit setup and loop
//==============================

// access your HomeKit characteristics defined in my_accessory.c
extern "C" homekit_server_config_t config;
extern "C" homekit_characteristic_t cha_switch_on1;
extern "C" homekit_characteristic_t cha_switch_on2;

static uint32_t next_heap_millis = 0;

#define PIN_SWITCH 2 //D4
#define PIN_SWITCH2 16 //D0

#define BUTTON1 4 //D2
#define BUTTON2 5 //D1

bool lastButton1State = HIGH;
bool lastButton2State = HIGH;
unsigned long lastDebounceTime1 = 0;
unsigned long lastDebounceTime2 = 0;
unsigned long debounceDelay = 50;

void handleButton(
	uint8_t pin, 
	bool &lastButtonState, 
	unsigned long &lastDebounceTime,
	homekit_characteristic_t &cha_switch,
	const char* lampName
) {
	bool readButtonState = digitalRead(pin);

	if(readButtonState != lastButtonState) {
		lastDebounceTime = millis();
	}

	if ((millis() - lastDebounceTime) > debounceDelay) {
        if (readButtonState == LOW) {  // Button is pressed (assuming pull-up)
            // Toggle the switch state
            bool new_state = !cha_switch.value.bool_value;
            cha_switch.value.bool_value = new_state;
            
            // Update the physical switch
            uint8_t relay_pin = (lampName[4] == '1') ? PIN_SWITCH : PIN_SWITCH2;
            digitalWrite(relay_pin, new_state ? LOW : HIGH);
            
            // Notify HomeKit of the state change
            homekit_characteristic_notify(&cha_switch, cha_switch.value);
            LOG_D("%s: %s (by button)", lampName, new_state ? "ON" : "OFF");
        }
    }

	lastButtonState = readButtonState;
}

//Called when the switch value is changed by iOS Home APP
void cha_switch_on_setter1(const homekit_value_t value) {
	bool on = value.bool_value;
	cha_switch_on1.value.bool_value = on;	//sync the value
	LOG_D("Lamp1: %s", on ? "ON" : "OFF");
  digitalWrite(PIN_SWITCH, on ? LOW : HIGH);
}

void cha_switch_on_setter2(const homekit_value_t value) {
	bool on = value.bool_value;
	cha_switch_on2.value.bool_value = on;	//sync the value
	LOG_D("Lamp2: %s", on ? "ON" : "OFF");
  digitalWrite(PIN_SWITCH2, on ? LOW : HIGH);
}

void my_homekit_setup() {
	pinMode(PIN_SWITCH, OUTPUT);
	digitalWrite(PIN_SWITCH, HIGH);

  pinMode(PIN_SWITCH2, OUTPUT);
	digitalWrite(PIN_SWITCH2, HIGH);

	pinMode(BUTTON1, INPUT_PULLUP);
	pinMode(BUTTON2, INPUT_PULLUP);

	//Add the .setter function to get the switch-event sent from iOS Home APP.
	//The .setter should be added before arduino_homekit_setup.
	//HomeKit sever uses the .setter_ex internally, see homekit_accessories_init function.
	//Maybe this is a legacy design issue in the original esp-homekit library,
	//and I have no reason to modify this "feature".
	cha_switch_on1.setter = cha_switch_on_setter1;
  	cha_switch_on2.setter = cha_switch_on_setter2;
	arduino_homekit_setup(&config);

	//report the switch value to HomeKit if it is changed (e.g. by a physical button)
	//bool switch_is_on = true/false;
	//cha_switch_on.value.bool_value = switch_is_on;
	//homekit_characteristic_notify(&cha_switch_on, cha_switch_on.value);
}

void my_homekit_loop() {
	arduino_homekit_loop();

	handleButton(BUTTON1, lastButton1State, lastDebounceTime1, cha_switch_on1, "Lamp1");
	handleButton(BUTTON2, lastButton2State, lastDebounceTime2, cha_switch_on2, "Lamp2");
	
	const uint32_t t = millis();
	if (t > next_heap_millis) {
		// show heap info every 5 seconds
		next_heap_millis = t + 5 * 1000;
		LOG_D("Free heap: %d, HomeKit clients: %d",
				ESP.getFreeHeap(), arduino_homekit_connected_clients_count());

	}
}
