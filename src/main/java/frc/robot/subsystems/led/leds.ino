#include <Adafruit_NeoPixel.h>

#ifdef __AVR__
#include <avr/power.h>
#endif

#define LED_PIN 3
#define LED_COUNT 24

Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

// ---------------- ENUMS ----------------
enum LightState {
	DEFAULT_GREEN,
	VISION_AIM_LOCKED,
	VISION_AIM_NOT_LOCKED,
	TURRET_MANUAL,

	HUB_SHIFT,
	ENDGAME_SHIFT,

	STATE_COUNT
};

enum LightPatternType {
	SOLID,
	FLASH_FAST,
	FLASH_SLOW,
	FLASH_BRIGHT_DARK
};

struct LightConfig {
	LightPatternType pattern;
	uint32_t color;
};

// Stores the pattern/color for every state defined in LightState enum
LightConfig configs[STATE_COUNT];

// Current State
LightState state = DEFAULT_GREEN;

unsigned long lastUpdate = 0;
bool ledOn = true;

// HELPER
void setAll(uint32_t color) {
	for (int i = 0; i < strip.numPixels(); i++) {
		strip.setPixelColor(i, color);
	}
	strip.show();
}

// PATTERNS
void DoPattern() {
	LightConfig cfg = configs[state];
	unsigned long now = millis();

	switch (cfg.pattern) {
		case SOLID:
			setAll(cfg.color);
			break;

		case FLASH_FAST:
			if (now - lastUpdate >= 100) {
				lastUpdate = now;
				ledOn = !ledOn;
				setAll(ledOn ? cfg.color : 0);
			}
			break;

		case FLASH_SLOW:
			if (now - lastUpdate >= 500) {
				lastUpdate = now;
				ledOn = !ledOn;
				setAll(ledOn ? cfg.color : 0);
			}
			break;

		case FLASH_BRIGHT_DARK:
			if (now - lastUpdate >= 100) {
				lastUpdate = now;
				ledOn = !ledOn;
				setAll(ledOn ? cfg.color : cfg.color / 2);
			}
			break;
	}
}

// ---------------- CONTROL ----------------
void setLight(int id) {
	if (id >= 0 && id < STATE_COUNT) {
		state = (LightState)id;
	}
}

// ---------------- SETUP ----------------
void setup() {
	pinMode(LED_BUILTIN, OUTPUT);
	Serial.begin(9600);

#if defined(__AVR_ATtiny85__) && (F_CPU == 16000000)
	clock_prescale_set(clock_div_1);
#endif

	strip.begin();
	strip.show();
	strip.setBrightness(50);

	// -------- CONFIGURE STATES --------

	configs[ENDGAME_SHIFT] = {
	    FLASH_BRIGHT_DARK,
	    strip.Color(0, 255, 255)  // GREEN
	};

	configs[HUB_SHIFT] = {
	    FLASH_BRIGHT_DARK,
	    strip.Color(255, 0, 255)  // GREEN
	};

	configs[DEFAULT_GREEN] = {
	    SOLID,
	    strip.Color(0, 255, 0)  // GREEN
	};

	configs[VISION_AIM_LOCKED] = {
	    FLASH_FAST,
	    strip.Color(255, 0, 0)  // RED
	};

	configs[VISION_AIM_NOT_LOCKED] = {
	    FLASH_SLOW,
	    strip.Color(0, 255, 0)  // GREEN
	};

	configs[TURRET_MANUAL] = {
	    SOLID,
	    strip.Color(255, 255, 0)  // CYAN
	};
}

void loop() {
	DoPattern();

	// Serial control
	if (Serial.available() > 0) {
		int incomingByte = Serial.read();
		setLight(incomingByte);

		Serial.print("Received: ");
		Serial.println(incomingByte);
	}
}