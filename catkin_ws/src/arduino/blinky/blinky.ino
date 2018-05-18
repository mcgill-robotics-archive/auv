#include <FastLED.h>
#include "sin_table.h"

#define NUM_LED_PER_STRIP 25
#define NUM_LED_STRIP 7
#define LED_COUNT NUM_LED_PER_STRIP* NUM_LED_STRIP
#define LED_OUT_PIN 13
#define LED_BRIGHTNESS 50

#define TIMEOUT_PERIOD 1000
#define PHASE_STEP 30
#define FRACTION_BIT_MASK 0x7FF
#define NUM_FRACTION_BIT 11

struct CRGB leds[LED_COUNT];
uint32_t segmentPhase[NUM_LED_PER_STRIP];

long lastTime;

void setup() {
  Serial.begin(115200);
  LEDS.addLeds<WS2812B, LED_OUT_PIN, GRB>(leds, LED_COUNT);
  LEDS.setBrightness(LED_BRIGHTNESS);
  LEDS.show();
}

uint8_t mapIntensity(uint16_t amplitude) {
  uint16_t fraction = amplitude & FRACTION_BIT_MASK;
  // Serial.println(fraction);
  if (fraction > (1 << 10)) {
    return (uint8_t)(((1 << 11) - fraction) >> 2);
  } else {
    return (uint8_t)(fraction >> 2);
  }
}

//  The LEDs are organized in a snake zigzag shape
//  setLedColor map xy coordinate into correct index
//   +------------x
//   |
//   |   0 >  1 >  2 >  3 >  4
//   |                       |
//   |                       |
//   |   9 <  8 <  7 <  6 <  5
//   |   |
//   y   |
//      10 > 11 > 12 > 13 > 14
//                          |
//                          |
//      19 < 18 < 17 < 16 < 15

void setLedColor(int x, int y, CRGB color) {
  if (y % 2) {
    leds[NUM_LED_PER_STRIP * (y + 1) - x - 1] = color;
  } else {
    leds[NUM_LED_PER_STRIP * y + x] = color;
  }
}

void colorLoop() {
  // clear all led to black
  for (uint16_t i = 0; i < LED_COUNT; i++) {
    leds[i] = CRGB::Black;
  }

  for (uint8_t i = 0; i < NUM_LED_PER_STRIP; i++) {
    // Increment the phase for each led
    segmentPhase[i] = (segmentPhase[i] + i + PHASE_STEP) & 0x0FFF;

    // Determin which led to turn on, and its intensity
    uint16_t amplitude = NUM_LED_STRIP * SIN_TABLE_256[segmentPhase[i] >> 4];
    uint8_t pixelIndex = (amplitude >> NUM_FRACTION_BIT);
    // Put on the correct led
    setLedColor(i, pixelIndex, CRGB(255, 0, 0));
  }
}

void serialLoop() {
  static int pixelIndex;

  unsigned long lastReceiveTime = millis();

  while (true) {
    if (Serial.available() > 2) {
      lastReceiveTime = millis();

      uint8_t buffer[3];  // Buffer to store three incoming bytes used to
                          // compile a single LED color

      for (uint8_t x = 0; x < 3; x++) {  // Read three incoming bytes
        uint8_t c = Serial.read();

        if (c < 255) {
          buffer[x] = c;  // Using 255 as a latch semaphore
        } else {
          LEDS.show();
          pixelIndex = 0;
          break;
        }

        if (x == 2) {  // If we received three serial bytes
          if (pixelIndex == LED_COUNT) {
            break;  // Prevent overflow by ignoring the pixel data beyond
                    // LED_COUNT
          }
          leds[pixelIndex] = CRGB(buffer[0], buffer[1], buffer[2]);
          pixelIndex++;
        }
      }
    }

    // If we haven't received data in 1 second, return to playing back our
    // animation
    if (millis() > lastReceiveTime + TIMEOUT_PERIOD) {
      return;
    }
  }
}

void loop() {
  // If we get some data, switch to passthrough mode
  if (Serial.available() > 0) {
    serialLoop();
  }

  // Draw the current pattern
  colorLoop();

  LEDS.show();
  delay(1);
}
