/*-------------------------------------------------------------------------
  Arduino library to control a wide variety of WS2811- and WS2812-based RGB
  LED devices such as Adafruit FLORA RGB Smart Pixels and NeoPixel strips.
  Currently handles 400 and 800 KHz bitstreams on 8, 12 and 16 MHz ATmega
  MCUs, with LEDs wired for various color orders.  Handles most output pins
  (possible exception with upper PORT registers on the Arduino Mega).

  Written by Phil Burgess / Paint Your Dragon for Adafruit Industries,
  contributions by PJRC, Michael Miller and other members of the open
  source community.

  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing products
  from Adafruit!

  -------------------------------------------------------------------------
  This file is part of the Adafruit NeoPixel library.

  NeoPixel is free software: you can redistribute it and/or modify
  it under the terms of the GNU Lesser General Public License as
  published by the Free Software Foundation, either version 3 of
  the License, or (at your option) any later version.

  NeoPixel is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with NeoPixel.  If not, see
  <http://www.gnu.org/licenses/>.
  -------------------------------------------------------------------------*/

#include "neo_pixel.h"

// Constructor when length, pin and type are known at compile-time:
Adafruit_NeoPixel::Adafruit_NeoPixel(uint16_t n, uint8_t p, neoPixelType t) :
  begun(false), brightness(0), pixels(NULL), endTime(0)
{
  updateType(t);
  updateLength(n);
  setPin(p);
}

// via Michael Vogt/neophob: empty constructor is used when strand length
// isn't known at compile-time; situations where program config might be
// read from internal flash memory or an SD card, or arrive via serial
// command.  If using this constructor, MUST follow up with updateType(),
// updateLength(), etc. to establish the strand type, length and pin number!
Adafruit_NeoPixel::Adafruit_NeoPixel() :
#ifdef NEO_KHZ400
  is800KHz(true),
#endif
  begun(false), numLEDs(0), numBytes(0), pin(-1), brightness(0), pixels(NULL),
  rOffset(1), gOffset(0), bOffset(2), wOffset(1), endTime(0)
{
}

Adafruit_NeoPixel::~Adafruit_NeoPixel() {
  if(pixels)   free(pixels);
  if(pin >= 0) pinMode(pin, INPUT);
}

void Adafruit_NeoPixel::begin(void) {
  if(pin >= 0) {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
  }
  begun = true;
}

void Adafruit_NeoPixel::updateLength(uint16_t n) {
  if(pixels) free(pixels); // Free existing data (if any)

  // Allocate new data -- note: ALL PIXELS ARE CLEARED
  numBytes = n * 3 ;
  if((pixels = (uint8_t *)malloc(numBytes))) {
    memset(pixels, 0, numBytes);
    numLEDs = n;
  } else {
    numLEDs = numBytes = 0;
  }
}

void Adafruit_NeoPixel::updateType(neoPixelType t) {
  rOffset = (t >> 4) & 0b11;
  gOffset = (t >> 2) & 0b11;
  bOffset =  t       & 0b11;
}

void Adafruit_NeoPixel::show(void) {
  if(!pixels) return;

  noInterrupts(); // Need 100% focus on instruction timing

  volatile uint16_t
    i   = numBytes; // Loop counter

  volatile uint8_t
   *ptr = pixels,   // Pointer to next byte
    b   = *ptr++,   // Current byte value
    hi,             // PORT w/output bit set high
    lo;             // PORT w/output bit set low

  volatile uint8_t n1, n2 = 0;  // First, next bits out
  hi = PORTB |  pinMask;
  lo = PORTB & ~pinMask;
  n1 = lo;
  if(b & 0x80) n1 = hi;

  asm volatile(
    "headB:"                  "\n\t"
    "out  %[port] , %[hi]"    "\n\t"
    "mov  %[n2]   , %[lo]"    "\n\t"
    "out  %[port] , %[n1]"    "\n\t"
    "rjmp .+0"                "\n\t"
    "sbrc %[byte] , 6"        "\n\t"
     "mov %[n2]   , %[hi]"    "\n\t"
    "out  %[port] , %[lo]"    "\n\t"
    "rjmp .+0"                "\n\t"
    "out  %[port] , %[hi]"    "\n\t"
    "mov  %[n1]   , %[lo]"    "\n\t"
    "out  %[port] , %[n2]"    "\n\t"
    "rjmp .+0"                "\n\t"
    "sbrc %[byte] , 5"        "\n\t"
    "mov  %[n1]   , %[hi]"    "\n\t"
    "out  %[port] , %[lo]"    "\n\t"
    "rjmp .+0"                "\n\t"
    "out  %[port] , %[hi]"    "\n\t"
    "mov  %[n2]   , %[lo]"    "\n\t"
    "out  %[port] , %[n1]"    "\n\t"
    "rjmp .+0"                "\n\t"
    "sbrc %[byte] , 4"        "\n\t"
    "mov  %[n2]   , %[hi]"    "\n\t"
    "out  %[port] , %[lo]"    "\n\t"
    "rjmp .+0"                "\n\t"
    "out  %[port] , %[hi]"    "\n\t"
    "mov  %[n1]   , %[lo]"    "\n\t"
    "out  %[port] , %[n2]"    "\n\t"
    "rjmp .+0"                "\n\t"
    "sbrc %[byte] , 3"        "\n\t"
    "mov  %[n1]   , %[hi]"    "\n\t"
    "out  %[port] , %[lo]"    "\n\t"
    "rjmp .+0"                "\n\t"
    "out  %[port] , %[hi]"    "\n\t"
    "mov  %[n2]   , %[lo]"    "\n\t"
    "out  %[port] , %[n1]"    "\n\t"
    "rjmp .+0"                "\n\t"
    "sbrc %[byte] , 2"        "\n\t"
    "mov  %[n2]   , %[hi]"    "\n\t"
    "out  %[port] , %[lo]"    "\n\t"
    "rjmp .+0"                "\n\t"
    "out  %[port] , %[hi]"    "\n\t"
    "mov  %[n1]   , %[lo]"    "\n\t"
    "out  %[port] , %[n2]"    "\n\t"
    "rjmp .+0"                "\n\t"
    "sbrc %[byte] , 1"        "\n\t"
    "mov  %[n1]   , %[hi]"    "\n\t"
    "out  %[port] , %[lo]"    "\n\t"
    "rjmp .+0"                "\n\t"
    "out  %[port] , %[hi]"    "\n\t"
    "mov  %[n2]   , %[lo]"    "\n\t"
    "out  %[port] , %[n1]"    "\n\t"
    "rjmp .+0"                "\n\t"
    "sbrc %[byte] , 0"        "\n\t"
    "mov  %[n2]   , %[hi]"    "\n\t"
    "out  %[port] , %[lo]"    "\n\t"
    "sbiw %[count], 1"        "\n\t"
    "out  %[port] , %[hi]"    "\n\t"
    "mov  %[n1]   , %[lo]"    "\n\t"
    "out  %[port] , %[n2]"    "\n\t"
    "ld   %[byte] , %a[ptr]+" "\n\t"
    "sbrc %[byte] , 7"        "\n\t"
    "mov  %[n1]   , %[hi]"    "\n\t"
    "out  %[port] , %[lo]"    "\n\t"
    "brne headB"              "\n"
  : [byte] "+r" (b), [n1] "+r" (n1), [n2] "+r" (n2), [count] "+w" (i)
  : [port] "I" (_SFR_IO_ADDR(PORTB)), [ptr] "e" (ptr), [hi] "r" (hi),
    [lo] "r" (lo));
  interrupts();
  endTime = micros(); // Save EOD time for latch on next call
}

// Set the output pin number
void Adafruit_NeoPixel::setPin(uint8_t p) {
  if(begun && (pin >= 0)) pinMode(pin, INPUT);
    pin = p;
    if(begun) {
      pinMode(p, OUTPUT);
      digitalWrite(p, LOW);
    }
#ifdef __AVR__
    port    = portOutputRegister(digitalPinToPort(p));
    pinMask = digitalPinToBitMask(p);
#endif
}

// Set pixel color from 'packed' 32-bit RGB color:
void Adafruit_NeoPixel::setPixelColor(uint16_t n, uint32_t c) {
  if(n < numLEDs) {
    uint8_t *p,
    r = (uint8_t)(c >> 16),
    g = (uint8_t)(c >>  8),
    b = (uint8_t)c;
    p = &pixels[n * 3];
    p[rOffset] = r;
    p[gOffset] = g;
    p[bOffset] = b;
  }
}
