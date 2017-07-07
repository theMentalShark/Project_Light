/***************************************************
  This is our GFX example for the Adafruit ILI9341 TFT FeatherWing
  ----> http://www.adafruit.com/products/3315

  Check out the links above for our tutorials and wiring diagrams

  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.
  MIT license, all text above must be included in any redistribution
 ****************************************************/

#include <Wire.h>      // this is needed even tho we aren't using it
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <Adafruit_STMPE610.h>

#ifdef ESP8266
#define STMPE_CS 16
#define TFT_CS   0
#define TFT_DC   15
#define SD_CS    2
#endif
#ifdef __AVR_ATmega32U4__
#define STMPE_CS 6
#define TFT_CS   9
#define TFT_DC   10
#define SD_CS    5
#endif
#ifdef ARDUINO_SAMD_FEATHER_M0
#define STMPE_CS 6
#define TFT_CS   9
#define TFT_DC   10
#define SD_CS    5
#endif
#ifdef TEENSYDUINO
#define TFT_DC   10
#define TFT_CS   4
#define STMPE_CS 3
#define SD_CS    8
#endif
#ifdef ARDUINO_STM32_FEATHER
#define TFT_DC   PB4
#define TFT_CS   PA15
#define STMPE_CS PC7
#define SD_CS    PC5
#endif
#define SAMPLE_SIZE 10

#if defined(ARDUINO_SAMD_ZERO) && defined(SERIAL_PORT_USBVIRTUAL)
// Required for Serial on Zero based boards
#define Serial SERIAL_PORT_USBVIRTUAL
#endif

Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC);
Adafruit_STMPE610 ts = Adafruit_STMPE610(STMPE_CS);

// This is calibration data for the raw touch data to the screen coordinates
#define TS_MINX 3928
#define TS_MAXX 191
#define TS_MINY 184
#define TS_MAXY 3652

int ledRes = 128;
int knobPinA = A0;
int knobPinB = A1;
int mode = 0;         //mode 0 for rgb+2, mode 1 for dual tone white;
int r, g, b, w;
int colorShift;
int targetR, targetG, targetB, targetW;
float ledChangeSmooth = 0.1;
int currentSample = 0;
int pKnobDataA, pKnobDataB;
int knobDataA[SAMPLE_SIZE], knobDataB[SAMPLE_SIZE ];
int knobValA, knobValB;
int scrR, scrG, scrB;
bool initPrc = true;
int uiW = 320;
int uiH = 180;
int px, py;
int highlightBoxSize = 3;


void setup()
{

  Serial.begin(115200);
  if (!ts.begin()) {
    Serial.println("Couldn't start touchscreen controller");
    while (1);
  }
  tft.begin();


  tft.setRotation(1);
  tft.fillScreen(ILI9341_BLACK);


  pinMode(knobPinA, INPUT);
  pinMode(knobPinB, INPUT);



  REG_GCLK_GENDIV = GCLK_GENDIV_DIV(1) |          // Divide the 48MHz clock source by divisor 1: 48MHz/1=48MHz
                    GCLK_GENDIV_ID(4);            // Select Generic Clock (GCLK) 4
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

  REG_GCLK_GENCTRL = GCLK_GENCTRL_IDC |           // Set the duty cycle to 50/50 HIGH/LOW
                     GCLK_GENCTRL_GENEN |         // Enable GCLK4
                     GCLK_GENCTRL_SRC_DFLL48M |   // Set the 48MHz clock source
                     GCLK_GENCTRL_ID(4);          // Select GCLK4
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

  // Enable the port multiplexer for the digital pin 11,13,17,18
  PORT->Group[g_APinDescription[13].ulPort].PINCFG[g_APinDescription[13].ulPin].bit.PMUXEN = 1;
  PORT->Group[g_APinDescription[11].ulPort].PINCFG[g_APinDescription[11].ulPin].bit.PMUXEN = 1;
  PORT->Group[g_APinDescription[17].ulPort].PINCFG[g_APinDescription[17].ulPin].bit.PMUXEN = 1;
  PORT->Group[g_APinDescription[18].ulPort].PINCFG[g_APinDescription[18].ulPin].bit.PMUXEN = 1;


  // Connect the TCC0 timer to digital output 11,13 and 17,18 - port pins are paired odd PMUO and even PMUXE
  // F & E specify the timers: TCC0, TCC1 and TCC2
  PORT->Group[g_APinDescription[11].ulPort].PMUX[g_APinDescription[11].ulPin >> 1].reg = PORT_PMUX_PMUXO_E | PORT_PMUX_PMUXE_E;
  PORT->Group[g_APinDescription[17].ulPort].PMUX[g_APinDescription[17].ulPin >> 1].reg = PORT_PMUX_PMUXO_E | PORT_PMUX_PMUXE_E;

  // Feed GCLK4 to TCC2 (and TC3)
  REG_GCLK_CLKCTRL = GCLK_CLKCTRL_CLKEN |         // Enable GCLK4 to TCC2 (and TC3)
                     GCLK_CLKCTRL_GEN_GCLK4 |     // Select GCLK4
                     GCLK_CLKCTRL_ID_TCC2_TC3;    // Feed GCLK4 to TCC2 (and TC3)
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

  // Feed GCLK4 to TCC1 and TCC0
  REG_GCLK_CLKCTRL = GCLK_CLKCTRL_CLKEN |         // Enable GCLK4 to TCC2 (and TC3)
                     GCLK_CLKCTRL_GEN_GCLK4 |     // Select GCLK4
                     GCLK_CLKCTRL_ID_TCC0_TCC1;    // Feed GCLK4 to TCC2 (and TC3)
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

  // Dual slope PWM operation: timers countinuously count up to PER register value then down 0
  REG_TCC2_WAVE |= TCC_WAVE_POL(0xF) |         // Reverse the output polarity on all TCC0 outputs
                   TCC_WAVE_WAVEGEN_DSBOTH;    // Setup dual slope PWM on TCC2
  while (TCC2->SYNCBUSY.bit.WAVE);               // Wait for synchronization
  // Dual slope PWM operation: timers countinuously count up to PER register value then down 0

  REG_TCC0_WAVE |= TCC_WAVE_POL(0xF) |         // Reverse the output polarity on all TCC0 outputs
                   TCC_WAVE_WAVEGEN_DSBOTH;    // Setup dual slope PWM on TCC0
  while (TCC0->SYNCBUSY.bit.WAVE);               // Wait for synchronization
  // Dual slope PWM operation: timers countinuously count up to PER register value then down 0

  // Each timer counts up to a maximum or TOP value set by the PER register,
  // this determines the frequency of the PWM operation:
  // freqnency = (Clock Frequence / Mode) / Resolution = (48Mhz / 2) / Resolution = 24Mhz / Resolution
  // Frenquency :
  // Resolution @ 2048steps = 11.71kHz
  REG_TCC2_PER = ledRes;         // Set the frequency of the PWM steps at resolution
  while (TCC2->SYNCBUSY.bit.PER);                // Wait for synchronization

  // Each timer counts up to a maximum or TOP value set by the PER register,
  // this determines the frequency of the PWM operation:
  REG_TCC0_PER = ledRes;         // Set the frequency of the PWM steps at resolution
  while (TCC0->SYNCBUSY.bit.PER);                // Wait for synchronization

  // Set the PWM signal to output 50% duty cycle
  REG_TCC2_CC1 = 0;         // TCC2 CC1 - PA17 - D13
  while (TCC2->SYNCBUSY.bit.CC1);                // Wait for synchronization
  REG_TCC2_CC0 = 0;         // TCC2 CC0 - PA16 - D11
  while (TCC2->SYNCBUSY.bit.CC0);                // Wait for synchronization
  REG_TCC0_CC0 = 0;         // TCC0 CC0 - PA04 - D17
  while (TCC0->SYNCBUSY.bit.CC0);                // Wait for synchronization
  REG_TCC0_CC1 = 0;         // TCC0 CC1 - PA05 - D18
  while (TCC0->SYNCBUSY.bit.CC1);                // Wait for synchronization

  // Divide the 48MHz signal by 1 giving 48MHz (20.83ns) TCC2 timer tick and enable the outputs
  REG_TCC2_CTRLA |= TCC_CTRLA_PRESCALER_DIV1 |    // Divide GCLK4 by 1
                    TCC_CTRLA_ENABLE;             // Enable the TCC2 output
  while (TCC2->SYNCBUSY.bit.ENABLE);              // Wait for synchronization

  // Divide the 48MHz signal by 1 giving 48MHz (20.83ns) TCC0 timer tick and enable the outputs
  REG_TCC0_CTRLA |= TCC_CTRLA_PRESCALER_DIV1 |    // Divide GCLK4 by 1
                    TCC_CTRLA_ENABLE;             // Enable the TCC0 output
  while (TCC0->SYNCBUSY.bit.ENABLE);              // Wait for synchronization

  drawRainbow(0, 0, uiW, uiH);
  initPrc = false;
}


void loop() {
  getKnobData();
  TS_Point p = ts.getPoint();


  // Scale from ~0->4000 to tft.width using the calibration #'s
  p.x = map(p.x, TS_MINX, TS_MAXX, 0, tft.height());
  p.y = map(p.y, TS_MINY, TS_MAXY, 0, tft.width());


  if (p.x > (tft.height() - uiH)) {
    //draw box
    tft.drawRect(p.y -  highlightBoxSize, tft.height() - p.x - highlightBoxSize, highlightBoxSize * 2 + 1, highlightBoxSize * 2 + 1, ILI9341_BLACK);
    delay(15);
    //clear previous box
    for (int i = -highlightBoxSize; i <= highlightBoxSize; i++) {
      for (int j = -highlightBoxSize; j <= highlightBoxSize; j++) {
        if (((i == -highlightBoxSize) || ( i == highlightBoxSize)) || ((j == -highlightBoxSize) || ( j == highlightBoxSize))) {
          int cx = px + i;
          int cy = py + j;
          if (cx > (tft.height() - uiH - 1)) {
            float s = float(tft.height() - cx) / float(uiH);
            float h = float(cy) / float(uiW) * 360;
            //      Serial.print("r:"); Serial.print(r);
            //      Serial.print("\tg:"); Serial.print(g);
            //      Serial.print("\tb:"); Serial.print(b);
            //      Serial.print("\ts:"); Serial.print(s);
            //      Serial.print("\th:"); Serial.println(h);
            setRGB(s, h, 1);
//            tft.drawPixel(cy, tft.height() - cx, tft.color565(255,255,255));
            tft.drawPixel(cy, tft.height() - cx, tft.color565(targetR / 4, targetG / 4, targetB / 4));
          }
        }
      }
    }
    //save pervious pos
    px = p.x;
    py = p.y;
  }
  float brightness = float(knobValB) / float(1023);
  if (p.x < (tft.height() - uiH) - 20) {
    Serial.println("mode change");
    if (p.y > (tft.height() / 2)) {
      changeMode(0);
    } else {
      changeMode(1);
    }
  }
  if (mode == 1) {
    int totalBrightness = knobValA;
    targetW = targetR = totalBrightness * brightness;
    targetG = targetB = totalBrightness * (1 - brightness);
    Serial.print("total brightness: ");
    Serial.print(totalBrightness);
    Serial.print("\tbrightness: ");
    Serial.println(brightness);
  } else {
    targetW = knobValA;
    float saturation = float(tft.height() - max(p.x, (tft.height() - uiH))) / float(uiH);
    float hue = float(p.y) / float(uiW) * 360;
    setRGB(saturation, hue, brightness);
  }
  setColor();
}

void changeMode (int m) {
  if (m == 1) {
    //draw black rect clear box,
    tft.fillRect(282, 202, 26, 26, tft.color565(0, 0, 0));
    //draw white rect select box,
    tft.fillRect(126, 206, 18, 18, tft.color565(255, 255, 255));
//    tft.setCursor(30, 205);
//    tft.setTextColor(ILI9341_WHITE);
//    tft.setTextSize(3);
//    tft.println("MONO");
//    tft.setCursor(175, 205);
//    tft.setTextColor(tft.color565(128, 128, 128));
//    tft.setTextSize(3);
//    tft.println("RGB+W");
//    
    tft.setCursor(15, 210);
    tft.setTextColor(ILI9341_WHITE);
    tft.setTextSize(2);
    tft.println("BI-COLOR");
    tft.setCursor(200, 210);
    tft.setTextColor(tft.color565(128, 128, 128));
    tft.setTextSize(2);
    tft.println("RGB+W");
  } else {
    //draw black rect clear box,
    tft.fillRect(122, 202, 26, 26, tft.color565(0, 0, 0));
    //draw white rect select box,
    tft.fillRect(286, 206, 18, 18, tft.color565(255, 255, 255));
//    tft.setCursor(30, 205);
//    tft.setTextColor(tft.color565(128, 128, 128));
//    tft.setTextSize(3);
//    tft.println("MONO");
//    tft.setCursor(175, 205);
//    tft.setTextColor(ILI9341_WHITE);
//    tft.setTextSize(3);
//    tft.println("RGB+W");
    
    tft.setCursor(15, 210);
    tft.setTextColor(tft.color565(128, 128, 128));
    tft.setTextSize(2);
    tft.println("BI-COLOR");
    tft.setCursor(200, 210);
    tft.setTextColor(ILI9341_WHITE);
    tft.setTextSize(2);
    tft.println("RGB+W");
  }
  mode = m;
}

float lerp(float a, float b, float x)
{
  return a + x * (b - a);
}

void changeLed () {
  //
  //    if (w < map(targetW, 0, 1024, 0, ledRes)) {
  //      w += ledChangeSpd;
  //    }else if (w > map(targetW, 0, 1024, 0, ledRes)) {
  //      w -= ledChangeSpd;
  //    }
  //
  //    if (r < map(targetR, 0, 255, 0, ledRes)) {
  //      r += ledChangeSpd;
  //    }else if (r > map(targetR, 0, 255, 0, ledRes)) {
  //      r -= ledChangeSpd;
  //    }
  //
  //    if (g < map(targetG, 0, 255, 0, ledRes)) {
  //      g += ledChangeSpd;
  //    }else if (g > map(targetG, 0, 255, 0, ledRes)) {
  //      g -= ledChangeSpd;
  //    }
  //
  //    if (b < map(targetB, 0, 255, 0, ledRes)) {
  //      b += ledChangeSpd;
  //    }else if (b > map(targetB, 0, 255, 0, ledRes)) {
  //      b -= ledChangeSpd;
  //    }
}


void getKnobData() {
  //Knob A data smooth;
  pKnobDataA -= knobDataA[currentSample];
  knobDataA[currentSample] = analogRead(knobPinA);
  pKnobDataA += knobDataA[currentSample];
  knobValA = pKnobDataA / SAMPLE_SIZE;

  //Knob B data smooth;
  pKnobDataB -= knobDataB[currentSample];
  knobDataB[currentSample] = analogRead(knobPinB);
  pKnobDataB += knobDataB[currentSample];
  knobValB = pKnobDataB / SAMPLE_SIZE;

  currentSample++;
  if (currentSample >= SAMPLE_SIZE) {
    currentSample = 0;
  }

}


void setColor () {

  w = lerp(float(w), map(targetW, 0, 1024, 0, ledRes), ledChangeSmooth);
  r = lerp(float(r), map(targetR, 0, 1024, 0, ledRes), ledChangeSmooth);
  g = lerp(float(g), map(targetG, 0, 1024, 0, ledRes), ledChangeSmooth);
  b = lerp(float(b), map(targetB, 0, 1024, 0, ledRes), ledChangeSmooth);
  Serial.print(w);
  Serial.print(" ");
  Serial.print(r);
  Serial.print(" ");
  Serial.print(g);
  Serial.print(" ");
  Serial.print(b);
  Serial.println(" ");

  // set the brightness of pin 9:
  //  analogWrite(led, brightness);
  REG_TCC2_CC1 = w;         // TCC2 CC1 - PA17 - D13
  while (TCC2->SYNCBUSY.bit.CC1);                // Wait for synchronization
  REG_TCC2_CC0 = r;         // TCC2 CC0 - PA16 - D11
  while (TCC2->SYNCBUSY.bit.CC0);                // Wait for synchronization
  REG_TCC0_CC0 = g;         // TCC0 CC0 - PA04 - D17
  while (TCC0->SYNCBUSY.bit.CC0);                // Wait for synchronization
  REG_TCC0_CC1 = b;         // TCC0 CC1 - PA05 - D18
  while (TCC0->SYNCBUSY.bit.CC1);                // Wait for synchronization
}


void setRGB(float s, float h, float v) {

  float  f, p, q, t;
  int i;
  int red = 0;
  int green = 0;
  int blue = 0;
  if (s == 0) {
    red = green = blue = 1023;
    return;
  }
  h /= 60;
  i = floor(h);
  f = h - i;
  p = v * (1 - s);
  q = v * (1 - s * f);
  t = v * (1 - s * (1 - f));
  switch (i) {
    case 0:
      red = round(1023.0 * v);
      green = round(1023.0 * t);
      blue = round(1023.0 * p);
      break;
    case 1:
      red = round(1023.0 * q);
      green = round(1023.0 * v);
      blue = round(1023.0 * p);
      break;
    case 2:
      red = round(1023.0 * p);
      green = round(1023.0 * v);
      blue = round(1023.0 * t);
      break;
    case 3:
      red = round(1023.0 * p);
      green = round(1023.0 * q);
      blue = round(1023.0 * v);
      break;
    case 4:
      red = round(1023.0 * t);
      green = round(1023.0 * p);
      blue = round(1023.0 * v);
      break;
    default:
      red = round(1023.0 * v);
      green = round(1023.0 * p);
      blue = round(1023.0 * q);
  }
  if (initPrc) {
    scrR = red / 4;
    scrG = green / 4;
    scrB = blue / 4;
  } else {
    targetR = red;
    targetG = green;
    targetB = blue;
  }
}


unsigned long drawRainbow(int x, int y, int width, int height) {
  tft.fillScreen(tft.color565(0, 0, 0));
  for (int i = 0; i <= height; i++) {
    for (int j = 0; j <= width; j++) {
      float s = float(i) / float(height);
      float h = float(j) / float(width) * 360;
      //      Serial.print("r:"); Serial.print(r);
      //      Serial.print("\tg:"); Serial.print(g);
      //      Serial.print("\tb:"); Serial.print(b);
      //      Serial.print("\ts:"); Serial.print(s);
      //      Serial.print("\th:"); Serial.println(h);
      setRGB(s, h, 1);
      tft.drawPixel(j, i, tft.color565(scrR, scrG, scrB));
      //      setColor();
      //      delay(5);
    }
  }
  tft.setCursor(15, 210);
  tft.setTextColor(tft.color565(128, 128, 128));
  tft.setTextSize(2);
  tft.println("BI-COLOR");
  tft.setCursor(200, 210);
  tft.setTextColor(ILI9341_WHITE);
  tft.setTextSize(2);
  tft.println("RGB+W");
  tft.fillRect(120, 200, 30, 30, tft.color565(255, 255, 255));
  tft.fillRect(280, 200, 30, 30, tft.color565(255, 255, 255));
  tft.fillRect(122, 202, 26, 26, tft.color565(0, 0, 0));
  tft.fillRect(282, 202, 26, 26, tft.color565(0, 0, 0));
  //  tft.fillRect(126, 206, 18, 18, tft.color565(255, 255, 255));
  tft.fillRect(286, 206, 18, 18, tft.color565(255, 255, 255));
}


