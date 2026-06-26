/*
 * TFT_graphicstest_PDQ.ino
 *
 * Spresense + ILI9341 graphics test sketch using LovyanGFX.
 *
 * This file is adapted from LovyanGFX / PDQ graphicstest examples.
 * For original licensing terms, refer to the upstream project (MIT):
 * https://github.com/lovyan03/LovyanGFX
 */

#define LGFX_USE_V1
#include "LGFX_SPRESENSE_ILI9341.hpp"
#include <LovyanGFX.hpp>

#ifdef min
#undef min
#endif
#ifdef max
#undef max
#endif
#include <algorithm>

static LGFX lcd;

unsigned long total = 0;
unsigned long tn = 0;

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("");
  Serial.println("");
  Serial.println("Lovyan's LovyanGFX library Test on Spresense!");
  
  lcd.init();
  lcd.setRotation(0);
  
  lcd.startWrite();
}

void loop(void)
{
  Serial.println(F("Benchmark               Time (microseconds)"));

  uint32_t usecHaD = testHaD();
  Serial.print(F("HaD pushColor            "));
  Serial.println(usecHaD);
  delay(100);

  uint32_t usecFillScreen = testFillScreen();
  Serial.print(F("Screen fill             "));
  Serial.println(usecFillScreen);
  delay(100);

  uint32_t usecText = testText();
  Serial.print(F("Text                    "));
  Serial.println(usecText);
  delay(100);

  uint32_t usecPixels = testPixels();
  Serial.print(F("Pixels                  "));
  Serial.println(usecPixels);
  delay(100);

  uint32_t usecLines = testLines(TFT_BLUE);
  Serial.print(F("Lines                   "));
  Serial.println(usecLines);
  delay(100);

  uint32_t usecFastLines = testFastLines(TFT_RED, TFT_BLUE);
  Serial.print(F("Horiz/Vert Lines        "));
  Serial.println(usecFastLines);
  delay(100);

  uint32_t usecRects = testRects(TFT_GREEN);
  Serial.print(F("Rectangles (outline)     "));
  Serial.println(usecRects);
  delay(100);

  uint32_t usecFilledRects = testFilledRects(TFT_YELLOW, TFT_MAGENTA);
  Serial.print(F("Rectangles (filled)     "));
  Serial.println(usecFilledRects);
  delay(100);

  uint32_t usecFilledCircles = testFilledCircles(10, TFT_MAGENTA);
  Serial.print(F("Circles (filled)        "));
  Serial.println(usecFilledCircles);
  delay(100);

  uint32_t usecCircles = testCircles(10, TFT_WHITE);
  Serial.print(F("Circles (outline)        "));
  Serial.println(usecCircles);
  delay(100);

  uint32_t usecTriangles = testTriangles();
  Serial.print(F("Triangles (outline)     "));
  Serial.println(usecTriangles);
  delay(100);

  uint32_t usecFilledTrangles = testFilledTriangles();
  Serial.print(F("Triangles (filled)      "));
  Serial.println(usecFilledTrangles);
  delay(100);

  uint32_t usecRoundRects = testRoundRects();
  Serial.print(F("Rounded rects (outline) "));
  Serial.println(usecRoundRects);
  delay(100);

  uint32_t usedFilledRoundRects = testFilledRoundRects();
  Serial.print(F("Rounded rects (filled)  "));
  Serial.println(usedFilledRoundRects);
  delay(100);

  Serial.println(F("Done!"));

  uint16_t c = 4;
  int8_t d = 1;
  for (int32_t i = 0; i < lcd.height(); i++)
  {
    lcd.drawFastHLine(0, i, lcd.width(), c);
    c += d;
    if (c <= 4 || c >= 11)
      d = -d;
  }
  
  lcd.setCursor(0, 0);
  lcd.setTextColor(TFT_MAGENTA);
  lcd.setTextSize(2);

  lcd.println(F("  LovyanGFX test"));

  lcd.setTextSize(1);
  lcd.setTextColor(TFT_WHITE);
  lcd.println(F(""));
  lcd.println(F(""));
  lcd.setTextColor(lcd.color565(0x80, 0x80, 0x80));

  lcd.println(F(""));

  lcd.setTextColor(TFT_GREEN);
  lcd.println(F("  Benchmark              microseconds"));
  lcd.println(F(""));
  lcd.setTextColor(TFT_YELLOW);

  lcd.setTextColor(TFT_CYAN); lcd.setTextSize(1);
  lcd.print(F("HaD pushColor      "));
  lcd.setTextColor(TFT_YELLOW);
  lcd.setTextSize(2);
  printnice(usecHaD);

  lcd.setTextColor(TFT_CYAN); lcd.setTextSize(1);
  lcd.print(F("Screen fill        "));
  lcd.setTextColor(TFT_YELLOW);
  lcd.setTextSize(2);
  printnice(usecFillScreen);

  lcd.setTextColor(TFT_CYAN); lcd.setTextSize(1);
  lcd.print(F("Text               "));
  lcd.setTextColor(TFT_YELLOW);
  lcd.setTextSize(2);
  printnice(usecText);

  lcd.setTextColor(TFT_CYAN); lcd.setTextSize(1);
  lcd.print(F("Pixels             "));
  lcd.setTextColor(TFT_YELLOW);
  lcd.setTextSize(2);
  printnice(usecPixels);

  lcd.setTextColor(TFT_CYAN); lcd.setTextSize(1);
  lcd.print(F("Lines              "));
  lcd.setTextColor(TFT_YELLOW);
  lcd.setTextSize(2);
  printnice(usecLines);

  lcd.setTextColor(TFT_CYAN); lcd.setTextSize(1);
  lcd.print(F("Horiz/Vert Lines   "));
  lcd.setTextColor(TFT_YELLOW);
  lcd.setTextSize(2);
  printnice(usecFastLines);

  lcd.setTextColor(TFT_CYAN); lcd.setTextSize(1);
  lcd.print(F("Rectangles         "));
  lcd.setTextColor(TFT_YELLOW);
  lcd.setTextSize(2);
  printnice(usecRects);

  lcd.setTextColor(TFT_CYAN); lcd.setTextSize(1);
  lcd.print(F("Rectangles-filled  "));
  lcd.setTextColor(TFT_YELLOW);
  lcd.setTextSize(2);
  printnice(usecFilledRects);

  lcd.setTextColor(TFT_CYAN); lcd.setTextSize(1);
  lcd.print(F("Circles            "));
  lcd.setTextColor(TFT_YELLOW);
  lcd.setTextSize(2);
  printnice(usecCircles);

  lcd.setTextColor(TFT_CYAN); lcd.setTextSize(1);
  lcd.print(F("Circles-filled     "));
  lcd.setTextColor(TFT_YELLOW);
  lcd.setTextSize(2);
  printnice(usecFilledCircles);

  lcd.setTextColor(TFT_CYAN); lcd.setTextSize(1);
  lcd.print(F("Triangles          "));
  lcd.setTextColor(TFT_YELLOW);
  lcd.setTextSize(2);
  printnice(usecTriangles);

  lcd.setTextColor(TFT_CYAN); lcd.setTextSize(1);
  lcd.print(F("Triangles-filled   "));
  lcd.setTextColor(TFT_YELLOW);
  lcd.setTextSize(2);
  printnice(usecFilledTrangles);

  lcd.setTextColor(TFT_CYAN); lcd.setTextSize(1);
  lcd.print(F("Rounded rects      "));
  lcd.setTextColor(TFT_YELLOW);
  lcd.setTextSize(2);
  printnice(usecRoundRects);

  lcd.setTextColor(TFT_CYAN); lcd.setTextSize(1);
  lcd.print(F("Rounded rects-fill "));
  lcd.setTextColor(TFT_YELLOW);
  lcd.setTextSize(2);
  printnice(usedFilledRoundRects);

  lcd.setTextSize(1);
  lcd.println(F(""));
  lcd.setTextColor(TFT_GREEN); lcd.setTextSize(2);
  lcd.print(F("Benchmark Complete!"));

  delay(60 * 1000L);
}

void printnice(int32_t v)
{
  char  str[32] = { 0 };
  sprintf(str, "%d", v);
  for (char *p = (str+strlen(str))-3; p > str; p -= 3)
  {
    memmove(p+1, p, strlen(p)+1);
    *p = ',';
  }
  while (strlen(str) < 10)
  {
    memmove(str+1, str, strlen(str)+1);
    *str = ' ';
  }
  lcd.println(str);
}

static inline uint32_t micros_start() __attribute__ ((always_inline));
static inline uint32_t micros_start()
{
  uint8_t oms = millis();
  while ((uint8_t)millis() == oms)
    ;
  return micros();
}

uint32_t testHaD()
{
  // Simplified HaD test - just fill screen with pattern
  lcd.fillScreen(TFT_BLACK);
  
  uint32_t start = micros_start();
  
  for (int i = 0; i < 16; i++)
  {
    uint16_t color = lcd.color565((i << 4) | i, (i << 4) | i, (i << 4) | i);
    lcd.fillScreen(color);
  }
  
  uint32_t t = micros() - start;
  
  lcd.fillScreen(TFT_BLACK);
  
  lcd.setTextColor(TFT_YELLOW);
  lcd.setTextSize(2);
  lcd.setCursor(8, 285);
  lcd.print(F("http://hackaday.io/"));
  lcd.setCursor(96, 302);
  lcd.print(F("Xark"));

  delay(3 * 1000L);
  
  return t;
}

uint32_t testFillScreen()
{
  uint32_t start = micros_start();
  lcd.fillScreen(TFT_WHITE);
  lcd.fillScreen(TFT_RED);
  lcd.fillScreen(TFT_GREEN);
  lcd.fillScreen(TFT_BLUE);
  lcd.fillScreen(TFT_BLACK);

  return (micros() - start)/5;
}

uint32_t testText()
{
  lcd.fillScreen(TFT_BLACK);
  uint32_t start = micros_start();
  lcd.setCursor(0, 0);
  lcd.setTextColor(TFT_WHITE,TFT_BLACK);
  lcd.setTextSize(1);
  lcd.println(F("Hello World!"));
  lcd.setTextColor(TFT_YELLOW);
  lcd.setTextSize(2);
  lcd.println(1234.56);
  lcd.setTextColor(TFT_RED);
  lcd.setTextSize(3);
  lcd.println(0xDEADBEEF, HEX);
  lcd.println();
  lcd.setTextColor(TFT_GREEN);
  lcd.setTextSize(5);
  lcd.println(F("Groop"));
  lcd.setTextSize(2);
  lcd.println(F("I implore thee,"));
  lcd.setTextColor(TFT_GREEN);
  lcd.setTextSize(1);
  lcd.println(F("my foonting turlingdromes."));
  lcd.println(F("And hooptiously drangle me"));
  lcd.println(F("with crinkly bindlewurdles,"));
  lcd.println(F("Or I will rend thee"));
  lcd.println(F("in the gobberwarts"));
  lcd.println(F("with my blurglecruncheon,"));
  lcd.println(F("see if I don't!"));
  lcd.println(F(""));
  lcd.println(F(""));
  lcd.setTextColor(TFT_MAGENTA);
  lcd.setTextSize(6);
  lcd.println(F("Woot!"));
  uint32_t t = micros() - start;
  delay(1000);
  return t;
}

uint32_t testPixels()
{
  int32_t w = lcd.width();
  int32_t h = lcd.height();

  uint32_t start = micros_start();
  lcd.startWrite();
  for (uint16_t y = 0; y < h; y++)
  {
    for (uint16_t x = 0; x < w; x++)
    {
      lcd.drawPixel(x, y, lcd.color565(x<<3, y<<3, x*y));
    }
  }
  lcd.endWrite();
  return micros() - start;
}

uint32_t testLines(uint16_t color)
{
  uint32_t start, t;
  int32_t x1, y1, x2, y2;
  int32_t w = lcd.width();
  int32_t h = lcd.height();

  lcd.fillScreen(TFT_BLACK);

  x1 = y1 = 0;
  y2 = h - 1;

  start = micros_start();

  for (x2 = 0; x2 < w; x2 += 6)
  {
    lcd.drawLine(x1, y1, x2, y2, color);
  }

  x2 = w - 1;
  for (y2 = 0; y2 < h; y2 += 6)
  {
    lcd.drawLine(x1, y1, x2, y2, color);
  }

  t = micros() - start;

  lcd.fillScreen(TFT_BLACK);

  x1 = w - 1;
  y1 = 0;
  y2 = h - 1;

  start = micros_start();

  for (x2 = 0; x2 < w; x2 += 6)
  {
    lcd.drawLine(x1, y1, x2, y2, color);
  }

  x2 = 0;
  for (y2 = 0; y2 < h; y2 += 6)
  {
    lcd.drawLine(x1, y1, x2, y2, color);
  }

  t += micros() - start;

  lcd.fillScreen(TFT_BLACK);

  x1 = 0;
  y1 = h - 1;
  y2 = 0;

  start = micros_start();

  for (x2 = 0; x2 < w; x2 += 6)
  {
    lcd.drawLine(x1, y1, x2, y2, color);
  }
  x2 = w - 1;
  for (y2 = 0; y2 < h; y2 += 6)
  {
    lcd.drawLine(x1, y1, x2, y2, color);
  }
  t += micros() - start;

  lcd.fillScreen(TFT_BLACK);

  x1 = w - 1;
  y1 = h - 1;
  y2 = 0;

  start = micros_start();

  for (x2 = 0; x2 < w; x2 += 6)
  {
    lcd.drawLine(x1, y1, x2, y2, color);
  }

  x2 = 0;
  for (y2 = 0; y2 < h; y2 += 6)
  {
    lcd.drawLine(x1, y1, x2, y2, color);
  }

  t += micros() - start;

  return t;
}

uint32_t testFastLines(uint16_t color1, uint16_t color2)
{
  uint32_t start;
  int32_t x, y;
  int32_t w = lcd.width();
  int32_t h = lcd.height();

  lcd.fillScreen(TFT_BLACK);

  start = micros_start();

  for (y = 0; y < h; y += 5)
    lcd.drawFastHLine(0, y, w, color1);
  for (x = 0; x < w; x += 5)
    lcd.drawFastVLine(x, 0, h, color2);

  return micros() - start;
}

uint32_t testRects(uint16_t color)
{
  uint32_t start;
  int32_t n, i, i2;
  int32_t cx = lcd.width() / 2;
  int32_t cy = lcd.height() / 2;

  lcd.fillScreen(TFT_BLACK);
  n = std::min(lcd.width(), lcd.height());
  start = micros_start();
  for (i = 2; i < n; i += 6)
  {
    i2 = i / 2;
    lcd.drawRect(cx-i2, cy-i2, i, i, color);
  }

  return micros() - start;
}

uint32_t testFilledRects(uint16_t color1, uint16_t color2)
{
  uint32_t start, t = 0;
  int32_t n, i, i2;
  int32_t cx = lcd.width() / 2 - 1;
  int32_t cy = lcd.height() / 2 - 1;

  lcd.fillScreen(TFT_BLACK);
  n = std::min(lcd.width(), lcd.height());
  for (i = n; i > 0; i -= 6)
  {
    i2 = i / 2;

    start = micros_start();

    lcd.fillRect(cx-i2, cy-i2, i, i, color1);

    t += micros() - start;

    lcd.drawRect(cx-i2, cy-i2, i, i, color2);
  }

  return t;
}

uint32_t testFilledCircles(uint8_t radius, uint16_t color)
{
  uint32_t start;
  int32_t x, y, w = lcd.width(), h = lcd.height(), r2 = radius * 2;

  lcd.fillScreen(TFT_BLACK);

  start = micros_start();

  for (x = radius; x < w; x += r2)
  {
    for (y = radius; y < h; y += r2)
    {
      lcd.fillCircle(x, y, radius, color);
    }
  }

  return micros() - start;
}

uint32_t testCircles(uint8_t radius, uint16_t color)
{
  uint32_t start;
  int32_t x, y, r2 = radius * 2;
  int32_t w = lcd.width() + radius;
  int32_t h = lcd.height() + radius;

  start = micros_start();

  for (x = 0; x < w; x += r2)
  {
    for (y = 0; y < h; y += r2)
    {
      lcd.drawCircle(x, y, radius, color);
    }
  }

  return micros() - start;
}

uint32_t testTriangles()
{
  uint32_t start;
  int32_t n, i;
  int32_t cx = lcd.width()/ 2 - 1;
  int32_t cy = lcd.height() / 2 - 1;

  lcd.fillScreen(TFT_BLACK);
  n = std::min(cx, cy);

  start = micros_start();

  for (i = 0; i < n; i += 5)
  {
    lcd.drawTriangle(
      cx, cy - i,
      cx - i, cy + i,
      cx + i, cy + i,
      lcd.color565(0, 0, i));
  }

  return micros() - start;
}

uint32_t testFilledTriangles()
{
  uint32_t start, t = 0;
  int32_t i;
  int32_t cx = lcd.width() / 2 - 1;
  int32_t cy = lcd.height() / 2 - 1;

  lcd.fillScreen(TFT_BLACK);

  start = micros_start();
  for (i = std::min(cx,cy); i > 10; i -= 5) {
    start = micros_start();
    lcd.fillTriangle(cx, cy - i, cx - i, cy + i, cx + i, cy + i,
      lcd.color565(0, i, i));
    t += micros() - start;
    lcd.drawTriangle(cx, cy - i, cx - i, cy + i, cx + i, cy + i,
      lcd.color565(i, i, 0));
  }

  return t;
}

uint32_t testRoundRects()
{
  uint32_t start;
  int32_t w, i, i2;
  int32_t cx = lcd.width() / 2 - 1;
  int32_t cy = lcd.height() / 2 - 1;

  lcd.fillScreen(TFT_BLACK);
  
  w = std::min(lcd.width(), lcd.height());
  
  start = micros_start();

  for (i = 0; i < w; i += 6)
  {
    i2 = i / 2;
    lcd.drawRoundRect(cx-i2, cy-i2, i, i, i/8, lcd.color565(i, 0, 0));
  }

  return micros() - start;
}

uint32_t testFilledRoundRects()
{
  uint32_t start;
  int32_t i, i2;
  int32_t cx = lcd.width() / 2 - 1;
  int32_t cy = lcd.height() / 2 - 1;

  lcd.fillScreen(TFT_BLACK);

  start = micros_start();

  for (i = std::min(lcd.width(), lcd.height()); i > 20; i -= 6)
  {
    i2 = i / 2;
    lcd.fillRoundRect(cx-i2, cy-i2, i, i, i/8, lcd.color565(0, i, 0));
  }

  return micros() - start;
}

/***************************************************
  Original sketch text:

  This is an example sketch for the Adafruit 2.2" SPI display.
  This library works with the Adafruit 2.2" TFT Breakout w/SD card
  ----> http://www.adafruit.com/products/1480
 
  Check out the links above for our tutorials and wiring diagrams
  These displays use SPI to communicate, 4 or 5 pins are required to
  interface (RST is optional)
  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.
  MIT license, all text above must be included in any redistribution

****************************************************/
