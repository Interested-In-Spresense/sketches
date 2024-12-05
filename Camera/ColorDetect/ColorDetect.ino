/*
 *  ColorDetect.ino - camera example sketch
 *  Copyright 2024, Spresense Users
 *
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 *  This is a test app for the camera library.
 *  This library can only be used on the Spresense with the FCBGA chip package.
 */

#include <stdio.h>  /* for sprintf */

#include <Camera.h>

#define BAUDRATE                (115200)

#define OFFSET_X  (112)
#define OFFSET_Y  (72)
#define CLIP_WIDTH  (96)
#define CLIP_HEIGHT  (96)
#define DETECT_WIDTH  (12)
#define DETECT_HEIGHT  (12)
#define DETECT_THRESH  (18) /* 18 / 36(6x6) */

//#define USE_LIB_ADAFRUIT_ILI9341
#ifdef USE_LIB_ADAFRUIT_ILI9341
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>

#define TFT_CS -1
#define TFT_RST 8
#define TFT_DC  9
#define TFT_ROTATION 3

Adafruit_ILI9341 tft = Adafruit_ILI9341(&SPI, TFT_DC, TFT_CS, TFT_RST);
#endif

typedef enum 
{
	DETECT_NONE,
	DETECT_RED,
	DETECT_YELLOW,
	DETECT_GREEN,
} DetectColor_t;

DetectColor_t current_color = DETECT_NONE;

/**
 * Print error message
 */

void printError(enum CamErr err)
{
  Serial.print("Error: ");
  switch (err)
    {
      case CAM_ERR_NO_DEVICE:
        Serial.println("No Device");
        break;
      case CAM_ERR_ILLEGAL_DEVERR:
        Serial.println("Illegal device error");
        break;
      case CAM_ERR_ALREADY_INITIALIZED:
        Serial.println("Already initialized");
        break;
      case CAM_ERR_NOT_INITIALIZED:
        Serial.println("Not initialized");
        break;
      case CAM_ERR_NOT_STILL_INITIALIZED:
        Serial.println("Still picture not initialized");
        break;
      case CAM_ERR_CANT_CREATE_THREAD:
        Serial.println("Failed to create thread");
        break;
      case CAM_ERR_INVALID_PARAM:
        Serial.println("Invalid parameter");
        break;
      case CAM_ERR_NO_MEMORY:
        Serial.println("No memory");
        break;
      case CAM_ERR_USR_INUSED:
        Serial.println("Buffer already in use");
        break;
      case CAM_ERR_NOT_PERMITTED:
        Serial.println("Operation not permitted");
        break;
      default:
        break;
    }
}

bool detectRed(int r, int g, int b)
{
  if((r>0x17) && (g<0x10) && (b<0x2)) return true;
  return false;
}

bool detectYellow(int r, int g, int b)
{
  if((r>0x13) && (g>0x23) && (b<0x2)) return true;
  return false;
}

bool detectGreen(int r, int g, int b)
{
  if((r<0x9) && (g>0x26) && (b<0x8)) return true;
  return false;
}

/**
 * Callback from Camera library when video frame is captured.
 */

void CamCB(CamImage img)
{
  if (!img.isAvailable()) return;

  // HWでの画像の切り出しと縮小
  CamImage small; 
  CamErr camErr = img.clipAndResizeImageByHW(small
            ,OFFSET_X ,OFFSET_Y 
            ,OFFSET_X+CLIP_WIDTH-1 ,OFFSET_Y+CLIP_HEIGHT-1 
            ,DETECT_WIDTH ,DETECT_HEIGHT);

  if (!small.isAvailable()) {puts("error!");return;}

  // 画像をYUVからRGB565に変換
  small.convertPixFormat(CAM_IMAGE_PIX_FMT_RGB565); 
  uint16_t* sbuf = (uint16_t*)small.getImgBuff();

#ifdef USE_LIB_ADAFRUIT_ILI9341
      /* You can use image data directly by using getImgSize() and getImgBuff().
       * for displaying image to a display, etc. */
      tft.drawRGBBitmap(0, 0, (uint16_t *)sbuf, DETECT_WIDTH, DETECT_HEIGHT);
#endif

  int detect_red = 0;
  int detect_yellow = 0;
  int detect_green = 0;

  for(int i=0;i<DETECT_WIDTH;i=i+2){
    for(int j=0;j<DETECT_HEIGHT;j=j+2){
      int k = i*DETECT_HEIGHT+j;
      int red = (sbuf[k] >> 11) & 0x1F;
      int green = (sbuf[k] >> 5) & 0x3F;
      int blue = sbuf[k]  & 0x1F;
//      printf("sbuf[%d]=%04x,r=%02x,g=%02x,b=%02x\n",k,sbuf[k],red,green,blue);
      if(detectRed(red,green,blue)) detect_red++;
      if(detectYellow(red,green,blue)) detect_yellow++;
      if(detectGreen(red,green,blue)) detect_green++;
    }
  }

  if(detect_red > DETECT_THRESH){
    current_color = DETECT_RED;
  }else if(detect_yellow > DETECT_THRESH){
    current_color = DETECT_YELLOW;
  }else if(detect_green > DETECT_THRESH){
    current_color = DETECT_GREEN;
  }else{
    current_color = DETECT_NONE;
  }
}

/**
 * @brief Initialize camera
 */
void setup()
{
  CamErr err;

  /* Open serial communications and wait for port to open */

  Serial.begin(BAUDRATE);
  while (!Serial)
    {
      ; /* wait for serial port to connect. Needed for native USB port only */
    }

#ifdef USE_LIB_ADAFRUIT_ILI9341
  /* Initialize TFT */
  tft.begin();
  tft.setRotation(TFT_ROTATION);
#endif

  /* begin() without parameters means that
   * number of buffers = 1, 30FPS, QVGA, YUV 4:2:2 format */

  Serial.println("Prepare camera");
  err = theCamera.begin();
  if (err != CAM_ERR_SUCCESS)
    {
      printError(err);
    }

  /* Start video stream.
   * If received video stream data from camera device,
   *  camera library call CamCB.
   */

  Serial.println("Start streaming");
  err = theCamera.startStreaming(true, CamCB);
  if (err != CAM_ERR_SUCCESS)
    {
      printError(err);
    }

  /* Auto white balance configuration */

  Serial.println("Set Auto white balance parameter");
  err = theCamera.setAutoWhiteBalanceMode(CAM_WHITE_BALANCE_DAYLIGHT);
  if (err != CAM_ERR_SUCCESS)
    {
      printError(err);
    }
 
}

/**
 * @brief Take picture with format JPEG per second
 */

void loop()
{
  switch(current_color){
    case DETECT_RED:
    Serial.println("Detect Red!");
    break;
    case DETECT_YELLOW:
    Serial.println("Detect Yellow!");
    break;
    case DETECT_GREEN:
    Serial.println("Detect Green!");
    break;
    case DETECT_NONE:
    default:
    break;
  }
  sleep(1);
}
