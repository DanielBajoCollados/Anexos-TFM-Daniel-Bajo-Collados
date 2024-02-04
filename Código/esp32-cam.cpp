#include <Arduino.h>

// ESP32Cam (AiThinker)
#include "esp_camera.h"

////////////////////////////////
//Configuration of the camera //
////////////////////////////////
#define CAM_PIN_PWDN 32
#define CAM_PIN_RESET -1 // software reset will be performed
#define CAM_PIN_XCLK 0
#define CAM_PIN_SIOD 26
#define CAM_PIN_SIOC 27

#define CAM_PIN_D7 35
#define CAM_PIN_D6 34
#define CAM_PIN_D5 39
#define CAM_PIN_D4 36
#define CAM_PIN_D3 21
#define CAM_PIN_D2 19
#define CAM_PIN_D1 18
#define CAM_PIN_D0 5
#define CAM_PIN_VSYNC 25
#define CAM_PIN_HREF 23
#define CAM_PIN_PCLK 22

static camera_config_t camera_config = {
    .pin_pwdn = CAM_PIN_PWDN,
    .pin_reset = CAM_PIN_RESET,
    .pin_xclk = CAM_PIN_XCLK,
    .pin_sccb_sda = CAM_PIN_SIOD,
    .pin_sccb_scl = CAM_PIN_SIOC,

    .pin_d7 = CAM_PIN_D7,
    .pin_d6 = CAM_PIN_D6,
    .pin_d5 = CAM_PIN_D5,
    .pin_d4 = CAM_PIN_D4,
    .pin_d3 = CAM_PIN_D3,
    .pin_d2 = CAM_PIN_D2,
    .pin_d1 = CAM_PIN_D1,
    .pin_d0 = CAM_PIN_D0,
    .pin_vsync = CAM_PIN_VSYNC,
    .pin_href = CAM_PIN_HREF,
    .pin_pclk = CAM_PIN_PCLK,

    // XCLK 20MHz or 10MHz for OV2640 double FPS (Experimental)
    .xclk_freq_hz = 20000000,
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,

    .pixel_format = PIXFORMAT_JPEG, // YUV422,GRAYSCALE,RGB565,JPEG
    .frame_size = FRAMESIZE_UXGA,   // QQVGA-UXGA, For ESP32, do not use sizes above QVGA when not JPEG. The performance of the ESP32-S series has improved a lot, but JPEG mode always gives better frame rates.

    .jpeg_quality = 12, // 0-63, for OV series camera sensors, lower number means higher quality
    .fb_count = 1,      // When jpeg mode is used, if fb_count more than one, the driver will work in continuous mode.
    .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
};


////////////////////////
// Initialize camera  //
////////////////////////
static esp_err_t initCamera()
{
  // initialize the camera
  esp_err_t err = esp_camera_init(&camera_config);
  if (err != ESP_OK)
  {
    return err;
  }
  return ESP_OK;
}


////////////////////////
// WiFi Configuration //
////////////////////////
#include <WiFi.h>

WiFiClient localClient;

const uint port = 11000;
//Wifi Data
const char *ssid = "RM2_WIFI";
const char *password = "R0BOMINERS*";
IPAddress ip(192, 168, 253, 100);
////////////////////////////////////////////////////

size_t msg_size = 1024;

// SD Card Config
#include "SD_MMC.h"
#include "FS.h"
#include <EEPROM.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "driver/rtc_io.h"

#define EEPROM_SIZE 1
#define SAVE_PIC false

int pictureNumber = 0;

int initSDCard()
{
  if (!SD_MMC.begin())
    return -1;

  uint8_t cardType = SD_MMC.cardType();
  if (cardType == CARD_NONE)
    return -1;
  return 0;
}

//I2C Communication
#include "Wire.h"
#include "rm2_message.h"

////////////////////////////////////////////////////////////////////////////////
// Handler that starts once a request has been recieved via i2c communication //
////////////////////////////////////////////////////////////////////////////////
void requestHandler();

#define ESP32_CAM_ADRESS 9

// Functions
void takePicture(double* MotorsPos, bool sendPic = true, bool savePic = false);

/////////////////////////////////////////////////////////
// Send a picture in numerous batches of size msg_size //
/////////////////////////////////////////////////////////
bool sendPicture(double* MotorsPos, uint8_t *buf, size_t size);

/////////////////////////////////////////////////////////////////
// Recieve a Picture from the SD card and then send it via tcp //
/////////////////////////////////////////////////////////////////
bool recievePicture(String pic_name);

// Definitions
#define TIMEOUT 10000

// Program
void setup()
{

  if (ESP_OK != initCamera())
    return;

  if (SAVE_PIC)
  {
    if (initSDCard() < 0)
      return;
  }

  if (!psramInit())
    return;

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED)
    delay(500);
  
  for (int i=0; i<3; i++){
    double MotorsPos[6]; 
    takePicture(MotorsPos, false, SAVE_PIC);
  }

  Wire.begin(ESP32_CAM_ADRESS, 14, 15, 400000);
  Wire.onRequest(requestHandler);
  pinMode(2, OUTPUT);
}

void loop(){}

bool sendPicture(double* MotorsPos, uint8_t *buf, size_t size)
{
  uint8_t msg_buf[msg_size];
  size_t rest = size % msg_size; // The last batch will have this size
  uint8_t pck_no = size/msg_size;
  uint8_t *img_size = (uint8_t*)&size;

  if (localClient.connect(ip, port))
  {

    // Send size of image to server
    localClient.write(img_size, sizeof(size));

    while (!localClient.available()){} // Wait for response

    String str = localClient.readStringUntil('\n'); // read entire response

    for (int i = 0; i < pck_no; i++) // Send the first batches
    {
      memcpy(msg_buf, buf + (i * msg_size), msg_size);
      localClient.write(msg_buf, msg_size);
      auto itime = millis();
      while (!localClient.available() && (millis()-itime <= TIMEOUT)){} // Wait for response
      if (millis() - itime > TIMEOUT)
        return false;

      String str = localClient.readStringUntil('\n'); // read entire response
    }

    if (rest) // Send the last batch
    {
      memcpy(msg_buf, buf + (size - rest), rest);
      localClient.write(msg_buf, rest);

      while (!localClient.available()){} // Wait for response

      String str = localClient.readStringUntil('\n'); // read entire response
    }

    msg_buf[0]=1;
    localClient.write(msg_buf, 1);

    //Wait until recieving either the requiered position or a set of zeros.
    while (!localClient.available()){} // Wait for response
    
    localClient.readBytes((uint8_t*)MotorsPos, sizeof(double)*6);

    return true;
  }
  return false;
}

bool recievePicture(String pic_name)
{
  uint8_t *buf;
  fs::FS &fs = SD_MMC;

  File pic_file = fs.open(pic_name.c_str(), FILE_READ, true);
  if (!pic_file)
    return false;

  size_t pic_len = pic_file.available();
  buf = (uint8_t *)ps_malloc(pic_len);
  for (int i = 0; i < pic_len; i++)
    buf[i] = pic_file.read();
  double MotorsPos[6];
  if (!sendPicture(MotorsPos, buf, pic_len))
    return false;

  return true;
}

void takePicture(double* MotorsPos, bool sendPic, bool savePic)
{
  // take picture
  camera_fb_t *pic = esp_camera_fb_get();
  if (!pic){
    return;
  }
  // save Pic on SD Card
  if (savePic){
    EEPROM.begin(EEPROM_SIZE);
    pictureNumber = EEPROM.read(0) + 1;
    String pic_name = "/picture_" + String(pictureNumber) + ".jpg";
    fs::FS &fs = SD_MMC;

    File pic_file = fs.open(pic_name.c_str(), FILE_WRITE);
    if (!pic_file)
      return;

    pic_file.write(pic->buf, pic->len);
    EEPROM.write(0, pictureNumber);
    EEPROM.commit();
    pic_file.close();
  }
  if (sendPic)
    sendPicture(MotorsPos, pic->buf, pic->len);

  esp_camera_fb_return(pic);
}

void requestHandler(){
  double MotorsPos[6];
  takePicture(MotorsPos);
  uint8_t buf[sizeof(MotorsPos)+2];
  memcpy(buf, MotorsPos, sizeof(MotorsPos));
  uint16_2byteConversor crc;
  crc.integer = CRC16::crc16(buf, sizeof(MotorsPos));
  buf[sizeof(MotorsPos)]=crc.sbytes[0];
  buf[sizeof(MotorsPos)+1]=crc.sbytes[1];
  Wire.write(buf, sizeof(MotorsPos)+2);
}