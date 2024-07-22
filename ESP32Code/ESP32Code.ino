#include <TensorFlowLite_ESP32.h>

#include "esp_camera.h"
#include <WiFi.h>
#include <Wire.h>
#include "camera_pins.h"
#include "misc.h"

#include <HTTPClient.h>
#include <WiFiMulti.h>

#include "model_data_32x32.h"

// #include "tensorflow/lite/experimental/micro/kernels/micro_ops.h"
// #include "tensorflow/lite/experimental/micro/micro_error_reporter.h"
// #include "tensorflow/lite/experimental/micro/micro_interpreter.h"
// #include "tensorflow/lite/experimental/micro/micro_mutable_op_resolver.h"
// #include "tensorflow/lite/schema/schema_generated.h"
// #include "tensorflow/lite/version.h"

#include "tensorflow/lite/micro/kernels/micro_ops.h"
#include "tensorflow/lite/micro/micro_error_reporter.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/schema/schema_generated.h"

#include <esp_heap_caps.h>
#include <esp_timer.h>
#include <esp_log.h>

#include "FS.h"                // SD Card ESP32
#include "SD_MMC.h"            // SD Card ESP32
#include "soc/soc.h"           // Disable brownour problems
#include "soc/rtc_cntl_reg.h"  // Disable brownour problems
#include "driver/rtc_io.h"
#include <EEPROM.h>            // read and write from flash memory

void startCameraServer();
void setupLedFlash(int pin);

const gpio_num_t interruptPin = GPIO_NUM_32;
const gpio_num_t testLedPin = GPIO_NUM_33;

// ===========================
// ML Model Code
// ===========================
namespace 
{
  tflite::ErrorReporter*    errorReporter = nullptr;
  const tflite::Model*      model         = nullptr;
  tflite::MicroInterpreter* interpreter   = nullptr;
  TfLiteTensor*             input         = nullptr;

  // #ifdef CONFIG_IDF_TARGET_ESP32S3
  // constexpr int scratchBufSize = 39 * 1024;
  // #else
  // constexpr int scratchBufSize = 0;
  // #endif

  // An area of memory to use for input, output, and intermediate arrays.
  constexpr int tensorArenaSize = 280 * 1024;
  static uint8_t *tensorArena = NULL; //[tensorArenaSize]; // Maybe we should move this to external
} 

void initModel()
{
  Serial.println("Initializing Model...");
  static tflite::MicroErrorReporter microErrorReporter;
  errorReporter = &microErrorReporter;

  model = tflite::GetModel(birdModelData);

  if (model->version() != TFLITE_SCHEMA_VERSION)
  {
    errorReporter->Report("Model version %d not equal to version %d.",
                         model->version(), 
                         TFLITE_SCHEMA_VERSION);
    return;
  }


  if (tensorArena == NULL)
  { 
    if (psramFound() && psramInit())
    {
      tensorArena = (uint8_t *) ps_malloc (tensorArenaSize * sizeof (uint8_t));
      if (tensorArena != NULL)
      {
        Serial.println("\nPSRAM is correctly initialized");
      }
      else
      {
        Serial.println("PSRAM not available");
        return;
      }

    }
    else
    {
      tensorArena = (uint8_t *) heap_caps_malloc(tensorArenaSize, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
      if (tensorArena == NULL)
      {
        printf("Couldn't allocate memory of %d bytes\n", tensorArenaSize);
        return;
      }
    }
  }


  Serial.println(ESP.getPsramSize());
  Serial.println(ESP.getFreePsram());

  static tflite::MicroMutableOpResolver<8> microOpResolver;
  microOpResolver.AddAdd();
  microOpResolver.AddConv2D();
  microOpResolver.AddDepthwiseConv2D();
  microOpResolver.AddFullyConnected();
  microOpResolver.AddMaxPool2D();
  microOpResolver.AddMul();
  microOpResolver.AddReshape();
  microOpResolver.AddSoftmax();


// {
//  'ADD',
//  'CONV_2D',
//  'DELEGATE',
//  'FULLY_CONNECTED',
//  'MAX_POOL_2D',
//  'MUL',
//  'RESHAPE',
//  'SOFTMAX'
// }

  // tflite::AllOpsResolver* resolver = new tflite::AllOpsResolver();

  // static tflite::MicroMutableOpResolver microOpResolver;
  // microOpResolver.AddBuiltin(tflite::BuiltinOperator_DEPTHWISE_CONV_2D,
  //                           tflite::Register_DEPTHWISE_CONV_2D(),1,6);
  // microOpResolver.AddBuiltin(tflite::BuiltinOperator_CONV_2D,
  //                           tflite::Register_CONV_2D(),1,6);
  // microOpResolver.AddBuiltin(tflite::BuiltinOperator_MAX_POOL_2D,
  //                           tflite::Register_MAX_POOL_2D(),1,6);
  // microOpResolver.AddBuiltin(tflite::BuiltinOperator_MUL,
  //                           tflite::Register_MUL(),1,6);

   // Build an interpreter to run the model with.
  static tflite::MicroInterpreter staticInterpreter(model, 
                                          microOpResolver, 
                                          tensorArena, 
                                          tensorArenaSize, 
                                          errorReporter);
  interpreter = &staticInterpreter;

  // Allocate memory from the tensorArena for the model's tensors.
  TfLiteStatus allocateStatus = interpreter->AllocateTensors();
  if (allocateStatus != kTfLiteOk)
  {
    errorReporter->Report("AllocateTensors() failed");
    return;
  }
  else
  {
    Serial.println("AllocateTensors() works");
  }

  // Get information about the memory area to use for the model's input.
  input = interpreter->input(0);

  // // Initialize Camera
  // TfLiteStatus initStatus = InitCamera(errorReporter);
  // if (initStatus != kTfLiteOk)
  // {
  //   TF_LITE_REPORT_ERROR(errorReporter, "InitCamera failed\n");
  //   return;
  // }
}

bool isBird()
{
  // Get image from provider.
  if (kTfLiteOk != takeSavePhoto(errorReporter, 
                            numCols, 
                            numRows, 
                            numChannels,
                            input->data.int8)) 
  {
    errorReporter->Report( "Image capture failed.");
  }

  // Run the model on this input and make sure it succeeds.
  if (kTfLiteOk != interpreter->Invoke())
  {
    errorReporter->Report( "Invoke failed.");
  }

  TfLiteTensor* output = interpreter->output(0);

  // Process the inference results.
  int8_t birdScore = output->data.uint8[birdIndex];
  int8_t noBirdScore = output->data.uint8[notBirdIndex];

  Serial.println("birdScore");
  Serial.println(birdScore);
  Serial.println("noBirdScore");
  Serial.println(noBirdScore);

  float birdScoreF = (birdScore - output->params.zero_point) * output->params.scale;
  float notBirdScoreF = (noBirdScore - output->params.zero_point) * output->params.scale;

  vTaskDelay(1); // to avoid watchdog trigger
}

// ===========================
// Wifi Code
// ===========================
// const char *ssid = "VIRGIN409";
// const char *password = "E64E7737E532";

const char *ssid = "dlink-7B9A";
const char *password = "beabj84316";


// const char *ssid = "Jhanvi";
// const char *password = "88888888";

void initWifi()
{
  Serial.println("Initializing WIFI...");
  WiFi.begin(ssid, password);
  WiFi.setSleep(false);

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println("WiFi connected");

  startCameraServer();

  Serial.print("Camera Ready! Use 'http://");
  Serial.print(WiFi.localIP());
  Serial.println("' to connect");
}

// ===========================
// Take Video
// ===========================

// Initialize the micro SD card
void initMicroSDCard()
{
  // Start Micro SD card
  Serial.println("Initializing SD Card...");
  if(!SD_MMC.begin("/sdcard", true))
  {
    Serial.println("SD Card Mount Failed");
    return;
  }
  uint8_t cardType = SD_MMC.cardType();
  if(cardType == CARD_NONE) {
    Serial.println("No SD Card attached");
    return;
  }

  Serial.print("SD Card Type: ");
  if(cardType == CARD_MMC) {
    Serial.println("MMC");
  } else if(cardType == CARD_SD) {
    Serial.println("SDSC");
  } else if(cardType == CARD_SDHC) {
    Serial.println("SDHC");
  } else {
    Serial.println("UNKNOWN");
  }
}

// ===========================
// Camera Code
// ===========================
void initCamera()
{
  Serial.println("Initializing Camera...");
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;  // for streaming
  config.grab_mode = CAMERA_GRAB_LATEST;
  // psramFound() are found
  config.frame_size = FRAMESIZE_QQVGA;
  config.jpeg_quality = 10;
  config.fb_count = 1;

//   camera_config_t config;
//   config.ledc_channel = LEDC_CHANNEL_0;
//   config.ledc_timer = LEDC_TIMER_0;
//   config.pin_d0 = Y2_GPIO_NUM;
//   config.pin_d1 = Y3_GPIO_NUM;
//   config.pin_d2 = Y4_GPIO_NUM;
//   config.pin_d3 = Y5_GPIO_NUM;
//   config.pin_d4 = Y6_GPIO_NUM;
//   config.pin_d5 = Y7_GPIO_NUM;
//   config.pin_d6 = Y8_GPIO_NUM;
//   config.pin_d7 = Y9_GPIO_NUM;
//   config.pin_xclk = XCLK_GPIO_NUM;
//   config.pin_pclk = PCLK_GPIO_NUM;
//   config.pin_vsync = VSYNC_GPIO_NUM;
//   config.pin_href = HREF_GPIO_NUM;
//   config.pin_sccb_sda = SIOD_GPIO_NUM;
//   config.pin_sccb_scl = SIOC_GPIO_NUM;
//   config.pin_pwdn = PWDN_GPIO_NUM;
//   config.pin_reset = RESET_GPIO_NUM;
//   config.xclk_freq_hz = 20000000;
//   config.frame_size = FRAMESIZE_UXGA;
//   config.pixel_format = PIXFORMAT_JPEG;  // for streaming
//   //config.pixel_format = PIXFORMAT_RGB565; // for face detection/recognition
//   config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
//   config.fb_location = CAMERA_FB_IN_PSRAM;
//   config.jpeg_quality = 12;
//   config.fb_count = 1;

//   // if PSRAM IC present, init with UXGA resolution and higher JPEG quality
//   //                      for larger pre-allocated frame buffer.
//   if (config.pixel_format == PIXFORMAT_JPEG) {
//     if (psramFound()) {
//       config.jpeg_quality = 10;
//       config.fb_count = 2;
//       config.grab_mode = CAMERA_GRAB_LATEST;
//     } else {
//       // Limit the frame size when PSRAM is not available
//       config.frame_size = FRAMESIZE_SVGA;
//       config.fb_location = CAMERA_FB_IN_DRAM;
//     }
//   } else {
//     // Best option for face detection/recognition
//     config.frame_size = FRAMESIZE_240X240;
// #if CONFIG_IDF_TARGET_ESP32S3
//     config.fb_count = 2;
// #endif
//   }

// #if defined(CAMERA_MODEL_ESP_EYE)
//   pinMode(13, INPUT_PULLUP);
//   pinMode(14, INPUT_PULLUP);
// #endif

//   // camera init
//   esp_err_t err = esp_camera_init(&config);
//   if (err != ESP_OK) {
//     Serial.printf("Camera init failed with error 0x%x", err);
//     return;
//   }

  esp_err_t error = esp_camera_init(&config);
  if (error != ESP_OK)
  {
    Serial.printf("Camera init failed with error 0x%x", error);
    return;
  }

  sensor_t *s = esp_camera_sensor_get();
  // initial sensors are flipped vertically and colors are a bit saturated
  if (s->id.PID == OV3660_PID)
  {
    Serial.println("Flipped");
    s->set_vflip(s, 1);        // flip it back
    s->set_brightness(s, 1);   // up the brightness just a bit
    s->set_saturation(s, -2);  // lower the saturation
  }
//   // drop down frame size for higher initial frame rate
//   if (config.pixel_format == PIXFORMAT_JPEG) {
//     s->set_framesize(s, FRAMESIZE_QVGA);
//   }

// #if defined(CAMERA_MODEL_M5STACK_WIDE) || defined(CAMERA_MODEL_M5STACK_ESP32CAM)
//   s->set_vflip(s, 1);
//   s->set_hmirror(s, 1);
// #endif

// #if defined(CAMERA_MODEL_ESP32S3_EYE)
//   s->set_vflip(s, 1);
// #endif

// // Setup LED FLash if LED pin is defined in camera_pins.h
// #if defined(LED_GPIO_NUM)
//   setupLedFlash(LED_GPIO_NUM);
// #endif
}

// Function to set timezone
void setTimezone(String timezone)
{
  // Serial.printf("  Setting Timezone to %s\n",timezone.c_str());
  setenv("TZ",timezone.c_str(),1);  //  Now adjust the TZ.  Clock settings are adjusted to show the new local time
  tzset();
}

// Connect to NTP server and adjust timezone
void initTime(String timezone)
{
  Serial.println("Init Time...");
  struct tm timeinfo;
  configTime(0, 0, "pool.ntp.org");    // First connect to NTP server, with 0 TZ offset
  if(!getLocalTime(&timeinfo))
  {
    Serial.println(" Failed to obtain time");
    return;
  }
  Serial.println("Got the time from NTP");
  // Now we can set the real timezone
  setTimezone(timezone);
}

// Get the picture filename based on the current ime
String getPictureFilename()
{
  struct tm timeinfo;
  if(!getLocalTime(&timeinfo))
  {
    Serial.println("Failed to obtain time");
    return "";
  }
  char timeString[20];
  strftime(timeString, sizeof(timeString), "%Y-%m-%d_%H-%M-%S", &timeinfo);
  Serial.println(timeString);
  String filename = "/picture_" + String(timeString) +".jpg";
  return filename; 
}

// ===========================
// Take Photo
// ===========================
TfLiteStatus takeSavePhoto(tflite::ErrorReporter* errorReporter, int imageWidth, 
                          int imageHeight, int channels, int8_t* imageData)
{
  // Take Picture with Camera
  camera_fb_t * fb = esp_camera_fb_get();
 
  //Uncomment the following lines if you're getting old pictures
  // esp_camera_fb_return(fb); // dispose the buffered image
  // fb = NULL; // reset to capture errors
  // fb = esp_camera_fb_get();
  
  if(!fb)
  {
    ESP_LOGE(TAG, "Camera capture failed");
    return kTfLiteError;
  }

  errorReporter->Report("Image Captured\n");

  if (fb->width == imageWidth && fb->height == imageHeight)
  {
    memcpy(imageData, fb->buf, fb->len);
  }
  else
  {
    // Trimming Image
    int post = 0;
    int startx = (fb->width - imageWidth) / 2;
    int starty = (fb->height - imageHeight) / 2;
    for (int y = 0; y < imageHeight; y++)
    {
      for (int x = 0; x < imageWidth; x++)
      {
        int getPos = (starty + y) * fb->width + startx + x;
        imageData[post] = fb->buf[getPos];
        post++;
      }
    }
  }
  // for (int i = 0; i < imageWidth * imageHeight; i++)
  // {
  //   imageData[i] = ((uint8_t *) fb->buf)[i] ^ 0x80;
  // }

  // Path where new picture will be saved in SD Card
  String path = getPictureFilename();
  Serial.printf("Picture file name: %s\n", path.c_str());
  
  // Save picture to microSD card
  fs::FS &fs = SD_MMC; 
  File file = fs.open(path.c_str(),FILE_WRITE);

  if(!file)
  {
    Serial.printf("Failed to open file in writing mode");
  } 
  else
  {
    file.write(fb->buf, fb->len); // payload (image), payload length
    Serial.printf("Saved: %s\n", path.c_str());
  }
  file.close();
  esp_camera_fb_return(fb);
  return kTfLiteOk;
}

/****************************************************************/
// Interrupt/Sleep Code
/****************************************************************/
void sleepNow ()
{
  // Serial.println("sleeping");
  // delay(35);
  // esp_deep_sleep_start();
}

int counter = 0;

bool birdIsGone()
{
  if (digitalRead(interruptPin) == LOW)
  {
    return true;
  }
  return false;
}

/****************************************************************/
// Server HTTP Code
/****************************************************************/

bool seedLowNotification()
{
  HTTPClient http;

  http.begin("http://34.205.232.48:8000/rt-notification");

  int responseCode = http.GET();

  // Negative on Error
  if (responseCode == HTTP_CODE_OK)
  {
    return true;
  }
  else
  {
    return false;
  }

}

/****************************************************************/
// Notification Code
/****************************************************************/
volatile uint8_t i2c_response = 0;
 
void receiveEvent(int howMany)
{
  uint8_t event = Wire.read();    // receive byte as an integer

  switch (event)
  {
    case SEED:
      Serial.println("Seed request");
      refillSeedNotification();
      break;
    case IS_BIRD:
      Serial.println("Is Bird?");
      if (isBird())
      {
        Serial.println("Bird!");
        i2c_response = BIRD;
      }
      else
      {
        Serial.println("No Bird");
        i2c_response = NO_BIRD;
      }
      break;
    default:
      Serial.println("Invalid request");
      Serial.println(event);         // print the integer
      i2c_response = INVALID_REQUEST;
      break;
  }
}

void requestEvent()
{
  Wire.write(i2c_response);
  i2c_response = RESET;
}

void refillSeedNotification()
{
  //Send notification
  if (seedLowNotification())
  {
    i2c_response = ACK;
  }
  else
  {
    i2c_response = NACK;
  }
}

/****************************************************************/
// Main Code
/****************************************************************/
void setup() 
{
  Serial.begin(115200);
  Wire.begin(SLAVE_ADDRESS);
  Wire.onRequest(requestEvent);
  Wire.onReceive(receiveEvent);


  initMicroSDCard();
  initModel();
  // initCamera();
  initWifi(); // Wifi has to be before Time
  initTime("EST5EDT,M3.2.0,M11.1.0"); // change this to universal time

  pinMode(interruptPin, INPUT);
  pinMode(testLedPin, OUTPUT);


  i2c_response = SETUP_FINISHED;
  Serial.println(i2c_response);
  // esp_sleep_enable_ext0_wakeup(interruptPin, 1); //1 = High, 0 = Low

}

void loop()
{
  if (birdIsGone())
  {
    digitalWrite(testLedPin, LOW);
    delay(200);
    sleepNow();
  }
  else
  {
    digitalWrite(testLedPin, HIGH);
  }
  delay(1000);
}
