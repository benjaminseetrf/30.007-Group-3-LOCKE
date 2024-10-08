#include <Arduino_GFX_Library.h>
#include <SD_MMC.h>
#include <TAMC_GT911.h>
#include <Wire.h>
#include <Audio.h>
#include "JpegFunc.h"
#include <esp_now.h>
#include <WiFi.h>


#define SCREEN_HD

#define JPEG_FILENAME_LOGO "/logo.jpg"
#define JPEG_FILENAME_COVER "/cover.jpg"
#define JPEG_FILENAME_COVER_01 "/cover01.jpg"

#define I2S_DOUT 19
#define I2S_BCLK 20
#define I2S_LRC 2

// microSD card
#define PIN_SD_CMD 11
#define PIN_SD_CLK 12
#define PIN_SD_D0 13

#define I2C_SDA_PIN 17
#define I2C_SCL_PIN 18
#define TOUCH_INT -1
#define TOUCH_RST 38

#define TOUCH_ROTATION ROTATION_NORMAL

#define TFT_BL 10

#ifdef SCREEN_HD
#define SCREEN_W 1024
#define SCREEN_H 600
#endif


int wordsonscreen = 50;

int backrest_diff;
int seat_diff;
int seat_prox_diff;

Arduino_ESP32RGBPanel *bus = new Arduino_ESP32RGBPanel(
    GFX_NOT_DEFINED /* CS */, GFX_NOT_DEFINED /* SCK */, GFX_NOT_DEFINED /* SDA */,
    40 /* DE */, 41 /* VSYNC */, 39 /* HSYNC */, 42 /* PCLK */,
    45 /* R0 */, 48 /* R1 */, 47 /* R2 */, 21 /* R3 */, 14 /* R4 */,
    5 /* G0 */, 6 /* G1 */, 7 /* G2 */, 15 /* G3 */, 16 /* G4 */, 4 /* G5 */,
    8 /* B0 */, 3 /* B1 */, 46 /* B2 */, 9 /* B3 */, 1 /* B4 */
);


#ifdef SCREEN_HD
Arduino_RPi_DPI_RGBPanel *gfx = new Arduino_RPi_DPI_RGBPanel(
    bus,
    SCREEN_W /* width */, 1 /* hsync_polarity */, 40 /* hsync_front_porch */, 48 /* hsync_pulse_width */, 128 /* hsync_back_porch */,
    SCREEN_H /* height */, 1 /* vsync_polarity */, 13 /* vsync_front_porch */, 3 /* vsync_pulse_width */, 45 /* vsync_back_porch */,
    1 /* pclk_active_neg */, 16000000 /* prefer_speed */, true /* auto_flush */);
#endif

TAMC_GT911 ts = TAMC_GT911(I2C_SDA_PIN, I2C_SCL_PIN, TOUCH_INT, TOUCH_RST, SCREEN_W, SCREEN_H);


String img_list[5] =
    {
        "/left.jpg",
        "/right.jpg",
        "/slouch.jpg",
        "/notslouch.jpg",
        "/centre.jpg"};


//-------------------------------------------------------------------------------------------------------
// ALL THE CODE BELOW IS FOR SENDING AND RECEIVING DATA UNCOMMENT WHEN NECESSARY
//uint8_t broadcastAddress[] = {0x08, 0xD1, 0xF9, 0xCA, 0x25, 0xCC};  // Use correct MAC address format, changed already

typedef struct struct_message {
    bool button1_pressed;
    bool button2_pressed;
    bool switch_flicked;

    bool outerlimit_on;
    bool innerlimit_on;

    int force_sensor1;
    int force_sensor2;
    int force_sensor3;
    int force_sensor4;
    int force_sensor5;
    int force_sensor6;

    float prox1_value;
    int prox1_status;
    float prox2_value;
    int prox2_status;
    float prox3_value;
    int prox3_status;
    float prox4_value;
    int prox4_status;
    float prox5_value;
    int prox5_status;
    float prox6_value;
    int prox6_status;


} struct_message;

//struct_message buttons;

struct_message screen;

esp_now_peer_info_t peerInfo;


//ALL THE CODE BELOW IS FOR MY VARIABLES THAT I WANT TO TRACK
// Enumeration for the Lean states
enum LeanDirection { LEFT, CENTRE, RIGHT };

// Variables for state
bool slouching = false;
LeanDirection lean = CENTRE;
bool lastSlouching = false;
bool lastActivated = true;  // Track the last activated state

// Timing variables
unsigned long slouchingTime = 0, slouchingStartTime = 0; 
unsigned long notslouchingTime = 0, notslouchingStartTime = 0;
unsigned long leanTime[3] = {0, 0, 0}; // Time for LEFT, CENTRE, RIGHT
unsigned long leanStartTime = 0;
LeanDirection lastLean = CENTRE;

bool displayStats = false; // Flag to toggle display mode
bool activated = false;

unsigned long totalTime = millis(); // Total operational time since program start

void pin_init();  // Forward declaration
void touch_init();  // Forward declaration //whats this


// pixel drawing callback
static int jpegDrawCallback(JPEGDRAW *pDraw)
{
  // Serial.printf("Draw pos = %d,%d. size = %d x %d\n", pDraw->x, pDraw->y, pDraw->iWidth, pDraw->iHeight);
  gfx->draw16bitBeRGBBitmap(pDraw->x, pDraw->y, pDraw->pPixels, pDraw->iWidth, pDraw->iHeight);
  return 1;
}




void Checking(void *pvParameters) {
  static bool displayUpdateNeeded = false;
  while (1){
  ts.read();  // Read the touch sensor state
// This changes the screen mode to display how much time the person has been sitting in a specific position, touch the screen to go back to regular display
  static unsigned long touchStartTime = 0; // Keep track of when the touch started
  const unsigned long touchDurationRequired = 3000; // Duration required for a valid touch (2000 ms = 2 seconds)

 if (ts.isTouched) {
    if (touchStartTime == 0) { // If this is the start of a new touch
        touchStartTime = millis();
    } else if (millis() - touchStartTime > touchDurationRequired) { // Check if the touch has lasted long enough
        activated = !activated;  // Toggle the activated state
        touchStartTime = 0; // Reset start time so it requires letting go and retouching

    // Debug output to the serial monitor (optional)
    Serial.print("Activated state changed to: ");
    Serial.println(activated ? "true" : "false");
  }
  
 }
 else {
    // Display logic based on 'activated'
    if (activated) {
        inputUpdates();
        updateLeanTime(lean);
        showStatisticsScreen(); // Continuously update the statistics screen
        lastActivated = true;
    } else {
        inputUpdates();
        if (lastActivated || lean != lastLean || slouching != lastSlouching) {
            updateDisplay(); // Update display if there was a change in state or first time switching from activated
            lastLean = lean;
            lastSlouching = slouching;
            lastActivated = false;  
          }
        }
      }
    }   
  }



void updateDisplay() {
 // Update the left side of the screen based on the lean direction
 if (lean == LEFT) {
   Serial.println("/left.jpg");
   jpegDraw("/left.jpg", jpegDrawCallback, true /* useBigEndian */,
                0 /* x */, 0 /* y */, SCREEN_W / 2 /* widthLimit */, SCREEN_H /* heightLimit */);

 } else if (lean == RIGHT) {
   Serial.println("/right.jpg");
   jpegDraw("/right.jpg", jpegDrawCallback, true /* useBigEndian */,
                0 /* x */, 0 /* y */, SCREEN_W / 2 /* widthLimit */, SCREEN_H /* heightLimit */);
 } else if (lean == CENTRE) {
   Serial.println("/centre.jpg");
//  gfx->fillRect(0, 0, SCREEN_W / 2, SCREEN_H, BLACK);  // Fill left half with black
   jpegDraw("/centre.jpg", jpegDrawCallback, true /* useBigEndian */,
                0 /* x */, 0 /* y */, SCREEN_W / 2  /* widthLimit */, SCREEN_H /* heightLimit */);
 }
 //Update the right side of the screen based on the slouching state
  if (slouching) {
    Serial.println("/slouch.jpg");

   jpegDraw("/slouch.jpg", jpegDrawCallback, true /* useBigEndian */,
               SCREEN_W / 2 /* x */, 0 /* y */, SCREEN_W / 2 /* widthLimit */, SCREEN_H /* heightLimit */);
 } else {
   Serial.println("/notslouch.jpg");
//   gfx->fillRect(SCREEN_W / 2, 0, SCREEN_W / 2, SCREEN_H, BLACK);  // Fill right half with black
   jpegDraw("/notslouch.jpg", jpegDrawCallback, true /* useBigEndian */,
               SCREEN_W / 2 /* x */, 0 /* y */, SCREEN_W / 2  /* widthLimit */, SCREEN_H   /* heightLimit */);
  }
    gfx->setCursor(430, wordsonscreen + 445);
    gfx->setTextSize(2);
    gfx->setTextColor(PURPLE);
    gfx->println("Hold Screen to");
    gfx->setCursor(480, wordsonscreen + 475);    
    gfx->println("go to");
    gfx->setCursor(405, wordsonscreen + 505);    
    gfx->println("statistics display");  
    delay(800);
}

void showStatisticsScreen() {
    gfx->fillScreen(0x04FF);
    gfx->setTextColor(WHITE);
    gfx->setTextSize(3);
//Left Time
    gfx->setCursor(50, wordsonscreen);
    gfx->println("Time Spent Leaning Left: ");
    gfx->setTextColor(RED);
    gfx->setTextSize(4);
    gfx->setCursor(50, wordsonscreen + 30);
    gfx->println(String(leanTime[LEFT] / 1000.0, 3) + " s");
//Right Time
    gfx->setTextColor(WHITE);
    gfx->setTextSize(3);
    gfx->setCursor(50, wordsonscreen + 80);
    gfx->println("Time Spent Leaning Right: ");
    gfx->setTextColor(RED);
    gfx->setTextSize(4);
    gfx->setCursor(50, wordsonscreen + 110);
    gfx->println(String(leanTime[RIGHT] / 1000.0, 3) + " s");
//Centre Time
    gfx->setTextColor(WHITE);
    gfx->setTextSize(3);
    gfx->setCursor(50, wordsonscreen + 160);
    gfx->println("Time Spent Sitting in the Centre: ");
    gfx->setTextColor(GREEN);
    gfx->setTextSize(4);
    gfx->setCursor(50, wordsonscreen + 190);
    gfx->println(String(leanTime[CENTRE] / 1000.0, 3) + " s");
//Slouch Time
    gfx->setTextColor(WHITE);
    gfx->setTextSize(3);
    gfx->setCursor(50, wordsonscreen + 240);
    gfx->println("Time Spent Slouching: ");
    gfx->setTextColor(RED);
    gfx->setTextSize(4);
    gfx->setCursor(50, wordsonscreen + 270);
    gfx->println(String((slouchingTime + (slouching ? millis() - slouchingStartTime : 0)) / 1000.0, 3) + " s");
//Upright Time
    gfx->setTextColor(WHITE);
    gfx->setTextSize(3);
    gfx->setCursor(50, wordsonscreen + 320);
    gfx->println("Time Spent Sitting Up Straight: ");
    gfx->setTextColor(GREEN);
    gfx->setTextSize(4);
    gfx->setCursor(50, wordsonscreen + 350);
    gfx->println(String((notslouchingTime + (!slouching ? millis() - notslouchingStartTime : 0)) / 1000.0, 3) + " s");

//Instructions to go back        
    gfx->setCursor(50, wordsonscreen + 450);
    gfx->setTextSize(3);
    gfx->setTextColor(PURPLE);
    gfx->println("Hold Screen to go back to active display");

    delay (800);
}

int leftthresholdLeaning = 100;  // Adjust based on calibration
int rightthresholdLeaning = 50;  // Adjust based on calibration

int PressureleftthresholdLeaning = 10;  // Adjust based on calibrationsss
int PressurerightthresholdLeaning = 100;  // Adjust based on calibration

int ProxleftthresholdLeaning = 150;  // Adjust based on calibration
int ProxrightthresholdLeaning = 100;  // Adjust based on calibration

int backrest_proxsensorsthreshold = 10; //Adjust based on calibration
int backrest_debugproxthreshold = 2000;

void inputUpdates() {
    static bool inCalibrationMode = false;  // Flag to check if in calibration mode
    if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim(); // Remove any trailing whitespace

    if (input == "calib") {
      inCalibrationMode = true;
      showCalibrationScreen();
      Serial.println("Switched to Calibration Screen");
      inCalibrationMode = false;
      updateDisplay();
      Serial.println("Returned to Regular Display");
    }

  else if (!inCalibrationMode){

    if (input == "leftthresh") {
      Serial.println("Enter new left threshold:");
      while (Serial.available() == 0) {} // Wait for input
      leftthresholdLeaning = Serial.parseInt();
      Serial.print("Left threshold updated to: ");
      Serial.println(leftthresholdLeaning);
    } else if (input == "rightthresh") {
      Serial.println("Enter new right threshold:");
      while (Serial.available() == 0) {} // Wait for input
      rightthresholdLeaning = Serial.parseInt();
      Serial.print("Right threshold updated to: ");
      Serial.println(rightthresholdLeaning);
    }
      else if (input == "proxsensorsthresh") {
      Serial.println("Enter new proximity sensor threshold:");
      while (Serial.available() == 0) {} // Wait for input
      backrest_proxsensorsthreshold = Serial.parseInt();
      Serial.print("Proximity sensor threshold updated to: ");
      Serial.println(backrest_proxsensorsthreshold);  
      }

    if (input == "slouching") {
      if (!slouching) {
        notslouchingTime += millis() - notslouchingStartTime; 
      }
      slouching = true;
      slouchingStartTime = millis();
    } else if (input == "notslouching") {
      if (slouching) {
        slouchingTime += millis() - slouchingStartTime; // Update slouching time
      }
      slouching = false;
      notslouchingStartTime = millis();
    }
    // Handle lean direction inputs

        // Handle lean direction inputs
        if (input == "left" || input == "centre" || input == "right") {
            updateLeanTime(input == "left" ? LEFT : input == "centre" ? CENTRE : RIGHT);
        }
        // Handle time display request
        if (input == "time") {
            // Ensure the current time is updated before displaying
            updateLeanTime(lean);
            Serial.println("Time Spent Leaning Left: " + String(leanTime[LEFT] / 1000.0, 3) + " s");
            Serial.println("Time Spent Leaning Right: " + String(leanTime[RIGHT] / 1000.0, 3) + " s");
            Serial.println("Time Spent Sitting in the Centre: " + String(leanTime[CENTRE] / 1000.0, 3) + " s");
            Serial.println("Time Spent Slouching: " + String((slouchingTime + (slouching ? millis() - slouchingStartTime : 0)) / 1000.0, 3) + " s");
            Serial.println("Time Spent Sitting Up Straight: " + String((notslouchingTime + (!slouching ? millis() - notslouchingStartTime : 0)) / 1000.0, 3) + " s");
        }
}
}
}


void updateLeanTime(LeanDirection newLean) {
    unsigned long currentMillis = millis();

    // Update the current state's time regardless of what it is
    leanTime[lean] += currentMillis - leanStartTime;  // This updates the time for any state including CENTRE

    // Only update the lean direction and reset the start time if the state actually changes
    if (lean != newLean) {
        lastLean = lean;
        lean = newLean;
    }

    leanStartTime = currentMillis; // Reset the lean timer regardless of whether it changed
}

void showCalibrationScreen() {
  gfx->fillScreen(RED);
  gfx->setTextColor(WHITE);
  gfx->setTextSize(5);
  gfx->setCursor(10, 160);
  gfx->println("Dear User,");
  gfx->setCursor(10, 210);
  gfx->println("Calibration Mode");
  gfx->setCursor(10, 260);
  gfx->println("will restart shortly.");
  gfx->setTextSize(6); 
  gfx->setCursor(10, 400);
  gfx->println("Please hold...");
  delay(5000);

/*
  gfx->fillScreen(RED);
  gfx->setTextColor(WHITE);
  gfx->setTextSize(5);
  gfx->setCursor(10, 210);
  gfx->println("The cycle will shift");
  gfx->setCursor(10, 260);
  gfx->println("back by 1 cycle in");
  gfx->setTextSize(6); 
  gfx->setCursor(10, 400);
  gfx->println("5s");
  delay(1000);

  gfx->fillScreen(WHITE);
  gfx->setTextColor(RED);
  gfx->setTextSize(5);
  gfx->setCursor(10, 210);
  gfx->println("The cycle will shift");
  gfx->setCursor(10, 260);
  gfx->println("back by 1 cycle in");
  gfx->setTextSize(6); 
  gfx->setCursor(10, 400);
  gfx->println("4s");
  delay(1000);

  gfx->fillScreen(RED);
  gfx->setTextColor(WHITE);
  gfx->setTextSize(5);
  gfx->setCursor(10, 210);
  gfx->println("The cycle will shift");
  gfx->setCursor(10, 260);
  gfx->println("back by 1 cycle in");
  gfx->setTextSize(6); 
  gfx->setCursor(10, 400);
  gfx->println("3s");
  delay(1000);

  gfx->fillScreen(WHITE);
  gfx->setTextColor(RED);
  gfx->setTextSize(5);
  gfx->setCursor(10, 210);
  gfx->println("The cycle will shift");
  gfx->setCursor(10, 260);
  gfx->println("back by 1 cycle in");
  gfx->setTextSize(6); 
  gfx->setCursor(10, 400);
  gfx->println("2s");
  delay(1000);

  gfx->fillScreen(RED);
  gfx->setTextColor(WHITE);
  gfx->setTextSize(5);
  gfx->setCursor(10, 210);
  gfx->println("The cycle will shift");
  gfx->setCursor(10, 260);
  gfx->println("back by 1 cycle in");
  gfx->setTextSize(6); 
  gfx->setCursor(10, 400);
  gfx->println("1s");
  delay(1000);

  gfx->fillScreen(RED);
  gfx->setTextColor(WHITE);
  gfx->setTextSize(6); 
  gfx->setCursor(200, 400);
  gfx->println("Shifting Back...");
  delay(5000);




*/
  gfx->fillScreen(RED);
  gfx->setTextColor(WHITE);
  gfx->setTextSize(5);
  gfx->setCursor(10, 160);
  gfx->println("Dear User,");
  gfx->setCursor(10, 210);
  gfx->println("Please sit in your");
  gfx->setCursor(10, 260);
  gfx->println("ideal seating posture");
  gfx->setTextSize(6); 
  gfx->setCursor(10, 400);
  gfx->println("Calibrating.");
  delay(1000);
  gfx->setCursor(10, 400);
  gfx->println("Calibrating..");
  delay(1000);
  gfx->setCursor(10, 400);
  gfx->println("Calibrating...");
  delay(5000);

  gfx->fillScreen(MAGENTA);
  gfx->setTextColor(WHITE);
  gfx->setTextSize(5);
  gfx->setCursor(10, 210);
  gfx->println("Please adjust yourself to");
  gfx->setCursor(10, 260);
  gfx->println("be leaning to the left.");
  gfx->setTextSize(6); 
  gfx->setCursor(10, 400);
  gfx->println("Calibrating.");
  delay(1000);
  gfx->setCursor(10, 400);
  gfx->println("Calibrating..");
  delay(1000);
  gfx->setCursor(10, 400);
  gfx->println("Calibrating...");
  delay(5000);


  gfx->fillScreen(BLUE);
  gfx->setTextColor(WHITE);
  gfx->setTextSize(5);
  gfx->setCursor(10, 210);
  gfx->println("Please adjust yourself to");
  gfx->setCursor(10, 260);
  gfx->println("be leaning to the right.");
  gfx->setTextSize(6); 
  gfx->setCursor(10, 400);
  gfx->println("Calibrating.");
  delay(1000);
  gfx->setCursor(10, 400);
  gfx->println("Calibrating..");
  delay(1000);
  gfx->setCursor(10, 400);
  gfx->println("Calibrating...");
  delay(5000);


  gfx->fillScreen(GREEN);
  gfx->setTextColor(BLACK);
  gfx->setTextSize(5);
  gfx->setCursor(10, 210);
  gfx->println("Please adjust yourself to");
  gfx->setCursor(10, 260);
  gfx->println("be slouching forward.");
  gfx->setTextSize(6); 
  gfx->setCursor(10, 400);
  gfx->println("Calibrating.");
  delay(1000);
  gfx->setCursor(10, 400);
  gfx->println("Calibrating..");
  delay(1000);
  gfx->setCursor(10, 400);
  gfx->println("Calibrating...");
  delay(5000);


  gfx->fillScreen(GREEN);
  gfx->setTextColor(BLACK);
  gfx->setTextSize(5);
  gfx->setCursor(10, 260);
  gfx->println("Resuming current training.");
  gfx->setTextSize(6); 
  gfx->setCursor(10, 400);
  gfx->println("Please hold...");
  delay(5000);
  




}



//THESE ARE ALL THE WIFI RELATED FUNCTIONS

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&screen, incomingData, sizeof(screen));
  Serial.print("Bytes received: ");
  Serial.println(len);

slouch_calc();
lean_calc();
display_hexa_prox_sensors();
display_hexa_pressure_sensors();


}

void slouch_calc() {
  int top = 0; 
  int bot = 0; 
  if(screen.prox5_status != 4) {     // if not out of range    
  top = screen.prox5_value;
  }
  // print sensor six reading
  if(screen.prox6_status != 4) {     // if not out of range    
  bot = screen.prox6_value;
  }

  backrest_diff = top - bot;

  if ( backrest_diff < backrest_debugproxthreshold && backrest_diff > backrest_proxsensorsthreshold) {
    slouching = true;
  }
  else {
    slouching = false;
  }
}

void lean_calc() {

  int proxleft = 0;
  int proxright = 0;

  if(screen.prox2_status != 4) {     // if not out of range    
  proxleft = screen.prox2_value;
  }
  // print sensor six reading
  if(screen.prox4_status != 4) {     // if not out of range    
  proxright = screen.prox4_value;
  }

  int TLe = screen.force_sensor1;
  int BRi = screen.force_sensor3;

  seat_prox_diff = proxright - proxleft;
  seat_diff = BRi - TLe;
  if (seat_diff > PressurerightthresholdLeaning) {
      lean = RIGHT; 
    } 
    else if (seat_diff < PressureleftthresholdLeaning) {
      lean = LEFT;
    }
    else {
      lean = CENTRE;
    }
/*
  if (screen.outerlimit_on) {
    if (seat_diff < PressurerightthresholdLeaning) {
      lean = RIGHT; 
    } 
    else if (seat_diff > PressureleftthresholdLeaning) {
      lean = LEFT;
    }
    else {
      lean = CENTRE;
    }
  }
  else if (screen.outerlimit_off) {
    if (seat_prox_diff > ProxleftthresholdLeaning) {
      lean = LEFT;
    }
    else if (seat_prox_diff < ProxrightthresholdLeaning) {
      lean = RIGHT;
    }
    else {
      lean = CENTRE;
    }*/
  }


//Display TOF Sensor readings
void display_hexa_prox_sensors() {
  
  int top = 0; 
  int bot = 0; 
  int proxleft = 0;
  int proxright = 0;


  if(screen.prox5_status != 4) {     // if not out of range    
  top = screen.prox5_value;
  }
  // print sensor six reading
  if(screen.prox6_status != 4) {     // if not out of range    
  bot = screen.prox6_value;
  }

  if(screen.prox2_status != 4) {     // if not out of range    
  proxleft = screen.prox2_value;
  }
  // print sensor six reading
  if(screen.prox4_status != 4) {     // if not out of range    
  proxright = screen.prox4_value;
  }

/*
  if (top == 0 && bot == 0){
    slouching = false;
  }
  else if (abs(top - bot) > 1000){
    slouching = false;
  }
  else if (abs(top - bot) > proxsensorsthreshold) {
    slouching = true;
  }
  else {
    slouching = false;
  }*/
  
    //print out the status for debugging
  Serial.print("Top Sensor: ");
  Serial.println(top);
  Serial.print("Bottom Sensor: ");
  Serial.println(bot);
  Serial.print("Slouching: ");
  Serial.println(slouching ? "Yes" : "No");

  
  //print out the status for debugging
  Serial.print("Left Prox Sensor: ");
  Serial.println(proxleft);
  Serial.print("Right Prox Sensor: ");
  Serial.println(proxright);

}






//Display TOF Sensor readings
void display_hexa_pressure_sensors() {

  int TRi = screen.force_sensor4;
  
  int TLe = screen.force_sensor1;
  int BRi = screen.force_sensor3;
  
  int BLe = screen.force_sensor2;
  int topBackrest = screen.force_sensor5;
  int bottomBackrest = screen.force_sensor6;

  // Calculate total pressure on each side - We need to replace these values with the readings from the sensors lol
  int totalLeft = TLe + BLe;
  int totalRight = TRi + BRi;
  
//  int top = topBackrest;
//  int bot = bottomBackrest;

/*if asdf{
  if (TLe == 0 && BLe == 0 && TRi == 0 && BRi == 0) {
    lean = CENTRE;
  } else if (totalLeft > totalRight && (totalLeft - totalRight) > leftthresholdLeaning) {
    lean = LEFT;
  } else if (totalRight > totalLeft && (totalRight - totalLeft) > rightthresholdLeaning) {
    lean = RIGHT;
  } else {
    lean = CENTRE;  // Default to upright if no significant leaning detected
  }*/
/*
  int seat_diff = TLe - BRi;

  if (seat_diff < PressurerightthresholdLeaning) {
    lean = RIGHT;
  } 
  else if (seat_diff > PressureleftthresholdLeaning) {
    lean = LEFT;
  }
  else {
    lean = CENTRE;
  }*/
//}

  //print out the status for debugging
  Serial.print("Top PRESSURE Left: ");
  Serial.println(TLe);
  Serial.print("Bottom PRESSURE Left: ");
  Serial.println(BLe);

  Serial.print("Top PRESS SURE Right: ");
  Serial.println(TRi);
  Serial.print("Bottom PRESSURE Right: ");
  Serial.println(BRi);

  Serial.println();

}

// BELOW IS ALL THE RANDOM SAMPLE CODE THAT I DID NOT MANAGE TO PROPERLY DETERMINE IF NECESSARY SO IM JUST LEAVING IN
//-------------------------------------------------------------------------------------------------------------------
void pin_init()
{
  pinMode(TFT_BL, OUTPUT);
  pinMode(TOUCH_RST, OUTPUT);

  digitalWrite(TFT_BL, LOW);
  delay(100);
  digitalWrite(TOUCH_RST, LOW);
  delay(1000);
  digitalWrite(TOUCH_RST, HIGH);
  delay(1000);
  digitalWrite(TOUCH_RST, LOW);
  delay(1000);
  digitalWrite(TOUCH_RST, HIGH);
  delay(1000);
}


void touch_init(void)
{
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  ts.begin();
  ts.setRotation(TOUCH_ROTATION);
}

// For test
int32_t w, h, n, n1, cx, cy, cx1, cy1, cn, cn1;
uint8_t tsa, tsb, tsc, ds;

static inline uint32_t micros_start() __attribute__((always_inline));
static inline uint32_t micros_start()

{
  uint8_t oms = millis();
  while ((uint8_t)millis() == oms)
    ;
  return micros();
}


void setup()
{

  Serial.begin(115200);
  Serial.println("ESP32S3 7inch LCD");

//--------- Transmitted Stuff ------------
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
//--------- End of Transmitted Stuff ------------

  pin_init();
  touch_init();


  // Init Display
  gfx->begin();
  SD_MMC.setPins(PIN_SD_CLK, PIN_SD_CMD, PIN_SD_D0);
  if (!SD_MMC.begin("/sdcard", true, true))
  {

    while (1)
    {
      Serial.println("Card Mount Failed");
      delay(1000);
    }
  }

  gfx->fillScreen(CYAN);
  gfx->setTextColor(BLACK);
  gfx->setTextSize(4);
  gfx->setCursor(10, 20);
  gfx->println("Welcome! I'm your Posture Buddy!");
  gfx->setCursor(10, 60);
  gfx->println("I'm here to help you track");
  gfx->setCursor(10, 100);
  gfx->println("your posture progress!");
  gfx->setCursor(10, 300);
  gfx->println("Tracking will commence shortly.");
  gfx->setTextSize(6); 
  gfx->setCursor(10, 340);
  gfx->println("Please hold...");

  delay(5000);


  leanStartTime = millis();
  notslouchingStartTime = millis();
  

  xTaskCreatePinnedToCore(Checking, "Checking", 20480, NULL, 2, NULL, ARDUINO_RUNNING_CORE);
  Serial.println("done checking");
}

void loop(){

}