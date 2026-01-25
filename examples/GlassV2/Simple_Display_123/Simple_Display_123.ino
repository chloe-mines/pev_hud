/*
==============================================================================
                    T-GLASS V2 RACEBOX SPEED DISPLAY
==============================================================================

PURPOSE:
    Bluetooth speedometer for T-Glass smart glasses that connects to RaceBox
    GPS devices to display real-time speed on the glasses' AMOLED display.

FUNCTIONALITY:
    • Automatically scans for and connects to RaceBox GPS devices via BLE
    • Parses UBX protocol messages to extract precise speed data
    • Displays current speed in km/h on T-Glass lens-aligned display
    • Shows connection status, GPS fix quality, and satellite count
    • Handles automatic reconnection if connection is lost

DISPLAY STATES:
    INIT  - Initializing hardware and BLE stack
    BLE   - Setting up Bluetooth Low Energy
    SETUP - Configuring BLE scan parameters  
    SCAN  - Scanning for RaceBox devices
    FIND  - RaceBox device found
    CONN  - Connecting to RaceBox
    LINK  - Connected but waiting for GPS data
    WAIT  - Receiving data but GPS not ready
    XSnn  - GPS fix status X with nn satellites (0S05 = no fix, 5 sats)
    ###.# - Current speed in km/h (when GPS fix is stable)
    NONE  - No devices found
    TOUT  - Scan timeout
    FAIL  - Connection error

HARDWARE REQUIREMENTS:
    • LilyGO T-Glass V2 smart glasses
    • RaceBox or RaceBox Mini GPS device
    • Both devices must have BLE capability

SOFTWARE DEPENDENCIES:
    • LilyGo_Wristband library for T-Glass hardware control
    • LV_Helper for LVGL display management
    • NimBLEDevice for Bluetooth Low Energy communication

USAGE:
    1. Upload this sketch to T-Glass V2
    2. Power on both T-Glass and RaceBox devices
    3. T-Glass will automatically find and connect to RaceBox
    4. Speed will display on lens when GPS acquires fix

GPS DRIFT FIX (Jan 2026):
    • Added horizontal accuracy validation (rejects readings >50cm accuracy)
    • Fixed coordinate parsing order (longitude @ offset 24, latitude @ offset 28)
    • Atomic coordinate updates (coordsUpdated flag set only after BOTH coords read)
    • Added accuracy display in COORD_DEBUG mode
    • This prevents drift caused by poor-quality GPS readings being used for
      lap timing/distance calculations while speed remains accurate

AUTHOR: T-Glass Racing Project
DATE: January 2026
==============================================================================
*/

#include "Arduino.h"
#include <LilyGo_Wristband.h>
#include <LV_Helper.h>
#include "NimBLEDevice.h"
#include <EEPROM.h>
#include <nvs_flash.h>

// RaceBox BLE UUIDs
const char* UART_SERVICE_UUID = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E";
const char* UART_RX_UUID      = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E";
const char* UART_TX_UUID      = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E";

// Objects
LilyGo_Class amoled;
lv_obj_t *number_label;

// BLE variables (using working example.cpp pattern)
static BLEUUID serviceUUID("6E400001-B5A3-F393-E0A9-E50E24DCCA9E");
static BLEUUID txCharUUID("6E400003-B5A3-F393-E0A9-E50E24DCCA9E");

static bool doConnect = false;
static bool connected = false;
static NimBLEAdvertisedDevice* myRaceBox = nullptr;
static NimBLERemoteCharacteristic* pRemoteCharacteristic = nullptr;

// Parsed data from RaceBox
float currentSpeed = 0.0;  // Speed in km/h
bool speedUpdated = false;
bool gpsStable = false;  // Track if GPS is providing consistent data
unsigned long lastGpsUpdateTime = 0;  // Track when we last got GPS data
unsigned long gpsStableTime = 0;  // When GPS became stable
bool hasGpsFix = false;
int gpsFxCount = 0;
uint8_t lastFixStatusFlags = 0xFF; // Track for display
uint8_t lastFixStatus = 0xFF;      // GPS fix status (0=no fix, 2=2D, 3=3D)
uint8_t numSatellites = 0;         // Number of satellites
int dataPacketsReceived = 0;

// GPS Coordinates
double currentLatitude = 0.0;      // Current latitude in degrees
double currentLongitude = 0.0;     // Current longitude in degrees
bool coordsUpdated = false;        // Flag indicating new coordinates received
uint32_t horizontalAccuracy = 0;   // Horizontal accuracy estimate in mm
bool gpsAccuracyPoor = false;      // Flag indicating if recent GPS accuracy is poor

// GPS Timestamp
uint32_t currentGpsTime = 0;       // GPS time of week (iTOW) in milliseconds
bool gpsTimeUpdated = false;       // Flag indicating new GPS time received

// Connection status
enum ConnectionState {
  STATE_STARTUP,      // Waiting for user to start BLE sequence
  STATE_SCANNING,
  STATE_DEVICE_FOUND,
  STATE_CONNECTING,
  STATE_CONNECTED,
  STATE_ERROR,
  STATE_NO_DEVICE,
  STATE_SCAN_TIMEOUT
};
ConnectionState currentState = STATE_STARTUP;
String errorReason = "";

// Display Mode System for Smart Hybrid Interface
enum DisplayMode {
  DISPLAY_SPEED,      // Primary: Show current speed
  DISPLAY_GPS,        // GPS status (fix + satellites)
  DISPLAY_LAP_CTRL,   // Lap timing controls
  DISPLAY_SET_LINE,   // "SET LINE?" confirmation mode
  DISPLAY_LINE_SAVED, // "LINE SET!" confirmation
  DISPLAY_BAD_FIX,    // "BAD FIX" - GPS too poor to set line
  DISPLAY_GPS_DEBUG,   // Show captured GPS coordinates
  DISPLAY_COUNTDOWN_3, // Countdown 3
  DISPLAY_COUNTDOWN_2, // Countdown 2
  DISPLAY_COUNTDOWN_1, // Countdown 1
  DISPLAY_COUNTDOWN_GO, // "GO" message
  DISPLAY_EEPROM_DEBUG, // Show EEPROM saved coordinates
  DISPLAY_BATTERY,    // T-Glass battery level
  DISPLAY_DUAL_BATTERY, // Both batteries on two lines
  DISPLAY_RB_BATTERY, // RaceBox battery level
  DISPLAY_BEST_LAP,   // Best lap of session
  DISPLAY_COORD_DEBUG, // Show raw coordinate values for debugging
  DISPLAY_DISTANCE_FROM_START, // Distance from start point (no timeout)
  DISPLAY_LAP_TIMER,   // Current lap duration timer (no timeout)
  DISPLAY_LAP_DEBUG,   // Distance + lap timer combined (2 lines)
  DISPLAY_LAP_SPEED,   // Lap timer + speed combined (2 lines, no timeout)
  DISPLAY_LAP_FLASH,   // Temporary lap time display ("LAP TIME:" + time)
  DISPLAY_LAP_DELTA,  // Delta comparison display (delta + time)
  DISPLAY_RESET,      // Reset menu option
  DISPLAY_RESET_CONFIRM // "SURE?" confirmation for reset
};
DisplayMode currentDisplayMode = DISPLAY_SPEED;

// Timing variables
unsigned long lastUpdate = 0;
const unsigned long updateInterval = 1000; // 1 second for status updates
unsigned long lastSpeedUpdate = 0;
const unsigned long speedUpdateInterval = 100; // 100ms for smooth speed updates
unsigned long lastLapTimerUpdate = 0;
const unsigned long lapTimerUpdateInterval = 10; // 10ms for very smooth lap timer updates
unsigned long scanStartTime = 0;
unsigned long stateChangeTime = 0; // Track when state changes occur
const unsigned long scanTimeout = 10000; // 10 seconds scan timeout

// Display Mode Variables
unsigned long lastDisplayModeChange = 0;
const unsigned long displayModeTimeout = 10000; // 10 seconds auto-return to speed
unsigned long displayModeStartTime = 0; // For reset confirmation timeout
unsigned long lastButtonPress = 0;
const unsigned long buttonDebounce = 50; // 50ms button debounce
const unsigned long longPressDuration = 500; // 500ms threshold for long press
const unsigned long doubleTapWindow = 400; // 400ms window for double tap
unsigned long lapFlashStart = 0;
const unsigned long lapFlashDuration = 5000; // 5 seconds to show lap time + delta
bool isLapFlashing = false;
String bestLapTime = "---.--"; // Best lap of session
int raceBoxBattery = -1; // RaceBox battery percentage (-1 = unknown)

// Lap timing state
bool lapInProgress = false;
uint32_t lapStartTime = 0;         // GPS time when lap started (for accuracy) - must match currentGpsTime type
unsigned long lapStartMillis = 0;      // System millis() when lap started (for smooth display)
uint32_t lastLapTime = 0;           // Last lap time in milliseconds - must match GPS time units
uint32_t bestLapTimeMs = 0; // Best lap in milliseconds (0 = no best yet)
bool wasNearFinishLine = false;
String currentLapTimeStr = "";
String deltaStr = ""; // Delta from best lap (+2.3 or -1.5 format)

// Countdown timing for debugging
unsigned long countdownStartTime = 0;
unsigned long badFixMessageStart = 0;  // Timer for BAD FIX message
const unsigned long countdownStepDuration = 1000; // 1 second per step

// Debug counters
unsigned long coordsUpdateCounter = 0;
unsigned long lapCheckCounter = 0;

// Finish Line Storage
double finishLineLat = 0.0;
double finishLineLon = 0.0;
bool finishLineSet = false;
const double crossingThreshold = 3.0; // 3 meters crossing detection (tight for 10Hz GPS)
const double minDistanceForNextLap = 6.0; // Must be this far away before next lap can trigger

// Finish line averaging (capture multiple readings for better accuracy)
const int finishLineReadingsCount = 10; // Average 10 GPS readings
double finishLineReadingsLat[10];
double finishLineReadingsLon[10];
int finishLineReadingsIndex = 0;
bool finishLineCapturing = false;
unsigned long finishLineCaptureStart = 0;
const unsigned long finishLineCaptureDuration = 3000; // 3 seconds to capture readings

// Speed stabilization
float stabilizedSpeed = 0.0;
const float speedThreshold = 0.5; // Below this, show 0.0
const float speedSmoothingFactor = 0.3; // Smoothing factor (0.0 = no smoothing, 1.0 = instant)

// BLE Device Callback (using working example.cpp pattern)
class MyAdvertisedDeviceCallbacks: public NimBLEAdvertisedDeviceCallbacks {
  void onResult(NimBLEAdvertisedDevice* advertisedDevice) {
    String deviceName = advertisedDevice->getName().c_str();
    String deviceAddr = advertisedDevice->getAddress().toString().c_str();
    
    // Log ALL devices found (for debugging)
    Serial.printf("Found device: '%s' (%s) RSSI: %d\n", 
                  deviceName.c_str(), deviceAddr.c_str(), advertisedDevice->getRSSI());
    
    // Check if device name contains "racebox" (case insensitive)
    deviceName.toLowerCase();
    if (deviceName.indexOf("racebox") >= 0) {
      Serial.printf("*** RACEBOX FOUND: %s ***\n", advertisedDevice->toString().c_str());
      
      // Store the device reference (key difference from our previous approach)
      myRaceBox = advertisedDevice;
      doConnect = true;
      
      // Stop scanning
      NimBLEDevice::getScan()->stop();
      currentState = STATE_DEVICE_FOUND;
      stateChangeTime = millis();
    }
  }
};

// BLE Client Callback
class MyClientCallback : public NimBLEClientCallbacks {
  void onConnect(NimBLEClient* pclient) {
    connected = true;
    currentState = STATE_CONNECTED;
    stateChangeTime = millis();
    Serial.println("Connected to RaceBox");
  }

  void onDisconnect(NimBLEClient* pclient) {
    connected = false;
    doConnect = false;
    errorReason = "Disconnected";
    currentState = STATE_ERROR;
    stateChangeTime = millis();
    Serial.println("Disconnected from RaceBox");
  }
};

// Reset function to clear BLE cache and reboot
void performSystemReset() {
    Serial.println("*** PERFORMING SYSTEM RESET ***");
    
    // Show reset in progress
    lv_label_set_text(number_label, "RESET..");
    lv_timer_handler();
    amoled.update();
    delay(1000);
    
    // Clear BLE bonds and deinitialize
    Serial.println("Clearing BLE bonds...");
    if (connected) {
        // Disconnect first if connected
        NimBLEDevice::getClientList()->front()->disconnect();
        delay(500);
    }
    
    // Delete all BLE bonds
    NimBLEDevice::deleteAllBonds();
    delay(500);
    
    // Deinitialize BLE stack
    NimBLEDevice::deinit();
    delay(500);
    
    // Clear NVS (Non-Volatile Storage)
    Serial.println("Clearing NVS...");
    nvs_flash_erase();
    delay(500);
    
    // Show completion
    lv_label_set_text(number_label, "DONE!");
    lv_timer_handler();
    amoled.update();
    delay(1000);
    
    Serial.println("*** RESET COMPLETE - REBOOTING ***");
    
    // Restart the ESP32
    ESP.restart();
}

// Calculate distance between two GPS points using Haversine formula
double calculateDistance(double lat1, double lon1, double lat2, double lon2) {
    const double R = 6371000; // Earth's radius in meters
    double dLat = (lat2 - lat1) * PI / 180.0;
    double dLon = (lon2 - lon1) * PI / 180.0;
    double a = sin(dLat/2) * sin(dLat/2) + 
               cos(lat1 * PI / 180.0) * cos(lat2 * PI / 180.0) * 
               sin(dLon/2) * sin(dLon/2);
    double c = 2 * atan2(sqrt(a), sqrt(1-a));
    return R * c; // Distance in meters
}

void checkLapCrossing() {
    // Increment lap check counter for debugging
    lapCheckCounter++;
    
    // Only check if we have valid GPS coordinates and a finish line set
    if (!coordsUpdated || finishLineLat == 0.0 || finishLineLon == 0.0) {
        return;
    }
    
    // Calculate distance to finish line
    double distanceToFinish = calculateDistance(currentLatitude, currentLongitude, 
                                               finishLineLat, finishLineLon);
    
    // Use hysteresis: must be far away before allowing next crossing
    static bool wasFarAway = false;
    static double maxDistanceFromLine = 0.0;
    
    // Track maximum distance from finish line
    if (distanceToFinish > maxDistanceFromLine) {
        maxDistanceFromLine = distanceToFinish;
    }
    
    // Consider "far away" if we've moved beyond the minimum distance
    if (distanceToFinish > minDistanceForNextLap) {
        wasFarAway = true;
    }
    
    bool nearFinishLine = (distanceToFinish <= crossingThreshold);
    
    // Debug output (more frequent and detailed for debugging lap issues)
    static unsigned long lastCrossingDebug = 0;
    if (millis() - lastCrossingDebug > 1000) { // Every 1 second for better debugging
        Serial.printf("LAP DEBUG: Dist=%.2fm, MaxDist=%.1fm, Near=%s, FarAway=%s, LapActive=%s, Thresh=%.1fm\n", 
                     distanceToFinish, maxDistanceFromLine,
                     nearFinishLine ? "Y" : "N", 
                     wasFarAway ? "Y" : "N",
                     lapInProgress ? "Y" : "N",
                     crossingThreshold);
        lastCrossingDebug = millis();
    }
    
    // Detect crossing: was far away, now near (with hysteresis)
    if (nearFinishLine && wasFarAway) {
        if (!lapInProgress) {
            // Lap should already be started after GO! - this case should rarely happen
            // But start a lap anyway if somehow we get here
            lapInProgress = true;
            lapStartTime = currentGpsTime;
            lapStartMillis = millis(); // Capture system time for smooth display
            wasFarAway = false; // Reset hysteresis
            maxDistanceFromLine = 0.0;
            Serial.printf("=== LAP STARTED (unexpected - should start after GO!) (dist=%.1fm) ===\n", distanceToFinish);
        } else {
            // Complete current lap and start new one
            lastLapTime = currentGpsTime - lapStartTime;
            
            // Convert to readable time format (minutes:seconds.tenths)
            unsigned long totalMs = lastLapTime;
            unsigned long minutes = totalMs / 60000;
            unsigned long seconds = (totalMs % 60000) / 1000;
            unsigned long tenths = (totalMs % 1000) / 100;
            
            char lapTimeBuffer[32];
            snprintf(lapTimeBuffer, sizeof(lapTimeBuffer), "%lu:%02lu.%lu", 
                    minutes, seconds, tenths);
            currentLapTimeStr = String(lapTimeBuffer);
            
            // Calculate delta from best lap
            if (bestLapTimeMs == 0 || lastLapTime < bestLapTimeMs) {
                // This is the new best lap!
                bestLapTimeMs = lastLapTime;
                bestLapTime = String(lapTimeBuffer); // Update the display string too!
                deltaStr = "BEST LAP";
                Serial.printf("=== NEW BEST LAP: %s (dist=%.1fm) ===\n", 
                             currentLapTimeStr.c_str(), distanceToFinish);
            } else {
                // Calculate delta (positive means slower, negative means faster)
                int32_t deltaMs = (int32_t)lastLapTime - (int32_t)bestLapTimeMs;
                float deltaSec = deltaMs / 1000.0;
                char deltaBuffer[16];
                if (deltaSec >= 0) {
                    snprintf(deltaBuffer, sizeof(deltaBuffer), "+%.1f", deltaSec);
                } else {
                    snprintf(deltaBuffer, sizeof(deltaBuffer), "%.1f", deltaSec);
                }
                deltaStr = String(deltaBuffer);
                Serial.printf("=== LAP COMPLETE: %s (delta: %s) (dist=%.1fm) ===\n", 
                             currentLapTimeStr.c_str(), deltaStr.c_str(), distanceToFinish);
            }
            
            // Flash the lap time
            isLapFlashing = true;
            lapFlashStart = millis();
            currentDisplayMode = DISPLAY_LAP_FLASH;
            
            // Start new lap
            lapStartTime = currentGpsTime;
            lapStartMillis = millis(); // Capture system time for smooth display
            wasFarAway = false; // Reset hysteresis
            maxDistanceFromLine = 0.0;
        }
    }
    
    wasNearFinishLine = nearFinishLine;
}

// Save finish line to EEPROM
void saveFinishLine() {
    EEPROM.begin(32);
    EEPROM.put(0, finishLineLat);
    EEPROM.put(8, finishLineLon);
    EEPROM.put(16, true); // finishLineSet flag
    EEPROM.commit();
    Serial.printf("Finish line saved: %.7f, %.7f\n", finishLineLat, finishLineLon);
}

// Load finish line from EEPROM
void loadFinishLine() {
    EEPROM.begin(32);
    EEPROM.get(0, finishLineLat);
    EEPROM.get(8, finishLineLon);
    EEPROM.get(16, finishLineSet);
    if (finishLineSet) {
        Serial.printf("Finish line loaded: %.7f, %.7f\n", finishLineLat, finishLineLon);
    } else {
        Serial.println("No finish line saved");
    }
}

// Font switching helper function
void setDisplayFont(bool useLargeFont) {
    if (useLargeFont) {
        #if LV_FONT_MONTSERRAT_16
        lv_obj_set_style_text_font(number_label, &lv_font_montserrat_16, 0);
        #elif LV_FONT_MONTSERRAT_14
        lv_obj_set_style_text_font(number_label, &lv_font_montserrat_14, 0);
        #else
        lv_obj_set_style_text_font(number_label, LV_FONT_DEFAULT, 0);
        #endif
    } else {
        #if LV_FONT_MONTSERRAT_12
        lv_obj_set_style_text_font(number_label, &lv_font_montserrat_12, 0);
        #elif LV_FONT_MONTSERRAT_10
        lv_obj_set_style_text_font(number_label, &lv_font_montserrat_10, 0);
        #else
        lv_obj_set_style_text_font(number_label, LV_FONT_DEFAULT, 0);
        #endif
    }
}

// Handle long press (menu cycling)
void handleLongPress() {
    unsigned long currentTime = millis();
    
    // Allow RESET option even when not connected, but other menus need connection
    if (currentState != STATE_CONNECTED && currentDisplayMode != DISPLAY_SPEED && currentDisplayMode != DISPLAY_RESET && currentDisplayMode != DISPLAY_RESET_CONFIRM) {
        return;
    }

    DisplayMode previousMode = currentDisplayMode;
    
    switch (currentDisplayMode) {
        case DISPLAY_SPEED:
            if (currentState == STATE_CONNECTED) {
                currentDisplayMode = DISPLAY_LAP_CTRL;
            } else {
                // When not connected, go directly to RESET option
                currentDisplayMode = DISPLAY_RESET;
            }
            break;
        case DISPLAY_LAP_CTRL:
            currentDisplayMode = DISPLAY_BATTERY;
            break;
        case DISPLAY_BATTERY:
        case DISPLAY_DUAL_BATTERY:
            currentDisplayMode = DISPLAY_COORD_DEBUG;
            break;
        case DISPLAY_COORD_DEBUG:
            currentDisplayMode = DISPLAY_DISTANCE_FROM_START;
            break;
        case DISPLAY_DISTANCE_FROM_START:
            currentDisplayMode = DISPLAY_LAP_TIMER;
            break;
        case DISPLAY_LAP_TIMER:
            currentDisplayMode = DISPLAY_LAP_DEBUG;
            break;
        case DISPLAY_LAP_DEBUG:
            currentDisplayMode = DISPLAY_LAP_SPEED;
            break;
        case DISPLAY_LAP_SPEED:
            currentDisplayMode = DISPLAY_RESET;
            break;
        case DISPLAY_RESET:
        case DISPLAY_RESET_CONFIRM:
            currentDisplayMode = DISPLAY_SPEED;
            break;
        case DISPLAY_SET_LINE:
            // Cancel line setting
            currentDisplayMode = DISPLAY_LAP_CTRL;
            Serial.println("Line setting cancelled");
            break;
        case DISPLAY_LINE_SAVED:
            currentDisplayMode = DISPLAY_LAP_CTRL;
            break;
        case DISPLAY_BAD_FIX:
            // Allow immediate return to lap control on button press
            currentDisplayMode = DISPLAY_LAP_CTRL;
            break;
        case DISPLAY_LAP_FLASH:
        case DISPLAY_LAP_DELTA:
            currentDisplayMode = DISPLAY_LAP_TIMER;
            isLapFlashing = false; // Stop the lap flash sequence
            break;
        default:
            // For any other modes, return to speed
            currentDisplayMode = DISPLAY_SPEED;
            break;
    }
    
    lastDisplayModeChange = millis();
    isLapFlashing = false;
    
    Serial.print("Long press: Changed from mode ");
    Serial.print((int)previousMode);
    Serial.print(" to mode ");
    Serial.println((int)currentDisplayMode);
    
    // Force display update immediately
    updateDisplayContent();
    updateDisplayContent();
    Serial.printf("Long press: %d -> %d\n", previousMode, currentDisplayMode);
}

// Handle double tap (lap timing functions)
void handleDoubleTap() {
    // Allow RESET functionality even when not connected
    if (currentState != STATE_CONNECTED && currentDisplayMode != DISPLAY_RESET && currentDisplayMode != DISPLAY_RESET_CONFIRM) {
        return;
    }
    
    switch (currentDisplayMode) {
        case DISPLAY_LAP_CTRL:
            // Enter line setting mode
            currentDisplayMode = DISPLAY_SET_LINE;
            Serial.println("Entering line setting mode");
            break;
        case DISPLAY_SET_LINE:
            // Start capturing finish line (average multiple readings)
            if (coordsUpdated || (currentLatitude != 0.0 && currentLongitude != 0.0)) {
                finishLineCapturing = true;
                finishLineCaptureStart = millis();
                finishLineReadingsIndex = 0;
                Serial.println("Started capturing finish line (averaging 10 readings over 3 seconds)...");
                currentDisplayMode = DISPLAY_LINE_SAVED; // Show "LINE SET!" while capturing
            } else {
                Serial.println("No GPS coordinates available");
                currentDisplayMode = DISPLAY_LAP_CTRL; // Cancel if no GPS
            }
            break;
        case DISPLAY_BATTERY:
            // Switch to dual battery display
            currentDisplayMode = DISPLAY_DUAL_BATTERY;
            Serial.println("Switching to dual battery display");
            break;
        case DISPLAY_DUAL_BATTERY:
            // Switch back to single battery display
            currentDisplayMode = DISPLAY_BATTERY;
            Serial.println("Switching to single battery display");
            break;
        case DISPLAY_RESET:
            // Double tap in reset mode - show confirmation
            currentDisplayMode = DISPLAY_RESET_CONFIRM;
            displayModeStartTime = millis();
            Serial.println("Reset confirmation requested");
            break;
        case DISPLAY_RESET_CONFIRM:
            // Double tap in reset confirmation - execute reset
            performSystemReset();
            break;
        default:
            // Double tap in other modes - ignore
            Serial.println("Double tap ignored in this mode");
            return;
    }
    
    lastDisplayModeChange = millis();
    updateDisplayContent();
}

// Initialize BLE sequence (called when user long presses in startup mode)
void initializeBLESequence() {
    Serial.println("User confirmed - starting BLE sequence...");
    
    // Setup scanner with working parameters
    lv_label_set_text(number_label, "SETUP");
    lv_timer_handler();
    amoled.update();
    NimBLEScan* pBLEScan = NimBLEDevice::getScan();
    pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
    pBLEScan->setInterval(45);
    pBLEScan->setWindow(15);
    pBLEScan->setActiveScan(true);
    delay(500); // Brief pause to show setup
    
    // Start scanning for RaceBox
    lv_label_set_text(number_label, "SCAN.");
    lv_timer_handler();
    amoled.update();
    scanStartTime = millis();
    currentState = STATE_SCANNING; // Move to scanning state
    Serial.println("Starting NimBLE scan for RaceBox devices...");
    pBLEScan->start(0, false); // Scan indefinitely until we find a device
}

void setup()
{
    Serial.begin(115200);
    delay(800);
    
    // Load finish line from EEPROM
    loadFinishLine();

    // Initialize the AMOLED display
    bool res = amoled.begin();
    if (!res)
    {
        while (1)
        {
            Serial.println("The board model cannot be detected, please raise the Core Debug Level to an error");
            delay(1000);
        }
    }

    // Configure display settings
    amoled.setRotation(0);
    amoled.setBrightness(255);

    // Initialize LVGL helper
    beginLvglHelper(amoled, false);

    // Clear the screen and set black background
    lv_obj_clean(lv_scr_act());
    lv_obj_set_style_bg_color(lv_scr_act(), lv_color_black(), 0);
    lv_obj_set_style_bg_opa(lv_scr_act(), LV_OPA_COVER, 0);

    // Create a label to display status - show INIT immediately
    number_label = lv_label_create(lv_scr_act());
    lv_label_set_text(number_label, "INIT");
    lv_obj_set_style_text_color(number_label, lv_color_white(), 0);

    // Use a smaller font for BLE status messages
    #if LV_FONT_MONTSERRAT_20
    lv_obj_set_style_text_font(number_label, &lv_font_montserrat_20, 0);
    #elif LV_FONT_MONTSERRAT_18
    lv_obj_set_style_text_font(number_label, &lv_font_montserrat_18, 0);
    #elif LV_FONT_MONTSERRAT_16
    lv_obj_set_style_text_font(number_label, &lv_font_montserrat_16, 0);
    #elif LV_FONT_MONTSERRAT_14
    lv_obj_set_style_text_font(number_label, &lv_font_montserrat_14, 0);
    #else
    lv_obj_set_style_text_font(number_label, LV_FONT_DEFAULT, 0);
    #endif

    // Position the label aligned with the lens
    lv_obj_align(number_label, LV_ALIGN_CENTER, -25, 55);
    
    // Update display to show INIT status
    lv_timer_handler();
    amoled.update();
    delay(500); // Brief pause to show INIT

    // Initialize BLE (using working example.cpp pattern)
    lv_label_set_text(number_label, "BLE");
    lv_timer_handler();
    amoled.update();
    Serial.println("Initializing NimBLE...");
    NimBLEDevice::init("T-Glass");
    delay(500); // Brief pause to show BLE init
    
    // PAUSE HERE - Show "START?" and wait for long press before continuing
    lv_label_set_text(number_label, "START?");
    lv_timer_handler();
    amoled.update();
    Serial.println("Display ready - Long press to start BLE connection sequence");
    
    // Don't proceed with BLE setup until user confirms
    // The rest will happen in the loop() when they long press
}

void loop()
{
    // Get current time for all timing operations
    unsigned long currentTime = millis();
    
    // Update the display
    amoled.update();
    lv_timer_handler();

    // Check for button press (boot button on T-Glass) - long press vs quick double tap
    static bool lastButtonState = HIGH;
    static unsigned long buttonPressStartTime = 0;
    static unsigned long lastQuickTapTime = 0;
    static bool waitingForSecondTap = false;
    bool currentButtonState = digitalRead(0);
    if (lastButtonState == HIGH && currentButtonState == LOW) {
        buttonPressStartTime = millis();
    }
    // Detect button release (LOW to HIGH transition)  
    else if (lastButtonState == LOW && currentButtonState == HIGH) {
        unsigned long pressDuration = millis() - buttonPressStartTime;
        
        // Debounce check
        if (pressDuration > buttonDebounce) {
            if (pressDuration >= longPressDuration) {
                // Long press detected
                if (currentState == STATE_STARTUP) {
                    // User wants to start BLE sequence
                    initializeBLESequence();
                } else {
                    // Normal long press - menu cycling
                    handleLongPress();
                }
                waitingForSecondTap = false; // Cancel any pending double tap
            } else {
                // Quick tap - check for double tap (but only if not in startup)
                if (currentState != STATE_STARTUP) {
                    unsigned long currentTime = millis();
                    if (waitingForSecondTap && (currentTime - lastQuickTapTime) < doubleTapWindow) {
                        // Second quick tap - double tap detected!
                        handleDoubleTap();
                        waitingForSecondTap = false;
                    } else {
                        // First quick tap - wait for potential second tap
                        lastQuickTapTime = currentTime;
                        waitingForSecondTap = true;
                    }
                }
            }
        }
    }
    
    // Handle single quick tap timeout (no second tap came)
    if (waitingForSecondTap && (millis() - lastQuickTapTime) > doubleTapWindow) {
        waitingForSecondTap = false;
        // Single quick tap - ignore for now (could add functionality later)
    }
    
    lastButtonState = currentButtonState;
    
    // Handle countdown sequence for GPS debugging
    if ((currentDisplayMode == DISPLAY_LINE_SAVED || 
         currentDisplayMode == DISPLAY_GPS_DEBUG ||
         currentDisplayMode == DISPLAY_COUNTDOWN_3 ||
         currentDisplayMode == DISPLAY_COUNTDOWN_2 ||
         currentDisplayMode == DISPLAY_COUNTDOWN_1 ||
         currentDisplayMode == DISPLAY_COUNTDOWN_GO) && 
        countdownStartTime > 0) {
        
        unsigned long elapsed = currentTime - countdownStartTime;
        
        if (currentDisplayMode == DISPLAY_LINE_SAVED && elapsed >= countdownStepDuration) {
            currentDisplayMode = DISPLAY_COUNTDOWN_3;
        } else if (currentDisplayMode == DISPLAY_COUNTDOWN_3 && elapsed >= countdownStepDuration * 2) {
            currentDisplayMode = DISPLAY_COUNTDOWN_2;
        } else if (currentDisplayMode == DISPLAY_COUNTDOWN_3 && elapsed >= countdownStepDuration * 2) {
            currentDisplayMode = DISPLAY_COUNTDOWN_2;
        } else if (currentDisplayMode == DISPLAY_COUNTDOWN_2 && elapsed >= countdownStepDuration * 3) {
            currentDisplayMode = DISPLAY_COUNTDOWN_1;
        } else if (currentDisplayMode == DISPLAY_COUNTDOWN_1 && elapsed >= countdownStepDuration * 4) {
            currentDisplayMode = DISPLAY_COUNTDOWN_GO;
        } else if (currentDisplayMode == DISPLAY_COUNTDOWN_GO && elapsed >= countdownStepDuration * 5) {
            currentDisplayMode = DISPLAY_LAP_DEBUG;
            countdownStartTime = 0; // Reset countdown timer
            
            // Start the lap timer immediately after GO! for better UX
            lapInProgress = true;
            lapStartTime = currentGpsTime;
            lapStartMillis = millis();
            Serial.println("GO! - Starting lap timer immediately");
        }
    }
    
    // Handle display mode auto-return to speed (but not during countdown sequence or distance mode)
    if (currentState == STATE_CONNECTED && 
        currentDisplayMode != DISPLAY_SPEED && 
        currentDisplayMode != DISPLAY_LAP_FLASH &&
        currentDisplayMode != DISPLAY_LAP_DELTA &&
        currentDisplayMode != DISPLAY_LINE_SAVED &&
        currentDisplayMode != DISPLAY_COUNTDOWN_3 &&
        currentDisplayMode != DISPLAY_COUNTDOWN_2 &&
        currentDisplayMode != DISPLAY_COUNTDOWN_1 &&
        currentDisplayMode != DISPLAY_COUNTDOWN_GO &&
        currentDisplayMode != DISPLAY_DISTANCE_FROM_START && // Distance mode stays active
        currentDisplayMode != DISPLAY_LAP_TIMER && // Lap timer stays active
        currentDisplayMode != DISPLAY_LAP_DEBUG && // Lap debug stays active
        currentDisplayMode != DISPLAY_LAP_SPEED && // Lap speed stays active
        currentDisplayMode != DISPLAY_RESET && // Reset menu stays active
        currentDisplayMode != DISPLAY_RESET_CONFIRM && // Reset confirmation stays active
        countdownStartTime == 0 &&
        currentTime - lastDisplayModeChange > displayModeTimeout) {
        currentDisplayMode = DISPLAY_SPEED;
        Serial.println("Auto-returning to speed display");
    }
    
    // Handle lap flash timeout and transition to delta
    if (isLapFlashing && currentTime - lapFlashStart > lapFlashDuration) {
        if (currentDisplayMode == DISPLAY_LAP_FLASH) {
            // Transition from lap time to delta display
            currentDisplayMode = DISPLAY_LAP_DELTA;
            Serial.println("Transitioning to delta display");
        } else if (currentDisplayMode == DISPLAY_LAP_DELTA) {
            // Finished showing delta, return to lap debug (duration + distance)
            isLapFlashing = false;
            currentDisplayMode = DISPLAY_LAP_DEBUG;
            Serial.println("Lap display complete - returning to lap debug");
        }
    }

    // Check if it's time to update the status
    if (currentTime - lastUpdate >= updateInterval) {
        updateConnectionStatus();
        lastUpdate = currentTime;
    }
    
    // Rapid speed updates when in speed display mode and connected
    if (currentState == STATE_CONNECTED && 
        currentDisplayMode == DISPLAY_SPEED && 
        currentTime - lastSpeedUpdate >= speedUpdateInterval) {
        updateDisplayContent();
        lastSpeedUpdate = currentTime;
    }
    
    // Very rapid lap timer updates for super smooth counting (every 10ms)
    if (currentState == STATE_CONNECTED && 
        (currentDisplayMode == DISPLAY_LAP_TIMER || 
         currentDisplayMode == DISPLAY_LAP_DEBUG || 
         currentDisplayMode == DISPLAY_LAP_SPEED) &&
        currentTime - lastLapTimerUpdate >= lapTimerUpdateInterval) {
        updateDisplayContent();
        lastLapTimerUpdate = currentTime;
    }

    // Handle BLE connection state machine
    handleBLEStateMachine();

    // Small delay to prevent excessive processing
    delay(10);
}

// Smart display content based on current mode
void updateDisplayContent() {
    // Allow RESET display even when not connected
    if (currentState != STATE_CONNECTED && currentDisplayMode != DISPLAY_RESET && currentDisplayMode != DISPLAY_RESET_CONFIRM) {
        return; // Only update content when connected (except for RESET)
    }
    
    char displayText[32];  // Increased buffer size
    bool useLargeFont = true;
    
    switch (currentDisplayMode) {
        case DISPLAY_SPEED:
            if (gpsStable) {
                // Show speed with lap status indicator (removed coordinate indicator)
                if (wasNearFinishLine && lapInProgress) {
                    snprintf(displayText, sizeof(displayText), "%.1f*", currentSpeed); // * means near line, lap active
                } else if (wasNearFinishLine) {
                    snprintf(displayText, sizeof(displayText), "%.1f+", currentSpeed); // + means near line  
                } else {
                    snprintf(displayText, sizeof(displayText), "%.1f", currentSpeed);
                }
            } else {
                // Animate FIXING with dots
                static unsigned long lastFixingAnimation = 0;
                static int fixingDots = 1;
                if (millis() - lastFixingAnimation > 500) { // Change every 500ms
                    fixingDots = (fixingDots % 3) + 1;
                    lastFixingAnimation = millis();
                }
                
                if (fixingDots == 1) {
                    snprintf(displayText, sizeof(displayText), "FIXING.");
                } else if (fixingDots == 2) {
                    snprintf(displayText, sizeof(displayText), "FIXING..");
                } else {
                    snprintf(displayText, sizeof(displayText), "FIXING...");
                }
            }
            useLargeFont = true;
            break;
            
        case DISPLAY_GPS:
            if (dataPacketsReceived == 0) {
                snprintf(displayText, sizeof(displayText), "NO GPS");
                useLargeFont = false;
            } else {
                snprintf(displayText, sizeof(displayText), "%dS%02d", lastFixStatus, numSatellites);
                useLargeFont = true;
            }
            break;
            
        case DISPLAY_LAP_CTRL:
            if (finishLineSet) {
                snprintf(displayText, sizeof(displayText), "LAP RDY");
            } else {
                snprintf(displayText, sizeof(displayText), "LAP OFF");
            }
            useLargeFont = false;
            break;
            
        case DISPLAY_SET_LINE:
            snprintf(displayText, sizeof(displayText), "SET LINE?");
            useLargeFont = false;
            break;
            
        case DISPLAY_LINE_SAVED:
            snprintf(displayText, sizeof(displayText), "LINE SET!");
            useLargeFont = false;
            break;
            
        case DISPLAY_BAD_FIX:
            snprintf(displayText, sizeof(displayText), "BAD FIX");
            useLargeFont = false;
            // Auto-return to lap control after 2 seconds
            if (millis() - badFixMessageStart > 2000) {
                currentDisplayMode = DISPLAY_LAP_CTRL;
            }
            break;
            
        case DISPLAY_COUNTDOWN_3:
            snprintf(displayText, sizeof(displayText), "3");
            useLargeFont = true;
            break;
            
        case DISPLAY_COUNTDOWN_2:
            snprintf(displayText, sizeof(displayText), "2");
            useLargeFont = true;
            break;
            
        case DISPLAY_COUNTDOWN_1:
            snprintf(displayText, sizeof(displayText), "1");
            useLargeFont = true;
            break;
            
        case DISPLAY_COUNTDOWN_GO:
            snprintf(displayText, sizeof(displayText), "GO!");
            useLargeFont = true;
            break;
            
        case DISPLAY_EEPROM_DEBUG: {
            // Show distance to finish line and debug counters
            double storedLat = 0.0, storedLon = 0.0;
            bool storedSet = false;
            
            // Read directly from EEPROM
            EEPROM.begin(32);
            EEPROM.get(0, storedLat);
            EEPROM.get(8, storedLon);
            EEPROM.get(16, storedSet);
            
            if (storedSet && (storedLat != 0.0 || storedLon != 0.0) && 
                currentLatitude != 0.0 && currentLongitude != 0.0) {
                // Calculate and show distance to finish line
                double dist = calculateDistance(currentLatitude, currentLongitude, storedLat, storedLon);
                snprintf(displayText, sizeof(displayText), "D:%.1fm C:%lu", dist, coordsUpdateCounter);
            } else {
                snprintf(displayText, sizeof(displayText), "E:NONE C:%lu", coordsUpdateCounter);
            }
            useLargeFont = false;
            break;
        }
            
        case DISPLAY_BATTERY: {
            // Battery heading - no percentage shown
            snprintf(displayText, sizeof(displayText), "BAT");
            useLargeFont = false;
            break;
        }
        
        case DISPLAY_DUAL_BATTERY: {
            // Both batteries on two lines (smaller font)
            int batteryPercent = amoled.getBatteryPercent();
            if (raceBoxBattery >= 0) {
                snprintf(displayText, sizeof(displayText), "TG:%d%%\nRB:%d%%", batteryPercent, raceBoxBattery);
            } else {
                snprintf(displayText, sizeof(displayText), "TG:%d%%\nRB:???", batteryPercent);
            }
            useLargeFont = false;
            break;
        }
        
        case DISPLAY_RB_BATTERY: {
            // RaceBox battery level
            if (raceBoxBattery >= 0) {
                snprintf(displayText, sizeof(displayText), "RB%d%%", raceBoxBattery);
            } else {
                snprintf(displayText, sizeof(displayText), "RB???");
            }
            useLargeFont = false;
            break;
        }
        
        case DISPLAY_COORD_DEBUG: {
            // Show raw coordinate values + accuracy for debugging
            if (coordsUpdated || (currentLatitude != 0.0 && currentLongitude != 0.0)) {
                float accMeters = horizontalAccuracy / 1000.0;
                snprintf(displayText, sizeof(displayText), "%.5f\n%.5f\nAcc:%.2fm", 
                        currentLatitude, currentLongitude, accMeters);
            } else {
                snprintf(displayText, sizeof(displayText), "NO COORDS");
            }
            useLargeFont = false;
            break;
        }
        
        case DISPLAY_DISTANCE_FROM_START: {
            // Distance from start point (finish line)
            if (finishLineSet && currentLatitude != 0.0 && currentLongitude != 0.0) {
                double distance = calculateDistance(currentLatitude, currentLongitude, 
                                                  finishLineLat, finishLineLon);
                if (distance < 1000) {
                    snprintf(displayText, sizeof(displayText), "%.1fm", distance);
                } else {
                    snprintf(displayText, sizeof(displayText), "%.2fkm", distance / 1000.0);
                }
            } else if (!finishLineSet) {
                snprintf(displayText, sizeof(displayText), "NO START");
            } else {
                snprintf(displayText, sizeof(displayText), "NO GPS");
            }
            useLargeFont = true;
            break;
        }
        
        case DISPLAY_LAP_TIMER: {
            // Show current lap timer duration with smooth real-time updates
            if (lapInProgress && lapStartMillis > 0) {
                // Use system time for smooth display updates
                unsigned long currentLapMs = millis() - lapStartMillis;
                
                // Convert to readable time format with hundredths (2 decimal places)
                unsigned long minutes = currentLapMs / 60000;
                unsigned long seconds = (currentLapMs % 60000) / 1000;
                unsigned long hundredths = (currentLapMs % 1000) / 10; // Hundredths of seconds
                
                // Format with 2 decimal places for smooth counting
                snprintf(displayText, sizeof(displayText), "%lu:%02lu.%02lu", 
                        minutes, seconds, hundredths);
            } else {
                snprintf(displayText, sizeof(displayText), "0:00.00");
            }
            useLargeFont = true;
            break;
        }
        
        case DISPLAY_LAP_DEBUG: {
            // Show distance and lap timer on 2 lines for debugging
            char distStr[16] = "NO GPS";
            char lapStr[16] = "0:00.0";
            
            // Get distance to finish line
            if (finishLineSet && currentLatitude != 0.0 && currentLongitude != 0.0) {
                double distance = calculateDistance(currentLatitude, currentLongitude, 
                                                  finishLineLat, finishLineLon);
                if (distance < 10) {
                    snprintf(distStr, sizeof(distStr), "%.1fm%s", distance, gpsAccuracyPoor ? "*" : "");
                } else if (distance < 1000) {
                    snprintf(distStr, sizeof(distStr), "%.0fm%s", distance, gpsAccuracyPoor ? "*" : "");
                } else {
                    snprintf(distStr, sizeof(distStr), "%.1fkm%s", distance / 1000.0, gpsAccuracyPoor ? "*" : "");
                }
            } else if (!finishLineSet) {
                snprintf(distStr, sizeof(distStr), "NO START");
            }
            
            // Get current lap timer with smooth real-time updates
            if (lapInProgress && lapStartMillis > 0) {
                // Use system time for smooth display updates
                unsigned long currentLapMs = millis() - lapStartMillis;
                unsigned long minutes = currentLapMs / 60000;
                unsigned long seconds = (currentLapMs % 60000) / 1000;
                unsigned long hundredths = (currentLapMs % 1000) / 10; // Hundredths of seconds
                snprintf(lapStr, sizeof(lapStr), "%lu:%02lu.%02lu", minutes, seconds, hundredths);
            }
            
            // Combine both on 2 lines (duration first, distance second)
            snprintf(displayText, sizeof(displayText), "T:%s\nD:%s", lapStr, distStr);
            useLargeFont = false; // Smaller font for 2 lines
            break;
        }
        
        case DISPLAY_LAP_SPEED: {
            // Show lap timer and speed on 2 lines (no timeout)
            char lapStr[16] = "0:00.0";
            char speedStr[16] = "0.0";
            
            // Get current lap timer
            if (lapInProgress && currentGpsTime > 0 && lapStartTime > 0) {
                unsigned long currentLapMs = currentGpsTime - lapStartTime;
                unsigned long minutes = currentLapMs / 60000;
                unsigned long seconds = (currentLapMs % 60000) / 1000;
                unsigned long tenths = (currentLapMs % 1000) / 100;
                snprintf(lapStr, sizeof(lapStr), "%lu:%02lu.%lu", minutes, seconds, tenths);
            }
            
            // Get current speed (with same logic as DISPLAY_SPEED)
            if (hasGpsFix && gpsFxCount > 3 && speedUpdated) {
                if (wasNearFinishLine && lapInProgress) {
                    snprintf(speedStr, sizeof(speedStr), "%.1f*", currentSpeed); // * means near line, lap active
                } else if (wasNearFinishLine) {
                    snprintf(speedStr, sizeof(speedStr), "%.1f+", currentSpeed); // + means near line
                } else {
                    snprintf(speedStr, sizeof(speedStr), "%.1f", currentSpeed);
                }
            } else {
                snprintf(speedStr, sizeof(speedStr), "READY");
            }
            
            // Combine both on 2 lines
            snprintf(displayText, sizeof(displayText), "L:%s\nS:%s", lapStr, speedStr);
            useLargeFont = false; // Smaller font for 2 lines
            break;
        }
        
        case DISPLAY_BEST_LAP:
            snprintf(displayText, sizeof(displayText), "%s", bestLapTime.c_str());
            useLargeFont = false; // Lap times are longer
            break;
            
        case DISPLAY_LAP_FLASH:
            // Show "LAP TIME:" on line 1, actual time on line 2
            snprintf(displayText, sizeof(displayText), "LAP TIME:\n%s", currentLapTimeStr.c_str());
            useLargeFont = false;
            break;
            
        case DISPLAY_LAP_DELTA:
            // Show delta on line 1, lap time on line 2
            snprintf(displayText, sizeof(displayText), "%s\n%s", deltaStr.c_str(), currentLapTimeStr.c_str());
            useLargeFont = false;
            break;
            
        case DISPLAY_RESET:
            snprintf(displayText, sizeof(displayText), "RESET");
            useLargeFont = true;
            break;
            
        case DISPLAY_RESET_CONFIRM:
            snprintf(displayText, sizeof(displayText), "SURE?");
            useLargeFont = true;
            // Auto-timeout after 5 seconds if no confirmation
            if (millis() - displayModeStartTime > 5000) {
                currentDisplayMode = DISPLAY_SPEED;
            }
            break;
    }
    
    // Apply font and content
    setDisplayFont(useLargeFont);
    
    // Set colors for countdown sequence
    switch (currentDisplayMode) {
        case DISPLAY_COUNTDOWN_3:
        case DISPLAY_COUNTDOWN_2:
        case DISPLAY_COUNTDOWN_1:
            // Bright red for countdown numbers
            lv_obj_set_style_text_color(number_label, lv_color_make(255, 0, 0), 0);
            break;
        case DISPLAY_COUNTDOWN_GO:
            // Bright green for GO!
            lv_obj_set_style_text_color(number_label, lv_color_make(0, 255, 0), 0);
            break;
        case DISPLAY_BAD_FIX:
            // Bright red for BAD FIX
            lv_obj_set_style_text_color(number_label, lv_color_make(255, 0, 0), 0);
            break;
        default:
            // White for all other modes
            lv_obj_set_style_text_color(number_label, lv_color_white(), 0);
            break;
    }
    
    // Debug output for LAP_TIMER display
    if (currentDisplayMode == DISPLAY_LAP_TIMER) {
        static unsigned long lastDisplayDebug = 0;
        if (millis() - lastDisplayDebug > 2000) {
            Serial.printf("DISPLAY: Setting text to '%s'\n", displayText);
            lastDisplayDebug = millis();
        }
    }
    
    lv_label_set_text(number_label, displayText);
}

void updateConnectionStatus() {
    switch (currentState) {
        case STATE_STARTUP:
            lv_label_set_text(number_label, "START?");
            break;
        case STATE_SCANNING:
            {
                // Animate SCAN with dots
                static unsigned long lastScanAnimation = 0;
                static int scanDots = 1;
                if (millis() - lastScanAnimation > 500) { // Change every 500ms
                    scanDots = (scanDots % 3) + 1;
                    lastScanAnimation = millis();
                }
                
                char scanText[16];
                if (scanDots == 1) {
                    snprintf(scanText, sizeof(scanText), "SCAN.");
                } else if (scanDots == 2) {
                    snprintf(scanText, sizeof(scanText), "SCAN..");
                } else {
                    snprintf(scanText, sizeof(scanText), "SCAN...");
                }
                lv_label_set_text(number_label, scanText);
            }
            break;
        case STATE_DEVICE_FOUND:
            lv_label_set_text(number_label, "FIND");
            break;
        case STATE_CONNECTING:
            lv_label_set_text(number_label, "CONN");
            break;
        case STATE_CONNECTED:
            if (speedUpdated) {
                // Handle finish line capture (averaging multiple readings)
                if (finishLineCapturing) {
                    // Check for timeout if GPS isn't updating
                    if (millis() - finishLineCaptureStart >= finishLineCaptureDuration) {
                        if (finishLineReadingsIndex == 0) {
                            // No readings captured at all - GPS problem
                            Serial.println("Finish line capture failed - no GPS updates received");
                            finishLineCapturing = false;
                            currentDisplayMode = DISPLAY_BAD_FIX; // Show BAD FIX message
                            badFixMessageStart = millis(); // Start timer for message
                        } else {
                            // Use whatever readings we got
                            Serial.printf("Finish line capture timeout - using %d readings\n", finishLineReadingsIndex);
                        }
                    }
                    
                    if (coordsUpdated) {
                        // Store this reading
                        if (finishLineReadingsIndex < finishLineReadingsCount) {
                            finishLineReadingsLat[finishLineReadingsIndex] = currentLatitude;
                            finishLineReadingsLon[finishLineReadingsIndex] = currentLongitude;
                            finishLineReadingsIndex++;
                            Serial.printf("Captured reading %d/%d: %.7f, %.7f\n", 
                                         finishLineReadingsIndex, finishLineReadingsCount,
                                         currentLatitude, currentLongitude);
                        }
                    }
                    
                    // Check if capture is complete (time or count)
                    if (finishLineReadingsIndex >= finishLineReadingsCount || 
                        (millis() - finishLineCaptureStart >= finishLineCaptureDuration && finishLineReadingsIndex > 0)) {
                        // Calculate average position
                        double sumLat = 0.0, sumLon = 0.0;
                        for (int i = 0; i < finishLineReadingsIndex; i++) {
                            sumLat += finishLineReadingsLat[i];
                            sumLon += finishLineReadingsLon[i];
                        }
                        finishLineLat = sumLat / finishLineReadingsIndex;
                        finishLineLon = sumLon / finishLineReadingsIndex;
                        finishLineSet = true;
                        finishLineCapturing = false;
                        saveFinishLine();
                        countdownStartTime = millis(); // Start debugging sequence
                        Serial.printf("Finish line set (averaged %d readings): %.7f, %.7f\n", 
                                     finishLineReadingsIndex, finishLineLat, finishLineLon);
                    }
                }
                
                // Check for lap crossing when coordinates are updated (FIRST, before resetting flags)
                if (coordsUpdated && !finishLineCapturing) {
                    coordsUpdateCounter++; // Track coordinate updates for debugging
                    checkLapCrossing();
                }
                
                // Debug output for coordinates (only print occasionally to avoid spam)
                static uint32_t lastCoordPrint = 0;
                static uint32_t lastGpsUpdate = 0;
                if (coordsUpdated) {
                    lastGpsUpdate = millis();
                    if (millis() - lastCoordPrint > 5000) { // Print every 5 seconds
                        Serial.printf("GPS: iTOW=%lu ms, Lat=%.7f, Lon=%.7f, Speed=%.1f km/h, Acc=%.2fm\n", 
                                      currentGpsTime, currentLatitude, currentLongitude, currentSpeed,
                                      horizontalAccuracy / 1000.0);
                        lastCoordPrint = millis();
                    }
                }
                
                // Warn if GPS updates have stopped
                static uint32_t lastGpsWarning = 0;
                if (millis() - lastGpsUpdate > 10000 && millis() - lastGpsWarning > 10000) {
                    Serial.println("WARNING: No GPS coordinate updates for 10+ seconds");
                    lastGpsWarning = millis();
                }
                
                // Always reset coordinate flags after processing
                if (coordsUpdated) {
                    coordsUpdated = false;
                    gpsTimeUpdated = false;
                }
                
                // Check if GPS has become unstable due to timeout
                if (gpsStable && millis() - lastGpsUpdateTime > 5000) {
                    Serial.println("GPS became unstable - no updates for 5+ seconds");
                    gpsStable = false;
                    gpsStableTime = 0;
                }
            }
            
            // Use smart display content based on current mode
            updateDisplayContent();
            break;
        case STATE_NO_DEVICE:
            lv_label_set_text(number_label, "NONE");
            break;
        case STATE_SCAN_TIMEOUT:
            lv_label_set_text(number_label, "TOUT");
            break;
        case STATE_ERROR:
            lv_label_set_text(number_label, "FAIL");
            break;
    }
}

void handleBLEStateMachine() {
    unsigned long currentTime = millis();
    
    switch (currentState) {
        case STATE_SCANNING:
            if (doConnect) {
                currentState = STATE_DEVICE_FOUND;
                stateChangeTime = currentTime;
                Serial.printf("Found RaceBox, preparing to connect...\n");
            } else if (currentTime - scanStartTime > scanTimeout) {
                Serial.println("Scan timeout - no RaceBox devices found");
                currentState = STATE_SCAN_TIMEOUT;
            }
            break;
            
        case STATE_DEVICE_FOUND:
            // Show FIND for a moment, then proceed to connect
            if (currentTime - stateChangeTime > 1000) { // Wait 1 second to show FIND
                currentState = STATE_CONNECTING;
                Serial.printf("Proceeding to connect to RaceBox...\n");
            }
            break;
            
        case STATE_CONNECTING:
            if (!connected && doConnect) {
                Serial.println("Attempting connection...");
                if (connectToRaceBox()) {
                    Serial.println("Connection successful!");
                    currentState = STATE_CONNECTED;
                } else {
                    Serial.println("Connection failed");
                    errorReason = "Connection failed";
                    currentState = STATE_ERROR;
                }
                doConnect = false;
            }
            break;
            
        case STATE_CONNECTED:
            // Connection is active, nothing to do for now
            if (!connected) {
                Serial.println("Connection lost");
                errorReason = "Connection lost";
                currentState = STATE_ERROR;
            }
            break;
            
        case STATE_SCAN_TIMEOUT:
        case STATE_NO_DEVICE:
        case STATE_ERROR:
            // Wait a bit then restart scanning
            if (currentTime - lastUpdate > 5000) { // Wait 5 seconds
                Serial.printf("Restarting scan after: %s\n", errorReason.c_str());
                currentState = STATE_SCANNING;
                doConnect = false;
                connected = false;
                myRaceBox = nullptr;
                errorReason = "";
                scanStartTime = currentTime;
                Serial.println("Starting new BLE scan...");
                
                // Setup scan with proper parameters
                NimBLEScan* pBLEScan = NimBLEDevice::getScan();
                pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
                pBLEScan->setInterval(45);
                pBLEScan->setWindow(15);
                pBLEScan->setActiveScan(true);
                pBLEScan->start(10, false); // Scan for 10 seconds
            }
            break;
    }
}

// UBX parsing function
void parseUBXPacket(uint8_t* data, size_t length) {
    // Look for UBX sync bytes (0xB5 0x62)
    for (size_t i = 0; i <= length - 8; i++) {
        if (data[i] == 0xB5 && data[i + 1] == 0x62) {
            // Found UBX header
            if (i + 7 < length) {
                uint8_t msgClass = data[i + 2];
                uint8_t msgId = data[i + 3];
                uint16_t payloadLen = (data[i + 5] << 8) | data[i + 4];
                
                // Check for RaceBox Data Message (Class 0xFF, ID 0x01) - this is the correct message type!
                if (msgClass == 0xFF && msgId == 0x01 && i + 6 + payloadLen <= length) {
                    dataPacketsReceived++;
                    
                    // GPS Time of Week (iTOW) is at offset 8 in RaceBox payload (4 bytes, little endian, ms)
                    size_t itowOffset = i + 6 + 8;
                    if (itowOffset + 3 < length) {
                        currentGpsTime = (uint32_t)(data[itowOffset] | 
                                                    (data[itowOffset + 1] << 8) |
                                                    (data[itowOffset + 2] << 16) |
                                                    (data[itowOffset + 3] << 24));
                        gpsTimeUpdated = true;
                    }
                    
                    // Parse GPS status fields (matching example.cpp)
                    size_t fixStatusOffset = i + 6 + 26;        // GPS fix status
                    size_t fixStatusFlagsOffset = i + 6 + 27;   // GPS fix flags 
                    size_t numSVsOffset = i + 6 + 29;           // Number of satellites
                    
                    if (fixStatusOffset < length) {
                        lastFixStatus = data[fixStatusOffset];
                    }
                    if (fixStatusFlagsOffset < length) {
                        lastFixStatusFlags = data[fixStatusFlagsOffset];
                        hasGpsFix = (lastFixStatusFlags & 0x01) != 0;
                    }
                    if (numSVsOffset < length) {
                        numSatellites = data[numSVsOffset];
                    }
                    
                    if (hasGpsFix) {
                        gpsFxCount++;
                    } else {
                        gpsFxCount = 0;
                    }
                    
                    // Longitude is at offset 24 in RaceBox payload (4 bytes, little endian, scaled by 1e-7)
                    size_t lonOffset = i + 6 + 24;
                    if (lonOffset + 3 < length) {
                        int32_t lonRaw = (int32_t)(data[lonOffset] | 
                                                   (data[lonOffset + 1] << 8) |
                                                   (data[lonOffset + 2] << 16) |
                                                   (data[lonOffset + 3] << 24));
                        currentLongitude = lonRaw * 1e-7;  // Convert to degrees
                    }
                    
                    // Latitude is at offset 28 in RaceBox payload (4 bytes, little endian, scaled by 1e-7)
                    size_t latOffset = i + 6 + 28;
                    if (latOffset + 3 < length) {
                        int32_t latRaw = (int32_t)(data[latOffset] | 
                                                   (data[latOffset + 1] << 8) |
                                                   (data[latOffset + 2] << 16) |
                                                   (data[latOffset + 3] << 24));
                        currentLatitude = latRaw * 1e-7;  // Convert to degrees
                    }
                    
                    // Horizontal accuracy is at offset 40 in RaceBox payload (4 bytes, mm)
                    size_t hAccOffset = i + 6 + 40;
                    if (hAccOffset + 3 < length) {
                        horizontalAccuracy = (uint32_t)(data[hAccOffset] | 
                                                       (data[hAccOffset + 1] << 8) |
                                                       (data[hAccOffset + 2] << 16) |
                                                       (data[hAccOffset + 3] << 24));
                        
                        // Only mark coordinates as updated if accuracy is good (<500mm = 50cm)
                        // This prevents using poor GPS readings that cause drift
                        if (horizontalAccuracy < 500) {
                            coordsUpdated = true;  // Mark that we have new, accurate coordinate data
                            gpsAccuracyPoor = false; // Good accuracy
                        } else {
                            coordsUpdated = false; // Skip this reading - accuracy too poor
                            gpsAccuracyPoor = true; // Mark accuracy as poor for display
                            static unsigned long lastAccWarn = 0;
                            if (millis() - lastAccWarn > 2000) {  // More frequent warnings for debugging
                                Serial.printf("WARNING: Poor GPS accuracy: %lu mm (%.1f m) - skipping\n", 
                                            horizontalAccuracy, horizontalAccuracy / 1000.0);
                                lastAccWarn = millis();
                            }
                        }
                        
                        // Always log accuracy for debugging GPS issues
                        static unsigned long lastAccLog = 0;
                        if (millis() - lastAccLog > 3000) {
                            Serial.printf("GPS Accuracy: %.1fm (using: %s)\n", 
                                        horizontalAccuracy / 1000.0, 
                                        coordsUpdated ? "YES" : "NO");
                            lastAccLog = millis();
                        }
                    }
                    
                    // Speed is at offset 48 in RaceBox payload (4 bytes, little endian, mm/s, SIGNED)
                    size_t speedOffset = i + 6 + 48;
                    if (speedOffset + 3 < length) {
                        // Parse as SIGNED 32-bit integer (matching Python's signed=True)
                        int32_t speedMmPerSec = (int32_t)(data[speedOffset] | 
                                                         (data[speedOffset + 1] << 8) |
                                                         (data[speedOffset + 2] << 16) |
                                                         (data[speedOffset + 3] << 24));
                        
                        // Convert mm/s to km/h (divide by 1000 for m/s, multiply by 3.6 for km/h)
                        float rawSpeed = (speedMmPerSec / 1000.0) * 3.6;
                        
                        // Apply speed stabilization
                        if (rawSpeed < speedThreshold) {
                            // Below threshold, gradually move to 0
                            stabilizedSpeed = stabilizedSpeed * (1.0 - speedSmoothingFactor);
                            if (stabilizedSpeed < 0.1) stabilizedSpeed = 0.0;
                        } else {
                            // Above threshold, smooth the reading
                            stabilizedSpeed = stabilizedSpeed * (1.0 - speedSmoothingFactor) + 
                                              rawSpeed * speedSmoothingFactor;
                        }
                        
                        currentSpeed = stabilizedSpeed;
                        speedUpdated = true;
                        
                        // Update GPS stability tracking
                        lastGpsUpdateTime = millis();
                        
                        // GPS is considered stable after receiving good data for 2+ seconds
                        if (hasGpsFix && gpsFxCount > 3) {
                            if (!gpsStable) {
                                if (gpsStableTime == 0) {
                                    gpsStableTime = millis(); // First good reading
                                } else if (millis() - gpsStableTime > 2000) {
                                    gpsStable = true; // Stable after 2 seconds
                                    Serial.println("GPS stabilized - switching to speed display");
                                }
                            }
                        } else {
                            // Lost good GPS fix
                            gpsStable = false;
                            gpsStableTime = 0;
                        }
                    }
                    
                    // Parse RaceBox battery information at offset 67 (official spec)
                    size_t batteryOffset = i + 6 + 67;
                    if (batteryOffset < length) {
                        uint8_t batteryByte = data[batteryOffset];
                        
                        // Check if this looks like Mini/Mini S (percentage) or Micro (voltage)
                        if ((batteryByte & 0x7F) <= 100) {
                            // RaceBox Mini/Mini S format:
                            // MSB = charging status, lower 7 bits = battery percentage
                            raceBoxBattery = batteryByte & 0x7F;  // Extract percentage (0-100)
                            bool isCharging = (batteryByte & 0x80) != 0;  // Extract charging bit
                            
                            // Debug output occasionally
                            static unsigned long lastBatteryDebug = 0;
                            if (millis() - lastBatteryDebug > 10000) {
                                Serial.printf("RB Battery: %d%% %s\\n", 
                                              raceBoxBattery, isCharging ? "(charging)" : "");
                                lastBatteryDebug = millis();
                            }
                        } else {
                            // RaceBox Micro format: input voltage * 10
                            // Convert to voltage and then estimate percentage (rough approximation)
                            float voltage = batteryByte / 10.0;
                            // Rough battery estimation: 11.1V = 0%, 12.6V = 100%
                            int estimatedPercent = (int)((voltage - 11.1) / (12.6 - 11.1) * 100);
                            if (estimatedPercent < 0) estimatedPercent = 0;
                            if (estimatedPercent > 100) estimatedPercent = 100;
                            raceBoxBattery = estimatedPercent;
                            
                            // Debug output occasionally  
                            static unsigned long lastVoltageDebug = 0;
                            if (millis() - lastVoltageDebug > 10000) {
                                Serial.printf("RB Voltage: %.1fV (est. %d%%)\\n", voltage, estimatedPercent);
                                lastVoltageDebug = millis();
                            }
                        }
                    }
                } else {
                    // Show other message types for debugging
                    Serial.printf("Other UBX message: Class=0x%02X, ID=0x%02X\n", msgClass, msgId);
                }
            }
        }
    }
}

// Notification callback with UBX parsing
static void notifyCallback(
  NimBLERemoteCharacteristic* pBLERemoteCharacteristic,
  uint8_t* pData,
  size_t length,
  bool isNotify) {
    
    // Parse UBX data for speed
    parseUBXPacket(pData, length);
}

// Connection function based on working example.cpp
bool connectToRaceBox() {
  Serial.println("Starting connection to RaceBox...");
  
  // Create a NimBLE client
  NimBLEClient* pClient = NimBLEDevice::createClient();
  if (pClient == nullptr) {
    Serial.println("Failed to create BLE client");
    return false;
  }
  Serial.println("Created BLE client");
  
  // Set client callbacks - use static instance to avoid heap issues
  static MyClientCallback clientCallback;
  pClient->setClientCallbacks(&clientCallback);
  
  // Connect to the remote BLE Server using the device reference (not address string)
  Serial.println("Attempting BLE connection...");
  pClient->setConnectTimeout(10); // 10 second timeout
  if (!pClient->connect(myRaceBox)) {  // This is the key difference!
    Serial.println("Failed to connect using device reference");
    NimBLEDevice::deleteClient(pClient);
    return false;
  }
  Serial.println("Connected to server");
  
  // Obtain a reference to the service
  NimBLERemoteService* pRemoteService = pClient->getService(serviceUUID);
  if (pRemoteService == nullptr) {
    Serial.print("Failed to find service UUID: ");
    Serial.println(serviceUUID.toString().c_str());
    pClient->disconnect();
    return false;
  }
  Serial.println("Found service");
  
  // Obtain a reference to the characteristic
  pRemoteCharacteristic = pRemoteService->getCharacteristic(txCharUUID);
  if (pRemoteCharacteristic == nullptr) {
    Serial.print("Failed to find characteristic UUID: ");
    Serial.println(txCharUUID.toString().c_str());
    pClient->disconnect();
    return false;
  }
  Serial.println("Found characteristic");
  
  // Read the value of the characteristic
  if(pRemoteCharacteristic->canRead()) {
    std::string value = pRemoteCharacteristic->readValue();
    Serial.print("Characteristic value: ");
    Serial.println(value.c_str());
  }
  
  // Turn on notifications (using newer API)
  if(pRemoteCharacteristic->canNotify()) {
    if(!pRemoteCharacteristic->subscribe(true, notifyCallback)) {
      Serial.println("Failed to subscribe to notifications");
      pClient->disconnect();
      return false;
    }
  }
  
  connected = true;
  Serial.println("Successfully connected and setup notifications");
  return true;
}
