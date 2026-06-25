#include <Arduino.h>
#include <ArduinoOTA.h>
#include <WiFi.h>
#include <Adafruit_NeoPixel.h>
#include <esp_sntp.h>
#include <ESP32Ping.h>   
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_task_wdt.h>
#include <Thermistor.h>
#include <NTC_Thermistor.h>
#include <TelnetStream.h>
#include "USB.h"
#include "tusb.h" // Native TinyUSB API
#include "secrets.h" // Local credentials (ignored by Git)

#define SENSOR_PIN 3

// Circuit and thermistor specifications
#define REFERENCE_RESISTANCE    9950  // 10k ohm series pull-up/pull-down resistor
#define NOMINAL_RESISTANCE      10000 // 10k ohm resistance at nominal temp
#define NOMINAL_TEMPERATURE     25    // Nominal temperature in Celsius
#define B_VALUE                 3950  // Beta coefficient
#define ESP32_ADC_RESOLUTION    4095  // 12-bit ADC maximum value
#define WDT_TIMEOUT_S           120

#define LOG_PRINT(x)    { Serial.print(x); TelnetStream.print(x); }
#define LOG_PRINTLN(x)  { Serial.println(x); TelnetStream.println(x); }
#define LOG_PRINTF(...) { Serial.printf(__VA_ARGS__); TelnetStream.printf(__VA_ARGS__); }

const IPAddress mothershipIp = IPAddress(10, 1, 1, 5);

volatile bool isMothershipUp = false;
volatile bool prepareForFlash = false;
volatile bool isBroken = false;

const char* ssid       = WIFI_SSID;
const char* wifi_password   = WIFI_PASSWORD;
const char* firmware_upload_password = "YOUR_PASSWORD_HERE";

const int LOGICAL_STEPS = 9;
const int PIN_NEOPIXELS = 6; 
const int NUM_PIXELS    = 60;

USBCDC USBSerial;

struct ColorPoint { float temp; uint8_t r, g, b; };


ColorPoint palette[] = {
    {20.0, 0, 255, 255},     // Cyan
    {25.0, 0, 0, 255},       // Deep Blue
    {30.0, 255, 0, 176},     // Pink
    {35.0, 255, 44, 0},      // Orange
    {40.0, 255, 255, 0},      // Yellow
    {45.0, 255, 255, 255  }   // White
};

volatile float currentTemp = -100;

Thermistor* thermistor = new NTC_Thermistor(
    SENSOR_PIN,
    REFERENCE_RESISTANCE,
    NOMINAL_RESISTANCE,
    NOMINAL_TEMPERATURE,
    B_VALUE,
    ESP32_ADC_RESOLUTION
);

Adafruit_NeoPixel strip(NUM_PIXELS, PIN_NEOPIXELS, NEO_GRB + NEO_KHZ800);

// --- Helper Functions ---

// --- DYNAMIC LUT CONFIGURATION ---
struct RGB { uint8_t r, g, b; };
RGB* tempLUT = nullptr; // Pointer for dynamic heap allocation

const float LUT_RESOLUTION = 0.1f; // Tweak this anytime
float LUT_MIN_TEMP = 0.0f;
int LUT_SIZE = 0;
float LUT_INV_RES = 0.0f; 

// The heavy interpolation function (Used ONLY during setup)
void calculateTemperatureRGB(float t, uint8_t &r, uint8_t &g, uint8_t &b) {
    int numPoints = sizeof(palette) / sizeof(palette[0]);
    if (t <= palette[0].temp) { r = palette[0].r; g = palette[0].g; b = palette[0].b; return; }
    if (t >= palette[numPoints - 1].temp) { r = palette[numPoints - 1].r; g = palette[numPoints - 1].g; b = palette[numPoints - 1].b; return; }

    for (int i = 0; i < numPoints - 1; i++) {
        if (t >= palette[i].temp && t <= palette[i+1].temp) {
            float range = palette[i+1].temp - palette[i].temp;
            float pct = (t - palette[i].temp) / range;
            r = palette[i].r + pct * (palette[i+1].r - palette[i].r);
            g = palette[i].g + pct * (palette[i+1].g - palette[i].g);
            b = palette[i].b + pct * (palette[i+1].b - palette[i].b);
            return;
        }
    }
    r = 0; g = 0; b = 0;
}

// Call this EXACTLY ONCE inside your setup() function
void prebakeLUT() {
    int numPoints = sizeof(palette) / sizeof(palette[0]);
    
    // Automatically extract boundaries from your palette array
    LUT_MIN_TEMP = palette[0].temp;
    float LUT_MAX_TEMP = palette[numPoints - 1].temp;
    
    // Calculate required array size and the inverse resolution for fast math
    LUT_SIZE = (int)((LUT_MAX_TEMP - LUT_MIN_TEMP) / LUT_RESOLUTION) + 1;
    LUT_INV_RES = 1.0f / LUT_RESOLUTION; 
    
    // Dynamically allocate contiguous RAM for the LUT
    tempLUT = new RGB[LUT_SIZE];
    
    // Bake the colors
    for (int i = 0; i < LUT_SIZE; i++) {
        float t = LUT_MIN_TEMP + (i * LUT_RESOLUTION);
        calculateTemperatureRGB(t, tempLUT[i].r, tempLUT[i].g, tempLUT[i].b);
    }
}

// The O(1) inline lookup for the main render loop
inline void getTemperatureRGB(float t, uint8_t &r, uint8_t &g, uint8_t &b) {
    // Fast multiplication using the precomputed inverse resolution
    int idx = (int)((t - LUT_MIN_TEMP) * LUT_INV_RES);
    
    // Clamp bounds
    if (idx < 0) idx = 0;
    if (idx >= LUT_SIZE) idx = LUT_SIZE - 1;
    
    r = tempLUT[idx].r;
    g = tempLUT[idx].g;
    b = tempLUT[idx].b;
}

void getSection(int cut, int &startIdx, int &endIdx) {
    if (cut < 0) cut = 0;
    if (cut >= LOGICAL_STEPS) cut = LOGICAL_STEPS - 1;
    
    startIdx = (cut * NUM_PIXELS) / LOGICAL_STEPS;
    endIdx = ((cut + 1) * NUM_PIXELS) / LOGICAL_STEPS - 1;
}

void waitProgress(int cut) {
    Serial.print(".");
    int startIdx, endIdx;
    getSection(cut, startIdx, endIdx);

    for(int i = startIdx; i <= endIdx; i++) {
        strip.setPixelColor(NUM_PIXELS - i - 1, strip.Color(255, 255, 0));
    }
    strip.show();
    vTaskDelay(pdMS_TO_TICKS(100)); 
    
    for(int i = startIdx; i <= endIdx; i++) {
        strip.setPixelColor(NUM_PIXELS - i - 1, strip.Color(0, 0, 0));
    }
    strip.show();
    vTaskDelay(pdMS_TO_TICKS(2900)); 
}

void showProgress(int cut) {
    int startIdx, endIdx;
    getSection(cut, startIdx, endIdx);

    for (int i = 0; i < NUM_PIXELS; i++) {
        strip.setPixelColor(NUM_PIXELS - i - 1, i <= endIdx ? strip.Color(0, 255, 0) : strip.Color(0, 0, 0));
    }            
    strip.show();
}


// --- FreeRTOS Tasks ---

void otaTask(void *pvParameters) {
    for (;;) {
        if (WiFi.status() == WL_CONNECTED) {
            ArduinoOTA.handle();
        }
        // Drastically reduced delay to allow immediate response to UDP broadcasts
        vTaskDelay(pdMS_TO_TICKS(10)); 
    }
}

void pingTask(void *pvParameters) {
    for (;;) {
        isMothershipUp = tud_mounted() && !tud_suspended();
        /*
        LOG_PRINTLN(tud_mounted()?"M+":"M-");
        LOG_PRINTLN(tud_suspended()?"S+":"S-");
        LOG_PRINTLN(isMothershipUp ? "UP":"DOWN");
        LOG_PRINTLN("----------------");
        */
        vTaskDelay(pdMS_TO_TICKS(3000));
    }
}

float smoothMothership = 1.0; 
float smoothTemp = -100; 

void ledDrawTemperature() {
    if (currentTemp < 0) {
        return;
    }

    if (smoothTemp < 0.0) {
        smoothTemp = currentTemp; 
    } 

    // Adjust the 0.005 multiplier to control how slowly the stripes narrow or recover.
    float targetMothership = isMothershipUp ? 1.0 : 0.0;
    smoothMothership += (targetMothership - smoothMothership) * 0.05;

    float spikeFactor = 20.0 - 19.0 * smoothMothership;

    // Adjust the 0.01 multiplier if you want the scrolling to lag/catch up faster.
    smoothTemp += (currentTemp - smoothTemp) * 0.001; 

    float viewportWidthCelsius = 0.23; 
    
    float cameraLeftEdgeTemp = smoothTemp - (viewportWidthCelsius / 2.0);
    float degreesPerPixel = viewportWidthCelsius / (float)NUM_PIXELS;

    // Wave frequencies in Temperature Space (Radians per Degree Celsius)
    float onFreq = 47.;     


    for (int i = 0; i < NUM_PIXELS; i++) {
        
        // x = the absolute coordinate on the infinite Celsius canvas
        float x = cameraLeftEdgeTemp + ((float)i * degreesPerPixel);

        //LOG_PRINTLN(currentTemp);
        // 1. Fetch the permanent color painted at x
        uint8_t baseR, baseG, baseB;
        getTemperatureRGB((float)x, baseR, baseG, baseB);


        // 2. Evaluate the permanent waves molded into x
        // --- ON STATE LOGIC ---
        float sinVal = sin(x * onFreq);
        float cosVal = cos(x * onFreq);
        float valOn = (sinVal/(1. + spikeFactor * cosVal * cosVal)) - 0.66 - 0.8 * (1.0 - smoothMothership); 
        if (valOn > 1.0) valOn = 1.0;
        if (valOn < -1.0) valOn = -1.0;  
        float finalBrightness = (valOn + 1.0) / 2.0; 
 
        float finalR = (baseR * finalBrightness);
        float finalG = (baseG * finalBrightness);
        float finalB = (baseB * finalBrightness);

        if (finalR > 255) finalR = 255;
        if (finalG > 255) finalG = 255;
        if (finalB > 255) finalB = 255;
        // strip.setPixelColor(i, strip.Color(0, baseG, baseB));
        strip.setPixelColor(i, strip.Color((uint8_t)finalR, (uint8_t)finalG, (uint8_t)finalB));
    }

    strip.show();

}

void ledTask(void *pvParameters) {
    while(true) {
        ledDrawTemperature();
        vTaskDelay(pdMS_TO_TICKS(33)); 
    }
}

// --- Initialization Block ---

void initHardware() {
    Serial.begin(115200);
    analogReadResolution(12);
    
    strip.begin();
    strip.clear();
    strip.show();    
    
    esp_task_wdt_config_t twdt_config = {
        .timeout_ms = WDT_TIMEOUT_S * 1000,
        .idle_core_mask = 0,
        .trigger_panic = true 
    };
    esp_task_wdt_reconfigure(&twdt_config);
    esp_task_wdt_add(NULL);
    
    showProgress(1);
}

void initWiFi() {
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, wifi_password);
    
    while (WiFi.status() != WL_CONNECTED) {
        waitProgress(2);
    }
    
    Serial.printf("\nWiFi Connected. IP: %s\n", WiFi.localIP().toString().c_str());
    esp_task_wdt_reset(); 
    showProgress(2);
}

void initOTA() {
    ArduinoOTA.setHostname("esp32-pc-soul");
    ArduinoOTA.setPort(1723); // Explicitly define this port
    ArduinoOTA.setPassword(firmware_upload_password); 

    ArduinoOTA.onStart([]() {
        prepareForFlash = true;
        Serial.println("\nOTA Update Started...");
    });
    
    ArduinoOTA.onEnd([]() {
        Serial.println("\nOTA Update Finished. Rebooting...");
        // Give the TCP stack a moment to send the final ACK to PlatformIO
        delay(1000); 
        
        // Force the hardware reset
        WiFi.disconnect();
        prepareForFlash = true;
    });
    
    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
        esp_task_wdt_reset(); // Feed watchdog during flash write to prevent panic
        Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    });
    
    ArduinoOTA.onError([](ota_error_t error) {
        Serial.printf("Error[%u]: ", error);
        if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
        else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
        else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
        else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
        else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });

    ArduinoOTA.begin();
    xTaskCreate(otaTask, "OTATask", 4096, NULL, 1, NULL);
}

void setup() {
    initHardware();
    initWiFi();
    initOTA();
    USB.begin();
    USBSerial.begin();
    TelnetStream.begin();
    prebakeLUT();
    xTaskCreate(ledTask,  "LEDTask",  4096, NULL, 1, NULL);
    xTaskCreate(pingTask, "PINGtask", 4096, NULL, 1, NULL);
}

void loop() {  

    currentTemp = thermistor->readCelsius();
    LOG_PRINTLN(currentTemp);
    
    if (WiFi.status() != WL_CONNECTED) {
        isBroken = true;
        //WiFi.reconnect(); // Attempt native stack recovery instead of relying on WDT panic
    } else {
        isBroken = false;
        esp_task_wdt_reset(); 
    }

    vTaskDelay(pdMS_TO_TICKS(1000));
}