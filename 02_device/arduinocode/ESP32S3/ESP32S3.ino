/*
 * Project: City Sensory AI - The Eye (Final Logic)
 * Board: Seeed Studio XIAO ESP32S3 (with Sense Expansion)
 * Function: Vision Inference, SD Logging, Image Saving, GPS Sync
 */

#include <UCL_RC15_inferencing.h> // Please ensure your Edge Impulse library is installed
#include "edge-impulse-sdk/dsp/image/image.hpp"
#include "esp_camera.h"
#include <FS.h>
#include <SD.h>
#include <SPI.h> 
#include <ArduinoJson.h>

// ==========================================
// 1. Pin Definitions & Global Variables
// ==========================================

// Camera pins (XIAO ESP32S3 Sense standard definitions)
#define PWDN_GPIO_NUM     -1
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM     10
#define SIOD_GPIO_NUM     40
#define SIOC_GPIO_NUM     39
#define Y9_GPIO_NUM       48
#define Y8_GPIO_NUM       11
#define Y7_GPIO_NUM       12
#define Y6_GPIO_NUM       14
#define Y5_GPIO_NUM       16
#define Y4_GPIO_NUM       18
#define Y3_GPIO_NUM       17
#define Y2_GPIO_NUM       15
#define VSYNC_GPIO_NUM    38
#define HREF_GPIO_NUM     47
#define PCLK_GPIO_NUM     13

// SD Card pins
#define SD_CS_PIN    21
#define SD_SCK_PIN   7
#define SD_MISO_PIN  8
#define SD_MOSI_PIN  9

// Global configuration
#define EI_CAMERA_RAW_FRAME_BUFFER_COLS 320
#define EI_CAMERA_RAW_FRAME_BUFFER_ROWS 240
#define EI_CAMERA_FRAME_BYTE_SIZE       3

static bool debug_nn = false; 
static bool is_initialised = false;
uint8_t *snapshot_buf; // Used to store resized image data
const char* logFileName = "/data_log.csv";
bool sdReady = false;
int imageCount = 0; 

// Camera configuration structure
static camera_config_t camera_config = {
    .pin_pwdn = PWDN_GPIO_NUM,
    .pin_reset = RESET_GPIO_NUM,
    .pin_xclk = XCLK_GPIO_NUM,
    .pin_sscb_sda = SIOD_GPIO_NUM,
    .pin_sscb_scl = SIOC_GPIO_NUM,
    .pin_d7 = Y9_GPIO_NUM, .pin_d6 = Y8_GPIO_NUM, .pin_d5 = Y7_GPIO_NUM, .pin_d4 = Y6_GPIO_NUM,
    .pin_d3 = Y5_GPIO_NUM, .pin_d2 = Y4_GPIO_NUM, .pin_d1 = Y3_GPIO_NUM, .pin_d0 = Y2_GPIO_NUM,
    .pin_vsync = VSYNC_GPIO_NUM, .pin_href = HREF_GPIO_NUM, .pin_pclk = PCLK_GPIO_NUM,
    .xclk_freq_hz = 20000000,
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,
    .pixel_format = PIXFORMAT_JPEG, 
    .frame_size = FRAMESIZE_QVGA, 
    .jpeg_quality = 12, 
    .fb_count = 2,       
    .fb_location = CAMERA_FB_IN_PSRAM, 
    .grab_mode = CAMERA_GRAB_LATEST,   
};

/* Function Declarations */
bool ei_camera_init(void);
void ei_camera_deinit(void);
bool ei_camera_capture(uint32_t img_width, uint32_t img_height, uint8_t *out_buf);
static int ei_camera_get_data(size_t offset, size_t length, float *out_ptr);
void saveResizedImageToSD(uint8_t *rgb_buf, int w, int h, String timeTag);
String getSystemTimeFormatted(); 
void waitForGPSLock(); // Newly added core blocking function

// ==========================================
// 2. Setup (System Initialization)
// ==========================================
void setup()
{
    Serial.begin(115200); 
    
    // Initialize communication with Nano
    // D6=RX, D7=TX (Corresponding to Nano's A3/A2)
    pinMode(D6, INPUT_PULLUP); 
    pinMode(D7, OUTPUT);       
    Serial1.begin(9600, SERIAL_8N1, D6, D7); 
    Serial1.setTimeout(2000); 

    Serial.println(">>> System Booting <<<");

    // 1. Initialize PSRAM and Camera
    if(psramInit()){ Serial.printf("PSRAM: %d bytes free\n", ESP.getFreePsram()); }
    if (!ei_camera_init()) { Serial.println("ERR: Camera Init Failed!"); while(1); }
    
    // 2. Initialize SD Card
    SPI.begin(SD_SCK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CS_PIN);
    if(!SD.begin(SD_CS_PIN)){ 
        Serial.println("ERR: SD Mount Failed!"); 
        sdReady = false; 
    } else { 
        Serial.println("SD Card OK."); 
        sdReady = true; 
        
        // If log file does not exist, write the header
        if (!SD.exists(logFileName)) {
            File f = SD.open(logFileName, FILE_WRITE);
            if(f) {
                f.println("Timestamp_ms,Time_Str,Lat,Lon,Void,Bio,Tech,Entropy,GSR,MOD");
                f.close();
            }
        }
    }

    // Allocate image buffer
    snapshot_buf = (uint8_t*)ps_malloc(EI_CAMERA_RAW_FRAME_BUFFER_COLS * EI_CAMERA_RAW_FRAME_BUFFER_ROWS * EI_CAMERA_FRAME_BYTE_SIZE);
    if(!snapshot_buf) { Serial.println("ERR: Buffer Malloc Failed"); while(1); }

    // 3. [Critical Step] Wait for GPS lock
    // The program will pause here until Nano reports valid coordinates
    waitForGPSLock();

    Serial.println(">>> GPS Locked. Main Loop Starting. <<<");
}

// ==========================================
// 3. Loop (Main Logic - Runs only after GPS lock)
// ==========================================
void loop()
{
    unsigned long loopStart = millis();

    // ----------------------------------------
    // A. Vision Inference
    // ----------------------------------------
    // Capture and resize image to model input size
    if (!ei_camera_capture(EI_CLASSIFIER_INPUT_WIDTH, EI_CLASSIFIER_INPUT_HEIGHT, snapshot_buf)) return;
    
    ei::signal_t signal;
    signal.total_length = EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT;
    signal.get_data = &ei_camera_get_data;
    ei_impulse_result_t result = { 0 };
    
    // Run classifier
    if (run_classifier(&signal, &result, debug_nn) != EI_IMPULSE_OK) return;

    // ----------------------------------------
    // B. Build Vision Data Payload
    // ----------------------------------------
    float val_void=0, val_bio=0, val_tech=0, val_entropy=0;
    String payload = "{";
    for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
        String label = result.classification[ix].label;
        float value = result.classification[ix].value;
        if (label == "The Void") val_void = value;
        if (label == "Bio-Resistance") val_bio = value;
        if (label == "Technological Unconscious") val_tech = value;
        if (label == "Material Entropy") val_entropy = value;
        payload += "\"" + label + "\":" + String(value, 2);
        if (ix < EI_CLASSIFIER_LABEL_COUNT - 1) payload += ",";
    }
    payload += "}";

    // ----------------------------------------
    // C. Communication Interaction (Send Vision -> Get Sensor data)
    // ----------------------------------------
    Serial1.println(payload); // 1. Send to Nano
    delay(100); 

    // 2. Prepare to receive data from Nano
    while(Serial1.available()) Serial1.read(); // Clear buffer
    Serial1.println("GET_GSR"); // Send request command

    // 3. Wait for reply
    int gsr_val = 0;
    float mod_val = 1.0;
    double lat_val = 0.0;
    double lon_val = 0.0;
    String time_str = ""; 
    
    unsigned long waitStart = millis();
    bool dataReceived = false;

    while (millis() - waitStart < 2000) { 
        if (Serial1.available()) {
            String line = Serial1.readStringUntil('\n');
            line.trim();
            
            int jsonStart = line.indexOf('{');
            if (jsonStart != -1) {
                String jsonStr = line.substring(jsonStart); 
                JsonDocument doc;
                DeserializationError error = deserializeJson(doc, jsonStr);
                
                if (!error) {
                    gsr_val = doc["GSR"];
                    mod_val = doc["MOD"];
                    lat_val = doc["LAT"];
                    lon_val = doc["LON"];
                    
                    // Time processing logic
                    if (doc.containsKey("TIME")) {
                        String t = doc["TIME"].as<String>();
                        // If GPS time is valid (not "00:00:00" and not empty)
                        if (t != "00:00:00" && t.length() > 0) {
                            time_str = t; 
                        } else {
                            time_str = getSystemTimeFormatted(); // Fallback to system time
                        }
                    } else {
                        time_str = getSystemTimeFormatted();
                    }

                    dataReceived = true;
                    Serial.printf("[RX] T:%s | GPS:%.6f, %.6f | Void:%.2f\n", time_str.c_str(), lat_val, lon_val, val_void);
                    break; 
                }
            }
        }
    }

    if (!dataReceived) {
        Serial.println("[WARN] Nano communication timeout. Using system time.");
        time_str = getSystemTimeFormatted();
    }

    // ----------------------------------------
    // D. Data Saving (CSV + BMP)
    // ----------------------------------------
    if (sdReady) {
        // 1. Write to CSV
        File f = SD.open(logFileName, FILE_APPEND);
        if (f) {
            f.print(loopStart); f.print(",");
            f.print(time_str); f.print(",");
            f.print(lat_val, 6); f.print(",");
            f.print(lon_val, 6); f.print(",");
            f.print(val_void, 2); f.print(",");
            f.print(val_bio, 2); f.print(",");
            f.print(val_tech, 2); f.print(",");
            f.print(val_entropy, 2); f.print(",");
            f.print(gsr_val); f.print(","); 
            f.println(mod_val, 2);
            f.close();
        }

        // 2. Save image (saved every frame, for training/backtracking)
        // Replace colons in time string, as filenames cannot contain ":"
        String cleanTime = time_str;
        cleanTime.replace(":", "-");
        // Call save function
        saveResizedImageToSD(snapshot_buf, EI_CLASSIFIER_INPUT_WIDTH, EI_CLASSIFIER_INPUT_HEIGHT, cleanTime);
    }

    // ----------------------------------------
    // E. Loop Delay Control
    // ----------------------------------------
    // Target loop time 6000ms, maintain rhythm
    long delayTime = 6000 - (millis() - loopStart);
    if (delayTime > 0) delay(delayTime);
}

// ==========================================
// Helper Function Definitions
// ==========================================

// 1. [Core Function] Blocking wait for GPS lock
void waitForGPSLock() {
    Serial.println("----------------------------------------");
    Serial.println("   WAITING FOR GPS SATELLITE LOCK...    ");
    Serial.println("   (System halted until Nano confirms)  ");
    Serial.println("----------------------------------------");

    while (true) {
        // Clear receive buffer
        while(Serial1.available()) Serial1.read();

        // Send query
        Serial1.println("GET_GSR");

        unsigned long waitStart = millis();
        bool validResponse = false;
        
        while (millis() - waitStart < 1500) {
            if (Serial1.available()) {
                String line = Serial1.readStringUntil('\n');
                int jsonStart = line.indexOf('{');
                if (jsonStart != -1) {
                    JsonDocument doc;
                    DeserializationError err = deserializeJson(doc, line.substring(jsonStart));
                    
                    if (!err) {
                        double lat = doc["LAT"];
                        // Criterion: As long as latitude is not 0.0, the GPS on Nano's end is locked
                        if (lat != 0.0) {
                            Serial.println("\n[SUCCESS] Signal Acquired! GPS is Ready.");
                            return; // Exit function, continue executing setup()
                        } else {
                            Serial.print("."); // Print dot, indicating waiting
                            validResponse = true;
                        }
                    }
                }
            }
        }
        
        if (!validResponse) Serial.print("?"); // Communication might be blocked/failing
        
        delay(1000); // Query once every second
    }
}

// 2. Get system running time (Min:Sec)
String getSystemTimeFormatted() {
    unsigned long now = millis();
    unsigned long seconds = now / 1000;
    unsigned long minutes = seconds / 60;
    unsigned long remainSeconds = seconds % 60;
    
    char buf[16];
    sprintf(buf, "%02lu:%02lu(sys)", minutes, remainSeconds); 
    return String(buf);
}

// 3. Save Image Buffer as BMP file
void saveResizedImageToSD(uint8_t *rgb_buf, int w, int h, String timeTag) {
    // Filename format: /img_time_count.bmp
    String fname = "/img_" + timeTag + "_" + String(imageCount++) + ".bmp";
    
    File f = SD.open(fname, FILE_WRITE);
    if (!f) return;

    // Calculate Padding (BMP row byte count must be a multiple of 4)
    int rowSize = w * 3;
    int padding = (4 - (rowSize % 4)) % 4;
    uint32_t fileSize = 54 + (rowSize + padding) * h;

    // BMP File Header (14 bytes)
    uint8_t bmpFileHeader[14] = {
        0x42, 0x4D,             // "BM"
        (uint8_t)(fileSize), (uint8_t)(fileSize >> 8), (uint8_t)(fileSize >> 16), (uint8_t)(fileSize >> 24),
        0, 0, 0, 0,             // Reserved
        54, 0, 0, 0             // Data offset
    };

    // BMP Info Header (40 bytes)
    int32_t negativeHeight = -h; // Negative height indicates Top-Down (normal visual orientation)
    uint8_t bmpInfoHeader[40] = {
        40, 0, 0, 0,            // Header size
        (uint8_t)(w), (uint8_t)(w >> 8), (uint8_t)(w >> 16), (uint8_t)(w >> 24),
        (uint8_t)(negativeHeight), (uint8_t)(negativeHeight >> 8), (uint8_t)(negativeHeight >> 16), (uint8_t)(negativeHeight >> 24),
        1, 0,                   // Planes
        24, 0,                  // Bit count (RGB)
        0, 0, 0, 0,             // Compression
        0, 0, 0, 0,             // Image size
        0, 0, 0, 0,             // X pixels/m
        0, 0, 0, 0,             // Y pixels/m
        0, 0, 0, 0,             // Colors used
        0, 0, 0, 0              // Important colors
    };

    f.write(bmpFileHeader, 14);
    f.write(bmpInfoHeader, 40);

    // Write pixel data (RGB -> BGR)
    uint8_t *rowBuf = (uint8_t *)malloc(rowSize + padding);
    if (!rowBuf) { f.close(); return; }

    uint8_t padByte = 0;
    
    for (int y = 0; y < h; y++) {
        for (int x = 0; x < w; x++) {
            int srcIdx = (y * w + x) * 3;
            // BMP uses BGR order
            rowBuf[x*3 + 0] = rgb_buf[srcIdx + 2]; // B
            rowBuf[x*3 + 1] = rgb_buf[srcIdx + 1]; // G
            rowBuf[x*3 + 2] = rgb_buf[srcIdx + 0]; // R
        }
        f.write(rowBuf, rowSize);
        // Write Padding
        for (int p=0; p<padding; p++) f.write(&padByte, 1);
    }
    
    free(rowBuf);
    f.close();
}

// 4. Edge Impulse Camera Interface Implementation (Kept as original)
bool ei_camera_init(void) {
    if (is_initialised) return true;
    if (esp_camera_init(&camera_config) != ESP_OK) return false;
    sensor_t * s = esp_camera_sensor_get();
    if (s->id.PID == OV5640_PID) {
        s->set_vflip(s, 1); s->set_hmirror(s, 0); s->set_lenc(s, 1); s->set_awb_gain(s, 1); 
    } else { s->set_vflip(s, 1); s->set_hmirror(s, 0); }
    is_initialised = true; return true;
}
void ei_camera_deinit(void) { esp_camera_deinit(); is_initialised = false; }
bool ei_camera_capture(uint32_t img_width, uint32_t img_height, uint8_t *out_buf) {
    if (!is_initialised) return false;
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) return false;
    bool converted = fmt2rgb888(fb->buf, fb->len, PIXFORMAT_JPEG, snapshot_buf);
    esp_camera_fb_return(fb);
    if(!converted) return false;
    if ((img_width != EI_CAMERA_RAW_FRAME_BUFFER_COLS) || (img_height != EI_CAMERA_RAW_FRAME_BUFFER_ROWS)) {
        ei::image::processing::crop_and_interpolate_rgb888(snapshot_buf, EI_CAMERA_RAW_FRAME_BUFFER_COLS, EI_CAMERA_RAW_FRAME_BUFFER_ROWS, out_buf, img_width, img_height);
    } else { memcpy(out_buf, snapshot_buf, img_width * img_height * 3); }
    return true;
}
static int ei_camera_get_data(size_t offset, size_t length, float *out_ptr) {
    size_t pixel_ix = offset * 3;
    size_t pixels_left = length;
    size_t out_ptr_ix = 0;
    while (pixels_left != 0) {
        out_ptr[out_ptr_ix] = (snapshot_buf[pixel_ix] << 16) + (snapshot_buf[pixel_ix + 1] << 8) + snapshot_buf[pixel_ix + 2];
        out_ptr_ix++; pixel_ix+=3; pixels_left--;
    }
    return 0;
}