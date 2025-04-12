#include "esp_camera.h"
#include <WiFi.h>
#include "esp_timer.h"
#include "img_converters.h"
#include "Arduino.h"
#include "fb_gfx.h"
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "esp_http_server.h"
#include "driver/ledc.h"
#include <WebSocketsServer.h>
#include <UniversalTelegramBot.h>
#include <WiFiClientSecure.h>

// WiFi credentials
const char* ssid = "my_phone";     // Enter your phone's hotspot name
const char* password = "187829200";   // Enter your hotspot password

WebSocketsServer webSocket = WebSocketsServer(82); // WebSocket on port 82

// Add these variables after existing definitions
unsigned long lastStreamTime = 0;
const int streamInterval = 100; // 10 fps for stream

// Add these configurations after your WiFi credentials
#define BOT_TOKEN "7830395553:AAHfQjN5nSuC96KlZyJuJEdqkRi3Xy7djx4"  // Get this from BotFather
#define CHAT_ID "8103002328"      // Get this from @userinfobot

// Add these objects after your other global variables
WiFiClientSecure client;
UniversalTelegramBot bot(BOT_TOKEN, client);

// Pin Definitions
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22
#define FLASH_GPIO_NUM     4

// Motor control pins
#define MOTOR_1_PIN_1    14 // IN1
#define MOTOR_1_PIN_2    15 // IN2
#define MOTOR_2_PIN_1    13 // IN3
#define MOTOR_2_PIN_2    12 // IN4

// PWM properties
#define MOTOR_FREQ       5000
#define MOTOR_RES        8

#define PART_BOUNDARY "123456789000000000000987654321"
static const char* _STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* _STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char* _STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

httpd_handle_t stream_httpd = NULL;
httpd_handle_t control_httpd = NULL;

// HTML for the control page
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <title>ESP32-CAM RC Control</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
        body { font-family: Arial; text-align: center; margin: 0px auto; padding: 0; background: #202020; color: white; }
        img { width: 100%; max-width: 800px; margin: 10px auto; transform: rotate(180deg);}
        .button { background-color: #4CAF50; border: none; color: white; padding: 15px 25px;
                 text-align: center; text-decoration: none; display: inline-block;
                 font-size: 18px; margin: 6px 3px; cursor: pointer; border-radius: 5px;
                 user-select: none; -webkit-user-select: none; }
        .button:active { background-color: #3e8e41; }
        .button.flash { background-color: #ff4444; }
        .control-box { position: fixed; bottom: 20px; left: 0; right: 0; 
                      background: rgba(0,0,0,0.7); padding: 10px; }
        #stream { margin-bottom: 100px; }
    </style>
</head>
<body>
    <img src="" id="stream" onerror="retryStream(this)">
    <div class="control-box">
        <button class="button" ontouchstart="startCommand('forward')" ontouchend="stopCommand()" 
                onmousedown="startCommand('forward')" onmouseup="stopCommand()">Forward</button><br>
        <button class="button" ontouchstart="startCommand('right')" ontouchend="stopCommand()"
                onmousedown="startCommand('right')" onmouseup="stopCommand()">Left</button>
        <button class="button" ontouchstart="stopCommand()" ontouchend="stopCommand()"
                onmousedown="stopCommand()">Stop</button>
        <button class="button" ontouchstart="startCommand('left')" ontouchend="stopCommand()"
                onmousedown="startCommand('left')" onmouseup="stopCommand()">Right</button><br>
        <button class="button" ontouchstart="startCommand('backward')" ontouchend="stopCommand()"
                onmousedown="startCommand('backward')" onmouseup="stopCommand()">Backward</button>
        <button class="button flash" onclick="toggleFlash()">Flash</button>
    </div>
    <script>
        let streamUrl = `${window.location.protocol}//${window.location.hostname}:81/stream`;
        let streamRetries = 0;
        const MAX_RETRIES = 3;

        function initStream() {
            let img = document.getElementById('stream');
            // For mobile browsers, try using a different method to load the stream
            if(/Android|webOS|iPhone|iPad|iPod|BlackBerry|IEMobile|Opera Mini/i.test(navigator.userAgent)) {
                img.src = streamUrl + '?t=' + new Date().getTime();
            } else {
                img.src = streamUrl;
            }
        }

        function retryStream(img) {
            if (streamRetries < MAX_RETRIES) {
                setTimeout(() => {
                    streamRetries++;
                    img.src = streamUrl + '?t=' + new Date().getTime();
                }, 1000);
            }
        }

        // Initialize stream
        initStream();

        // Periodically refresh stream for mobile devices
        if(/Android|webOS|iPhone|iPad|iPod|BlackBerry|IEMobile|Opera Mini/i.test(navigator.userAgent)) {
            setInterval(() => {
                let img = document.getElementById('stream');
                img.src = streamUrl + '?t=' + new Date().getTime();
            }, 5000);
        }

        let speedInterval;
        let currentSpeed = 0;
        let flashState = false;
        const MAX_SPEED = 255;
        const SPEED_INCREMENT = 51; // Increase by 20% each step
        const SPEED_INTERVAL = 100; // Update speed every 100ms

        function startCommand(command) {
            currentSpeed = 128;
            fetchCommand(command + '?speed=' + currentSpeed);
            
            speedInterval = setInterval(() => {
                if (currentSpeed < MAX_SPEED) {
                    currentSpeed = Math.min(MAX_SPEED, currentSpeed + SPEED_INCREMENT);
                    fetchCommand(command + '?speed=' + currentSpeed);
                }
            }, SPEED_INTERVAL);
        }

        function stopCommand() {
            clearInterval(speedInterval);
            currentSpeed = 0;
            fetchCommand('stop');
        }

        function toggleFlash() {
            flashState = !flashState;
            fetchCommand(flashState ? 'flash_on' : 'flash_off');
        }

        async function fetchCommand(command) {
            try {
                await fetch('/control?cmd=' + command);
            } catch (e) {
                console.error('Control error:', e);
            }
        }
    </script>
</body>
</html>
)rawliteral";

void setupMotors() {
    // Configure timer for all channels
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_8_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = MOTOR_FREQ,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&ledc_timer);

    // Configure motor pins
    int pins[4] = {MOTOR_1_PIN_1, MOTOR_1_PIN_2, MOTOR_2_PIN_1, MOTOR_2_PIN_2};
    for(int i = 0; i < 4; i++) {
        ledc_channel_config_t ledc_channel = {
            .gpio_num = pins[i],
            .speed_mode = LEDC_HIGH_SPEED_MODE,
            .channel = (ledc_channel_t)i,
            .intr_type = LEDC_INTR_DISABLE,
            .timer_sel = LEDC_TIMER_0,
            .duty = 0,
            .hpoint = 0
        };
        ledc_channel_config(&ledc_channel);
    }
    
    // Set all motors to zero initially
    for(int i = 0; i < 4; i++) {
        ledc_set_duty(LEDC_HIGH_SPEED_MODE, (ledc_channel_t)i, 0);
        ledc_update_duty(LEDC_HIGH_SPEED_MODE, (ledc_channel_t)i);
    }
}

// Add flash control function
void controlFlash(bool state) {
    pinMode(FLASH_GPIO_NUM, OUTPUT);
    digitalWrite(FLASH_GPIO_NUM, state ? HIGH : LOW);
}

// Modified controlMotors function
void controlMotors(const char* command) {
    static int currentSpeed = 255;
    
    // Parse speed parameter if present
    char cmd[32];
    int speed = 255;
    if (strchr(command, '?') != NULL) {
        sscanf(command, "%[^?]?speed=%d", cmd, &speed);
    } else {
        strcpy(cmd, command);
    }


    if (strcmp(cmd, "stop") == 0) {
        // Set all channels to zero
        for(int i = 0; i < 4; i++) {
            ledc_set_duty(LEDC_HIGH_SPEED_MODE, (ledc_channel_t)i, 0);
            ledc_update_duty(LEDC_HIGH_SPEED_MODE, (ledc_channel_t)i);
        }
    }
    else if (strcmp(cmd, "forward") == 0) {
        ledc_set_duty(LEDC_HIGH_SPEED_MODE, (ledc_channel_t)0, speed);
        ledc_set_duty(LEDC_HIGH_SPEED_MODE, (ledc_channel_t)1, 0);
        ledc_set_duty(LEDC_HIGH_SPEED_MODE, (ledc_channel_t)2, speed);
        ledc_set_duty(LEDC_HIGH_SPEED_MODE, (ledc_channel_t)3, 0);
    }
    else if (strcmp(cmd, "backward") == 0) {
        ledc_set_duty(LEDC_HIGH_SPEED_MODE, (ledc_channel_t)0, 0);
        ledc_set_duty(LEDC_HIGH_SPEED_MODE, (ledc_channel_t)1, speed);
        ledc_set_duty(LEDC_HIGH_SPEED_MODE, (ledc_channel_t)2, 0);
        ledc_set_duty(LEDC_HIGH_SPEED_MODE, (ledc_channel_t)3, speed);
    }
    else if (strcmp(cmd, "left") == 0) {
        ledc_set_duty(LEDC_HIGH_SPEED_MODE, (ledc_channel_t)0, 0);
        ledc_set_duty(LEDC_HIGH_SPEED_MODE, (ledc_channel_t)1, speed);
        ledc_set_duty(LEDC_HIGH_SPEED_MODE, (ledc_channel_t)2, speed);
        ledc_set_duty(LEDC_HIGH_SPEED_MODE, (ledc_channel_t)3, 0);
    }
    else if (strcmp(cmd, "right") == 0) {
        ledc_set_duty(LEDC_HIGH_SPEED_MODE, (ledc_channel_t)0, speed);
        ledc_set_duty(LEDC_HIGH_SPEED_MODE, (ledc_channel_t)1, 0);
        ledc_set_duty(LEDC_HIGH_SPEED_MODE, (ledc_channel_t)2, 0);
        ledc_set_duty(LEDC_HIGH_SPEED_MODE, (ledc_channel_t)3, speed);
    }

    for(int i = 0; i < 4; i++) {
        ledc_update_duty(LEDC_HIGH_SPEED_MODE, (ledc_channel_t)i);
    }
}

static esp_err_t control_handler(httpd_req_t *req) {
    char* buf;
    size_t buf_len;
    char command[32] = {0,};

    buf_len = httpd_req_get_url_query_len(req) + 1;
    if (buf_len > 1) {
        buf = (char*)malloc(buf_len);
        if(!buf) {
            return ESP_FAIL;
        }
        if (httpd_req_get_url_query_str(req, buf, buf_len) == ESP_OK) {
            if (httpd_query_key_value(buf, "cmd", command, sizeof(command)) == ESP_OK) {
                if (strcmp(command, "flash_on") == 0) {
                    controlFlash(true);
                } else if (strcmp(command, "flash_off") == 0) {
                    controlFlash(false);
                } else {
                    controlMotors(command);
                }
            }
        }
        free(buf);
    }
    
    httpd_resp_set_type(req, "text/plain");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, "OK", 2);
}

static esp_err_t index_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, (const char *)index_html, strlen(index_html));
}

// Modify stream handler for better performance
static esp_err_t stream_handler(httpd_req_t *req) {
    camera_fb_t * fb = NULL;
    esp_err_t res = ESP_OK;
    char * part_buf[64];
    static int64_t last_frame = 0;
    
    res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
    if(res != ESP_OK) return res;

    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Cache-Control", "no-cache, no-store, must-revalidate");
    
    while(true) {
        fb = esp_camera_fb_get();
        if (!fb) {
            Serial.println("Camera capture failed");
            res = ESP_FAIL;
            break;
        }

        size_t hlen = snprintf((char *)part_buf, 64, _STREAM_PART, fb->len);
        res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
        if(res == ESP_OK) {
            res = httpd_resp_send_chunk(req, (const char *)part_buf, hlen);
        }
        if(res == ESP_OK) {
            res = httpd_resp_send_chunk(req, (const char *)fb->buf, fb->len);
        }
        
        esp_camera_fb_return(fb);
        
        if(res != ESP_OK) {
            break;
        }

        // Control frame rate (30fps max)
        int64_t frame_time = esp_timer_get_time() - last_frame;
        if(frame_time < 33333) { // 1000000/30 ≈ 33333
            delayMicroseconds(33333 - frame_time);
        }
        last_frame = esp_timer_get_time();
    }
    return res;
}

// Modify server config for better performance
void startCameraServer() {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 80;
    config.ctrl_port = 32767;
    config.max_open_sockets = 4;    // Increased for better concurrent connections
    config.lru_purge_enable = true;
    config.recv_wait_timeout = 5;   // Reduced wait timeout
    config.send_wait_timeout = 5;   // Reduced wait timeout
    config.core_id = 0;            // Run on core 0
    
    httpd_uri_t index_uri = {
        .uri       = "/",
        .method    = HTTP_GET,
        .handler   = index_handler,
        .user_ctx  = NULL
    };

    httpd_uri_t control_uri = {
        .uri       = "/control",
        .method    = HTTP_GET,
        .handler   = control_handler,
        .user_ctx  = NULL
    };

    if (control_httpd != NULL) {
        httpd_stop(control_httpd);  // Stop if already running
    }
    
    Serial.println("Starting web server on port: 80");
    if (httpd_start(&control_httpd, &config) == ESP_OK) {
        httpd_register_uri_handler(control_httpd, &index_uri);
        httpd_register_uri_handler(control_httpd, &control_uri);
        Serial.println("Web server started");
    } else {
        Serial.println("Error starting web server!");
        return;
    }

    config.server_port = 81;
    config.ctrl_port = 32768;

    httpd_uri_t stream_uri = {
        .uri       = "/stream",
        .method    = HTTP_GET,
        .handler   = stream_handler,
        .user_ctx  = NULL
    };

    if (stream_httpd != NULL) {
        httpd_stop(stream_httpd);  // Stop if already running
    }

    Serial.println("Starting stream server on port: 81");
    if (httpd_start(&stream_httpd, &config) == ESP_OK) {
        httpd_register_uri_handler(stream_httpd, &stream_uri);
        Serial.println("Stream server started");
    } else {
        Serial.println("Error starting stream server!");
    }
}

// Add WebSocket event handler
void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
    switch(type) {
        case WStype_DISCONNECTED:
            Serial.printf("[%u] Disconnected!\n", num);
            break;
        case WStype_CONNECTED:
            Serial.printf("[%u] Connected!\n", num);
            break;
        case WStype_TEXT:
            // Handle control commands
            if(length < 32) {
                char command[32];
                memcpy(command, payload, length);
                command[length] = '\0';
                controlMotors(command);
            }
            break;
    }
}

// Add task pinning to setup
void setup() {
    // Pin this task to core 0
    xTaskCreatePinnedToCore(
        controlTask,    // Function to implement the task
        "controlTask", // Name of the task
        10000,         // Stack size in words
        NULL,          // Task input parameter
        1,            // Priority of the task
        NULL,         // Task handle
        0             // Core where the task should run
    );
    
    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); // Disable brownout detector
    
    Serial.begin(115200);
    
    // Initial flash sequence
    pinMode(FLASH_GPIO_NUM, OUTPUT);
    digitalWrite(FLASH_GPIO_NUM, HIGH);
    delay(2000);
    digitalWrite(FLASH_GPIO_NUM, LOW);
    
    // Set motor pins as outputs and ensure they're off
    pinMode(MOTOR_1_PIN_1, OUTPUT);
    pinMode(MOTOR_1_PIN_2, OUTPUT);
    pinMode(MOTOR_2_PIN_1, OUTPUT);
    pinMode(MOTOR_2_PIN_2, OUTPUT);
    
    digitalWrite(MOTOR_1_PIN_1, LOW);
    digitalWrite(MOTOR_1_PIN_2, LOW);
    digitalWrite(MOTOR_2_PIN_1, LOW);
    digitalWrite(MOTOR_2_PIN_2, LOW);
    
    // Initialize motors with PWM
    setupMotors();
    
    // Make sure all motors are off after PWM setup
    controlMotors("stop");
    
    // Camera configuration
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
    config.pin_sscb_sda = SIOD_GPIO_NUM;
    config.pin_sscb_scl = SIOC_GPIO_NUM;
    config.pin_pwdn = PWDN_GPIO_NUM;
    config.pin_reset = RESET_GPIO_NUM;
    config.xclk_freq_hz = 20000000;  // 20MHz XCLK for OV2640
    config.pixel_format = PIXFORMAT_JPEG;
    
    // Optimize camera settings for better performance
    if(psramFound()) {
        config.frame_size = FRAMESIZE_VGA;    // Reduced to VGA for better performance
        config.jpeg_quality = 10;             // Reduced quality for faster transmission
        config.fb_count = 2;
        config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;  // Changed grab mode
    } else {
        config.frame_size = FRAMESIZE_QVGA;   // Even smaller for non-PSRAM boards
        config.jpeg_quality = 12;
        config.fb_count = 1;
        config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
    }

    // Initialize camera
    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        Serial.printf("Camera init failed with error 0x%x", err);
        delay(1000);
        ESP.restart();
    }

    // Optimize sensor settings for better real-time performance
    sensor_t * s = esp_camera_sensor_get();
    if (s) {
        s->set_framesize(s, config.frame_size);
        s->set_quality(s, config.jpeg_quality);
        s->set_brightness(s, 0);     // Default brightness
        s->set_contrast(s, 0);       // Default contrast
        s->set_saturation(s, 0);     // Default saturation
        s->set_special_effect(s, 0); // No special effects
        s->set_whitebal(s, 1);      // Enable Auto White Balance
        s->set_awb_gain(s, 1);      // Enable AWB Gain
        s->set_wb_mode(s, 0);       // Auto WB
        s->set_exposure_ctrl(s, 1);  // Enable Auto Exposure
        s->set_aec2(s, 1);          // Enable AEC DSP
        s->set_ae_level(s, 0);      // Default AE Level
        s->set_aec_value(s, 300);   // AEC Value
        s->set_gain_ctrl(s, 1);     // Enable Auto Gain
        s->set_agc_gain(s, 0);      // Default Gain
        s->set_gainceiling(s, (gainceiling_t)0); // Auto Gain Ceiling
        s->set_bpc(s, 0);          // Disable BPC
        s->set_wpc(s, 0);          // Disable WPC
        s->set_raw_gma(s, 1);      // Enable GMA
        s->set_lenc(s, 0);         // Disable LENC
        s->set_hmirror(s, 0);      // No mirror
        s->set_vflip(s, 0);        // No flip
        s->set_dcw(s, 1);          // Enable DCW
        s->set_colorbar(s, 0);     // Disable Colorbar
    }

    // Wi-Fi connection
    WiFi.begin(ssid, password);
    int retries = 0;
    while (WiFi.status() != WL_CONNECTED && retries < 30) {
        delay(500);
        Serial.print(".");
        retries++;
    }
    
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("Failed to connect to WiFi! Restarting...");
        ESP.restart();
    }

    Serial.println("");
    Serial.println("WiFi connected");
    Serial.print("Camera Ready! Use 'http://");
    Serial.print(WiFi.localIP());
    Serial.println("' to connect");

    // Add this line to send IP address via Telegram
    sendTelegramMessage(WiFi.localIP().toString().c_str());

    // Wait for WiFi to initialize properly
    delay(100);  // Add small delay for WiFi to stabilize

    // Start camera server
    startCameraServer();

    Serial.print("Camera Stream Ready! Connect to the AP '");
    Serial.print(ssid);
    Serial.print("' using password '");
    Serial.print(password);
    Serial.print("', then go to http://");
    Serial.println(WiFi.localIP());

    // Add after WiFi connection
    webSocket.begin();
    webSocket.onEvent(webSocketEvent);
}

// Add this function before setup()
void sendTelegramMessage(const char* ipAddress) {
    client.setInsecure(); // Skip certificate validation
    String message = "ESP32-CAM is connected with IP: ";
    message += ipAddress;
    
    if (bot.sendMessage(CHAT_ID, message, "")) {
        Serial.println("IP Address sent to Telegram");
    } else {
        Serial.println("Failed to send IP to Telegram");
    }
}

// Control task running on core 0
void controlTask(void * parameter) {
    for(;;) {
        if (WiFi.status() != WL_CONNECTED) {
            Serial.println("WiFi disconnected, reconnecting...");
            WiFi.reconnect();
            delay(1000);
        }
        delay(100); // Give other tasks time to run
        
        webSocket.loop();
        
        // Stream camera frames over WebSocket
        unsigned long currentTime = millis();
        if (currentTime - lastStreamTime >= streamInterval) {
            camera_fb_t * fb = esp_camera_fb_get();
            if (fb) {
                webSocket.broadcastBIN(fb->buf, fb->len);
                esp_camera_fb_return(fb);
            } else {
                Serial.println("Failed to capture frame");
            }
            lastStreamTime = currentTime;
        }
        
        delay(1);
    }
}

// Modified loop to run on core 1
void loop() {
    // Main control loop now runs on core 1
    delay(1); // Small delay to prevent watchdog triggers
}