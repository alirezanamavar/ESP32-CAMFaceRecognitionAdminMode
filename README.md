# ESP32-CAM Face Recognition Door Lock System

This project implements a face recognition door lock system using the ESP32-CAM module. The system captures images, detects and recognizes faces, and controls a door lock via a relay. It also provides a web interface for monitoring and controlling the system using WebSockets.

## Table of Contents

1. [Introduction](#introduction)
2. [Hardware Requirements](#hardware-requirements)
3. [Libraries and Definitions](#libraries-and-definitions)
4. [Global Variables and Constants](#global-variables-and-constants)
5. [Function Declarations](#function-declarations)
6. [Face Recognition Configuration](#face-recognition-configuration)
7. [Setup Function](#setup-function)
8. [Face Recognition Initialization](#face-recognition-initialization)
9. [Door Control Functions](#door-control-functions)
10. [Main Loop](#main-loop)
11. [Detailed Explanation of Face Detection and Recognition](#detailed-explanation-of-face-detection-and-recognition)
12. [Neural Network Usage](#neural-network-usage)
13. [Libraries and Functions](#libraries-and-functions)
14. [Conclusion](#conclusion)

## Introduction

This project implements a face recognition door lock system using the ESP32-CAM module. The system captures images, detects and recognizes faces, and unlocks the door when an authorized face is recognized. It also provides a web interface for monitoring and controlling the system using WebSockets.

## Hardware Requirements

- **ESP32-CAM Module:** For capturing images and processing face recognition.
- **Relay Module:** To control the door lock mechanism.
- **Power Supply:** To power the ESP32-CAM and the relay.

## Libraries and Definitions

```cpp
#include <ArduinoWebsockets.h>     // WebSockets library for communication between ESP32 and browser
#include "esp_http_server.h"       // HTTP server library for ESP32
#include "esp_timer.h"             // Timer library for ESP32
#include "esp_camera.h"            // Camera library for ESP32-CAM
#include "camera_index.h"          // Contains HTML page for web interface
#include "Arduino.h"               // Core Arduino functions
#include "fd_forward.h"            // Face detection functions
#include "fr_forward.h"            // Face recognition functions
#include "fr_flash.h"              // Functions for storing face data in flash memory

const char* ssid = "iPhone";       // Wi-Fi network SSID
const char* password = "12345678"; // Wi-Fi network password

#define ENROLL_CONFIRM_TIMES 5     // Number of times to confirm face enrollment
#define FACE_ID_SAVE_NUMBER 100    // Maximum number of faces to store

#define CAMERA_MODEL_AI_THINKER    // Define camera model as AI Thinker
#include "camera_pins.h"           // Include camera pin definitions
```

- **ArduinoWebsockets.h:** Enables WebSocket communication between the ESP32 and a web browser.
- **esp_http_server.h:** Allows hosting an HTTP server on the ESP32.
- **esp_timer.h:** Provides timing functions.
- **esp_camera.h:** Enables camera functionality on the ESP32-CAM.
- **camera_index.h:** Contains the HTML page for the web interface.
- **fd_forward.h, fr_forward.h, fr_flash.h:** Provide face detection and recognition capabilities, including storing face data in flash memory.

## Global Variables and Constants

```cpp
using namespace websockets;        // Use the websockets namespace
WebsocketsServer socket_server;    // Create an instance of the WebSockets server

camera_fb_t * fb = NULL;           // Pointer to the camera frame buffer

long current_millis;
long last_detected_millis = 0;     // Time of the last face detection

#define relay_pin 2                // GPIO pin connected to the relay
unsigned long door_opened_millis = 0; // Time when the door was opened
long interval = 5000;              // Duration the door remains unlocked (in milliseconds)
bool face_recognised = false;      // Flag indicating if a face was recognized
```

- **ENROLL_CONFIRM_TIMES:** Number of times a face must be detected for successful enrollment.
- **FACE_ID_SAVE_NUMBER:** Maximum number of faces that can be stored.
- **relay_pin:** GPIO pin connected to the relay controlling the door lock mechanism.
- **door_opened_millis:** Records the time when the door was last unlocked.
- **interval:** Time in milliseconds the door remains unlocked after recognizing a face.
- **face_recognised:** Flag indicating whether a face has been recognized.

## Function Declarations

```cpp
void app_facenet_main();           // Function to initialize face recognition
void app_httpserver_init();        // Function to initialize the HTTP server
```

## Face Recognition Configuration

### Image Processing Result Structure

```cpp
typedef struct
{
  uint8_t *image;                  // Pointer to image data
  box_array_t *net_boxes;          // Pointer to detected face boxes
  dl_matrix3d_t *face_id;          // Pointer to face ID matrix
} http_img_process_result;
```

- **image:** Holds the image data.
- **net_boxes:** Contains detected face bounding boxes.
- **face_id:** Unique identifier extracted from a face image.

### Face Detection Configuration with MTCNN

```cpp
static inline mtmn_config_t app_mtmn_config()
{
  mtmn_config_t mtmn_config = {0};     // Initialize MTCNN configuration structure
  mtmn_config.type = FAST;             // Set detection type to FAST
  mtmn_config.min_face = 80;           // Minimum face size to detect (in pixels)
  mtmn_config.pyramid = 0.707;         // Pyramid scaling factor
  mtmn_config.pyramid_times = 4;       // Number of pyramid levels
  mtmn_config.p_threshold.score = 0.6; // P-Net score threshold
  mtmn_config.p_threshold.nms = 0.7;   // P-Net NMS threshold
  mtmn_config.p_threshold.candidate_number = 20; // Number of P-Net candidates
  mtmn_config.r_threshold.score = 0.7; // R-Net score threshold
  mtmn_config.r_threshold.nms = 0.7;   // R-Net NMS threshold
  mtmn_config.r_threshold.candidate_number = 10; // Number of R-Net candidates
  mtmn_config.o_threshold.score = 0.7; // O-Net score threshold
  mtmn_config.o_threshold.nms = 0.7;   // O-Net NMS threshold
  mtmn_config.o_threshold.candidate_number = 1;  // Number of O-Net candidates
  return mtmn_config;                   // Return the configured MTCNN settings
}
mtmn_config_t mtmn_config = app_mtmn_config();    // Initialize MTCNN configuration
```

- **mtmn_config_t:** Configuration structure for the Multi-Task Cascaded Convolutional Neural Network (MTCNN) used for face detection.
- **Thresholds:** Define the sensitivity and accuracy of face detection at different stages (P-Net, R-Net, O-Net).

### Face ID List and Aligned Face Matrix

```cpp
face_id_name_list st_face_list;                   // Face ID list
static dl_matrix3du_t *aligned_face = NULL;       // Pointer to aligned face image data

httpd_handle_t camera_httpd = NULL;               // HTTP server handle
```

- **st_face_list:** Stores face IDs and associated names.
- **aligned_face:** Pointer to the aligned face image data.

### FSM (Finite State Machine) States

```cpp
typedef enum
{
  START_STREAM,        // State to start video streaming
  START_DETECT,        // State to start face detection
  SHOW_FACES,          // State to show the list of faces
  START_RECOGNITION,   // State to start face recognition
  START_ENROLL,        // State to start face enrollment
  ENROLL_COMPLETE,     // State when enrollment is complete
  DELETE_ALL,          // State to delete all faces
} en_fsm_state;
en_fsm_state g_state = START_RECOGNITION;  // Auto-start face recognition on boot
```

### HTTP Response Structure

```cpp
typedef struct
{
  char enroll_name[ENROLL_NAME_LEN];  // Name used for enrollment
} httpd_resp_value;

httpd_resp_value st_name;             // Instance of HTTP response value
```

## Setup Function

```cpp
void setup() {
  Serial.begin(115200);            // Initialize serial communication at 115200 baud
  Serial.setDebugOutput(true);     // Enable debug output
  Serial.println();

  digitalWrite(relay_pin, LOW);    // Ensure the relay is initially off (door locked)
  pinMode(relay_pin, OUTPUT);      // Set the relay pin as an output

  camera_config_t config;          // Create a camera configuration structure
  config.ledc_channel = LEDC_CHANNEL_0; // Set LEDC PWM channel for camera clock
  config.ledc_timer = LEDC_TIMER_0;     // Set LEDC timer for camera clock
  // Assign camera data pins D0-D7
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;      // Assign external clock pin
  config.pin_pclk = PCLK_GPIO_NUM;      // Assign pixel clock pin
  config.pin_vsync = VSYNC_GPIO_NUM;    // Assign vertical sync pin
  config.pin_href = HREF_GPIO_NUM;      // Assign horizontal reference pin
  config.pin_sscb_sda = SIOD_GPIO_NUM;  // Assign SCCB data pin
  config.pin_sscb_scl = SIOC_GPIO_NUM;  // Assign SCCB clock pin
  config.pin_pwdn = PWDN_GPIO_NUM;      // Assign power-down pin
  config.pin_reset = RESET_GPIO_NUM;    // Assign reset pin
  config.xclk_freq_hz = 20000000;       // Set XCLK frequency to 20 MHz
  config.pixel_format = PIXFORMAT_JPEG; // Set pixel format to JPEG

  // Initialize with high specs to pre-allocate larger buffers
  if (psramFound()) {                   // Check if PSRAM is available
    config.frame_size = FRAMESIZE_UXGA; // Set frame size to UXGA (1600x1200)
    config.jpeg_quality = 10;           // Set JPEG quality (lower number = higher quality)
    config.fb_count = 2;                // Use two frame buffers
  } else {
    config.frame_size = FRAMESIZE_SVGA; // Set frame size to SVGA (800x600)
    config.jpeg_quality = 12;           // Set JPEG quality
    config.fb_count = 1;                // Use one frame buffer
  }

#if defined(CAMERA_MODEL_ESP_EYE)
  pinMode(13, INPUT_PULLUP);            // Configure button pins for ESP-EYE
  pinMode(14, INPUT_PULLUP);
#endif

  // Camera initialization
  esp_err_t err = esp_camera_init(&config); // Initialize the camera with the configuration
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err); // Print error message if initialization fails
    return;                      // Exit the setup function
  }

  sensor_t * s = esp_camera_sensor_get(); // Get the camera sensor handle
  s->set_framesize(s, FRAMESIZE_QVGA);    // Set frame size to QVGA (320x240)

#if defined(CAMERA_MODEL_M5STACK_WIDE)
  s->set_vflip(s, 1);                     // Flip the image vertically
  s->set_hmirror(s, 1);                   // Mirror the image horizontally
#endif

  WiFi.begin(ssid, password);             // Begin Wi-Fi connection
  while (WiFi.status() != WL_CONNECTED) { // Wait until connected
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");

  app_httpserver_init();                  // Initialize the HTTP server
  app_facenet_main();                     // Initialize face recognition components
  socket_server.listen(82);               // Start WebSocket server on port 82

  Serial.print("Camera Ready! Use 'http://");
  Serial.print(WiFi.localIP());           // Print the IP address
  Serial.println("' to connect");
}
```

- **Serial Communication:** Initializes serial communication for debugging.
- **Relay Setup:** Configures the relay pin as an output and ensures the door is locked initially.
- **Camera Configuration:** Sets up the camera with appropriate settings, including resolution and frame buffers.
- **Wi-Fi Connection:** Connects to the specified Wi-Fi network.
- **Server Initialization:** Starts the HTTP and WebSocket servers.
- **Face Recognition Initialization:** Calls `app_facenet_main()` to set up face recognition.

## Face Recognition Initialization

```cpp
void app_facenet_main()
{
  face_id_name_init(&st_face_list, FACE_ID_SAVE_NUMBER, ENROLL_CONFIRM_TIMES); // Initialize the face ID list
  aligned_face = dl_matrix3du_alloc(1, FACE_WIDTH, FACE_HEIGHT, 3);            // Allocate memory for aligned face data
  read_face_id_from_flash_with_name(&st_face_list);                            // Load saved face IDs from flash memory
}
```

- **face_id_name_init():** Initializes the face ID list with the specified capacity and enrollment confirmations.
- **dl_matrix3du_alloc():** Allocates memory for storing the aligned face image.
- **read_face_id_from_flash_with_name():** Loads saved face IDs and names from flash memory into `st_face_list`.

## Door Control Functions

### Open Door Function

```cpp
void open_door(WebsocketsClient &client) {
  if (digitalRead(relay_pin) == LOW) {
    digitalWrite(relay_pin, HIGH); // Energize the relay to unlock the door
    Serial.println("Door Unlocked");
    client.send("door_open");      // Inform the client
    door_opened_millis = millis(); // Record the time when the door was unlocked
  }
}
```

- **open_door():** Activates the relay to unlock the door and records the unlock time.

## Main Loop

```cpp
void loop() {
  auto client = socket_server.accept();   // Accept a new WebSocket client
  client.onMessage(handle_message);       // Set the message handler

  dl_matrix3du_t *image_matrix = dl_matrix3du_alloc(1, 320, 240, 3); // Allocate memory for the image matrix
  http_img_process_result out_res = {0};     // Initialize image processing result structure
  out_res.image = image_matrix->item;        // Set the image pointer to the image matrix data

  send_face_list(client);                // Send the list of faces to the client
  client.send("STREAMING");              // Inform the client that streaming has started

  while (client.available()) {           // While client is connected
    client.poll();                       // Process incoming messages

    if (millis() - interval > door_opened_millis) { // Check if it's time to lock the door
      digitalWrite(relay_pin, LOW); // De-energize the relay to lock the door
    }

    fb = esp_camera_fb_get();            // Capture a frame from the camera

    if (g_state == START_DETECT || g_state == START_ENROLL || g_state == START_RECOGNITION)
    {
      out_res.net_boxes = NULL;          // Reset detection results
      out_res.face_id = NULL;

      fmt2rgb888(fb->buf, fb->len, fb->format, out_res.image); // Convert the frame buffer to RGB888 format

      out_res.net_boxes = face_detect(image_matrix, &mtmn_config); // Perform face detection on the image

      if (out_res.net_boxes)
      {
        // Align the detected face and check if alignment was successful
        if (align_face(out_res.net_boxes, image_matrix, aligned_face) == ESP_OK)
        {
          out_res.face_id = get_face_id(aligned_face); // Extract face ID from the aligned face
          last_detected_millis = millis();             // Update the last detected time

          if (g_state == START_DETECT) {
            client.send("FACE DETECTED");              // Inform client
          }

          if (g_state == START_ENROLL)
          {
            int left_sample_face = do_enrollment(&st_face_list, out_res.face_id); // Enroll the face ID
            char enrolling_message[64];
            sprintf(enrolling_message, "SAMPLE NUMBER %d FOR %s", ENROLL_CONFIRM_TIMES - left_sample_face, st_name.enroll_name);
            client.send(enrolling_message);           // Inform client of enrollment progress
            if (left_sample_face == 0)
            {
              ESP_LOGI(TAG, "Enrolled Face ID: %s", st_face_list.tail->id_name);
              g_state = START_STREAM;                // Reset state to streaming
              char captured_message[64];
              sprintf(captured_message, "FACE CAPTURED FOR %s", st_face_list.tail->id_name);
              client.send(captured_message);         // Inform client that enrollment is complete
              send_face_list(client);                // Update face list on client side
            }
          }

          if (g_state == START_RECOGNITION  && (st_face_list.count > 0))
          {
            face_id_node *f = recognize_face_with_name(&st_face_list, out_res.face_id); // Recognize the face
            if (f)
            {
              char recognised_message[64];
              sprintf(recognised_message, "DOOR OPEN FOR %s", f->id_name);
              open_door(client);                     // Unlock the door
              client.send(recognised_message);       // Inform client
            }
            else
            {
              client.send("FACE NOT RECOGNISED");    // Inform client that face was not recognized
            }
          }
          dl_matrix3d_free(out_res.face_id);         // Free the face ID matrix memory
        }

      }
      else
      {
        if (g_state != START_DETECT) {
          client.send("NO FACE DETECTED");           // Inform client that no face was detected
        }
      }

      if (g_state == START_DETECT && millis() - last_detected_millis > 500) { // If detecting but no face detected
        client.send("DETECTING");                   // Inform client
      }

    }

    client.sendBinary((const char *)fb->buf, fb->len); // Send the frame buffer to the client

    esp_camera_fb_return(fb);                   // Return the frame buffer to the driver
    fb = NULL;                                  // Reset the frame buffer pointer
  }
}
```

- **Client Handling:** Accepts a WebSocket client and sets up the message handler.
- **Image Processing:** Captures images, performs face detection, alignment, and recognition.
- **State Management:** Based on the current state, the system performs actions such as streaming, detecting faces, enrolling new faces, or recognizing faces.
- **Door Control:** Unlocks the door if a recognized face is detected.
- **Client Communication:** Sends updates and image frames to the client via WebSockets.
- **Memory Management:** Frees allocated memory to prevent leaks.

## Detailed Explanation of Face Detection and Recognition

### Face Detection with MTCNN

- **MTCNN (Multi-Task Cascaded Convolutional Neural Network):** A neural network framework used for face detection.
- **Stages of MTCNN:**
  - **P-Net (Proposal Network):** Quickly generates candidate face regions.
  - **R-Net (Refinement Network):** Refines the candidate regions.
  - **O-Net (Output Network):** Produces final face bounding boxes and landmarks.
- **Configuration Parameters:**
  - **Score Thresholds:** Determine the confidence level required to consider a detection valid.
  - **NMS (Non-Maximum Suppression):** Eliminates overlapping bounding boxes to reduce false positives.
  - **Candidate Numbers:** Limits the number of proposals to improve performance.

### Face Alignment

- **Purpose:** Standardizes the position, size, and orientation of the face for better recognition accuracy.
- **Process:** Uses facial landmarks (e.g., eyes, nose) to align the face image.

### Face Recognition with Neural Networks

- **Feature Extraction:**
  - The aligned face image is processed by a neural network to extract a face ID (feature vector).
  - **get_face_id():** Function that returns a numerical representation of the face.
- **Face Matching:**
  - **recognize_face_with_name():** Compares the extracted face ID with stored IDs.
  - **Similarity Metrics:** Uses Euclidean distance or cosine similarity to determine if the faces match.
- **Enrollment Confirmation:**
  - Multiple confirmations are required during enrollment to ensure the accuracy of the stored face ID.

## Neural Network Usage

- **Pre-trained Models:** The system uses pre-trained neural networks for both face detection and recognition.
- **Face Detection Network:** MTCNN is used to detect faces in images.
- **Face Recognition Network:** Extracts unique features from faces to create a face ID.
- **Feature Vectors:** High-dimensional numerical representations that uniquely identify a face.

## Libraries and Functions

### Key Libraries

- **esp_camera.h:** Camera initialization and image capture functions.
- **esp_timer.h:** Timing functions, such as `millis()`.
- **fd_forward.h and fr_forward.h:** Provide face detection and recognition functionalities.
- **fr_flash.h:** Functions for reading and writing face data to flash memory.

### Important Functions

- **Camera Functions:**
  - **esp_camera_init():** Initializes the camera.
  - **esp_camera_fb_get():** Captures an image frame.
  - **esp_camera_fb_return():** Releases the frame buffer.

- **Image Processing Functions:**
  - **fmt2rgb888():** Converts image data to RGB888 format.
  - **dl_matrix3du_alloc() and dl_matrix3du_free():** Allocate and free memory for image data.

- **Face Detection and Recognition Functions:**
  - **face_detect():** Detects faces in an image.
  - **align_face():** Aligns face images.
  - **get_face_id():** Extracts face IDs from aligned faces.
  - **recognize_face_with_name():** Matches face IDs with stored IDs.

- **Flash Memory Functions:**
  - **read_face_id_from_flash_with_name():** Loads face IDs from flash.
  - **enroll_face_id_to_flash_with_name():** Saves a new face ID to flash.
  - **delete_face_id_in_flash_with_name():** Deletes a face ID from flash.
  - **delete_face_all_in_flash_with_name():** Deletes all face IDs from flash.

### Memory Management

- **dl_matrix3du_alloc() and dl_matrix3du_free():** Manage memory allocation and deallocation for images.
- **dl_matrix3d_free():** Frees memory allocated for face ID data.

## Conclusion

This code provides a comprehensive solution for a face recognition-based door lock system using the ESP32-CAM module. By leveraging neural networks for face detection and recognition, it can accurately identify authorized individuals and control access through a door lock mechanism. The use of flash memory allows for persistent storage of face data, making the system robust and reliable. The web interface powered by WebSockets enables the user to monitor and control the system through a user-friendly interface.

---

**Note:** Ensure that all the required libraries are included in your project and that the ESP32-CAM module is correctly connected and configured. Proper power supply and relay wiring are crucial for the safe and reliable operation of the door lock mechanism.
