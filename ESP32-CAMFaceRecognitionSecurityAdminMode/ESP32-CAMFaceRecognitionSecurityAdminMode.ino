#include <ArduinoWebsockets.h>     // Include the Arduino WebSockets library
#include "esp_http_server.h"       // Include the ESP32 HTTP server library
#include "esp_timer.h"             // Include the ESP32 timer library
#include "esp_camera.h"            // Include the ESP32 camera library
#include "camera_index.h"          // Include the camera index for web interface
#include "Arduino.h"               // Include the Arduino core library
#include "fd_forward.h"            // Include face detection forward declarations
#include "fr_forward.h"            // Include face recognition forward declarations
#include "fr_flash.h"              // Include face recognition flash storage functions

const char* ssid = "iPhone";       // Define the Wi-Fi SSID
const char* password = "12345678"; // Define the Wi-Fi password

#define ENROLL_CONFIRM_TIMES 5     // Number of times to confirm enrollment
#define FACE_ID_SAVE_NUMBER 100    // Maximum number of faces to save

#define CAMERA_MODEL_AI_THINKER    // Define the camera model as AI Thinker
#include "camera_pins.h"           // Include camera pin definitions

using namespace websockets;        // Use the websockets namespace
WebsocketsServer socket_server;    // Create a WebSockets server instance

camera_fb_t * fb = NULL;           // Initialize the frame buffer pointer to NULL

long current_millis;
long last_detected_millis = 0;     // Variable to store the last face detection time

#define relay_pin 2                // Define the relay pin (GPIO 2)
unsigned long door_opened_millis = 0; // Variable to store the time when the door was opened
long interval = 5000;              // Duration (in milliseconds) to keep the door unlocked
bool face_recognised = false;      // Flag to indicate if a face was recognized

void app_facenet_main();           // Function prototype for face recognition initialization
void app_httpserver_init();        // Function prototype for HTTP server initialization

// Structure to hold image processing results
typedef struct
{
  uint8_t *image;                  // Pointer to the image data
  box_array_t *net_boxes;          // Pointer to the detected face boxes
  dl_matrix3d_t *face_id;          // Pointer to the face ID matrix
} http_img_process_result;

// Function to configure the MTCNN face detection
static inline mtmn_config_t app_mtmn_config()
{
  mtmn_config_t mtmn_config = {0};     // Initialize MTCNN configuration structure
  mtmn_config.type = FAST;             // Set detection type to FAST
  mtmn_config.min_face = 80;           // Set minimum face size to detect
  mtmn_config.pyramid = 0.707;         // Set pyramid scaling factor
  mtmn_config.pyramid_times = 4;       // Set number of pyramid levels
  mtmn_config.p_threshold.score = 0.6; // Set P-Net score threshold
  mtmn_config.p_threshold.nms = 0.7;   // Set P-Net non-maximum suppression threshold
  mtmn_config.p_threshold.candidate_number = 20; // Set P-Net candidate number
  mtmn_config.r_threshold.score = 0.7; // Set R-Net score threshold
  mtmn_config.r_threshold.nms = 0.7;   // Set R-Net non-maximum suppression threshold
  mtmn_config.r_threshold.candidate_number = 10; // Set R-Net candidate number
  mtmn_config.o_threshold.score = 0.7; // Set O-Net score threshold
  mtmn_config.o_threshold.nms = 0.7;   // Set O-Net non-maximum suppression threshold
  mtmn_config.o_threshold.candidate_number = 1;  // Set O-Net candidate number
  return mtmn_config;                   // Return the configured MTCNN settings
}
mtmn_config_t mtmn_config = app_mtmn_config();    // Initialize MTCNN configuration

face_id_name_list st_face_list;                   // Initialize the face ID list structure
static dl_matrix3du_t *aligned_face = NULL;       // Pointer for aligned face data

httpd_handle_t camera_httpd = NULL;               // HTTP server handle

// Enumeration for different FSM states
typedef enum
{
  START_STREAM,        // State for starting the video stream
  START_DETECT,        // State for starting face detection
  SHOW_FACES,          // State for showing the list of faces
  START_RECOGNITION,   // State for starting face recognition
  START_ENROLL,        // State for starting face enrollment
  ENROLL_COMPLETE,     // State when enrollment is complete
  DELETE_ALL,          // State for deleting all faces
} en_fsm_state;
en_fsm_state g_state = START_RECOGNITION;  // Auto-start face recognition on boot

// Structure to hold HTTP response values
typedef struct
{
  char enroll_name[ENROLL_NAME_LEN];  // Name used for enrollment
} httpd_resp_value;

httpd_resp_value st_name;             // Instance of HTTP response value

void setup() {
  Serial.begin(115200);            // Initialize serial communication at 115200 baud
  Serial.setDebugOutput(true);     // Enable debug output
  Serial.println();

  digitalWrite(relay_pin, LOW);    // Ensure the relay is initially off (door locked)
  pinMode(relay_pin, OUTPUT);      // Set the relay pin as an output

  camera_config_t config;          // Create a camera configuration structure
  config.ledc_channel = LEDC_CHANNEL_0; // Set LEDC PWM channel
  config.ledc_timer = LEDC_TIMER_0;     // Set LEDC timer
  config.pin_d0 = Y2_GPIO_NUM;          // Assign camera data pins D0-D7
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;      // Assign camera clock pin
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

// Handler for the root URI "/"
static esp_err_t index_handler(httpd_req_t *req) {
  httpd_resp_set_type(req, "text/html");          // Set response content type to HTML
  httpd_resp_set_hdr(req, "Content-Encoding", "gzip"); // Set header for gzip encoding
  return httpd_resp_send(req, (const char *)index_ov2640_html_gz, index_ov2640_html_gz_len); // Send the webpage
}

// Define the URI handler structure
httpd_uri_t index_uri = {
  .uri       = "/",             // Match the root URI "/"
  .method    = HTTP_GET,        // Handle GET requests
  .handler   = index_handler,   // Function to handle the request
  .user_ctx  = NULL             // No user context
};

void app_httpserver_init ()
{
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();    // Use default HTTP server configuration
  if (httpd_start(&camera_httpd, &config) == ESP_OK) // Start the HTTP server
    Serial.println("httpd_start");
  {
    httpd_register_uri_handler(camera_httpd, &index_uri); // Register the root URI handler
  }
}

void app_facenet_main()
{
  face_id_name_init(&st_face_list, FACE_ID_SAVE_NUMBER, ENROLL_CONFIRM_TIMES); // Initialize the face ID list
  aligned_face = dl_matrix3du_alloc(1, FACE_WIDTH, FACE_HEIGHT, 3);            // Allocate memory for aligned face data
  read_face_id_from_flash_with_name(&st_face_list);                            // Load saved face IDs from flash memory
}

// Function to handle enrollment of a new face
static inline int do_enrollment(face_id_name_list *face_list, dl_matrix3d_t *new_id)
{
  ESP_LOGD(TAG, "START ENROLLING");
  int left_sample_face = enroll_face_id_to_flash_with_name(face_list, new_id, st_name.enroll_name); // Enroll the face ID
  ESP_LOGD(TAG, "Face ID %s Enrollment: Sample %d",
           st_name.enroll_name,
           ENROLL_CONFIRM_TIMES - left_sample_face);
  return left_sample_face; // Return the number of samples left
}

// Function to send the list of faces to the client
static esp_err_t send_face_list(WebsocketsClient &client)
{
  client.send("delete_faces"); // Tell browser to delete all faces
  face_id_node *head = st_face_list.head;
  char add_face[64];
  for (int i = 0; i < st_face_list.count; i++) // Loop through current faces
  {
    sprintf(add_face, "listface:%s", head->id_name); // Format the face name
    client.send(add_face); // Send face to browser
    head = head->next;
  }
}

// Function to delete all faces
static esp_err_t delete_all_faces(WebsocketsClient &client)
{
  delete_face_all_in_flash_with_name(&st_face_list); // Delete all faces from flash
  client.send("delete_faces"); // Inform the client to delete faces
}

// Handler for incoming WebSocket messages
void handle_message(WebsocketsClient &client, WebsocketsMessage msg)
{
  if (msg.data() == "stream") {
    g_state = START_STREAM;     // Set state to start streaming
    client.send("STREAMING");   // Inform client
  }
  if (msg.data() == "detect") {
    g_state = START_DETECT;     // Set state to start detection
    client.send("DETECTING");   // Inform client
  }
  if (msg.data().substring(0, 8) == "capture:") {
    g_state = START_ENROLL;     // Set state to start enrollment
    char person[FACE_ID_SAVE_NUMBER * ENROLL_NAME_LEN] = {0,};
    msg.data().substring(8).toCharArray(person, sizeof(person)); // Extract person name
    memcpy(st_name.enroll_name, person, strlen(person) + 1);     // Copy name for enrollment
    client.send("CAPTURING");   // Inform client
  }
  if (msg.data() == "recognise") {
    g_state = START_RECOGNITION; // Set state to start recognition
    client.send("RECOGNISING");  // Inform client
  }
  if (msg.data().substring(0, 7) == "remove:") {
    char person[ENROLL_NAME_LEN * FACE_ID_SAVE_NUMBER];
    msg.data().substring(7).toCharArray(person, sizeof(person)); // Extract person name
    delete_face_id_in_flash_with_name(&st_face_list, person);    // Delete face ID
    send_face_list(client); // Reset faces in the browser
  }
  if (msg.data() == "delete_all") {
    delete_all_faces(client);    // Delete all faces
  }
}

// Function to unlock the door
void open_door(WebsocketsClient &client) {
  if (digitalRead(relay_pin) == LOW) {
    digitalWrite(relay_pin, HIGH); // Energize the relay to unlock the door
    Serial.println("Door Unlocked");
    client.send("door_open");      // Inform client
    door_opened_millis = millis(); // Record the time when the door was unlocked
  }
}

void loop() {
  auto client = socket_server.accept();   // Accept a new WebSocket client
  client.onMessage(handle_message);       // Set the message handler

  dl_matrix3du_t *image_matrix = dl_matrix3du_alloc(1, 320, 240, 3); // Allocate memory for the image matrix
  http_img_process_result out_res = {0};     // Initialize image processing result structure
  out_res.image = image_matrix->item;        // Set the image pointer to the image matrix data

  send_face_list(client);                // Send the list of faces to the client
  client.send("STREAMING");              // Inform client that streaming has started

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
