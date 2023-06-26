//MEO-E9F720
//53349fb336
//192.168.1.171
//1883

#include "esp_camera.h"
#include <WiFi.h>
#include <PubSubClient.h>

#include <base64.h>
#include "ArduinoJson.h"

// Select camera model
//#define CAMERA_MODEL_WROVER_KIT
//#define CAMERA_MODEL_ESP_EYE
//#define CAMERA_MODEL_M5STACK_PSRAM
//#define CAMERA_MODEL_M5STACK_WIDE
#define CAMERA_MODEL_AI_THINKER

#include "camera_pins.h"



// Flash
#define LED_BUILTIN 4

//WIFI config
const char* ssid = "NOTEVITORIA";
const char* password = "123vba456";

//MQTT config
bool useMQTT = true;
const char* mqttServer = "192.168.1.171";
const char* HostName = "mqtt.vitoria.pc";
const char* mqttUser = "";
const char* mqttPassword = "";
const char* topic_PHOTO = "MOTION/DETECTION";
const char* topic_PUBLISH = "PICTURE";
const char* topic_FLASH = "FLASH";
const int MAX_PAYLOAD = 60000;

bool flash;

WiFiClient espClient;
PubSubClient client(espClient);



void startCameraServer();

void setup() {

  // Define Flash as an output
  pinMode(LED_BUILTIN, OUTPUT);

  // Initialise the Serial Communication
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();

  // Config Camera Settings
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
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  //init with high specs to pre-allocate larger buffers
  if(psramFound()){
    config.frame_size = FRAMESIZE_UXGA;
    config.jpeg_quality = 10;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }

  flash = true;

  // Not used in our project
  #if defined(CAMERA_MODEL_ESP_EYE)
    pinMode(13, INPUT_PULLUP);
    pinMode(14, INPUT_PULLUP);
  #endif

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  sensor_t * s = esp_camera_sensor_get();
  //initial sensors are flipped vertically and colors are a bit saturated
  if (s->id.PID == OV3660_PID) {
    s->set_vflip(s, 1);//flip it back
    s->set_brightness(s, 1);//up the blightness just a bit
    s->set_saturation(s, -2);//lower the saturation
  }
  //drop down frame size for higher initial frame rate
  s->set_framesize(s, FRAMESIZE_QVGA);

  // Not used in our project
  #if defined(CAMERA_MODEL_M5STACK_WIDE)
    s->set_vflip(s, 1);
    s->set_hmirror(s, 1);
  #endif

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");

  startCameraServer();

  Serial.print("Camera Ready! Use 'http://");
  Serial.print(WiFi.localIP());
  Serial.println("' to connect");

  // Set MQTT Connection
  client.setServer(mqttServer, 1883);
  client.setBufferSize (MAX_PAYLOAD); //This is the maximum payload length
  client.setCallback(callback);
}

void callback(String topic, byte* message, unsigned int length) {
  String messageTemp;
  Serial.println(topic);
  for (int i = 0; i < length; i++) {
    messageTemp += (char)message[i];
  }
  if (topic == topic_PHOTO) {
    if(messageTemp == "on"){
      take_picture();
      //set_flash();
    }
    
  }
  //if (topic == topic_FLASH) {}
}

void take_picture() {
  camera_fb_t * fb = NULL;
  if(flash){ digitalWrite(LED_BUILTIN, HIGH);};
  Serial.println("Taking picture");
  fb = esp_camera_fb_get(); // used to get a single picture.
  if (!fb) {
    Serial.println("Camera capture failed");
    return;
  }
  Serial.println("Picture taken");
  digitalWrite(LED_BUILTIN, LOW);
  sendMQTT(fb->buf, fb->len);
  esp_camera_fb_return(fb); // must be used to free the memory allocated by esp_camera_fb_get().
  
}

void set_flash() {
    flash = !flash;
    Serial.print("Setting flash to ");
    Serial.println (flash);
    if(!flash){
      for (int i=0; i<6; i++){
        digitalWrite(LED_BUILTIN, HIGH);
        delay(100);
        digitalWrite(LED_BUILTIN, LOW);
        delay(100);
      }
    }
    if(flash){
      for (int i=0; i<3; i++){
        digitalWrite(LED_BUILTIN, HIGH);
        delay(500);
        digitalWrite(LED_BUILTIN, LOW);
        delay(100);
      }
    }
}


void sendMQTT(const uint8_t * buf, uint32_t len) {
  Serial.println("Enviando imagem...");
  
  camera_fb_t * fb = NULL;
  fb = esp_camera_fb_get(); // Captura uma nova imagem
  
  // Verifica se a captura da imagem foi bem-sucedida
  if (!fb) {
    Serial.println("Falha na captura da imagem da câmera");
    return;
  }



  String encrypt = base64::encode(fb->buf, fb->len);


  // Realiza a conversão para o formato JPEG
  size_t jpg_len = 0;
  uint8_t *jpg_buf = NULL;
  bool jpeg_conversion_ok = frame2jpg(fb, 80, &jpg_buf, &jpg_len);
  //bool frame2jpg(camera_fb_t *fb, int quality, uint8_t **out_buf, size_t *out_len);



  // Libera a memória da imagem capturada
  esp_camera_fb_return(fb);

  // Verifica se a conversão para JPEG foi bem-sucedida
  if (!jpeg_conversion_ok) {
    Serial.println("Falha na conversão da imagem para JPEG");
    return;
  }

  

  // Envia a imagem em formato JPEG
  if (jpg_len > MAX_PAYLOAD) {
    Serial.println("Imagem muito grande, aumente o valor de MAX_PAYLOAD");
  } else {
    Serial.print("Imagem enviada? : ");
    Serial.print(encrypt);
    
    
    if (client.publish(topic_PUBLISH, encrypt.c_str())) {
      Serial.println(" - publicado com sucesso");
    } else {
      Serial.println(" - falha ao publicar");
    }
    
    /*
    char* img;
    for (int i = 0; i < encrypt.length(); i++) {
    img += (char)encrypt[i];
  }
    client.publish(topic_PUBLISH, img);
  }
  */

  // Libera a memória do buffer JPEG
  if (jpg_buf) {
    free(jpg_buf);
  }
}}



void reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    if (client.connect(HostName, mqttUser, mqttPassword)) {
      Serial.println("connected");
      client.subscribe(topic_PHOTO);
      client.subscribe(topic_FLASH);
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
}