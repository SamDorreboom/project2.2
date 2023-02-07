#include <WiFi.h>
#include <WiFiUdp.h>
#include <cJSON.h>
#include "secret.h"



#include <Adafruit_BME280.h>
#include <Adafruit_Sensor.h>



#define NODE_RED_IP "192.168.137.23" //Ip adres node red dashboard
#define NODE_RED_PORT 49152 // UDP port node red
#define WIFI_TIMEOUT_MS 20000 // 20 second WiFi connection timeout 20 seconden 
#define WIFI_RECOVER_TIME_MS 30000 // Wait 30 seconds after a failed connection attempt


//Vertaald de binnenkomende voltage van de stretch sensor om naar procenten. 
float floatMap(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

//Queue to collect sensor data
QueueHandle_t data_queue;

//start bme280
Adafruit_BME280 bme;

QueueHandle_t audit_queue;

void get_values(void *pvParameters) {

  for (;;) {
    //Get all sensor data and saves it in variable
    float temperature = bme.readTemperature();
    float humidity = bme.readHumidity();
    float pressure = bme.readPressure() / 100;
    int node = 1;
    float analogValue = analogRead(32);
    float stifness = floatMap(analogValue, 0, 4095, 0, 100);

    //Put all sensor data in a json object en transform it in a string.
    cJSON *root = cJSON_CreateObject();
    cJSON_AddNumberToObject(root, "node100", node);
    cJSON_AddNumberToObject(root, "temp100", temperature);
    cJSON_AddNumberToObject(root, "hum100", humidity);
    cJSON_AddNumberToObject(root, "pres100", pressure);
    cJSON_AddNumberToObject(root, "stretch100", stifness);
    char *json_string = cJSON_PrintUnformatted(root);

    //variable for queue item when full
    char queue_item_verwijderen;
    //free spaces in queue
    int free_space = uxQueueSpacesAvailable(audit_queue);

	// Check if queue full is. If gueue is full then remove the oldest item in the queue.
    if (free_space == 0) {
      if (xQueueReceive(audit_queue, (void *) &queue_item_verwijderen, 0) == pdTRUE) {
        xQueueSend(audit_queue, (void *)&json_string, 0);
        }

      } else {
          xQueueSend(audit_queue, (void *)&json_string, 0);
          
  }
  //Wait a second
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  }



}

void WiFi_connect(void* parameter){
  for(;;){
    //Check if wifi is connected and wait for 10 seconds.
    if(WiFi.status() == WL_CONNECTED){
      vTaskDelay(10000 / portTICK_PERIOD_MS);
      continue;
    }
    //Make connection to the internet.
    WiFi.begin(SSID, PWD);

    unsigned long startAttemptTime = millis();
    //If connection failed wait for some seconds.
    while (WiFi.status() != WL_CONNECTED && 
            millis() - startAttemptTime < WIFI_TIMEOUT_MS){}


      if(WiFi.status() != WL_CONNECTED){
          vTaskDelay(WIFI_RECOVER_TIME_MS / portTICK_PERIOD_MS);
			      continue;
      }

  }

}


void udp_json_task(void *pvParameters)
{

    WiFiUDP udp;
    udp.begin(NODE_RED_PORT);
    //Variable for the json string in queue.
    char *json_string;

    while (1) {
      //If there is a active internet connection send the json string over udp to node red.
      if(WiFi.status() == WL_CONNECTED){

        int free_space = uxQueueMessagesWaiting(audit_queue);

        if (free_space > 0) {

          xQueueReceive(audit_queue, (void *) &json_string, 10);

          udp.beginPacket(NODE_RED_IP, NODE_RED_PORT);
          udp.write((const uint8_t *)json_string, strlen(json_string));
          udp.endPacket();
          vTaskDelay(500 / portTICK_PERIOD_MS);
        }

    }


}
  vTaskDelete(NULL);
}

void setup() {
  //Start bme
  bme.begin(0x76);
  int app_cpu = xPortGetCoreID();
  BaseType_t rc;
  audit_queue = xQueueCreate(200, (4 *sizeof(char)));
  delay(2000); // Allow USB to connect
  //Task for wifi connection/check.
  xTaskCreatePinnedToCore(WiFi_connect, "WiFi_connect_task", 5000, NULL, 15, NULL, app_cpu);
  //Task for collect sensor data.
  xTaskCreatePinnedToCore(get_values, "get_values", 8192, NULL, 5, NULL, app_cpu);
  //Task for sending sensor data.
  xTaskCreatePinnedToCore(udp_json_task, "udp_json_task", 8192, NULL, 5, NULL, app_cpu);




}

void loop (){
  vTaskDelete(nullptr);
}
