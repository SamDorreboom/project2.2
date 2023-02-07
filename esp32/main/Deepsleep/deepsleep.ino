#define uS_TO_S_FACTOR 1000000
#define TIME_TO_SLEEP 15
RTC_DATA_ATTR int bootCount = 0;

float floatMap(float x, float in_min, float in_max, float out_min, float out_max) {
return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

int previousStiffnessPercent = 0;

// the setup routine runs once when you press reset:
void setup() {
// initialize serial communication at 115200 bits per second:
Serial.begin(115200);
++bootCount;

}

void print_wakeup_reason(){
  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch(wakeup_reason)
  {

    case ESP_SLEEP_WAKEUP_TIMER : Serial.println("Wakeup caused by timer"); break;

    default : Serial.printf("Wakeup was not caused by deep sleep: %d\n",wakeup_reason); break;
  }
}


void loop() {
int analogValue = analogRead(4);
float voltage = floatMap(analogValue, 0, 4095, 0, 3.3);

// Convert the analog value into a percentage of stiffness:
int stiffnessPercent = floatMap(analogValue, 0, 4095, 0, 100);

Serial.print("Analog: ");
Serial.print(analogValue);
Serial.print(", Voltage: ");
Serial.print(voltage);
Serial.print(", Stiffness: ");
Serial.print(stiffnessPercent);
Serial.println("%");


if (abs(stiffnessPercent - previousStiffnessPercent) > 4 || stiffnessPercent > 20) {
previousStiffnessPercent = stiffnessPercent;
Serial.println("Stiffness change greater than 4% or if stiffness greater than 20%, staying awake");
delay(1000);
} 

else {
  Serial.println("Going to deep sleep for 15 sec");
  previousStiffnessPercent = stiffnessPercent;
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
  Serial.flush(); 
  esp_deep_sleep_start();
}
}
