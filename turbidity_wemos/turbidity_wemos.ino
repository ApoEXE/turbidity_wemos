#include <EEPROM.h>  // to store last calibration value (blanco, Vclear)
#include <Wire.h>    // for LCD display (with I2S interphase)
#include <ESP8266WiFi.h>
#include <PubSubClient.h>

const char* ssid = "TP-JAVI";
const char* password = "xavier1234";
const char* mqtt_server = "192.168.0.250";
const char* TOPIC = "Tanque1/canal/turbidity/sensor1";
WiFiClient espClient;
PubSubClient client(espClient);
//messages
#define MSG_BUFFER_SIZE (50)
char msg[MSG_BUFFER_SIZE];


int sensor = 0;  // variable for averaging
int n = 100;     // number of samples to average
int sensorValue = 0;
float voltage = 0.00;
float turbidity = 0.00;
float Vclear = 2.85;  // Output voltage to calibrate (with clear water).
float ntu;

int pushcalib = 1;  // variable for pin D2 status.
bool once = true;   // only calibrate once on
void reconnect();
void setup_wifi();
void callback(char* topic, byte* payload, unsigned int length);

void setup() {
  pinMode(BUILTIN_LED, OUTPUT);  // Initialize the BUILTIN_LED pin as an output
  pinMode(A0, INPUT);
  Serial.begin(115200);
  //Serial display
  Serial.println("turbidity sensor");
  //EEPROM.begin(512);
  EEPROM.get(0, Vclear);  // recovers the last Vclear calibration value stored in ROM.

  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);

  //delay(3000);  // Pause for 3 seg
}

void loop() {
  EEPROM.get(0, Vclear);  // recovers the last Vclear calibration value stored in ROM.
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  if (pushcalib == HIGH) {
    // If the push button is not pushed, do the normal sensing routine:
    for (int i = 0; i < n; i++) {
      sensor += analogRead(A0);  // read the input on analog pin 1 (turbidity sensor analog output)
      delay(10);
    }
    sensorValue = sensor / n;                    // average the n values
    voltage = sensorValue * (5.000 / 1023.000);  // Convert analog (0-1023) to voltage (0 - 5V)
    if (voltage < 2.5) {
      ntu = 3000;
    } else {
      ntu = -1120.4 * sqrt(voltage) + 5742.3 * voltage - 4352.9;
    }

    turbidity = 100.00 - (voltage / Vclear) * 100.00;  // as relative percentage; 0% = clear water;



    // Serial display
    Serial.print(sensorValue);
    Serial.print(", ");
    Serial.print(voltage, 3);
    Serial.print(", ");
    Serial.print(ntu, 3);
    Serial.print(", ");
    Serial.println(Vclear, 3);
    snprintf(msg, MSG_BUFFER_SIZE, "%.2f", ntu);
    client.publish(TOPIC, msg);
    sensor = 0;  // resets for averaging

  } else {
    Serial.println("Calibrating");
    for (int i = 0; i < n; i++) {
      sensor += analogRead(A0);  // read the input on analog pin 1:
      delay(10);
    }
    sensorValue = sensor / n;
    Vclear = sensorValue * (5.000 / 1023.000);  // Converts analog (0-1023) to voltage (0-5V):
    sensor = 0;
    EEPROM.put(0, Vclear);  // stores Vclear in ROM
    EEPROM.end();
    pushcalib = 1;
  }

  delay(500);  // Pause for 1 seconds. // sampling rate
}


void setup_wifi() {

  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  randomSeed(micros());

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void callback(char* topic, byte* payload, unsigned int length) {

  if (strcmp(TOPIC, topic) == 0) {

    if ((char)payload[0] == '0' && once) {
      Serial.println("cmd received calibration");
      digitalWrite(BUILTIN_LED, !digitalRead(BUILTIN_LED));
      pushcalib = 0;
      once = false;
    }
  }
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "ESP8266Client-";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect(clientId.c_str())) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      client.publish(TOPIC, "Turbidity Sensor");
      // ... and resubscribe
      client.subscribe(TOPIC);
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}
