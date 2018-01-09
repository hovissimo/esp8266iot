#include <ArduinoJson.h>
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#include <PubSubClient.h>

#define WIFI_SSID "Misadventure"
#define WIFI_PASSWORD "maleficent"

#define MQTT_HOST "test.mosquitto.org"
#define MQTT_PORT 1883
#define MQTT_USER ""
#define MQTT_PASSWORD ""
#define MQTT_CLIENT_ID "esp8266"
#define MQTT_WILL_TOPIC "hovis/esp8266/status"
#define MQTT_WILL_PAYLOAD "offline"
#define MQTT_WILL_QOS 1
#define MQTT_WILL_RETAIN true

// These pin numbers align to GPIO pin numbering, not ESP-12 numbering
#define RELAY 15     // ESP-12: D8
#define LED 2        // ESP-12: D4
#define REDBUTTON 14 // ESP-12: D5
#define REDLED 12    // ESP-12: D6

struct State {
  bool redled;
  bool relay;
  bool redbutton;
};

#define JSON_STATE_SIZE (JSON_OBJECT_SIZE(3))

void serialize_state(const State& state, char* json_out, size_t MAXSIZE) {
  StaticJsonBuffer<JSON_STATE_SIZE> buffer;
  JsonObject& root = buffer.createObject();
  root["redled"] = state.redled;
  root["relay"] = state.relay;
  root["redbutton"] = state.redbutton;
  root.printTo(json_out, MAXSIZE);
}

ESP8266WebServer server(80);

// adapted from:
// https://home-assistant.io/blog/2015/10/11/measure-temperature-with-esp8266-and-report-to-mqtt/
WiFiClient espClient;
PubSubClient client(espClient);

long last_message = 0;
char msg[50];
State state;
int value = 0;
int retries_left = 5;
bool redbutton_triggered = false;

void handleNotFound(){
  digitalWrite(LED, 0);
  String message = "File Not Found\n\n";
  message += "URI: ";
  message += server.uri();
  message += "\nMethod: ";
  message += (server.method() == HTTP_GET)?"GET":"POST";
  message += "\nArguments: ";
  message += server.args();
  message += "\n";
  for (uint8_t i=0; i<server.args(); i++){
    message += " " + server.argName(i) + ": " + server.arg(i) + "\n";
  }
  server.send(404, "text/plain", message);
  delay(10);
  digitalWrite(LED, 1);
}

void handle_root() {
  digitalWrite(LED, 0);
  const int json_size = 200;
  char json_out[json_size];
  serialize_state(state, json_out, json_size);

  server.send(200, "text/plain", json_out);
  delay(10); // to see the LED
  digitalWrite(LED, 1);
}

void setup_http() {
  server.on("/", handle_root);
  server.onNotFound(handleNotFound);
  //server.on("/inline", [](){
  //  server.send(200, "text/plain", "this works as well");
  //});

  server.begin();
  Serial.println("HTTP server started");
}

String get_wifi_status() {
  switch (WiFi.status()) {
    case WL_IDLE_STATUS:
    return String("Wifi is changing modes.");
    break;

    case WL_NO_SSID_AVAIL:
    return String("SSID: '") + WIFI_SSID + " is not available.";
    break;

    case WL_CONNECTED:
    return String("Connected to ") + WIFI_SSID;
    break;

    case WL_CONNECT_FAILED:
    return String("Could not connect to ") + WIFI_SSID;
    break;

    case WL_DISCONNECTED:
    return String("Wifi not in station mode.");
    break;

    default:
    return String("Wifi status error");
  }
}

void setup_network() {
  digitalWrite(LED, 0); // led on indicates waiting for network setup
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.println("");

  // Wait for connection
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print("Connecting to ");
    Serial.print(WIFI_SSID);
    Serial.println("...");
    delay(1000);
  }
  Serial.println(get_wifi_status());
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  if (MDNS.begin("esp8266")) {
    Serial.println("MDNS responder started");
  }
  digitalWrite(LED, 1); // off
}

void setup_mqtt() {
  client.setServer(MQTT_HOST, 1883);
  client.setCallback(handle_mqtt_message);
}

void handle_mqtt_message(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  char message[50];
  ByteToChar(payload, message, length);
  Serial.println(message);

  if (!strcmp(topic, "hovis/redled")) {
    handle_redled(message, length);
  }

  if (!strcmp(topic, "hovis/relay")) {
    handle_relay(message, length);
  }
}

void handle_redled(char* msg, unsigned int length) {
  if (!strcmp(msg, "on")) {
    digitalWrite(REDLED, 1);
    state.redled = true;
  }
  if (!strcmp(msg, "off")) {
    digitalWrite(REDLED, 0);
    state.redled = false;
  }
}

void handle_relay(char* msg, unsigned int length) {
  Serial.println("handling relay");
  if (!strcmp(msg, "on")) {
    digitalWrite(RELAY, 0); // Relay module I'm using is active low
    state.relay = true;
  }
  if (!strcmp(msg, "off")) {
    digitalWrite(RELAY, 1);
    state.relay = false;
  }
}

void ByteToChar(byte* bytes, char* chars, unsigned int count){
  for(unsigned int i = 0; i < count; i++)
    chars[i] = (char)bytes[i];
  chars[count] = (char)0;
}

void reconnect_mqtt() {
  while (!client.connected() && retries_left-- > 0) {
    Serial.println("Attempting MQTT connection...");
    if (client.connect(
          MQTT_CLIENT_ID,
          MQTT_WILL_TOPIC,
          MQTT_WILL_QOS,
          MQTT_WILL_RETAIN,
          MQTT_WILL_PAYLOAD
          )) {
      Serial.println("MQTT Connected.");
      client.publish("hovis/esp8266/status", "online", 1);
      client.subscribe("hovis/#");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

void redbutton_isr() {
  // REDBUTTON is an inverted signal (hardware debounce)
  state.redbutton = !digitalRead(REDBUTTON);
  redbutton_triggered = true;
}

void setup(void) {
  state.redled = false;
  state.relay = false;
  state.redbutton = false;

  pinMode(LED, OUTPUT);
  pinMode(REDLED, OUTPUT);
  pinMode(RELAY, OUTPUT);
  digitalWrite(REDLED, LOW);
  digitalWrite(RELAY, LOW);
  attachInterrupt(digitalPinToInterrupt(REDBUTTON), redbutton_isr, CHANGE);
  Serial.begin(115200);

  setup_network();
  setup_http();
  setup_mqtt();
}

void loop(void){
  if (!client.connected()) { reconnect_mqtt(); }
  client.loop();
  server.handleClient();

  if (redbutton_triggered) {
    redbutton_triggered = false;
    client.publish("hovis/esp8266/redbutton", (state.redbutton ? "pressed" : "unpressed"));
  }
}
