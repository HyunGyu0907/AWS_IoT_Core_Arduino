#include <Adafruit_Sensor.h> // 통합 센서 라이브러리
#include <DHT.h> // 온습도 센서 사용을 위한 라이브러리
#include <ArduinoBearSSL.h> // Arduino에 대한 BearSSL포트 BearSSL: C로 작성된 SSL/TLS 프로토콜
#include <ArduinoECCX08.h> // Atmel/Microchip ECC508/ECC608 암호화 칩용 라이브러리
#include <ArduinoMqttClient.h> // MQTT 통신 사용을 위한 라이브러리
#include <WiFiNINA.h> // 와이파이 사용을 위한 라이브러리
#include <WiFiUdp.h> // timeServer에서 Unix time을 가져오기 위한 라이브러리
#include <stdio.h>
#include <Arduino.h>
#include <UUID.h> // UUID 생성을 위한 라이브러리
#include <SPI.h>

#define DHTPIN A0 // 온도값 출력핀 A0
#define DHTTYPE DHT11 // 센서 타입 DHT11

#include "arduino_secrets.h"

const char ssid[]        = SECRET_SSID; // WiFi ssid
const char pass[]        = SECRET_PASS; // WiFi 비밀번호
const char broker[]      = SECRET_BROKER; // AWS EndPoint
const char* certificate  = SECRET_CERTIFICATE; // AWS 인증서

IPAddress timeServer(162, 159, 200, 123); // pool.ntp.org NTP server

unsigned int localPort = 2390; // local port to listen for UDP packets
const int NTP_PACKET_SIZE = 48; // NTP timestamp is in the first 48 bytes of the message

byte packetBuffer[NTP_PACKET_SIZE]; // buffer to hold incoming and outgoing packets

DHT dht(DHTPIN, DHTTYPE);
WiFiUDP Udp;
WiFiClient    wclient; // MQTT 클라이언트를 만들기 위한 WiFi 클라이언트
BearSSLClient sslClient(wclient);
MqttClient    mqttClient(sslClient); // Set up MQTT client
UUID uuid;

const char* pub_topic = "/grepfa/v1/sensor/C2A43213-0E0E-8DC0-DC8A-4152685E0932";


void setup() {
  Serial.begin(9600); // 시리얼 통신(9600) 오픈
  dht.begin();

  if(WiFi.status() == WL_NO_MODULE){ // 와이파이 모듈 체크
   Serial.println("Communication with WiFi module failed!"); // 모듈 체크, 오류메시지 출력
  }

  // Set a callback to get the current time
  // used to validate the servers certificate
  ArduinoBearSSL.onGetTime(getTime);

  // Set the ECCX08 slot to use for the private key
  // and the accompanying public certificate for it
  sslClient.setEccSlot(0, certificate);

  // Set the message callback, this function is
  // called when the MQTTClient receives a message
  // mqttClient.onMessage(onMessageReceived);

  Udp.begin(localPort);
}

void loop() {
  uuid.generate();

  if(WiFi.status() != WL_CONNECTED){
    connectWiFi();
  }

  if(!mqttClient.connected()){
    connectMQTT();
  }

  int t = dht.readTemperature(); // DHT11 온습도 센서에서 온도값 읽어오기
  int h = dht.readHumidity(); // DHT11 온습도 센서에서 습도값 읽어오기
  char buf[1024];

  sendNTPpacket(timeServer); // send an NTP packet to a time server
  if(Udp.parsePacket()){
    Udp.read(packetBuffer, NTP_PACKET_SIZE);

    unsigned long highWord = word(packetBuffer[40], packetBuffer[41]);
    unsigned long lowWord = word(packetBuffer[42], packetBuffer[43]);
    unsigned long secsSince1900 = highWord << 16 | lowWord;
    const unsigned long seventyYears = 2208988800UL;
    unsigned long epoch = secsSince1900 - seventyYears;

    sprintf(buf, "{\"timestamp\": \"%d\",\"thing_id\": \"C2A43213-0E0E-8DC0-DC8A-4152685E0932\",\"payload_id\": \"%s\",\"values\": [{\"channel\": \"%d\",\"type\": \"temp\",\"value\": \"%d\"},{\"channel\": \"%d\",\"type\": \"humi\",\"value\": \"%d\"}]}", epoch, uuid.toCharArray(), 0, t, 0, h);
  }

  Serial.println("Publishing message");
  // send message, the print interface can be used to set the message contents
  // mqttClient.setBufferSize(1024);
  mqttClient.beginMessage(pub_topic);
  mqttClient.print(buf); // MQTT publish
  mqttClient.endMessage();
  delay(1000);
}

unsigned long getTime(){
  // get the current time from the WiFi module
  return WiFi.getTime();
}

unsigned long sendNTPpacket(IPAddress& address){
  memset(packetBuffer, 0, NTP_PACKET_SIZE);

  packetBuffer[0] = 0b11100011; // LI, Version, Mode
  packetBuffer[1] = 0; // Stratum, or type of clock
  packetBuffer[2] = 6; // Polling Interval
  packetBuffer[3] = 0xEC; // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12] = 49;
  packetBuffer[13] = 0x4E;
  packetBuffer[14] = 49;
  packetBuffer[15] = 52;

  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp
  Udp.beginPacket(address, 123); // NTP requests are to port 123
  Udp.write(packetBuffer, NTP_PACKET_SIZE);
  Udp.endPacket();
}

void connectWiFi(){
  Serial.print("Attempting to connect to SSID: "); // WiFi 연결시도
  Serial.print(ssid);
  Serial.print(" ");

  while(WiFi.begin(ssid, pass) != WL_CONNECTED){ // WiFi가 연결되지 않았을 때
    // failed, retry
    Serial.print("  try again in 5 seconds");
    delay(5000);
  }
  Serial.println();

  // WiFi가 연결되었을 때
  Serial.println("Yor're connected to the network");
  Serial.println();
}

void connectMQTT(){
  Serial.print("Attempting to MQTT broker: "); // MQTT 연결시도
  Serial.print(broker);
  Serial.println(" ");

  while(!mqttClient.connect(broker, 8883)){ // MQTT가 연결되지 않았을 때
    // failed, retry
    Serial.println("  try again in 5 seconds");
    delay(5000);
  }
  Serial.println();

  // MQTT가 연결되었을 때
  Serial.println("Yor're connected to the MQTT broker");
  Serial.println();

  // subscribe to a topic
  // mqttClient.subscribe("device/sub");
}

/* void onMessageReceived(int messageSize){
  // we received a message, print out the topic and contents
  Serial.print("Received a message with topic");
  Serial.print(mqttClient.messageTopic());
  Serial.print("', length ");
  Serial.print(messqgeSize);
  Serial.println(" bytes:");

  // use the Stream interface to print the contents
  while(mqttClient.available()){
    Serial.print((char)mqttClient.read());
  }
  Serial.println();
  Serial.println();
}
*/