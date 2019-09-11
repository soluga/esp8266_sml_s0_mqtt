// 223244 Strom_Gas_MQTT.ino
/*
Flash real id:   00164068
Flash real size: 4194304 bytes

Flash ide  size: 4194304 bytes
Flash ide speed: 40000000 Hz
Flash ide mode:  DIO
Flash Chip configuration ok.
*/

#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <SoftwareSerial.h> // https://github.com/plerup/espsoftwareserial

#include <pcf8574_esp.h>
#include <Wire.h>

#include <ESP8266httpUpdate.h>

#include <uCRC16BPBLib.h>

#define enable_ir true;
 
static const uint8_t D0   = 16;
static const uint8_t D1   = 5;  // PCF857x SCL
static const uint8_t D2   = 4;  // PCF857x SDA
#ifdef enable_ir
// Live:
static const uint8_t D3   = 0;  // --- IR-Send
static const uint8_t D4   = 2;  // IR-Rcv
#else
// Zum Parken:
static const uint8_t D3   = 12;  // --- IR-Send
static const uint8_t D4   = 13;  // IR-Rcv
#endif
static const uint8_t D5   = 14; // PCF857x Interrupt
static const uint8_t D6   = 12;
static const uint8_t D7   = 13;
static const uint8_t D8   = 15;
static const uint8_t D9   = 3;
static const uint8_t D10  = 1;


const char* SSID = "WifiSSID";
const char* PSK = "WifiPassword";

const char* MQTT_BROKER = "mqtt.yourdomain.com";
#define mqtt_user "MQTTUsername"
#define mqtt_password "MQTTPasword"
unsigned long lastMQTTPublish = 0;
int PublishTimeMQTTSeconds = 1;

unsigned long lastUpdateCheck = 0;
int UpdateCheckMinutes = 1; // 0 deaktiviert automatisches Update
int IncUpdateCheckAfterCheckCount = 1440; // Nach einem Tag (1440 Checks) wird das Intervall erhöht
int UpdateCheckIncMinutes = 15; // ...auf 15 Minuten
int UpdateCheckCount = 0;

const char* SOFTWAREVERSION="19072904";
const char* UPDATESERVERFILE="/update_ota/getupdate.php";
const char* UPDATESERVER="iot-update.yourdomain.com";
const int UPDATESERVERPORT=80;

WiFiClient espClient;
PubSubClient mqttClient(espClient);


bool inMQTTSend = false;

byte inByte; // for reading from serial
//const int smlDataBufferSize = 700;
const int smlDataBufferSize = 1400;
byte smlMessage[smlDataBufferSize]; // for storing the the isolated message. Mine was 280 bytes, but may vary...
int smlMaxDataLimit = smlDataBufferSize-100;
const byte startSequence[] = { 0x1B, 0x1B, 0x1B, 0x1B, 0x01, 0x01, 0x01, 0x01 }; // see sml protocol
const byte stopSequence[]  = { 0x1B, 0x1B, 0x1B, 0x1B, 0x1A };

// 1-0:16.7.0     Gesamtleistung aktuell
const byte GesamtleistungAktuell[] =       { 0x07, 0x01, 0x00, 0x10, 0x07, 0x00, 0xFF, 0x01, 0x01, 0x62, 0x1B, 0x52, 0x00, 0x55 }; //sequence preceeding the current "Wirkleistung" value (4 Bytes)
// 1-0:36.7.0*255 Gesamtleistung L1
const byte GesamtleistungL1[] =            { 0x07, 0x01, 0x00, 0x24, 0x07, 0x00, 0xFF, 0x01, 0x01, 0x62, 0x1B, 0x52, 0x00, 0x55 }; //sequence preceeding the current "Wirkleistung" value (4 Bytes)
// 1-0:56.7.0*255 Gesamtleistung L2
const byte GesamtleistungL2[] =            { 0x07, 0x01, 0x00, 0x38, 0x07, 0x00, 0xFF, 0x01, 0x01, 0x62, 0x1B, 0x52, 0x00, 0x55 }; //sequence preceeding the current "Wirkleistung" value (4 Bytes)
// 1-0:76.7.0*255 Gesamtleistung L3
const byte GesamtleistungL3[] =            { 0x07, 0x01, 0x00, 0x4C, 0x07, 0x00, 0xFF, 0x01, 0x01, 0x62, 0x1B, 0x52, 0x00, 0x55 }; //sequence preceeding the current "Wirkleistung" value (4 Bytes)
// 1-0:1.8.0*255  Zählerstand
//const byte Zaehlerstand[] =                { 0x07, 0x01, 0x00, 0x01, 0x08, 0x00, 0xFF, 0x65, 0x00, 0x00, 0x01, 0x82, 0x01, 0x62, 0x1E, 0x52, 0xFF, 0x59 }; //sequence predeecing the current "Gesamtverbrauch" value (8 Bytes)
const byte Zaehlerstand[] =                { 0x07, 0x01, 0x00, 0x01, 0x08, 0x00, 0xFF, 0x65, 0x00, 0x00, 0x01, 0x82, 0x01, 0x62, 0x1E, 0x52, 0xFF, 0x59 }; //sequence predeecing the current "Gesamtverbrauch" value (8 Bytes)
const byte Zaehlerstand2[] =                { 0x07, 0x01, 0x00, 0x01, 0x08, 0x00, 0xFF, 0x65, 0x00, 0x00, 0x01, 0xb0, 0x01, 0x62, 0x1E, 0x52, 0xFF, 0x59 }; //sequence predeecing the current "Gesamtverbrauch" value (8 Bytes)
//                                                                     08, 0x00, 0xff, 0x65, 0x00, 0x00, 0x01, 0xb0, 0x01, 0x62, 0x1e, 0x52, 0xff, 0x59000000
int smlIndex;     // represents the actual position in smlMessage
int smlStartIndex;

// Init
// Konfiguration IR-Port SML
#define RX_PIN D4 // For Infrared-Rcv
#define TX_PIN D3  // For Infrared-Transmit
SoftwareSerial infraredHead( RX_PIN, TX_PIN, false, 256); // RX, TX, Inverse, Buffer



// Konfiguration S0-Zähler (I2C-GPIO-Interface)
PCF857x pcf8574(0x20, &Wire, true);
#define PIN_INT D5
#define PIN_SDA D2
#define PIN_SCL D1

void SerialPrintln(String AMessage = "")
{
  #ifndef enable_ir
    Serial.println(AMessage);
  #endif
}

void SerialPrint(String AMessage)
{
  #ifndef enable_ir
    Serial.print(AMessage);
  #endif
}

void setup_wifi() {
  delay(10);
  SerialPrintln();
  SerialPrint("Connecting to ");
  SerialPrintln(SSID);

  WiFi.begin(SSID, PSK);

  while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      SerialPrint(".");
  }

  SerialPrintln("");
  SerialPrintln("WiFi connected");
  SerialPrintln("IP address: ");
  SerialPrintln(WiFi.localIP().toString());
}

boolean MQTTcheck()
{
  if (!mqttClient.connected())
  {
    //ESP.wdtDisable();
 
    SerialPrintln("Reconnecting...");
    String strmqtt_clientname = "ESP8266Client_";
    strmqtt_clientname += ESP.getChipId();
    char mqtt_clientname[strmqtt_clientname.length()+1];
    strmqtt_clientname.toCharArray(mqtt_clientname, strmqtt_clientname.length()+1);
  
    if (!mqttClient.connect(mqtt_clientname, mqtt_user, mqtt_password)) {
      SerialPrint("failed, rc=");
      SerialPrint(String(mqttClient.state()));
  //    SerialPrintln(" retrying in 5 seconds");
  //    delay(5000);
    }

    //ESP.wdtEnable(1000);
  }    
}

boolean internalMQTTsend(String ATopic, String AValue, bool ARetained = true) 
{
//  SerialPrintln(AValue);
  char topic[ATopic.length()+1];
  ATopic.toCharArray(topic, ATopic.length()+1);

  if (!MQTTcheck())
    return false;

  if (mqttClient.connected())
    return mqttClient.publish(topic, AValue.c_str(), ARetained);
  else
    return false;
}


boolean MQTTsend(String ATopic, String AValue, bool ARetained = true) 
{
  bool result = false;
  int retryCnt = 0;
  int maxRetryCnt = 5;

  while ((retryCnt >= 0) &&
         (retryCnt < maxRetryCnt))
  {
    retryCnt = retryCnt + 1;

    if (!inMQTTSend) 
    {
      inMQTTSend = true;
      result = internalMQTTsend(ATopic, AValue, ARetained);
      inMQTTSend = false;

      retryCnt = -1;
    } else {
      SerialPrint("w");
      delay(20);
    }
  }
  
  return result;
}

boolean MQTTsend(String ATopic, float AValue, bool ARetained = true) 
{
  return MQTTsend(ATopic, String(AValue), ARetained);
}

boolean MQTTsend(String ATopic, int AValue, bool ARetained = true) 
{
  return MQTTsend(ATopic, String(AValue), ARetained);
}

void reconnect() {
  return;
  
  while (!mqttClient.connected()) {
    SerialPrintln("Reconnecting...");
    String strmqtt_clientname = "ESP8266Client_";
    strmqtt_clientname += ESP.getChipId();
    char mqtt_clientname[strmqtt_clientname.length()+1];
    strmqtt_clientname.toCharArray(mqtt_clientname, strmqtt_clientname.length()+1);
    if (!mqttClient.connect(mqtt_clientname, mqtt_user, mqtt_password)) {
      SerialPrint("failed, rc=");
      SerialPrint(String(mqttClient.state()));
      SerialPrintln(" retrying in 5 seconds");
      delay(5000);
    }
  }
}

bool LastExtGPIOStates[8] = {false, false, false, false, false, false, false, false};
int GPIOCounts[8] = {0, 0, 0, 0, 0, 0, 0, 0};

bool CheckKey(byte key, byte num) { //0, 1, 2, 3
  return (not (key & (1 << num)));
}

void CheckForUpdate() {
    String UpdateFile = UPDATESERVERFILE;
    UpdateFile += "?chipid=";
    UpdateFile += ESP.getChipId();
    UpdateFile += "&currver=";
    UpdateFile += SOFTWAREVERSION;
    UpdateFile += "&sdkver=";
    UpdateFile += ESP.getSdkVersion();
  
    t_httpUpdate_return ret = ESPhttpUpdate.update(UPDATESERVER, UPDATESERVERPORT, UpdateFile, SOFTWAREVERSION);
 
    switch(ret) {
        case HTTP_UPDATE_FAILED:
            SerialPrintln(F("UPDATE FEHLGESCHLAGEN!"));
            SerialPrintln(F(""));
            SerialPrintln(ESPhttpUpdate.getLastErrorString());
            break;
 
        case HTTP_UPDATE_NO_UPDATES:
            SerialPrintln(F("KEIN UPDATE VORHANDEN!"));
            SerialPrintln(F(""));
            break;
 
        case HTTP_UPDATE_OK:
            SerialPrintln(F("UPDATE ERFOLGREICH!"));
            ESP.restart();
            delayMicroseconds(1000);
            break;
    }

}

unsigned long timer;

void findStopSequence() {
  smlStartIndex = -1;
  bool debugSML = false;
  
  SerialPrint("j");
  int maxData = 1000;
  int currData = 0;
  while ((infraredHead.available()) &&
         (currData <= maxData))
  {
    currData++;
//    SerialPrint("k");
    
    inByte = infraredHead.read();
    //SerialPrint("l");
    if (debugSML)
    {
      if ((timer > 0) &&
          (timer+100 < millis()))
        SerialPrintln();
      timer = millis();
      char Hex[3];
      char *myPtr = &Hex[0]; //or just myPtr=charArr; but the former described it better.
      snprintf(myPtr, 3, "%02x", inByte); //convert a byte to character string, and save 2 characters (+null) to charArr;
      SerialPrint(Hex);
    }
    smlMessage[smlIndex] = inByte;
    smlIndex++;

    if (smlIndex > smlMaxDataLimit)
    {
      memset(smlMessage, 0, sizeof(smlMessage)); // clear the buffer
      smlIndex = 0;
      return;
    }
//    SerialPrint("m");

    if (smlIndex > sizeof(stopSequence)+3)
    {
      bool foundStopSeq = true;
      for (int i = 0; i <= sizeof(stopSequence)-1; i++)
      {
        if (smlMessage[smlIndex-3-sizeof(stopSequence)+i] != stopSequence[i])
        {
          foundStopSeq = false;
          break;
        }
      }

//      SerialPrint("n");
      if (foundStopSeq)
      {
        // Ende der SML-Nachricht. Nachprüfen, wo sie angefangen hat:
        for (int i = smlIndex-sizeof(startSequence)-1; i >= 0; i--)
        {
          bool foundStartSeq = true;
          for (int k = 0; k <= sizeof(startSequence)-1; k++)
          {
            if (smlMessage[i+k] != startSequence[k])
            {
              foundStartSeq = false;
              break;
            }
          }
          if (foundStartSeq)
          {
            smlStartIndex = i;
            smlIndex--;
            
            SerialPrint("o");
            if (checkCRC())
            {
              publishMessage();
            } else {
              SerialPrint("C");
              memset(smlMessage, 0, sizeof(smlMessage)); // clear the buffer
              smlIndex = 0;
            }

            SerialPrint("p");
            return;
          }
        }
      }
    }

//    delay(10);
  }
}

bool checkCRC() {
  uCRC16BPBLib crc;
  bool debugCRC = false;

  /*
    Die Startposition der Pürfsummenberechnung ist das erste Byte von 1B 1B 1B 1B 01 01 01 01, 
    die letzte noch in der Prüfsumme enthaltene Position ist 
    (erstes Byte der Sequenz 1B 1B 1B 1B 1A) + 5 Positionen 
    (das ist das Byte nach der Sequenz 1B 1B 1B 1B 1A; dieses ist noch enthalten), 
    direkt danach schließen sich die 2 Byte der so berechneten Prüfsumme an.
  */

  if (debugCRC)
    SerialPrintln("CRC-checking:");

  int arrSize = 2 * (smlIndex-smlStartIndex) + 1;
  char smlMessageAsString[arrSize];
  char *myPtr = &smlMessageAsString[0]; //or just myPtr=charArr; but the former described it better.
  for (int i = smlStartIndex; i <= smlIndex; i++) {
    snprintf(myPtr, 3, "%02x", smlMessage[i]); //convert a byte to character string, and save 2 characters (+null) to charArr;
    myPtr += 2; //increment the pointer by two characters in charArr so that next time the null from the previous go is overwritten.
  }
  if (debugCRC)
  {
    SerialPrint("Message: ");
    SerialPrintln(smlMessageAsString);
  }

  // CRC-Start und -Ende finden:
  int CRCStart = -1;
  int CRCEnde = -1;  
  for (int i = smlStartIndex; i <= smlIndex; i++) {
    if ((CRCStart == -1) &&
        (i < smlIndex - sizeof(startSequence) - 1 ))
    {
      bool matches = true;
      for (int j = 0; j < sizeof(startSequence); j++)
      {
        if (smlMessage[i+j] != startSequence[j])
        {
          matches = false;
          break;        
        }
      }

      if (matches)
        CRCStart = i;
    }
    if ((CRCEnde == -1) &&
        (i < smlIndex - sizeof(stopSequence) - 1 ))
    {
      bool matches = true;
      for (int j = 0; j < sizeof(stopSequence); j++)
      {
        if (smlMessage[i+j] != stopSequence[j])
        {
          matches = false;
          break;        
        }
      }

      if (matches)
        CRCEnde = i + sizeof(stopSequence); // Füllbyte-Länge gehört auch zum CRC!
    }
  }

  int CRCarrSize = 2 * (CRCEnde - CRCStart) + 1;
  char CRCsmlMessageAsString[CRCarrSize];
  char *CRCmyPtr = &CRCsmlMessageAsString[0]; //or just myPtr=charArr; but the former described it better.
  for (int i = CRCStart; i <= CRCEnde; i++) {
    snprintf(CRCmyPtr, 3, "%02x", smlMessage[i]); //convert a byte to character string, and save 2 characters (+null) to charArr;
    CRCmyPtr += 2; //increment the pointer by two characters in charArr so that next time the null from the previous go is overwritten.
  }
  if (debugCRC)
  {
    SerialPrint("CRC-Message: ");
    SerialPrintln(CRCsmlMessageAsString);
  }

  byte CRCReceived[2];
  if (CRCEnde >= 0)
  {
    // Empfangener CRC wird hier gedreht:
    CRCReceived[0] = smlMessage[CRCEnde+2];
    CRCReceived[1] = smlMessage[CRCEnde+1];
  }
  else
  {
    CRCReceived[0] = 0x00;
    CRCReceived[1] = 0x00;
  }

  char CRCrcvAsString[5];
  char *CRCrcvmyPtr = &CRCrcvAsString[0]; //or just myPtr=charArr; but the former described it better.
  for (int i = 0; i <= 1; i++) {
    snprintf(CRCrcvmyPtr, 3, "%02x", CRCReceived[i]); //convert a byte to character string, and save 2 characters (+null) to charArr;
    CRCrcvmyPtr += 2; //increment the pointer by two characters in charArr so that next time the null from the previous go is overwritten.
  }
  if (debugCRC)
  {
    SerialPrint("CRC-Received: ");
    SerialPrintln(CRCrcvAsString);
  }
/*
  char CRCrcvAsString[5];
  char *CRCrcvmyPtr = &CRCrcvAsString[0]; //or just myPtr=charArr; but the former described it better.
  for (int i = CRCEnde+1; i <= CRCEnde+2; i++) {
    snprintf(CRCrcvmyPtr, 3, "%02x", smlMessage[i]); //convert a byte to character string, and save 2 characters (+null) to charArr;
    CRCrcvmyPtr += 2; //increment the pointer by two characters in charArr so that next time the null from the previous go is overwritten.
  }
  if (debugCRC)
  {
    SerialPrint("CRC-Received: ");
    SerialPrintln(CRCrcvAsString);
  }
*/
  
  crc.reset(); // : Resets internal state
  //uCRC16BPBLibObject->feedBit(bool) : Feeds a bit
  //uCRC16BPBLibObject->feedByte(char) : Feeds a byte
  for (int i = CRCStart; i <= CRCEnde; i++) {
    char c = char(smlMessage[i]);
    crc.feedByte(c);
//    SerialPrint(c);
  }
  
  uint16_t CRCcalcInt16 = crc.getResult();
  if (debugCRC)
    SerialPrintln((String)CRCcalcInt16);
  byte CRCcalc[2];
  CRCcalc[0] = CRCcalcInt16 & 0xff;
  CRCcalc[1] = (CRCcalcInt16 >> 8);  

  char CRCcalcAsString[5];
  char *CRCcalcmyPtr = &CRCcalcAsString[0]; //or just myPtr=charArr; but the former described it better.
  for (int i = 0; i <= 1; i++) {
    snprintf(CRCcalcmyPtr, 3, "%02x", CRCcalc[i]); //convert a byte to character string, and save 2 characters (+null) to charArr;
    CRCcalcmyPtr += 2; //increment the pointer by two characters in charArr so that next time the null from the previous go is overwritten.
  }
  if (debugCRC)
  {
    SerialPrint("CRC-Calc: ");
    SerialPrintln(CRCcalcAsString);
  }

  bool CRC_OK = false;
  if ((CRCcalc[0] == CRCReceived[0]) &&
      (CRCcalc[1] == CRCReceived[1]))
    CRC_OK = true;

  if (debugCRC)
  {
    if (CRC_OK)
      SerialPrintln("CRC: Success");
    else
      SerialPrintln("CRC: Failed");
  }

  return CRC_OK;
}

void publishMessage() {
  int arrSize = 2 * (smlIndex-smlStartIndex) + 1;
  char smlMessageAsString[arrSize];
  char *myPtr = &smlMessageAsString[0]; //or just myPtr=charArr; but the former described it better.

  for (int i = smlStartIndex; i <= smlIndex; i++) {
    snprintf(myPtr, 3, "%02x", smlMessage[i]); //convert a byte to character string, and save 2 characters (+null) to charArr;
    myPtr += 2; //increment the pointer by two characters in charArr so that next time the null from the previous go is overwritten.
  }

  //SerialPrintln(smlMessageAsString); // for debuging
  //parseSMLMessage(smlMessageAsString);
  int resGesamtleistungAktuell = findSequence_4bytes(GesamtleistungAktuell, sizeof(GesamtleistungAktuell));
  int resGesamtleistungL1 = findSequence_4bytes(GesamtleistungL1, sizeof(GesamtleistungL1));
  int resGesamtleistungL2 = findSequence_4bytes(GesamtleistungL2, sizeof(GesamtleistungL2));
  int resGesamtleistungL3 = findSequence_4bytes(GesamtleistungL3, sizeof(GesamtleistungL3));
  float resZaehlerstand = (float)findSequence_8bytes(Zaehlerstand, sizeof(Zaehlerstand))/10000;
  if (resZaehlerstand == 0)
    resZaehlerstand = (float)findSequence_8bytes(Zaehlerstand2, sizeof(Zaehlerstand2))/10000;
  feedWDT();

  SerialPrint(".");
/*  
  SerialPrintln("");
  SerialPrint("Gesamt: ");
  SerialPrintln(resGesamtleistungAktuell);
  SerialPrint("L1: ");
  SerialPrintln(resGesamtleistungL1);
  SerialPrint("L2: ");
  SerialPrintln(resGesamtleistungL2);
  SerialPrint("L3: ");
  SerialPrintln(resGesamtleistungL3);
  SerialPrint("Zählerstand: ");
  SerialPrintln(resZaehlerstand);
*/

  if (resGesamtleistungAktuell != 0)
    MQTTsend("home/zaehler/strom/gesamt_saldiert", resGesamtleistungAktuell);
  if (resGesamtleistungL1 != 0)
    MQTTsend("home/zaehler/strom/l1_saldiert", resGesamtleistungL1);
  if (resGesamtleistungL2 != 0)
    MQTTsend("home/zaehler/strom/l2_saldiert", resGesamtleistungL2);
  if (resGesamtleistungL3 != 0)
    MQTTsend("home/zaehler/strom/l3_saldiert", resGesamtleistungL3);
  if (resZaehlerstand != 0)
    MQTTsend("home/zaehler/strom/zaehlerstand", resZaehlerstand);

  feedWDT();

  memset(smlMessage, 0, sizeof(smlMessage)); // clear the buffer
  smlIndex = 0;
}

int findSequence_4bytes(const byte *ASeq, int ASeqLen) {
  int result = 0;
  byte temp; //temp variable to store loop search data
  byte power[4]; //array that holds the extracted 4 byte "Wirkleistung" value
  int searchIndex = 0;
 
  for(int x = smlStartIndex; x < sizeof(smlMessage); x++){ //for as long there are element in the exctracted SML message 
    temp = smlMessage[x]; //set temp variable to 0,1,2 element in extracted SML message
    if (temp == ASeq[searchIndex]) //compare with power sequence
    {
      searchIndex++;
      if (searchIndex == ASeqLen) //sizeof(ASeq)) //in complete sequence is found
      {
        for(int y = 0; y< 4; y++){ //read the next 4 bytes (the actual power value)
          power[y] = smlMessage[x+y+1]; //store into power array
        }
        result = (power[0] << 24 | power[1] << 16 | power[2] << 8 | power[3]); //merge 4 bytes into single variable to calculate power value
        
        searchIndex = 0;
      }
    }
    else {
      searchIndex = 0;
    }
  }

  return result;
}

int findSequence_8bytes(const byte* ASeq, int ASeqLen) {
  int result = 0;
  byte temp;
  byte consumption[8]; //array that holds the extracted 8 byte "Gesamtverbrauch" value
  int searchIndex = 0;
  
  for(int x = smlStartIndex; x < sizeof(smlMessage); x++){
    temp = smlMessage[x];
    if (temp == ASeq[searchIndex])
    {
      searchIndex++;
      if (searchIndex == ASeqLen)
      {
        for(int y = 0; y< 8; y++){
          //hier muss für die folgenden 8 Bytes hoch gezählt werden
          consumption[y] = smlMessage[x+y+1];
        }
        result = (consumption[0] << 56 | consumption[1] << 48 | consumption[2] << 40 | consumption[32] | consumption[4] << 24 | consumption[5] << 16 | consumption[6] << 8 | consumption[7]); //combine and turn 8 bytes into one variable
        searchIndex = 0;
      }
    }
    else {
      searchIndex = 0;
    }
  }

   return result;
}

bool PCFInterruptFlag = false;

void ICACHE_RAM_ATTR PCFInterrupt() {
  PCFInterruptFlag = true;
  readPCF();
  PCFInterruptFlag = false;
}

bool processingInterrupt = false;

void readPCF() {
//SerialPrint("I");
  if (processingInterrupt)
    return;
  
  processingInterrupt = true;

//SerialPrint(">");
  byte b = pcf8574.read8();
//SerialPrint("<");
  byte keys = ((~b) >> 0) & 0xFF;

  for (int i = 0; i <= 7; i++) {
    bool isActive = CheckKey(keys, i);
    if (LastExtGPIOStates[i] != isActive) {
      LastExtGPIOStates[i] = isActive;
      if (isActive) {
        GPIOCounts[i] = GPIOCounts[i] + 1;
        SerialPrintln("");
        SerialPrint("o");
        SerialPrint(String(i));
        SerialPrintln("");
      }
    }
  }

//  delay(1000);

  processingInterrupt = false;
}

void setup() {
  #ifdef enable_ir
    //********** CHANGE PIN FUNCTION  TO GPIO **********
    //GPIO 1 (TX) swap the pin to a GPIO.
    pinMode(1, FUNCTION_3); 
    //GPIO 3 (RX) swap the pin to a GPIO.
    pinMode(3, FUNCTION_3); 
    //**************************************************
  #else
    Serial.begin(115200);
  #endif
  
  
  SerialPrint("Version: ");
  SerialPrintln(SOFTWAREVERSION);

  setup_wifi();
  CheckForUpdate(); 
  
  mqttClient.setServer(MQTT_BROKER, 1883);

  Wire.pins(PIN_SDA, PIN_SCL);//SDA - D1, SCL - D2
  Wire.begin();

  //Specsheets say PCF8574 is officially rated only for 100KHz I2C-bus
  //PCF8575 is rated for 400KHz
  Wire.setClock(100000L);
  pcf8574.begin( 0x00 ); //8 pin input
  
  pinMode(PIN_INT, INPUT_PULLUP);
  pcf8574.resetInterruptPin();
  attachInterrupt(digitalPinToInterrupt(PIN_INT), PCFInterrupt, FALLING);
  infraredHead.begin(9600);
}

void feedWDT()
{
  delay(0);
//  yield();
//  ESP.wdtFeed();
}

void loop() {
  int i;

  SerialPrint("a");

  if (PCFInterruptFlag)
  {
    PCFInterruptFlag = false;
    readPCF();
  }

  SerialPrint("b");
  
  feedWDT();
//  SerialPrintln("1");
/*  
  if ((UpdateCheckMinutes > 0) and
      (lastUpdateCheck+UpdateCheckMinutes*60*1000 < millis())) {
    CheckForUpdate();
    lastUpdateCheck = millis();

    UpdateCheckCount = UpdateCheckCount + 1;
    if ((UpdateCheckMinutes != UpdateCheckIncMinutes) &&
        (UpdateCheckCount > IncUpdateCheckAfterCheckCount))
      UpdateCheckMinutes = UpdateCheckIncMinutes;
  }
*/
//  SerialPrintln("2");
  
  feedWDT();
  SerialPrint("c");
/*  
  if (!mqttClient.connected()) {
      reconnect();
  }
*/
//  SerialPrintln("3");
  
  feedWDT();
  SerialPrint("d");
  if (lastMQTTPublish+PublishTimeMQTTSeconds*1000 < millis()) {
    for (i = 0; i <= 7; i++) {
      if (GPIOCounts[i] > 0) {
        int LogCount = GPIOCounts[i]; // Zwischenspeichern, falls zwischenzeitlich noch gezählt wird

//        SerialPrint(i);

        String strTopic = "";
//        String strTopic = "home/strom/zaehler/";
//        strTopic += String(i);

        if (i == 0)
          strTopic = "home/zaehler/strom/trockner";
        else if (i == 1)
          strTopic = "home/zaehler/strom/waschmaschine";
        else if (i == 2)
          strTopic = "home/zaehler/strom/abstellraum";
        else if (i == 3)
          strTopic = "home/zaehler/strom/kueche";
        else if (i == 4)
          strTopic = "home/zaehler/strom/spuelmaschine";
        else if (i == 5)
          strTopic = "home/zaehler/strom/edv";
        else if (i == 6)
          strTopic = "home/zaehler/strom/eltern";
        else if (i == 7)
          strTopic = "home/zaehler/gas";

        feedWDT();

        if ((strTopic != "") &&
            (MQTTsend(strTopic, LogCount)))
          GPIOCounts[i] = GPIOCounts[i] - LogCount;
        feedWDT();
      }
    }
//  SerialPrintln("4");

    SerialPrint("e");
    lastMQTTPublish = millis();
  }

  SerialPrint("f");
  feedWDT();
  mqttClient.loop(); // Process incoming MQTT and *maintain connection*

  SerialPrint("g");
//  SerialPrintln("5");

  feedWDT();
/*  if (PCFInterruptFlag) 
  {
    ESP.wdtDisable();
    
    for (i = 0; i <= 7; i++) {
      bool isActive = (pcf8574.read(i)==1);

      if (LastExtGPIOStates[i] != isActive) {
        if (isActive)
          MQTTsend("home/zaehler", "active");
        else
          MQTTsend("home/zaehler", "inactive");
          MQTTsend("home/zaehler", i);
        SerialPrint("PCF8574 Input ");
        SerialPrint(i);
        SerialPrint(" is ");
        if (isActive)
          SerialPrintln("HIGH");
        else
          SerialPrintln("LOW");
        LastExtGPIOStates[i] = isActive;
        if (isActive) {
//          if (GPIOCounts[i] <= 1000)
            GPIOCounts[i] = GPIOCounts[i] + 1;
          MQTTsend("home/zaehler", GPIOCounts[i]);
        }
      }
    }

/*
    SerialPrint("o");
    byte b = pcf8574.read8();
    SerialPrint("x");
    SerialPrintln("");
    SerialPrintln( "INT: " + String(b));

    byte keys = ((~b) >> 0) & 0xFF;

    for (i = 0; i <= 7; i = i + 1) {
      bool isActive = CheckKey(keys, i);
      if (LastExtGPIOStates[i] != isActive) {
        LastExtGPIOStates[i] = isActive;
        if (isActive) {
          GPIOCounts[i] = GPIOCounts[i] + 1;
        }
      }
    }
    PCFInterruptFlag = false;

    ESP.wdtEnable(2000);
  }
*/  
  feedWDT();
  SerialPrint("h");

  findStopSequence();
  SerialPrint("i");
  
  feedWDT();
  delay(10);
  SerialPrintln("p");
}
