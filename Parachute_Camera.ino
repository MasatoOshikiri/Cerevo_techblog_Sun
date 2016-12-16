#include <arduino.h>
#include <SPI.h>
#include <SD.h>
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <ESP.h>

#define PIC_PKT_LEN    128        //data length of each read, dont set this too big because ram is limited
#define PIC_FMT_VGA    7
#define PIC_FMT_CIF    5
#define PIC_FMT_OCIF   3
#define CAM_ADDR       0

#define PIC_FMT        PIC_FMT_VGA

File myFile;

const char ssid[] = "ParaCam";
const char pass[] = "test12345";
WiFiUDP udp;
unsigned int localPort = 10000;
const int OSC_PACKET_SIZE = 256;
char packetBuffer[OSC_PACKET_SIZE];

const byte cameraAddr = (CAM_ADDR << 5);  // addr
const int buttonPin = 4;                 // the number of the pushbutton pin
unsigned long picTotalLen = 0;            // pictre length
int picNameNum = 0;

/*********************************************************************/
void setup()
{
  ESP.wdtFeed();
  Serial.begin(115200);
  Serial1.begin(115200);
  WiFi.softAP(ssid, pass);
  IPAddress myIP = WiFi.softAPIP();
  Serial1.print("AP IP address: ");  Serial1.println(myIP);

  Serial1.println("Starting UDP");
  udp.begin(localPort);
  Serial1.print("Local port: ");  Serial1.println(udp.localPort());
  
  pinMode(buttonPin, INPUT);    // initialize the pushbutton pin as an input
  Serial1.println("Initializing SD card....");
  pinMode(15,OUTPUT);          // CS pin of SD Card Shield
  
  if (!SD.begin(15)) {
    Serial1.print("sd init failed");
    return;
  }
  Serial1.println("sd init done.");
  initialize();
}
/*********************************************************************/
void loop()
{
  int n = 0;
  int rlen = 0;
  while(1){
    ESP.wdtFeed();
    
    Serial1.println("\r\nPress the button to take a picture");
    while (!(rlen = udp.parsePacket())){
       ESP.wdtFeed();
    }//wait for UDP Command
    Serial1.println("taking");
    if(rlen){
      udp.read(packetBuffer, (rlen > OSC_PACKET_SIZE) ? OSC_PACKET_SIZE : rlen);
      if (strncmp(&packetBuffer[0], "/osc/", 5) == 0) {
        Serial1.println(packetBuffer);
        delay(200);
        if (n == 0) preCapture();
        Capture();
        Serial1.print("Saving picture...");
        GetData();
      }
      Serial1.print("\r\nDone ,number : ");
      Serial1.println(n);
      n++ ;
    }
  }
}
/*********************************************************************/
void clearRxBuf()
{
  while (Serial.available()) 
  {
    Serial.read(); 
  }
}
/*********************************************************************/
void sendCmd(char cmd[], int cmd_len)
{
  for (char i = 0; i < cmd_len; i++) Serial.write(cmd[i]); 
}
/*********************************************************************/
void initialize()
{   
  char cmd[] = {0xaa,0x0d|cameraAddr,0x00,0x00,0x00,0x00} ;  
  unsigned char resp[6];

  Serial1.print("initializing camera...");
  
  while (1) 
  {
    ESP.wdtFeed();
    sendCmd(cmd,6);
    if (Serial.readBytes((char *)resp, 6) != 6)
    {
      Serial1.print(".");
      continue;
    }
    if (resp[0] == 0xaa && resp[1] == (0x0e | cameraAddr) && resp[2] == 0x0d && resp[4] == 0 && resp[5] == 0) 
    {
      if (Serial.readBytes((char *)resp, 6) != 6) continue; 
      if (resp[0] == 0xaa && resp[1] == (0x0d | cameraAddr) && resp[2] == 0 && resp[3] == 0 && resp[4] == 0 && resp[5] == 0) break; 
    }
  }  
  cmd[1] = 0x0e | cameraAddr;
  cmd[2] = 0x0d;
  sendCmd(cmd, 6); 
  Serial1.println("\nCamera initialization done.");
}
/*********************************************************************/
void preCapture()
{
  char cmd[] = { 0xaa, 0x01 | cameraAddr, 0x00, 0x07, 0x00, PIC_FMT };  
  unsigned char resp[6]; 
  
  while (1)
  {
    ESP.wdtFeed();
    clearRxBuf();
    sendCmd(cmd, 6);
    if (Serial.readBytes((char *)resp, 6) != 6) continue; 
    if (resp[0] == 0xaa && resp[1] == (0x0e | cameraAddr) && resp[2] == 0x01 && resp[4] == 0 && resp[5] == 0) break; 
  }
}
void Capture()
{
  char cmd[] = { 0xaa, 0x06 | cameraAddr, 0x08, PIC_PKT_LEN & 0xff, (PIC_PKT_LEN>>8) & 0xff ,0}; 
  unsigned char resp[6];

  while (1)
  {
    ESP.wdtFeed();
    clearRxBuf();
    sendCmd(cmd, 6);
    if (Serial.readBytes((char *)resp, 6) != 6) continue;
    if (resp[0] == 0xaa && resp[1] == (0x0e | cameraAddr) && resp[2] == 0x06 && resp[4] == 0 && resp[5] == 0) break; 
  }
  cmd[1] = 0x05 | cameraAddr;
  cmd[2] = 0;
  cmd[3] = 0;
  cmd[4] = 0;
  cmd[5] = 0; 
  while (1)
  {
    ESP.wdtFeed();
    clearRxBuf();
    sendCmd(cmd, 6);
    if (Serial.readBytes((char *)resp, 6) != 6) continue;
    if (resp[0] == 0xaa && resp[1] == (0x0e | cameraAddr) && resp[2] == 0x05 && resp[4] == 0 && resp[5] == 0) break;
  }
  cmd[1] = 0x04 | cameraAddr;
  cmd[2] = 0x1;
  while (1) 
  {
    ESP.wdtFeed();
    clearRxBuf();
    sendCmd(cmd, 6);
    if (Serial.readBytes((char *)resp, 6) != 6) continue;
    if (resp[0] == 0xaa && resp[1] == (0x0e | cameraAddr) && resp[2] == 0x04 && resp[4] == 0 && resp[5] == 0)
    {
      if (Serial.readBytes((char *)resp, 6) != 6)
      {
        continue;
      }
      if (resp[0] == 0xaa && resp[1] == (0x0a | cameraAddr) && resp[2] == 0x01)
      {
        picTotalLen = (resp[3]) | (resp[4] << 8) | (resp[5] << 16); 
        Serial1.print("picTotalLen:");
        Serial1.println(picTotalLen);
        break;
      }
    }
  }
  
}
/*********************************************************************/
void GetData()
{
  unsigned int pktCnt = (picTotalLen) / (PIC_PKT_LEN - 6); 
  if ((picTotalLen % (PIC_PKT_LEN-6)) != 0) pktCnt += 1;
  
  char cmd[] = { 0xaa, 0x0e | cameraAddr, 0x00, 0x00, 0x00, 0x00 };  
  unsigned char pkt[PIC_PKT_LEN];
  
  char picName[] = "pic00.jpg";
  picName[3] = picNameNum/10 + '0';
  picName[4] = picNameNum%10 + '0';
  
  if (SD.exists(picName))
  {
    SD.remove(picName);
  }
  
  myFile = SD.open(picName, FILE_WRITE); 
  if(!myFile){
    Serial1.println("myFile open fail...");
  }
  else{
    for (unsigned int i = 0; i < pktCnt; i++)
    {
      cmd[4] = i & 0xff;
      cmd[5] = (i >> 8) & 0xff;
      
      int retry_cnt = 0;
    retry:
      delay(10);
      clearRxBuf(); 
      sendCmd(cmd, 6); 
      uint16_t cnt = Serial.readBytes((char *)pkt, PIC_PKT_LEN);
      
      unsigned char sum = 0; 
      for (int y = 0; y < cnt - 2; y++)
      {
        sum += pkt[y];
      }
      if (sum != pkt[cnt-2])
      {
        if (++retry_cnt < 100) goto retry;
        else break;
      }
      
      myFile.write((const uint8_t *)&pkt[4], cnt-6); 
      //if (cnt != PIC_PKT_LEN) break;
    }
    cmd[4] = 0xf0;
    cmd[5] = 0xf0; 
    sendCmd(cmd, 6); 
  }
  myFile.close();
  picNameNum ++;
}

