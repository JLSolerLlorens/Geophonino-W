/*Geophonino-W is a: 
 * Wireless multichannel seismic noise recorder open source system for array measurements 
 * The computer programming consists of: 
 * Geophonino_W_ACN.ino: The Arduino Sketch for the acquisition control nodes.
 * Geophonino_W_MCN.ino: The Arduino Sketch for the management control node.
 * Geophonino_W_MCN.pde: The user interface developed by using Processing software.
 * Last update: 11/06/2019
 * Email contact: jl.soler@ua.es, juanjo@dfists.ua.es*/
 
#include "RTClib.h"
#include <XBee.h>
#include <TinyGPS.h>
#include <RTCDue.h>
#include <DueTimer.h>
#include <SdFat.h>
//#define DEBUG 1 //if you uncomment this line, you will see the debug messages by a serial monitor
//#ifdef DEBUG  #endif 
const int chipSelect = 4;

SdFat sd;
RTCDue rtc(XTAL);

bool sta = true; //state change for clk_write ouput
bool openFileok = false;

XBee xbee = XBee();
Rx16Response rx16 = Rx16Response();

TinyGPS gps;

#define GPS_TX_DIGITAL_OUT_PIN 2//5
#define GPS_RX_DIGITAL_OUT_PIN 4//6

union
{
  unsigned long ul;
  byte b[4];
} acquiredtime;

union
{
  unsigned long ul;
  byte b[4];
} tmillis;

union
{
  float f;
  byte b[4];
} longitude;

union
{
  float f;
  byte b[4];
} latitude;

float hdopgps=0;
double sumlat,sumlon=0;
double mplat,mplon=0;
int sumpesos=0;

int year;
byte month, day, hour, minute, second, centisecond, hundredths, numsatellites;
unsigned long age;
bool clocktime = true;

// data that is being carried within a packet.
/*Payload Size 100 Bytes IN 802.15.4 (Series 1) XBee */
uint8_t payload[18] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

//Tx16Request(Destination address, FFFF for broadcast, data, size of data)
Tx16Request tx = Tx16Request(0x000D, payload, sizeof(payload));//0X000D->13

TxStatusResponse txStatus = TxStatusResponse();

uint8_t remoteAddress = 0;
//Data packet id sended on payload
uint8_t msgid = 0;
uint8_t rmsgid = 0;//Request message id

int led = 13;

byte srsel = 0;
float addtime = 0;
volatile unsigned long initime = 0;
volatile unsigned long timeISR = 0;

bool endacq = true; //endacq=false during acquisition
bool waitingXbeePackets = true;

volatile bool buffAct = 0;

volatile bool clearbuf0 = 0;
volatile bool clearbuf1 = 0;

volatile int sample0 = 0;
volatile int sample1 = 0;

volatile int timesam0[2000] = {0};
volatile int timesam1[2000] = {0};
volatile uint16_t datab0 [2000] = {0};
volatile uint16_t datab1 [2000] = {0};

File dataFile;
File gpsFile;
File gpsFile2;
int copyGpsRawData;
bool gpsdatetimeOk=false;
bool existGpsFile=false;
bool firsttime = true; //true while sdcard not initzialized
char nomFile[20] = "";
char nomgpsFile[20] = "";
int cont = 1000; //milliseconds
int cont2 = 0; //milliseconds
int duration = 0;

volatile byte myStationId = 0;

void setup()
{
  Serial.begin(57600);
  Serial.println("Geophonio_W_ACN.ino");
  xbee.setSerial(Serial);
  pinMode(led, OUTPUT);
  pinMode(3, OUTPUT); //CLK signal for MAX7404
  pinMode(5, OUTPUT); //Gain selector for LTB5
  pinMode(6, OUTPUT); //Gain selector for LTB5
  pinMode(7, OUTPUT); //Gain selector for LTB5

  digitalWrite(led, LOW);
  
  // Serial1 is GPS
  Serial1.begin(9600);//38400);
  pinMode(GPS_TX_DIGITAL_OUT_PIN, INPUT); //GPS data INPUT
  pinMode(GPS_RX_DIGITAL_OUT_PIN, INPUT); //GPS data INPUT

  sd.begin(chipSelect, SPI_EIGHTH_SPEED); // SPI_HALF_SPEED SPI_QUARTER_SPEED SPI_EIGHTH_SPEED SPI_SIXTEENTH_SPEED
} 
void loop()
{
  if (waitingXbeePackets) //Adquisition mode not enabled
  {
    #ifdef DEBUG
      Serial.println("waitingXbeePackets");
    #endif
    waitXbeePacket();
  }

  if (!endacq)
  {
    //**********************¿Should clean Buffer 0?*********************
    if (!endacq & clearbuf0)
    {
      clearbuf0 = 0;
      SdFile::dateTimeCallback(dateTime);
      dataFile=sd.open(nomFile, O_WRITE | O_APPEND | O_AT_END);
      for (int i = 0; i < sample0; i++) {
        dataFile.println(String(timesam0[i])+","+String(datab0[i]));
      }
      dataFile.close();
      sample0 = 0;
    }
    //**********************¿Should clean Buffer 1?*********************
    else if (!endacq & clearbuf1)
    {
      clearbuf1 = 0;
      SdFile::dateTimeCallback(dateTime);
      dataFile=sd.open(nomFile, O_WRITE | O_APPEND | O_AT_END);      
      for (int i = 0; i < sample1; i++) {
        dataFile.println(String(timesam1[i])+","+String(datab1[i]));
      }
      dataFile.close();
      sample1 = 0;
    }
    //**************** ¿Should clean Buffers? ********************

    //Get GPS data
    if (millis() - initime > (cont + 250 * myStationId) & !endacq)
    {
      cont = cont + 2000; 
      #ifdef DEBUG
        Serial.print("NomgpsFile: ");
        Serial.println(nomgpsFile);
      #endif
      addgpsdataToFile();
      //sendadqtimenow();
    }
    //send a signal to GUI
    if (millis() - initime > (cont2 + 3000 * (myStationId)) & !endacq)
    {
      #ifdef DEBUG
        Serial.println("adq_now_ok " + String(((millis() - initime) / 1000)));
      #endif
      cont2 = cont2 + 36000; //Every 36000 miliseconds
      msgid = 8;
      //addgpsdataToFile();
      sendadqtimenow();
    }
    //***************************** END ACQUISITION *****************************
    if (((millis() - initime) > (duration * 1000 + addtime)) & !endacq)
    {
      Timer3.stop();
      Timer4.stop();
      Serial.println("Timer parado");

      /*SEND TO THE GUI SIGNAL OF ACQUISITION WAS ENDED*/
      msgid = 9;
      delay(250 * myStationId);
      sendadqtimenow();
      /*SEND TO THE GUI SIGNAL OF ACQUISITION WAS ENDED*/
      SdFile::dateTimeCallback(dateTime);
      dataFile=sd.open(nomFile, O_WRITE | O_APPEND | O_AT_END);
      if (buffAct == 1) //If buffer active is 1, write first to file buffer0
      {
        if (!endacq & sample0 > 0)
        {
          clearbuf0 = 0;
          for (int i = 0; i < sample0; i++) {
            dataFile.println(String(timesam0[i])+","+String(datab0[i]));
          }
          sample0 = 0;
        }
        if (!endacq & sample1 > 0)
        {
          clearbuf1 = 0;
          for (int i = 0; i < sample1; i++) {
            dataFile.println(String(timesam1[i])+","+String(datab1[i]));
          }
          sample1 = 0;
        }
      }
      else    //If buffer active is 0, write first to file buffer1
      {
        if (!endacq & sample1 > 0)
        {
          clearbuf1 = 0;
          for (int i = 0; i < sample1; i++) {
            dataFile.println(String(timesam1[i])+","+String(datab1[i]));
          }
          sample1 = 0;
        }
        if (!endacq & sample0 > 0)
        {
          clearbuf0 = 0;
          for (int i = 0; i < sample0; i++) {
            dataFile.println(String(timesam0[i])+","+String(datab0[i]));
          }
          sample0 = 0;
        }
      }
      dataFile.close();

      if (saveCalculatedPosition())
      {
      #ifdef DEBUG
        Serial.println("SaveCalculatedPosition_OK");
      #endif
      }
      else
      {
        #ifdef DEBUG
        Serial.println("SaveCalculatedPosition_KO");
        #endif  
      }
      mplat=0;
      sumlat=0;
      mplon=0;
      sumlon=0;
      sumpesos=0;

      endacq = true;
      clearbuf0 = 0;
      clearbuf1 = 0;
      sample1 = 0;
      sample0 = 0;
      buffAct = 0;
      cont = 1000;
      cont2 = 0;
      Serial.println("adq_end");
      endacq = true;
      waitingXbeePackets = true;
      existGpsFile=false;
    }
  
  }//End If (!endacq);
}
void waitXbeePacket()
{
  //wait xbee packet conection from receiver
  xbee.readPacket(200);
  while (!xbee.getResponse().isAvailable())
  {
    //Serial.println("Waiting request packet from receiver");
    xbee.readPacket(200);
  }
  readxbeepacket();
}
void readxbeepacket()
{
  if (xbee.getResponse().isAvailable())
  {
    Serial.print("Response available from Xbee Id: ");
    if (xbee.getResponse().getApiId() == RX_16_RESPONSE)
    {
      xbee.getResponse().getRx16Response(rx16);
      remoteAddress = rx16.getRemoteAddress16();
      Serial.println(remoteAddress);
      rmsgid = rx16.getData(0);
      Serial.print("rmsgid: ");
      Serial.println(rmsgid);
      switch (rmsgid)
      {
        case 0: //Establish first connection
          msgid = 0;
          myStationId = rx16.getData(1);
          #ifdef DEBUG
            Serial.print("My station ID is: ");
            Serial.println(myStationId);
          #endif
          //while (!startconnection());
          if (startconnection())
          {
            #ifdef DEBUG
              Serial.println("Connection packet sent succesfully!");
            #endif
          }
          else
          {
            #ifdef DEBUG
              Serial.println("Connection packet NOT sent KO");
            #endif
          }
          break;
        case 1:  //Send gpsData
          msgid = 1;
          if (sendgpsdata())
          {
            #ifdef DEBUG
              Serial.println("GPS packet sent succesfully!");
            #endif
            if (gpsdatetimeOk)
            {
              if(!existGpsFile)
                openGPSFile();  
              Serial.println(saveGPSData());
              Serial.println("Returned form saveGpsData");
              gpsdatetimeOk=false;
            }
          }
          else
          {
            #ifdef DEBUG
              Serial.println("GPS packet NOT sent KO");
            #endif
          }
          break;
        case 2: //Configure adquisition parameters received
          #ifdef DEBUG
            Serial.print("Configure acq parameters received. R_Msgid= ");
            Serial.println(rmsgid);
          #endif
          msgid = 2;
          if (getAdqConfPacket())
          {
            #ifdef DEBUG
              Serial.println("Configura packet received ok");
            #endif
          }
          else
          {
            waitingXbeePackets = true;
            #ifdef DEBUG
              Serial.println("Configura packet NOT received KO");
            #endif
          }
          break;
        case 3:
          tmillis.ul = millis();
          payload[0] = 10;
          payload[1] = tmillis.b[0];
          payload[2] = tmillis.b[1];
          payload[3] = tmillis.b[2];
          payload[4] = tmillis.b[3];

          if (sendxbeepaket())
          {
            Serial.println("SYNC packet sended OK. Millis time sended is: ");
            Serial.println(tmillis.ul);
          }
          else
            Serial.println("Error sending SYNC packet response");

          break;
        case 4:
          tmillis.ul = millis();
          payload[0] = 11;
          payload[1] = tmillis.b[0];
          payload[2] = tmillis.b[1];
          payload[3] = tmillis.b[2];
          payload[4] = tmillis.b[3];

          if (sendxbeepaket())
          {
            Serial.println("Start acquisition millis packet sended OK. Millis time sended is: ");
            Serial.println(tmillis.ul);
          }
          else
            Serial.println("Error sending SYNC packet response");

          break;
        case 5:
          tmillis.b[0] = rx16.getData(1);
          tmillis.b[1] = rx16.getData(2);
          tmillis.b[2] = rx16.getData(3);
          tmillis.b[3] = rx16.getData(4);
          waitingXbeePackets = false;
          waitforlaunchAcq();
      }
    }
  }
}
bool sendadqtimenow()
{
  #ifdef DEBUG
    Serial.println("Send_ADQ_TimeNow");
  #endif
  payload[0] = msgid; //msgid=8 Acquiring data for second X  msgid=9 Acquisition is finished
  acquiredtime.ul = ((millis()-initime) / 1000);
  payload[1] = acquiredtime.b[0];
  payload[2] = acquiredtime.b[1];
  payload[3] = acquiredtime.b[2];
  payload[4] = acquiredtime.b[3];
  xbee.send(tx);
  //Clean payload always
  for (int i = 0; i < 5; i++) {
    payload[i] = 0;
  }
}
void addgpsdataToFile()
{
  if(readgpsdata()==5)
  {
    if(!existGpsFile)
      openGPSFile(); 
    Serial.println(saveGPSData());
    Serial.println("Returned from saveGpsData called from addgpsdataToFile");
  }
}
bool sendgpsdata()
{
  payload[0] = msgid;
  switch (readgpsdata())
  {
    case 1:
      payload[1] = 1;
      break;
    case 2:
      payload[1] = 2;
      break;
    case 3:
      payload[1] = 3;
      break;
    case 4:
      payload[1] = 4;
      break;
    case 5:
      payload[1] = 5;
      payload[2] = day;
      payload[3] = month;
      payload[4] = hour;
      payload[5] = minute;
      payload[6] = second;
      payload[7] = highByte(year);
      payload[8] = lowByte(year);
      payload[9] = latitude.b[0];
      payload[10] = latitude.b[1];
      payload[11] = latitude.b[2];
      payload[12] = latitude.b[3];
      payload[13] = longitude.b[0];
      payload[14] = longitude.b[1];
      payload[15] = longitude.b[2];
      payload[16] = longitude.b[3];
      payload[17] = numsatellites;
      gpsdatetimeOk=true;
      break;
  }
  return (sendxbeepaket());
}
/*readgpsdata() returns:
  1 - If gps connection is not detected.
  2 - If gps connection is detected but the data come in is not valid.
  3 - If No lock detected = invalid age.
  4 - If No valid datetime detected = invalid age.
  5 - If gps data (lock and datetime) is valid.
*/
byte readgpsdata()
{

  bool newData = false;
  unsigned long chars = 0;
  byte responsecode = 0;
  bool gpsconnection = false;
  unsigned short sentences, failed;
  unsigned long timegpsin, timegpsput, starttime, wait;
  #ifdef DEBUG
    Serial.println("Readgpsdata");
  #endif
  starttime = millis();
  for (unsigned long start = millis(); millis() - start < 1000;)
  {
    while (Serial1.available())
    {
      int c = Serial1.read();
      //Serial.print((char)c); // if you uncomment this line, you will see the raw data from the GPS 
      ++chars;
      if (gps.encode(c)) // Did a new valid sentence come in?
      {
        timegpsin = millis();
        newData = true;
        break;
      }
    }
  }
  if (newData)
  { // Did a new valid sentence come in?
    Serial.println("A new valid sentence come in");
    gps.crack_datetime(&year, &month, &day, &hour, &minute, &second, &hundredths, &age);
    if (year<2018)
    {
      responsecode = 2;//year<2018, it is a not valid data
    }
    rtc.setTime(hour, minute, second);
    rtc.setDate(day, month, year);
    #ifdef DEBUG
      Serial.println("Date and Time of the Arduino clock");
      Serial.print(rtc.getDay());
      Serial.print("/");
      Serial.print(rtc.getMonth());
      Serial.print("/");
      Serial.print(rtc.getYear());
      Serial.print("\t");
      // ...time...
      Serial.print(rtc.getHours());
      Serial.print(":");
      Serial.print(rtc.getMinutes());
      Serial.print(":");
      Serial.println(rtc.getSeconds());
    #endif
    gps.f_get_position(&latitude.f, &longitude.f, &age);
    if (age == TinyGPS::GPS_INVALID_AGE)
    {
      responsecode = 3;
      #ifdef DEBUG
        Serial.println("No lock detected!");
      #endif
    }
    else
    {
      latitude.f == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : latitude.f;
      longitude.f == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : longitude.f;
      numsatellites = gps.satellites();
      hdopgps=gps.hdop();
      #ifdef DEBUG
        Serial.println("GPS locked, data is valid!");
        
      #endif
    }

    gps.crack_datetime(&year, &month, &day, &hour, &minute, &second, &hundredths, &age);
    if (age == TinyGPS::GPS_INVALID_AGE)
    {
      responsecode = 4;
      #ifdef DEBUG
        Serial.println("Invalid datetime data");
      #endif
    }
    else
    {
      responsecode = 5;
      #ifdef DEBUG
        Serial.print("Year: "); Serial.print(year); Serial.print(", Month: "); Serial.print(month); Serial.print(", Day: ");  Serial.println(day);
        Serial.print(hour); Serial.print(":"); Serial.print(minute); Serial.print(":"); Serial.println(second); Serial.print("Hundredths: "); Serial.println(hundredths);
      #endif
    }
  }
  else
  {
    responsecode = 2;
  }
  if (responsecode == 0)
  {
    responsecode = 1;
    #ifdef DEBUG
      Serial.println("Check GPS connection");
    #endif
  }
  return responsecode;
}
bool startconnection()
{
  payload[0] = msgid;
  return (sendxbeepaket());
}
bool sendxbeepaket()
{
  bool sendok = false;
  xbee.send(tx);
  delay(50);
  if (xbee.readPacket(200))
  {
    if (xbee.getResponse().getApiId() == TX_STATUS_RESPONSE)
    {
      xbee.getResponse().getZBTxStatusResponse(txStatus);
      if (txStatus.getStatus() == SUCCESS)
      {
        #ifdef DEBUG
          Serial.println("Success, ACK ok");
        #endif
        switch (msgid) 
        {
          case 0:  //
            msgid = 1;
            break;
        }
        #ifdef DEBUG
          Serial.print("Sended msgid: ");
          Serial.println(msgid);
        #endif
        sendok = true;
      }
      else
      {
        // the remote XBee did not receive our packet. is it powered on?
        #ifdef DEBUG
          Serial.println("ERROR, ACK kO");
        #endif
        sendok = false;
      }
    }
  }
  else if (xbee.getResponse().isError())
  {
    #ifdef DEBUG
      Serial.println("Error reading packet.  Error code: ");
      Serial.println(xbee.getResponse().getErrorCode());
    #endif
    blinkLed(led, 5, 1000);
    sendok = false;
  }
  else
  {
    #ifdef DEBUG
      Serial.println("No ACK response received and no error detected");
    #endif
    // local XBee did not provide a timely TX Status Response. Radio is not configured properly or connected    
    sendok = false;
  }
  //Clean payload always
  for (int i = 0; i < sizeof(payload); i++) {
    payload[i] = 0;
  }
  return sendok;
}
bool getAdqConfPacket()
{
  bool getConfok = false;
  byte gain = rx16.getData(1); //gain
  byte high = rx16.getData(2); //highByte(duration);
  byte low = rx16.getData(3); //lowByte(duration);
  duration = word(high, low);
  srsel = rx16.getData(4); //srsel;// 0="10ms, 100-Hz", 1= "4ms, 250-Hz",     2= "Sr 2ms, 500-Hz",     3= "Sr 1ms, 1000-Hz"
  byte gLtb = rx16.getData(5); //gLtb; // 0-0 1-1 2-2 3-5 4-10 5-20 6-50 7-100
  byte hh = rx16.getData(6);//rtc.getHours(); HH;
  byte mm = rx16.getData(7);//rtc.getMinutes(); MM;
  byte ss = rx16.getData(8);//rtc.getSeconds(); SS;
  byte dayMCN=rx16.getData(9);
  byte monthMCN=rx16.getData(10);
  byte highyear=rx16.getData(11);
  byte lowyear=rx16.getData(12);
  int yearMCN=word(highyear, lowyear);
  #ifdef DEBUG
    Serial.println(gain);
    Serial.println(duration);
    Serial.println(srsel);
    Serial.println(gLtb);
    Serial.print(hh); Serial.print(":"); Serial.print(mm); Serial.print(":"); Serial.print(ss);
  #endif
  for( int i = 0; i < sizeof(nomFile);  ++i )
    nomFile[i] = (char)0;
  //All parameters are ok
  //Open file for store acquired data
  sprintf(nomFile, "%02u%02u%02u_%02u%02u%04u.s%02u",hh,mm,ss,dayMCN,monthMCN,yearMCN,myStationId); //%02u unsigned 2digits with leading zero
  Serial.println(nomFile);
  delay(2000);    
  openFileok = openFile(); 
  Serial.print("openFileok=");
  Serial.println(openFileok);
  configAnalogInput(gain, gLtb, srsel); //Clk signal for Max7404 cut-off frequency is launched here
  Serial.println("Config parameters OK");
  #ifdef DEBUG
    Serial.println("nomFile");
    Serial.println(nomFile);
  #endif
  getConfok = true;
  if (getConfok) //Send message to receiver indicating configuration is OK
  {
    Serial.println("Acquisition configuration parametres received OK");
    msgid = 6;
    payload[0] = msgid;
    if (sendxbeepaket())
      Serial.println("Response to conf packet sended OK");
    else
      Serial.println("Error sending conf packet response");
  }
  else //Send message to receiver indicating configuration error
  {
    msgid = 7;
    payload[0] = msgid;
    sendxbeepaket();
  }
  return (getConfok);
}
void configAnalogInput(byte gain, byte gLtb, byte srsel)
{
  #ifdef DEBUG
    Serial.println("configAnalogInput");
  #endif

  bool G0 = false;
  bool G1 = false;
  bool G2 = false;

  analogReadResolution(12);
  //gain -> 1=x1,    2=x2,    3=x4
  // Set up the configuration of each ADC channel
  REG_ADC_MR = (REG_ADC_MR & 0xFF0FFFFF) | 0x00B00000;
  ADC->ADC_WPMR &= ~(1 << ADC_WPMR_WPEN); //unlock register

  //ADC->ADC_COR = 0xFFFF0000;    //Uncomment to set fully differential mode to all chanels
  ADC->ADC_COR = 0x00000000;      //Set single ended mode to all chanels

  /*set STARTUP time ADC to a smaller number,*/
  unsigned long reg;
  REG_ADC_MR = (REG_ADC_MR & 0xFFF0FFFF) | 0x00020000;
  reg = REG_ADC_MR;
  
  switch (gain)
  {
    case (1):
       ADC->ADC_CGR=0x15555555; //set gain = 1 for all chanels 0x15555555=>0101010101010101010101010101010101
      break;
    case (2):
      ADC->ADC_CGR=0xAAAAAAAA; //set gain = 2 for all chanels 
      break;
    case (3):
       ADC->ADC_CGR=0xFFFFFFFF; //set gain = 4 for all chanels
      break;
  }

  Serial.println("gLtb value" + gLtb);
  switch (gLtb)
  {
    case (0): //000
      G2 = false;  G1 = false;  G0 = false;
      break;
    case (1): //001
      G2 = false;  G1 = false;  G0 = true;
      break;
    case (2): //010
      G2 = false;  G1 = true;  G0 = false;
      break;
    case (3): //011
      G2 = false;  G1 = true;  G0 = true;
      break;
    case (4): //100
      G2 = true;  G1 = false;  G0 = false;
      break;
    case (5): //101
      G2 = true;  G1 = false;  G0 = true;
      break;
    case (6): //110
      G2 = true;  G1 = true;  G0 = false;
      break;
    case (7): //111
      G2 = true;  G1 = true;  G0 = true;
      break;
  }
  
  digitalWrite(7, G2);
  digitalWrite(6, G1);
  digitalWrite(5, G0);
  
  switch (srsel) //0="10ms, 100-Hz", 1= "4ms, 250-Hz",     2= "Sr 2ms, 500-Hz"
  {
    case 0://10ms
      Timer4.attachInterrupt(clkwrite).setFrequency(8000).start(); //Clk=4 KHz    fc=40 Hz
      break;
    case 1://4ms
      Timer4.attachInterrupt(clkwrite).setFrequency(20000).start(); //Clk=10 KHz    fc=100 Hz
      break;
    case 2://2ms
      Timer4.attachInterrupt(clkwrite).setFrequency(40000).start(); //Clk=20 KHz    fc=200 Hz
      break;
  }

}
/*Interrupt Service Routine to generate clock signal for MAX7404 filter*/
void clkwrite()
{
  digitalWrite(3, sta);
  sta = !sta;
}
bool saveCalculatedPosition()
{
  bool response=false;
  Serial.println("Save Position");
  snprintf(nomgpsFile, sizeof(nomgpsFile), "%02u%02u%02u_%02u%02u%04u.p%02u",hour,minute,second,day,month,year,myStationId);

  if (!sd.begin(chipSelect, SPI_EIGHTH_SPEED)& firsttime){
    Serial.println("Card failed, or not present");
    return (false);
  }
  Serial.println("card initialized.");
  firsttime = false;
  if (sd.exists(nomgpsFile)){
    sd.remove(nomgpsFile);
    Serial.println("File Deleted");
  }
  SdFile::dateTimeCallback(dateTime);
  gpsFile=sd.open(nomgpsFile, O_CREAT | O_WRITE | O_EXCL);
  
  if(gpsFile)
  {
    Serial.println("Position file opened OK");
    response=true;
    mplat=sumlat/sumpesos;
    mplon=sumlon/sumpesos;
    gpsFile.println("Position DATA for station "+String(myStationId));
    gpsFile.println("Latitude,Longitude,sumlat,sumlon,sumpesos");
    gpsFile.println(String(mplat,10)+","+String(mplon,10)+","+String(sumlat,10)+","+String(sumlon,10)+","+String(sumpesos,10));
    Serial.println(String(mplat,10)+","+String(mplon,10)+","+String(sumlat,10)+","+String(sumlon,10)+","+String(sumpesos,10));
    gpsFile.close();
  }
  else
    Serial.println("Position file can't be opened KO");
  return(response);
}
bool saveGPSData()
{
  Serial.println("Save GPS data");
  Serial.println(nomgpsFile);
  SdFile::dateTimeCallback(dateTime);
  gpsFile2=sd.open(nomgpsFile, O_WRITE | O_APPEND | O_AT_END);
  if(gpsFile2)
  {
    Serial.println("Gps file reopened OK");
    Serial.print(String(latitude.f,8)+","+String(longitude.f,8)+","+String(hdopgps,4)+","+String(numsatellites)+"\r\n");    
    gpsFile2.print(String(hour)+":"+String(minute)+":"+String(second)+","+String(latitude.f,10)+","+String(longitude.f,10)+","+String(hdopgps,4)+","+String(numsatellites)+"\r\n");    
    gpsFile2.close();
    if (latitude.f>0 & longitude.f!=0 and hdopgps>10 and hdopgps<200 and numsatellites>5)
    {
      int peso=200-hdopgps;
      sumlat+=latitude.f*peso;
      sumlon+=longitude.f*peso;
      sumpesos+=peso;
    }
  }
  else
    Serial.println("Gps file can't be opened KO");
  return(gpsFile2);
}
bool openGPSFile()
{
  snprintf(nomgpsFile, sizeof(nomgpsFile), "%02u%02u%02u_%02u%02u%04u.g%02u",hour,minute,second,day,month,year,myStationId); //%02u unsigned 2digits with leading zero
  Serial.println(nomgpsFile);
  if (!sd.begin(chipSelect, SPI_EIGHTH_SPEED)& firsttime){
    Serial.println("Card failed, or not present");
    return (false);
  }
  Serial.println("card initialized.");
  firsttime = false;
  if (sd.exists(nomgpsFile)){
    sd.remove(nomgpsFile);
    Serial.println("File Deleted");
  }
  SdFile::dateTimeCallback(dateTime);
  gpsFile=sd.open(nomgpsFile, O_CREAT | O_WRITE | O_EXCL);
  if(gpsFile)
  {
    Serial.println("GPS file initialized OK");
    gpsFile.println("GPS DATA for station "+String(myStationId));
    gpsFile.println("Time,Latitude,Longitude,Hdop,NumSatellites");
    gpsFile.close();  
    existGpsFile=true;
  }
  else
    Serial.println("Error initalizing GPS data file");
  return(gpsFile);
}
bool openFile()
{
  bool response=false;
  if (!sd.begin(chipSelect, SPI_EIGHTH_SPEED)& firsttime){
    Serial.println("Card failed, or not present");
    return (false);
  }
  Serial.println("card initialized.");
  firsttime = false;
  if (sd.exists(nomFile)){
    sd.remove(nomFile);
    Serial.println("File Deleted");
  }
  SdFile::dateTimeCallback(dateTime);
  dataFile=sd.open(nomFile, O_CREAT | O_WRITE | O_EXCL);
  Serial.print(" ** File open value:");
  Serial.println(dataFile);
  if (dataFile)
    response=true;
  dataFile.close();
  Serial.print("response:");
  Serial.println(response);
  return (response);
}
void blinkLed(int pin, int times, int wait) {
  for (int i = 0; i < times; i++) {
    digitalWrite(pin, HIGH);
    delay(wait);
    digitalWrite(pin, LOW);
    if (i + 1 < times) {
      delay(wait);
    }
  }
}
void waitforlaunchAcq()
{
  while (millis() < tmillis.ul)
  {
  }
  launchAcquisition();
}
void launchAcquisition()
{
  if (openFileok)
  {
    Serial.println("openfileok=1");
    //srsel=->  0= "10ms, 100-Hz",  1= "4ms, 250-Hz",  2= "Sr 2ms, 500-Hz"
    switch (srsel)
    {
      case (0):
        //delay(16000);
        initime = millis();
        Timer3.attachInterrupt(isrADC).start(10000); // Every 10 ms -> 100 samples per second
        endacq = false;
        addtime = 11; //milliseconds
        break;
      case (1):
        //delay(16000);
        initime = millis();
        Timer3.attachInterrupt(isrADC).start(4000); // Every 4 ms -> 250 samples per second
        endacq = false;
        addtime = 5; //millisecons
        break;
      case (2):
        //delay(16000);
        initime = millis();
        Timer3.attachInterrupt(isrADC).start(2000); // Every 2 ms -> 500 samples per second
        endacq = false;
        addtime = 3;
        break;
    }
  }
  #ifdef DEBUG
    Serial.println(nomgpsFile);
    Serial.println("FIN_launchAcquisition");
  #endif
}
/* Interrupt Service Routine to acquire data*/
void isrADC() 
{
  if (buffAct)
  {
    timesam1[sample1] = (millis() - initime);
    datab1[sample1] = analogRead(0); 
    sample1++;
    if (sample1 > 500 & sample0 == 0)
    {
      clearbuf1 = 1;
      buffAct = !buffAct;
    }
  }
  else
  {
    timesam0[sample0] = (millis() - initime);
    datab0[sample0] = analogRead(0);
    sample0++;
    if (sample0 > 500 & sample1 == 0)
    {
      clearbuf0 = 1;
      buffAct = !buffAct;
    }
  }
}
void dateTime(uint16_t* date, uint16_t* time) {    
  // return date using FAT_DATE macro to format fields
  *date = FAT_DATE(rtc.getYear(), rtc.getMonth(), rtc.getDay());

  // return time using FAT_TIME macro to format fields
  *time = FAT_TIME(rtc.getHours(), rtc.getMinutes(), rtc.getSeconds());
}
