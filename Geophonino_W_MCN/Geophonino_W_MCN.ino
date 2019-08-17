/*Geophonino-W is a 
 * Wireless multichannel seismic noise recorder open source system for array measurements 
 * The computer programming consists of: 
 * Geophonino_W_ACN.ino: The Arduino Sketch for the acquisition control nodes.
 * Geophonino_W_MCN.ino: The Arduino Sketch for the management control node.
 * Geophonino_W_MCN.pde: The user interface developed by using Processing software.
 * Last update: 11/06/2019
 * Email contact: jl.soler@ua.es, juanjo@dfists.ua.es*/

#include "RTClib.h"
#include <SPI.h>
#include <SdFat.h>
#include <XBee.h>
#include <DueTimer.h>

RTC_DS3231 rtc;
//#define DEBUG 1 //if you uncomment this line, you will see the debug messages by a serial monitor
//#ifdef DEBUG  #endif 

#define BUF_SZ   400
int req_index = 0;
char separator[] = "&";
char ser_req[BUF_SZ] = {0};
volatile bool syncStations = false;
volatile bool readStations = false;
volatile bool sendConfPacket = false;
volatile bool readAdqPackets = false;
volatile bool allStConfiguredOK=false;
volatile bool allStAdqEnd=false;
volatile bool readAdqPacketsStart=false;
byte trys=0;

SdFat sd;
File logFile;
File stDataFile;
char nomFile[] = "";
char stNomFile[] = "";

int duration = 0;
byte gain = 0; //1=x1,    2=x2,    3=x4
byte gLtb = 0; //0=x0, 1=x1, 2=x2, 3=x5, 4=x10, 5=x20, 6=x50, 7=x100
byte srsel = 0; // 0="Sr 10ms, 100-Hz" 1= "Sr 4ms, 250-Hz",  2= "Sr 2ms, 500-Hz"
byte hh = 0, mm = 0, ss = 0;

String gainStr = "", timeRecStr = "", sRateStr = "", nomFichStr = "", overW = "", gLtbStr = "", hhStr = "", mmStr = "", ssStr = "";

String cabSrStr = "";
String cabGainStr = "";
String cabGLtbStr = "";
String cabTimeRecStr = "";

int numSt = 0; //number of active stations

const int chipSelect = 4;
bool firsttime = true;

union
{
  unsigned long ul;
  byte b[4];
} acquiredtime;

union
{
  unsigned long ul;
  byte b[4];
} tacqstaD[13];

union
{
  unsigned long ul;
  byte b[4];
} tmillisD[13];

union
{
  float f;
  byte b[4];
} longitudeD[13];

union
{
  float f;
  byte b[4];
} latitudeD[13];

int yearD[13] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
byte monthD[13] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
byte dayD[13] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
byte hourD[13] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
byte minuteD[13] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
byte secondD[13] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
byte hundredthsD[13] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
byte numsatellitesD[13] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

bool gpsSt[13] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; 
bool activeSt[13] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; //By default 12 active stations
byte stError[13] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; //By default error station counter is set to 0.
union
{
  float f;
  byte b[4];
} longitude[13];

union
{
  float f;
  byte b[4];
} latitude[13];

int yearACN[13] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
byte monthACN[13] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
byte dayACN[13] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
byte hourACN[13] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
byte minuteACN[13] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
byte secondACN[13] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
byte hundredths[13] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
byte numsatellites[13] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
unsigned long age;

bool issetMCNTime=false;
int yearMCN = 0;
byte monthMCN = 0;
byte dayMCN = 0;
byte hourMCN = 0;
byte minuteMCN = 0;
byte secondMCN = 0;

unsigned long tini=0;
unsigned long tfin=0;
unsigned long tdif[2]={0, 0};
unsigned long tstation=0;

/*Payload Size 100 Bytes IN 802.15.4 (Series 1) XBee */
uint8_t payload[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

XBee xbee = XBee();
Rx16Response rx16 = Rx16Response();
Tx16Request tx = Tx16Request(0, payload, sizeof(payload));
TxStatusResponse txStatus = TxStatusResponse();

uint8_t option = 0;
uint8_t data = 0;

uint8_t remoteAddress = 0;

volatile uint8_t msgid [13] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

volatile uint8_t rmsgid[13] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

volatile byte currentSt = 0;

void setup()
{
  Serial.begin(57600);
  Serial1.begin(57600);
  Serial.println("Geophonino_W_MCN.ino");
  xbee.setSerial(Serial1);
  rtc.begin();
  sd.begin(chipSelect, SPI_HALF_SPEED);
  waitGuiContact();
}
void serialEvent1() 
{
  #ifdef DEBUG
    Serial.println("Sea1");
  #endif
  delay(200);
  if(!readAdqPacketsStart)
  {
   char val = Serial1.read();
    if (val == '~')
    {
      readxbeepacket();
      Serial1.flush();
      return;
    }
  }
}
void serialEvent() 
{
  #ifdef DEBUG
    Serial.println("serialEvent");
  #endif
  delay(200);
  if(!readAdqPacketsStart)
  {
    dataSerialInput();  
  }
}
void loop()
{
  if (readStations)
  {
    #ifdef DEBUG
      Serial.println("readStations true");
    #endif  
    for (currentSt = 1; currentSt <= numSt; currentSt++)
    {
      if(currentSt>0 & currentSt<13) //avoid error if one satations send a invalid id
      {
        Serial.println(currentSt);
        tx.setAddress16(currentSt);
        payload[0] = rmsgid[currentSt]; //avoid error 
        if (payload[0]>20)
          payload[0]=0;
        payload[1] = currentSt;
        #ifdef DEBUG
          Serial.println(rmsgid[currentSt]);
        #endif  
        if (sendxbeepacket());
        {
          xbee.readPacket(1900); //This will wait up to 1900 milliseconds for packet to arrive before returning
          readxbeepacket();
          Serial.println("save_st_now"); //ask GUI if user want to go to save GPS station data now
          delay(600);
        }
        if (Serial.available())
          serialEvent();
        if(!readStations)
          currentSt=numSt+1;
      }
    }
    Serial.println("save_st_now"); //ask GUI if user want to go to save GPS station data now
  }
  currentSt=numSt;
  
  if (sendConfPacket)
  {
    #ifdef DEBUG
      Serial.println("sendConfStations");
    #endif    
    allStConfiguredOK=true;
    if(!issetMCNTime)//Stablish the same filename (datetime) for all remote stations
      setMCNTime();//don't change name along the configuration process
    for (currentSt = 1; currentSt <= numSt; currentSt++)
    {
      Serial.print("activeSt[currentSt]= ");
      Serial.println(activeSt[currentSt]);
      delay(100);
      if(activeSt[currentSt])
      {
        if(msgid[currentSt]!=6)
        {  
          allStConfiguredOK=false;
          setAdqParameters(); //load parameters on PayLoad
          tx.setAddress16(currentSt);
          if (sendxbeepacket());
          {
            #ifdef DEBUG
              Serial.println("Sendxbeepacket=true");
            #endif
            xbee.readPacket(5000); //This will wait up to 5000 milliseconds for packet to arrive before returning
            readxbeepacket();
            delay(200);         
          }
        }
        else
        {
        Serial.print("Parameters already loaded OK for station");
        Serial.println(currentSt);
        }        
      }
    }
    if (allStConfiguredOK)
    {
      Serial.println("ALL Stations loaded parameters OK");
      sendConfPacket=false; //When all station are configured for data acquisition, STOP Send Configuration Packets
      syncStations=true;
    }
    else
    {
      for (currentSt = 1; currentSt <= numSt; currentSt++)
      {         
        if(msgid[currentSt]!=6)
        {
          stError[currentSt]+=1;
          if (stError[currentSt]==40)
            activeSt[currentSt]=0;
        }
      }
    }
  }

  if (syncStations)
  {
    Serial.println("SyncStations true");
    for (currentSt = 1; currentSt <= numSt; currentSt++)
    {
      for(trys=1;trys<=2;trys++)
      {
        if(activeSt[currentSt])
        {
          tx.setAddress16(currentSt);
          payload[0] = 3;
          payload[1] = currentSt;
          tini=millis();
          if (sendxbeepacket());
          {
            xbee.readPacket(3000); //This will wait up to 3000 milliseconds for packet to arrive before returning
            Serial.println(trys);
            readxbeepacket();
            Serial.println("StationAdqStatusId" + String(currentSt) + "Sync packet " +String(trys)+ " received."); 
          }
          if (trys==2 and (abs(tdif[trys]-tdif[trys-1]))>3)
          {
            Serial.println(String(tdif[trys])+"-"+String(tdif[trys-1]));
            trys=0;
            stError[currentSt]+=1;
            if (stError[currentSt]==40)
              activeSt[currentSt]=0;
          }
          else
            if(trys==2)
              Serial.println("StationAdqStatusId" + String(currentSt) + " Sync packet " +String(trys)+ " received.");
        }
      }
    }
    Serial.println("All stations Sync_packets_received"); //ask GUI if user want to go to save GPS station data now
    unsigned long millisnow=millis();
    unsigned long taMcn=millisnow+(mm*60*1000)+(ss*1000);
    Serial.println("Send Acq Time to Stations");
    for (currentSt = 1; currentSt <= numSt; currentSt++)
    {
      if(activeSt[currentSt])
      {
        tx.setAddress16(currentSt);
        payload[0] = 4;
        payload[1] = currentSt;
        tini=millis();
        if (sendxbeepacket())
        {
          xbee.readPacket(3000); //This will wait up to 3000 milliseconds for packet to arrive before returning
          readxbeepacket();
          tacqstaD[currentSt].ul=tmillisD[currentSt].ul+(taMcn-millis());
          Serial.println("Fix local MCN time to start acquisition"+String(tacqstaD[currentSt].ul));
          payload[0]=5;
          payload[1]=tacqstaD[currentSt].b[0];
          payload[2]=tacqstaD[currentSt].b[1];
          payload[3]=tacqstaD[currentSt].b[2];
          payload[4]=tacqstaD[currentSt].b[3];
          payload[5] = currentSt;
          if(sendxbeepacket())
          {
            Serial.println("StationAdqStatusId" + String(currentSt) + " Trigger programmed in " +String(tacqstaD[currentSt].ul)+" ms. succesfully");
          }
        }
      }
    }
    syncStations=false;
    readAdqPackets=true; //Start to read confirmation AdqPackets
    readAdqPacketsStart=true;
    //reset time synchronization variables
    tini=0;
    tfin=0;
    tdif[1]=0;
    tdif[2]=0;
    tstation=0;
  }
    
  if (readAdqPackets)
  {
    #ifdef DEBUG
      Serial.println("readAdqOkPackets true");
    #endif
    xbee.readPacket(200); //This will wait up to 200 milliseconds for packet to arrive before returning
    readxbeepacket();
    allStAdqEnd=true;
    for (currentSt = 1; currentSt <= numSt; currentSt++)
    {
      if (msgid[currentSt]!=9)
        allStAdqEnd=false;
      if(!activeSt[currentSt]) //currentSt station has been deactivated
        Serial.println("StationAdqStatusId" + String(currentSt) + " Station " + String(currentSt) + " connection timed out.");
    }
    if(allStAdqEnd)
    {
      readAdqPackets=false;
      Serial.println("Acquisition finished for all stations");
      for (currentSt = 1; currentSt <= numSt; currentSt++)
      {
        stError[currentSt]=0;
        activeSt[currentSt]=0;
        gpsSt[currentSt]=0;
      }
    }   
  }  
  
  if(allStAdqEnd)
  {
    for (currentSt = 1; currentSt <= numSt; currentSt++)
    {
        if (msgid[currentSt]!=9)
          Serial.print("NO msgid 9 received from station: ");
          Serial.print(currentSt); 
          allStAdqEnd=false;
    }
    readAdqPacketsStart=false;
    issetMCNTime=false;
  }
}//Loop Ends
void setAdqParameters()
{ 
  payload[0] = 2;
  payload[1] = gain;// gain;
  Serial.write(payload[1]);
  payload[2] = highByte(duration);// duration
  Serial.write(payload[2]);
  payload[3] = lowByte(duration);// duration
  Serial.write(payload[3]);
  payload[4] = srsel;// 0="10ms, 100-Hz", 1= "4ms, 250-Hz", 2= "Sr 2ms, 500-Hz"
  payload[5] = gLtb; // 0-x0 1-x1 2-x2 3-x5 4-x10 5-x20 6-x50 7-x100
  payload[6] = hourMCN;
  payload[7] = minuteMCN;
  payload[8] =secondMCN;
  payload[9]=dayMCN;
  payload[10]=monthMCN;
  payload[11]=highByte(yearMCN);
  payload[12]=lowByte(yearMCN);
  payload[13] = currentSt;
}
void dataSerialInput()
{
  char val = Serial.read(); // read it and store it in val
  if (val == '~')
  {
    readxbeepacket();
    Serial.flush();
    return;
  }
  while (val != '\n')
  {
    ser_req[req_index] = val;          // save Serial request character
    req_index++;
    val = Serial.read();
  }
  Serial.flush();
  if (StrContains(ser_req, "Contacted"))
  {
    //
    Serial.println("GUI conected");
  }
  /************************     LOAD ST     *****************************/
  if (StrContains(ser_req, "load_st"))
  {
    //load_st&numSt=2
    Serial.println("modo load_st");
    Serial.println(ser_req);

    String numStStr = "";

    char sfind[] = "numSt";
    numStStr = StrValueExtract(ser_req, sfind, separator);
    numSt = numStStr.toInt();
    Serial.println(numSt);

    if (numSt > 0) //If number of stations is higher than zero, read stations gps data
    {
      readStations = true;
      Serial.println("readStations=TRUE");
    }
  }
  /************************     SAVE ST     *****************************/
  if (StrContains(ser_req, "save_st") & readStations) //if readStations=false
  {
    readStations = false; //Stop GPS data acquisition
    Serial.println("READSTATIONS=FALSE");

    //save_st&nomFileSt=almst
    Serial.println("modo save_st");
    Serial.println(ser_req);

    String stNomFileStr = "";

    char sfind[] = "nomFileSt";
    stNomFileStr = StrValueExtract(ser_req, sfind, separator);
    Serial.println(stNomFileStr);
    if (stNomFileStr == "-1") {
      stNomFileStr = "StPosD.dat"; //Status position default filename
    }

    stNomFileStr.toCharArray(stNomFile, sizeof(stNomFileStr));

    if (saveStFile())
      Serial.println("save_st_ok");
    else
      Serial.println("save_st_ko");
    stDataFile.close();
  }

  /************************     CONFIGURE ADQUISITION *******************/
  //conf_adq&nomF=mutxamel&overW=N&Gain=2&tR=15&sR=0&GLtb=2&HH=11&MM=22&SS=33
  if (StrContains(ser_req, "conf_adq"))
  {
    #ifdef DEBUG
      Serial.println("Conf_adq received by serial");
    #endif
    char sfind[] = "Gain";
    gainStr = StrValueExtract(ser_req, sfind, separator);
    if (gainStr == "-1") {
      gainStr == "1";
    }
    Serial.println(gainStr);
    gain = (byte) gainStr.toInt();
    switch (gain)
    {
      case 1:
        cabGainStr = "1";
        break;
      case 2:
        cabGainStr = "2";
        break;
      case 3:
        cabGainStr = "4";
        break;
    }
    strcpy(sfind, "tR");
    timeRecStr = StrValueExtract(ser_req, sfind, separator);
    cabTimeRecStr = timeRecStr;
    if (timeRecStr == "-1") {
      timeRecStr == "1";
    }
    Serial.println(timeRecStr);
    duration = timeRecStr.toInt();
    strcpy(sfind, "sR");
    sRateStr = StrValueExtract(ser_req, sfind, separator);
    if (sRateStr == "-1") {
      sRateStr == "1";
    }
    Serial.println(sRateStr);
    srsel = (byte) sRateStr.toInt();
    switch (srsel) //srsel 0="10ms, 100-Hz", 1= "4ms, 250-Hz",     2= "Sr 2ms, 500-Hz"
    {
      case 0:
        cabSrStr = "10";
        break;
      case 1:
        cabSrStr = "4";
        break;
      case 2:
        cabSrStr = "2";
        break;
    }
    strcpy(sfind, "GLtb");
    gLtbStr = StrValueExtract(ser_req, sfind, separator);
    if (gLtbStr == "-1") {
      gLtbStr == "0";
    }
    Serial.println(gLtbStr);
    gLtb = (byte) gLtbStr.toInt();
    switch (gLtb) // 0-x0 1-x1 2-x2 3-x5 4-x10 5-x20 6-x50 7-x100
    {
      case 0:
        cabGLtbStr = "0";
        break;
      case 1:
        cabGLtbStr = "1";
        break;
      case 2:
        cabGLtbStr = "2";
        break;
      case 3:
        cabGLtbStr = "5";
        break;
      case 4:
        cabGLtbStr = "10";
        break;
      case 5:
        cabGLtbStr = "20";
        break;
      case 6:
        cabGLtbStr = "50";
        break;
      case 7:
        cabGLtbStr = "100";
        break;
    }
    strcpy(sfind, "HH");
    hhStr = StrValueExtract(ser_req, sfind, separator);
    hh =(byte) hhStr.toInt();
    Serial.println(hh);

    strcpy(sfind, "MM");
    mmStr = StrValueExtract(ser_req, sfind, separator);
    mm = (byte) mmStr.toInt();
    Serial.println(mm);

    strcpy(sfind, "SS");
    ssStr = StrValueExtract(ser_req, sfind, separator);
    ss = (byte) ssStr.toInt();
    Serial.println(ss);
    SdFile::dateTimeCallback(dateTime);
    stDataFile=sd.open(stNomFile, O_WRITE | O_APPEND);
    Serial.println("Record configuration parameters");
    if (stDataFile)
    {
      Serial.println("Gain,"+cabGainStr+",Duration,"+cabTimeRecStr+",Samplerate,"+cabSrStr+",LtbAmplification,"+cabGLtbStr);
      stDataFile.println("Gain,"+cabGainStr+",Duration,"+cabTimeRecStr+",Samplerate,"+cabSrStr+",LtbAmplification,"+cabGLtbStr);
      stDataFile.close();
    }    
    sendConfPacket = true; //send the configuration parameters to all stations in loop
  }
  req_index = 0;
  StrClear(ser_req, BUF_SZ);
}
bool saveStFile()
{ 
  if (!sd.begin(chipSelect, SPI_HALF_SPEED)& firsttime)
  {
    Serial.println("Card failed, or not present");
    return (false);
  }
  else
  {
    firsttime = false;
    Serial.println("card initialized.");
  }
  if (sd.exists(stNomFile)) {
    sd.remove(stNomFile);
    Serial.println("Stations File Deleted");
  }
  Serial.println(stNomFile);
  SdFile::dateTimeCallback(dateTime);
  stDataFile=sd.open(stNomFile, O_CREAT | O_WRITE | O_EXCL);
  if (stDataFile)
  {
    for (int i = 1; i <= numSt; i++)
    {
      stDataFile.print(i); stDataFile.print(",");
      stDataFile.print(latitudeD[i].f, 8); stDataFile.print(",");
      stDataFile.print(longitudeD[i].f, 8); stDataFile.print(",");
      stDataFile.print(numsatellitesD[i]); stDataFile.print(",");
      stDataFile.print(hourD[i]); stDataFile.print(":"); stDataFile.print(minuteD[i]); stDataFile.print(":"); stDataFile.print(secondD[i]); stDataFile.print(",");
      stDataFile.print(dayD[i]); stDataFile.print("/"); stDataFile.print(monthD[i]); stDataFile.print("/"); stDataFile.println(yearD[i]);

      /*Set saved values in the GUI*/

      Serial.print("StationData");
      Serial.print(i); Serial.print(" | ");
      Serial.print(latitudeD[i].f, 8); Serial.print(" | ");
      Serial.print(longitudeD[i].f, 8); Serial.print(" | ");
      Serial.print(numsatellitesD[i]); Serial.print(" | ");
      Serial.print(hourD[i]); Serial.print(":"); Serial.print(minuteD[i]); Serial.print(":"); Serial.print(secondD[i]);  Serial.print(" | ");
      Serial.print(dayD[i]); Serial.print("/"); Serial.print(monthD[i]); Serial.print("/"); Serial.println(yearD[i]);
    }
  }
  else
    Serial.println("Error al abrir el fichero");
  return (stDataFile);
}
void readxbeepacket()
{
  #ifdef DEBUG  
    Serial.println("readxbeepacket called");
  #endif
  delay(100); 
  if (xbee.getResponse().isAvailable())
  {
    if (xbee.getResponse().getApiId() == RX_16_RESPONSE)
    {
      #ifdef DEBUG  
      Serial.println("GetApiId:");
      Serial.println(xbee.getResponse().getApiId());
      #endif
      Serial.print("Response available from Xbee Id: ");
      xbee.getResponse().getRx16Response(rx16);
      remoteAddress = rx16.getRemoteAddress16();
      Serial.println(remoteAddress);

      msgid[remoteAddress] = rx16.getData(0);
      Serial.print("msgid: ");
      Serial.println(msgid[remoteAddress]);
      switch (msgid[remoteAddress])
      {
        case 0:
          if (readStations)
            newstationdetected();
            activeSt[currentSt]=1;
          break;
        case 1:
        case 2:
        case 3:
        case 4:
        case 5:
          if (readStations)
            loadgpsdata();
          break;
        case 6:
          Serial.println("StationAdqStatusId" + String(remoteAddress) + " Acquisition configuration parameters received OK");
          break;
        case 7:
          Serial.println("StationAdqStatusId" + String(remoteAddress) + " Error in configuration parameters");
          break;
        case 8:
          acquiredtime.b[0]=rx16.getData(1);
          acquiredtime.b[1]=rx16.getData(2);
          acquiredtime.b[2]=rx16.getData(3);
          acquiredtime.b[3]=rx16.getData(4);
          Serial.println("StationAdqStatusId" + String(remoteAddress) + " Adquisition completed for second: " + String(acquiredtime.ul));
          break;
        case 9:
            Serial.println("StationAdqStatusId" + String(remoteAddress)+" Acquisition ended for station: "+ String(remoteAddress));
          break;
        case 10:
          tfin=millis();
          tmillisD[remoteAddress].b[0] = rx16.getData(1);
          tmillisD[remoteAddress].b[1] = rx16.getData(2);
          tmillisD[remoteAddress].b[2] = rx16.getData(3);
          tmillisD[remoteAddress].b[3] = rx16.getData(4);
          //tstation[remoteAddress]=tmillisD[remoteAddress].ul;
          tdif[trys]=tfin-tini;
          Serial.println("Sta, Tini, Tdif, Tfin, Tsta");
          Serial.println(String(remoteAddress)+","+String(tini)+","+String(tdif[trys])+","+String(tfin)+","+String(tmillisD[remoteAddress].ul));
          break;
        case 11:
          tfin=millis();
          tmillisD[remoteAddress].b[0] = rx16.getData(1);
          tmillisD[remoteAddress].b[1] = rx16.getData(2);
          tmillisD[remoteAddress].b[2] = rx16.getData(3);
          tmillisD[remoteAddress].b[3] = rx16.getData(4);         
          tdif[trys]=tfin-tini;
          Serial.println("Sta, Tini, Tdif, Tfin, Tsta");
          Serial.println(String(remoteAddress)+","+String(tini)+","+String(tdif[trys])+","+String(tfin)+","+String(tmillisD[remoteAddress].ul));
          break;
      }
    }
    else //If it is not a Xbee packet, it is a Serial port command from GUI
    {
      #ifdef DEBUG
        Serial.print("Xbee packet received but NO RX_16_RESPONSE received");
        Serial.print(xbee.getResponse().getApiId());
        Serial.flush();
      #endif
    }
  }
  else
  #ifdef DEBUG 
    Serial.println("No response available");
  #endif
    Serial.flush();
}
bool sendxbeepacket()
{
  bool sendok=false;
  int init = millis();
  xbee.send(tx);
  delay(200);
  #ifdef DEBUG
    Serial.print("XbeePacket sended to: ");
    Serial.println(currentSt);
    Serial.print("Sizeof Payload: ");
    Serial.println(sizeof(payload));
    for (int i = 0; i < 9; i++)
        Serial.println(payload[i]);
  #endif
  bool xbeeread=xbee.readPacket(5000);
  if (xbeeread)
  {
    delay(200);
    init = millis() - init;
    Serial.print("Xbee packet received in: ");
    Serial.print(init);
    Serial.print(" ms from: ");
    Serial.println(currentSt);
    delay(200);
    if (xbee.getResponse().getApiId() == TX_STATUS_RESPONSE)
    {
      xbee.getResponse().getZBTxStatusResponse(txStatus);
      if (txStatus.getStatus() == SUCCESS)
      {
        Serial.println("Success, ACK ok");
        switch (rmsgid[currentSt])
        {
          case 0:
            rmsgid[currentSt] = 1;
            break;
        }
        sendok=true;
      }
      else
      {
        // the remote XBee did not receive our packet. is it powered on?
        Serial.println("ERROR, ACK kO");
        sendok=false;
      }
    }
  }
  else if (xbee.getResponse().isError())
  {
    Serial.println("Error reading packet.  Error code: ");
    Serial.println(xbee.getResponse().getErrorCode());
    sendok=false;
  }
  else
  {
    init = millis() - init;
    Serial.print("Respuesta ACK not received in (ms):");
    Serial.println(init);
    sendok=false;
  }
  
  //Clean payload always
  for (int i = 0; i < sizeof(payload); i++) {
      payload[i]=0;
    }
  return(sendok);  
}
void newstationdetected()
{
  Serial.println("StationStatusId" + String(remoteAddress) +"New station detected, address: " + String(remoteAddress));
}
void loadgpsdata()
{
  switch (rx16.getData(1))
  {
    case 0:
      Serial.println("StationStatusId" + String(remoteAddress) + "Gps not detected");
      break;
    case 2:
      Serial.println("StationStatusId" + String(remoteAddress) + "Gps data is not valid");
      break;
    case 3:
      Serial.println("StationStatusId" + String(remoteAddress) + "No lock detected");
      break;
    case 4:
      Serial.println("StationStatusId" + String(remoteAddress) + "Datetime is not valid");
      break;
    case 5:
      dayACN[remoteAddress] = rx16.getData(2);
      monthACN[remoteAddress] = rx16.getData(3);
      hourACN[remoteAddress] = rx16.getData(4);
      minuteACN[remoteAddress] = rx16.getData(5);
      secondACN[remoteAddress] = rx16.getData(6);

      byte high = rx16.getData(7);
      byte low = rx16.getData(8);
      yearACN[remoteAddress] = word(high, low);

      latitude[remoteAddress].b[0] = rx16.getData(9);
      latitude[remoteAddress].b[1] = rx16.getData(10);
      latitude[remoteAddress].b[2] = rx16.getData(11);
      latitude[remoteAddress].b[3] = rx16.getData(12);

      longitude[remoteAddress].b[0] = rx16.getData(13);
      longitude[remoteAddress].b[1] = rx16.getData(14);
      longitude[remoteAddress].b[2] = rx16.getData(15);
      longitude[remoteAddress].b[3] = rx16.getData(16);
      numsatellites[remoteAddress] = rx16.getData(17);

      Serial.print("StationData");
      Serial.print(remoteAddress); Serial.print(" | ");
      Serial.print(latitude[remoteAddress].f, 8); Serial.print(" | ");
      Serial.print(longitude[remoteAddress].f, 8); Serial.print(" | ");
      Serial.print(numsatellites[remoteAddress]); Serial.print(" | ");
      Serial.print(hourACN[remoteAddress]); Serial.print(":"); Serial.print(minuteACN[remoteAddress]); Serial.print(":"); Serial.print(secondACN[remoteAddress]); Serial.print(" | ");
      Serial.print(dayACN[remoteAddress]); Serial.print("/"); Serial.print(monthACN[remoteAddress]); Serial.print("/"); Serial.println(yearACN[remoteAddress]);
      break;
  }
  if (numsatellites[remoteAddress] >= numsatellitesD[remoteAddress] & numsatellites[remoteAddress]<250 & yearACN[remoteAddress]>0 & dayACN[remoteAddress]>0 & monthACN[remoteAddress]>0)
    updategpsdataforsave();
}
void updategpsdataforsave()
{
  gpsSt[currentSt]=1;
  dayD[remoteAddress] = dayACN[remoteAddress];
  monthD[remoteAddress] = monthACN[remoteAddress];
  hourD[remoteAddress] = hourACN[remoteAddress];
  minuteD[remoteAddress] = minuteACN[remoteAddress];
  secondD[remoteAddress] = secondACN[remoteAddress];
  yearD[remoteAddress] = yearACN[remoteAddress];
  latitudeD[remoteAddress].f = latitude[remoteAddress].f;
  longitudeD[remoteAddress].f = longitude[remoteAddress].f;
  numsatellitesD[remoteAddress] = numsatellites[remoteAddress];
}
void waitGuiContact() /*Program is waiting in a loop while a contact message is not received*/
{
  while (Serial.available() <= 0) {
    Serial.println("A");   // Waiting
    delay(300);
  }
}
bool existFile()
{
  if (!sd.begin(chipSelect, SPI_HALF_SPEED)& firsttime){
    Serial.println("Card failed, or not present");
    return (false);
  }
  Serial.println("card initialized.");
  firsttime = false;
  if (sd.exists(nomFile)) {
    return true;
    Serial.println("File exist");
  }
  else
  {
    return false;
  }
}
/************************************* String functions ********************************/
// Searches for the string sfind in the string str - Returns 1 string found - Returns 0 string not found
char StrContains(char *str, char *sfind)
{
#ifdef DEBUG
  //Serial.println("strcontains start");
#endif
  char found = 0;
  char index = 0;
  char len;

  len = strlen(str);

  if (strlen(sfind) > len) {
    return 0;
  }
  while (index < len) {
    if (str[index] == sfind[found]) {
      found++;
      if (strlen(sfind) == found) {
        return 1;
      }
    }
    else {
      found = 0;
    }
    index++;
  }
  return 0;
}
void StrClear(char *str, char lengthw)
{
  for (int i = 0; i < lengthw; i++) {
    str[i] = 0;
  }
}
String StrValueExtract(char *str, char *sfind, char *separator)
{
  /* StrValueExtract return variables that was found in a string
    str string to analize
    sfind variable name
    separator separator usually &
    str example:"conf_adq&nomF=nomfile&overW=N&Gain=2&tR=15&sR=1"
  */
  unsigned int found = 0;
  unsigned int index = 0;
  int len;
  char value[] = "";
  unsigned int indValue = 0;
  bool founded = false;
  len = strlen(str);
  if (strlen(sfind) > len) {
    Serial.println("ERROR, string finded is longer than original string");
  }
  while (index < len) {
    if ((str[index] == sfind[found]) | founded) {
      found++;
      if ((strlen(sfind) == found & str[index + 1] == '=') | founded) {
        founded = true;
        if (str[index + 2] != separator[0] & (index + 2 < strlen(str)))
        {
          value[indValue] = str[index + 2];
          indValue++;
        }
        else
        {
          value[indValue] = '\0';
          return String(value);
        }
      }
    }
    else {
      found = 0;
    }
    index++;
  }
  Serial.println("NotFound");
  value[0] = '-';
  value[1] = '1';
  value[2] = '\\';
  value[3] = '0';
  return String(value);
}
/************************************* End String Functions ********************************/
void dateTime(uint16_t* date, uint16_t* time)
{
    DateTime now = rtc.now();

  // return date using FAT_DATE macro to format fields
  *date = FAT_DATE(now.year(), now.month(), now.day());

  // return time using FAT_TIME macro to format fields
  *time = FAT_TIME(now.hour(), now.minute(), now.second());
}
void setMCNTime()
{
  issetMCNTime=true;
  DateTime now = rtc.now();
  yearMCN = now.year();
  monthMCN = now.month();
  dayMCN = now.day();
  hourMCN = now.hour();
  minuteMCN = now.minute();
  secondMCN = now.second();
}

