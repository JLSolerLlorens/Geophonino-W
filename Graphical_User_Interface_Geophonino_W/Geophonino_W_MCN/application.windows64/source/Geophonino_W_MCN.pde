/*Geophonino-W is a: 
 * Wireless multichannel seismic noise recorder open source system for array measurements 
 * The computer programming consists of: 
 * Geophonino_W_ACN.ino: The Arduino Sketch for the acquisition control nodes.
 * Geophonino_W_MCN.ino: The Arduino Sketch for the management control node.
 * Geophonino_W_MCN.pde: The user interface developed by using Processing software.
 * Last update: 21/05/2019
 * Email contact: jl.soler@ua.es, juanjo@dfists.ua.es*/
 
import controlP5.*;
import processing.serial.*;

/*Initialization of variables*/
Serial myPort;
String val; //store data received in the serial port
boolean firstContact = false;  //is false while Processing isn't connected to Arduino
ControlP5 cp5; //cp5 printer for GUI
DropdownList ddlCom,ddlGain,ddlSr,ddlGainLtb; //DDL Drop Down List objtects

PFont fontTit = createFont("arial",20);
PFont fontTex = createFont("arial",12);
PFont fontTexMini = createFont("arial",10);

Textfield status; //Text field for notifications to the user
Textfield stationStatus;

int myColorBackground = 255; //Default Background

int state=1; 

OutputStream OutputFileRecived; 
  
byte [] FileBuffer=new byte[0];

String folder=null; //Folder when the file downloaded from Arduino will be saved

String overwriteFile="N"; //When selected name already exist, user can overwrite it. By default, N=false.

/* Variables progress bar */
int count=0;
int ini=0;
Integer duration=0;
Integer tamFile=0;
/* End variables progress bar */

/* Variables wireless stations */
int stationId=0;
int numActiveSt=0;
/* End variables wireless stations */

String command="";
String stFileName="";

int time;
int wait = 3000;


void setup() {
  size(1100,650); //Window size 1000,650
  time = millis();//store the current time
  cp5 = new ControlP5(this);

  cp5.addTextlabel("textWarnings").setText("Current status: ")
  .setPosition(50,620).setColorValue(0).setFont(fontTex).setVisible(false);;
    
  cp5.addTextfield("Status")
     .setPosition(150,620)
     .setSize(550,20)
     .setFocus(true)
     .setFont(fontTex)
     .setColorValue(255)
     .setColorBackground(color(200, 200, 200))
     .setVisible(false);;

   status = ((Textfield)cp5.getController("Status"));
   status.setValue("Welcome to Geophonino-W MCN administrator GUI");
   
  cp5.addTextlabel("Connection").setText("1º Connection configuration")
  .setPosition(50,20).setColorValue(0).setFont(fontTit);
  
  cp5.addTextlabel("txtNumSt").setText("Number of stations:")
  .setPosition(50,60).setColorValue(0).setFont(fontTex); 
  
  cp5.addTextfield("numSt")
     .setPosition(170,60)
     .setSize(100,18)
     .setFocus(true)
     .setFont(fontTex)
     .setColorValue(255)
     .setColorBackground(color(200, 200, 200))
      .getCaptionLabel().hide();
      
      
  cp5.addTextlabel("txtSerial").setText("Select the Geophonino-W connection port")
  .setPosition(340,60).setColorValue(0).setFont(fontTex); 
 
  cp5.addButton("Connect...")
     .setValue(0)
     .setPosition(670,60)
     .setSize(100,14)
     .setId(1);
   
   cp5.addButton("Refresh port list")
     .setValue(0)
     .setPosition(790,60)
     .setSize(100,14)
     .setId(2);

  cp5.addTextlabel("StationStatus").setText("2º Station positions and date time")
  .setPosition(50,100).setColorValue(0).setFont(fontTit);
  
   cp5.addTextlabel("txtNameStFile").setText("Stations positions file name: ")
  .setPosition(500,100).setColorValue(0).setFont(fontTex);
    
  cp5.addTextfield("stFileName")
     .setPosition(660,100)
     .setSize(100,18)
     .setFocus(true)
     .setFont(fontTex)
     .setColorValue(255)
     .setColorBackground(color(200, 200, 200))
      .getCaptionLabel().hide();
  
  cp5.addTextlabel("TitleRow1").setText("Id | Latitude | Longitud | Satellites | Hour:Minute:Second | Day/Month/Year")
  .setPosition(50,130).setColorValue(0).setFont(fontTex);
  
  cp5.addTextfield("St1")
   .setPosition(50,160)
   .setSize(400,17)
   .setFocus(true)
   .setFont(fontTex)
   .setColorValue(255)
   .setColorBackground(color(200, 200, 200))
   .getCaptionLabel().hide()
   .setVisible(false);
   
   cp5.addTextfield("St2")
   .setPosition(50,180)
   .setSize(400,17)
   .setFocus(true)
   .setFont(fontTex)
   .setColorValue(255)
   .setColorBackground(color(200, 200, 200))
   .getCaptionLabel().hide()
   .setVisible(false);

   cp5.addTextfield("St3")
   .setPosition(50,200)
   .setSize(400,17)
   .setFocus(true)
   .setFont(fontTex)
   .setColorValue(255)
   .setColorBackground(color(200, 200, 200))
   .getCaptionLabel().hide();

   cp5.addTextfield("St4")
   .setPosition(50,220)
   .setSize(400,17)
   .setFocus(true)
   .setFont(fontTex)
   .setColorValue(255)
   .setColorBackground(color(200, 200, 200))
   .getCaptionLabel().hide();   
   status = ((Textfield)cp5.getController("Status"));
   status.setValue("Welcome Geophonino-W configuration");
  
   cp5.addTextfield("St5")
   .setPosition(50,240)
   .setSize(400,17)
   .setFocus(true)
   .setFont(fontTex)
   .setColorValue(255)
   .setColorBackground(color(200, 200, 200))
   .getCaptionLabel().hide();   
   status = ((Textfield)cp5.getController("Status"));
   status.setValue("Welcome Geophonino-W configuration");
   
   cp5.addTextfield("St6")
   .setPosition(50,260)
   .setSize(400,17)
   .setFocus(true)
   .setFont(fontTex)
   .setColorValue(255)
   .setColorBackground(color(200, 200, 200))
   .getCaptionLabel().hide();   
   
  cp5.addTextlabel("TitleRow2").setText("Id | Latitude | Longitude | Satellites | Hour:Minute:Second | Day/Month/Year")
  .setPosition(500,130).setColorValue(0).setFont(fontTex);
  
  cp5.addTextfield("St7")
   .setPosition(500,160)
   .setSize(400,17)
   .setFocus(true)
   .setFont(fontTex)
   .setColorValue(255)
   .setColorBackground(color(200, 200, 200))
   .getCaptionLabel().hide();
   
   cp5.addTextfield("St8")
   .setPosition(500,180)
   .setSize(400,17)
   .setFocus(true)
   .setFont(fontTex)
   .setColorValue(255)
   .setColorBackground(color(200, 200, 200))
   .getCaptionLabel().hide();

   cp5.addTextfield("St9")
   .setPosition(500,200)
   .setSize(400,17)
   .setFocus(true)
   .setFont(fontTex)
   .setColorValue(255)
   .setColorBackground(color(200, 200, 200))
   .getCaptionLabel().hide();

   cp5.addTextfield("St10")
   .setPosition(500,220)
   .setSize(400,17)
   .setFocus(true)
   .setFont(fontTex)
   .setColorValue(255)
   .setColorBackground(color(200, 200, 200))
   .getCaptionLabel().hide();   
   status = ((Textfield)cp5.getController("Status"));
   status.setValue("Welcome Geophonino-W configuration");
  
   cp5.addTextfield("St11")
   .setPosition(500,240)
   .setSize(400,17)
   .setFocus(true)
   .setFont(fontTex)
   .setColorValue(255)
   .setColorBackground(color(200, 200, 200))
   .getCaptionLabel().hide();   
   status = ((Textfield)cp5.getController("Status"));
   status.setValue("Welcome Geophonino-W configuration");
   
   cp5.addTextfield("St12")
   .setPosition(500,260)
   .setSize(400,17)
   .setFocus(true)
   .setFont(fontTex)
   .setColorValue(255)
   .setColorBackground(color(200, 200, 200))
   .getCaptionLabel().hide();   
   
   cp5.addButton("Configure acquisition")
     .setValue(0)
     .setPosition(950,260) //260
     .setSize(100,14)
     .setId(9); 
   
   status = ((Textfield)cp5.getController("Status"));
   status.setValue("Welcome Geophonino-W configuration");
   
  ddlCom = cp5.addDropdownList("serial").setPosition(560, 75);
  for (int i = 0; i < Serial.list().length; i = i+1) {
    ddlCom.addItem(Serial.list()[i], i+1); 
  }
  if (Serial.list().length==0)
  {
     status.setValue("Error, COM Port not detected, connect Geophonino-W and press CONNECT... button.");
  }
  customizeDl(ddlCom);
   
  cp5.addTextlabel("ConfAcquisition").setText("3º Data acquisition configuration")
  .setPosition(50,290).setColorValue(0).setFont(fontTit)     ;
  
  cp5.addTextlabel("Comment").setText("Set configuration parameters and press SEND CONFIGURATION button")
  .setPosition(50,330).setColorValue(0).setFont(fontTex);
  
      
  cp5.addTextlabel("txtDura").setText("Duration (s): ")
  .setPosition(50,370).setColorValue(0).setFont(fontTex);
  
  cp5.addTextfield("dura")
     .setPosition(130,368)
     .setSize(100,18)
     .setFocus(true)
     .setFont(fontTex)
     .setColorValue(255)
     .setColorBackground(color(200, 200, 200))
     .setColorCaptionLabel(200)
     .getCaptionLabel().hide();
     
  cp5.addButton("Send configuration")
     .setValue(0)
     .setPosition(50,410)
     .setSize(100,14)
     .setId(3); 
     
  cp5.addButton("Get configuration")
     .setValue(0)
     .setPosition(180,260)
     .setSize(100,14)
     .setId(4)
     .setVisible(false);
      
    cp5.addTextlabel("txtGain").setText("Gain Ltb5:")
  .setPosition(365,405).setColorValue(0).setFont(fontTex); 
  
  ddlGainLtb = cp5.addDropdownList("gainLtb").setPosition(430, 420);
  ddlGainLtb.addItem("0", 0);
  ddlGainLtb.addItem("1", 1);
  ddlGainLtb.addItem("2", 2);  
  ddlGainLtb.addItem("5", 3);
  ddlGainLtb.addItem("10", 4);
  ddlGainLtb.addItem("20", 5);
  ddlGainLtb.addItem("50", 6);
  ddlGainLtb.addItem("100", 7);
  
  customizeDl(ddlGainLtb);
  
  cp5.addTextlabel("txtStartRecord").setText("Start record in:  HH         MM         SS")
  .setPosition(550,405).setColorValue(0).setFont(fontTex); 
  
 cp5.addTextfield("hour")
     .setPosition(660,405)
     .setSize(20,18)
     .setFocus(true)
     .setFont(fontTex)
     .setColorValue(255)
     .setColorBackground(color(200, 200, 200))
      .getCaptionLabel().hide();
 
 cp5.addTextfield("minute")
     .setPosition(710,405)
     .setSize(20,18)
     .setFocus(true)
     .setFont(fontTex)
     .setColorValue(255)
     .setColorBackground(color(200, 200, 200))
      .getCaptionLabel().hide();
      
 cp5.addTextfield("second")
     .setPosition(755,405)
     .setSize(20,18)
     .setFocus(true)
     .setFont(fontTex)
     .setColorValue(255)
     .setColorBackground(color(200, 200, 200))
      .getCaptionLabel().hide();      
 
  cp5.addTextlabel("txtSps").setText("Sampling Rate:")
  .setPosition(550,368).setColorValue(0).setFont(fontTex); 
  
  ddlSr = cp5.addDropdownList("ms/sps").setPosition(640, 385);
  ddlSr.addItem("10.0 ms/100 sps",0);
  ddlSr.addItem("4.0 ms/250 sps", 1);
  ddlSr.addItem("2.0 ms/500 sps", 2);
  customizeDl(ddlSr);
  
 cp5.addTextlabel("txtAmplification").setText("Amplification:")
  .setPosition(350,370).setColorValue(0).setFont(fontTex); 
  
  ddlGain = cp5.addDropdownList("x").setPosition(430, 385);
  ddlGain.addItem("1.0", 1);
  ddlGain.addItem("2.0", 2);
  ddlGain.addItem("4.0", 3);  
  customizeDl(ddlGain);
  
  cp5.addTextlabel("AcquireData").setText("4º Data Acquisition")
  .setPosition(50,460).setColorValue(0).setFont(fontTit)     ;
  
  cp5.addTextfield("St1rec")
   .setPosition(50,490)
   .setSize(400,17)
   .setFocus(true)
   .setFont(fontTex)
   .setColorValue(255)
   .setColorBackground(color(200, 200, 200))
   .getCaptionLabel().hide()
   .setVisible(false);
   
   cp5.addTextfield("St2rec")
   .setPosition(50,510)
   .setSize(400,17)
   .setFocus(true)
   .setFont(fontTex)
   .setColorValue(255)
   .setColorBackground(color(200, 200, 200))
   .getCaptionLabel().hide()
   .setVisible(false);

   cp5.addTextfield("St3rec")
   .setPosition(50,530)
   .setSize(400,17)
   .setFocus(true)
   .setFont(fontTex)
   .setColorValue(255)
   .setColorBackground(color(200, 200, 200))
   .getCaptionLabel().hide();

   cp5.addTextfield("St4rec")
   .setPosition(50,550)
   .setSize(400,17)
   .setFocus(true)
   .setFont(fontTex)
   .setColorValue(255)
   .setColorBackground(color(200, 200, 200))
   .getCaptionLabel().hide();   
   status = ((Textfield)cp5.getController("Status"));
   status.setValue("Welcome Geophonino-W configuration");
  
   cp5.addTextfield("St5rec")
   .setPosition(50,570)
   .setSize(400,17)
   .setFocus(true)
   .setFont(fontTex)
   .setColorValue(255)
   .setColorBackground(color(200, 200, 200))
   .getCaptionLabel().hide();   
   status = ((Textfield)cp5.getController("Status"));
   status.setValue("Welcome Geophonino-W configuration");
   
   cp5.addTextfield("St6rec")
   .setPosition(50,590)
   .setSize(400,17)
   .setFocus(true)
   .setFont(fontTex)
   .setColorValue(255)
   .setColorBackground(color(200, 200, 200))
   .getCaptionLabel().hide();  
    
   cp5.addTextfield("St7rec")
   .setPosition(500,490)
   .setSize(400,17)
   .setFocus(true)
   .setFont(fontTex)
   .setColorValue(255)
   .setColorBackground(color(200, 200, 200))
   .getCaptionLabel().hide();
   
   cp5.addTextfield("St8rec")
   .setPosition(500,510)
   .setSize(400,17)
   .setFocus(true)
   .setFont(fontTex)
   .setColorValue(255)
   .setColorBackground(color(200, 200, 200))
   .getCaptionLabel().hide();

   cp5.addTextfield("St9rec")
   .setPosition(500,530)
   .setSize(400,17)
   .setFocus(true)
   .setFont(fontTex)
   .setColorValue(255)
   .setColorBackground(color(200, 200, 200))
   .getCaptionLabel().hide();

   cp5.addTextfield("St10rec")
   .setPosition(500,550)
   .setSize(400,17)
   .setFocus(true)
   .setFont(fontTex)
   .setColorValue(255)
   .setColorBackground(color(200, 200, 200))
   .getCaptionLabel().hide();   
   status = ((Textfield)cp5.getController("Status"));
   status.setValue("Welcome Geophonino-W configuration");
  
   cp5.addTextfield("St11rec")
   .setPosition(500,570)
   .setSize(400,17)
   .setFocus(true)
   .setFont(fontTex)
   .setColorValue(255)
   .setColorBackground(color(200, 200, 200))
   .getCaptionLabel().hide();   
   status = ((Textfield)cp5.getController("Status"));
   status.setValue("Welcome Geophonino-W configuration");
   
   cp5.addTextfield("St12rec")
   .setPosition(500,590)
   .setSize(400,17)
   .setFocus(true)
   .setFont(fontTex)
   .setColorValue(255)
   .setColorBackground(color(200, 200, 200))
   .getCaptionLabel().hide();   
  
  textFont(fontTex); //It select font type for every field that don't define font type
  
}

void customizeDl(DropdownList ddl) {
  // Format DropDownList
  ddl.enableCollapse();
  ddl.setBackgroundColor(color(190));
  ddl.actAsPulldownMenu(true);
  ddl.setItemHeight(20);
  ddl.setBarHeight(15);
  ddl.captionLabel().style().marginTop = 3;
  ddl.captionLabel().style().marginLeft = 3;
  ddl.valueLabel().style().marginTop = 10;
  ddl.setColorBackground(color(200, 200, 200));
  ddl.setColorActive(color(0, 128));
  ddl.setColorLabel(color(0,0,0));
}


void draw() {
  background(myColorBackground);
  switch(state)
  {
    case(1)://1º Configure Connection
      rect(0,90,width,height-135);
      fill(150);
      for (int i = 1; i <= 12; i = i+1) {
        cp5.controller("St"+i).setVisible(false);
        cp5.controller("St"+i+"rec").setVisible(false);
      }
    break;
    case(2)://2º Get stations positions
      rect(0,0,width,90);
      fill(150);
      rect(0,290,width,150);
      fill(150);  
      
      for (int i = 1; i <= numActiveSt; i = i+1) {
        cp5.controller("St"+i).setVisible(true);
      }
    break;
    case(3): //3º Save GPS data
      rect(0,0,width,90);
      fill(150);
      rect(0,290,width,150);
      fill(150);  
    break;
    case (4): //4º GPS data saved. Data acquisition configuration mode.
        rect(0,0,width,290);
        fill(150);
        rect(0,440,width,150);
        fill(150);  
    break;
    case(5)://5º Acquiring data from sensor
       fill(150);
       rect(0,0,width,440);     
       rect(280,462,map(count-ini,0,duration,0,500),19);
       for (int i = 1; i <= numActiveSt; i = i+1) {
         cp5.controller("St"+i+"rec").setVisible(true);
       }
    break;
  }
}


void controlEvent(ControlEvent theEvent) 
{
  // DropdownList is of type ControlGroup.
  // A controlEvent will be triggered from inside the ControlGroup class.
  // therefore you need to check the originator of the Event with
  // if (theEvent.isGroup())
  // to avoid an error message thrown by controlP5.
  
  if (theEvent.isGroup()) 
  {
    // check if the Event was triggered from a ControlGroup
    println("event from group : "+theEvent.getGroup().getValue()+" from "+theEvent.getGroup());
  } 
  else if (theEvent.isController()) 
  {
    switch(theEvent.getController().getId()) 
    {
      
    case(1): //Connect with Geophonino-W in the seleccted port
         int portSel=int(ddlCom.getValue());//Get selected port
         numActiveSt= int(cp5.get(Textfield.class,"numSt").getText());
         println(portSel);
         if (portSel==0)
         {
           status.setValue("Error, COM Port not selected.");
         }
         else if (numActiveSt<0 | numActiveSt>12)
         {
           status.setValue("Error, select a number of active stations between 1 and 12.");
         }
         else
         {
           myPort = new Serial(this, Serial.list()[portSel-1], 57600);
           myPort.bufferUntil('\n');
           
           state=2; //When the connection is configured the state is changed
         } 
    break;  
    
    case(2)://Refresh port list
        println("Refresh port list");
        String[][]itemsddlCom= ddlCom.getListBoxItems();
        println(itemsddlCom.length);
        if(Serial.list().length>itemsddlCom.length)
        {
          for (int i = 0; i < Serial.list().length; i = i+1)
          {
                ddlCom.addItem(Serial.list()[i], i+1); 
          }
        }
        if (Serial.list().length==0)
        {
           status.setValue("Error, COM Port not tetected, connect Geophonino-W and press Refresh... button.");
        }
    break;//END Refresh port list
    
    case(3): //Send data acquisition configuration to Geophonino-W
      println("Send Configuration");
      
      String dura=cp5.get(Textfield.class,"dura").getText();
      duration= int(cp5.get(Textfield.class,"dura").getText());
     
      String hours=(cp5.get(Textfield.class,"hour").getText());
      String minutes=(cp5.get(Textfield.class,"minute").getText());
      String seconds=(cp5.get(Textfield.class,"second").getText());
      
      int srSel=int(ddlSr.getValue());
      int gainSel=int(ddlGain.getValue());
      int gainLtb=int(ddlGainLtb.getValue());
      int hour=int(hours);
      int minute=int(minutes);
      int second=int(seconds);
      
      if(duration<=0)
        status.setValue("You must enter a numeric value in duration");
      else
        if(gainSel<1)
          status.setValue("You must select amplification value");
        else
          if(srSel<0)
            status.setValue("You must select sample rate");
          else
            if(gainLtb<0)
              status.setValue("You must select LTB5 amplification");
            else
              if(hour<0|hour>24|minute<0|minute>59|second<0|second>59)
                status.setValue("You must enter a valid hour to start record");
              else //All parrameters are OK, send configuration to Geophonino-W already
              {
                command="conf_adq&Gain="+gainSel+"&tR="+dura+"&sR="+srSel+"&GLtb="+gainLtb+"&HH="+hours+"&MM="+minutes+"&SS="+seconds+"\n";
                myPort.write(command);
                println(command);
                state=5; //When data acquisition is configured the state is changed
                if (overwriteFile=="Y")
                {
                  status.setValue("Configuration sent successfully, file will be overwrited");
                  overwriteFile="N";
                }
                else
                  status.setValue("Configuration sent successfully");  
              }
    break; //END Send data acquisition configuration to Geophonino-W
    
    case(5): //Acquire data
      println("***************************Acquire data**********************");
      command="adq_now\n";
      println(command);
      status.setValue("Initializing data acquisition...");
      state=4; //When acquisition starts the state is changed
      count=0; 
    break;//END Acquire data Now
    
    case(6): //Not used yet
      
    break;
    
    case(7):
      println("7");
      state=2;
    break;  
    
    case(8)://STOP Data acquisition
      println("8");
      command="stop_acq\n";
      println(command);
      myPort.write(command);
      status.setValue("Data acquisition stoped by user...");
      state=4;
    break;      
    case(9): //Save GPS data to file
      println("9");
      stFileName= cp5.get(Textfield.class,"stFileName").getText();
      
      if (stFileName.equals(""))
        status.setValue("You must enter a stations file name");
      else
        if(stFileName.length()>20)
        {
          Textfield stFileNameField = ((Textfield)cp5.getController("stFileName"));
          stFileName=stFileName.substring(0,8);
          stFileNameField.setValue(stFileName);
          status.setValue("Stations file name have been shorted to 20 characters, press CONFIGURE ACQUISITION button again");
        }
       else //All parrameters are OK, send configuration to Geophonino-W already
        {
          state=3;
          println("state=3");
        }
         break;   
  }
  }
}

void delay(int ms) 
{ 
  int time = millis(); 
  while (millis () - time < ms); 
} 

/*It's called when data is available in the serial port*/
void serialEvent(Serial myPort) 
{
  val = myPort.readStringUntil('\n');
  println(val);
  if (val != null) 
  {
    //Trim whitespace and formatting characters (like carriage return)
    val = trim(val);  
    if (firstContact == false) //no connection yet
    {
      if (val.equals("A")) {
        myPort.clear();
        firstContact = true;
        myPort.write("Contacted\n");
        println("Contact");
        status.setValue("Connection established succesfully");
        myPort.clear();
        String command="load_st" + "&numSt="+numActiveSt+"\n";
        println(command);
        myPort.write(command);
      }
    }
    else //If we've already established contact, it keeps getting and parsing data 
    { 
      if(val.equals("save_st_ok"))
      {
        status.setValue("Stations positions saved succesfully by Geophonino-W");
        state=4; //When data acquisition is configured the state is changed
        println("state changed to 4, file station position was saved succesfully");
        pause();
      }
      if(val.equals("save_st_ko"))
      {
        status.setValue("Error saving stations positions");
        println("state changed to 3, file station position was saved succesfully");
        state=3; //Error saving gps data, send command again
        pause();
      }
        
      if(val.equals("conf_adq_filexist"))
      {
        status.setValue("Filename already exists, select other filename or press send configuration again for overwrite it");
        overwriteFile="Y";
        state=2;
      }
      if(val.equals("conf_adq_ok"))
        status.setValue("Configuration was received succesfully by Geophonino-W");
        
      if(val.equals("adq_now_ok"))
        status.setValue("Data acquisition inicialized by Geophonino-W");
      else
        if(val.indexOf("adq_now_ok")>-1)
        {
           status.setValue("Acquiring samples for second: " + val.substring(10,val.length()));
           count=int(val.substring(10,val.length())); //Update counter for progress bar
           if (count==duration)
           {
             status.setValue("Data file " + cp5.get(Textfield.class,"fileName").getText() + " generated suscesfully." );
           }
         }
      if(val.indexOf("sendF_size")>-1) 
       {
           println("sendF_size");
           tamFile=int(val.substring(10,val.length()));
           println(tamFile);
       }       
      if(val.equals("sendF_now")) 
       {
           println("sendF_nowRecibido");
           state=5;
           status.setValue("File " + cp5.get(Textfield.class,"fileName").getText() + " open succesfully");  
       }
       if (val.indexOf("StationStatus")>-1)
       {  // Set Station status text value
          // Message example-> StationStatusId4Gps data is not valid
          stationId=int(val.substring(15,17));
          if (stationId==0)
            stationId=int(val.substring(15,16));
          println("StStatR"+stationId);
          stationStatus=((Textfield)cp5.getController("St"+stationId));
          if (stationId<10)
            stationStatus.setValue(val.substring(16,val.length()));
          else
            stationStatus.setValue(val.substring(17,val.length()));
       }
       if (val.indexOf("StationData")>-1)
       {  //Set Station data values
           println(val);
          stationId=int(val.substring(11,13));
          if (stationId==0)
             stationId=int(val.substring(11,12));
          println("StDataR"+stationId);
          stationStatus=((Textfield)cp5.getController("St"+stationId));
          stationStatus.setValue(val.substring(11,val.length()));
          
       }
       //StationStatusId
       if (val.indexOf("StationAdqStatusId")>-1)
       {  //Set Station data values
          println("StationAdqStatusId");
          stationId=int(val.substring(18,20));
          println(stationId);
          if (stationId==0)
            stationId=int(val.substring(18,19));
          println("StStatusR"+stationId);
          stationStatus=((Textfield)cp5.getController("St"+stationId+"rec"));
          if (stationId<10)
            stationStatus.setValue(val.substring(19,val.length()));
          else
            stationStatus.setValue(val.substring(20,val.length()));
       }
       if(val.equals("save_st_now")) 
       {
           
         println("save_st_now");
         if (state==3)
         {
          command="save_st" + "&nomFileSt="+stFileName+"\n";
          myPort.write(command);
          status.setValue("Save positions command sent successfully");
         }    
       }
     } 
  }
}
