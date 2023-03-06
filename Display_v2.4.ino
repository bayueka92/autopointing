

#include <UTFT.h>
#include <UTFT_Geometry.h>
#include <tinyFAT.h>
#include <UTFT_tinyFAT.h>
#include <UTouch.h>
#include <UTFT_Buttons.h>
#include <EEPROM.h>
#include <TinyGPS.h>

//List of commands Compass available to serial communication
#define CMD        0x00  // define blank message to send to clear path
#define GET_VER    0x11  // Defines address of GET_VERSION
#define GET_ANG8   0x12  // Angle as single byte 0-255
#define GET_ANG16  0x13  // Angle as two bytes, high byte first 0-3600
#define GET_PITCH  0x14  // Pitch angle +/- 0-85deg
#define GET_ROLL   0x15  // Roll angle +/- 0-85deg
#define GET_MAG_R  0x21  // Raw magnetic data, 16bit signed: X high, X low, Y high, Y low, Z high, Z low
#define Get_ACC_R  0x22  // Raw accelerometer data, 16 bit signed: X high, X low, Y high, Y low, Z high, Z low
#define GET_ALL    0x23  // Address to get: angle high, angle low (0-3600), pitch, roll +/- 0-85deg

#define machinenumber "14coba44idwx"
#define machinePASS "A12345678B"
#define companyname "PT. ABCOBAtes"
#define companyphone "021-84319888"

// Declare which fonts we will be using
extern uint8_t SmallFont[]; //8x12 full
extern uint8_t BigFont[]; //16x16 full
extern uint8_t Ubuntu[]; //24x32 full
extern uint8_t UbuntuBold[]; //24x32 full
extern uint8_t SixteenSegment96x144Num[]; //96x144 Numeric
extern uint8_t SevenSegNumFont[]; // 32x50 Numeric
extern uint8_t SixteenSegment40x60[]; //40x60 Full

UTFT myGLCD(ITDB50, 38, 39, 40, 41);
UTFT_tinyFAT  myFiles(&myGLCD);
UTouch        myTouch(6,5,4,3,2);
UTFT_Buttons  myButtons(&myGLCD, &myTouch);
UTFT_Geometry geo(&myGLCD);


TinyGPS gps;

const int mot1CW = 8;
const int mot1CCW = 9;
const int mot2CW = 10;
const int mot2CCW = 11;
//const int mot3CW = 12;
//const int mot3CCW = 13;

const int limit1 = 46;
const int limit2 = 47;
const int limit3 = 48;

int max_x, max_y;
int x, y, val, tutupan, nilaitutup;
int sinyal=0, buf, index=0;
int ceksinyal;
double xmillis;
int sinyalbesar=0, sinyalbesar2=0;
double lamadelay;
int rangepoint=26, rangewaktu=20000, rangeerror=15;
int bataspoint=10;
double batasatas=0, batasbawah=0, bataselv=0; 
int  adjustfine=0;
int ulangpoint=1;
int showdisplay=0;
int passlimit=0;
boolean clearscreen=0;
boolean play=0, page=0;
boolean cekarah=0, arahmotor;
boolean gerakazimuth=0, gerakelevasi=0;
boolean flag, modem;
boolean manualatas, manualbawah, manualkanan, manualkiri;
boolean finish=0;
boolean fine=0;
boolean buka=0,tutup=1;
boolean tuning=0;
boolean GPSstat=0,COMPASSstat=0,MODEMstat=0;
boolean statusAz, statusEl;
char satelit[15];
char data[10];
char passwd[10];
char key;
float longGPS=0, latGPS=0, longSat=118; 
double A,B,C,D,E,F;
double degtorad = 0.0174532925, radtodeg = 57.2957795;
double curAzimuth, elevasi, curElevasi;
double xcurAzimuth, xcurElevasi;
double pointazimuth, pointElevasi;
unsigned long chars;
unsigned short sentences, failed;
byte highByte, lowByte;

void ResetOpen(){

  sinyal=0;
  index=0;
  sinyalbesar=0;
  batasatas=0;
  batasbawah=0; 
  adjustfine=0;
  ulangpoint=1;
  gerakazimuth=0;
  gerakelevasi=0;
  play=0; 
  page=0;
  cekarah=0;
  gerakazimuth=0;
  gerakelevasi=0;
  finish=0;
  fine=0;
  buka=0;
  tutup=1;
  tuning=0;
  statusEl=0;
  statusAz=0;
  curAzimuth=0;
  elevasi=0;
  curElevasi=0;
  xcurAzimuth=0; 
  xcurElevasi=0;
  pointazimuth=0; 
  pointElevasi=0;
  GPSstat=0;
  COMPASSstat=0;
  MODEMstat=0;
}

void PlayCheck(int x){
  if(x==0){
    myGLCD.setColor(255,0,0);
    myGLCD.drawCircle(400,240,110);
    myGLCD.setColor(0,0,0);
    myGLCD.fillCircle(400,240,110);
    myGLCD.setColor(0,255,0);
    geo.fillTriangle(480, 240, 350, 300, 350, 180);
    if(tutup==0){
      TutupAntenna();
      buka=0; tutup=1;
    }
  }
  else if(x==1){
    if(buka==0 && digitalRead(limit1)==0){
      //if(GPSstat==1 && COMPASSstat==1 && MODEMstat==1){      
      myGLCD.setColor(0,255,0);
      myGLCD.drawCircle(400,240,110);
      myGLCD.setColor(0,0,0);
      myGLCD.fillCircle(400,240,110);
      myGLCD.setFont(BigFont);
      myGLCD.setColor(0,0,0);
      myGLCD.print("Process", 360, 230);
      BukaAntenna();
      buka=1; tutup=0;
      //}
    }
    else{
      myGLCD.setColor(0,255,0);
      myGLCD.drawCircle(400,240,110);
      myGLCD.setColor(0,0,0);
      myGLCD.fillCircle(400,240,110);
      myGLCD.setFont(BigFont);
      myGLCD.setColor(0,0,0);
      myGLCD.print("Process", 360, 230);
      TutupAntenna();
      buka=0; tutup=1;
    }
    myGLCD.setColor(0,255,0);
    myGLCD.drawCircle(400,240,110);
    myGLCD.setColor(0,0,0);
    myGLCD.fillCircle(400,240,110);    
    myGLCD.setColor(255,0,0);
    geo.fillTriangle(340, 240, 470, 300, 470, 180);
  }  
}

void SiteLocation(){
  myGLCD.setFont(SmallFont);
  myGLCD.setColor(255,0,0);
  myGLCD.print("Longitude", 565, 30);
  myGLCD.setFont(Ubuntu);
  myGLCD.setColor(255,0,0);
  myGLCD.printNumF(longGPS, 2, 565, 44);
  if((longGPS/10)<1)
    myGLCD.print("  ",637,44);
  else if((longGPS/10)<10)
    myGLCD.print(" ",661,44);
  myGLCD.setFont(SmallFont);
  myGLCD.setColor(255,0,0);
  myGLCD.print("Latitude", 565, 78);
  myGLCD.setFont(Ubuntu);
  myGLCD.setColor(255,0,0);
  myGLCD.printNumF(latGPS, 2, 565, 92);
  if((latGPS/10)<1)
    myGLCD.print("  ",637,92);
  else if((latGPS/10)<10)
    myGLCD.print(" ",661,92);
  myGLCD.setFont(BigFont);
  myGLCD.setColor(255,255,255);
  myGLCD.drawRect(545,15,790,128);
  myGLCD.setBackColor(VGA_BLACK);
  myGLCD.setColor(255,255,255);
  myGLCD.print("Site Location", 555, 9);  
}

void Copyright(){
  myGLCD.setFont(SmallFont);
  myGLCD.setBackColor(VGA_BLACK);
  myGLCD.setColor(255,255,255);  // ver. cegah error
  myGLCD.print("Copyright(C) 2016 SaVii System v2.3.9", 10, 466);  
}

void SensorCheck(){
  myGLCD.setFont(SmallFont);
  myGLCD.setBackColor(VGA_BLACK);
  myGLCD.setColor(255,255,255);
  myGLCD.drawRect(327,385,790,470);
  myGLCD.setBackColor(VGA_BLACK);
  myGLCD.setColor(255,255,255);
  myGLCD.print("Peripheral Status", 337, 380);
  myGLCD.setFont(BigFont);
  myGLCD.setColor(155,255,55);
  myGLCD.print("Controller OK", 337, 410);
  
  if(longGPS==0 && latGPS==0){
    myGLCD.setColor(255,0,0);
    myGLCD.print("GPS Check!", 337, 435);
    GPSstat=0;
  }
  else{
    myGLCD.setColor(0,255,0);
    myGLCD.print("GPS OK    ", 337, 435);
    GPSstat=1;
  }
  
  if(curAzimuth<=0||curAzimuth>=5000){
    myGLCD.setColor(255,0,0);
    myGLCD.print("Compass Check!", 565, 410);
    COMPASSstat=0;
  }
  else{
    myGLCD.setColor(0,255,0);
    myGLCD.print("Compass OK    ", 565, 410);  
    COMPASSstat=1;
  }
  
  if(sinyal==0){
    myGLCD.setColor(255,0,0);
    myGLCD.print("Modem Check!", 565, 435);   
    MODEMstat=0;  
  }
  else{
    myGLCD.setColor(0,255,0);
    myGLCD.print("Modem OK     ", 565, 435);
    ceksinyal=1;    
    MODEMstat=1;
  }
  
}

void SatelitName(){  
  BacaEEPROM();
  BacaKompas();
  //myGLCD.setFont(Ubuntu);
  //myGLCD.setColor(0,0,255);
  //myGLCD.printNumI(curAzimuth, CENTER, 20);
  myGLCD.setFont(SmallFont);
  myGLCD.setColor(0,0,255);
  myGLCD.print("click to change", CENTER, 45);
  myGLCD.setFont(UbuntuBold);
  myGLCD.setColor(0,0,255);
  myGLCD.print(satelit, CENTER, 60);  
}

void KuatSinyal(){
  BacaSinyal();
  if(sinyal>sinyalbesar)
    sinyalbesar=sinyal;
  if(ceksinyal==0){
    if(sinyal>1){
      ceksinyal=1;  
    }
    else{
      myGLCD.setFont(BigFont);
      myGLCD.setColor(255,255,255);
      myGLCD.print("Modem Not", 581, 181);
      myGLCD.print("Connected", 581, 211);    
      myGLCD.print("Click to", 581, 241);    
      myGLCD.print("retry", 581, 271);
      //myGLCD.print("Failed",581,347);  // --> SQF nambah tulisan
    }
  }
  else if(ceksinyal==1){
    myGLCD.setFont(SixteenSegment96x144Num);
    myGLCD.setColor(255,0,0);
    if((sinyal/10)<1){
      myGLCD.print("0",581,181);
      myGLCD.printNumI(sinyal, 677, 181);      
    }
    else
      myGLCD.printNumI(sinyal, 581, 181);
    if(sinyal<31){
      myGLCD.setFont(BigFont);
      myGLCD.setColor(255,0,0);
      myGLCD.print("            ",581,327);
      myGLCD.print("Failed",581,327);
      }
    else{
      myGLCD.setFont(BigFont);
      myGLCD.setColor(0,255,0);
      myGLCD.print("Connected",581, 327); 
    }      
  }
  else if(ceksinyal==2){
    if(sinyal>1){
      ceksinyal=1;  
    }
    else{
      myGLCD.setFont(BigFont);
      myGLCD.setColor(255,255,255);
      myGLCD.print("Retriying", 581, 181);
      myGLCD.print(".........", 581, 211);    
      myGLCD.print("        ", 581, 241);    
      myGLCD.print("     ", 581, 271);
      Serial1.print('A');
      delay(5000);
    }  
  }
  
  myGLCD.setFont(BigFont);
  myGLCD.setColor(255,255,255);
  myGLCD.drawRect(545,146,790,352);
  myGLCD.setBackColor(VGA_BLACK);
  myGLCD.setColor(255,255,255);
  myGLCD.print("SQF", 555, 138);  
}

void ManualButton(){  
  if(manualkiri==0&&manualkanan==0){
    myGLCD.setColor(255,255,255);
    MotorAzimuthStop();
    myGLCD.drawRoundRect(40,251,134,345);
    geo.fillTriangle(60, 298, 114, 271, 114, 325);
    myGLCD.drawRoundRect(144,251,238,345);  
    geo.fillTriangle(164, 271, 164, 325, 218, 298);
  }
  else if(manualkiri==1){
    MotorAzimuthCCW();
    myGLCD.setColor(255,0,0);
    myGLCD.drawRoundRect(40,251,134,345);
    geo.fillTriangle(60, 298, 114, 271, 114, 325);
  }
  else if(manualkanan==1){
    MotorAzimuthCW();
    myGLCD.setColor(255,0,0);
    myGLCD.drawRoundRect(144,251,238,345);  
    geo.fillTriangle(164, 271, 164, 325, 218, 298);
  }
  
  
  if(manualbawah==0&&manualatas==0){
    myGLCD.setColor(255,255,255);
    MotorElevasiStop();
    myGLCD.drawRoundRect(40,355,134,449);  
    geo.fillTriangle(87, 429, 60, 375, 114, 375);
    myGLCD.drawRoundRect(144,351,238,449);  
    geo.fillTriangle(164, 429, 218, 429, 191, 375);    
  }
  else if(manualbawah==1){
    if(digitalRead(limit1)==0){
      MotorElevasiStop(); 
    }
    else{      
      MotorElevasiTutup();
    }
    myGLCD.setColor(255,0,0);
    myGLCD.drawRoundRect(40,355,134,449);  
    geo.fillTriangle(87, 429, 60, 375, 114, 375);
  }
  else if(manualatas==1){
    if(digitalRead(limit3)==0){
      MotorElevasiStop(); 
    }
    else{      
      MotorElevasiBuka();
    }
    myGLCD.setColor(255,0,0);
    myGLCD.drawRoundRect(144,351,238,449);  
    geo.fillTriangle(164, 429, 218, 429, 191, 375);
  }  
}

void TargetCurrent(){
  myGLCD.setFont(BigFont);
  myGLCD.setBackColor(VGA_BLACK);
  myGLCD.setColor(255,255,255);
  myGLCD.drawLine(139,20,139,220);
  myGLCD.drawLine(40,120,238,120);
  myGLCD.drawRect(40,20,238,221);
  myGLCD.setBackColor(VGA_BLACK);
  myGLCD.setColor(255,255,255);
  myGLCD.print("Target", 50, 15);
  myGLCD.print("Current", 50, 115);
  
  myGLCD.setFont(SmallFont);
  myGLCD.setColor(255,0,0);
  myGLCD.print("Elevasi", 50, 45);
  myGLCD.setFont(BigFont);
  myGLCD.setColor(255,0,0);
  myGLCD.printNumF(E, 1, 50, 60);
    if((E/10)<1)
    myGLCD.print("  ",98,160);
  else if((E/10)<10)
    myGLCD.print(" ",114,160);
  
  myGLCD.setFont(SmallFont);
  myGLCD.setColor(255,0,0);
  myGLCD.print("Azimuth", 149, 45);
  myGLCD.setFont(BigFont);
  myGLCD.setColor(255,0,0);
  myGLCD.printNumF(A, 1, 149, 60);
  if((A/10)<1)
    myGLCD.print("  ",197,160);
  else if((A/10)<10)
    myGLCD.print(" ",213,160);
  
  myGLCD.setFont(SmallFont);
  myGLCD.setColor(255,0,0);
  myGLCD.print("Elevasi", 50, 145);
  myGLCD.setFont(BigFont);
  myGLCD.setColor(255,0,0);
  myGLCD.printNumF(curElevasi, 1, 50, 160);
  if((curElevasi/10)<1)
    myGLCD.print("  ",98,160);
  else if((curElevasi/10)<10)
    myGLCD.print(" ",114,160);
  
  myGLCD.setFont(SmallFont);
  myGLCD.setColor(255,0,0);
  myGLCD.print("Azimuth", 149, 145);
  myGLCD.setFont(BigFont);
  myGLCD.setColor(255,0,0);
  myGLCD.printNumF((curAzimuth/10),0, 149, 160);
  if((curAzimuth/100)<1)
    myGLCD.print("  ",197,160);
  else if((curAzimuth/100)<10)
    myGLCD.print(" ",213,160);  
}

void PilihSatelit(){
  myGLCD.setFont(Ubuntu);
  myGLCD.setColor(255,255,255);
  myGLCD.print("BRISat", 100, 120);
  myGLCD.print("Palapa D", 100, 200);
  myGLCD.print("Telkom 3S", 100, 280);
  myGLCD.print("Measat 3", 500, 120);
  myGLCD.print("Thaicom", 500, 200);
  myGLCD.print("Apstar 7", 500, 280);
  myGLCD.setFont(UbuntuBold);
  myGLCD.setColor(0,255,0);
  myGLCD.print("OK", 700, 400);
   
}

void BacaEEPROM()
{
  val=EEPROM.read(0);
  if(val==1)
    {strcpy(satelit,"BRISat"); longSat=150.5;}
  else if(val==2)
    {strcpy(satelit,"Palapa D"); longSat=113;}
  else if(val==3)
    {strcpy(satelit,"Telkom 3S"); longSat=118;}
  else if(val==4)
    {strcpy(satelit,"Measat 3"); longSat=91.5;}
  else if(val==5)
    {strcpy(satelit,"Thaicom"); longSat=78.5;}
  else if(val==6)
    {strcpy(satelit,"Apstar 7"); longSat=76.5;}
  
}

void ManualDisplay(){  
  myGLCD.setColor(255,255,255);
  MotorAzimuthStop();
  myGLCD.drawRoundRect(30,10,770,70);
  myGLCD.setFont(UbuntuBold);
  myGLCD.setColor(0,0,255);
  myGLCD.print("Back", CENTER, 24);
    
  if(manualkiri==0&&manualkanan==0){
    myGLCD.setColor(255,255,255);
    MotorAzimuthStop();
    myGLCD.drawRoundRect(30,80,395,265);
    geo.fillTriangle(160, 173, 265, 120, 265, 226);
    myGLCD.drawRoundRect(405,80,770,265);  
    geo.fillTriangle(535, 120, 535, 226, 640, 173);
  }
  else if(manualkiri==1){
    MotorAzimuthCCW();
    myGLCD.setColor(255,0,0);
    myGLCD.drawRoundRect(30,80,395,265);
    geo.fillTriangle(160, 173, 265, 120, 265, 226);
  }
  else if(manualkanan==1){
    MotorAzimuthCW();
    myGLCD.setColor(255,0,0);
    myGLCD.drawRoundRect(405,80,770,265);  
    geo.fillTriangle(535, 120, 535, 226, 640, 173);
  }
  
  
  if(manualbawah==0&&manualatas==0){
    myGLCD.setColor(255,255,255);
    MotorElevasiStop();
    myGLCD.drawRoundRect(30,275,395,460);  
    geo.fillTriangle(213, 420, 160, 315, 266, 315);
    myGLCD.drawRoundRect(405,275,770,460);  
    geo.fillTriangle(588, 315, 535, 420, 641, 420);    
  }
  else if(manualbawah==1){
    if(digitalRead(limit1)==0){
      MotorElevasiStop(); 
    }
    else{      
      MotorElevasiTutup();
    }
    myGLCD.setColor(255,0,0);
    myGLCD.drawRoundRect(30,275,395,460);  
    geo.fillTriangle(213, 420, 160, 315, 266, 315);
  }
  else if(manualatas==1){
    if(digitalRead(limit3)==0){
      MotorElevasiStop(); 
    }
    else{      
      MotorElevasiBuka();
    }
    myGLCD.setColor(255,0,0);
    myGLCD.drawRoundRect(405,275,770,460);  
    geo.fillTriangle(588, 315, 535, 420, 641, 420);
  }
  if (myTouch.dataAvailable()){
      myTouch.read();
      x=myTouch.getX();
      y=myTouch.getY();
      if((y>=1) && (y<=60)){ // back kembali
        //myGLCD.print("MM", 50, 45);
        showdisplay=1;
        clearscreen=0;
      }
      if((y>=120) && (y<=200)){
        if((x>=30) && (x<=350)){
          myGLCD.print("Kiri ", 50, 45);
          if(manualkiri==0&&manualkanan==0)
            manualkiri=1;
          else
            manualkiri=0;
        }
        if((x>=450) && (x<=770)){
          myGLCD.print("Kanan", 50, 45);
          if(manualkanan==0&&manualkiri==0)
            manualkanan=1;
          else
            manualkanan=0;
        }
      }
      if((y>=300) && (y<=460)){
        if((x>=30) && (x<=350)){
          myGLCD.print("Tutup ", 50, 45);
          if(manualbawah==0&&manualatas==0)
            manualbawah=1;
          else
            manualbawah=0;
        }
        if((x>=450) && (x<=770)){
          myGLCD.print("Buka", 50, 45);
          if(manualatas==0&&manualbawah==0)
            manualatas=1;
          else
            manualatas=0;
        }
      }  
  }  
}

void PasswordDisplay(){  
  myGLCD.setColor(255,255,255);
  MotorAzimuthStop();
  //myGLCD.drawRoundRect(30,10,770,70);
  myGLCD.setFont(SixteenSegment40x60);
  myGLCD.setColor(255,255,255);
  
  myGLCD.drawRoundRect(15,350,215,455); //OK
  myGLCD.print("OK", 75, 372);
  myGLCD.drawRoundRect(220,350,340,455); //Clear
  myGLCD.setColor(255,0,0);
  myGLCD.print("C", 260, 372);
  
  myGLCD.setColor(255,255,255);
  myGLCD.drawRoundRect(345,20,450,125); // 1 
  myGLCD.print("1", 377, 42); //+32,+22
  myGLCD.drawRoundRect(345,130,450,235); // 4 
  myGLCD.print("4", 377, 152);
  myGLCD.drawRoundRect(345,240,450,345); // 7 
  myGLCD.print("7", 377, 262);
  myGLCD.drawRoundRect(345,350,450,455); // * 
  myGLCD.print("*", 377, 372);
  
  myGLCD.drawRoundRect(455,20,560,125); // 2  
  myGLCD.print("2", 487, 42);
  myGLCD.drawRoundRect(455,130,560,235); // 5 
  myGLCD.print("5", 487, 152);
  myGLCD.drawRoundRect(455,240,560,345); // 8 
  myGLCD.print("8", 487, 262);
  myGLCD.drawRoundRect(455,350,560,455); // 0 
  myGLCD.print("0", 487, 372);
  
  myGLCD.drawRoundRect(565,20,670,125); // 3  
  myGLCD.print("3", 597, 42); 
  myGLCD.drawRoundRect(565,130,670,235); // 6  
  myGLCD.print("6", 597, 152);
  myGLCD.drawRoundRect(565,240,670,345); // 9  
  myGLCD.print("9", 597, 262);
  myGLCD.drawRoundRect(565,350,670,455); // .  
  myGLCD.print(".", 597, 372);
  
  myGLCD.drawRoundRect(675,20,780,125); // A 
  myGLCD.print("A", 707, 42);  
  myGLCD.drawRoundRect(675,130,780,235); // B 
  myGLCD.print("B", 707, 152); 
  myGLCD.drawRoundRect(675,240,780,345); // C 
  myGLCD.print("C", 707, 262); 
  myGLCD.drawRoundRect(675,350,780,455); // D 
  myGLCD.print("D", 707, 372); 
  
  
  
     
  if(passlimit==0){
     for(passlimit=0;passlimit<10;passlimit++)passwd[passlimit]='*'; 
     myGLCD.setFont(UbuntuBold);
     myGLCD.setColor(255,0,0);
     myGLCD.print(passwd, 50, 150);
     passlimit=0;

      myGLCD.setFont(BigFont);
      myGLCD.setColor(0,255,0);
      myGLCD.print("Masukkan Password", 50, 45);
      myGLCD.setFont(SmallFont);
      myGLCD.setColor(255,255,255);
      myGLCD.print("Machine Number: ", 50, 260);
      myGLCD.print(machinenumber, 175, 260);
      myGLCD.print(companyname, 50, 277);
      myGLCD.print(companyphone, 50, 294);
     
  }
  
  if (myTouch.dataAvailable()){
    myTouch.read();
    x=myTouch.getX();
    y=myTouch.getY();
    if((x>=20) && (x<=195)){
      if(!strcmp(passwd,machinePASS)){
        for(passlimit=0;passlimit<10;passlimit++)passwd[passlimit]='*'; 
        myGLCD.setFont(UbuntuBold);
        myGLCD.setColor(255,0,0);
        myGLCD.print(passwd, 50, 150);
        passlimit=0;
        showdisplay=1;
        clearscreen=0;
      }
      else{
        for(passlimit=0;passlimit<10;passlimit++)passwd[passlimit]='*'; 
        myGLCD.setFont(UbuntuBold);
        myGLCD.setColor(255,0,0);
        myGLCD.print(passwd, 50, 150);
        passlimit=0;
        
        myGLCD.setFont(BigFont);
        myGLCD.setColor(0,255,0);
        for(int i=0;i<3;i++){
          myGLCD.print("Password Salah!!", 50, 210);
          delay(400);
          myGLCD.print("                ", 50, 210);
          delay(400);
        }
      }
    }
    if((x>=250) && (x<=320)){
      //myGLCD.print("C", 50, 45);
      for(passlimit=0;passlimit<10;passlimit++)passwd[passlimit]='*';
      myGLCD.setFont(UbuntuBold);
      myGLCD.setColor(255,0,0);
      myGLCD.print(passwd, 50, 150);
      passlimit=0;      
    }
    if((x>=345) && (x<=425)){
      if((y>=20) && (y<=100)){
        passwd[passlimit]='1';
      }
      if((y>=155) && (y<=210)){
        passwd[passlimit]='4';
      }
      if((y>=265) && (y<=320)){
        passwd[passlimit]='7';
      }
      if((y>=375) && (y<=430)){
        passwd[passlimit]='*';
      }     
      if(passlimit<9)passlimit=passlimit+1;   
    }
    if((x>=475) && (x<=540)){
      if((y>=20) && (y<=100)){
        passwd[passlimit]='2';
      }
      if((y>=155) && (y<=210)){
        passwd[passlimit]='5';
      }
      if((y>=265) && (y<=320)){
        passwd[passlimit]='8';
      }
      if((y>=375) && (y<=430)){
        passwd[passlimit]='0';
      }
      if(passlimit<9)passlimit=passlimit+1;      
    }    
    
    if((x>=585) && (x<=650)){
      if((y>=20) && (y<=100)){
        passwd[passlimit]='3';
      }
      if((y>=155) && (y<=210)){
        passwd[passlimit]='6';
      }
      if((y>=265) && (y<=320)){
        passwd[passlimit]='9';
      }
      if((y>=375) && (y<=430)){
        passwd[passlimit]='#';
      }     
      if(passlimit<9)passlimit=passlimit+1;   
    }    
    if((x>=695) && (x<=760)){
      if((y>=20) && (y<=100)){
        passwd[passlimit]='A';
      }
      if((y>=155) && (y<=210)){
        passwd[passlimit]='B';
      }
      if((y>=265) && (y<=320)){
        passwd[passlimit]='C';
      }
      if((y>=375) && (y<=430)){
        passwd[passlimit]='D';
      }
      if(passlimit<9)passlimit=passlimit+1;      
    }    
    
     myGLCD.setFont(UbuntuBold);
     myGLCD.setColor(255,0,0);
     myGLCD.print(passwd, 50, 150);
     myGLCD.setFont(BigFont);
     myGLCD.setColor(0,255,0);
     myGLCD.print("Masukkan Password", 50, 45);
     myGLCD.setFont(SmallFont);
     myGLCD.setColor(255,255,255);
     myGLCD.print("Machine Number: ", 50, 260);
     myGLCD.print(machinenumber, 175, 260);
     myGLCD.print(companyname, 50, 277);
     myGLCD.print(companyphone, 50, 294);
     delay(100);
     //delay(10);
  }  
  
}


void StartSoftware(){  
  myGLCD.setFont(UbuntuBold);
  myGLCD.setColor(255,255,255);
  myGLCD.print("Kontroller Parabola Otomatis", CENTER, 24);
  
  myGLCD.setColor(255,255,255);
  myGLCD.drawRoundRect(30,80,770,265);  
  myGLCD.drawRoundRect(30,275,395,460);
  myGLCD.drawRoundRect(405,275,770,460);
  
  myGLCD.setFont(SixteenSegment40x60);
  myGLCD.setColor(0,125,20);
  myGLCD.print("START OTOMATIS", 120, 142);
  myGLCD.print("MANUAL", 92, 337);
  myGLCD.print("SETTING", 447, 337);
  myGLCD.setFont(SmallFont);
  myGLCD.setColor(255,255,255);
  //myGLCD.print("Elevasi", 50, 45);
  
  if (myTouch.dataAvailable()){
      myTouch.read();
      x=myTouch.getX();
      y=myTouch.getY();
      if((y>=80) && (y<=200)){ // Start Auto
        myGLCD.print("Start", 50, 45);
        //showdisplay=2;
        //clearscreen=0;
      }
      if((y>=300) && (y<=460)){
        if((x>=30) && (x<=350)){ //Start Manual
          //myGLCD.print("Manu ", 50, 45);
          showdisplay=2;
          clearscreen=0;
        }
        if((x>=450) && (x<=770)){ //Start Setting
          myGLCD.print("Setting ", 50, 45);
          //showdisplay=1;
          //clearscreen=0;
        }
      }  
  } 
}

void MotorAzimuthCCW()
{
  digitalWrite(10, HIGH);
  digitalWrite(11, LOW);  
}

void MotorAzimuthCW()
{
  digitalWrite(10, LOW);
  digitalWrite(11, HIGH);   
}

void MotorAzimuthStop()
{
  digitalWrite(10, HIGH);
  digitalWrite(11, HIGH);  
}

void MotorElevasiTutup()
{
  digitalWrite(9, HIGH);
  digitalWrite(8, LOW);   
}

void MotorElevasiBuka()
{
  digitalWrite(9, LOW);
  digitalWrite(8, HIGH);   
}

void MotorElevasiStop()
{
  digitalWrite(9, HIGH);
  digitalWrite(8, HIGH);   
}


void HitungAzimuth()
{
  C = sin(latGPS*degtorad);
  B = tan(degtorad*(abs(longGPS-longSat)));
  A = abs(radtodeg*(atan(B/C)));     
  
  if(latGPS>0)
  {
    if(longGPS < longSat)
      A = 180 - A; 
    else
      A = 180 + A;
  }
  else
  {
    if(longGPS < longSat)
      A = A; 
    else
      A = 360 - A;
  }
  
  
  ///////////////////////////////di coba data dummy
  //A=50;
  
  if(cekarah==0 && A>=0){
    if((A-(curAzimuth/10))>0&&(A-(curAzimuth/10))<180){
      arahmotor=0;
    }
    else if(A-(curAzimuth/10)<0&&(A-(curAzimuth/10))>-180){
      arahmotor=1;   
    }
    else if(A-(curAzimuth/10)<0&&(A-(curAzimuth/10))<-180){
      arahmotor=0;
    }
    else if(A-(curAzimuth/10)>0&&(A-(curAzimuth/10))>180){
      arahmotor=1;
    }
  } 
}

void HitungElevasi()
{
  D = (cos(latGPS*degtorad)*cos(degtorad*(abs(longGPS-longSat))))-0.151;
  
  F = sqrt(1-pow((cos(latGPS*degtorad)*cos(degtorad*(abs(longGPS-longSat)))),2));
  E = radtodeg*(atan(D/F));  
  
  if(E>81)E=81; 
  ///////////////////////////////di coba data dummy
  //E=60;
}

void BacaKompas() 
{
  Serial3.write(GET_ANG16);
  delay(200);
  if(Serial3.available() > 0 ) {
    highByte= Serial3.read();
    lowByte= Serial3.read();
  }
  
  Serial3.write(GET_PITCH);
  delay(200);
  if(Serial3.available() > 0 ) {
    curElevasi = Serial3.read();
    curElevasi = curElevasi + 17;
    if(curElevasi>=255){
      curElevasi = curElevasi - 255;      
    }
  }
  
  curAzimuth=(highByte<<8)+lowByte;
  //curAzimuth=(360*kompas)/255;
  //curElevasi=(360*elevasi)/255;
  
  //myGLCD.setFont(Ubuntu);
  //myGLCD.setColor(0,0,255);
  //myGLCD.printNumI(arahmotor, CENTER, 20);
}

void BacaGPS()
{
  bool newData = false;
  unsigned long chars;
  unsigned short sentences, failed;

    // For one second we parse GPS data and report some key values
  for (unsigned long start = millis(); millis() - start < 1000;)
  {
    while (Serial2.available())
    {
      char c = Serial2.read();
      // Serial.write(c); // uncomment this line if you want to see the GPS data flowing
      if (gps.encode(c)) // Did a new valid sentence come in?
        newData = true;
    }
  }

  if (newData)
  {
    float flat, flon;
    unsigned long age;
    gps.f_get_position(&flat, &flon, &age);
    //Serial.print("LAT=");
    flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat, 6;
    latGPS = flat; 
    //Serial.print(latGPS);
    //Serial.print(" LON=");
    flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon, 6;
    longGPS = flon;
    //Serial.print(longGPS);
  }
  
  gps.stats(&chars, &sentences, &failed);
}

void BacaSinyal()
{
  while(Serial1.available()) {
    buf = Serial1.read();
    Serial.write(buf);
    Serial.print(";  ");
    Serial.print(buf);
    Serial.println(";");
    if(flag){
      data[index++] = buf;
    }
    if(buf== '%'){
      sinyal = atoi(data);
      index = 0;
      flag = true;
    }
    if(buf== 'C'){
      index = 0;
      modem = 1;
    }
    if(buf== 'F'){
      index = 0;
      modem = 0;
    }
  }      
}

void CekBatas(){
  BacaKompas();
  pointazimuth = curAzimuth/10;
  if((pointazimuth+rangepoint)>360){
    batasatas=rangepoint-(360-pointazimuth);
    batasbawah=pointazimuth-rangepoint;  
  }
  else if((pointazimuth-rangepoint)<0){
    batasatas=pointazimuth+rangepoint;
    batasbawah=360-(rangepoint-pointazimuth);
  }
  else{
    batasatas=pointazimuth+rangepoint;
    batasbawah=pointazimuth-rangepoint;  
  }  
}

////// edit error
void CekError(){
  BacaKompas();
  if(curAzimuth<=0 || curAzimuth>=5000 || ulangpoint>4){
    MotorAzimuthStop();
      delay(60000);
      delay(60000);
      delay(60000);
      delay(60000);
      delay(60000);  
  }
  else if(curAzimuth>=((batasatas*10)+(rangeerror*10))){
    MotorAzimuthStop();
      delay(60000);
      delay(60000);
      delay(60000);
      delay(60000);
      delay(60000);  
  }
  else if(curAzimuth<=((batasbawah*10)-(rangeerror*10))){
    MotorAzimuthStop();
      delay(60000);
      delay(60000);
      delay(60000);
      delay(60000);
      delay(60000);  
  }
}

void BukaAntenna()
{
  arahmotor=0;
  HitungAzimuth();
  HitungElevasi();
  cekarah=0;
  myGLCD.setColor(255,255,255);
  myGLCD.setFont(SmallFont);
  MotorElevasiBuka();
  delay(70000);
  BacaKompas();TargetCurrent(); 
  nilaitutup = curAzimuth/20;
  EEPROM.write(1,nilaitutup);
  while(curElevasi<0||curElevasi>=10){
    BacaKompas();TargetCurrent();
    MotorElevasiBuka();
    delay(100);
  }
  HitungAzimuth();
  HitungElevasi();
  cekarah=1;
  xcurElevasi = curElevasi;
  xcurAzimuth = curAzimuth;
  MotorElevasiStop();
  delay(200);
  if(arahmotor==0){    
    //delay(lamadelay);   
    while(statusEl==0||statusAz==0){
      BacaKompas();TargetCurrent();
      if(statusAz==0){MotorAzimuthCW();  delay(100);}
      if(statusEl==0){MotorElevasiBuka();  delay(100);}
      BacaKompas();TargetCurrent();
      if(curElevasi<=(E+12)&&curElevasi>=(E+5)){
      //if(curElevasi<=(E+5)&&curElevasi>=(E-2)){     /// --> asli nya // dirubah di jakarta 14012016
        MotorElevasiStop();
        delay(100);
        //break;
        statusEl=1;
      }
      if(curAzimuth<=((A*10)+20)&&curAzimuth>=((A*10)-20)){
        MotorAzimuthStop();
        delay(100);
        //break;
        statusAz=1;
      } 
    }
        
  }
  else if(arahmotor==1){
    //delay(lamadelay); 
    while(statusEl==0||statusAz==0){
      BacaKompas();TargetCurrent();
      if(statusAz==0){MotorAzimuthCCW();  delay(100);}
      if(statusEl==0){MotorElevasiBuka();  delay(100);}
      BacaKompas();TargetCurrent();
      if(curElevasi<=(E+12)&&curElevasi>=(E+5)){
      //if(curElevasi<=(E+5)&&curElevasi>=(E-2)){  /// --> asli nya // dirubah di jakarta 14012016
        MotorElevasiStop();
        delay(100);
        //break;
        statusEl=1;
      }
      if(curAzimuth<=((A*10)+20)&&curAzimuth>=((A*10)-20)){
        MotorAzimuthStop();
        delay(100);
        //break;
        statusAz=1;
      }
      //////   ketahuan error 100416 CekError();  /// error
    }
         
  }
  //MotorElevasiBuka();
  //delay(200);
  
  /*while(1){
    MotorElevasiBuka();
    delay(1);
    BacaKompas();TargetCurrent();
    if(curElevasi<=(E+5)&&curElevasi>=(E-2)){
      MotorElevasiStop();
      delay(100);
      break;
    }  
  }*/
  finish=0;
  adjustfine=0;  
  ulangpoint=0;
  sinyalbesar=0;
  //while(finish==0&&ulangpoint<5){
    KuatSinyal();
    if(sinyal>=75){finish=1; MotorElevasiStop(); MotorAzimuthStop();  delay(200); }
    if(finish==0&&adjustfine==0){
    CekBatas();    
    while(adjustfine==0&&finish==0&&ulangpoint<4){  
      /// cek CW
      while(adjustfine==0&&finish==0){
        if(arahmotor=0&&ulangpoint>=2){   ///// --> Jakarta 14012016 Nambah batas klo berhenti di terakhir di posisi start
          MotorAzimuthStop();  
          delay(200);
        }
        else{
          MotorAzimuthCW();
          delay(1); 
     
          BacaKompas();TargetCurrent();
          if(curAzimuth<=((batasatas*10)+25)&&curAzimuth>=((batasatas*10)-30)){
            MotorAzimuthStop();  
            delay(200); 
            break;     
          }
        }
        CekError();  /// error
        KuatSinyal();
        //if(sinyal>=75){finish=1; MotorElevasiStop(); MotorAzimuthStop();  delay(200); break;}
        if(sinyal>=30){adjustfine=1;  MotorElevasiStop(); MotorAzimuthStop();  delay(200);  break;}
        //if(sinyal>=60){adjustfine=2;  MotorElevasiStop(); MotorAzimuthStop();  delay(200);  break;}   
      }
      MotorElevasiStop();
      MotorAzimuthStop();
      delay(200);
      
      
      //// Elevasi naik
      MotorElevasiStop();
      MotorAzimuthStop();
      delay(2000);
      BacaKompas();TargetCurrent();
      if(curElevasi>=80)
        bataselv=curElevasi+0.8;//0.8 //// nambah elevasi
      else if(curElevasi<80)
        bataselv=curElevasi+1.2;//1.2
      while(adjustfine==0&&finish==0){
        
        if(arahmotor=0&&ulangpoint>=2){   ///// --> Jakarta 14012016 Nambah batas klo berhenti di terakhir di posisi start
          MotorElevasiStop();  
          delay(200);
        }
        else{
          BacaKompas();TargetCurrent();
          if(digitalRead(limit3)==0){
            MotorElevasiStop();  
            delay(200); 
            //adjustfine=5;
            break;
          }  
          if(curElevasi>=bataselv){
            MotorElevasiStop();  
            delay(200); 
            break;
          }  
          MotorElevasiBuka();
          delay(1);
       
          BacaKompas();TargetCurrent();
          if(digitalRead(limit3)==0){
            MotorElevasiStop();  
            delay(200); 
            //adjustfine=5;
            break;
          }      
          if(curElevasi>=bataselv){
            MotorElevasiStop();  
            delay(200); 
            break;
          }
        }  
      }
      
      MotorElevasiStop();
      MotorAzimuthStop();
      delay(200);
     
     //// Cek CCW
      //CekBatas();   
      while(adjustfine==0&&finish==0){
        if(arahmotor=1&&ulangpoint>=2){   ///// --> Jakarta 14012016 Nambah batas klo berhenti di terakhir di posisi start
          MotorAzimuthStop();  
          delay(200);
        }
        else{
          MotorAzimuthCCW();
          delay(1); 
        
          BacaKompas();TargetCurrent();
          if(curAzimuth<=((batasbawah*10)+25)&&curAzimuth>=((batasbawah*10)-30)){
            MotorAzimuthStop();  
            delay(200); 
            break;     
          }
        }
        CekError(); /// error
        KuatSinyal();
        //if(sinyal>=75){finish=1; MotorElevasiStop(); MotorAzimuthStop();  delay(200); break;}
        if(sinyal>=30){adjustfine=3;  MotorElevasiStop(); MotorAzimuthStop();  delay(200);  break;}
        //if(sinyal>=50){adjustfine=4;  MotorElevasiStop(); MotorAzimuthStop();  delay(200);  break;}   
      }
      
      
      MotorElevasiStop();
      MotorAzimuthStop();
      delay(200);
      
      //// Elevasi naik
      
      MotorElevasiStop();
      MotorAzimuthStop();
      delay(2000);
      BacaKompas();TargetCurrent();
      if(curElevasi>=80)
        bataselv=curElevasi+1.5;  //0.8    /// nambah elevasi
      else if(curElevasi<80)
        bataselv=curElevasi+2.1;  //1.2
      while(adjustfine==0&&finish==0){
          
        if(arahmotor=1&&ulangpoint>=2){   ///// --> Jakarta 14012016 Nambah batas klo berhenti di terakhir di posisi start
          MotorElevasiStop();  
          delay(200);
        }
        else{
          BacaKompas();TargetCurrent();
          if(digitalRead(limit3)==0){
            MotorElevasiStop();  
            delay(200); 
            break;
          }        
          if(curElevasi>=bataselv){
            MotorElevasiStop();  
            delay(200); 
            break;
          }  
          if(arahmotor=1&&ulangpoint>=2){   ///// --> Jakarta 14012016 Nambah batas klo berhenti di terakhir di posisi start
            MotorElevasiStop();  
            delay(200);
          }
          MotorElevasiBuka();
          delay(1);
        
          BacaKompas();TargetCurrent();
          if(digitalRead(limit3)==0){
            MotorElevasiStop();  
            delay(200); 
            //adjustfine=5;
            break;
          }  
          if(curElevasi>=bataselv){
            MotorElevasiStop();  
            delay(200); 
            break;
          }
        }  
      }
      
      MotorElevasiStop();
      MotorAzimuthStop();
      delay(200);
      ulangpoint=ulangpoint+1;
    }
    }
    
    CekError();  //error
    
    
  //}
   //if(adjustfine==1){
    while(finish==0){
      if(adjustfine==1)MotorAzimuthCW();  else if(adjustfine==3)MotorAzimuthCCW();
      delay(1);
      BacaKompas();TargetCurrent();
      KuatSinyal();
      CekError(); /// error
      if(sinyal>=75){
        sinyalbesar=75;
        MotorElevasiStop(); 
        MotorAzimuthStop();  
        delay(200); 
        while(finish==0){
          if(adjustfine==1)MotorAzimuthCW();  else if(adjustfine==3)MotorAzimuthCCW();
          delay(1);
          BacaKompas();TargetCurrent();
          KuatSinyal();
          CekError(); /// error
          if(sinyal>sinyalbesar)
            sinyalbesar=sinyal;
          if(sinyal<73){    
            MotorAzimuthStop();  
            delay(200); 
            while(sinyal<=(sinyalbesar-3)){
              if(adjustfine==1)MotorAzimuthCCW();  else if(adjustfine==3)MotorAzimuthCW();
              delay(1);
              KuatSinyal();
              CekError(); /// error
            }
            MotorAzimuthStop();
            delay(200);
            finish=1; 
          }
        } 
        break;
      }
      if(sinyal>sinyalbesar)
        sinyalbesar=sinyal;
      if(sinyal<=(sinyalbesar-20)){
      //if(sinyal<=25){  /// --> ini aslinya // di rubah di jakarta 14012016
        MotorAzimuthStop();  
        delay(200); 
        while(sinyal<=(sinyalbesar-4)){
          if(adjustfine==1)MotorAzimuthCCW();  else if(adjustfine==3)MotorAzimuthCW();
          delay(1);
          KuatSinyal();
          CekError(); /// error
        }
        MotorAzimuthStop();
        MotorElevasiStop();
        delay(200);
      
    
        
        ///
        sinyalbesar=0;   
        while(finish==0){
          BacaKompas();TargetCurrent();
          KuatSinyal();
          if(sinyal>=77){
            finish=1;
            MotorElevasiStop(); 
            MotorAzimuthStop();  
            delay(200);
            break;
          }
          if(sinyal>sinyalbesar)  //baru
            sinyalbesar=sinyal; //baru
          MotorElevasiBuka();
          delay(100);
          MotorElevasiStop();  delay(1);
          
          ////edited 02/04/16
          
          KuatSinyal();/// nambah
          //sinyalbesar=sinyal; //// nambah
          if(sinyal<25  || digitalRead(limit3)==0){
            MotorElevasiStop();  
            delay(100); 
            sinyalbesar2=sinyalbesar;
            while(sinyal<=(sinyalbesar-6)){
              MotorElevasiTutup();
              delay(100);
              MotorElevasiStop();  delay(1);
              KuatSinyal();             
              BacaKompas();TargetCurrent();
              if(curElevasi<=(E-10)){
                while(curElevasi>=(E+10)|| digitalRead(limit3)==0){
                  MotorElevasiBuka();
                  delay(100);
                  MotorElevasiStop();  delay(1);
                  KuatSinyal();           
                  if(sinyal>sinyalbesar2)  //baru
                  sinyalbesar2=sinyal; //baru   
                  BacaKompas();TargetCurrent();
                }
                MotorElevasiStop();
                delay(200);
                while(sinyal<=(sinyalbesar2-2)){
                  MotorElevasiTutup();
                  delay(50);
                  MotorElevasiStop();  delay(1);
                  KuatSinyal();
                  if(sinyal>=75){
                    finish=1;  
                    MotorElevasiStop();
                    MotorAzimuthStop();
                    delay(200);
                    while(curElevasi>=(E+10)|| digitalRead(limit3)==0){
                      MotorElevasiBuka();
                      delay(50);
                      MotorElevasiStop();  delay(1);
                      KuatSinyal();           
                      if(sinyal>sinyalbesar2)  //baru
                      sinyalbesar2=sinyal; //baru   
                      BacaKompas();TargetCurrent();
                      if(sinyal>=80){finish=1;  MotorElevasiStop();  MotorAzimuthStop();  delay(200);  break;}
                    }
                    MotorElevasiStop();
                    delay(200);
                    KuatSinyal();
                    while(sinyal<=(sinyalbesar2-2)){
                      MotorElevasiTutup();
                      delay(50);
                      MotorElevasiStop();  delay(1);
                      KuatSinyal();
                      if(sinyal>=75){  
                        finish=1;  
                        MotorElevasiStop();  MotorAzimuthStop();  delay(200);  
                        KuatSinyal();
                        if(sinyal<40){
                          while(sinyal<55 || digitalRead(limit3)==0){
                            MotorElevasiBuka();
                            delay(50);
                            MotorElevasiStop();  delay(1);
                            KuatSinyal();
                          }
                          MotorElevasiStop();  MotorAzimuthStop();  delay(200);
                          KuatSinyal();
                          if(sinyal<40){
                            while(sinyal<45){
                              MotorElevasiTutup();
                              delay(50);
                              MotorElevasiStop();  delay(1);
                              KuatSinyal();
                            }
                          }
                          MotorElevasiStop();  MotorAzimuthStop();  delay(200);
                        MotorElevasiTutup(); delay(100);MotorElevasiStop();  MotorAzimuthStop();  delay(200);}
                        break;}
                    }
                    KuatSinyal();
                    if(sinyalbesar2>=40){
                      finish=1;
                      if(sinyal<40){
                          while(sinyal<55 || digitalRead(limit3)==1){
                            MotorElevasiBuka();
                            delay(50);
                            MotorElevasiStop();  delay(1);
                            KuatSinyal();
                          }
                          MotorElevasiStop();  MotorAzimuthStop();  delay(200);
                          KuatSinyal();
                          if(sinyal<40){
                            while(sinyal<45){
                              MotorElevasiTutup();
                              delay(50);
                              MotorElevasiStop();  delay(1);
                              KuatSinyal();
                            }
                          }
                      }
                    }
                  }
                }
                while(finish==0){
                  MotorAzimuthCW;
                  delay(1);
                  BacaKompas();TargetCurrent();
                  KuatSinyal();
                  CekError(); /// error           
                  if(sinyal>sinyalbesar2)  //baru
                  sinyalbesar2=sinyal; //baru   
                  if(sinyal<(sinyalbesar2-7)||sinyal<8){
                    MotorAzimuthStop();  
                    delay(200); 
                    while(sinyal<=(sinyalbesar2-2)){  /// edit
                      MotorAzimuthCCW();
                      delay(1);
                      KuatSinyal();
                      CekError(); /// error
                    }
                    MotorAzimuthStop();
                    delay(200);
                    finish=1; 
                  }
                } 
              }
            }
            MotorElevasiStop();
            delay(200);
            finish=1;
          }
        }
      }
    }
      
    MotorElevasiStop();
    MotorAzimuthStop();
    delay(200); 
  //}
    MotorElevasiStop();
    MotorAzimuthStop();
    delay(200); 
  
  MotorElevasiStop();
  MotorAzimuthStop();
  delay(2000);  
}

void TutupAntenna(){
  tutupan = EEPROM.read(1);
  tutupan = tutupan*2;  
  BacaKompas();TargetCurrent();
  myGLCD.setFont(SmallFont);
  myGLCD.setColor(255,0,0);
  myGLCD.printNumI(tutupan, 10, 10 );
  while(curElevasi>4&&curElevasi<245){
    MotorElevasiTutup();
    delay(100);
    BacaKompas();TargetCurrent();
  }
  MotorElevasiStop();
  delay(3000);
  BacaKompas();TargetCurrent();
//myGLCD.setFont(SmallFont);
//myGLCD.setColor(255,0,0);
//myGLCD.printNumI(tutupan, 10, 10 );
  if((tutupan-(curAzimuth/10))>0&&(tutupan-(curAzimuth/10))<180){
    arahmotor=0;
  }
  else if(tutupan-(curAzimuth/10)<0&&(tutupan-(curAzimuth/10))>-180){
    arahmotor=1;      
  }
  else if(tutupan-(curAzimuth/10)<0&&(tutupan-(curAzimuth/10))<-180){
    arahmotor=0;
  }
  else if(tutupan-(curAzimuth/10)>0&&(tutupan-(curAzimuth/10))>180){
    arahmotor=1;
  }
  
  if(arahmotor==0){
    while(digitalRead(limit2)==1){MotorAzimuthCW();  delay(100);}
  }
  else if(arahmotor==1){
    while(digitalRead(limit2)==1){MotorAzimuthCCW();  delay(100);}
  }  
  delay(2500);
  MotorAzimuthStop();
  delay(1000);
  while(digitalRead(limit1)==1){MotorElevasiTutup();  delay(100);}
  MotorElevasiStop();
  delay(1000);
  ResetOpen();
}


void setup()
{
  myGLCD.InitLCD();
  myGLCD.setFont(SmallFont);
  myGLCD.setBackColor(VGA_BLACK);
  max_x = myGLCD.getDisplayXSize();
  max_y = myGLCD.getDisplayYSize();
  myGLCD.clrScr();
  
  myTouch.InitTouch();
  myTouch.setPrecision(PREC_MEDIUM);
  /*
  machinenumber[0]='1';
  machinenumber[1]='4';
  machinenumber[2]='c';
  machinenumber[3]='o';
  machinenumber[4]='b';
  machinenumber[5]='a';
  machinenumber[6]='4';
  machinenumber[7]='4';
  machinenumber[8]='i';
  machinenumber[9]='d';
  machinenumber[10]='x';
  machinenumber[11]='w';
 */ 
 
  
   Serial.begin(9600);
   Serial1.begin(9600);  // Ethernet
   Serial2.begin(9600);  // GPS
   Serial3.begin(9600);  // Compass
   
   pinMode(8, OUTPUT);
   pinMode(9, OUTPUT);
   pinMode(10, OUTPUT);
   pinMode(11, OUTPUT);
   digitalWrite(8, HIGH);
   digitalWrite(9, HIGH);
   digitalWrite(10, HIGH);
   digitalWrite(11, HIGH); 
   
   pinMode(limit1, INPUT);
   pinMode(limit2, INPUT);
   pinMode(limit3, INPUT);
  delay(1000);
  //PlayCheck(play);
}

/*void loop()
{
  if(page==0){
    TargetCurrent();
    ManualButton();
    Copyright();
    SatelitName();
    SensorCheck();
    SiteLocation();
    KuatSinyal();
    
    BacaEEPROM();
    if(latGPS==0 && longGPS==0)
    {
      BacaGPS();
    }  
    BacaKompas();
    BacaSinyal();
    HitungAzimuth();
    HitungElevasi();
    

    if (myTouch.dataAvailable()){
      myTouch.read();
      x=myTouch.getX();
      y=myTouch.getY();
      if((x>=330) && (x<=480)){
        if((y>=180) && (y<=300)){
          if(play==0)
            { play=1; PlayCheck(play);}
          else
            { play=0; PlayCheck(play);} 
        } 
        if((y>=30) && (y<=70)){
          myGLCD.clrScr();
          page=1;  
        }
      }
      if((x>=40) && (x<=100)){
        if((y>=250) && (y<=320)){
        ///manualkiri
          if(manualkiri==0&&manualkanan==0)
            manualkiri=1;
          else
            manualkiri=0;
        }
        if((y>=370) && (y<=440)){
        //manualbawah
          if(manualbawah==0&&manualatas==0)
            manualbawah=1;
          else
            manualbawah=0;
        }  
      }  
      
      if((x>=160) && (x<=230)){
        if((y>=250) && (y<=320)){
        //manualkanan
          if(manualkanan==0&&manualkiri==0)
            manualkanan=1;
          else
            manualkanan=0;
        }
        if((y>=370) && (y<=440)){
        //manualatas
          if(manualatas==0&&manualbawah==0)
            manualatas=1;
          else
            manualatas=0;
        }
      }
    
      
      if((x>=560) && (x<=750)){
        if((y>=150) && (y<=340)){
          if(ceksinyal==0)  
            ceksinyal=2;
          else if(ceksinyal==2)
            ceksinyal=0;  
        }
      }
      
    }
  }
  
  
  if(page==1){
    SatelitName();
    PilihSatelit();
    
    if (myTouch.dataAvailable()){
      myTouch.read();
      x=myTouch.getX();
      y=myTouch.getY();
      myGLCD.clrScr();
      
      if((x>=100) && (x<=300)){
        if((y>=115) && (y<=155))
          val=1;
        else if((y>=195) && (y<=235))
          val=2;
        else if((y>=275) && (y<=315))
          val=3;      
      } 
      
      if((x>=500) && (x<=800)){
        if((y>=115) && (y<=155))
          val=4;
        else if((y>=195) && (y<=235))
          val=5;
        else if((y>=275) && (y<=315))
          val=6;
        else if((y>=385) && (y<=435))
          {myGLCD.clrScr();page=0;PlayCheck(play);}     
      } 
      
      
      EEPROM.write(0, val);
      BacaEEPROM();
    }
  }
  
}

*/

void loop()
{
  if(showdisplay==0){
    if(!clearscreen){myGLCD.clrScr();clearscreen=1;}
    PasswordDisplay();
  }
  
  if(showdisplay==1){
    if(!clearscreen){myGLCD.clrScr();clearscreen=1;}
    //passlimit=0;
    StartSoftware();
  }
  
  
  if(showdisplay==2){  ///start display
    if(!clearscreen){myGLCD.clrScr();clearscreen=1;}
    ManualDisplay();
  }
}

void serialEvent1(){
  while(Serial1.available()) {
    buf = Serial1.read();
    Serial.write(buf);
    Serial.print(";  ");
    Serial.print(buf);
    Serial.println(";");
    if(flag){
      data[index++] = buf;
    }
    if(buf== '%'){
      sinyal = atoi(data);
      index = 0;
      flag = true;
    }
    if(buf== 'C'){
      index = 0;
      modem = 1;
      myGLCD.clrScr();
      page=0;
    }
    if(buf== 'F'){
      index = 0;
      modem = 0;
    }
  }
  
  if(page==0){
    myGLCD.setFont(SixteenSegment96x144Num);
    myGLCD.setColor(255,0,0);
    if(sinyal>0){     /////----> Tampil Nilai SQF
      if((sinyal/10)<1){
        myGLCD.print("0",581,181);
        myGLCD.printNumI(sinyal, 677, 181);
        //myGLCD.print("Failed",581,327);
      }
      else{
        myGLCD.printNumI(sinyal, 581, 181);
      }
      if(sinyal<31){
        myGLCD.setFont(BigFont);
        myGLCD.setColor(255,0,0);
        myGLCD.print("            ",581,327);
        myGLCD.print("Failed",581,327);
      }
      else{
        myGLCD.setFont(BigFont);
        myGLCD.setColor(0,255,0);
        myGLCD.print("Connected",581, 327); 
      }
    }  
  }
}
