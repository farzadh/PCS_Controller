/*
Tesla Model 3 Power Conversion System Controller Software. Alpha version.

V6 features auto enable and disable of PCS and DCDC function when car reports on.
also experimantal version for LIM testing.

Copyright 2021 
Damien Maguire

What works : Enable/disable and set output for dcdc converter on both US and EU version.Tested on bench and in vehicle. 
12v derived precharge tested on bench and seems to work.

Charge function tested on US version to 1.4kw AC.
Using V1 PCS controller hardware : https://github.com/damienmaguire/Tesla-Model-3-Charger

Messages are sent over IPC can only! The PCS does not seem to require any messages over its CP can.
PWM lines are not PWM lines. Just logic level enables.
DCDC must be at 5v to enable.
CHG must be at 5v to enable.
*/
#include <Metro.h>
#include <due_can.h>  
#include <due_wire.h> 
#include <DueTimer.h>  
#include <Wire_EEPROM.h> 
#include <AverageValue.h>


#define SerialDEBUG SerialUSB
 template<class T> inline Print &operator <<(Print &obj, T arg) { obj.print(arg); return obj; } //Allow streaming
 
#define PCS_ENABLE 50
#define PWM_DCDC 8
#define PWM_CHG 9
#define EVSE_PILOT 2
#define EVSE_ACT 44
#define EVSE_PROX A0
#define OUT1 48
#define OUT2 49
#define IN1 6
#define IN2 7
#define led 13

Metro timer_diag = Metro(1100);
Metro timer_Frames100 = Metro(100);
Metro timer_Frames10 = Metro(10);
Metro timer_Frames50 = Metro(50);

float AClim,ACamps,ACpwr,DCDCvolts,DCDCamps,HVvolts,LVvolts;
float ACvolts=0;

float IOut_PhA=0;
float IOut_PhB=0;
float IOut_PhC=0;
float IOut_Total=0;

uint8_t iac_lim = 20; 
uint16_t CHGpwr=0;
uint16_t voltsSetPnt = 280; //set point for charger HV., changed from original of 340 to 300
uint16_t vcuHVvolts=280; //modified by me from 0 to 300
uint16_t chgPwrSetPnt = 0;
uint16_t maxChgPwr = 1500;
uint16_t DCSetPoint = 1450; //DC set point scaled by 100
bool mux545=true;
bool mux3b2=true;
bool Backup2c4=true;

bool DCDCact=false;
bool CHGact=false;
bool PCSact=false;
bool USpcs=true;
bool Menudisp=true;
byte Count545=0;
bool ACState=false;
bool CANState=false;
bool autoCHG=false;
bool CHGhvreq=false;
bool CHGpwror=false;
byte limMode=0x00;
byte PCSBootId=0;
bool short2B2=false;

CAN_FRAME outFrame;  //A structured variable according to due_can library for transmitting CAN data.
CAN_FRAME inFrame;    //structure to keep inbound inFrames

////////////////////////////////////////////////////////////////////////
//*********EVSE VARIABLE   DATA ******************
byte Proximity = 0;
//proximity status values for type 1
#define Unconnected 0 // 3.3V
#define Buttonpress 1 // 2.3V
#define Connected 2 // 1.35V

volatile uint32_t pilottimer = 0;
volatile uint16_t timehigh, duration = 0;
volatile uint16_t accurlim = 0;
volatile uint16_t accurlimTMP = 0;
volatile int dutycycle = 0;
byte CPtype=2;  //set type 2 charge socket

uint16_t cablelim =32; // Type 2 cable current limit
//////////////////////////////////////////////////////////////////////

// Number of values to calculate with. Prevents memory problems
const long MAX_VALUES_NUM = 40;//using this to smooth out the pilot current reading.

AverageValue<long> averageValue(MAX_VALUES_NUM);

void setup() 
{
  Can0.begin(CAN_BPS_500K);   // IPC CAN To PCS
  Can0.watchFor();
  Serial.begin(115200);  //Initialize our USB port which will always be redefined as SerialUSB to use the Native USB port tied directly to the SAM3X processor.
  Serial2.begin(19200); //setup serial 2 for wifi access
  pinMode(PCS_ENABLE,OUTPUT);
  pinMode(PWM_DCDC,OUTPUT);
  pinMode(PWM_CHG,OUTPUT);
  pinMode(EVSE_ACT,OUTPUT);
  pinMode(OUT1,OUTPUT);
  pinMode(OUT2,OUTPUT);
  pinMode(led,OUTPUT);
  pinMode(IN1,INPUT);
  pinMode(IN2,INPUT);
  pinMode(EVSE_PILOT,INPUT);
  pinMode(EVSE_PROX,INPUT);
  
  digitalWrite(EVSE_ACT,LOW); //turn off evse at start

  digitalWrite(PCS_ENABLE,LOW);  //pcs off at startup
  //digitalWrite(PWM_DCDC,LOW); //set dcdc pwm line high
  //digitalWrite(PWM_CHG,HIGH); //set charger pwm line high
}

  
void loop()
{
  checkCAN();
  if(timer_diag.check())
  {
    if(Menudisp)
    {
      handle_Wifi();    
    }
  }

  if(timer_Frames10.check())
  {
    if(PCSact)
    {
      Msgs10ms();
    }
  }

  if(timer_Frames50.check())
  {
    if(PCSact)
    {
      Msgs50ms();
    }
  }

  if(timer_Frames100.check())
  {
//    if(PCSact) 
//    {
//      Msgs100ms();
//    }
    digitalWrite(PCS_ENABLE, PCSact ? HIGH : LOW); //set PCS enable line low/high
    digitalWrite(PWM_DCDC, (DCDCact & PCSact) ? HIGH:LOW); //LOW : HIGH); //set dcdc pwm line low/high
    digitalWrite(PWM_CHG, (CHGact & PCSact) ? HIGH:LOW); // LOW : HIGH); //set charger pwm line low/high
    digitalWrite(EVSE_ACT, ACState ? HIGH : LOW); //turn on/off evse
    if(PCSact) 
    {
      Msgs100ms();
    }
    
    if(autoCHG) //if we are in auto charge mode
    {
      Pilotcalc();  //calc pilot current
    }
    evseread();   //check for evse connection.  
  }
  checkforinput();
}

void Msgs10ms() //10ms messages here
{
//------------------------------------------------
  outFrame.id = 0x13D;//Required by post 2020 firmwares. Mirrors some content in 0x23D.
  outFrame.length = 6;
  outFrame.extended = 0;          
  outFrame.rtr=1;
  outFrame.data.bytes[0] = CHGact ? 0x05 : 0x0A;
  outFrame.data.bytes[1]=2*iac_lim; //charge current limit. gain 0.5. 0x40 = 64 dec =32A. Populate AC lim in here.
  outFrame.data.bytes[2]=0XAA;
  outFrame.data.bytes[3]=0X1A;
  outFrame.data.bytes[4]=0xFF;
  outFrame.data.bytes[5]=0x02;
  Can0.sendFrame(outFrame); //send to pcs IPC can 
//------------------------------------------------
  //This message is the heart of the beast. controls dcdc modes and en/dis of dcdc and charger.
  outFrame.id = 0x22A;       //HVS PCS control. kills hvp mia alert
  outFrame.length = 4;            
  outFrame.extended = 0;          
  outFrame.rtr=1;                 
  outFrame.data.bytes[0]=0x00; //precharge request voltage. 16 bit signed int. scale 0.1. Bytes 0 and 1. 
  outFrame.data.bytes[1]=0x00;
  uint8_t activation = 0x0;
  if(DCDCact && CHGact)
  {
    activation = 0x0D;
  }
  else if(DCDCact && !CHGact)
  {
    activation = 0x09;
  }
  else if(!DCDCact &&  CHGact)
  {
    activation = 0x05;
  }
  outFrame.data.bytes[2]= ((vcuHVvolts&0xF)<<4) | activation;
  outFrame.data.bytes[3]= (vcuHVvolts>>4)&0xFF;
  Can0.sendFrame(outFrame); //send to pcs IPC can 
//------------------------------------------------
  outFrame.id = 0x3B2;       //BMS log message
  outFrame.length = 8;            
  outFrame.extended = 0;          
  outFrame.rtr=1;       
  if(mux3b2)
  {
    outFrame.data.bytes[0]=0xE5;//mux 5=charging.
    outFrame.data.bytes[1]=0x0D;
    outFrame.data.bytes[2]=0xEB;
    outFrame.data.bytes[3]=0xFF;
    outFrame.data.bytes[4]=0x0C;
    outFrame.data.bytes[5]=0x66;
    outFrame.data.bytes[6]=0xBB;
    outFrame.data.bytes[7]=0x11;
    /* original    
    outFrame.data.bytes[0]=0x5E;  //kills bms mia
    outFrame.data.bytes[1]=0x0F; 
    outFrame.data.bytes[2]=0xF9;  
    outFrame.data.bytes[3]=0xFF;   
    outFrame.data.bytes[4]=0x00;
    outFrame.data.bytes[5]=0xCB; 
    outFrame.data.bytes[6]=0xB6;
    outFrame.data.bytes[7]=0x04; */
    mux3b2=false;
    Can0.sendFrame(outFrame); //send to pcs IPC can
  }
  else
  {
    outFrame.data.bytes[0]=0xE3;//mux 3=charge termination
    outFrame.data.bytes[1]=0x5D;
    outFrame.data.bytes[2]=0xFB;
    outFrame.data.bytes[3]=0xFF;
    outFrame.data.bytes[4]=0x0C;
    outFrame.data.bytes[5]=0x66;
    outFrame.data.bytes[6]=0xBB;
    outFrame.data.bytes[7]=0x06;
    /* original is below, above is from stm32 firmware
    outFrame.data.bytes[0]=0x5D;  
    outFrame.data.bytes[1]=0x0F; 
    outFrame.data.bytes[2]=0xF9;  
    outFrame.data.bytes[3]=0xFF;   
    outFrame.data.bytes[4]=0x00;
    outFrame.data.bytes[5]=0xCB; 
    outFrame.data.bytes[6]=0xB6;
    outFrame.data.bytes[7]=0x04;*/
    mux3b2=true;
    Can0.sendFrame(outFrame); //send to pcs IPC can
  }
}
    
void Msgs50ms()                       //50ms messages here
{
  outFrame.id = 0x545;       //VCFront unknown message
  outFrame.length = 8;            
  outFrame.extended = 0;          
  outFrame.rtr=1;       
  if(mux545)
  {
    outFrame.data.bytes[0]=0x14;  //kills vcfront mia
    outFrame.data.bytes[1]=0x00; 
    outFrame.data.bytes[2]=0x3f;  
    outFrame.data.bytes[3]=0x70;   
    outFrame.data.bytes[4]=0x9f;
    outFrame.data.bytes[5]=0x01; 
    outFrame.data.bytes[6]=(Count545<<4)|0xA;
    PCS_cksum(outFrame.data.bytes, 0x545);
    mux545=false;
    Can0.sendFrame(outFrame); //send to pcs IPC can
  }
  else
  {
    outFrame.data.bytes[0]=0x03;  //kills vcfront mia
    outFrame.data.bytes[1]=0x19; 
    outFrame.data.bytes[2]=0x64;  
    outFrame.data.bytes[3]=0x32;   
    outFrame.data.bytes[4]=0x19;
    outFrame.data.bytes[5]=0x00; 
    outFrame.data.bytes[6]=(Count545<<4);
    PCS_cksum(outFrame.data.bytes, 0x545);
    mux545=true;
    Can0.sendFrame(outFrame); //send to pcs IPC can
  }
  Count545++;
  if(Count545>0x0F)
  {
    Count545=0;
  }
}

void Msgs100ms()                      ////100ms messages here
{
//------------------------------------------------------
  outFrame.id = 0x20A;       //HVP contactor state     
  outFrame.length = 6;//8;            
  outFrame.extended = 0;          
  outFrame.rtr=1;                 
  outFrame.data.bytes[0]=0xf6;  //basically telling the pcs we have closed main contactors.
  outFrame.data.bytes[1]=0x15;  //and are ready to party.
  outFrame.data.bytes[2]=0x09;  //may need to change if we are using dcdc precharge mode....
  outFrame.data.bytes[3]=0x82;   //CP CAN
  outFrame.data.bytes[4]=0x18;
  outFrame.data.bytes[5]=0x01; 
  //outFrame.data.bytes[6]=0x00;
  //outFrame.data.bytes[7]=0x00;
  Can0.sendFrame(outFrame); //send to pcs IPC can
//------------------------------------------------------
  outFrame.id = 0x212;            //BMS ready message
  outFrame.length = 8;            
  outFrame.extended = 0;          
  outFrame.rtr=1;                 
  outFrame.data.bytes[0]=0xb9;  
  outFrame.data.bytes[1]=0x1c; 
  outFrame.data.bytes[2]=0x94;  
  outFrame.data.bytes[3]=0xad;   
  outFrame.data.bytes[4]=0xc3; //0xc1;
  outFrame.data.bytes[5]=0x15; 
  outFrame.data.bytes[6]=0x06; //0x36;
  outFrame.data.bytes[7]=0x63; //0x6b;
  Can0.sendFrame(outFrame); //send to pcs
//------------------------------------------------------
  outFrame.id = 0x21D;       //CP evse status. Connected? how much amps available? from cp ecu.     
  outFrame.length = 8;       //Pilot current in byte 1 as an 8 bit unsigned int scale 0.5     
  outFrame.extended = 0;     //Cable current limit in byte 3 bits 24-30 as a 7 bit unsigned int scale 1.     
  outFrame.rtr=1;                 
  outFrame.data.bytes[0]=USpcs ? 0x5D : 0x2D;  //2d for EU , 5d for US
  outFrame.data.bytes[1]=0x20; 
  outFrame.data.bytes[2]=0x00;  
  outFrame.data.bytes[3]=0x20;   
  outFrame.data.bytes[4]=0x80;
  outFrame.data.bytes[5]=0x00; 
  outFrame.data.bytes[6]=0x60;
  outFrame.data.bytes[7]=0x10;
  Can0.sendFrame(outFrame); //send to pcs IPC can
//------------------------------------------------------
  //BMS Contactor request.Static msg.
  outFrame.id = 0x232;       
  outFrame.length = 8;
  outFrame.extended = 0;
  outFrame.rtr=1;          
  outFrame.data.bytes[0]= 0x0A;
  outFrame.data.bytes[1]= 0x02;
  outFrame.data.bytes[2]= 0xD5;
  outFrame.data.bytes[3]= 0x09;
  outFrame.data.bytes[4]= 0xCB;
  outFrame.data.bytes[5]= 0x04;
  outFrame.data.bytes[6]= 0x00;
  outFrame.data.bytes[7]= 0x00;
  Can0.sendFrame(outFrame); //send to pcs IPC can
//------------------------------------------------------
  outFrame.id = 0x23D;       //CP AC charge current limit    
  outFrame.length = 4; //!USpcs ? 4 : 2;
  outFrame.extended = 0;
  outFrame.rtr=1;
  outFrame.data.bytes[0] = CHGact ? 0x05 : 0x0A;  
  outFrame.data.bytes[1] = 2 * iac_lim; //0x00; 
  outFrame.data.bytes[2] = 0xFF;  //Internal max current limit.
  outFrame.data.bytes[3] = 0x0F; 
  Can0.sendFrame(outFrame); //send to pcs IPC can
//------------------------------------------------------
  outFrame.id = 0x25D;       //CP unknown static msg     
  outFrame.length = 8;         //kills cp mia   
  outFrame.extended = 0;          
  outFrame.rtr=1;                 
  outFrame.data.bytes[0]= USpcs ? 0xD8 : 0xD9;  
  outFrame.data.bytes[1]=0x8c; 
  outFrame.data.bytes[2]=0x01;  
  outFrame.data.bytes[3]=0xb5;   
  outFrame.data.bytes[4]=0x4a;
  outFrame.data.bytes[5]=0xc1; 
  outFrame.data.bytes[6]=0x0a;
  outFrame.data.bytes[7]=0xe0;
  Can0.sendFrame(outFrame); //send to pcs IPC can
//------------------------------------------------------
  outFrame.id = 0x2B2;       //Possible PCS charge power request message.kills bms mia.
  //outFrame.length = USpcs ? 3 : 5;        //This message is used to request charge power from the pcs. 
  outFrame.length = short2B2 ? 3 : 5; 
  outFrame.extended = 0;      //kills bms mia
  outFrame.rtr=1;              //US hvcon sends this as dlc=3. EU sends as dlc=5. A missmatch here will trigger a can rationality error.
  outFrame.data.bytes[0]=lowByte(chgPwrSetPnt);  //0x0578 = 1400 dec = 1400Watts. So 16 bit unsigned scale of 1 ... ? or possible kW with a scale of 0.001
  outFrame.data.bytes[1]=highByte(chgPwrSetPnt);  //byte 2 bit 1 may be an ac charge enable.
  outFrame.data.bytes[2] = CHGact ? 0x02 : 0x00;
  outFrame.data.bytes[3]=0x00;
  outFrame.data.bytes[4]=0x00;
  Can0.sendFrame(outFrame); //send to pcs IPC can
//------------------------------------------------------
  outFrame.id = 0x321; //VCFront sensors. Static.
  outFrame.length = 8;            
  outFrame.extended = 0;          
  outFrame.rtr=1;                 
  outFrame.data.bytes[0]=0x2C;  //kills vcfront mia
  outFrame.data.bytes[1]=0xB6; 
  outFrame.data.bytes[2]=0xA8;  
  outFrame.data.bytes[3]=0x7F;   
  outFrame.data.bytes[4]=0x02;
  outFrame.data.bytes[5]=0x7F; 
  outFrame.data.bytes[6]=0x00;
  outFrame.data.bytes[7]=0x00;
  Can0.sendFrame(outFrame); //send to pcs IPC can
//------------------------------------------------------
  outFrame.id = 0x333;       //UI charge request message
  outFrame.length = 5; //outFrame.length = 4; 4 did not work for me, throws CAN rationality error
  outFrame.extended = 0;          
  outFrame.rtr=1;                 
  outFrame.data.bytes[0]=0x04;  //kills ui mia
  outFrame.data.bytes[1]=0x30; //48A limit
  outFrame.data.bytes[2]=0x29;  
  outFrame.data.bytes[3]=0x07;
  outFrame.data.bytes[4]=0x00;
  Can0.sendFrame(outFrame); //send to pcs IPC can
//------------------------------------------------------
  outFrame.id = 0x3A1;       //VCFront vehicle status
  outFrame.length = 8;       //This message contains the 12v dcdc target setpoint.  
  outFrame.extended = 0;     //bits 16-26 as an 11bit unsigned int. scale 0.01    
  outFrame.rtr=1;                 
  outFrame.data.bytes[0]=0x09;  //kills vcfront mia
  outFrame.data.bytes[1]=0x62; 
  outFrame.data.bytes[2]=DCSetPoint & 0xFF; //original: 0x78, 78 , 9 gives us a 14v target. fhessar: 0xCB0 should translate to 12V
  outFrame.data.bytes[3]=((DCSetPoint >> 8)|0x98);  //original number: 0x9d
  outFrame.data.bytes[4]=0x08;
  outFrame.data.bytes[5]=0x2C; 
  outFrame.data.bytes[6]=0x12;
  outFrame.data.bytes[7]=0x5A;
  Can0.sendFrame(outFrame); //send to pcs IPC can
//------------------------------------------------------
  digitalWrite(led,!digitalRead(led));//blink led every time we fire this interrrupt.
}

void checkCAN()
{
  if(Can0.available())
  {
    Can0.read(inFrame);
    if(inFrame.id == 0x264)//AC input data
    {
      AClim = (uint16_t)(((inFrame.data.bytes[5]<<8 | inFrame.data.bytes[4])&0x3ff)*0.1);
      ACpwr = ((inFrame.data.bytes[3])*.1);
      ACvolts = (((inFrame.data.bytes[1]<<8 | inFrame.data.bytes[0])&0x3FFF)*0.033);
      ACamps = (((inFrame.data.bytes[2]<<9 | inFrame.data.bytes[1])>>7)*0.1);
    }
    else if(inFrame.id == 0x224)//dcdc data
    {
      DCDCamps = (((inFrame.data.bytes[3]<<8 | inFrame.data.bytes[2])&0xFFF)*0.1); //dcdc actual current. 12 bit unsigned int in bits 16-27. scale 0.1.
    }
    else if(inFrame.id == 0x2C4)//dcdc data from  log messages. This is a big muxed mess. We only want mux 6 for our needs.
    {
      byte MuxID = (inFrame.data.bytes[0]);
      //SerialDEBUG.print("<><><><><         READ LOW and HIGH VOLTAGE BATTERIES\n");
      if((MuxID==0xE6) || (MuxID==0xC6))//if in mux 6 grab the info...
      {
        HVvolts = (((inFrame.data.bytes[3]<<8 | inFrame.data.bytes[2])&0xFFF)*0.146484); //measured hv voltage. 12 bit unsigned int in bits 16-27. scale 0.146484. 
        LVvolts = ((((inFrame.data.bytes[1]<<9 | inFrame.data.bytes[0])>>6))*0.0390625); //measured lv voltage. 10 bit unsigned int in bits 5-14. scale 0.0390626. 
        Backup2c4 = false;
      }
      else if((MuxID==0x04)&&(Backup2c4))//if we dont get HV volts then switch to backup.
      {
        HVvolts = ((((inFrame.data.bytes[7]<<8 | inFrame.data.bytes[6])>>3)&0xFFF)*0.146484); //measured hv voltage. 12 bit unsigned int in bits 51-62. scale 0.146484.
        LVvolts = 0;//In new firmware DCDC LV is not in 0x2C4.
      }

      bool GotDCI;
      MuxID = (inFrame.data.bytes[0] & 0x1F);
      if(MuxID==0x00)                    //Calculate total DC output current from all 3 charger modules.
      {
        IOut_PhA=((inFrame.data.bytes[4]))*0.1f;
        GotDCI=true;
      }
      else if(MuxID==0x01)
      {
        IOut_PhB=((inFrame.data.bytes[4]))*0.1f;
        GotDCI=true;
      }
      else if(MuxID==0x02)
      {
        IOut_PhC=((inFrame.data.bytes[4]))*0.1f;
        GotDCI=true;
      }
      IOut_Total=IOut_PhA+IOut_PhB+IOut_PhC;
    }
    else if(inFrame.id == 0x204)
    {
      ////PCS Charge status message
      //See PCSCAN.cpp in stm32 software for details
    }
    else if (inFrame.id == 0x2A4)
    {
      //PCS Temps
      //See PCSCAN.cpp in stm32 software for details
    }
    else if (inFrame.id == 0x3A4)
    {
      //PCS Alert Matrix
      //0=None, 1=CP_MIA, 2=BMS_MIA, 3=HVP_MIA, 4=UNEX_AC, 5=CHG_VRAT, 6=DCDC_VRAT, 7=VCF_MIA, 8=CAN_RAT, 9=UI_MIA
      byte PCSAlertPage = inFrame.data.bytes[0]&0xf;
      process_alert_matrix();
    }
    else if (inFrame.id == 0x424)
    {
      //From param_prj.h file in stm32 SW: 
      // #define ALERTS       "0=None, 1=01chgHwInputOc, 2=02chgHwOutputOc, 3=03chgHwInputOv, 4=04chgHwIntBusOv, 5=05chgOutputOv, 
      //6=06chgPrechargeFailedScr, 7=07chgPhaseTempHot, 8=08chgPhaseOverTemp, 9=09chgPfcCurrentRegulation, 10=10chgIntBusVRegulation, 
      //11=11chgLlcCurrentRegulation, 12=12chgPfcIBandTracerFault, 13=13chgPrechargeFailedBoost, 14=14chgTempRationality, 15=15chg12vUv, 
      //16=16chgAllPhasesFaulted, 17=17chgWallPowerRemoval, 18=18chgUnknownGridConfig, 19=19acChargePowerLimited, 20=20chgEnableLineMismatch, 
      //21=21hvpMia, 22=22bmsMia, 23=23cpMia, 24=24vcfrontMia, 25=25cpu2Malfunction, 26=26watchdogAlarmed, 27=27chgInsufficientCooling, 
      //28=28chgOutputUv, 29=29chgPowerRationality, 30=30canRationality, 31=31uiMia, 32=32unused, 33=33hvBusUv, 34=34hvBusOv, 35=35lvBusUv, 
      //36=36lvBusOv, 37=37resonantTankOc, 38=38claFaulted, 39=39sdModuleClkFault, 40=40dcdcMaxPowerReached, 41=41dcdcOverTemp, 42=42dcdcEnableLineMismatch,
      //43=43hvBusPrechargeFailure, 44=4412vSupportRegulation, 45=45hvBusLowImpedance, 46=46hvBusHighImpedence, 47=47lvBusLowImpedance, 
      //48=48lvBusHighImpedance, 49=49dcdcTempRationality, 50=50dcdc12VsupportFaulted, 51=51chgIntBusUv, 52=52acVoltageNotPresent, 
      //53=53chgInputVDropHigh, 54=54chgInputVDropTooHigh, 55=55chgLineImedanceHigh, 56=56chgLineImedanceTooHigh, 57=57chgInputOverFreq, 
      //58=58chgInputUnderFreq, 59=59chgInputOvRms, 60=60chgInputOvPeak, 61=61chgVLineRationality, 62=62chgILineRationality, 
      //63=63chgVOutRationality, 64=64chgIOutRationality, 65=65chgPllNotLocked, 66=66dcdcHvRationality, 67=67dcdcLvRationality, 
      //68=68dcdcTankvRationality, 69=69chgPfcLineDidt, 70=70chgPfcLineDvdt, 71=71chgPfcILoopRationality, 72=72cpu2ClaStopped, 
      //73=73unexpectedAcInputVoltage, 74=74hvBusDischargeFailure, 75=75hvBusDischargeTimeout, 76=76dcdcEnDeassertedErr, 77=77microGridEnergyLow, 
      //78=78chgStopDcdcTooHot, 79=79eepromOperationError, 80=80damagedPhaseDetected, 81=81dcdcPchgTimeout, 82=82dcdcPchgUnsafeDiVoltage, 
      //83=83triggerOdin, 84=84unused, 85=85dcdcFetsNotSwitching, 86=86dcdcInsufficientCooling, 87=87nvramRecordStatusError, 88=88pchgParameters, 
      //89=89hvBusDischargeIrrational, 90=90expectedAcVoltageSourceMissing, 91=91chgIntBusRationality, 92=92chgPowerLimitedByBusRipple, 
      //93=93powerRailRationality,  94=94pcsDcdcNeedService, 95=95dcdcSensorlessModeActive, 96=96microGridOverLoaded, 97=97rebootPhaseDetected, 
      //98=98gridFreqDroopDetectedSilent, 99=99microGridOverLoadedSilent, 100=100microGridEnergyLowSilent, 101=101phMachineModelIrrational, 
      //102=102resetWithDCDCCmdAsserted"

      //PCS Alert Log
      byte PCSAlertId = inFrame.data.bytes[0];
      SerialDEBUG.print(PCSAlertId);
      SerialDEBUG.print(print_pcs_alert(PCSAlertId));
      SerialDEBUG.print(": ");
      for (byte i = 1; i < inFrame.length; i++)
      {
        SerialDEBUG.print(inFrame.data.bytes[i], HEX);
        SerialDEBUG.print(",");
      }
      SerialDEBUG.print("\n");
      if(PCSAlertId == 0x1E)    //0x1E = Alert30 = CAN rationality.
      {
        short AlertCANId = ((inFrame.data.bytes[4]<<8 | inFrame.data.bytes[3]));//Grab the CAN ID from the alert.
        byte AlertRxError=(inFrame.data.bytes[2]&0x07);//Grab the alert detail
        SerialDEBUG.print("        ALERT 30 CANId, RXError: ");
        SerialDEBUG.print(AlertCANId);
        SerialDEBUG.print(", ");
        SerialDEBUG.print(AlertRxError);
        SerialDEBUG.print("\n");
        if (AlertCANId == 0x2B2)
        {
          if(AlertRxError==0x2)
          {
            SerialDEBUG.print("        ALERT 30 CAN RAIONALITY = msg 0x2B2 is too short so change to long.\n");
          }
          if(AlertRxError==0x1)
          {
            SerialDEBUG.print("        ALERT 30 CAN RAIONALITY = msg 0x2B2 is too long so change to short.\n");
          }
        }
      }
    }
    else if (inFrame.id == 0x504)
    {
      //PCS Boot ID
      PCSBootId = inFrame.data.bytes[7];
      /*SerialDEBUG.print("<><><> PCS BOOT ID = ");
      SerialDEBUG.print(PCSBootId);
      SerialDEBUG.print("\n");*/
    }
    else if (inFrame.id == 0x76C)
    {
    }
  }
}

void process_alert_matrix()
{
  SerialDEBUG.print(inFrame.id, HEX);
  SerialDEBUG.print(":");
  for (int i = 0; i < inFrame.length; i++)
  {
    SerialDEBUG.print(inFrame.data.bytes[i], HEX);
    SerialDEBUG.print(",");
  }
  byte PCSAlertPage = inFrame.data.bytes[0] & 0xF;
  uint32_t* err_matrix;
  if (PCSAlertPage == 0)
  {
    SerialDEBUG.print(" -- ");
    err_matrix = (uint32_t*) inFrame.data.bytes;
    for(byte i = 4; i < 32; i++)
    {
      uint32_t mask = 1;
      mask <<= i;
      if(err_matrix[0] & mask)
      {
        SerialDEBUG.print(print_pcs_alert(i - 3));
        SerialDEBUG.print(", ");
      }
    }
    for(byte i = 0; i < 32; i++)
    {
      uint32_t mask = 1;
      mask <<= i;
      if(err_matrix[1] & mask)
      {
        SerialDEBUG.print(print_pcs_alert(i+29));
        SerialDEBUG.print(", ");
      }
    }
  }
  else if (PCSAlertPage == 1)
  {
    SerialDEBUG.print(" -- ");
    err_matrix = (uint32_t*) inFrame.data.bytes;
    for(byte i = 4; i < 32; i++)
    {
      uint32_t mask = 1;
      mask <<= i;
      if(err_matrix[0] & mask)
      {
        SerialDEBUG.print(print_pcs_alert(i+57));
        SerialDEBUG.print(", ");
      }
    }
    for(byte i = 0; i < 32; i++)
    {
      uint32_t mask = 1;
      mask <<= i;
      if(err_matrix[1] & mask)
      {
        SerialDEBUG.print(print_pcs_alert(i+89));
        SerialDEBUG.print(", ");
      }
    }    
  }
  else
  {
    SerialDEBUG.println("Unexpected PCS ALERT PAGE");
    SerialDEBUG.println("Unexpected PCS ALERT PAGE");
    SerialDEBUG.println("Unexpected PCS ALERT PAGE");
    SerialDEBUG.println("Unexpected PCS ALERT PAGE");
  }
  SerialDEBUG.println("");
}

char* print_pcs_alert(byte alert_id)
{
  switch (alert_id)
  {
    case 0:
      return " None";
      break;
    case 1:
      return " 01chgHwInputOc";
      break;
    case 2:
      return " 02chgHwOutputOc";
      break;
    case 3:
      return " 03chgHwInputOv";
      break;
    case 4:
      return " 04chgHwIntBusOv";
      break;
    case 5:
      return " 05chgOutputOv";
      break;
    case 6:
      return " 06chgPrechargeFailedScr";
      break;
    case 7:
      return " 07chgPhaseTempHot";
      break;
    case 8:
      return " 08chgPhaseOverTemp";
      break;
    case 9:
      return " 09chgPfcCurrentRegulation";
      break;
    case 10:
      return " 10chgIntBusVRegulation";
      break;
    case 11:
      return " 11chgLlcCurrentRegulation";
      break;
    case 12:
      return " 12chgPfcIBandTracerFault";
      break;
    case 13:
      return " 13chgPrechargeFailedBoost";
      break;
    case 14:
      return " 14chgTempRationality";
      break;
    case 15:
      return " 15chg12vUv";
      break;
    case 16:
      return " 16chgAllPhasesFaulted";
      break;
    case 17:
      return " 17chgWallPowerRemoval";
      break;
    case 18:
      return " 18chgUnknownGridConfig";
      break;
    case 19:
      return " 19acChargePowerLimited";
      break;
    case 20:
      return " 20chgEnableLineMismatch";
      break;
    case 21:
      return " 21hvpMia";
      break;
    case 22:
      return " 22bmsMia";
      break;
    case 23:
      return " 23cpMia";
      break;
    case 24:
      return " 24vcfrontMia";
      break;
    case 25:
      return " 25cpu2Malfunction";
      break;
    case 26:
      return " 26watchdogAlarmed";
      break;
    case 27:
      return " 27chgInsufficientCooling";
      break;
    case 28:
      return " 28chgOutputUv";
      break;
    case 29:
      return " 29chgPowerRationality";
      break;
    case 30:
      return " 30canRationality";
      break;
    case 31:
      return " 31uiMia";
      break;
    case 32:
      return " 32unused";
      break;
    case 33:
      return " 33hvBusUv";
      break;
    case 34:
      return " 34hvBusOv";
      break;
    case 35:
      return " 35lvBusUv";
      break;
    case 36:
      return " 36lvBusOv";
      break;
    case 37:
      return " 37resonantTankOc";
      break;
    case 38:
      return " 38claFaulted";
      break;
    case 39:
      return " 39sdModuleClkFault";
      break;
    case 40:
      return " 40dcdcMaxPowerReached";
      break;
    case 41:
      return " 41dcdcOverTemp";
      break;
    case 42:
      return " 42dcdcEnableLineMismatch";
      break;
    case 43:
      return " 43hvBusPrechargeFailure";
      break;
    case 44:
      return " 44-12vSupportRegulation";
      break;
    case 45:
      return " 45hvBusLowImpedance";
      break;
    case 46:
      return " 46hvBusHighImpedence";
      break;
    case 47:
      return " 47lvBusLowImpedance";
      break;
    case 48:
      return " 48lvBusHighImpedance";
      break;
    case 49:
      return " 49dcdcTempRationality";
      break;
    case 50:
      return " 50dcdc12VsupportFaulted";
      break;
    case 51:
      return " 51chgIntBusUv";
      break;
    case 52:
      return " 52acVoltageNotPresent";
      break;
    case 53:
      return " 53chgInputVDropHigh";
      break;
    case 54:
      return " 54chgInputVDropTooHigh";
      break;
    case 55:
      return " 55chgLineImedanceHigh";
      break;
    case 56:
      return " 56chgLineImedanceTooHigh";
      break;
    case 57:
      return " 57chgInputOverFreq";
      break;
    case 58:
      return " 58chgInputUnderFreq";
      break;
    case 59:
      return " 59chgInputOvRms";
      break;
    case 60:
      return " 60chgInputOvPeak";
      break;
    case 61:
      return " 61chgVLineRationality";
      break;
    case 62:
      return " 62chgILineRationality";
      break;
    case 63:
      return " 63chgVOutRationality";
      break;
    case 64:
      return " 64chgIOutRationality";
      break;
    case 65:
      return " 65chgPllNotLocked";
      break;
    case 66:
      return " 66dcdcHvRationality";
      break;
    case 67:
      return " 67dcdcLvRationality";
      break;
    case 68:
      return " 68dcdcTankvRationality";
      break;
    case 69:
      return " 69chgPfcLineDidt";
      break;
    case 70:
      return " 70chgPfcLineDvdt";
      break;
    case 71:
      return " 71chgPfcILoopRationality";
      break;
    case 72:
      return " 72cpu2ClaStopped";
      break;
    case 73:
      return " 73unexpectedAcInputVoltage";
      break;
    case 74:
      return " 74hvBusDischargeFailure ";
      break;
    case 75:
      return " 75hvBusDischargeTimeout";
      break;
    case 76:
      return " 76dcdcEnDeassertedErr";
      break;
    case 77:
      return " 77microGridEnergyLow";
      break;
    case 78:
      return " 78chgStopDcdcTooHot";
      break;
    case 79:
      return " 79eepromOperationError";
      break;
    case 80:
      return " 80damagedPhaseDetected";
      break;
    case 81:
      return " 81dcdcPchgTimeout";
      break;
    case 82:
      return " 82dcdcPchgUnsafeDiVoltage";
      break;
    case 83:
      return " 83triggerOdin";
      break;
    case 84:
      return " 84unused";
      break;
    case 85:
      return " 85dcdcFetsNotSwitching";
      break;
    case 86:
      return " 86dcdcInsufficientCooling";
      break;
    case 87:
      return " 87nvramRecordStatusError";
      break;
    case 88:
      return " 88pchgParameters";
      break;
    case 89:
      return " 89hvBusDischargeIrrational";
      break;
    case 90:
      return " 90expectedAcVoltageSourceMissing ";
      break;
    case 91:
      return " 91chgIntBusRationality";
      break;
    case 92:
      return " 92chgPowerLimitedByBusRipple";
      break;
    case 93:
      return " 93powerRailRationality";
      break;
    case 94:
      return " 94pcsDcdcNeedService";
      break;
    case 95:
      return " 95dcdcSensorlessModeActive";
      break;
    case 96:
      return " 96microGridOverLoaded";
      break;
    case 97:
      return " 97rebootPhaseDetected";
      break;
    case 98:
      return " 98gridFreqDroopDetectedSilent";
      break;
    case 99:
      return " 99microGridOverLoadedSilent";
      break;
    case 100:
      return " 100microGridEnergyLowSilent";
      break;
    case 101:
      return " 101phMachineModelIrrational";
      break;
    case 102:
      return " 102resetWithDCDCCmdAsserted";
      break;
    default:
      return " UNKNOWN";
  }

  return "";
}


void handle_Wifi()
{
  /*
   * 
   * Routine to send data to wifi on serial 2
  The information will be provided over serial to the esp8266 at 19200 baud 8n1 in the form :
  vxxx,ixxx,pxxx,mxxxx,nxxxx,oxxx,rxxx,qxxx* where :
  
  v=pack voltage (0-700Volts)
  i=current (0-1000Amps)
  p=power (0-300kw)
  m=half pack voltage (0-500volts)
  n=Amp Hours (0-300Ah)
  o=KiloWatt Hours (0-150kWh)
  r=HV Box Temp (0-100C)
  q=third pack Volts (0-500Volts)
  *=end of string
  xxx=three digit integer for each parameter eg p100 = 100kw.
  updates will be every 1000ms approx.
  
  v100,i200,p35,m3000,n4000,o20,r100,q50*
  */
    
  Serial2.print("v100,i200,p35,m3000,n4000,o20,r30,q50*"); //test string
  
  //digitalWrite(13,!digitalRead(13));//blink led every time we fire this interrrupt.
  SerialDEBUG.print("***************** EVBMW PCS CONTROLLER V1 *****************\n");
  SerialDEBUG.print("CHARGER: (AC Pwr, AC Lmt, AC current, AC Vlt, DC Vlt, VCU DC Vlt, Chg Pwr, Max Chg Pwr)\n         (");
  SerialDEBUG.print(ACpwr,1);
  SerialDEBUG.print(" kW, ");
  SerialDEBUG.print(AClim,1);
  SerialDEBUG.print(" A, ");
  SerialDEBUG.print(ACamps,1);
  SerialDEBUG.print(" A, ");
  SerialDEBUG.print(ACvolts,2);
  SerialDEBUG.print(" V, ");
  SerialDEBUG.print(HVvolts,2);
  SerialDEBUG.print(" V, ");
  SerialDEBUG.print(vcuHVvolts);
  SerialDEBUG.print(" V, ");
  SerialDEBUG.print(chgPwrSetPnt);
  SerialDEBUG.print(" W, ");
  SerialDEBUG.print(maxChgPwr);
  SerialDEBUG.print(" W)\n");
  SerialDEBUG.print("DCDC: (Voltage, Current)=(");
  SerialDEBUG.print(LVvolts,2);
  SerialDEBUG.print(" V, ");
  SerialDEBUG.print(DCDCamps,2);
  SerialDEBUG.print(" A\n");
  SerialDEBUG.print("EVSE: Prox Status: ");
  SerialDEBUG.print(Proximity);
  SerialDEBUG.print(", Cable Limit: ");
  SerialDEBUG.print(cablelim/1000);
  SerialDEBUG.print("Amps, Pilot Current: ");
  SerialDEBUG.print(accurlim/1000);
  SerialDEBUG.print("Amps\n");
  SerialDEBUG.print("CMDs: 'c' AC. 'f' DCDC. 'g' charge. 'e' PCS, 'm' US/EU, 'q' disp; 'pxxxx' eg p1500; 'vxxx' eg v360\n");
  SerialDEBUG.print("AC:");
  SerialDEBUG.print(ACState ? "ON, CAN:" : "OFF, CAN:");
  SerialDEBUG.print(CANState ? "ON, DCDC:" : "OFF, DCDC:");
  SerialDEBUG.print(DCDCact ? "ON, Charge:" : "OFF, Charge:");
  SerialDEBUG.print(CHGact ? "ON, PCS: " : "OFF, PCS:");
  SerialDEBUG.print(PCSact ? "ON, Type: " : "OFF, Type: ");
  SerialDEBUG.print(USpcs ? "US, Boot ID: " : "EU, Boot ID: ");
  SerialDEBUG.print(PCSBootId);
  SerialDEBUG.print("\n");
}

void checkforinput()
{ 
  //Checks for keyboard input from Native port 
   if (SerialDEBUG.available()) 
     {
      int inByte = SerialDEBUG.read();
      switch (inByte)
         {
          case 'c':            
            ACState=!ACState;//toggle evse on/off
            break;
  
          case 'd':    
            CANState=!CANState;//toggle can messages to pcs on/off
            break;

           case 'f':    
            DCDCact=!DCDCact;//toggle dcdc command bit
            break;
            
            case 'g':    
            CHGact=!CHGact;//toggle charger command bit
            break;             

            case 'e':    
            PCSact=!PCSact;//toggle PCS enable bit
            break; 

            case 'm':    
            USpcs=!USpcs;//toggle PCS type
            break;

            case 'q':    
            Menudisp=!Menudisp;//toggle serial display on/off
            break;
             
            case 'p':    
            maxChgPwr = SerialDEBUG.parseInt();
            break; 

            case 'v':    
            voltsSetPnt = SerialDEBUG.parseInt();
            break; 

            case 'o':    
            CHGpwror=!CHGpwror;//toggle PCS type
            break;                                 
          }    
      }
}

///////////////////////EVSE SECTION//////////////////////////////////////////////////////////////////////////////////////////
void evseread()
{
  uint16_t val=0;
  val = analogRead(EVSE_PROX);     // read the input pin
 
  if ( CPtype == 2)
  {
    if ( val > 950)
    {
      Proximity = Unconnected;
    }
    else
    {
      Proximity = Connected;
      if ( val < 950 && val > 800)
      {
        cablelim = 13000;
      }
      if ( val < 800 && val > 700)
      {
        cablelim = 20000;
      }
      if ( val < 600 && val > 450)
      {
        cablelim = 32000;
      }
      if ( val < 400 && val > 250)
      {
        cablelim = 63000;
      }
    }
  }

  if ( CPtype == 1)
  {
    if ( val > 800)
    {
      Proximity = Unconnected;
    }
    else
    {
      if ( val > 550)
      {
        Proximity = Buttonpress;
      }
      else
      {
        Proximity = Connected;
      }
    }
  }
}

void Pilotcalc()
{
  if (  digitalRead(EVSE_PILOT) == HIGH)
  {
    duration = micros() - pilottimer;
    pilottimer = micros();
  }
  else
  {
    accurlimTMP = (micros() - pilottimer) * 100 / duration * 600; //Calculate the duty cycle then multiply by 600 to get mA current limit
    averageValue.push(accurlimTMP);
    accurlim = averageValue.average();
  }
}

void PCS_cksum(uint8_t *data, uint16_t id)
{
  data[7] = 0;
  uint16_t checksum_calc=0;
  for(int b=0; b < 7; b++)
  {
    checksum_calc = checksum_calc + data[b];
  }
  checksum_calc += id + (id >> 8);
  checksum_calc &= 0xFF;
  data[7] = (uint8_t)checksum_calc;
}
