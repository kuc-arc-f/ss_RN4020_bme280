
/*
  SoftwareSerial, RN4020 BLE (Low Power version)
  with Central= raspBerryPi , BME280 Sensor
 *  v 0.9.5
*/
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <SoftwareSerial.h>
#include <BME280_MOD-1022.h>
#include <Wire.h>

SoftwareSerial mySerial(5, 6); /* RX:D5, TX:D6 */
String mBuff="";
const int mVoutPin = 0;
const int mPinBLE =8;
const int mOK_CODE=1;
const int mNG_CODE=0;
uint32_t mTimer_runMax= 0;
const int mNextSec   = 300; //Sec
const int mMax_runSec= 5; //Sec

const int mMode_RUN  = 1;
const int mMode_WAIT = 2; 
int mMode =0;

const char mDevice_name[3+1]="D13";

// LOW power
#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif
volatile int wdt_cycle;

//
long convert_Map(long x, long in_min, long in_max, long out_min, long out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

//
int Is_resWait(String value, uint32_t maxMsec ){
  int ret= mNG_CODE;
  uint32_t tm_maxWait=millis();
  int iLen= value.length();
  String sBuff="";
  int iBool=1;
    while(iBool ){
        while( mySerial.available() ){
            char c= mySerial.read();
            Serial.print( c );
            if( (c != 0x0a ) && (c != 0x0d ) ){
                sBuff.concat(c );
            }            
        } //end_while
        if(  (int )sBuff.length() >= iLen ){ iBool=0;  }      
        delay(100);
        uint32_t tmMax =millis() -tm_maxWait;
        if(tmMax > (uint32_t)maxMsec ){ 
          Serial.println("#Error-Is_resWait:maxSec!");
          iBool=0; 
        } 
    }//end_while_1
    Serial.println("");
    if(sBuff.length() < 1){ return ret; }
    int iLenBuff= sBuff.length();
    int iSt=iLenBuff -(iLen); 
    Serial.println("iLenBuff="+ String(iLenBuff)+",iSt="+ String(iSt) ) ;
    String sRecv = sBuff.substring(iSt   , iLen );
    Serial.println( "sBuff="+ sBuff+ " ,sRecv="+ sRecv );
    if(sRecv == value ){
      ret= mOK_CODE;
    }
  return ret;  
}
//
void proc_getSSerial(){
  while( mySerial.available() ){
    char c= mySerial.read();
Serial.print( c );
  } //end_while
}

int mCounter=0;
//
void proc_sendCmd(){
  int iWaitTm= 5000;
  char cBuff[24 +1];
  int iTemp = BME280_getTemperature();
  int iHumi = BME280_getHumidity();
  int iPress= BME280_getPressure();
//  sprintf(cBuff , "SN,%s%06d000001\r", mDevice_name,iSenNum );
  sprintf(cBuff , "SN,%s%06d%06d%05d\r", mDevice_name,iTemp , iHumi , iPress );
  //SN
  Serial.println( "#Start SN, cBuff="+  String(cBuff)  );
   mySerial.print(String(cBuff) );
   if( Is_resWait("AOK", iWaitTm ) == mNG_CODE ){ return; }
   else{Serial.println("#OK SN" ); };
   proc_getSSerial();
//   wait_forSec(3);
   //R
   Serial.println( "#Start R" );
   mySerial.print("R,1\r");
   delay(1000);
   if( Is_resWait("RebootCMD", iWaitTm ) == mNG_CODE ){ return; }
   else{Serial.println("#OK R1" ); };
   proc_getSSerial();
   //wait_forSec(3);
   
   //A
   Serial.println( "#Start A" );
//   mySerial.print("A\r");
   mySerial.print("A,0064,07D0\r");    //100msec, total= 2000mSec
  if( Is_resWait("AOK", iWaitTm ) == mNG_CODE ){ return; }
   else{Serial.println("#OK A" ); };
   proc_getSSerial();
   mCounter= mCounter+1;
   wait_forSec(3 );
}

//
void wait_forSec(int wait){
   for(int i=0; i<wait; i++){
    delay(1000);
    Serial.println("#wait_forMsec: "+ String(i) );    
  }
}
//
void setup() {
  mMode = mMode_RUN;
  pinMode(mPinBLE ,OUTPUT);
  Serial.begin( 9600 );
  mySerial.begin( 9600 );
  Serial.println("#Start-setup-SS");
  init_BME280();
  setup_watchdog(6 );                    // WDT setting
  //wait
  delay(1000 );
  proc_getSSerial(); //cear-buff
}
//
void read_bme280(){
  while (BME280.isMeasuring()) {
  }
  BME280.readMeasurements(); 
}

//
void loop() {
  //const int iWait =2000;
//  delay(100);
//  Serial.println( "mTimer="+ String(mTimer) + ",millis=" + String(millis()) );
  if(mMode ==mMode_RUN){
      read_bme280();
      print_bme280();
      if(mTimer_runMax <= 0 ){
          mTimer_runMax =  ((uint32_t)mMax_runSec * 1000) + millis();
      }
      if(millis() < mTimer_runMax){
          digitalWrite(mPinBLE, HIGH);
          proc_sendCmd();
      }else{
          mTimer_runMax=0;
          digitalWrite(mPinBLE,LOW  );
          mMode = mMode_WAIT;
      }
  }else{
    mMode = mMode_RUN;
    for(int i=0; i< mNextSec ; i++){
        system_sleep();                       // power down ,wake up WDT 
        //Serial.print("i=");
        //delay(15);
        //Serial.println(i);
        //delay(15);
    }
  }
  
}

//
void system_sleep() {
  cbi(ADCSRA,ADEN);                     // ADC power off
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);  // power down mode
  sleep_enable();
  sleep_mode();                         // sleep
  sleep_disable();                      // WDT time up
  sbi(ADCSRA,ADEN);                     // ADC ON
}

// WDT setting, param : time
// 0=16ms, 1=32ms,2=64ms,3=128ms,4=250ms,5=500ms
// 6=1 sec,7=2 sec, 8=4 sec, 9= 8sec
void setup_watchdog(int ii) {           // 
  byte bb;
  int ww;
  if (ii > 9 ) ii=9;
  bb=ii & 7;
  if (ii > 7) bb|= (1<<5);
  bb|= (1<<WDCE);
  ww=bb;
  MCUSR &= ~(1<<WDRF);
  // start timed sequence
  WDTCSR |= (1<<WDCE) | (1<<WDE);
  // set new watchdog timeout value
  WDTCSR = bb;
  WDTCSR |= _BV(WDIE);
}  //setup_watchdog

ISR(WDT_vect) {                         // WDT, time Up process
  wdt_cycle++;           
}






