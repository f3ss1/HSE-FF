#define PIXELS 1200 // default: 800 (3648 max). Use 800 for the Uno.
 
char cmdBuffer[16];
int cmdIndex;
int exposureTime = 12; // play with this for a good exposure time

bool flag = false;

unsigned int data;
unsigned long time1, time2;
byte V;
byte data_V[PIXELS];
 
#define CLOCK PORTD  // use PORTD for the UNO (make room for MOSI, MISO, and SCK for the 23LC1024)
// PORTD are digital pins 7 to 0? Seems to be OK-ish only for 7 to 3.
#include <util/delay_basic.h>


/* Pinout seems to be a lot of fun:
* 1 is for nothing;
* 2 is for nothing;
* 3 is for nothing;
* 4 is for 2;
* 5 is for 5; OK
* 6 is for 6; OK
* 7 is for ;
* 8 is for 
*/

#define ICG (1<<5) // PD5 (Digital Pin 5)
#define MCLK (1<<3) // PD3 (Digital Pin 3)
#define SH (1<<6)   // PD6 (Digital Pin 6) 

#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit)) // CBI is somewhat standard solution to make LOW?
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit)) // Same as CBI but for HIGH



void setup(){

  ADCSRA |= (1 << ADEN); // Включаем АЦП

  ADMUX |= (0 << REFS1)|(1 << REFS0) // выставляем опорное напряжение Vcc
  |(1 << MUX0)|(1 << MUX1)|(0 << MUX2)|(0 << MUX3); // снимать сигнал будем с входа AC3

  // Init CLOCK lines to OUTPUT.

  DDRD |= (SH | ICG | MCLK); // Set the clock lines to output mode

  CLOCK |= ICG; // ICG to HIGH

  // Enable the serial port.
  Serial.begin(115200);
  
  TCCR2A = _BV(COM2A1) | _BV(COM2B1) | _BV(WGM21) | _BV(WGM20); // inverting mode, fast PWM
  TCCR2B = _BV(WGM22) | _BV(CS20); // no prescaling
  OCR2A = 15; // counter limit: 255
  OCR2B = 7;

  cbi(ADCSRA, ADPS2); // sbi is HIGH, cbi is LOW
  sbi(ADCSRA, ADPS1);
  sbi(ADCSRA, ADPS0);

}

void loop() {
  
  /* 0.0047031 = Vcc / 1024 */
  //Serial.print("V = "); Serial.print(V, 3); Serial.println(" V");
  //Serial.write("Ti Pidor");
  /*if (not flag) {
    readCCD(PIXELS, exposureTime);
    for (int i = 0; i < PIXELS; i++){
      //Serial.println(i);
      Serial.println(data_V[i]);
    }
    flag = true;
  }
  */
  CLOCK &= ~ICG; // set ICG low
  delayMicroseconds(1); // timing requirement (1000 ns max)
  CLOCK |= SH;   // set SH high
  delayMicroseconds(2); // timing requirement (1000 ns max)
  CLOCK &= ~SH;  // set SH low
  delayMicroseconds(5); // timing requirement (min 1000ns, typ 5000 ns)
  CLOCK |= ICG;  // set ICG high
  _delay_loop_1(1);  // wait a tick (supposed to be t4 in the datasheet, 20 ns)
  delayMicroseconds(14780);   // delay an entire cycle
}

void readCCD(unsigned int pixels, int expos){
  //byte warmUps=20; // number of readings to adjust to new exposure time (set to >10)
  /*for(uint32_t x=0; x < pixels; x++){
    Serial.print(x);
    sram.WriteUnsignedInt(2 * x, 0); // initialize buffer
  }
  */
  Serial.println("Warmup stated");

  // Warmup
  for (int y=0; y<20; y++){
    CLOCK &= ~ICG; // set ICG low
    delayMicroseconds(1); // timing requirement (1000 ns max)
    CLOCK |= SH;   // set SH high
    delayMicroseconds(2); // timing requirement (1000 ns max)
    CLOCK &= ~SH;  // set SH low
    delayMicroseconds(5); // timing requirement (min 1000ns, typ 5000 ns)
    CLOCK |= ICG;  // set ICG high
    _delay_loop_1(1);  // wait a tick (supposed to be t4 in the datasheet, 20 ns)
    delayMicroseconds(14780);   // delay an entire cycle
  }

  Serial.println("Warmup ended");

  // Main cycle reborn

  CLOCK &= ~ICG; // set ICG low
  delayMicroseconds(1); // timing requirement (1000 ns max)
  CLOCK |= SH;   // set SH high
  delayMicroseconds(2); // timing requirement (1000 ns max)
  CLOCK &= ~SH;  // set SH low
  delayMicroseconds(5); // timing requirement (min 1000ns, typ 5000 ns)
  CLOCK |= ICG;  // set ICG high
  //_delay_loop_1(1);  // wait a tick (supposed to be t4 in the datasheet, 20 ns)

  //delayMicroseconds(128);
  
  for (int i = 0; i < pixels; i++) {
    V = analogRead(A0); // Переводим в вольты
    data_V[i] = V;
    //delayMicroseconds(4);
  }
  // Main cycle
  /*
    
    delayMicroseconds(128 + (4 * y));   // delay until the correct first pixel

    for(int i = y; i < pixels; i += 24){ // should amount to 152 pixel readings
      sram.WriteUnsignedInt(i * 2, analogRead(A0)); // takes about 64 usec
      Serial.print(analogRead(A0)); Serial.print(";");
      delayMicroseconds(40); // delay between readings
    }
    delayMicroseconds(4a * (24 - y) + 52); // skip the last pixels plus trailing dummy outputs
    delay(expos);
  }
  */
}


/*
void sendData(void){ // Sends diode array data to serial monitor
  for (uint32_t x=0;x<PIXELS;x++){
    if(x>0){Serial.print(";");} // for CSV format
    Serial.print((float)analogRead(A0));
  }
  Serial.println();
}
*/
 
/*void loop(){
  //#ifdef MENUMODE
  // Begin serial commands
    if (Serial.available()){  // Listening for serial monitor commands
      cmdBuffer[cmdIndex++] = Serial.read();
    }
    if (cmdBuffer[0] == 'r'){ // read the CCD (and send data to the serial monitor)
      //readCCD(PIXELS,exposureTime);
      sendData();
      Serial.println("Enter choice:");
    }
    else if (cmdBuffer[0] == 'e'){
      if (--exposureTime < 0) exposureTime = 0;
      Serial.print("Exposure time ");
      Serial.println(exposureTime);
    }
    else if (cmdBuffer[0] == 'E'){
      if (++exposureTime > 200) exposureTime = 200;
      Serial.print("Exposure time ");
      Serial.println(exposureTime);
    }
    cmdBuffer[0] = '\0'; // erase the command buffer
    cmdIndex = 0;
  //#endif // end of MENUMODE serial commands
  */
  /*
  #ifndef MENUMODE
    readCCD(PIXELS,exposureTime);
    sendData();
    //delay(1000);  // optional delay for continuous reading
    
  #endif

}
*/