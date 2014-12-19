#include <PlainFFT.h>

PlainFFT FFT = PlainFFT(); /* Create FFT object */

/* 
These values can be changed in order to evaluate the functions 
*/
#define SAMPLE_SIZE 64
#define LEDS 5
#define PDS 4
#define PD_HIST_LENGTH 5
#define NONBLOCK_2_BLOCK_PERCENTAGE 40
#define BLOCK_2_NONBLOCK_PERCENTAGE -50
double signalFrequency = 1000;
double samplingFrequency = 20000;
/* 
These are the input and output vectors 
Input vectors receive computed results from FFT
*/
double vImag[SAMPLE_SIZE];
double vReal[SAMPLE_SIZE];

double adcVals0[SAMPLE_SIZE];        //values from the ADC
double adcVals1[SAMPLE_SIZE];        //values from the ADC
double adcVals2[SAMPLE_SIZE];        //values from the ADC
double adcVals3[SAMPLE_SIZE];        //values from the ADC

volatile boolean adcBusy;
int freq_2187_db;
int freq_2500_db;
int freq_3750_db;
int freq_5000_db;
int freq_5625_db;

//shadow hist
boolean start = false;
int hist_counter = 0;
int pd_previous[PDS][LEDS];
int pd_current[PDS][LEDS];
int shadow_maps_previous[PDS][LEDS];
int shadow_maps_current[PDS][LEDS];

int get_shadow_map();

void setup() {
  Serial.begin(9600); // use the serial port
  
  //set up the timer
  TCCR1B = 0;                //stop the timer
  TCCR1A = 0;
  TIFR1 = 0xFF;              //ensure all interrupt flags are cleared
  OCR1A = 99;
  //timer runs at 16MHz / 100 / 8 (prescaler 8) = 20kHz (50 us between samples)
  OCR1B = 99;
  cli();
  TCNT1 = 0;                 //clear the timer
  TIMSK1 = _BV(OCIE1B);      //enable timer interrupts
  sei();
  TCCR1B = _BV(WGM12) | _BV(CS11);    //start the timer, ctc mode, prescaler 8
  
  //set up ADC
  ADMUX = (1<<REFS0);
  // ADC Enable and prescaler of 4
  ADCSRA = (1<<ADEN)|(1<<ADPS1);
}

 
// read adc value
int adc_read(uint8_t ch)
{
    // select the corresponding channel 0~7
    // ANDing with '7' will always keep the value
    // of 'ch' between 0 and 7
    ch &= 0b00000111;  // AND operation with 7
    ADMUX = (ADMUX & 0xF8)|ch;     // clears the bottom 3 bits before ORing
 
    // start single conversion
    // write '1' to ADSC
    ADCSRA |= (1<<ADSC);
 
    // wait for conversion to complete
    // ADSC becomes '0' again
    // till then, run loop continuously
    while(ADCSRA & (1<<ADSC));
 
    return (ADC);
}

void get_shadow_map(boolean pd0, boolean pd1, boolean pd2, boolean pd3)
{
    int i,j;
    //PD0
    if (pd0){
      Serial.println(" ");
      for (i = 0; i < SAMPLE_SIZE; i++) {
        vReal[i] = adcVals0[i];
        vImag[i] = 0;
        Serial.println(vReal[i]);
      }
      FFT.Windowing(vReal, SAMPLE_SIZE, FFT_WIN_TYP_HAMMING, FFT_FORWARD);	/* Weigh data */
      FFT.Compute(vReal, vImag, SAMPLE_SIZE, FFT_FORWARD); /* Compute FFT */
      FFT.ComplexToMagnitude(vReal, vImag, SAMPLE_SIZE); /* Compute magnitudes */
      int peak_index = FFT.MajorPeak(vReal, SAMPLE_SIZE, samplingFrequency);
  //    for (i = 0; i < SAMPLE_SIZE/2; i++) {
  //        j = vReal[i];
  //        Serial.println(j);
  //    }
      freq_2187_db = vReal[7];
      freq_2500_db = vReal[8];
      freq_3750_db = vReal[12];
      freq_5000_db = vReal[16];
      freq_5625_db = vReal[18];
//      print_PD_freq_DB(0);
      Serial.println(peak_index);
      if (freq_2187_db > 0){
        start = true;
      }else{
        start = false;
      }
      start = true;
      if (start){
        pd_current[0][0] = pd_current[0][0] + freq_2187_db;
        pd_current[0][1] = pd_current[0][1] + freq_2500_db;
        pd_current[0][2] = pd_current[0][2] + freq_3750_db;
        pd_current[0][3] = pd_current[0][3] + freq_5000_db;
        pd_current[0][4] = pd_current[0][4] + freq_5625_db;
      }
    }
    
    //PD1
    if (pd1){
      for (i = 0; i < SAMPLE_SIZE; i++) {
          vReal[i] = adcVals1[i];
          vImag[i] = 0;
      }
      FFT.Windowing(vReal, SAMPLE_SIZE, FFT_WIN_TYP_HAMMING, FFT_FORWARD);	/* Weigh data */
      FFT.Compute(vReal, vImag, SAMPLE_SIZE, FFT_FORWARD); /* Compute FFT */
      FFT.ComplexToMagnitude(vReal, vImag, SAMPLE_SIZE); /* Compute magnitudes */
      int peak_index = FFT.MajorPeak(vReal, SAMPLE_SIZE, samplingFrequency);
  //    for (i = 0; i < SAMPLE_SIZE/2; i++) {
  //        j = vReal[i];
  //        Serial.println(j);
  //    }
      freq_2187_db = vReal[7];
      freq_2500_db = vReal[8];
      freq_3750_db = vReal[12];
      freq_5000_db = vReal[16];
      freq_5625_db = vReal[18];
//      print_PD_freq_DB(1);
//      Serial.println(peak_index);
      if (freq_2187_db > 0){
        start = true;
      }else{
        start = false;
      }
      
      if (start){
        pd_current[1][0] = pd_current[1][0] + freq_2187_db;
        pd_current[1][1] = pd_current[1][1] + freq_2500_db;
        pd_current[1][2] = pd_current[1][2] + freq_3750_db;
        pd_current[1][3] = pd_current[1][3] + freq_5000_db;
        pd_current[1][4] = pd_current[1][4] + freq_5625_db;
      }
    }
    
    //PD2
    if(pd2){
      for (i = 0; i < SAMPLE_SIZE; i++) {
          vReal[i] = adcVals2[i];
          vImag[i] = 0;
      }
      FFT.Windowing(vReal, SAMPLE_SIZE, FFT_WIN_TYP_HAMMING, FFT_FORWARD);	/* Weigh data */
      FFT.Compute(vReal, vImag, SAMPLE_SIZE, FFT_FORWARD); /* Compute FFT */
      FFT.ComplexToMagnitude(vReal, vImag, SAMPLE_SIZE); /* Compute magnitudes */
      int peak_index = FFT.MajorPeak(vReal, SAMPLE_SIZE, samplingFrequency);
  //    for (i = 0; i < SAMPLE_SIZE/2; i++) {
  //        j = vReal[i];
  //        Serial.println(j);
  //    }
      freq_2187_db = vReal[7];
      freq_2500_db = vReal[8];
      freq_3750_db = vReal[12];
      freq_5000_db = vReal[16];
      freq_5625_db = vReal[18];
//      print_PD_freq_DB(2);
//      Serial.println(peak_index);
      if (freq_2187_db > 0){
        start = true;
      }else{
        start = false;
      }
      
      if (start){
        pd_current[2][0] = pd_current[2][0] + freq_2187_db;
        pd_current[2][1] = pd_current[2][1] + freq_2500_db;
        pd_current[2][2] = pd_current[2][2] + freq_3750_db;
        pd_current[2][3] = pd_current[2][3] + freq_5000_db;
        pd_current[2][4] = pd_current[2][4] + freq_5625_db;
      }
    }
    
    //PD3
    if(pd3){
//      Serial.println(" ");
      for (i = 0; i < SAMPLE_SIZE; i++) {
          vReal[i] = adcVals3[i];
          vImag[i] = 0;
//          Serial.println(vReal[i]);
      }
      FFT.Windowing(vReal, SAMPLE_SIZE, FFT_WIN_TYP_HAMMING, FFT_FORWARD);	/* Weigh data */
      FFT.Compute(vReal, vImag, SAMPLE_SIZE, FFT_FORWARD); /* Compute FFT */
      FFT.ComplexToMagnitude(vReal, vImag, SAMPLE_SIZE); /* Compute magnitudes */
      int peak_index = FFT.MajorPeak(vReal, SAMPLE_SIZE, samplingFrequency);
  //    for (i = 0; i < SAMPLE_SIZE/2; i++) {
  //        j = vReal[i];
  //        Serial.println(j);
  //    }
      freq_2187_db = vReal[7];
      freq_2500_db = vReal[8];
      freq_3750_db = vReal[12];
      freq_5000_db = vReal[16];
      freq_5625_db = vReal[18];
//      print_PD_freq_DB(3);
//      Serial.println(peak_index);
      if (freq_2187_db > 0){
        start = true;
      }else{
        start = false;
      }
      
      if (start){
        pd_current[3][0] = pd_current[3][0] + freq_2187_db;
        pd_current[3][1] = pd_current[3][1] + freq_2500_db;
        pd_current[3][2] = pd_current[3][2] + freq_3750_db;
        pd_current[3][3] = pd_current[3][3] + freq_5000_db;
        pd_current[3][4] = pd_current[3][4] + freq_5625_db;
      }
    }
    
    if (start){
      hist_counter++;
//      Serial.println(hist_counter);
    }
    
//    if (hist_counter == PD_HIST_LENGTH){
//      hist_counter = 0;
//      if (pd_previous[0][0] > 0){
//        int i,j;
//        for (i = 0; i < 1; i++){
//          for (j = 0; j < LEDS; j++){
//            double change_ratio = (pd_previous[i][j] - pd_current[i][j]) * 100.0 / pd_previous[i][j];
////            Serial.println(pd_previous[i][j]);
////            Serial.println(pd_current[i][j]);
////            Serial.println(change_ratio);
//            if (shadow_maps_previous[i][j] == 0){
//              if (change_ratio > NONBLOCK_2_BLOCK_PERCENTAGE){
//                shadow_maps_current[i][j] = 1;
//              }else{
//                shadow_maps_current[i][j] = 0;
//              }
//            }else{
//              if (change_ratio < BLOCK_2_NONBLOCK_PERCENTAGE){
//                shadow_maps_current[i][j] = 0;
//              }else{
//                shadow_maps_current[i][j] = 1;
//              }
//            }
//            //do something
//            
//            //
//            shadow_maps_previous[i][j] = shadow_maps_current[i][j];
//            pd_previous[i][j] = pd_current[i][j];
//            pd_current[i][j]= 0;
//          }
//        }
//        for (i = 0; i < 1; i++){
//          for (j = 0; j < LEDS; j++){
//            Serial.print(shadow_maps_current[i][j]);
//            Serial.print(" ");
//          }
//          Serial.println(" ");
//        }
////        Serial.println(" ");
//      }else{
//        int i,j;
//        for (i = 0; i < PDS; i++){
//          for (j = 0; j < LEDS; j++){
//            pd_previous[i][j] = pd_current[i][j];
//            pd_current[i][j]= 0;
//          }
//        }
//      }
//    }
}

int adc_result;

// the loop function runs over and over again forever
void loop() {
  delay(1000);
  unsigned long tStart;                     //timer interrupt time from micros()
  unsigned long tEnd;                       //adc interrupt time from micros()
  int i,j;
  for (i = 0; i < PDS; i++){
    for (j = 0; j < LEDS; j++){
      pd_previous[i][j] = 0;
      pd_current[i][j]= 0;
      shadow_maps_previous[i][j]= 0;
      shadow_maps_current[i][j]= 0;
    }
  }
  
  while(1){
    if(adcBusy){
      adcBusy = false;
      if (i < SAMPLE_SIZE){
//        tStart = micros();
        //Read sensor data
        adcVals0[i] =  adc_read(0);            //save the analog reading
//        adcVals1[i] =  adc_read(1);            //save the analog reading
//        adcVals2[i] =  adc_read(2);            //save the analog reading
//        adcVals3[i] =  adc_read(3);            //save the analog reading
//        tEnd = micros();
        i++;
      }else{
        i = 0;
//        tStart = micros();
//        Serial.println("start");
        get_shadow_map(true, false, false, false);
//        tEnd = micros();
//        Serial.println();
//        Serial.print("Major frequency (Hz): ");
//        Serial.println(peak_index);
//        Serial.println(tEnd-tStart);

//        int j = 0;
//        Serial.println("start");
//        for (j=0; j< SAMPLE_SIZE; j++){
//          adc_result = adcVals0[j];
//          Serial.println(adc_result);
//        }
        
        Serial.flush();
//        delay(1000);
      }
    }   
  }
}

void print_PD_freq_DB(int PD_index){
//    Serial.print("PD");
//    Serial.print(PD_index);
//    Serial.println(":");
    Serial.print(" ");
    Serial.print(freq_2187_db);
    Serial.print(" ");
    Serial.print(freq_2500_db);
    Serial.print(" ");
    Serial.print(freq_3750_db);
    Serial.print(" ");
    Serial.print(freq_5000_db);
    Serial.print(" ");
    Serial.print(freq_5625_db);
    Serial.println(" ");
}

ISR(TIMER1_COMPB_vect)
{
    adcBusy = true;
}
