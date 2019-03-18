/*************************
 * Cloud Controller v0.1
 * PD0 = Top Button
 * PD4 = Mid Button
 * PD7 = Bottom Button
 * 
 * PC2 = Slide Pot
 * PC1 = Right Pot
 * PC0 = Left Pot
 ***********************/
#include <FastLED.h>
#define NUM_LEDS 194
#define DATA_PIN 5 
CRGB leds[NUM_LEDS];

volatile int pot_slider_val;
volatile int pot_R_val;
volatile int pot_L_val;

ISR(ADC_vect){
  cli();

  uint8_t current_MUX_setting = 0b0001111 & ADMUX; //Read out MUX[3:0] by masking ADMUX.
  switch(current_MUX_setting)
  {
    case(0): //This means we've just read ADC0 i.e. Left Potentiometer
    pot_L_val = ADCL | (ADCH << 8); //Read conversion result
    ADMUX |= (1 << MUX0); //Set MUX to ADC1
    ADCSRA |= (1 << ADSC); //ADC start conversion
    break;
    
    case(1): //This means we've just read ADC1 i.e. Right Potentiometer
    pot_R_val = ADCL | (ADCH << 8); //Read conversion result
    ADMUX |= (1 << MUX1); //Set MUX to ADC2
    ADMUX &= ~(1 << MUX0);
    ADCSRA |= (1 << ADSC); //ADC start conversion
    break;
    
    case(2): //This means we've just read ADC2 i.e. Slide Potentiometer
    pot_slider_val = ADCL | (ADCH << 8);
    //No further conversions after this one. Next one will be kicked off again by Timer1
    break;
  }

  sei();
}

ISR(TIMER1_COMPA_vect){
  cli();

  ADMUX &= 0b11110000; //Clear MUX[3:0] to set first conversion to ADC0 
  ADCSRA |= (1 << ADSC); //ADC start conversion
  
  sei();
}

uint8_t cur_state, changed_pins, prev_state;
ISR(PCINT2_vect){
  cli();

  cur_state = PIND;
  changed_pins = cur_state ^ prev_state;
  if( !( cur_state & (1 << PD0) ) && changed_pins & (1 << PD0) )
  {
    Serial.println("You pushed Button A");
  }
  
  if( !( cur_state & (1 << PD4) ) && changed_pins & (1 << PD4) )
  {
    Serial.println("You pushed Button B");
  }

  if( !( cur_state & (1 << PD7) ) && changed_pins & (1 << PD7) )
  {
    Serial.println("You pushed Button C");
  }
  prev_state = cur_state;

  sei();
}

void setup() {
  Serial.begin(9600);

  //BEGIN ADC SETUP
  DDRC = 0; //Set all Pin Cs to input
  ADMUX = 0; //REFS1 & REFS0 = 0b00 = Use AREF; ADLAR = 0 = Result is right oriented; MUX[3:0] = 0b0000 = set to ADC0.
  ADMUX |= (1 << REFS0); 
  ADCSRA =  (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); //Set prescaler to 128. 16Mhz/128 = 125kHz. 10bit results are not reliable over 200kHz
  ADCSRA |= (1 << ADIE); //Enable interrupt flag. 
  ADCSRA |= (1 << ADEN); //Enable ADC

  pot_slider_val = 0;
  pot_R_val = 0;
  pot_L_val = 0;
  //END ADC SETUP


  //BEGIN TIMER SETUP
  //Set up timer 1 for Clear Timer on Compare mode
  //CTC (Mode 4): WGM13 = 0; WGM12 = 1; WGM11 = 0; WGM10 = 0
  //Internal clock - 64x prescaler: CS12 = 0; CS11 = 1; CS10 = 1; See Table 16-5
  //f_OC1A = f_clkIO/( prescaler * ( 1 + OCR1A ) ) Page 132
  //f_clkIO = 16MHz on Arduino Genuino
  TCCR1A = TCCR1B = 0;
  OCR1A = 12499; //Setting timer to fire every 100ms.
  TCCR1B = (1 << WGM12) | (1 << CS11) | (1 << CS10);

  TCNT1 = 0;
  TIMSK1 |= (1 << OCIE1A); //Output Compare A Match Interrupt Enable
  //END TIMER SETUP


  //BEGIN BUTTON SETUP
  DDRD = (1 << PD1) ; //DDRx = 0 means pin is an input, DDRx = 1 means pin is an output. PD1 is the data pin for fastLED.
  PORTD = (1 << PD0) | (1 << PD4) | (1 << PD7);
  PCMSK2 = (1 << PD0) | (1 << PD4) | (1 << PD7); //Enable pin change interrrupt on PD0, PD4 and PD7
  PCICR = (1 << PCIE2); //PCIE2 is the interrupt enable for PCINT[23:16]
  //END BUTTON SETUP

  FastLED.addLeds<WS2812B, DATA_PIN, GRB>(leds, NUM_LEDS);
  FastLED.setBrightness(50);
  
  sei();

}

void loop() {
  int pos = map(pot_slider_val, 0, 1023, 0, 194);
  for( int i = 0; i < NUM_LEDS; i++) {
      if(abs(i - pos) < 11){
        leds[i] = CHSV(map(pot_slider_val,0,1023,0,255),255,255);
      }else{
        leds[i] = CRGB::Black;
      }
  }
  FastLED.show();
  FastLED.delay(1000/40);
}
