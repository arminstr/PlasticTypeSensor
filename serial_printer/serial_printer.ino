#include <SPI.h>

char buf [100];
volatile byte pos;
volatile boolean process_it;

void setup() {
  Serial.begin(250000);

  // turn on SPI in slave mode
  SPCR |= _BV(SPE);
  
  // turn on interrupts
  SPCR |= _BV(SPIE);

  pos = 0;
  process_it = false;
}

// SPI interrupt routine
ISR (SPI_STC_vect)
{
  char c = SPDR;
  // add to buffer if room
  if (pos < sizeof buf)
    {
    buf [pos++] = c;
    
    // example: newline means time to process buffer
    if (c == '\n')
      process_it = true;
      
    }  // end of room available
}

void loop() {
  if (process_it) {
      buf [pos] = 0;  
      Serial.print(buf);
      pos = 0;
      process_it = false;
  }  // end of flag set
}
