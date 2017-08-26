#define ESCNb  4

ESC ESC0, ESC1, ESC2, ESC3; // Used inside and outside ISR, so it must be volatile
int g_currESC = -1;

inline void SetPWM_f5(volatile uint16_t *TCNTn, volatile uint16_t* OCRnA)
{
  static bool firstLoop = true;
  if ( firstLoop) {
    g_currESC = 0;
    PORTD = B00010000;
    firstLoop = false;
  }

  if ( g_currESC == 0 ) {
    PORTD ^= B00110000; // Reset pin PD4 and set pin PD5 using XOR
    *OCRnA = ESC1.GetTicks();
  } else if ( g_currESC == 1 ) {
    PORTD ^= B01100000; // Reset pin PD5 and set pin PD6 using XOR
    *OCRnA = ESC2.GetTicks();
  } else if ( g_currESC == 2 ) {
    PORTD ^= B11000000; // Reset pin PD6 and set pin 7 using XOR
    *OCRnA = ESC3.GetTicks();
  } else {
    PORTD ^= B10010000; // Reset pin PD7 and set pin PD4 using XOR
    g_currESC = -1;
    *OCRnA = ESC0.GetTicks();
  }
  *TCNTn = 0; // Reset timer
  g_currESC++;
}
