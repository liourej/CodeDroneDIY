#define ESCNb  4

ESC ESC0, ESC1, ESC2, ESC3; // Used inside and outside ISR, so it must be volatile
ESC ESCList[ESCNb] = { ESC0, ESC1, ESC2, ESC3};
ESC ESCListTimer[ESCNb] = { ESC0, ESC1, ESC2, ESC3};
volatile bool g_NewVal = false;
int g_currESC = -1;
const uint16_t g_T = usToTicks((1 / 400) * 1000000); // Refresh period (ticks) 400Hz working

inline int GetMinESC(int _minCase, int _maxCase, ESC _ESCs[ESCNb])
{
  int idMinESC = 0;
  uint16_t ticksTemp = 65535;
  for (int i = _minCase; i < _maxCase; i++)
  {
    if ( ticksTemp > _ESCs[i].GetTicks() )
    {
      ticksTemp = _ESCs[i].GetTicks();
      idMinESC = i;
    }
  }
  return idMinESC;
}

inline void SortESC(ESC _ESCs[ESCNb])
{
  ESC ESCtemp;
  int ESCCurr = 0;
  int minESC = 0;
  while ( ESCCurr < ESCNb )
  {
    minESC = GetMinESC( ESCCurr, ESCNb, _ESCs);
    ESCtemp = _ESCs[ESCCurr];
    _ESCs[ESCCurr] = _ESCs[minESC];
    _ESCs[minESC] = ESCtemp;
    ESCCurr++;
  }
}

inline void SetPWM_f4(volatile uint16_t *TCNTn, volatile uint16_t* OCRnA)
{
  // Set all ESC to HIGH
  if ( g_currESC >= ESCNb )
  {
    g_currESC = 0;

    if ( g_NewVal )
    {
      for (int i = 0; i < ESCNb; i++)
        ESCListTimer[i] = ESCList[i];
      g_NewVal = false;
    }

    *OCRnA = ESCListTimer[g_currESC].GetTicks();
    *TCNTn = 0; // Reset timer
    PORTB = B00001111; // sets digital pins 0, 1, 2, 3 HIGH
    return;
  }

  byte Mask;
  Mask = PORTB; // Get PORTB state
  Mask ^= ( 1 << ESCListTimer[g_currESC].PORTBPin ); // Reset current pin in mask
  g_currESC++;
  while ( ( (ESCListTimer[g_currESC].GetTicks() < *TCNTn ) || ((*OCRnA - ESCListTimer[g_currESC].GetTicks()) < 100) ) && (g_currESC < ESCNb) ) // Check if there is more pin to reset
  {
    Mask ^= (1 << ESCListTimer[g_currESC].PORTBPin);
    g_currESC++;
  }

  PORTB &= Mask;

  if ( g_currESC < ESCNb )
  {
    *OCRnA = ESCListTimer[g_currESC].GetTicks();
  } else {
    // Wait for period
    *OCRnA = g_T - *TCNTn;//usToTicks(3333 - ESCList[1].GetPWM()); // 300Hz TODO: change ESCList[1] by last one
    // 1440 -> Refresh period 1/(1060+1440) = 400Hz to 1/(1860+1440) = 244 Hz
    *TCNTn = 0; // Reset timer
  }
}

inline void SetPWM_f5(volatile uint16_t *TCNTn, volatile uint16_t* OCRnA)
{
  static bool firstLoop = true;
  if ( firstLoop) {
    PORTB = B00000001;
    firstLoop = false;
  }

  if ( g_currESC == 0 ) {
    PORTB = B00000010; // Reset pin 0 and set pin 1 using XOR
    //PORTB ^= B00000011; // Reset pin 0 and set pin 1 using XOR
    *OCRnA = ESC1.GetTicks();
  } else if ( g_currESC == 1 ) {
    PORTB = B00000100;
   // PORTB ^= B00000110; // Reset pin 1 and set pin 0 using XOR
    *OCRnA = ESC2.GetTicks();
  } else if ( g_currESC == 2 ) {
    PORTB = B00001000;
   // PORTB ^= B00001100; // Reset pin 2 and set pin 1 using XOR
    *OCRnA = ESC3.GetTicks();
  } else {
    PORTB = B00000001;
   // PORTB ^= B00001001; // Reset pin 3 and set pin 0 using XOR
    g_currESC = -1;
    *OCRnA = ESC0.GetTicks();
  }
  *TCNTn = 0; // Reset timer
  g_currESC++;
}
