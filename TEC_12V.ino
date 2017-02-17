  /*
  TEC_12V
  Calculate PID parameters, drive H-Bridge with two PWM outputs, one for positive, one for negative.
  Sensor uses 10K resistor to +5V and 10K thermistor. At 10 bits, about 10 counts / degree C, 512 = 25C
  PID constants determined empirically. Large test TEC needs about 1A at low voltage
  Large TEC: KP 10 KI 1/2, KD 0. Setting is in counts,

*/
#include <math.h>
#include <Wire.h>

int lookup( float fTarget );

// temperature settings for 10K3A thermistor from Thermistor spreadsheet
float fTempLookup[] = { 10.0, 15.0, 20.0, 25.0, 30.0, 35.0, 40.0, 45.0, 50.0, 55.0, 60.0, 65.0, 70.0, 75.0, 80.0, 85.0, 90.0, 95.0, 100.0 };
int nTempLookup[] = { 818, 777, 731, 683, 632, 580, 528, 477, 429, 383, 340, 301, 266, 234, 206, 181, 159, 139, 122 };

#define T10C  818
#define T15C  777
#define T20C  731
#define T25C  683
#define T30C  632
#define T35C  580
#define T40C  528
#define T45C  477
#define T50C  429
#define T55C  383
#define T60C  340
#define T65C  301
#define T70C  266
#define T75C  234
#define T80C  206
#define T85C  181
#define T90C  159
#define T95C  139
#define T100C 122

#define ADCNUM 1024.0 
#define RPU 4990.0    // 4.99K
#define SH_A 1.129241E-03
#define SH_B 2.341077E-04
#define SH_C 8.775468E-08
#define KEL 273.15    // 0C

#define I2C_ADDR 0x0d
#define DEBUG_MODE 0

#define NAVE 32
#define OUT_MAX 255
int ledPin = 13;
int PWM1Pin = 10; // 9 is OC1A, 10 is OC1B
int PWM2Pin = 9;
int ThermPin = A0;

volatile int Set = T25C;   // Set temperature here
volatile byte tecOn = 0;    // 0 = OFF, 1 = ON

/*  PID parameters tuned for large TEC: 5V, 2A. Must set V+ to 7V */
/*
  #define INT_MAX 400   // Note max Int range is INT_MAX * KI_NUM / KI_DEN should probably ~ 255
  #define KP      5
  #define KI_NUM  1   // Numerator since this may be a fraction
  #define KI_DEN  2   // Denominator
  #define KD      0
  #define LOOP_TIME 300
*/
/*  PID parameters tuned for Galaxy 12V TEC: Set V+ to 12V */
#define INT_MAX 2000   // Note max Int range is INT_MAX * KI_NUM / KI_DEN should be ~ 255
#define KP      10
#define KI_NUM  1   // Numerator since this may be a fraction
#define KI_DEN  8   // Denominator
#define KD      0
#define LOOP_TIME 200

int out;
int err, Pterm, Iterm = 0, Dterm, PID, oldErr;

volatile float g_fLastTempReading = 0;
byte g_Control = 0;
float g_fSetpoint = 0;

// the setup routine runs once when you press reset:
void setup() {
  // initialize serial communication at 9600 bits per second:
#if DEBUG_MODE
  Serial.begin(9600);
#endif
  pinMode(ledPin, OUTPUT);
  pinMode(PWM1Pin, OUTPUT);
  pinMode(PWM2Pin, OUTPUT);

  Wire.begin(I2C_ADDR);
  Wire.onReceive( i2cReceiveEvent );
  Wire.onRequest( i2cRequestEvent );
}

// the loop routine runs over and over again forever:
void loop() {
  int i;
  int adcTemp;
  float f_t;  //Temperature
  float f_r;  //resistance
  
  // read the input on analog pin 0:
  adcTemp = 0;
  for (i = 0; i < NAVE; i++)
  {
    adcTemp += analogRead(ThermPin);
  }
  adcTemp /= NAVE;

  oldErr = err;    // for D term
  err = adcTemp - Set ;
  Iterm += err;
  Dterm = err - oldErr;
  if (Iterm > INT_MAX) Iterm = INT_MAX;   // Limit Iterm
  if (Iterm < -INT_MAX) Iterm = -INT_MAX;
  Pterm = err;
  PID = KP * Pterm  + KI_NUM * Iterm / KI_DEN  + Dterm * KD;
  out = PID ;

  if (tecOn == 0)  //Turn TEC OFF
  {
    out = 0;
  }

  // First clip the values
  if (out > OUT_MAX) out = OUT_MAX;
  if (out < -OUT_MAX) out = -OUT_MAX;
  
  // Calculate thermistor resistance, then temperature using Steinhart-Hart 
  // T = 1/( A + B log(R) + C * log(R)^3 ) - 273.15
  f_r = RPU / ((ADCNUM / adcTemp) -1);
  f_t = (1/(SH_A + SH_B * log(f_r) + SH_C * pow(log(f_r), 3.0))) - KEL;
  g_fLastTempReading = f_t;
  
#if DEBUG_MODE
  Serial.print("T: ");
  Serial.print(f_t);
  Serial.print(" Err: ");
  Serial.print(err);
  Serial.print(" Out: ");
  Serial.println(out);
#endif

  // Build bidir PWM using 2 analogWrite() PWMs
  if (out > 0) //set both PWMs to 0
  {
    analogWrite(PWM2Pin, 0);
    analogWrite(PWM1Pin, out);
  }
  if (out == 0)
  {
    analogWrite(PWM1Pin, 0);
    analogWrite(PWM2Pin, 0);
  }
  if (out < 0)
  {
    analogWrite(PWM1Pin, 0);
    analogWrite(PWM2Pin, -out);
  }

  delay(LOOP_TIME);        // delay in between reads for stability
}

void i2cReceiveEvent(int numReceived) 
{
  if( numReceived < 5 )
    return;
    
  // looking for 5 bytes, control byte + 4 byte float
  g_Control = Wire.read();

  byte *pbyTemp = (byte *) &g_fSetpoint;
  pbyTemp[0] = Wire.read();
  pbyTemp[1] = Wire.read();
  pbyTemp[2] = Wire.read();
  pbyTemp[3] = Wire.read();

  if( g_fSetpoint < 20.0f )
    g_fSetpoint = 20.0f;
  else if ( g_fSetpoint > 100.0f )
    g_fSetpoint = 100.0f;

  // Calculate the resistance value that's closest to the requested setpoint
  Set = lookup( g_fSetpoint );
  Serial.print("Setpoint changed: ");
  Serial.println(g_fSetpoint);

  if( g_Control & 1 )
  {
    digitalWrite( ledPin, HIGH );
    tecOn = 1;
  }
  else
  {
    digitalWrite( ledPin, LOW );
    tecOn = 0;
  }
  return;
}

void i2cRequestEvent() 
{
  // Send back the current Temp
  byte *pbyTemp = (byte *) &g_fLastTempReading;
  int n = Wire.write( pbyTemp, 4 );
  return;
}

int lookup( float fTarget )
{
 int i;
 int nRtn;

  // Find the closest temp from the array without exceeding the target
  for( i = 0; i < (sizeof(fTempLookup)/sizeof(float)-1); i++ ) 
  {
    if( fTempLookup[i+1] > fTarget )
    {
      break;
    }
  }

  // Perform linear interpolation on the corresponding integer values
  float ratio = ( fTarget - fTempLookup[i] ) / (fTempLookup[i+1] - fTempLookup[i]);
  nRtn = (int) ( ratio * ( nTempLookup[i+1] - nTempLookup[i] ) + nTempLookup[i] );

  return nRtn;
}

