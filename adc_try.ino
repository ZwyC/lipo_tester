#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_ADS1015.h>

#define LCD_I2C_ADDRESS 0x3F

#define BATTERY_VOLT A3
#define FET_PIN      13

LiquidCrystal_I2C lcd(LCD_I2C_ADDRESS, 16, 2);
Adafruit_ADS1115 adc;

const unsigned long LcdDelay = 500;
unsigned long LcdTime = millis() + LcdDelay;
const unsigned long SerialDelay = 1000 * 15;
unsigned long SerialTime = millis() + SerialDelay;
const unsigned long AdcDelay = 500;
unsigned long AdcTime = millis() + AdcDelay;
const unsigned long CapacityDelay = 1000;
unsigned long CapacityTime = millis() + CapacityDelay;

int BRDisp = 0;

const float ref1v1 = 1.096;
const float ar_multi = 23000.0 / (23000.0+67930.0);
const float ref5v = ref1v1 / ar_multi;

int16_t adc0, adc1, adc2, adc3, adcdif23;
float multiplier = 0.015625F;

unsigned long prevMillis = 0;
unsigned long millisPassed = 0;

bool done = false;
bool discharge = false;
bool RChkDone = false;
float BattIR = 0.0;
float battery = 0.0;
float current = 0.0;
float mAh = 0.0;

void setup() {
  analogReference(INTERNAL);
  Serial.begin(115200);
  pinMode( FET_PIN, OUTPUT);
  pinMode( 13, OUTPUT);
  digitalWrite( FET_PIN, LOW);
  digitalWrite( 13, LOW);
  delay(100);

  initLcd();
  initAdc();
}

void loop() {
  updateAdc();

  if ( battery > 2.8) {
    if ( discharge == false) {
      delay( 1000*5);
      digitalWrite( FET_PIN, HIGH);
      digitalWrite( 13, HIGH);
      discharge = true;
    }
  } else if ( discharge == true) {
    digitalWrite( FET_PIN, LOW);
    digitalWrite( 13, LOW);
    done = true;
  }

  if ( CapacityTime < millis()) {
    if ( adcdif23 > 1.0 ) {
      millisPassed = millis() - prevMillis;
      current = float(adcdif23) * multiplier * 10.0;
      mAh += (current * millisPassed / 3600000.0);
      prevMillis = millis();
    } else {
      prevMillis = millis();
    }
    CapacityTime = millis() + CapacityDelay;
  }


  if ( RChkDone == false && adcdif23 > 1.0 && battery < 3.700 ) {
    float prev_battery = battery;
    unsigned long millisSkip = millis();

    current = float(adcdif23) * multiplier * 10.0;
    digitalWrite( FET_PIN, LOW);
    digitalWrite( 13, LOW);
    delay( 500);
    battery = 0.0;
    for ( int i = 0; i < 10; i++) {
      battery += analogRead(BATTERY_VOLT);
      delay(2);
    }
    battery /= 10.0;
    battery *= ref5v / 1024.0;
    BattIR = (( battery - prev_battery) / (current / 1000.0)) * 1000.0;
    RChkDone = true;
    digitalWrite( FET_PIN, HIGH);
    digitalWrite( 13, HIGH);
    prevMillis += (millis() - millisSkip);
    delay(100);
  }
  
  updateLcd();
  updateSerial();

}

void updateAdc() {
  if (AdcTime < millis()) {
    battery = 0.0;
    for ( int i = 0; i < 10; i++) {
      battery += analogRead(BATTERY_VOLT);
      delay(2);
    }
    battery /= 10.0;
    battery *= ref5v / 1024.0;
    //    battery = (analogRead(BATTERY_VOLT) * ref5v)/1023;
    //    adc0 = adc.readADC_SingleEnded(0);
    //    adc1 = adc.readADC_SingleEnded(1);
    //    adc2 = adc.readADC_SingleEnded(2);
    //    adc3 = adc.readADC_SingleEnded(3);
    adcdif23 = adc.readADC_Differential_2_3();

    AdcTime = millis() + AdcDelay;
  }
}

void updateLcd() {
  if (LcdTime < millis()) {
    /*    lcd.setCursor(0, 0);
        lcd.print("       ");
        lcd.setCursor(0, 0);
        lcd.print(adc0 * multiplier, 3);
    */
    lcd.setCursor(0, 0);
    lcd.print("       ");
    lcd.setCursor(0, 0);
    if ( done == true || BattIR == 0.0 || BRDisp++ > 6) {
      lcd.print(mAh, 0);
      lcd.print( "mAh");
    } else {
      lcd.print(BattIR, 0);
      lcd.print("mOhm");
    }
    if (BRDisp == 11) BRDisp = 0;

    lcd.setCursor(8, 0);
    lcd.print("       ");
    lcd.setCursor(10, 0);
    lcd.print(battery, 3);
    lcd.print( "V");
    lcd.setCursor(0, 1);
    lcd.print("        ");
    lcd.setCursor(0, 1);
    //lcd.print(adcdif23 * multiplier, 3);
    if ( done == true ) {
      lcd.print(BattIR, 0);
      lcd.print("mOhm");
    } else {
      lcd.print((adcdif23 * multiplier * 10 / 1000) * battery, 3);
      lcd.print("W");
    }
    lcd.setCursor(8, 1);
    lcd.print("        ");
    lcd.setCursor(8, 1);
    lcd.print(adcdif23 * multiplier * 10, 2);
    lcd.print("mA");
    LcdTime = millis() + LcdDelay;
    /*
        lcd.setCursor(8, 1);
        lcd.print("        ");
        lcd.setCursor(8, 1);
        lcd.print(adc3 * multiplier, 3);
        LcdTime = millis() + LcdDelay;
    */
  }
}

void updateSerial() {
  if (SerialTime < millis()) {
    //    Serial.print(" Adc0 = ");
    Serial.print("$");
    Serial.print(int(battery * 1000));
    Serial.print(" ");
    Serial.print(int(mAh));
    Serial.print(" ");
    Serial.print(int(adcdif23 * multiplier * 10));
    //Serial.print(" ");
    //Serial.print(adc2*multiplier);
    Serial.println("; ");
    //Serial.println(adc3 * multiplier);
    SerialTime = millis() + SerialDelay;
  }
}

void initLcd() {
  lcd.init();
  lcd.backlight();
  lcd.print("ZwyC");
  delay(500);

}

void initAdc() {

  // The ADC input range (or gain) can be changed via the following
  // functions, but be careful never to exceed VDD +0.3V max, or to
  // exceed the upper and lower limits if you adjust the input range!
  // Setting these values incorrectly may destroy your ADC!
  //                                                                ADS1015  ADS1115
  //                                                                -------  -------
  //adc.setGain(GAIN_TWOTHIRDS);  // 2/3x gain +/- 6.144V  1 bit = 3mV      0.1875mV (default)
  //adc.setGain(GAIN_ONE);        // 1x gain   +/- 4.096V  1 bit = 2mV      0.125mV
  // adc.setGain(GAIN_TWO);        // 2x gain   +/- 2.048V  1 bit = 1mV      0.0625mV
  // adc.setGain(GAIN_FOUR);       // 4x gain   +/- 1.024V  1 bit = 0.5mV    0.03125mV
  adc.setGain(GAIN_EIGHT);      // 8x gain   +/- 0.512V  1 bit = 0.25mV   0.015625mV
  // adc.setGain(GAIN_SIXTEEN);    // 16x gain  +/- 0.256V  1 bit = 0.125mV  0.0078125mV

  adc.begin();
}

