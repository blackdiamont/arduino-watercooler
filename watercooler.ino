#include <LiquidCrystal.h>

double Thermistor(int RawADC) {
 double Temp;
 Temp = log(1000.0*((1024.0/RawADC-1))); // 1000.0 <- change this according to ntc 
 Temp = 1 / (0.001129148 + (0.000234125 + (0.0000000876741 * Temp * Temp ))* Temp );
 Temp = Temp - 273.15;            // Convert Kelvin to Celcius
 //Temp = (Temp * 9.0)/ 5.0 + 32.0; // Convert Celcius to Fahrenheit
 return Temp;
}

int waterpwm(int temp) {
  int pwm;
  switch(temp) {
    case 20 : pwm = round(12.75);
    break;
    case 21 : pwm = round(12.75 * 2);
    break;
    case 22 : pwm = round(12.75 * 3);
    break;
    case 23 : pwm = round(12.75 * 4);
    break;
    case 24 : pwm = round(12.75 * 5);
    break;
    case 25 : pwm = round(12.75 * 6);
    break;
    case 26 : pwm = round(12.75 * 7);
    break;
    case 27 : pwm = round(12.75 * 8);
    break;
    case 28 : pwm = round(12.75 * 9);
    break;
    case 29 : pwm = round(12.75 * 10);
    break;
    case 30 : pwm = round(12.75 * 11);
    break;
    case 31 : pwm = round(12.75 * 12);
    break;
    case 32 : pwm = round(12.75 * 13);
    break;
    case 33 : pwm = round(12.75 * 14);
    break;
    case 34 : pwm = round(12.75 * 15);
    break;
    case 35 : pwm = round(12.75 * 16);
    break;
    case 36 : pwm = round(12.75 * 17);
    break;
    case 37 : pwm = round(12.75 * 18);
    break;
    case 38 : pwm = round(12.75 * 19);
    break;
    case 39 : pwm = round(12.75 * 20);
    break;
    default : pwm = 100;
  }
  return pwm;
}

// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(12, 11, 7, 6, 5, 4);

int ntc_th, ntc_th2, ntc_avg, but_pr = 0, pwm_led = 1, pwm_cur = 160, pwm_per, i = 0, tacho = 0;
float Temp, flank = 0, upm;
char test[20], hyst[10];
// read RPM and calculate
const int numreadings = 10;
int readings[numreadings];
unsigned long average = 0;
int index = 0;
unsigned long total; 

volatile int rpmcount = 0;//see http://arduino.cc/en/Reference/Volatile 
unsigned long rpm = 0;
unsigned long lastmillis = 0, lms = 0;

void rpm_fan(){ /* this code will be executed every time the interrupt 0 (pin2) gets low.*/
  rpmcount++;
}
 
void setup() {
  TCCR1B = TCCR1B & 0b11111000 | 0x01; // Setzt Timer1 (Pin 9 und 10) auf 31300Hz
  Serial.begin(9600);
  Serial.println("Watercooling System");
  lcd.begin(20, 4);
  pinMode(3, INPUT_PULLUP);
  pinMode(8, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(13, OUTPUT);
  digitalWrite(8, HIGH);
  digitalWrite(13, HIGH);
  pinMode(10, INPUT_PULLUP);
  pinMode(A4, INPUT_PULLUP);
  pinMode(A5, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(3), rpm_fan, FALLING);
   //lcd
  lcd.setCursor(0, 0);
  lcd.print("Watercooling System");
  lcd.setCursor(0, 1);
  lcd.print("Testing on Arduino");
  lcd.setCursor(0, 2);
  lcd.print("Nano Atmel 328");
  lcd.setCursor(0, 3);
  lcd.print("I Hope it Works!");
  delay(1000);
  lcd.clear();
}

void loop() {
  ntc_th = Thermistor(analogRead(0));
  ntc_th2 = Thermistor(analogRead(2));
  hyst[i] = ntc_th;
  i++;
  //fancontrol
  if (millis() - lastmillis >= 1000){  /*Uptade every one second, this will be equal to reading frecuency (Hz).*/
 
 //detachInterrupt(0);    //Disable interrupt when calculating
 total = 0;  
 readings[index] = rpmcount * 60;  /* Convert frecuency to RPM, note: this works for one interruption per full rotation. For two interrups per full rotation use rpmcount * 30.*/
 Serial.println(rpmcount);
 for (int x=0; x<=9; x++){
   total = total + readings[x];
 }
 
 average = total / numreadings;
 rpm = average;
 
 rpmcount = 0; // Restart the RPM counter
 index++;
 if(index >= numreadings){
  index=0; 
 } 
 
 
if (millis() > 5000){  // wait for RPMs average to get stable

 Serial.print(" RPM = ");
 Serial.println(rpm);
 lcd.setCursor(0 ,0);
 sprintf(test,"FAN RPM avg %d ",rpm);
 lcd.print(test);
}
 
 lastmillis = millis(); // Update lasmillis
  //attachInterrupt(0, rpm_fan, FALLING); //enable interrupt
  }
//rest
  if(i == 9) {
    i=0;
    ntc_avg = (hyst[0] + hyst[1] + hyst[2] + hyst[3] + hyst[4] + hyst[5] + hyst[6] + hyst[7] + hyst[8] + hyst[9]) / 10;
  }
    //pwm leds PWM MAX 1493-1506 RPM, PWM 125 1074 - 1081, PWM 65 784 - 795
  if(pwm_led == 1) {
    digitalWrite(8, LOW);
    digitalWrite(13, HIGH);
    //temp
    pwm_cur = waterpwm(ntc_avg + 2);
    
  } else {
    digitalWrite(8, HIGH);
    digitalWrite(13, LOW);
  }
  if (millis() - lms >= 100) {
    if(digitalRead(10) == 0) {
      if(but_pr == 0) {
        but_pr = 1;
        lcd.clear();
        if(pwm_led == 0) {
          pwm_led = 1;
        } else {
          pwm_led = 0;
          pwm_cur = 125;
        }
      }
    }
    if(digitalRead(A4) == 0) {
      if(but_pr == 0 && pwm_led == 0 && pwm_cur <= 250) {
        pwm_cur = pwm_cur + 5;
      }
    } 
    if(digitalRead(A5) == 0) {
      if(but_pr == 0 && pwm_led == 0 && pwm_cur >= 5) {
        pwm_cur = pwm_cur - 5;
      }
    }
    /*lcd.setCursor(0, 0);
    sprintf(test,"But %d  ",but_pr);
    lcd.print(test);*/
    lcd.setCursor(0, 1);
    sprintf(test,"PWM %d avgTmp %d ",pwm_cur, ntc_avg);
    lcd.print(test);
    lcd.setCursor(0, 2);
    sprintf(test,"Thermistor 1: %d C ",ntc_th); // Temperatur vom NTC Thermistor
    lcd.print(test);
    lcd.setCursor(0, 3);
    sprintf(test,"Thermistor 2: %d C ",ntc_th2); // Temperatur vom NTC Thermistor
    lcd.print(test);
    lms = millis();
  }
  if(digitalRead(10) == 1 && but_pr == 1) {
    but_pr = 0;
  }
  analogWrite(9, pwm_cur);
}
