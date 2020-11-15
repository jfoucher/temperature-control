

#define PIN_VAR 1
#define PIN_TEMP 2
#define PIN_OUT 3

#define __AVR_ATtiny85__
//
// Small Simple OLED library demo
//

#include <ss_oled.h>
bool fail = false;
bool err = false;
//PID constants
double kp = 80;
double ki = 0; // 2 * 0.5 / 120000;
double kd =  0; //2 * 3 * 120000 / 40;


unsigned long currentTime, previousTime = 0, prevTime = 0,  prevTime2 = 0, outPrevMicros = 0, outMicros;
double elapsedTime;
double error;
double lastError;
double input, output, setPoint;
double cumError, rateError;
double oldSetPoint = 0, oldInput = 0;


void setup() {
  //pinMode(PIN_TEMP, INPUT);
  pinMode(PIN_OUT, OUTPUT);
int rc;


  // put your setup code here, to run once:
//rc = oledInit(OLED_128x64, 0, 0, -1, -1,400000L); // use standard I2C bus at 400Khz
rc = oledInit(OLED_128x64, 0, 0, 0xb0, 0xb2, 0, 400000L); // for ATtiny85, use P0 as SDA and P2 as SCL
  if (rc != OLED_NOT_FOUND)
  {
    char *msgs[] = {"SSD1306 @ 0x3C", "SSD1306 @ 0x3D","SH1106 @ 0x3C","SH1106 @ 0x3D"};
    oledFill(0, 1);
    //oledWriteString(0,0,0,msgs[rc], FONT_NORMAL, 0, 1);
    //delay(200);
  } else {
    fail = true;
  }
}

void loop() {
  char buf[8];
  currentTime = millis();
  
  //writeOutput(output);
  if (currentTime - prevTime > 60000) {
    oledFill(0, 1);
    prevTime = currentTime;
  }
  
  if (currentTime - previousTime > 100) {
    double val = analogRead(PIN_TEMP);
    double set = analogRead(PIN_VAR);
    if (val < 1000 && val > 50) {
      double temp = 75 - 0.102 * val;
      //temp = 0.000087 * val * val - 0.0208 * val + 11.3;
    
      double setTemp = 20 + 0.0294 * set;
  
      double a = 0.01;
      double newSetPoint = 1023 / 30 * (setTemp - 20);
      double newInput = 1023 / 30 * (temp - 20);
      
      setPoint += a * (newSetPoint - oldSetPoint);
      input += a * (newInput - oldInput);
  
      oldInput = input;
      oldSetPoint = setPoint;
      
      output = computePID(input) / 4;
    
      if (temp > 50) {
        output = 0;
      }

      analogWrite(PIN_OUT, output);
      char buf[8];
     
    
      if(!fail && (currentTime - prevTime2) > 1000) {
        if (err == true) {
          err = false;
          error = 0;
          cumError = 0;
          oledFill(0, 1);
        }
        
        prevTime2 = currentTime;
        itoa((int)round(temp*10), buf, 10);
        oledWriteString(0, 06, 0,buf, FONT_STRETCHED, 0, 1);
        oledWriteString(0, 66, 0,(char *)"o", FONT_SMALL, 0, 1);
        oledWriteString(0, 76, 0,(char *)"C", FONT_STRETCHED, 0, 1);
        oledWriteString(0, 0, 3,(char *)"          ", FONT_SMALL, 0, 1);
        itoa((int)output, buf, 10);
        oledWriteString(0, 0, 3,buf, FONT_SMALL, 0, 1);
        for (int i = 0; i <= 21;i++) {
          if (i < round(output / 12)) {
            //oledWriteString(0, i*6, 3,(char *)" ", FONT_SMALL, 1, 1);
            oledWriteString(0, i*6, 4,(char *)" ", FONT_SMALL, 1, 1);
          } else {
            //oledWriteString(0, i*6, 3,(char *)" ", FONT_SMALL, 0, 1);
            oledWriteString(0, i*6, 4,(char *)" ", FONT_SMALL, 0, 1);
          }
        } 
        
        //itoa((int)output/128, buf, 10);
        //oledWriteString(0, 10, 3,(char *)buf, FONT_STRETCHED, 0, 1);
        itoa((int)round(setTemp * 10), buf, 10);
        oledWriteString(0, 06, 6,buf, FONT_STRETCHED, 0, 1);
        oledWriteString(0, 66, 6,(char *)"o", FONT_SMALL, 0, 1);
        oledWriteString(0, 76, 6,(char *)"C", FONT_STRETCHED, 0, 1);
      }
    } else {
      if (!err) {
         oledFill(0, 1);
      }
      err = true;
     
      oledWriteString(0, 0, 0,(char *)"Veuillez connecter", FONT_SMALL, 0, 1);
      oledWriteString(0, 0, 1,(char *)"la chanceliere", FONT_SMALL, 0, 1);
      analogWrite(PIN_OUT, 0);
      itoa((int)round(val), buf, 10);
      oledWriteString(0, 0, 2,buf, FONT_NORMAL, 0, 1);
    }
  }
} /* main() */

int writeOutput(int value) {
  outMicros = micros();
  double timer = (2550 - output * 10); //256 / (output + 0.001);
  if (outMicros - outPrevMicros > timer) {
    digitalWrite(PIN_OUT, HIGH);
    delayMicroseconds(10);
    digitalWrite(PIN_OUT, LOW);
    outPrevMicros = outMicros;
  }
}

double computePID(double inp){
  
        currentTime = millis();                //get current time
        elapsedTime = (double)(currentTime - previousTime);        //compute time elapsed from previous computation

        
        
        error = setPoint - inp;                                // determine error
//        if (error < 0) {
//          previousTime = currentTime;
//          cumError = 0;
//          return 0;
//        }
        cumError += error * elapsedTime;                // compute integral
        rateError = (error - lastError)/elapsedTime;   // compute derivative
        double tmpKi = ki;
        if (rateError < 0) {
          tmpKi = 1 * ki;
        }
 
        double out = kp*error + tmpKi*cumError + kd*rateError;                //PID output               
 
        lastError = error;                                //remember current error
        previousTime = currentTime;                        //remember current time

        if (out >= 1023) {
          return 1023;
        }
        if (out <= 0) {
          return 0;
        }

        return out;                                        //have function return the PID output
}
