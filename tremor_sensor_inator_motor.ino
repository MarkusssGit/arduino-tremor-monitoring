/******************************************************************************
 * Project    : Tremor Sensor Inator
 * Author     : Jochem de Roos
 *
 * Description: Implementation file for main processing
******************************************************************************/

#include <arduinoFFT.h>
#include <Arduino_LSM6DS3.h>

#include "fast_fourier.h"
#include "ringbuffer.h"
#include "PE1MEW_Timer.h"
#include "butterworth.h"

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

#define MEASUREMENT_PERIOD 20   // measure at 50 Hz

float constants[28] = {1, 0, -1, 1, -0.553571802689660552587724851036909967661, 0.647175086870397153759881803125608712435, 0.374173203914198171382565760723082348704,
                       1, 0, -1, 1, -1.704384872779503456996508248266763985157, 0.841449820765228451158179723279317840934, 0.374173203914198171382565760723082348704,
                       1, 0, -1, 1, -1.330150365892840635950733485515229403973, 0.528282274914411265065439238242106512189, 0.325424490465780646974991441311431117356,
                       1, 0, -1, 1, -0.746363977154120283863392160128569230437, 0.318615410428532364051079639466479420662, 0.318615410428532364051079639466479420662
                      };

int counter = 0;
float value1 = 0;

butterworth butterworth_x(constants);    //identical, we want to check for all three directions of motion (watch could be rotated/backwards without affecting output)
butterworth butterworth_y(constants);
butterworth butterworth_z(constants);

PE1MEW_Timer t2;

float x, y, z = 0.0;
float temp1;

double frequency_tremor[3] = {0.0};
double amplitude_tremor[3] = {0.0};

fast_fourier fft_x(50,'x',&frequency_tremor[0],&amplitude_tremor[0]); //50Hz is frequency at which you're measuring
fast_fourier fft_y(50,'y',&frequency_tremor[1],&amplitude_tremor[1]);
fast_fourier fft_z(50,'z',&frequency_tremor[2],&amplitude_tremor[2]);

void setup() {
  //pinMode(12, OUTPUT);
  Serial.begin(9600);
  delay(8000);
  t2.setExpiry(MEASUREMENT_PERIOD);
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");
  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

}

void loop() {

  if (t2.getExpired()) {        
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);   

    if (IMU.accelerationAvailable())
    {
      IMU.readAcceleration(x, y, z);
      z *= -1;
      //y *= -1;
      temp1 = x;
      x = y;
      y = temp1;

    }
    float ac_1[3] = {x, y, z};
    float ac_2[3] = {a.acceleration.x, a.acceleration.y, a.acceleration.z};
    
    for(int i=0; i<3; i++){
     ac_1[i] += ac_2[i]/9.81;
      ac_1[i] /= 2;
    }
    x = ac_1[0];
    y = ac_1[1];
    z = ac_1[2];

    
    //fft_x.Fft_Input(butterworth_x.filter((value1*0.01)+x)); // for testing with t3 sine
    long int tx_one = millis();
    fft_x.input (butterworth_x.filter(x));
    long int tx_two = millis();
    long int ty_one = millis();
    fft_y.input (butterworth_y.filter(y));
    long int ty_two = millis();
    long int tz_one = millis();
    fft_z.input (butterworth_z.filter(z));
    long int tz_two = millis();

    // output for user, only the axis with the highest amplitude will be displayed

    //int D4 = 12;

    if (fft_x.get_amplitude() > fft_y.get_amplitude())
    {
      if (fft_x.get_amplitude() > fft_z.get_amplitude())
      {
        
        if (fft_x.get_frequency() > 3)
        {
          Serial.print("Time taken by the task: "); Serial.print(tx_two-tx_one); Serial.println(" milliseconds");
          //fft_x.print_tremor();
          //Serial.print("TREMOR DETECTED");
          //Serial.print(" ");
          /*digitalWrite(D4, HIGH);
          delay(10);
          digitalWrite(D4, LOW);
          delay(10);
          */
        }  
        else {
          //Serial.println("--");
        }      
      }
      else 
      {
        
        if (fft_z.get_frequency() > 3)
        {
          Serial.print("Time taken by the task: "); Serial.print(tz_two-tz_one); Serial.println(" milliseconds");
          //fft_z.print_tremor();
          //Serial.print("TREMOR DETECTED");
          //Serial.print(" ");
          /*
          digitalWrite(D4, HIGH);
          delay(10);
          digitalWrite(D4, LOW);
          delay(10);
          */
        }
        else {
          //Serial.println("--");
        }  
      }
    }
    else{
      if (fft_y.get_amplitude() > fft_z.get_amplitude()) {
        
        if (fft_y.get_frequency() > 3)
        {
          Serial.print("Time taken by the task: "); Serial.print(ty_two-ty_one); Serial.println(" milliseconds");
          //fft_y.print_tremor();
          //Serial.print("TREMOR DETECTED");
          //Serial.print(" ");
          /*
          digitalWrite(D4, HIGH);
          delay(10);
          digitalWrite(D4, LOW);
          delay(10);
          */
        }
        else {
          //Serial.println("--");
        }  
      } 
      else
      {
        
        if (fft_z.get_frequency() > 3)
        {
          Serial.print("Time taken by the task: "); Serial.print(tz_two-tz_one); Serial.println(" milliseconds");
          //fft_z.print_tremor();
          //Serial.print("TREMOR DETECTED");
          //Serial.print(" ");
          /*
          digitalWrite(D4, HIGH);
          delay(10);
          digitalWrite(D4, LOW);
          delay(10);
          */
        }
        else {
          //Serial.println("--");
        }  
      } 
    }
    
    /*
    int D4 = 12;

    void setup() 
    { 
      pinMode(D4, OUTPUT);
    } 
    
    void loop() {
        digitalWrite(D4, HIGH);
        delay(10);
        digitalWrite(D4, LOW);
        delay(10);
    } 
    */



    /*
    fft_x.input (butterworth_x.filter(g.gyro.x));
    fft_y.input (butterworth_y.filter(g.gyro.y));
    fft_z.input (butterworth_z.filter(g.gyro.z));

    // output for user, only the axis with the highest amplitude will be displayed
    //if (fft.get_fre)
    if (fft_x.get_amplitude() > fft_y.get_amplitude())
    {
      if (fft_x.get_amplitude() > fft_z.get_amplitude()) fft_x.print_tremor();
      else fft_y.print_tremor();
    }
    else{
      if (fft_y.get_amplitude() > fft_z.get_amplitude()) fft_y.print_tremor();
      else fft_z.print_tremor();
    }
    */

    if(!t2.setExpiry(MEASUREMENT_PERIOD)){
      delay           (MEASUREMENT_PERIOD);
      t2.setExpiry    (MEASUREMENT_PERIOD);
    }
  }
}
