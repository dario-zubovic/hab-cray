/*
    High altitude balloon cosmic ray detector project
    Copyright (C) 2015  Dario Zubovic

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <SPI.h>
#include <SD.h>
#include <AltSoftSerial.h>
#include <PinChangeInt.h>
#include <TinyGPS.h>
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <Wire.h>
#include <avr/wdt.h>

/*
Libraries:
AltSoftSerial: https://github.com/PaulStoffregen/AltSoftSerial
PinChangeInt: https://github.com/GreyGnome/PinChangeInt
TinyGPS: http://arduiniana.org/libraries/tinygps/
I2Cdevlib: https://github.com/jrowberg/i2cdevlib
*/

/////////////////////////////////PINS
const byte LED = 5;
//const byte GM1 = 8;
//const byte GM2 = 7;
//const byte GM_AND = 6;
//const byte GPS_RX = 2;
//const byte GPS_TX = 3;
//const byte GYRO_SCL_ANALOG = 5;  //i2c
//const byte GYRO_SDA_ANALOG = 4;  //i2c
//const byte SD_CS = 10;
//const byte SD_SCK = 13;  //SPI
//const byte SD_MOSI = 11;  //SPI
//const byte SD_MISO = 12;  //SPI

/////////////////////////////////GPS
AltSoftSerial GPSserial(3, 2); // RX, TX
TinyGPS gps;
float gps_lat, gps_lon, gps_altitude;
unsigned long gps_position_age, gps_time_age;
int gps_year;
byte gps_month, gps_day, gps_hour, gps_minute, gps_second, gps_hundredths;

/////////////////////////////////GYRO
MPU6050 mpu(0x68);
boolean useMPU = false;

int16_t ax, ay, az; // accelerometer axes
int16_t gx, gy, gz; //gyroscope axes

/////////////////////////////////INTERRUPTS
volatile unsigned long GM1_count = 0L;
volatile unsigned long GM2_count = 0L;
volatile unsigned long GM_AND_count = 0L;

void GM1_interrupt() {
  GM1_count++;
}
void GM2_interrupt() {
  GM2_count++;
}
void GM_AND_interrupt() {
  GM_AND_count++;
}

/////////////////////////////////variables
unsigned long last_datalog_write_millis = 0;
unsigned long last_gyro_millis = 0;
boolean first_write = true;

/////////////////////////////////SETUP
void setup() {
  //clear timer right away in order to not get stuck in infinite reset loop
  wdt_reset();
  wdt_disable();

  //LED indicator
  pinMode(LED, OUTPUT);
  digitalWrite(LED, HIGH);

  //init serial
  Serial.begin(9600);

  Serial.println(F("I0")); // I0 = Starting setup...
  
  //SD setup
  if(!SD.begin(10)) {
    Serial.println(F("E0")); //E0 = Card failed or not present!
    digitalWrite(LED, LOW);
    delay(2000);
    digitalWrite(LED, HIGH);
    return;
  }
  
  //gps setup
  GPSserial.begin(9600);
  byte mode[] = {0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x08, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  for(int i = 0; i < sizeof(mode); i++) {
    GPSserial.write(mode[i]);
    GPSserial.flush();
  }
  GPSserial.println();
  GPSserial.flush();
  
  //gyro setup
  //Serial.print(F("Initializing MPU6050... "));
  // join I2C bus (I2Cdev library doesn't do this automatically)
  Wire.begin();
  TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
  mpu.initialize();
  if(!mpu.testConnection()) {
    useMPU = false;
    digitalWrite(LED, LOW);
    delay(500);
    digitalWrite(LED, HIGH);

    Serial.println(F("E3")); //E3 = Not using MPU 6050 Gyro/acc
  } else {
    useMPU = true;
    
    mpu.setXAccelOffset(69.53372273); //TODO beter calibration
    mpu.setYAccelOffset(701.3089899);
    mpu.setZAccelOffset(7659.171257);
    mpu.setXGyroOffset(350.954957);
    mpu.setYGyroOffset(20.43722386);
    mpu.setZGyroOffset(367.8086022);
    
    Serial.println(F("I2")); //I2 = Using MPU 6050 Gyro/acc
  }
  
  //interrupt setup
  pinMode(8, INPUT);
  attachPinChangeInterrupt(8, GM1_interrupt, RISING);
  pinMode(7, INPUT);
  attachPinChangeInterrupt(7, GM2_interrupt, RISING);
  pinMode(6, INPUT);
  attachPinChangeInterrupt(6, GM_AND_interrupt, RISING);

  //start watchdog timer of 4 seconds
  wdt_enable(WDTO_4S);
  
  //finish
  Serial.println(F("I1")); // I1 = Init completed successfully!
  digitalWrite(LED, LOW);
}

/////////////////////////////////LOOP
void loop() {
    wdt_reset(); // Bee Gees - Stayin' Alive

    /////////////// GPS data is available, read it!
    while (GPSserial.available() > 0) {
      if(gps.encode(GPSserial.read())) {
        gps.f_get_position(&gps_lat, &gps_lon, &gps_position_age);
        gps_altitude = gps.f_altitude();
        gps_position_age = millis() - gps_position_age;
        
        gps.crack_datetime(&gps_year, &gps_month, &gps_day, &gps_hour, &gps_minute, &gps_second, &gps_hundredths, &gps_time_age);
        gps_time_age = millis() - gps_time_age;
      }
    }
    
    wdt_reset(); // reset watchdog timer

    //////////////// time to write to SD card
    if(millis() - last_datalog_write_millis > 1000) {
      mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz); // read MPU6050 raw data
      last_gyro_millis = millis();

      File dataFile = SD.open("l.txt", FILE_WRITE);
      
      if(dataFile) {
        if(first_write) {
          dataFile.println("#fresh start#");
          first_write = false;
        }

        //time
        dataFile.print(millis());
        dataFile.print(" ");
        
        //geiger-muller counter data
        dataFile.print(GM1_count);
        GM1_count = 0;
        dataFile.print(" ");
        dataFile.print(GM2_count);
        GM2_count = 0;
        dataFile.print(" ");
        dataFile.print(GM_AND_count);
        GM_AND_count = 0;
        dataFile.print(" ");
        
        //GPS data
        dataFile.print(gps_lat, 5); //lat
        dataFile.print(" ");
        dataFile.print(gps_lon, 5); //lon
        dataFile.print(" ");
        dataFile.print(gps_altitude, 2); //alt
        dataFile.print(" ");
        dataFile.print(gps_position_age); //position age
        dataFile.print(" ");
        dataFile.print(gps.satellites()); //number of satellites visible
        dataFile.print(" ");
        dataFile.print(gps.hdop()); //horizontal dilution of precision
        dataFile.print(" ");
        dataFile.print(gps_hour*10000 + gps_minute*100 + gps_second + gps_hundredths/1000, 4); //time
        dataFile.print(" ");
        dataFile.print(gps_time_age); //time age
        
        if(useMPU) {
          //gyroscope data
          dataFile.print(ax); //acc X
          dataFile.print(" ");
          dataFile.print(ay); //acc Y
          dataFile.print(" ");
          dataFile.print(az); //acc Z
          dataFile.print(" ");
          dataFile.print(gx); //gyro X
          dataFile.print(" ");
          dataFile.print(gy); //gyro Y
          dataFile.print(" ");
          dataFile.print(gz); //gyro Z
          dataFile.print(" ");
          dataFile.print(last_gyro_millis); //time
        }

        //done
        dataFile.println();
        dataFile.close();
      } else {
        Serial.println(F("E2")); // E2 = Can't open the file
        digitalWrite(LED, HIGH);
      }
      
      last_datalog_write_millis = millis();
    }
}