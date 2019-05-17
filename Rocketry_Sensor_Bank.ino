#include <MacRocketry_SD_Logger.h>
#include <MPU9255.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

#define g 9.81
#define magnetometer_cal 0.06
#define SEALEVELPRESSURE_HPA (1013.25)


//MPU9255 Library from https://github.com/Bill2462/MPU9255-Arduino-Library
MPU9255 mpu;
MacRocketry_SD_Logger sd;
Adafruit_BME280 bme; //I2C BME

double c_accel_x;
double c_accel_y;
double c_accel_z;
double c_gyro_x;
double c_gyro_y;
double c_gyro_z;
double c_mag_x;
double c_mag_y;
double c_mag_z;

double temperature;
double pressure;
double altitude;
double humidity;
long timeSinceStart;

//Default process acceleration method.
double process_acceleration(int input, scales sensor_scale)
{
  /*
  To get acceleration in 'g', each reading has to be divided by :
   -> 16384 for +- 2g scale (default scale)
   -> 8192  for +- 4g scale
   -> 4096  for +- 8g scale
   -> 2048  for +- 16g scale
  */
  double output = 1;

  //for +- 2g

  if(sensor_scale == scale_2g)
  {
    output = input;
    output = output/16384;
    output = output*g;
  }

  //for +- 4g
  if(sensor_scale == scale_4g)
  {
    output = input;
    output = output/8192;
    output = output*g;
  }

  //for +- 8g
  if(sensor_scale == scale_8g)
  {
    output = input;
    output = output/4096;
    output = output*g;
  }

  //for +-16g
  if(sensor_scale == scale_16g)
  {
    output = input;
    output = output/2048;
    output = output*g;
  }

  return output;
}

//process raw gyroscope data
//input = raw reading from the sensor, sensor_scale = selected sensor scale
//returns : angular velocity in degrees per second
double process_angular_velocity(int16_t input, scales sensor_scale )
{
  /*
  To get rotation velocity in dps (degrees per second), each reading has to be divided by :
   -> 131   for +- 250  dps scale (default value)
   -> 65.5  for +- 500  dps scale
   -> 32.8  for +- 1000 dps scale
   -> 16.4  for +- 2000 dps scale
  */

  //for +- 250 dps
  if(sensor_scale == scale_250dps)
  {
    return input/131;
  }

  //for +- 500 dps
  if(sensor_scale == scale_500dps)
  {
    return input/65.5;
  }

  //for +- 1000 dps
  if(sensor_scale == scale_1000dps)
  {
    return input/32.8;
  }

  //for +- 2000 dps
  if(sensor_scale == scale_2000dps)
  {
    return input/16.4;
  }

  return 0;
}

//process raw magnetometer data
//input = raw reading from the sensor, sensitivity =
//returns : magnetic flux density in μT (in micro Teslas)
double process_magnetometer_flux(int16_t input, double sensitivity)
{
  /*
  To get magnetic flux density in μT, each reading has to be multiplied by sensitivity
  (Constant value different for each axis, stored in ROM), then multiplied by some number (calibration)
  and then divided by 0.6 .
  (Faced North each axis should output arround 31 µT without any metal / walls around
  Note : This manetometer has really low initial calibration tolerance : +- 500 LSB !!!
  Scale of the magnetometer is fixed -> +- 4800 μT.
  */
  return (input*magnetometer_cal*sensitivity)/0.6;
}

void setup() {
  Serial.begin(115200);

  //Try to initialize the IMU
   if(mpu.init())
  {
    Serial.println("initialization failed");
  }
  else
  {
    Serial.println("initialization succesful!");
  }

  //Automated Testing
  if(mpu.testIMU())
  {
    Serial.println("IMU testing failed!");
  }

  if(mpu.testMag())
  {
    Serial.println("Magnetometer testing failed!");
  }

  //Set IMU sensitivity range to 16g, 1000 deg/s
  mpu.set_gyro_scale(scale_1000dps);
  mpu.set_acc_scale(scale_16g); //TODO: If adaptive, see how long it takes for the accelerometer to register the sensitvity change
  
  //Try to connect to the SD card. Pin 10 default.

  if (sd.getConnectFile()){
    Serial.println("File IO");
  } else {
    Serial.println("cannot open file");
  }
  
  //Start the BME on the alternate address (0x76 by grounding SD0)
  if (!bme.begin(0x76) )
  {
        Serial.println("Could not find a valid BME280 sensor, check wiring!");
  }

  //Serial.println(sd.maxInt());
}


void loop() 
{
  //Take measurements and write to the SD card.
  mpu.read_acc();
  mpu.read_gyro();
  mpu.read_mag();
  //First convert binary data to values based on the current mode
  //These methods must be called after the read methods to get new values!
  c_accel_x = process_acceleration(mpu.ax, scale_16g);
  c_accel_y = process_acceleration(mpu.ay, scale_16g);
  c_accel_z = process_acceleration(mpu.az, scale_16g);
  c_gyro_x = process_angular_velocity(mpu.gx, scale_1000dps);
  c_gyro_y = process_angular_velocity(mpu.gy, scale_1000dps);
  c_gyro_z = process_angular_velocity(mpu.gz, scale_1000dps);
  c_mag_x = process_magnetometer_flux(mpu.mx, mpu.mx_sensitivity);
  c_mag_y = process_magnetometer_flux(mpu.my, mpu.my_sensitivity);
  c_mag_z = process_magnetometer_flux(mpu.mz, mpu.mz_sensitivity);

  //Time since the sensor logging started
  timeSinceStart = millis();

  //Stuff for BME here
  //BME takes about 26-28 ms/measurement 
  //IMU takes 3-4 ms/measurement

  //bme methods to find temperature, pressure, and humidity. Altitude is calculated from pressure, so we can do it in post.
  temperature = bme.readTemperature();
  pressure = bme.readPressure(); //Pressure in Pascals
  humidity = bme.readHumidity();

  char buff[12];
  Serial.print(timeSinceStart);
  Serial.print(", ");

  sprintf(buff, "%d", timeSinceStart);
  sd.writeBuffer(buff);
  sd.writeBuffer(", ");

  //Each dtostrf just writes the corresponding IMU parameter to the SD buffer
  //2nd argument = length of string
  //3rd argument = # of characters after decimal place
  //4th argument = array to store results (in this case SD buffer)
  dtostrf(c_accel_x, 5,3, buff);
  Serial.print(buff);
  Serial.print(", ");
  sd.writeBuffer(buff);
  sd.writeBuffer(", ");
  
  dtostrf(c_accel_y, 5,3, buff);
  Serial.print(buff);
  Serial.print(", ");
  sd.writeBuffer(buff);
  sd.writeBuffer(", ");
  
  dtostrf(c_accel_z, 5,3, buff);
  Serial.print(buff);
  Serial.print(", ");
  sd.writeBuffer(buff);
  sd.writeBuffer(", ");

  dtostrf(c_gyro_x, 5,3, buff);
  Serial.print(buff);
  Serial.print(", ");
  sd.writeBuffer(buff);
  sd.writeBuffer(", ");
  
  dtostrf(c_gyro_y, 5,3, buff);
  Serial.print(buff);
  Serial.print(", ");
  sd.writeBuffer(buff);
  sd.writeBuffer(", ");
  
  dtostrf(c_gyro_z, 5,3, buff);
  Serial.print(buff);
  Serial.print(", ");
  sd.writeBuffer(buff);
  sd.writeBuffer(", ");

  dtostrf(c_mag_x, 5,3, buff);
  Serial.print(buff);
  Serial.print(", ");
  sd.writeBuffer(buff);
  sd.writeBuffer(", ");
  
  dtostrf(c_mag_y, 5,3, buff);
  Serial.print(buff);
  Serial.print(", ");
  sd.writeBuffer(buff);
  sd.writeBuffer(", ");
  
  dtostrf(c_mag_z, 5,3, buff);
  Serial.print(buff);
  Serial.print(", ");
  sd.writeBuffer(buff);
  sd.writeBuffer(", ");
  
  dtostrf(temperature, 5,2, buff);
  Serial.print(buff);
  Serial.print(", ");
  sd.writeBuffer(buff);
  sd.writeBuffer(", ");
  
  dtostrf(pressure, 6,2, buff);
  Serial.print(buff);
  Serial.print(", ");
  sd.writeBuffer(buff);
  sd.writeBuffer(", ");
  
  dtostrf(humidity, 5,3, buff);
  Serial.println(buff);
  sd.writeBuffer(buff);
  sd.writeBuffer("\n");
  
  delay(500);
}
