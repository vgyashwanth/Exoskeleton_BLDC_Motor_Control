#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
uint16_t BNO055_SAMPLERATE_DELAY_MS = 100;
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29, &Wire);
sensors_event_t orientationData , angVelocityData , linearAccelData, magnetometerData, accelerometerData, gravityData; // events
#define speed_out A0
#define enable_ 26
#define start_ 24
#define enable_clock_wise_m1 30
#define enable_anti_clock_wise_m1 32
#define enable_clock_wise_m2 34
#define enable_anti_clock_wise_m2 36
void BNO_get_data(void);
void clock_wise(double);
void anti_clock_wise(double);
void enable_clockwise_m1(void);
void enable_anticlockwise_m1(void);
void enable_clockwise_m2(void);
void enable_anticlockwise_m2(void);
void stop_motors_m1_m2(void);

double Current_angle(sensors_event_t* event);
int start = 1; // for monitoring whether it is started first time or not;
//int index = 0; // used for indexing the trajectory
//double track[]={0.0,10.0,7.8,3.0,2.1,5.0,4.2,1.4,1.5,2.5,0.5,3.3,7.3,10.0,11.5,12.3,12.1,10.0,4.8,2.8,10.3,15.0,15.6,13.8,11.3,10.0,0.0}; // used for storing the trajectory
//int max_size = sizeof(track)/sizeof(double); // used for storing the size
void home_position(void);
void setup(void)
{
  Serial.begin(115200);
  pinMode(enable_,OUTPUT);
  pinMode(speed_out,OUTPUT);
  pinMode(start_,INPUT_PULLUP);
  pinMode(enable_clock_wise_m1,OUTPUT);
  pinMode(enable_anti_clock_wise_m1,OUTPUT);
    pinMode(enable_clock_wise_m2,OUTPUT);
  pinMode(enable_anti_clock_wise_m2,OUTPUT);
  while (!Serial) delay(10);  // wait for serial port to open!

  Serial.println("Orientation Sensor Test"); Serial.println("");

  /* Initialise the sensor */
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.println("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    Serial.println("Stucked in while(1) Loop");
    while (1);
  }

  delay(1000);
}
////////////////MAIN FUNCTION//////////////////////
void loop(void)
{    bool turn_off = digitalRead(start_);
    while(turn_off){ // the program will stay here until we turn_on the driver
      Serial.println("Device is OFF");
      turn_off = digitalRead(start_);
      BNO_get_data();
      Serial.print("Current degree: ");
     Serial.println((Current_angle(&orientationData)));
       stop_motors_m1_m2();
    //   index = 0;  // making the index to initial position
      delay(200);
    }
  // digitalWrite(enable_,HIGH);
    
    turn_off = digitalRead(start_);
    if((turn_off==0) && (start==1)){
      home_position();
      start++;
    }
    
anti_clock_wise(20.1); 
clock_wise(20.1);





    
#ifdef trajectroy
   for(;index<(max_size-1);index++){
       double degree_to_rotate = track[index+1] - track[index];
       turn_off = digitalRead(start_);
       if(turn_off) // in order to stop the motors when the device is off suddenly
          break;
       if(degree_to_rotate<0)
         clock_wise(track[index+1]);
         else
         anti_clock_wise(track[index+1]);    
    } 
 
    stop_motors_m1_m2(); // normal stop for one cycle just for checking purpose
#endif 
   /* BNO_get_data();
    double angle = Current_angle(&orientationData);  
    Serial.print("Knee angle(degree): ");
    Serial.println(angle);*/

}
//////////////MAIN FUNCTION END//////////////////

void clock_wise(double setpoint){
          digitalWrite(enable_,HIGH); // for starting the motor 
          Serial.println("clockwise Mode");
          BNO_get_data();
         double cur_angle = Current_angle(&orientationData);  
          setpoint = cur_angle+setpoint;
          Serial.print("Set point: ");
          Serial.println(setpoint);
          double error = setpoint - Current_angle(&orientationData); 
          //int ang = Current_angle(&orientationData); 
          while(error>(1.0) ){
            enable_clockwise_m1();
            BNO_get_data();
            error = setpoint - Current_angle(&orientationData); 
            Serial.print("Error(degree): ");
            Serial.println(error);
          }
           enable_clockwise_m2();
         // analogWrite(speed_out, 127); // for stopping the motor
         //  digitalWrite(enable_,LOW); // for stopping the motor
          Serial.println("MOTOR STOPPED IN CLOCK WISE DIRECTION"); 
       
         
}
void anti_clock_wise(double setpoint){
           digitalWrite(enable_,HIGH);// for starting the motor
          Serial.println("Anti clockwise Mode");
          BNO_get_data();
         setpoint = Current_angle(&orientationData) - setpoint;
         double error = Current_angle(&orientationData) - setpoint;
          while(error>1.0){
            enable_anticlockwise_m1();
            BNO_get_data();
            error = Current_angle(&orientationData) - setpoint;
            Serial.print("Error(degree): ");
            Serial.println(error);
          }
          enable_anticlockwise_m2();
          //   analogWrite(speed_out, 127); // for stopping the motor
          // digitalWrite(enable_,LOW); // for stopping the motor
          Serial.println("MOTOR STOPPED IN ANTI CLOCK WISE DIRECTION"); 
         
}
/********************Multiple Trajectory******************/
#ifdef trajectroy
void clock_wise(double setpoint){
         BNO_get_data();
         enable_clockwise_m1();
         while((360.0-Current_angle(&orientationData))>=(setpoint+0.5))
        { Serial.print("Current angle in clockwise: ");
          Serial.println(360.0-Current_angle(&orientationData));
          BNO_get_data(); }          
}
void anti_clock_wise(double setpoint){
      BNO_get_data();
       enable_anticlockwise_m1();
         while((360.0-Current_angle(&orientationData))<=(setpoint))
        { Serial.print("Current angle in anti clockwise: ");
          Serial.println(360.0-Current_angle(&orientationData));
          BNO_get_data(); } 
  
  }

#endif
/*******************MULTIPLE TRAJECTORY END******************/

double Current_angle(sensors_event_t* event) {
  double x = -1000000, y = -1000000 , z = -1000000; //dumb values, easy to spot problem
  if (event->type == SENSOR_TYPE_ACCELEROMETER) {
    Serial.print("Accl:");
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
  }
  else if (event->type == SENSOR_TYPE_ORIENTATION) {
    //Serial.print("Orient:");
    x = event->orientation.x;
    y = event->orientation.y;
    z = event->orientation.z;
  }
  else if (event->type == SENSOR_TYPE_MAGNETIC_FIELD) {
    Serial.print("Mag:");
    x = event->magnetic.x;
    y = event->magnetic.y;
    z = event->magnetic.z;
  }
  else if (event->type == SENSOR_TYPE_GYROSCOPE) {
    Serial.print("Gyro:");
    x = event->gyro.x;
    y = event->gyro.y;
    z = event->gyro.z;
  }
  else if (event->type == SENSOR_TYPE_ROTATION_VECTOR) {
    Serial.print("Rot:");
    x = event->gyro.x;
    y = event->gyro.y;
    z = event->gyro.z;
  }
  else if (event->type == SENSOR_TYPE_LINEAR_ACCELERATION) {
    Serial.print("Linear:");
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
  }
  else if (event->type == SENSOR_TYPE_GRAVITY) {
    Serial.print("Gravity:");
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
  }
  else {
    Serial.print("Unk:");
  }
      if((x==0.00) && (y==0.00) && (z==0.00)){
           Serial.println("Error in Sensor.........!!!!!!");
           Serial.println("Motor is going to disable");
           Serial.println("Stucked in while(1) Loop");
           digitalWrite(enable_,LOW); // diasble the motor
           while(1); // infinity loop
        
      }
      else
      if((x>=0.00) && (x<=5.00)) // our design constraints when we are at home position
      return (360.0); // we have to change to zero when we are going with multiple trajectory
      else
      return x;
 
}
void BNO_get_data(void){
  
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
  bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
  bno.getEvent(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER);
  bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
  bno.getEvent(&gravityData, Adafruit_BNO055::VECTOR_GRAVITY);
  
  }
  void home_position(void){
      BNO_get_data();
      double error = 360 -  Current_angle(&orientationData);
      while(error>1){
          analogWrite(speed_out,255);
          BNO_get_data();
          error = 360 -  Current_angle(&orientationData);
      }
      Serial.println("Motor reached home position Sucessfully");
   
   //   digitalWrite(enable_,LOW);
   
  }
void enable_clockwise_m1(void){
     digitalWrite(enable_clock_wise_m1,HIGH);
     digitalWrite(enable_anti_clock_wise_m1,LOW);
     return ;
  } 
void enable_anticlockwise_m1(void){
   digitalWrite(enable_clock_wise_m1,LOW);
   digitalWrite(enable_anti_clock_wise_m1,HIGH);
   return ; 
  }
void enable_clockwise_m2(void){
     digitalWrite(enable_clock_wise_m2,HIGH);
     digitalWrite(enable_anti_clock_wise_m2,LOW);
     return ;
  } 
void enable_anticlockwise_m2(void){
   digitalWrite(enable_clock_wise_m2,LOW);
   digitalWrite(enable_anti_clock_wise_m2,HIGH);
   return ; 
  }  
void stop_motors_m1_m2(void){

       digitalWrite(enable_clock_wise_m1,LOW);
       digitalWrite(enable_anti_clock_wise_m1,LOW);
       digitalWrite(enable_clock_wise_m2,LOW);
       digitalWrite(enable_anti_clock_wise_m2,LOW);
  
  }
