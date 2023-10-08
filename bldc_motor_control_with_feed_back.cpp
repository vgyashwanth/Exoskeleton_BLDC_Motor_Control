/****************************************************DIGITAL OPEN LOOP MODE CONTROL****************************************************/
#include <ModbusMaster.h>
#include<Wire.h> 
ModbusMaster slave1;
//MPU SETUP
const int MPU_addr=0x68;
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
 
int minVal=265;
int maxVal=402;
 
double x;
double y;
double z;
//MPU SETUP END
unsigned int pwm_input = 400; // for controlling the speed
bool device_status = 0;
void preTransmission(void); // used for setting the modbus communication           
void postTransmission(void);// used for setting the modbus communication
void clock_wise_with_pwm(unsigned int, float);
void anti_clock_wise_with_pwm(unsigned int, float);
void stop_motor(void);
float mpu_angle(void);
void home_position(bool);
////////////////SETUP FUNCTION//////////////
void setup() {
  //MPU SETUP
     Wire.begin();
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x6B);
    Wire.write(0);
    Wire.endTransmission(true);
  // MPU SETUP END
   pinMode(3, OUTPUT); // for controlling the communication
   pinMode(2, OUTPUT); // for controlling the communication
   pinMode(4, INPUT);// for checking the device status
    digitalWrite(3,0);
    digitalWrite(2,0);
    Serial.begin(9600);            
    slave1.begin(1,Serial);           //In this case our slave ID = 1
    slave1.preTransmission(preTransmission);        
    slave1.postTransmission(postTransmission);
  

}
///////////////SETUP FUNCTION END//////////
////////////////MAIN FUNCTION////////////////
void loop() {
        device_status = digitalRead(4);
         float mpu_position = mpu_angle();
         bool mpu_180_360 = ((mpu_position > 180.0f) && (mpu_position <360.0f) );
         bool mpu_0_180 = ((mpu_position > 0.0f) && (mpu_position <180.0f) );
        //Initial Startup
        if(device_status && (mpu_180_360 || mpu_0_180 )){
          Serial.println("Device entered into resetting state");
          home_position(mpu_180_360);
          }
           else{ // for indicating the angle when the device is off
         float cur_deg = mpu_angle();
        Serial.print("Current position of the shaft: ");
        Serial.println(cur_deg);
        delay(1000);
        }
        //After reaching home position
         mpu_position = mpu_angle();
         mpu_0_180 = ((mpu_position > 0) && (mpu_position <180) );
        //MAIN CONTROL ALGORITHM WILL START FROM HERE
        while(device_status && mpu_0_180){
          if(!device_status) // coming out from the loop when the device is suddenly off
                break;
        clock_wise_with_pwm(pwm_input,90);
              device_status = digitalRead(4);
               if(!device_status) // coming out from the loop when the device is suddenly off
                break;
           stop_motor();
        anti_clock_wise_with_pwm(pwm_input, 0);
              device_status = digitalRead(4);
               if(!device_status) // coming out from the loop when the device is suddenly off
                break;
           stop_motor();
           device_status = digitalRead(4);
            mpu_position = mpu_angle();
            mpu_0_180 = ((mpu_position > 0) && (mpu_position <180) );
                
        }
       
              stop_motor(); // when we come out from the loop
}
///////////////MAIN FUNCTION END////////////
void preTransmission(void)            
{
 digitalWrite(3, 1);             
 digitalWrite(2, 1);
}
void postTransmission(void)

{
digitalWrite(3, 0);
digitalWrite(2, 0);
} 
void clock_wise_with_pwm(unsigned int pwm, float degree_to_rotate){

        
          slave1.writeSingleRegister(4,pwm);    // used to set the pwm input 
          slave1.writeSingleRegister(2,0x0209);       // used for  selecting the direction                        
          slave1.writeSingleRegister(0,0x01FF);        // used to save parameter in eeporm 
         
          
            Serial.println("Monitoring angle in Clock wise");
            float degree_covered = mpu_angle(); //reading the current angle
         
            bool run_loop = 1;
            while(run_loop ){
                   device_status = digitalRead(4);
                     if(!device_status) // coming out from function
                     {
                      Serial.println("Device OFF in clock_wise direction");
                      return ;
                      }
                  
                   
                 Serial.print("Angle covered:  ");
                 Serial.println(degree_covered); 
                degree_covered = mpu_angle();
                float error = (degree_to_rotate - degree_covered); // for storing the error
             //   Serial.print("    Error: ");
             //   Serial.println(error);
               
                run_loop = (error >=3.0f); // if the error is less then 1 then jump out from the loop and enter into break mode
              }
            
    
            return ;           
  
  }
void anti_clock_wise_with_pwm(unsigned int pwm, float degree_to_rotate){
  
  
          slave1.writeSingleRegister(4,pwm); // used to set the pwm input   
          slave1.writeSingleRegister(2,0x0201);  // used for  selecting the direction                           
          slave1.writeSingleRegister(0,0x01FF);  // used to save parameter in eeporm

         
            Serial.println("Monitoring angle in  Anti Clock wise");
            float degree_covered = mpu_angle(); //reading the current angle
           
           
            bool run_loop = 1;
            while(run_loop ){
               device_status = digitalRead(4);
                     if(!device_status) // coming out from function
                     {
                      Serial.println("Device OFF in anti_clock_wise direction");
                      return ;
                      }
                float error = degree_to_rotate - degree_covered; // for storing the error
                 error = -(error); // for making the error positive in order for magnitude
             
                 Serial.print("Angle covered:  ");
                 Serial.println(error);
               //    Serial.print("   Error: ");
               // Serial.println(error); 
                degree_covered = mpu_angle();
               
                run_loop = (error >=3.0f);
              }     




  
  
  
  
  }  
void stop_motor(void){
             Serial.println("enter into stop_mode");
            slave1.writeSingleRegister(2,0x0203);  // used for  selecting the direction                           
            slave1.writeSingleRegister(0,0x01FF);  // used to save parameter in eeporm
  
  
  }
float mpu_angle(void){
  

            Wire.beginTransmission(MPU_addr);
Wire.write(0x3B);
Wire.endTransmission(false);
Wire.requestFrom(MPU_addr,14,true);
AcX=Wire.read()<<8|Wire.read();
AcY=Wire.read()<<8|Wire.read();
AcZ=Wire.read()<<8|Wire.read();
int xAng = map(AcX,minVal,maxVal,-90,90);
int yAng = map(AcY,minVal,maxVal,-90,90);
int zAng = map(AcZ,minVal,maxVal,-90,90);
 
x= RAD_TO_DEG * (atan2(-yAng, -zAng)+PI);
y= RAD_TO_DEG * (atan2(-xAng, -zAng)+PI);
z= RAD_TO_DEG * (atan2(-yAng, -xAng)+PI);
 

//Serial.print("AngleZ= ");
//Serial.println(z-360); // for our requirement
//Serial.println("-----------------------------------------");
//delay(400);
  return (z); // based on our requirment we can change
  
  }  
//Always in Anticlock wise direction  
void home_position(bool angle_180_360){
            if(angle_180_360){
              
              clock_wise_with_pwm(pwm_input, 362);
              stop_motor();
              Serial.println("Home position reached sucessfully in clockwise direction");
           return ;
              
              
              }
          else{
  
           anti_clock_wise_with_pwm(pwm_input, 0);
           stop_motor(); 
           Serial.println("Home position reached sucessfully in anti clockwise direction");
           return ;
  
          }
  }

  
  
