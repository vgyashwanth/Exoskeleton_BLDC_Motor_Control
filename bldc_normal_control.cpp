#include <ModbusMaster.h> 
#include<Wire.h>
/******GLOBAL VARIABLES BLOCK******************/
ModbusMaster slave1;
//MPU6050 SETUP
const int MPU_addr=0x68;
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
 
int minVal=265;
int maxVal=402;
 
double x;
double y;
double z;
// MPU SETUP END
int poles_to_cover = 0; // for storing the number of poles to cover for the corrosponding degree to rotate
int frequency_of_motor_inHz=7;
float degree_rotated=0.0;
 int clock_wise_poles_count = 0;       // used to store the poles covered in clockwise direction
 int anti_clock_wise_poles_count = 0; // used to store the poles covered in anti clockwise direction
bool __clockwise=1;
bool __anti_clockwise=0;
bool driver_status = 0; // used for monitoring the drive status on or off for calling the set_to_home_position() function
#define enable_pin_of_driver 4  // 5volt pin of driver connected to arudino pin 4 for monitoring the driver status
#define buffer_ 0
/******GLOBAL VARIABLES BLOCK END*************/

/******FUNCTIONS BLOCK*****************/
void clockwise(void);// used to set the motor to rotate in clockwise direction
void anti_clockwise(void); // used to set the motor to rotate in anti_clockwise direction
uint32_t time_required_for_rotating_given_degree(float, int); // used to calculate the time requried
void preTransmission(void); // used for setting the modbus communication           
void postTransmission(void);// used for setting the modbus communication
void monitor_angle(unsigned int,bool);
void set_to_home_position( int,  int);
void reset_driver(void);
float mpu_angle(void);
/********FUNCTIONS BLOCK END***********/

///////////SETUP BLOCK /////////////
void setup() {
  
    Serial.begin(9600);
    //MPU6050 SETUP
    Wire.begin();
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x6B);
    Wire.write(0);
    Wire.endTransmission(true);
    //MPU6050 END
    pinMode(enable_pin_of_driver,INPUT); // used for monitoring the drive status on or off for calling the set_to_home_position() function
    pinMode(3, OUTPUT); // for controlling the communication
    pinMode(2, OUTPUT); // for controlling the communication
    digitalWrite(3,0);
    digitalWrite(2,0);
    Serial.begin(9600);            
    slave1.begin(1,Serial);           
    slave1.preTransmission(preTransmission);        
    slave1.postTransmission(postTransmission);
  



}
//////////SET UP BLOCK END///////////

/////////MAIN BLOCK/////////////////
void loop() {
           float mpu_deg = mpu_angle();
        //   Serial.print("   MPU angle at home position: ");
        //   Serial.println(mpu_deg);

      driver_status = digitalRead(enable_pin_of_driver);
      if(driver_status){ // during the intial power up of the motor i.e driver is on for the first time
        
        set_to_home_position(clock_wise_poles_count, anti_clock_wise_poles_count);
        
        }
       
        // Run the main program until the driver is ON 
     while(driver_status){ // driver is ON

           driver_status = digitalRead(enable_pin_of_driver);
        if(!driver_status){
       //   Serial.println("Driver is off");
          break;
          }
          
      uint32_t time_required =  time_required_for_rotating_given_degree(90,frequency_of_motor_inHz); //milli seconds with sampling
          
      clockwise();
      for(int i=0;i<(poles_to_cover-buffer_);i++){ 
        driver_status = digitalRead(enable_pin_of_driver);
        if(!driver_status){
       //   Serial.println("Driver is off");
          break;
          }
        monitor_angle(time_required,__clockwise);
        
        }
        
         driver_status = digitalRead(enable_pin_of_driver);
        if(!driver_status){
       //   Serial.println("Driver is off");
          break;
          }
        
        anti_clockwise();
         for(int i=0;i<(poles_to_cover-buffer_);i++){

               driver_status = digitalRead(enable_pin_of_driver);
        if(!driver_status){
       //   Serial.println("Driver is off");
          break;
          }
        
         monitor_angle(time_required,__anti_clockwise);
        
        }

         driver_status = digitalRead(enable_pin_of_driver);
        if(!driver_status){
        //  Serial.println("Driver is off");
          break;
          }
         clock_wise_poles_count = 0;
         anti_clock_wise_poles_count = 0; 
    
       driver_status = digitalRead(enable_pin_of_driver);
     }
       
       Serial.print("poles covered in clock wise direction: ");
       Serial.println(clock_wise_poles_count);
       Serial.print("poles covered in anticlock wise direction: ");
       Serial.println(anti_clock_wise_poles_count);
       Serial.println("Driver is Off in void loop()");
      delay(1000);

}
////////MAIN BLOCK END/////////////

//////FUNCTIONS DEFINATIONS///////
/*used for setting the transmission*/
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
/*end of the transmission functions */
void clockwise(void){
          Serial.println("Motor Rotates in clockwise Direction");
          slave1.writeSingleRegister(6,frequency_of_motor_inHz);   // setting the speed of the motor
          slave1.writeSingleRegister(12,poles_to_cover);          // setting no of poles need to rotate before entering into break condition
          slave1.writeSingleRegister(2,0x0109);                    // enables motor to rotate anti_CW 
          slave1.writeSingleRegister(0,0x01FF);                    // for writing the parameter into EEPROM
          return 0;
          
  }
void anti_clockwise(void){
        Serial.println("Motor Rotates in anti clockwise Direction");
        slave1.writeSingleRegister(6,frequency_of_motor_inHz);     // setting the speed of the motor
        slave1.writeSingleRegister(12,poles_to_cover);            // setting no of poles need to rotate before entering into break condition
        slave1.writeSingleRegister(2,0x0101);                      // enables motor to rotate CW 
        slave1.writeSingleRegister(0,0x01FF);                      // for writing the parameter into EEPROM
        return 0;
    
    
    }
uint32_t time_required_for_rotating_given_degree(float degree_to_rotate, int frequency_of_motor_inHz){

          // converting degree_to_rotate to required_no_of_poles_to_cover using global variable to store poles i.e poles_to_cover
            poles_to_cover = (degree_to_rotate*190)/360;
            
          // required time for rotating 360 degree at the given frequency
          
            float time_required = 226600.0/(frequency_of_motor_inHz);       /*226600=(11.33*20*1000)*/
            unsigned int time_required_for_poles_to_cover = (poles_to_cover*time_required)/190;
            
            // dividing the time_required_for_poles_to_cover into small samples in order to monitor the rotation of motor
  
             time_required_for_poles_to_cover = time_required_for_poles_to_cover/poles_to_cover ; //time_required for one pole
  
  
          return time_required_for_poles_to_cover; // milli seconds
  
  
  
  }

void monitor_angle(unsigned int delay_time, bool __direction){

    unsigned char result=slave1.readHoldingRegisters(12,1);
              if(result==slave1.ku8MBSuccess){
              //  Serial.print("currrent degree: ")
              ;
              }
                else
                {
                  Serial.println("Monitoring Fail");
                  return ;
                  }        
               int current_pole_value=slave1.getResponseBuffer(0x00);
               if(!__direction){  //for anticlock wise direction
                    anti_clock_wise_poles_count  = poles_to_cover - current_pole_value; // used to store the anti clock wise pole count 
                    degree_rotated=current_pole_value*(1.89473);
                    Serial.print(degree_rotated);
                    float mpu_deg = mpu_angle();
                 //   Serial.print("   MPU angle: ");
                    Serial.println(mpu_deg);
                      delay(delay_time);
                      return ;
                }
               // for clockwise direction
               current_pole_value=poles_to_cover - current_pole_value;
               clock_wise_poles_count = current_pole_value; // used to store clock wise pole count
               degree_rotated = current_pole_value*(1.89473);
               
               Serial.print(degree_rotated);
               float mpu_deg = mpu_angle();
                  //  Serial.print("   MPU angle: ");
                    Serial.println(mpu_deg);
               delay(delay_time);
                return ;
  
  
   
}
void set_to_home_position( int poles_covered_in_clock_wise, int poles_covered_in_anti_clock_wise){
  
         //   reset_driver();  // for reseting the driver into normal mode             
  
            int net_poles_to_cover =(int)((poles_covered_in_clock_wise) - (poles_covered_in_anti_clock_wise)) ;
            
            if(net_poles_to_cover == 0) {
           //   Serial.print("Shaft is at home position");
              clock_wise_poles_count = 0;
              anti_clock_wise_poles_count = 0;
              return;
              
              }
                // for motor to rotate in anti clock wise direction
            if(net_poles_to_cover > 0){

                    anti_clockwise();
                  //  Serial.print("Motor need to cover this many poles to reach home position: ");
                  //  Serial.println(net_poles_to_cover);
                    // Motor shaft is ahead in clockwise direction so we need to rotate in anti clock wise direction
                    // converting poles to degree;
                    float degree_to_rotate = (net_poles_to_cover*360)/190;
              
                      uint32_t time_required = time_required_for_rotating_given_degree(degree_to_rotate,frequency_of_motor_inHz);
                      
                      for(int i = 0; i <(poles_to_cover); i++){
                          delay(time_required);
                          Serial.println("Reseting to home position");
                        }
                        clock_wise_poles_count = 0;
                        anti_clock_wise_poles_count = 0;
                        
                  //    Serial.println("Shaft reseted back to Home position Sucess fully after rotating anti clock wise direction");
                      return ; 
              
              
              }

                    // for motor to rotate in clock wise direction
               if(net_poles_to_cover < 0){


                        clockwise();
                    // Motor shaft is ahead in anti clockwise direction so we need to rotate in clock wise direction
                    // converting poles to degree;
                    net_poles_to_cover = -(net_poles_to_cover);
                 //   Serial.print("Motor need to cover this many poles to reach home position: ");
                 //   Serial.println(net_poles_to_cover);
                    float degree_to_rotate = (net_poles_to_cover*360)/190;
              
                      uint32_t time_required = time_required_for_rotating_given_degree(degree_to_rotate,frequency_of_motor_inHz);
                      
                      for(int i = 0; i <(poles_to_cover); i++){
                         delay(time_required);
                    //     Serial.println("Reseting to home position");
                        }
                        clock_wise_poles_count = 0;
                        anti_clock_wise_poles_count = 0;
                  //    Serial.println("Shaft reseted back to Home position Sucess fully after rotating clock wise direction");
                      return ; 
              
              
              }
   }

 void reset_driver(void){
  
         slave1.writeSingleRegister(0,0x0008); 
         slave1.writeSingleRegister(0,0x01FF);
         unsigned char result = slave1.readHoldingRegisters(0,1);
         if(result == slave1.ku8MBSuccess){
          result = slave1.getResponseBuffer(0x00);
          if(result==8)
          Serial.println("Driver resetted Sucessfully");
          else
          Serial.println("Driver reseting is fail");
          
          
          
          
          }
          Serial.println("RESET DRIVER IS FAIL");
         return ;
  
  
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
  return (z-270); // based on our requirment we can change
  
  }
  




    

              
    
