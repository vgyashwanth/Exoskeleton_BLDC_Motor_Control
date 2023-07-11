#include <ModbusMaster.h>              
ModbusMaster slave1;
uint16_t a=0;
uint8_t result; 
uint8_t home_result;
static uint16_t b=0;  
static uint16_t c=0; 
static uint16_t d=0;
float f=0;
int e;
int i;
int sum=0;
//float degree[]={10.0,7.8,3.0,-2.1,-5.0,-4.2,-1.4,1.5,2.5,0.5,-3.3,-7.3,-10.0,-11.5,-12.3,-12.1,-10.0,-4.8,2.8,10.3,15.0,15.6,13.8,11.3,10.0};//path trajectory
float degree[]={-90,0};
int size_of_array=2;    //array size
float degree_to_rotate=0;
int speed_of_frequency_inHz=10;
float t=0;
int p=0;
static int j=0;
float delay_generator(int,int);
int clockwise(void);
int rotate_clockwise(void);
int home_position(void);
int anti_clockwise(void);
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
void setup()

{   pinMode(8,INPUT);
    pinMode(4,INPUT);
    pinMode(10,INPUT);
    pinMode(3, OUTPUT);
    pinMode(2, OUTPUT);
    digitalWrite(3,0);
    digitalWrite(2,0);
    Serial.begin(9600);            
    slave1.begin(1,Serial);           
    slave1.preTransmission(preTransmission);        
    slave1.postTransmission(postTransmission);
  }
void loop()
{ 
 
 //Serial.print("current state of the pole:");
// Serial.println(b);
    e=digitalRead(4);
    
   if(e==1) 
    home_position();
  while(e==1){
  //  Serial.println("Driver is in ON ");
  //  Serial.println("Starting from first");
     for(;j<size_of_array;){                                        //loop runner
 if(j==0){                                                          //first degree of rotation
      degree_to_rotate=degree[j];
    if(degree_to_rotate>0){
    e=digitalRead(4);
    if(e!=1)
      continue;
      t=delay_generator(degree_to_rotate,speed_of_frequency_inHz); //to generate delay
   //   Serial.print("delay requried:");
   //   Serial.println(t);
      clockwise();                                              //to rotate in clockwise direction
        e=digitalRead(4);
        if(e!=1)
    continue;
        for(i=0;i<11;i++){

              result=slave1.readHoldingRegisters(12,1);
              if(result==slave1.ku8MBSuccess&&e==1){
           //     Serial.print("status of reading1:");
            //    Serial.println(result);
               a=slave1.getResponseBuffer(0x00);
                  b=a;
           //    Serial.print("lenght1:");
           //  Serial.println(a) ;
               delay(t);
               
                }
                else{
               //   Serial.print("status of reading1:");
               //   Serial.println(result);
                  break;
                }
          
          }

            Serial.print("current status of the position:");  
            Serial.println(degree[j]);
            sum=sum+degree_to_rotate;
           }

else if(degree_to_rotate<0){


                degree_to_rotate=-(degree_to_rotate);
     e=digitalRead(4);
    if(e!=1)
      continue;
      t=delay_generator(degree_to_rotate,speed_of_frequency_inHz); //to generate delay
    //  Serial.print("delay requried:");
    //  Serial.println(t);
      anti_clockwise();                     //to rotate in anticlockwise direction
        e=digitalRead(4);
        if(e!=1)
    continue;
        for(i=0;i<11;i++){

              result=slave1.readHoldingRegisters(12,1);
              if(result==slave1.ku8MBSuccess&&e==1){
            //    Serial.print("status of reading1:");
            //    Serial.println(result);
               a=slave1.getResponseBuffer(0x00);
               b=a;
            //   Serial.print("lenght1:");
            // Serial.println(a) ;
               delay(t);
               
                }
                else{
              //    Serial.print("status of reading1:");
              //    Serial.println(result);
                  break;
                }
          
          }
          
             Serial.print("current status of the position:");
             Serial.println(degree[j]);
            sum=sum-degree_to_rotate;
            
            
            }
           
           }
          //   Serial.print("current position:");
         //   Serial.println(degree[j]);
           
           degree_to_rotate=degree[j]-degree[++j];    //effective distance
     if(degree_to_rotate<0){                          //rotate in clockwise direction
            
                  degree_to_rotate=-(degree_to_rotate);
               
                     
     
    e=digitalRead(4);
    if(e!=1)
      continue;
      t=delay_generator(degree_to_rotate,speed_of_frequency_inHz); //to generate delay
  //   Serial.print("delay requried:");
   // Serial.println(t);
     clockwise();
        e=digitalRead(4);
        if(e!=1)
    continue;
        for(i=0;i<11;i++){

              result=slave1.readHoldingRegisters(12,1);
              if(result==slave1.ku8MBSuccess&&e==1){
            //    Serial.print("status of reading1:");
            //   Serial.println(result);
               a=slave1.getResponseBuffer(0x00);
               b=a;
            //  Serial.print("lenght1:");
            //   Serial.println(a) ;
               delay(t);
               
                }
                else{
               //   Serial.print("status of reading1:");
              //    Serial.println(result);
                  break;
                }
                 
          }
                  Serial.print("current status of the position:");
                   Serial.println(degree[j]);
            
              sum=sum+degree_to_rotate;
          // Serial.print("current position:");
         //  Serial.println(degree[j]);
            }
         
 else if(degree_to_rotate>0){//rotate in anticlockwise direction
             
                     e=digitalRead(4);
                   if(e!=1)
                   continue;
             t=delay_generator(degree_to_rotate,speed_of_frequency_inHz); //to generate delay
     // Serial.print("delay requried:");
     // Serial.println(t);
     anti_clockwise();
        e=digitalRead(4);
        if(e!=1)
    continue;
        for(i=0;i<11;i++){

              result=slave1.readHoldingRegisters(12,1);
              if(result==slave1.ku8MBSuccess&&e==1){
           //     Serial.print("status of reading1:");
            //    Serial.println(result);
               a=slave1.getResponseBuffer(0x00);
               b=a;
            //   Serial.print("lenght1:");
            //   Serial.println(a) ;
               delay(t);
               
                }
                else{
            //    Serial.print("status of reading1:");
            //    Serial.println(result);
                  break;
                }
         
          }  
          
              Serial.print("current status of the position:");
               Serial.println(degree[j]);
              
               sum=sum-degree_to_rotate;
              
              
              }
           //   Serial.print("current position:");
           //   Serial.println(degree[j]);
              
 }

  if(sum>0){    //after completing array comeing back to reset position

      degree_to_rotate=sum;       //reset position
                          
                         e=digitalRead(4);
                   if(e!=1)
                   continue;
             t=delay_generator(degree_to_rotate,speed_of_frequency_inHz); //to generate delay
     // Serial.print("delay requried:");
    //  Serial.println(t);
     anti_clockwise();
        e=digitalRead(4);
        if(e!=1)
    continue;
        for(i=0;i<11;i++){

              result=slave1.readHoldingRegisters(12,1);
              if(result==slave1.ku8MBSuccess&&e==1){
             //   Serial.print("status of reading1:");
             //   Serial.println(result);
               a=slave1.getResponseBuffer(0x00);
               b=a;
             //  Serial.print("lenght1:");
             //  Serial.println(a) ;
               delay(t);
               
                }
                else{
              //   Serial.print("status of reading1:");
             //    Serial.println(result);
                  break;
                }
         
          }

      
      Serial.print("reset to home position with degree(anti_clock_wise direction):");
      Serial.println(sum);
      
      }
      else if(sum<0){
        sum=-(sum);
        degree_to_rotate=sum;
                          
     
    e=digitalRead(4);
    if(e!=1)
      continue;
      t=delay_generator(degree_to_rotate,speed_of_frequency_inHz); //to generate delay
    // Serial.print("delay requried:");
   //  Serial.println(t);
     clockwise();
        e=digitalRead(4);
        if(e!=1)
    continue;
        for(i=0;i<11;i++){

              result=slave1.readHoldingRegisters(12,1);
              if(result==slave1.ku8MBSuccess&&e==1){
            //    Serial.print("status of reading1:");
            //   Serial.println(result);
               a=slave1.getResponseBuffer(0x00);
               b=a;
            //   Serial.print("lenght1:");
           //    Serial.println(a) ;
               delay(t);
               
                }
                else{
              //    Serial.print("status of reading1:");
              //    Serial.println(result);
                  break;
                }
                 
          }

          
      Serial.print("reset to home position with degree(clock_wise direction):");
      Serial.println(sum);
         }
       //  delay(5000);
       //  while(1);                                                                        //LOOP STOPPER
              j=0;
               sum=0;                                       
        }
        
        }
      
  int clockwise(void){

          slave1.writeSingleRegister(6,speed_of_frequency_inHz);
          slave1.writeSingleRegister(12,p);
          slave1.writeSingleRegister(2,0x0109);
          slave1.writeSingleRegister(0,0x01FF);
          return 0;
  }
  int anti_clockwise(void){
        slave1.writeSingleRegister(6,speed_of_frequency_inHz);
        slave1.writeSingleRegister(12,p);
        slave1.writeSingleRegister(2,0x0101);
        slave1.writeSingleRegister(0,0x01FF);
        return 0;
    
    
    }
  int home_position(void){

           home_result=slave1.readHoldingRegisters(2,1);
           if(home_result==slave1.ku8MBSuccess){
            Serial.print("home_result:");
            Serial.println(home_result);
            c=slave1.getResponseBuffer(0x00);
            Serial.print("current state of the pole:");
            Serial.println(c);
                if(c==265){
                            d=p-b+((3*speed_of_frequency_inHz)/60);
                           
                        slave1.writeSingleRegister(6,speed_of_frequency_inHz);
                        slave1.writeSingleRegister(12,d);
                        slave1.writeSingleRegister(2,0x0101);
                        slave1.writeSingleRegister(0,0x01FF);
                        f=(d/190.0)*f+1100;
                        Serial.print("delay_want:");
                        Serial.println(f);
                        delay(f);
                        
                        Serial.println("reached home sucessfully(clock wise)");
                        }
                if(c==257){
                          b=b-((3*speed_of_frequency_inHz)/60);
                        slave1.writeSingleRegister(6,speed_of_frequency_inHz);
                        slave1.writeSingleRegister(12,b);
                        slave1.writeSingleRegister(2,0x0101);
                        slave1.writeSingleRegister(0,0x01FF);
                        f=(b/190.0)*f+300;
                        Serial.print("delay_want:");
                        Serial.println(f);
                        delay(f);
                        Serial.println("reached home sucessfully(anti clockwise)");
                }
              
              }
            
            
            
            }
            float delay_generator(int a,int b){
              
       
         p=(a*190.0)/360;
         t=226600.0/b;    //to calculate the delay
         f=t;
         t=t/11;
         t=(p/190.0)*t;
              return t;
              
              }

              
    
