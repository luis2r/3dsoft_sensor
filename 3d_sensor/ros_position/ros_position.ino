#include "Encoder.h"
#include <ros.h>
#include <std_msgs/Float32.h>




// --- Mapeamento de Hardware ---
#define    encoder_C1   2                     //Conexão C1 do encoder
#define    encoder_C2   4                     //Conexão C2 do encoder
#define    encoder_Z    3                     //Conexão Z do encoder   
#define    PWM1         5                     // PWM motor pin      //Saída pwm para controle de velocida do motor
#define    InA1         6                     // INA motor pin    //Controle In1 do driver L298N
#define    InB1         7                     // INB motor pin    



unsigned long lastMilli = 0;                    // loop timing 
unsigned long timeinterval = 0;
unsigned long encodertotal = 0;
unsigned long count = 0;
unsigned long  countAnt = 0;

int speed_req = 4;                            // speed (Set Point)

int speed_act = 0;                              // speed (actual value)
int PWM_val = 0;                                // (25% = 64; 50% = 127; 75% = 191; 100% = 255)
float Kp =   1;                                // PID proportional control Gain
float Kd =    4;                                // PID Derivitave control gain
              
                   

//Set up the ros node and publisher
std_msgs::Float32 angle_msg;
ros::Publisher pub_angle("angle", &angle_msg);
ros::NodeHandle nh;

Encoder myEncoder(encoder_C1, encoder_C2);
float degree;
float scale=0.087890625;
byte      Encoder_C1Last;
int       pulse_number;

          



void setup()
{
  nh.initNode();
  nh.advertise(pub_angle);
  pinMode(encoder_C1,  INPUT);                
  pinMode(encoder_C2,  INPUT);                
   
  //Configura saída para controle do motor (in1 do driver)
  pinMode(InA1, OUTPUT);
  pinMode(InB1, OUTPUT);
  pinMode(PWM1, OUTPUT);

  analogWrite(PWM1, PWM_val);
  digitalWrite(InB1, LOW);
  digitalWrite(InA1, HIGH);
  
  attachInterrupt(1, ai1, RISING);

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(12, OUTPUT);

}



void loop()
{

   angle_msg.data = myEncoder.read()*scale;
   pub_angle.publish(&angle_msg);


   timeinterval = millis()-lastMilli;
   lastMilli = millis();


   count = encodertotal+ myEncoder.read(); 
   Serial.print("SP:");             Serial.print(speed_req);     Serial.print("  RPM:");          Serial.print(speed_act);   Serial.print("  PWM:");    Serial.print(PWM_val);     Serial.print("count:");             Serial.print(count);   Serial.print("  ant:");          Serial.println(countAnt);  
   speed_act = (((count - countAnt)*60000)/(timeinterval*4096));          // 
   PWM_val= updatePid(PWM_val, speed_req, speed_act);                        // compute PWM value
   analogWrite(PWM1, PWM_val);
   countAnt = count;
   delay(10);
   
   nh.spinOnce();
} //end loop



void ai1() {
    encodertotal= encodertotal+ myEncoder.read();
    Serial.print("enc tot:");             Serial.print(encodertotal);   Serial.print("  myenco:");          Serial.println(myEncoder.read());
    myEncoder.write(0);

    digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
    digitalWrite(12, HIGH);   // turn the LED on (HIGH is the voltage level)

    delay(10);                       // wait for a second
    digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
    digitalWrite(12, LOW);    // turn the LED off by making the voltage LOW


  


}




int updatePid(int command, int targetValue, int currentValue)   {             // compute PWM value
float pidTerm = 0;                                                            // PID correction
int error=0;                                  
static int last_error=0;                             
 error = abs(targetValue) - abs(currentValue); 
 pidTerm = (Kp * error) + (Kd * (error - last_error));                            
 last_error = error;
 return constrain(command + int(pidTerm), 0, 255);
}
