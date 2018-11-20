#include "Encoder.h"
#include <ros.h>
#include <std_msgs/Float32.h>
// --- Mapeamento de Hardware ---
#define    encoder_C1   2                     //Conexão C1 do encoder
#define    encoder_C2   4                     //Conexão C2 do encoder
#define    encoder_Z    3                     //Conexão Z do encoder
#define    pwm_out      5                     //Saída pwm para controle de velocida do motor
#define    motor1       6                     //Controle In1 do driver L298N

//Set up the ros node and publisher
std_msgs::Float32 angle_msg;
ros::Publisher pub_angle("angle", &angle_msg);
ros::NodeHandle nh;

Encoder myEncoder(encoder_C1, encoder_C2);
float degree;
float scale=0.087890625;
byte      Encoder_C1Last;
int       pulse_number,
          pwm_value = 128,
          vel = 280;    
          //vel = 50; 
          
void motor_control();   //Função para controle do motor


void setup()
{
  nh.initNode();
  nh.advertise(pub_angle);
  pinMode(encoder_C1,  INPUT);                //Configura entrada C1 para leitura do encoder
  pinMode(encoder_C2,  INPUT);                //Configura entrada C2 para leitura do encoder
  pinMode(motor1,     OUTPUT);                //Configura saída para controle do motor (in1 do driver)
   //Z rising pulse from encodenren activated ai1(). AttachInterrupt 1 is DigitalPin nr 3 on moust Arduino.
  attachInterrupt(1, ai1, RISING);
}



void loop()
{
   motor_control();    //chama função para controle do motor
   //delay(100);       //taxa de atualização
   angle_msg.data = myEncoder.read()*scale;
   pub_angle.publish(&angle_msg);
   //delay(1000);
   nh.spinOnce();
} //end loop





void motor_control()
{
  //Serial.println (float(myEncoder.read())*scale);
  
  if (vel >= 512)                             //vel maior ou igual a 512? (metade do valor de 10 bits 1024)
  {                                           //Sim...
    digitalWrite(motor1, LOW);                //Desliga bit de controle motor1
    pwm_value = map(vel, 512, 1023, 0, 255);  //normaliza valor do pwm de acordo com potenciômetro
    analogWrite(pwm_out, pwm_value);          //gera pwm proporcional
  } //end if vel
  else                                        //Senão...
  {                                           //vel menor que 512
    digitalWrite(motor1, HIGH);               //Liga bit de controle motor1
    pwm_value = map(vel, 511, 0, 0, 255);     //normaliza valor do pwm de acordo com potenciômetro
    analogWrite(pwm_out, pwm_value);          //gera pwm proporcional
  }
} //end motor_control

void ai1() {
    myEncoder.write(0);
}
