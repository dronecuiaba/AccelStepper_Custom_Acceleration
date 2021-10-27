#include <AccelStepper.h>
 
#define buttonPinA 18
#define buttonPinB 19
int enPinMotor = 14; // Enable Pin
int encoder_pos = 0;
int motorRight = HIGH;
int motorLeft = HIGH;
 
#define stepper_pin_step 13
#define stepper_pin_dir  12
 
// Enc have 24 steps per revolution
// The motor have 800 steps per revolution
// Want: 1 encoder rev = 1 stepper rev
// 800 / 24 = 33.3333333333...
float steps_per_pulse = 33.3333333333;
float rampaAcceleration = 1.05; // diminua esse valor para deixar aceleração mais lenta (não menos que 1)
float rampaDesacceleration = 0.90; // diminua esse valor para deixar a desaceleração mais lenta (não maior que 1)
int initialMaxSpeedMotor = 100;
int initialAccelerationMotor = 100;
int lastMaxSpeedMotor = initialMaxSpeedMotor;
int lastAccelerationMotor = initialAccelerationMotor;
int maxSpeedMotor = 5000;
int accelerationMotor = 5000;

unsigned long keyPrevMillis = 0;
const unsigned long keyIntervalMs = 40; // intervalo em milessegundos para o incremento da velocidade e aceleração
 
AccelStepper stepper(1, stepper_pin_step, stepper_pin_dir);
 
void setup() {
  pinMode(enPinMotor, OUTPUT); //enable motor drive
  
  pinMode(buttonPinA, INPUT_PULLUP);
  pinMode(buttonPinB, INPUT_PULLUP);

  Serial.begin(9600);
}
 
void loop() {
  // read encoder
  motorRight = digitalRead(buttonPinA);
  motorLeft = digitalRead(buttonPinB);

  // botoes de direção pressionados
  if (motorRight == LOW || motorLeft == LOW) {
    // entra no if a cada "keyIntervalMs" milessegundos e incrementa o valor de velocidade e aceleração
    if (millis() - keyPrevMillis >= keyIntervalMs) {
        keyPrevMillis = millis();

        if(lastMaxSpeedMotor * rampaAcceleration <= maxSpeedMotor){
          lastMaxSpeedMotor = lastMaxSpeedMotor * rampaAcceleration;
          stepper.setMaxSpeed(lastMaxSpeedMotor);
        }

        if(lastAccelerationMotor * rampaAcceleration <= accelerationMotor){
          lastAccelerationMotor = lastAccelerationMotor * rampaAcceleration;
          stepper.setAcceleration(lastAccelerationMotor);
        }
    }

    // define a direção
    if (motorRight == LOW) {
      encoder_pos--;
    } else if (motorLeft == LOW) {
      encoder_pos++;
    }

    stepper.moveTo(round(encoder_pos*steps_per_pulse)); //o "encoder_pos" basicamente define a direção e multiplica o valor positivo ou negativo pela quantidade de passos
  } else {
    //Para desacelerando...
    if (millis() - keyPrevMillis >= keyIntervalMs) {
      keyPrevMillis = millis();

      if(lastMaxSpeedMotor * rampaDesacceleration >= initialMaxSpeedMotor){
        lastMaxSpeedMotor = lastMaxSpeedMotor * rampaDesacceleration;
        stepper.setMaxSpeed(lastMaxSpeedMotor);
      }
      else {
        stepper.stop();
      }

      if(lastAccelerationMotor * rampaDesacceleration >= initialAccelerationMotor){
        lastAccelerationMotor = lastAccelerationMotor * rampaDesacceleration;
        stepper.setAcceleration(lastAccelerationMotor);
      }
    }
    
    encoder_pos = 0;

    if(!stepper.isRunning()){
      Serial.println("MOTOR PAROU");
    }
  }

  stepper.run();
}
