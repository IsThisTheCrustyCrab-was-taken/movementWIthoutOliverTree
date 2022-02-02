/**
 * @file BID_motor.ino
 * @brief Skeleton code for robotic competition
 *
 * @author Agent P. (skeleton:Jotaro)
 *
 */
#include <Arduino.h>
#define TODO 0 //TODO: uncomment this if you want to work on your robot

// pin configuration
int pin_ENA = 5; //PWM output
int pin_ENB = 6; //PWM output
int pin_IN1 = 4;
int pin_IN2 = 3;
int pin_IN3 = 7;
int pin_IN4 = 8;

int pin_sensor = A0;
//int pin_sensor_2 = A1; ... additional sensor here

// PWM
int left_power = 0; //pwm power = [0, 255]
int right_power = 0; //pwm power = [0, 255]

void motorTest();

void setMotorDirection(bool l, bool r);
void driveForward(int pwr);
void rotateRobot(int pwr);
void steer(int powL, int powR);
void stopRobot();

int classify(int input);
void handleTransition(bool good);
void handleGoodTransition();
void handleBadTransition();
int color[2]; // 1=White, 2=Gray, 3=Black //color[1] = last seen Color
int transitions[3];


void setup(){
    // set motor control
    pinMode(pin_ENA, OUTPUT);//==leftPWM
    pinMode(pin_ENB, OUTPUT);//==rightPWM

    pinMode(pin_IN1, OUTPUT);//==left
    pinMode(pin_IN2, OUTPUT);//==left
    pinMode(pin_IN3, OUTPUT);//==right
    pinMode(pin_IN4, OUTPUT);//==right

    //your other initialization here.
    pinMode(pin_sensor, INPUT);
    Serial.begin(9600);
}

void loop(){

    /*analogWrite(pin_ENB, 170);
    analogWrite(pin_ENA, 170);
    setMotorDirection(1,1);*/

    motorTest();


    /*//Luis Try at doing stuff:
    // we need: color, lastcolor -> color[2];
    color[0] = classify(analogRead(pin_sensor));
    // 1=White, 2=Gray, 3=Blac

        switch (color[0]){
         case 1:
             switch (color[1]) {
                 case 1:break;
                 case 2:
                     handleTransition(0);
                     break;
                 case 3:
                     handleTransition(1);
                     break;
                 default:
                     Serial.print("PROBLEM AMK");
             }
             break;
         case 2:
             switch (color[1]) {
                 case 1:
                     handleTransition(1);
                     break;
                 case 2:break;
                 case 3:
                     handleTransition(0);
                     break;
                 default:
                     Serial.print("PROBLEM AMK");
             }
             break;
         case 3:
             switch (color[1]) {
                 case 1:
                     handleTransition(0);
                     break;
                 case 2:
                     handleTransition(1);
                     break;
                 case 3:break;
                 default:
                     Serial.print("PROBLEM AMK");
             }
             break;
         default:
             Serial.print("############CLASSIFY RETURNING (X<0 OR 3<X)");
        }
        //color[0] = color[1];*/
}

void handleTransition(bool good) {
    // we need: delta time 1 and 2 -> deltaT[2];
    transitions[2] = transitions[1];
    transitions[1] = transitions[0];
    transitions[0] = millis();

    switch (good){
        case true:
            switch (transitions[2]-transitions[1] > transitions[1]-transitions[0]) {
                case true:
                    //steer more to the left to make your time more better
                    steer(255,200);
                    delay(100);
                    break;
                case false:
                    //steer back to the Right again plus what u steered left from the last decision
                    steer(145,255);
                    delay(100);
                    return;
            }
            break;
        case false:
            //90 degree rotation
            steer(-255,255);
            delay(100);//TODO: wie viel delay brauchn wa ?

            return;
    }
}

void slightLeft(){

}


//TODO: Looking at this code, how should robot behave on a ground? Sketch robot's path on a paper.
void motorTest(){
    // Your robot will dance like below.
    driveForward(100);
    delay(1000);
    driveForward(255);
    delay(1000);
    driveForward(-100);
    delay(1000);
    driveForward(-255);
    delay(1000);
    rotateRobot(100);
    delay(1000);
    rotateRobot(-100);
    delay(1000);
    stopRobot();
    delay(1000);
}

//TODO: Complete motor controling logic here. See the kyub workshop page (https://kyub.com/model/5ff2a9d003b03900586faeb1) for more detail.
void setMotorDirection(bool l, bool r){ // if true then forward, if false then reverse
    if(l){
        //TODO: hint: MOTOR A CLOCKWISE
        digitalWrite(pin_IN1, HIGH);
        digitalWrite(pin_IN2, LOW);
    }
    else{
        //TODO: MOTOR A BLOCK -> myb low low
        digitalWrite(pin_IN1, HIGH);
        digitalWrite(pin_IN2, HIGH);
    }

    if(r){
        //TODO: MOTOR B COUNTER-CLOCKWISE
        digitalWrite(pin_IN3, LOW);
        digitalWrite(pin_IN4, HIGH);
    }else{
        //TODO: Motor B BLOCK -> myb low low
        digitalWrite(pin_IN3, HIGH);
        digitalWrite(pin_IN4, HIGH);
    }
}

void steer(int l_pow, int r_pow){  // l = [-255, 255], r = [-255, 255]. Negative->reverse, 0->stop, Positive->forward.
    bool l=true;
    bool r=true;
    if(l<0)l=false;
    if(r<0)r=false;
    setMotorDirection(l, r);
    analogWrite(pin_ENA, l_pow);
    analogWrite(pin_ENB, r_pow);
}

void driveForward(int pow){
    steer(pow, pow);
}

void rotateRobot(int pow){
    steer(pow, -pow);
}

void stopRobot(){
    steer(0, 0);
}

int classify(int a){
    return a%3;
}















/*//TODO: Test your motor code using this function. Remove this if your robot is behaving as motorTes
analogWrite(pin_ENA,127);
analogWrite(pin_ENB, 127);
setMotorDirection(1,0);
Serial.print("Linkes Rad sollte jetzt vorwäts drehen");
delay(3000);
setMotorDirection(0,1);
Serial.print("Rechtes Rad sollte jetzt vorwäts drehen");
delay(3000);
setMotorDirection(1,1);
Serial.print("Beide Räder sollten jetzt vorwäts drehen");
delay(3000);
setMotorDirection(0,0);
Serial.print("STOPPPEN!");
delay(5000)
Serial.print("running motorTest(); ....");
motorTest();


TODO: make your robot logic here.
int sensorVal = analogRead(pin_sensor);
int sensorVal2 = analogRead(pin_sensor_2);
int color_pred = predict(sensorVal, sensorVal2);

switch(color_pred){
    case 0:
    doSomething();
    break;

    case 1:
    doSomethingElse();
    break;

    default:
    break;
}
delay(10); //recommended to wait for 10ms to use analogRead
*/
