#include <Servo.h>

#define MOTOR_SLEEP 10

Servo servo;  
int motor = 2;  
int angle = 90;  
Servo servo_arr[3];

void setup() {
    servo.attach(motor);
    Serial.begin(9600);  

    Serial.println("Enter the u or d"); 
    Serial.println("u = angle + 90"); 
    Serial.println("d = angle - 90\n");  
    // --
    //servo_arr[0].attach(
}

void servo_mv(Servo servo) {
  
}

void loop() {
    if (Serial.available()) {  
        char input = Serial.read(); 

        if (input == 'u') {
            Serial.print("+90");  
            for (int i = 0; i < 90; i++) {  
                angle = angle + 1;
                if (angle > 180) 
                    angle = 180;
                servo.write(angle);
                delay(MOTOR_SLEEP);
            }
            Serial.print("\t");
            Serial.println(angle);
        } 
        else if (input == 'd') {
            Serial.print("\t-90\t"); 
            for (int i = 0; i < 90; i++) {  
                angle = angle - 1;
                if (angle <= 0) 
                    angle = 0;
                servo.write(angle);
                delay(MOTOR_SLEEP);
            }
            Serial.println(angle);  
        } 
        else {  
            Serial.println("wrong character!!");
        }
    }
}