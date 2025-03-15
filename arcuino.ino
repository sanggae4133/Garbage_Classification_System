#include <Servo.h>

#define MOTOR_SLEEP 10

//Servo servo;  
//int motor = 2;  
int angle = 90; 

Servo servo_arr[3];
#define SERVO_PLA 6
#define SERVO_WASTE 2
#define SERVO_PAPER 10

void setup() {
    servo_arr[0].attach(SERVO_PLA);
    servo_arr[1].attach(SERVO_WASTE);
    servo_arr[2].attach(SERVO_PAPER);
    // servo.attach(motor);
    Serial.begin(9600);  

    //Serial.println("Enter the u or d"); 
    //Serial.println("u = angle + 90"); 
    //Serial.println("d = angle - 90\n");
    // --
    //servo_arr[0].attach(
}

void servo_mv(Servo servo,int move) {
    if (move == 0) {    // up
        for (int i = 0; i < 90; i++) {  
            angle = angle + 1;
            if (angle > 180) 
                angle = 180;
            servo.write(angle);
            delay(MOTOR_SLEEP);
        }
        //Serial.print("\t");
        //Serial.println(angle);
        Serial.write('u');
    } 
    else if (move == 1) {   // down
        //Serial.print("\t-90\t"); 
        for (int i = 0; i < 90; i++) {  
            angle = angle - 1;
            if (angle <= 0) 
                angle = 0;
            servo.write(angle);
            delay(MOTOR_SLEEP);
        }
        //Serial.println(angle);  
        Serial.write('d');
    } 
    else {  
        //Serial.println("wrong character!!");
    }
}

void loop() {
    if (Serial.available()) {  
        // "u0"
        char *input = Serial.read(); 
        int mv_type, mt_num;

        if(intput[0] == "u") {
            mv_type = 0;
        } 
        else if(input[0] == 'd') {
            mv_type = 1;
        }
        else {
            mv_type = -1;
            // Serial.println("Wrrong CHarater!");
        }
        
        if (input[1] == '0') mt_num = 0;
        else if (input[1] == '1') mt_num = 1;
        else if (input[1] == '2') mt_num = 2;
        else mt_num = -1;

        if (mv_type != -1 && mt_num != -1) {
            servo_mv(servo_arr[mt_num], mv_type);
        }
        else {
            Serial.write("INVALID");
        }

        /*
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
        */
    }
}