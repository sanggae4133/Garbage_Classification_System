#include <Servo.h>

#define MOTOR_SLEEP 10

int angle = 90; 

Servo servo_arr[3];
#define SERVO_PLA 6
#define SERVO_WASTE 2
#define SERVO_PAPER 10

void setup() {
    servo_arr[0].attach(SERVO_PLA);
    servo_arr[1].attach(SERVO_WASTE);
    servo_arr[2].attach(SERVO_PAPER);
    
    Serial.begin(9600);  
    Serial.println("Ready to receive commands (u0, d1, etc.)");
}

void servo_mv(Servo servo, int move) {
    if (move == 0) {    // up
        for (int i = 0; i < 90; i++) {  
            angle = angle + 1;
            if (angle > 180) 
                angle = 180;
            servo.write(angle);
            delay(MOTOR_SLEEP);
        }
        Serial.write('u');
    } 
    else if (move == 1) {   // down
        for (int i = 0; i < 90; i++) {  
            angle = angle - 1;
            if (angle <= 0) 
                angle = 0;
            servo.write(angle);
            delay(MOTOR_SLEEP);
        }
        Serial.write('d');
    }
}

void loop() {
    if (Serial.available()) {
        String input = Serial.readStringUntil('\n'); // 개행 문자('\n')까지 읽기
        
        // **입력된 데이터 확인 (디버깅용)**
        Serial.print("Received: ");
        Serial.println(input);

        if (input.length() < 2) {  // 최소 두 글자 필요
            Serial.println("INVALID");
            return;
        }

        char cmd = input[0];  // 첫 번째 문자
        char num = input[1];  // 두 번째 문자
        int mv_type = -1, mt_num = -1;

        if (cmd == 'u') mv_type = 0;
        else if (cmd == 'd') mv_type = 1;

        if (num >= '0' && num <= '2') mt_num = num - '0'; // '0' -> 0, '1' -> 1, '2' -> 2

        if (mv_type != -1 && mt_num != -1) {
            Serial.print("Executing command: ");
            Serial.print(cmd);
            Serial.println(num);
            servo_mv(servo_arr[mt_num], mv_type);
        } else {
            Serial.println("INVALID");
        }
    }
}
