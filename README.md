#include <Servo.h>

// Define sound sensor pins
#define SOUND_SENSOR_B 2     // Back
//#define SOUND_SENSOR_L A0    // Left (using Pin Change Interrupt)
#define SOUND_SENSOR_R 13    // Right (using Pin Change Interrupt)
#define SOUND_SENSOR_F 3     // Front

volatile int sensor1Count = 0;
volatile int sensor2Count = 0;

// Define ultrasonic sensor pins
#define TRIG_PIN 10          // Ultrasonic Trig pin
#define ECHO_PIN 12          // Ultrasonic Echo pin

// Define servo motor pin
#define SERVO_PIN 9          // Servo motor

// Define motor control pins
#define IN1 4                // Motor A forward
#define IN2 7                // Motor A backward
#define IN3 8                // Motor B forward
#define IN4 11               // Motor B backward
#define ENA 6
#define ENB 5

Servo servoMotor;
int distanceThreshold = 20;  // Distance threshold for obstacle avoidance (cm)

bool flag =0;

void setup() {
    Serial.begin(9600);

    // Initialize sound sensor pins
    pinMode(SOUND_SENSOR_B, INPUT_PULLUP);
    //pinMode(SOUND_SENSOR_L, INPUT_PULLUP); // Left sensor on pin A0 (PCINT)
    pinMode(SOUND_SENSOR_R, INPUT_PULLUP); // Right sensor on pin 13 (PCINT)
    pinMode(SOUND_SENSOR_F, INPUT_PULLUP);

    // Attach interrupts for front and back sensors
    attachInterrupt(digitalPinToInterrupt(SOUND_SENSOR_F), onSensorF, RISING);
    attachInterrupt(digitalPinToInterrupt(SOUND_SENSOR_B), onSensorB, RISING);

    // Enable Pin Change Interrupt for SOUND_SENSOR_R on pin 13 (PCINT5)
    PCICR |= (1 << PCIE0);     // Enable PCINT0 group (pins 8 to 13)
    PCMSK0 |= (1 << PCINT5);   // Enable PCINT for pin 13

    

    // Initialize ultrasonic sensor pins
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);

    // Initialize motor control pins
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);

    // Initialize the servo motor
    servoMotor.attach(SERVO_PIN);
}

void loop() {
  obstacleAvoidanceTaskFn();
}

// Sensor interrupt handlers
void onSensorF() { 
    
    moveForward();
    delay(500);
    stopMovement();
}

void onSensorB() {
    reverse();
    delay(500);
    stopMovement();
}

// Pin Change Interrupt Service Routine for PCINT0 group (pins 8-13)
ISR(PCINT0_vect) {
    if (digitalRead(SOUND_SENSOR_R) == HIGH) {
        
        turnLeft(2000);
        delay(500);
        stopMovement();
        
    }
}

// Task 1: Sound detection and movement control

// Task 2: Obstacle avoidance using ultrasonic sensor
int obstacleAvoidanceTaskFn() {
    moveForward();
    moveservo(90);     
    int distance = getDistance(); 
    delay(500); 
    if (distance < distanceThreshold) {  
        stopMovement();
        moveservo(0);                  
        distance = getDistance(); 
        delay(500);
        if (distance < distanceThreshold) {  
            moveservo(180);    
            distance = getDistance();  // Get distance from right
            delay(500);
            if (distance < distanceThreshold) {  // Obstacles on all sides
                stopMovement();  // No clear path, stop the robot
                return 0;        // Indicate that no path is available
            } 
            else {
                moveBackward(1000);
                turnRight(1000);     // Right side is clear, turn right
                return 1;        // Indicate a successful right turn
            }
        } 
        else {
            moveBackward(1000);
            turnLeft(1000);          // Left side is clear, turn left
            return 2;            // Indicate a successful left turn
        }
    } 
    else {
        moveForward();          // Front is clear, move forward
        return 3;               // Indicate forward movement
    }
}

// Task 3: Servo control for scanning the area

void moveservo(int x) {
    int current_angle = servoMotor.read();
    int increment = 1;
    if (x < current_angle) {
        increment = -1;
    }
    while (current_angle != x) {
        current_angle += increment;
        servoMotor.write(current_angle);
        delay(5);
    }
}

// Function to measure distance using ultrasonic sensor
int getDistance() {
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    long duration = pulseIn(ECHO_PIN, HIGH);
    int distance = duration * 0.034 / 2;

    return distance;  // Return distance in cm
}

// Movement functions
void moveForward() {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);

    analogWrite(ENA, 50);
    analogWrite(ENB, 50);
}

void reverse() {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);

    analogWrite(ENA, 50);
    analogWrite(ENB, 50);
}

void moveBackward(int duration) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENA, 50);
    analogWrite(ENB, 50);
  delay(duration);
    stopMovement();
}
void stopMovement() {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);

    analogWrite(ENA, 50);
    analogWrite(ENB, 50);
}
void turnLeft(int duration) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);

    analogWrite(ENA, 50);
    analogWrite(ENB, 150);
    delay(duration);  // Delay for turning
    stopMovement();
}
void turnRight(int duration) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);

    analogWrite(ENA, 150);
    analogWrite(ENB, 50);

    delay(duration);  // Delay for turning
    stopMovement();
}


## Future Improvements
- ROS integration
- Better path planning
