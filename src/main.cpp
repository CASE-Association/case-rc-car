/**
 * @file main.cpp
 * @author Isak Åslund (aslundisak@gmail.com)
 * @brief Wirelessly controlled  RC car by tilting your hand. Uses an Arduino (MCU), MPU6050 (IMU) and NRF24 (wireless transceiver)
 * @comment This code is heavily inspired and dependant on the I2Cdev and MPU6050 library from jrowberg. Check it out on github.
 * @links https://github.com/jrowberg/i2cdevlib and https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050 
 * @version 0.1
 * @date 2019-08-28
 */

// ================================================================
// ===                     Includes                             ===
// ================================================================
#include <Arduino.h>
#include <SPI.h>
#include <Servo.h>
#include <math.h>
#include "RF24.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"  
#include "Wire.h"

// ================================================================
// ===                      Defines                             ===
// ================================================================
//#define TRANSMITTER     // Uncommet for trasmitter, comment for receiver.
#define DEBUG             // Deactivating serial print greatly increases the responsiveness of the system.

#define INTERRUPT_PIN   2 // Used for MPU6050 data ready interrupt
#define TURN_PIN        3 // PWM pin for servo controlling turning
#define THROTTLE_PIN    5 // PWM pin for ESC controlling speed

// ================================================================
// ===                     Global variables                     ===
// ================================================================

// Since the ESC and servo expects servo PWM signals which is 20ms apart and between 1-2ms long
// it can be hard to use normal PWM signal that's why we use the servo library. 
Servo servo;
Servo ESC;

// NRF24
RF24 radio(7, 8);                   // 7 - CE, 8 -CSN
const byte address[6] = "00001";    // Adress for communicating
unsigned long lastData = 0;         // Used for knowing if data has been received recently
typedef struct{
    int throttle;
    int turn;
}message_t;                         // Struct for making sending/receiving easy
message_t msg; 

// MPU6050
int16_t ax, ay, az;     // Acceleration X, Y, Z
int16_t gx, gy, gz;     // Gyro values X, Y, Z
int lastValue;          // Used for creating a low pass filter for turning angle
MPU6050 mpu;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
    Wire.begin();
    Wire.setClock(400000); // 400kHz I2C clock

    Serial.begin(9600);

    Serial.println("=============================================================")
    Serial.println("----- Welcome to CASE rf car controller by jazzy hands! -----");
    Serial.print("----- This device is a: ");
    

    #ifdef TRANSMITTER
        Serial.println("TRANSMITTER -----");

        // Setup NRF24 as transmitter
        radio.begin();
        radio.openWritingPipe(address);
        radio.setPALevel(RF24_PA_MAX); 
        radio.setDataRate(RF24_2MBPS);
        radio.stopListening();

        // Initialize MPU
        Serial.println(F("Initializing MPU..."));
        mpu.initialize();
        pinMode(INTERRUPT_PIN, INPUT);

        // Verify connection
        Serial.println(F("Testing MPU connections..."));
        Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

        // Load and configure the DMP
        Serial.println(F("Initializing DMP on MPU..."));
        devStatus = mpu.dmpInitialize();

        // Calibration data
        // see calibration.txt for more info. 
        mpu.setXGyroOffset(148);
        mpu.setYGyroOffset(68);
        mpu.setZGyroOffset(33);
        mpu.setZAccelOffset(1185);

        // make sure it worked (returns 0 if so)
        if (devStatus == 0) {
            // Calibration Time: generate offsets and calibrate our MPU6050
            mpu.CalibrateAccel(6);
            mpu.CalibrateGyro(6);
            mpu.PrintActiveOffsets();
            Serial.println(F("Enabling DMP..."));
            mpu.setDMPEnabled(true);

            // enable Arduino interrupt detection
            Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
            Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
            Serial.println(F(")..."));
            attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
            mpuIntStatus = mpu.getIntStatus();

            // set our DMP Ready flag so the main loop() function knows it's okay to use it
            Serial.println(F("DMP ready and running!"));
            dmpReady = true;

            // get expected DMP packet size for later comparison
            packetSize = mpu.dmpGetFIFOPacketSize();
        } else {
            // ERROR!
            // 1 = initial memory load failed
            // 2 = DMP configuration updates failed
            // (if it's going to break, usually the code will be 1)
            Serial.print(F("DMP Initialization failed (code "));
            Serial.print(devStatus);
            Serial.println(F(")"));
        }
    #else
        Serial.println("RECIEVER -----");

        // 1-2ms where 1.5ms is 90deg on servo and no throttle on ESC
        servo.attach(TURN_PIN, 1000, 2000);
        ESC.attach(THROTTLE_PIN, 1000, 2000);

        // Setup as receiver
        radio.begin();
        radio.openReadingPipe(0, address);
        radio.setPALevel(RF24_PA_MAX);
        radio.setDataRate(RF24_2MBPS);
        radio.startListening();
    #endif
}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
    #ifdef TRANSMITTER
        // if programming failed, don't try to do anything
        if (!dmpReady) return;

        // wait for MPU interrupt or extra packet(s) available
        while (!mpuInterrupt && fifoCount < packetSize) {
            if (mpuInterrupt && fifoCount < packetSize) {
            // try to get out of the infinite loop 
            fifoCount = mpu.getFIFOCount();
            }  
            // other program behavior stuff here
        }

        // reset interrupt flag and get INT_STATUS byte
        mpuInterrupt = false;
        mpuIntStatus = mpu.getIntStatus();

        // get current FIFO count
        fifoCount = mpu.getFIFOCount();
        if(fifoCount < packetSize){
                //Lets go back and wait for another interrupt. We shouldn't be here, we got an interrupt from another event
                // This is blocking so don't do it   while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
        }
        // check for overflow (this should never happen unless our code is too inefficient)
        else if ((mpuIntStatus & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024) {
            // reset so we can continue cleanly
            mpu.resetFIFO();
            //  fifoCount = mpu.getFIFOCount();  // will be zero after reset no need to ask
            Serial.println(F("FIFO overflow!"));

        // otherwise, check for DMP data ready interrupt (this should happen frequently)
        } else if (mpuIntStatus & _BV(MPU6050_INTERRUPT_DMP_INT_BIT)) {

            // read a packet from FIFO
            while(fifoCount >= packetSize){ 
    
                mpu.getFIFOBytes(fifoBuffer, packetSize);
                // track FIFO count here in case there is > 1 packet available
                // (this lets us immediately read more without waiting for an interrupt)
                fifoCount -= packetSize;
            }

        // Get raw motion data
        mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
        // We use gravity vector on X and Y axis to determine how to accelerate/decelerate and turn.
        // This makes it possible to walk around in a room and rotate your entire body without affecting which way turns where.
        // You will know what I mean if you try and use absolute orientation angles.

        // The RC car is extremly fast (40+ km/h )
        // We have to map the accelerometer value from min/max to servo signal (0-180) and scale it to reduce max speed 
        // A accelerometer value of 0 means no acceleration and should map to 90 which means no throttle. 0 is full reverse and 180 full forward.
        msg.throttle = map(ax, -17000, 17000, 80, 110);
        // The ESC has slower backward speed than forward so we inscrease the speed backwards to make it easier to use.
        if(msg.throttle < 88){
            msg.throttle = msg.throttle / 2;
        }

        // We want the full motion for turning so we only map the acceleration value to a value form 0-180 where 0 maps to 90 (no turn)
        msg.turn = map(ay, -17000, 17000, 0, 180);
        // We apply a crude low pass filter on the turn signal to reduce vibrations in the servo.
        msg.turn = 0.8 * msg.turn + 0.2 * lastValue;
        lastValue = msg.turn;

        #ifdef DEBUG
            Serial.print("Data - ax, ay\t");
            Serial.print( msg.throttle );
            Serial.print("\t");
            Serial.println( msg.turn );
        #endif

        // Send the data
        radio.write(&msg, sizeof(msg));        
        }
    #else  // Receiver
        if (radio.available()) {
            // Read and store the data in the struct. 
            message_t data; 
            radio.read(&data, sizeof(data));

            #ifdef DEBUG
            Serial.print("RECIEVED - throttle, turn:\t");
            Serial.print(data.throttle);
            Serial.print("\t");
            Serial.println(data.turn);
            #endif

            // The Receiver does no processing of data
            // This makes it possible to only update the transmitter when updating code, neat! :) 
            servo.write( data.turn );
            ESC.write( data.throttle );

            // Udate that we got data
            lastData = millis();
        }else{
            //If there hasn't been data for 200ms (something went wrong or transmitter power off), send default values to the car and stop it.
            if(millis() - lastData > 200){
                servo.write(90);
                ESC.write(90);
            }
        }
    #endif
}