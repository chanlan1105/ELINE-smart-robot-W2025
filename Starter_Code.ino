/*
 * @Author: ELEGOO
 * @Date: 2019-10-22 11:59:09
 * @LastEditTime: 2020-12-18 14:14:35
 * @LastEditors: Changhua
 * @Description: Smart Robot Car V4.0
 * @FilePath: 
 */
#include <avr/wdt.h>
#include "MPU6050_6Axis_MotionApps20.h"
#include "ApplicationFunctionSet_xxx0.h"

#define debug 1

MPU6050 mpu;

int const INTERRUPT_PIN = 2;  // Define the interruption #0 pin
bool blinkState;

/*---MPU6050 Control/Status Variables---*/
bool DMPReady = false;  // Set true if DMP init was successful
uint8_t MPUIntStatus;   // Holds actual interrupt status byte from MPU
uint8_t devStatus;      // Return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // Expected DMP packet size (default is 42 bytes)
uint8_t FIFOBuffer[64]; // FIFO storage buffer

/*---Orientation/Motion Variables---*/ 
Quaternion q;           // [w, x, y, z]         Quaternion container
VectorInt16 gy;         // [x, y, z]            Gyro sensor measurements
VectorFloat gravity;    // [x, y, z]            Gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   Yaw/Pitch/Roll container and gravity vector

/*------Interrupt detection routine------*/
volatile bool MPUInterrupt = false;     // Indicates whether MPU6050 interrupt pin has gone high
void DMPDataReady() {
  MPUInterrupt = true;
}

void setup()
{
    // put your setup code here, to run once:
    Application_FunctionSet.ApplicationFunctionSet_Init();
    wdt_enable(WDTO_2S);

    #if debug == 0
        #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
            Wire.begin();
            Wire.setClock(400000); // 400kHz I2C clock. Comment on this line if having compilation difficulties
        #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
            Fastwire::setup(400, true);
        #endif

        Serial.begin(9600); // Begin serial output
        while (!Serial);

        /*Initialize device*/
        Serial.println(F("Initializing I2C devices..."));
        mpu.initialize();
        pinMode(INTERRUPT_PIN, INPUT);

        /*Verify connection*/
        Serial.println(F("Testing MPU6050 connection..."));
        if (mpu.testConnection() == false) {
            Serial.println("MPU6050 connection failed");
            while(true);
        }
        else {
            Serial.println("MPU6050 connection successful");
        }

        /* Initialize and configure the DMP*/
        Serial.println(F("Initializing DMP..."));
        devStatus = mpu.dmpInitialize();

        /* Supply your gyro offsets here, scaled for min sensitivity */
        mpu.setXGyroOffset(0);
        mpu.setYGyroOffset(0);
        mpu.setZGyroOffset(0);
        mpu.setXAccelOffset(0);
        mpu.setYAccelOffset(0);
        mpu.setZAccelOffset(0);

        /* Making sure it worked (returns 0 if so) */ 
        if (devStatus == 0) {
            mpu.CalibrateAccel(6);  // Calibration Time: generate offsets and calibrate our MPU6050
            mpu.CalibrateGyro(6);
            Serial.println("These are the Active offsets: ");
            mpu.PrintActiveOffsets();
            Serial.println(F("Enabling DMP..."));   //Turning ON DMP
            mpu.setDMPEnabled(true);

            /*Enable Arduino interrupt detection*/
            Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
            Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
            Serial.println(F(")..."));
            attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), DMPDataReady, RISING);
            MPUIntStatus = mpu.getIntStatus();

            /* Set the DMP Ready flag so the main loop() function knows it is okay to use it */
            Serial.println(F("DMP ready! Waiting for first interrupt..."));
            DMPReady = true;
            packetSize = mpu.dmpGetFIFOPacketSize(); //Get expected DMP packet size for later comparison
        } 
        else {
            Serial.print(F("DMP Initialization failed (code ")); //Print the error code
            Serial.print(devStatus);
            Serial.println(F(")"));
            // 1 = initial memory load failed
            // 2 = DMP configuration updates failed
        }
    #endif
}

void loop()
{
    //put your main code here, to run repeatedly :
    wdt_reset();
    // Application_FunctionSet.ApplicationFunctionSet_SensorDataUpdate();
    // Application_FunctionSet.ApplicationFunctionSet_KeyCommand();
    Application_FunctionSet.ApplicationFunctionSet_RGB();
    // Application_FunctionSet.ApplicationFunctionSet_Follow();
    // Application_FunctionSet.ApplicationFunctionSet_Obstacle();
    // Application_FunctionSet.ApplicationFunctionSet_Tracking();
    Application_FunctionSet.ApplicationFunctionSet_Rocker();
    Application_FunctionSet.ApplicationFunctionSet_Standby();
    Application_FunctionSet.ApplicationFunctionSet_IRrecv();
    Application_FunctionSet.ApplicationFunctionSet_SerialPortDataAnalysis();

    // Application_FunctionSet.CMD_ServoControl_xxx0();
    // Application_FunctionSet.CMD_MotorControl_xxx0();
    // Application_FunctionSet.CMD_CarControlTimeLimit_xxx0();
    // Application_FunctionSet.CMD_CarControlNoTimeLimit_xxx0();
    // Application_FunctionSet.CMD_MotorControlSpeed_xxx0();
    // Application_FunctionSet.CMD_LightingControlTimeLimit_xxx0();
    // Application_FunctionSet.CMD_LightingControlNoTimeLimit_xxx0();
    // Application_FunctionSet.CMD_ClearAllFunctions_xxx0();

    #if debug == 0
        if (!DMPReady) return; // Stop if DMP programming fails.

        /* Read a packet from FIFO */
        if (mpu.dmpGetCurrentFIFOPacket(FIFOBuffer)) { // Get the Latest packet 
            /* Display Euler angles in degrees */
            mpu.dmpGetQuaternion(&q, FIFOBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            Serial.print("ypr\t");
            Serial.print(ypr[0] * 180/M_PI);
            Serial.print("\t");
            Serial.print(ypr[1] * 180/M_PI);
            Serial.print("\t");
            Serial.println(ypr[2] * 180/M_PI);
        }
    #endif
}
