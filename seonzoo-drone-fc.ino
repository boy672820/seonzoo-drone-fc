#include "I2Cdev.h"
#include <Servo.h>

#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 mpu;
#define OUTPUT_READABLE_YAWPITCHROLL

#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

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

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };

float pid_p_gain_roll = 1.3;               //Gain setting for the roll P-controller
float pid_i_gain_roll = 0.04;              //Gain setting for the roll I-controller
float pid_d_gain_roll = 18.0;              //Gain setting for the roll D-controller
int pid_max_roll = 400;                    //Maximum output of the PID-controller (+/-)

float pid_error_temp;
float pid_i_mem_roll, pid_roll_setpoint, gyro_roll_input, pid_output_roll, pid_last_roll_d_error;
int setpoint = 0;
int max_throttle = 50;
int min_throttle = 0;

Servo esc1, esc2, esc3, esc4;
String input_str = "";



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
  // join I2C bus (I2Cdev library doesn't do this automatically)
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
      Wire.begin();
      TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
      Fastwire::setup(400, true);
  #endif

  // initialize serial communication
  // (115200 chosen because it is required for Teapot Demo output, but it's
  // really up to you depending on your project)
  Serial.begin(115200);
  while (!Serial); // wait for Leonardo enumeration, others continue immediately

  // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3v or Ardunio
  // Pro Mini running at 3.3v, cannot handle this baud rate reliably due to
  // the baud timing being too misaligned with processor ticks. You must use
  // 38400 or slower in these cases, or use some kind of external separate
  // crystal solution for the UART timer.

  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // wait for ready
  Serial.println(F("\nSend any character to begin DMP programming and demo: "));
//    while (Serial.available() && Serial.read()); // empty buffer
//    while (!Serial.available());                 // wait for data
//    while (Serial.available() && Serial.read()); // empty buffer again

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
      // turn on the DMP, now that it's ready
      Serial.println(F("Enabling DMP..."));
      mpu.setDMPEnabled(true);

      // enable Arduino interrupt detection
      Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
      attachInterrupt(0, dmpDataReady, RISING);
      mpuIntStatus = mpu.getIntStatus();

      // set our DMP Ready flag so the main loop() function knows it's okay to use it
      Serial.println(F("DMP ready! Waiting for first interrupt..."));
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

  esc1.attach( 3 );  // Front-Left
  esc2.attach( 10 ); // Front-Right
  esc3.attach( 9 );  // Rear-Right
  esc4.attach( 11 ); // Rear-Left

  esc1.write( setpoint );
  esc2.write( setpoint );
  esc3.write( setpoint );
  esc4.write( setpoint );
}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
  // if programming failed, don't try to do anything
  if (!dmpReady) return;

  // wait for MPU interrupt or extra packet(s) available
  while (!mpuInterrupt && fifoCount < packetSize) {
      // other program behavior stuff here
      // .
      // .
      // .
      // if you are really paranoid you can frequently test in between other
      // stuff to see if mpuInterrupt is true, and if so, "break;" from the
      // while() loop to immediately process the MPU data
      // .
      // .
      // .
  }

  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
      // reset so we can continue cleanly
      mpu.resetFIFO();
      Serial.println(F("FIFO overflow!"));

  // otherwise, check for DMP data ready interrupt (this should happen frequently)
  } else if (mpuIntStatus & 0x02) {
      // wait for correct available data length, should be a VERY short wait
      while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

      // read a packet from FIFO
      mpu.getFIFOBytes(fifoBuffer, packetSize);
      
      // track FIFO count here in case there is > 1 packet available
      // (this lets us immediately read more without waiting for an interrupt)
      fifoCount -= packetSize;

      #ifdef OUTPUT_READABLE_QUATERNION
          // display quaternion values in easy matrix form: w x y z
          mpu.dmpGetQuaternion(&q, fifoBuffer);
//          Serial.print("quat\t");
//          Serial.print(q.w);
//          Serial.print("\t");
//          Serial.print(q.x);
//          Serial.print("\t");
//          Serial.print(q.y);
//          Serial.print("\t");
//          Serial.println(q.z);
      #endif

      #ifdef OUTPUT_READABLE_EULER
          // display Euler angles in degrees
          mpu.dmpGetQuaternion(&q, fifoBuffer);
          mpu.dmpGetEuler(euler, &q);
//          Serial.print("euler\t");
//          Serial.print(euler[0] * 180/M_PI);
//          Serial.print("\t");
//          Serial.print(euler[1] * 180/M_PI);
//          Serial.print("\t");
//          Serial.println(euler[2] * 180/M_PI);
      #endif

      #ifdef OUTPUT_READABLE_YAWPITCHROLL
          // display Euler angles in degrees
          mpu.dmpGetQuaternion(&q, fifoBuffer);
          mpu.dmpGetGravity(&gravity, &q);
          mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
//          Serial.print("ypr\t");
//          Serial.print(ypr[0] * 180/M_PI);
//          Serial.print("\t");
//          Serial.print(ypr[1] * 180/M_PI);
//          Serial.print("\t");
//          Serial.println(ypr[2] * 180/M_PI);
      #endif

      #ifdef OUTPUT_READABLE_REALACCEL
          // display real acceleration, adjusted to remove gravity
          mpu.dmpGetQuaternion(&q, fifoBuffer);
          mpu.dmpGetAccel(&aa, fifoBuffer);
          mpu.dmpGetGravity(&gravity, &q);
          mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
//          Serial.print("areal\t");
//          Serial.print(aaReal.x);
//          Serial.print("\t");
//          Serial.print(aaReal.y);
//          Serial.print("\t");
//          Serial.println(aaReal.z);
      #endif

      #ifdef OUTPUT_READABLE_WORLDACCEL
          // display initial world-frame acceleration, adjusted to remove gravity
          // and rotated based on known orientation from quaternion
          mpu.dmpGetQuaternion(&q, fifoBuffer);
          mpu.dmpGetAccel(&aa, fifoBuffer);
          mpu.dmpGetGravity(&gravity, &q);
          mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
          mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
//          Serial.print("aworld\t");
//          Serial.print(aaWorld.x);
//          Serial.print("\t");
//          Serial.print(aaWorld.y);
//          Serial.print("\t");
//          Serial.println(aaWorld.z);
      #endif
  
      #ifdef OUTPUT_TEAPOT
          // display quaternion values in InvenSense Teapot demo format:
          teapotPacket[2] = fifoBuffer[0];
          teapotPacket[3] = fifoBuffer[1];
          teapotPacket[4] = fifoBuffer[4];
          teapotPacket[5] = fifoBuffer[5];
          teapotPacket[6] = fifoBuffer[8];
          teapotPacket[7] = fifoBuffer[9];
          teapotPacket[8] = fifoBuffer[12];
          teapotPacket[9] = fifoBuffer[13];
//          Serial.write(teapotPacket, 14);
          teapotPacket[11]++; // packetCount, loops at 0xFF on purpose
      #endif

      // blink LED to indicate activity
      blinkState = !blinkState;
//      digitalWrite(LED_PIN, blinkState);
  }
  
  /** Sub routine */
  calculate_pid();
  serial_esc();

  gyro_roll_input = ypr[ 1 ];
}

/**
 * calculate_pid()
 */
void calculate_pid() {
  //Roll calculations
  pid_error_temp = gyro_roll_input - pid_roll_setpoint;
  pid_i_mem_roll += pid_i_gain_roll * pid_error_temp;
  if(pid_i_mem_roll > pid_max_roll)pid_i_mem_roll = pid_max_roll;
  
  else if(pid_i_mem_roll < pid_max_roll * -1)pid_i_mem_roll = pid_max_roll * -1;

  pid_output_roll = pid_p_gain_roll * pid_error_temp + pid_i_mem_roll + pid_d_gain_roll * (pid_error_temp - pid_last_roll_d_error);
  if(pid_output_roll > pid_max_roll)pid_output_roll = pid_max_roll;
  else if(pid_output_roll < pid_max_roll * -1)pid_output_roll = pid_max_roll * -1;

  pid_last_roll_d_error = pid_error_temp;
}

/**
 * serial_esc()
 */
void serial_esc() {
  while ( Serial.available() ) {
    char in_char = ( char ) Serial.read();
    input_str += in_char;
  }

  if ( ! input_str.equals( "" ) ) {

    if ( input_str.equals( "a" ) ) setpoint += 5;
    else if ( input_str.equals( "b" ) ) setpoint -= 5;

    input_str = "";
  }

  int roll_left_setpoint = 0, roll_right_setpoint = 0;

  if ( pid_output_roll > 0 ) {
    roll_left_setpoint = ( int ) pid_output_roll;
  }
  if ( pid_output_roll < 0 ) {
    roll_right_setpoint = ( int ) pid_output_roll * -1;
  }

  roll_left_setpoint += setpoint;
  roll_right_setpoint += setpoint;

  if ( roll_left_setpoint > max_throttle ) roll_left_setpoint = max_throttle;
  if ( roll_right_setpoint > max_throttle ) roll_right_setpoint = max_throttle;
  if ( roll_left_setpoint < min_throttle ) roll_left_setpoint = min_throttle;
  if ( roll_right_setpoint < min_throttle ) roll_right_setpoint = min_throttle;
  
  Serial.print( "roll_left: " );  Serial.print( roll_left_setpoint );
  Serial.print( " | " );
  Serial.print( "roll_right: " ); Serial.print( roll_right_setpoint );
  Serial.println();

//  esc1.write( roll_left_setpoint );
//  esc4.write( roll_left_setpoint );
//  esc2.write( roll_right_setpoint );
//  esc3.write( roll_right_setpoint );
}
