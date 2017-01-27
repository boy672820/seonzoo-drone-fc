#include <Wire.h>
#include <Servo.h>

#define mpu_add 0x68 //mpu6050 address
 
long ac_x, ac_y, ac_z, gy_x, gy_y, gy_z; //acc, gyro data 
 
double deg_x, deg_y, deg_z; // angle, deg data
double dgy_y, dgy_x, dgy_z; //double type acc data
double angle_roll = 0, angle_pitch = 0, angle_yaw = 0;

float pid_p_gain_roll = 1.3;               //Gain setting for the roll P-controller
float pid_i_gain_roll = 0.04;              //Gain setting for the roll I-controller
float pid_d_gain_roll = 18.0;              //Gain setting for the roll D-controller
int pid_max_roll = 400;                    //Maximum output of the PID-controller (+/-)

float pid_error_temp;
float pid_i_mem_roll, pid_roll_setpoint, gyro_roll_input, pid_output_roll, pid_last_roll_d_error;
int setpoint = 0;
int max_throttle = 200;
int min_throttle = 0;

Servo esc1, esc2, esc3, esc4;
String input_str = "";


/**
 * Arduino main setup routine
 */
void setup() {
  Serial.begin(115200);

  Wire.begin();  //set I2C
  Wire.beginTransmission(mpu_add);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  
  esc1.attach( 3 );  // Front-Left
  esc2.attach( 10 ); // Front-Right
  esc3.attach( 9 );  // Rear-Right
  esc4.attach( 11 ); // Rear-Left

  esc1.write( setpoint );
  esc2.write( setpoint );
  esc3.write( setpoint );
  esc4.write( setpoint );
}


/**
 * Arduino main loop
 */
void loop() {
  calculate_pid();
  angle();
  serial_esc();

  gyro_roll_input = angle_roll;
}


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


void angle() {
  Wire.beginTransmission(mpu_add) ; //get acc data
  Wire.write(0x3B) ;
  Wire.endTransmission(false) ;
  Wire.requestFrom(mpu_add, 6, true) ;
  ac_x = Wire.read() << 8 | Wire.read() ;
  ac_y = Wire.read() << 8 | Wire.read() ;
  ac_z = Wire.read() << 8 | Wire.read() ;
  Wire.beginTransmission(mpu_add) ; //get gyro data
  Wire.write(0x43) ;
  Wire.endTransmission(false) ;
  Wire.requestFrom(mpu_add, 6, true) ;
  gy_x = Wire.read() << 8 | Wire.read() ;
  gy_y = Wire.read() << 8 | Wire.read() ;
  gy_z = Wire.read() << 8 | Wire.read() ;

  deg_x = atan2(ac_x, ac_z) * 180 / PI ;  //rad to deg
  deg_y = atan2(ac_y, ac_z) * 180 / PI ;
  deg_z = atan2(ac_x, ac_y) * 180 / PI ;
  dgy_x = gy_y / 131. ;  //16-bit data to 250 deg/sec
  dgy_y = gy_z / 131. ;  //16-bit data to 250 deg/sec
  dgy_z = gy_x / 131. ;  //16-bit data to 250 deg/sec
  angle_roll = (0.95 * (angle_roll + (dgy_x * 0.001))) + (0.05 * deg_x) ; //complementary filter
  angle_pitch = (0.95 * (angle_pitch + (dgy_y * 0.001))) + (0.05 * deg_y) ;
  angle_yaw = (0.95 * (angle_yaw + (dgy_z * 0.001))) + (0.05 * deg_z) ;
}


void serial_esc() {
  int roll_left_setpoint = 0, roll_right_setpoint = 0;

  while ( Serial.available() ) {
    char in_char = ( char ) Serial.read();
    input_str += in_char;
  }

  if ( ! input_str.equals( "" ) ) {

    if ( input_str.equals( "a" ) ) setpoint += 5;
    else if ( input_str.equals( "b" ) ) setpoint -= 5;

    input_str = "";
  }

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
  
//  Serial.print( "roll_left: " );  Serial.print( roll_left_setpoint );
//  Serial.print( " | " );
//  Serial.print( "roll_right: " ); Serial.print( roll_right_setpoint );
//  Serial.println();
//  delay( 15 );

  esc1.write( roll_left_setpoint );
  esc4.write( roll_left_setpoint );
  esc2.write( roll_right_setpoint );
  esc3.write( roll_right_setpoint );
}
