#include <math.h>
#include <SoftwareSerial.h>
#include <ros.h>
#include <std_msgs/Float64MultiArray.h>
#include "ros.h"
#include "geometry_msgs/Twist.h"

#define left_first_value 0
/////////////////// 로봇팔 길이 고려해서 목표 x y값에 빼고 더해야함(27cm , 45cm) 전체 test 전 arm_mission =0 으로 초기화 mission이 1일때 돌다가 멈추다가 돌다가 멈추다가 이렇게 하기
int mission_3_count=0;

float arm_mission = 0;

double last_delta_s = 0;
double last_delta_o = 0;
double target_delta_s = 0;
double target_delta_o = 0;

float target_x = 0;
float target_y = 0;
float topic_remove = 0;

double last_x = 0;
double last_y = 0;
double last_th = 0;

double micro_x = 0;
double micro_y = 0;
double micro_th = 0;

int micro_check_s = 0;
int micro_check_o = 0;


int overload = 0;
double test = 0;
int input_gain = 1;    // ros에 주는 각도 값에 곱해지는거
int receive_gain = 1;  // 받는 각도값에 곱해지는거
double m_s = 0;
double m_th = 0;
int s_count = 0;
int o_count = 0;
int main_count = 0;

int xy_count = 0;
int th_count = 0;

//PID
float speed = 0;
float speed_r = 0;
float slowdown = 0;
float slowdown_r = 0;

int success = 0;


float Kp = 5, Kd = 2, Ki = 10;
float P_term = 0, D_term = 0, I_term = 0;
float pid = 0;


float Kp_a = 2.5, Kd_a = 0.65, Ki_a = 3.125;
float P_term_a = 0, D_term_a = 0, I_term_a = 0;
float pid_a = 0;


float error = 0;  //dis
float previous_error = 0;
float old_error = 0;


float error_a = 0;  //angle
float previous_error_a = 0;
float old_error_a = 0;

//Dead Reckoning
double first_car_angle = 0;

double car_angle = 0;
double current_x = 0;
double current_y = 0;


double previous_car_angle = 0;
double previous_current_x = 0;
double previous_current_y = 0;


double setting_car_angle = 0;
double setting_x = 0;
double setting_y = 0;

double delta_s = 0;
double delta_o = 0;

double input_delta_o = 0;
double pid_delta_o = 0;



double ll = 0.515;
double rr = 0.0625;

double s_init = 0;

double s_save = 0;
double th_save = 0;

//ros publisher
float topic_x = 0;
float topic_y = 0;
float topic_th = 0;

ros::NodeHandle nh;
std_msgs::Float64MultiArray float_array_msg;
ros::Publisher parrot("parrot", &float_array_msg);

float send_state = 0.0;
std_msgs::Float64MultiArray state_msg;
ros::Publisher motor_state("motor_state", &state_msg);



//ros subscriber
int stop_flag = 0;
float cmd_vel_x = 0;
float cmd_vel_y = 0;
float cmd_vel_z = 0;  //angle velocity

float cmd_vel_s = 0;
float cmd_vel_th = 0;

float mission = 0;
float old_mission = 0;
float mission_event = 0;


void velCallback(const geometry_msgs::Twist& vel) {
  cmd_vel_x = vel.linear.x;
  cmd_vel_y = vel.linear.y;
  cmd_vel_z = vel.angular.z;
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", velCallback);

std_msgs::Float64MultiArray curl_mission;

double tan_box = 0;
int curl_count = 0;


void curlCallback(const std_msgs::Float64MultiArray& msg) {
  if (msg.data_length >= 4) {
    curl_mission = msg;
    mission = curl_mission.data[0];
    target_x = -curl_mission.data[1];
    topic_remove = curl_mission.data[2];
    target_y = curl_mission.data[3];

    if (target_x != 0 || target_y != 0) {
      curl_count++;
      if (curl_count == 1) {
        target_delta_s = sqrt(target_x * target_x + target_y * target_y) - 0.27-0.05;
        if (target_x == 0) {
          target_delta_o = 0;
        } else {
          tan_box = target_x / (target_y-0.05+0.31);
          target_delta_o = -atan(tan_box);
        }
        last_x = current_x;
        last_y = current_y;
        last_th = car_angle;
      }
    }

    if (mission != old_mission) {
      mission_event = old_mission;
    }
    old_mission = mission;
  }
}

ros::Subscriber<std_msgs::Float64MultiArray> sub2("jetson_state", &curlCallback);

std_msgs::Float64MultiArray arm_state;

void arm_Callback(const std_msgs::Float64MultiArray& msg) {
  if (msg.data_length >= 1) {
    arm_state = msg;
    arm_mission = arm_state.data[0];
  }
}

ros::Subscriber<std_msgs::Float64MultiArray> sub3("arm_state", &arm_Callback);

char old_direction;
char old_old_direction;

int check_direction = 0;
char direction;
char velocity;
int gain_velocity;



unsigned long prev_time;
float prev_x = 0;
float prev_y = 0;
float prev_car_angle = 0;
unsigned long current_time = 0;
float dt = 0;
SoftwareSerial BLE_Serial(50, 51);  // RX, TX
char rx_buf[3];

// Pin select
const int encoderAPin = 2;
const int encoderBPin = 3;

const int motorPWM = 9;
const int motorIN1 = 22;
const int motorIN2 = 23;

const int encoderAPin_r = 20;
const int encoderBPin_r = 21;

const int motorPWM_r = 8;
const int motorIN1_r = 27;
const int motorIN2_r = 25;

//RPM

double rpm_type_m = 0;

double rpm_type_m_r = 0;





//angle
volatile long long int encoderCount = 0;    //left
volatile long long int encoderCount_r = 0;  //right

volatile long long int rpm_encoderCount = 0;
volatile long long int rpm_encoderCount_r = 0;

const long targetCount = 4028;  // run one cycle ->get pulse

volatile long long int pr_encoderCount = 0;    //left
volatile long long int pr_encoderCount_r = 0;  //right

double angle = 0.0015598771864894704;



double current_angle = 0;

double target_angle = 0;

double current_angle_r = 0;

double target_angle_r = 0;



//distance

double wheel = 0.225 * PI;  // wheel Round

double target_length = 0;

double current_distance = 0;
double current_distance_r = 0;




//Last Move
float error_d = 0;
float previous_error_d = 0;
float old_error_d = 0;

float error_a_d = 0;
float previous_error_a_d = 0;
float old_error_a_d = 0;

float P_term_d = 0;
float D_term_d = 0;
float I_term_d = 0;
float pid_d = 0;

float P_term_a_d = 0;
float D_term_a_d = 0;
float I_term_a_d = 0;
float pid_a_d = 0;

float Kp_d = 0.625, Kd_d = 0.25, Ki_d = 1.25;
float Kp_a_d = 0.078125, Kd_a_d = 0.03125, Ki_a_d = 0.1375;


//otherwise

unsigned int tp = 0, tc = 0;
unsigned int tp_m = 0, tc_m = 0;
unsigned int priority_tp = 0, priority_tc = 0;

boolean control_1 = 1, control_2 = 0;      // 0,1 CW 1,0 CCW
boolean control_1_r = 0, control_2_r = 1;  // 0,1 CW 1,0 CCW

int move_state = 0;  // front:0 back:1
int move_state_r = 0;
boolean ble_check = 0;

void setup() {
  delay(5000);

  nh.initNode();
  nh.advertise(motor_state);
  nh.advertise(parrot);
  nh.subscribe(sub);
  nh.subscribe(sub2);
  nh.subscribe(sub3);


  float_array_msg.data_length = 10;  // Adjust the length according to the number of elements
  float_array_msg.data = new float[float_array_msg.data_length];

  state_msg.data_length = 1;
  state_msg.data = new float[state_msg.data_length];

  pinMode(encoderAPin, INPUT);
  pinMode(encoderBPin, INPUT);

  pinMode(motorPWM, OUTPUT);
  pinMode(motorIN1, OUTPUT);
  pinMode(motorIN2, OUTPUT);

  pinMode(encoderAPin_r, INPUT);
  pinMode(encoderBPin_r, INPUT);

  pinMode(motorPWM_r, OUTPUT);
  pinMode(motorIN1_r, OUTPUT);
  pinMode(motorIN2_r, OUTPUT);

  Serial.begin(115200);
  BLE_Serial.begin(9600);
  motorControl(0, 0);


  attachInterrupt(digitalPinToInterrupt(encoderAPin), handleAEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderBPin), handleBEncoder, CHANGE);

  attachInterrupt(digitalPinToInterrupt(encoderAPin_r), handleAEncoder_r, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderBPin_r), handleBEncoder_r, CHANGE);

  target_length = 0;
  target_angle = target_length * angle * targetCount / wheel;
  target_angle_r = target_length * angle * targetCount / wheel;

  tp = millis();
  tp_m = millis();
  prev_time = millis();
}


void loop() {


  motorControl(control_1, control_2);
  motorControl_r(control_1_r, control_2_r);
  analogWrite(motorPWM, fabs(speed));
  analogWrite(motorPWM_r, fabs(speed_r));

  current_time = millis();

  nh.spinOnce();

  if (current_time - prev_time >= 50) {
    current_angle = ((rpm_encoderCount - pr_encoderCount) * angle);

    current_angle_r = ((rpm_encoderCount_r - pr_encoderCount_r) * angle);

    rpm_type_m = 60 * (fabs(rpm_encoderCount) - fabs(pr_encoderCount)) / 0.05 / targetCount;
    pr_encoderCount = rpm_encoderCount;

    rpm_type_m_r = 60 * (fabs(rpm_encoderCount_r) - fabs(pr_encoderCount_r)) / 0.05 / targetCount;
    pr_encoderCount_r = rpm_encoderCount_r;

    delta_s = ((current_angle_r + current_angle) * rr) / 2;

    delta_o = ((current_angle_r - current_angle) * rr) / ll;

    input_delta_o = previous_car_angle + (delta_o / 2);

    current_x = previous_current_x + cos((input_delta_o)) * delta_s;
    current_y = previous_current_y + sin((input_delta_o)) * delta_s;
    car_angle = previous_car_angle + delta_o;

    pid_delta_o = delta_o;


    topic_x += current_x - previous_current_x;
    topic_y += current_y - previous_current_y;
    topic_th += car_angle - previous_car_angle;
    test = car_angle - previous_car_angle;

    previous_car_angle = car_angle;
    previous_current_x = current_x;
    previous_current_y = current_y;

    encoderCount = 0;
    encoderCount_r = 0;
    m_s = delta_s / 0.05;

    last_delta_s = sqrt((current_x - last_x) * (current_x - last_x) + (current_y - last_y) * (current_y - last_y));
    last_delta_o = (car_angle - last_th);

    dt = (current_time - prev_time) / 1000.0;

    float_array_msg.data[0] = topic_x;
    float_array_msg.data[1] = topic_y;
    float_array_msg.data[2] = input_gain * topic_th;
    float_array_msg.data[3] = dt;
    float_array_msg.data[4] = mission;
    float_array_msg.data[5] = target_delta_s;
    float_array_msg.data[6] = target_delta_o;
    float_array_msg.data[7] = error_d;
    float_array_msg.data[8] = error_a_d;
    float_array_msg.data[9] = xy_count;





    parrot.publish(&float_array_msg);
    prev_time = current_time;

    state_msg.data[0] = send_state;

    motor_state.publish(&state_msg);



    if (mission == 0) {


      error = (cmd_vel_s - delta_s / 0.05);
      error_a = (cmd_vel_th - pid_delta_o / 0.05);

      P_term = (error - previous_error) * Kp;
      D_term = (error - 2 * previous_error + old_error) * Kd;
      I_term = error * Ki;
      pid = P_term + D_term + I_term;


      P_term_a = (error_a - previous_error_a) * Kp_a;
      D_term_a = (error_a - 2 * previous_error_a + old_error_a) * Kd_a;
      I_term_a = error_a * Ki_a;
      pid_a = P_term_a + D_term_a + I_term_a;

      if (stop_flag == 0) {
        speed = speed + pid - pid_a;
        speed_r = speed_r + pid + pid_a;
      } else {
        speed = speed - (speed) / 5;
        speed_r = speed_r - (speed_r) / 5;
      }

      old_error = previous_error;
      old_error_a = previous_error_a;
      previous_error = error;
      previous_error_a = error_a;
      //constant_value();
      check_pwm();
      check_pwm_r();


      check_move_state();
      check_move_state_r();

      cmd_vel_calculate();
    }

    else if (mission == 1) {
      if (abs(th_save) >= 0.19188888888888888888888888888889) {
        speed = 0;
        speed_r = 0;
        s_init = 0;
        s_save = 0;
        main_count++;
        if (main_count >= 80) {
          th_save = 0;
          main_count = 0;
        }
      } else {
        speed = 26;
        speed_r = -20;

        control_1 = 1, control_2 = 0;
        control_1_r = 1, control_2_r = 0;
        move_state = 0;
        move_state_r = 0;
        th_save += (delta_o);
      }
    }

    else if (mission == 2) {
      speed = 0;
      speed_r = 0;
      s_init = 0;
      s_save = 0;
      stop_flag = 0;

    }

    else if (mission == 3) {
      
      if(mission_3_count==0){
      speed = 0;
      speed_r = 0;
      s_init = 0;
      s_save = 0;
      stop_flag = 0; 
      }
      mission_3_count++;
      //미션을 나중에 1로 고치기
      if (arm_mission == 1) {
        if (th_count == 0) {
          error_a_d = (target_delta_o - last_delta_o);

          if (abs(error_a_d) <= 0.01) {
            th_count = 1;
            pid_a_d = 0;
            speed = 15;
            speed_r = 10;
            last_th = car_angle;
            last_x = current_x;
            last_y = current_y;
            last_delta_s=0;
            old_error_d = 0;
            old_error_a_d = 0;
            previous_error_d = 0;
            previous_error_a_d = 0;
            //last_delta_o 초기화 안시켜도 되나?

          } else {
            P_term_a_d = (error_a_d - previous_error_a_d) * Kp_a_d;
            D_term_a_d = (error_a_d - 2 * previous_error_a_d + old_error_a_d) * Kd_a_d;
            I_term_a_d = error_a_d * Ki_a_d;
            pid_a_d = P_term_a_d + D_term_a_d + I_term_a_d;
          }

          speed = speed - pid_a_d;
          speed_r = speed_r + pid_a_d;

          old_error_d = previous_error_d;
          old_error_a_d = previous_error_a_d;
          previous_error_d = error_d;
          previous_error_a_d = error_a_d;
          check_pwm();
          check_pwm_r();

          check_move_state();
          check_move_state_r();
        } else if (th_count == 1) {
          error_d = (target_delta_s - last_delta_s);
          error_a_d = (0 - last_delta_o);

          if (error_d <= 0.005) {
            pid_d = 0;
            pid_a_d = 0;
            xy_count = 1;
            speed = 0;
            speed_r = 0;
            th_count = 2;
            old_error_d = 0;
            old_error_a_d = 0;
            previous_error_d = 0;
            previous_error_a_d = 0;

          } else {
            P_term_d = (error_d - previous_error_d) * Kp_d;
            D_term_d = (error_d - 2 * previous_error_d + old_error_d) * Kd_d;
            I_term_d = error_d * Ki_d;
            pid_d = P_term_d + D_term_d + I_term_d;

            P_term_a_d = (error_a_d - previous_error_a_d) * Kp_a_d;
            D_term_a_d = (error_a_d - 2 * previous_error_a_d + old_error_a_d) * Kd_a_d;
            I_term_a_d = error_a_d * Ki_a_d;
            pid_a_d = P_term_a_d + D_term_a_d + I_term_a_d;
          }

          speed = speed + pid_d - pid_a_d;
          speed_r = speed_r + pid_d + pid_a_d;

          old_error_d = previous_error_d;
          old_error_a_d = previous_error_a_d;
          previous_error_d = error_d;
          previous_error_a_d = error_a_d;
          check_pwm();
          check_pwm_r();

          check_move_state();
          check_move_state_r();
        }

        else if (xy_count == 1) {
          error_a_d = (-target_delta_o - last_delta_o);
          if (abs(error_a_d) <= 0.01) {
            xy_count = 2;
            pid_a_d = 0;
            speed = 0;
            speed_r = 0;
            last_th = car_angle;
            old_error_d = 0;
            old_error_a_d = 0;
            previous_error_d = 0;
            previous_error_a_d = 0;
            send_state = 1.0;

          } else {
            P_term_a_d = (error_a_d - previous_error_a_d) * Kp_a_d;
            D_term_a_d = (error_a_d - 2 * previous_error_a_d + old_error_a_d) * Kd_a_d;
            I_term_a_d = error_a_d * Ki_a_d;
            pid_a_d = P_term_a_d + D_term_a_d + I_term_a_d;
          }

          speed = speed - pid_a_d;
          speed_r = speed_r + pid_a_d;

          old_error_d = previous_error_d;
          old_error_a_d = previous_error_a_d;
          previous_error_d = error_d;
          previous_error_a_d = error_a_d;
          check_pwm();
          check_pwm_r();

          check_move_state();
          check_move_state_r();
        }
      }
    }
  }
}



void capacitor_safe(void) {
  if (old_old_direction == '4' || old_old_direction == '6') {
    if (fabs(speed) > 35 || fabs(speed_r) > 35)
      direction = '5';
  }
  if (old_direction == '4' || old_direction == '6') {
    if (fabs(speed) > 35 || fabs(speed_r) > 35)
      direction = '5';
  }
}


void cmd_vel_calculate(void) {
  cmd_vel_s = cmd_vel_x;
  cmd_vel_th = receive_gain * cmd_vel_z;
  if (cmd_vel_x == 0 && cmd_vel_z == 0) {
    stop_flag = 1;
  } else {
    stop_flag = 0;
  }
}

void motorControl(int in1, int in2) {
  digitalWrite(motorIN1, in1);
  digitalWrite(motorIN2, in2);
}

void motorControl_r(int in1, int in2) {
  digitalWrite(motorIN1_r, in1);
  digitalWrite(motorIN2_r, in2);
}

void check_pwm() {
  if (speed >= 200) {
    speed = 200;
  } else if (speed <= -200) {
    speed = -200;
  }
}

void check_pwm_r() {
  if (speed_r >= 200) {
    speed_r = 200;
  } else if (speed_r <= -200) {
    speed_r = -200;
  }
}

void check_move_state() {
  if (speed >= 0) {
    move_state = 0;
    control_1 = 1, control_2 = 0;
  } else {
    move_state = 1;
    control_1 = 0, control_2 = 1;
  }
}

void check_move_state_r() {
  if (speed_r >= 0) {
    move_state_r = 0;
    control_1_r = 0, control_2_r = 1;
  } else {
    move_state_r = 1;
    control_1_r = 1, control_2_r = 0;
  }
}

double calculate_target_Angle(double setting_x, double setting_y, double first_car_angle) {

  double angle = first_car_angle;

  if (setting_x != 0 && setting_y != 0) {

    angle = atan(setting_y / setting_x);

    if (setting_x > 0) {
      if (setting_y > 0) {
        angle = first_car_angle + atan(setting_y / setting_x);
      } else {
        angle = first_car_angle + atan(setting_y / setting_x);
      }
    } else {
      if (setting_y > 0) {
        angle = first_car_angle + atan(setting_y / setting_x);

      } else {
        angle = first_car_angle + atan(setting_y / setting_x);
      }
    }
  } else {

    if (setting_x == 0) {
      if (setting_y > 0) {
        angle = first_car_angle + PI / 2;
      } else {
        angle = first_car_angle - PI / 2;
      }
    }
    if (setting_y == 0) {
      angle = first_car_angle;
    }
  }

  return angle;
}
void constant_value() {
  if (speed < left_first_value && speed > -left_first_value) {
    if (pid + pid_a >= 0) {
      speed = left_first_value;
    } else if (pid + pid_a <= 0) {
      speed = -left_first_value;
    }
  }
}

void handleAEncoder() {
  if (digitalRead(encoderAPin) == digitalRead(encoderBPin)) {
    encoderCount--;
    rpm_encoderCount--;

  } else {
    encoderCount++;
    rpm_encoderCount++;
  }
}

void handleBEncoder() {
  if (digitalRead(encoderAPin) == digitalRead(encoderBPin)) {
    encoderCount++;
    rpm_encoderCount++;
  } else {
    encoderCount--;
    rpm_encoderCount--;
  }
}

void handleAEncoder_r() {
  if (digitalRead(encoderAPin_r) == digitalRead(encoderBPin_r)) {
    encoderCount_r++;
    rpm_encoderCount_r++;

  } else {
    encoderCount_r--;
    rpm_encoderCount_r--;
  }
}

void handleBEncoder_r() {
  if (digitalRead(encoderAPin_r) == digitalRead(encoderBPin_r)) {
    encoderCount_r--;
    rpm_encoderCount_r--;
  } else {
    encoderCount_r++;
    rpm_encoderCount_r++;
  }
}

/*if (th_count == 0) {
        error_a_d = (target_delta_o - last_delta_o);

        if (abs(error_a_d) <= 0.01) {
          th_count = 1;
          pid_a_d = 0;
          speed = 0;
          speed_r = -5;
          last_th = car_angle;
          old_error_d = 0;
          old_error_a_d = 0;
          previous_error_d = 0;
          previous_error_a_d = 0;

        } else {
          P_term_a_d = (error_a_d - previous_error_a_d) * Kp_a_d;
          D_term_a_d = (error_a_d - 2 * previous_error_a_d + old_error_a_d) * Kd_a_d;
          I_term_a_d = error_a_d * Ki_a_d;
          pid_a_d = P_term_a_d + D_term_a_d + I_term_a_d;
        }

        speed = speed - pid_a_d;
        speed_r = speed_r + pid_a_d;

        old_error_d = previous_error_d;
        old_error_a_d = previous_error_a_d;
        previous_error_d = error_d;
        previous_error_a_d = error_a_d;
        check_pwm();
        check_pwm_r();

        check_move_state();
        check_move_state_r();
      } else if (th_count == 1) {
        error_d = (target_delta_s - last_delta_s);
        error_a_d = (0 - last_delta_o);

        if (error_d <= 0.05) {
          pid_d = 0;
          pid_a_d = 0;
          xy_count = 1;
          speed = 0;
          speed_r = 0;
          th_count = 2;
          old_error_d = 0;
          old_error_a_d = 0;
          previous_error_d = 0;
          previous_error_a_d = 0;

        } else {
          P_term_d = (error_d - previous_error_d) * Kp_d;
          D_term_d = (error_d - 2 * previous_error_d + old_error_d) * Kd_d;
          I_term_d = error_d * Ki_d;
          pid_d = P_term_d + D_term_d + I_term_d;

          P_term_a_d = (error_a_d - previous_error_a_d) * Kp_a_d;
          D_term_a_d = (error_a_d - 2 * previous_error_a_d + old_error_a_d) * Kd_a_d;
          I_term_a_d = error_a_d * Ki_a_d;
          pid_a_d = P_term_a_d + D_term_a_d + I_term_a_d;
        }

        speed = speed + pid_d - pid_a_d;
        speed_r = speed_r + pid_d + pid_a_d;

        old_error_d = previous_error_d;
        old_error_a_d = previous_error_a_d;
        previous_error_d = error_d;
        previous_error_a_d = error_a_d;
        check_pwm();
        check_pwm_r();

        check_move_state();
        check_move_state_r();
      }

      else if (xy_count == 1) {
        error_a_d = (-target_delta_o - last_delta_o);

        if (abs(error_a_d) <= 0.01) {
          xy_count = 2;
          pid_a_d = 0;
          speed = 0;
          speed_r = 0;
          last_th = car_angle;
          old_error_d = 0;
          old_error_a_d = 0;
          previous_error_d = 0;
          previous_error_a_d = 0;

        } else {
          P_term_a_d = (error_a_d - previous_error_a_d) * Kp_a_d;
          D_term_a_d = (error_a_d - 2 * previous_error_a_d + old_error_a_d) * Kd_a_d;
          I_term_a_d = error_a_d * Ki_a_d;
          pid_a_d = P_term_a_d + D_term_a_d + I_term_a_d;
        }

        speed = speed - pid_a_d;
        speed_r = speed_r + pid_a_d;

        old_error_d = previous_error_d;
        old_error_a_d = previous_error_a_d;
        previous_error_d = error_d;
        previous_error_a_d = error_a_d;
        check_pwm();
        check_pwm_r();

        check_move_state();
        check_move_state_r();
      }*/