// Include Libraries to be used
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32.h>
#include <stdio.h>
#include <math.h>

// Define Macros
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// Define GPIO pins
#define LED_PIN 23     
#define PWM_PIN 27     
#define In1  12        
#define In2  14        
#define EncA 25        
#define EncB 26        

// PWM Settings
#define PWM_FRQ 5000   
#define PWM_RES 8      
#define PWM_CHNL 0     

// Define range for RPM and PWM
#define RPM_MIN_VAL -160
#define RPM_MAX_VAL 160
#define PWM_MIN_VAL -255
#define PWM_MAX_VAL 255

// PID Parameters
float kp = 0.89, ki = 0.983, kd = 0.00468 , N = 500; 
float sample_time = 0.0002439; 
float integral = 0.0, prev_error = 0.0, prev_derivative = 0.0;
float prev_pwm_set_point = 0.0;
float set_point = 0.0;
float control_signal = 0.0;



int32_t tiempo_act = 0, tiempo_ant = 0, delta_tiempo = 2e9;
float posicion = 0, posactual = 0, posanterior = 0, velocidad = 0;
float resolucion = 0.5056;  
int pulsos = 712;      
int32_t contador = 0;
volatile bool BSet = 0;
volatile bool ASet = 0;
volatile bool encoderDirection = false;
int pwm = 0;        

// Declare Micro-ROS entities
rcl_node_t node;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_subscription_t setpoint_sub;
rcl_publisher_t rpm_publisher;
rcl_publisher_t error_publisher;
rcl_timer_t timer;
std_msgs__msg__Float32 msg;
rcl_publisher_t pwm_publisher;

void error_loop() {
  while (1) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN)); 
    delay(100);
  }
}

// Función para leer las señales del encoder y definir el sentido de giro
void IRAM_ATTR Encoder() {
  bool BNew = digitalRead(EncB);
  bool ANew = digitalRead(EncA);
  if (BNew == ANew) {
    if (!encoderDirection) { contador++; }
    encoderDirection = true;
  } else {
    if (encoderDirection) { contador--; }
    encoderDirection = false;
  }
  tiempo_act = micros();
  delta_tiempo = tiempo_act - tiempo_ant;
  tiempo_ant = tiempo_act;
}

// Motor direction control functions
void derecha() {
  digitalWrite(In1, HIGH);
  digitalWrite(In2, LOW);
}

void izquierda() {
  digitalWrite(In1, LOW);
  digitalWrite(In2, HIGH);
}

void detener_motor() {
  digitalWrite(In1, LOW);
  digitalWrite(In2, LOW);
  ledcWrite(PWM_CHNL, 0);
}

void set_point_callback(const void * msgin) {  
  const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *)msgin;
  float new_set_point = constrain(msg->data, RPM_MIN_VAL, RPM_MAX_VAL);

  // Ajusta el paso dependiendo de la magnitud del cambio
  float max_step_change = fabs(new_set_point - set_point) * 0.2; 
  max_step_change = constrain(max_step_change, 1.0, 10.0); 
  
  if (fabs(new_set_point - set_point) > max_step_change) {
    set_point += (new_set_point > set_point) ? max_step_change : -max_step_change;
  } else {
    set_point = new_set_point;
  }
}

void pose() {
  int32_t delta_tiempo_local;
  
  noInterrupts();  
  delta_tiempo_local = delta_tiempo;
  interrupts();    

  if (delta_tiempo_local > 0) {  
    velocidad = 60000000.0 / (pulsos * delta_tiempo_local);
  } else {
    velocidad = 0;  
  }

  if (!encoderDirection) {
    velocidad *= -1;  
  }
}

void control_loop_callback(rcl_timer_t * timer, int64_t last_call_time) {
  if (timer == NULL) return;

  float error = set_point - velocidad;

  // Integración con Anti-Windup
  if (!((control_signal >= RPM_MAX_VAL && error > 0) || (control_signal <= RPM_MIN_VAL && error < 0))) {
    integral += error * sample_time;
    integral = constrain(integral, -10, 10);
  }

  // Derivativo con filtrado N
  float derivative = (N * (error - prev_error) + prev_derivative * (1 - N * sample_time)) / (1 + N * sample_time);

  // Control PID
  control_signal = kp * error + ki * integral + kd * derivative;
  control_signal = constrain(control_signal, RPM_MIN_VAL, RPM_MAX_VAL);

  msg.data = control_signal;
  RCSOFTCHECK(rcl_publish(&rpm_publisher, &msg, NULL));

  float pwm_output = map(control_signal, RPM_MIN_VAL, RPM_MAX_VAL, PWM_MIN_VAL, PWM_MAX_VAL);
  pwm_output = constrain(pwm_output, PWM_MIN_VAL, PWM_MAX_VAL);

  if (pwm_output > 0) {
    derecha();
  } else if (pwm_output < 0) {
    izquierda();
  } else {
    detener_motor();
  }

  ledcWrite(PWM_CHNL, abs(pwm_output));

  // Actualizar valores anteriores
  prev_error = error;
  prev_derivative = derivative;

  msg.data = pwm_output;
  RCSOFTCHECK(rcl_publish(&pwm_publisher, &msg, NULL));
}


void setup() {
  set_microros_transports();

  pinMode(LED_PIN, OUTPUT);
  pinMode(PWM_PIN, OUTPUT);
  pinMode(In1, OUTPUT);
  pinMode(In2, OUTPUT);

  digitalWrite(LED_PIN, HIGH);

  ledcSetup(PWM_CHNL, PWM_FRQ, PWM_RES);
  ledcAttachPin(PWM_PIN, PWM_CHNL);

  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  RCCHECK(rclc_node_init_default(&node, "control_deepsilk", "", &support));

  RCCHECK(rclc_subscription_init_default(
    &setpoint_sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "set_point_deepsilk"));

  RCCHECK(rclc_publisher_init_default(
    &rpm_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "motor_output_deepsilk"));

  const unsigned int timer_timeout = 0.0002439; 
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    control_loop_callback));

  RCCHECK(rclc_publisher_init_default(
    &pwm_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "pwm_output"));

  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &setpoint_sub, &msg, &set_point_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
}

void loop() {
  static uint32_t last_time = 0;
  if (millis() - last_time >= 1) {
    last_time = millis();
    RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
  }
}
