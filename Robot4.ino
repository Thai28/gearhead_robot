// Nhập các thư viện
#include <PS2X_lib.h> // Thư viện PS2X cho tay cầm PS2
#include "Adafruit_TCS34725.h" // Thư viện cảm biến màu sắc TCS 34725 của Adafruit
#include <stdio.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
// Classes
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);
PS2X ps2x;
// PS2
#define PS2_DAT 12
#define PS2_CMD 13
#define PS2_SEL 15
#define PS2_CLK 14
// DC Motors
# define SHOOTER_MOTOR_PIN_A 8
# define SHOOTER_MOTOR_PIN_B 9
# define LEFT_WHEEL_MOTOR_PIN_A 10
# define LEFT_WHEEL_MOTOR_PIN_B 11
# define RIGHT_WHEEL_MOTOR_PIN_A 12
# define RIGHT_WHEEL_MOTOR_PIN_B 13
# define INTAKE_MOTOR_PIN_A 14
# define INTAKE_MOTOR_PIN_B 15
// Servo
# define SORTER_SERVO_PIN 4
# define WHITE_BALL_SERVO_PIN 5
# define BLACK_BALL_SERVO_PIN_A 6
# define BLACK_BALL_SERVO_PIN_B 7
// Speed
# define TOP_SPEED 4095
# define NORMAL_SPEED 2048
# define STOP_SPEED 0
// State variables
bool halfAutomaticMode = false; 
bool ballLaunchMotorState = false; 
bool ballLaunchGateState = false;
bool ballDropGateState = false;
void setup()
{
  initilizeRobot();
}
void loop()
{
  robotFunction();
  delay(50); // Đặt độ trễ để tránh bị quá tải do vòng lặp
}
// Initilization
void initilizeRobot()
{
  Serial.begin(115200);
  initilizeMotors();
  setupPS2controller();
  initilizeColorSensor();
  Serial.println("Hoan thanh thiet lap");
}
void initilizeMotors()
{
  Wire.begin(); // Bắt đầu giao tiếp I2C
  pwm.begin(); // Khởi chạy drive của PWM
  pwm.setOscillatorFrequency(27000000); // Thiết lập tần số giao động
  pwm.setPWMFreq(1600); // Thiết lập tần số giao động là 1600Hz
  Wire.setClock(400000); 
  for (i = 8; i < 15; i++) // Đặt giá trị PWM ban đầu của các DC Motors là 0
  {
    pwm.setPWM(i, 0, 0);
  }
  pwm.setPWMFreq(50); // Đặt tần số PWM cho motor
}
void setupPS2controller()
{
  int err = -1;
  while (err != 0)
  {
    err = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, true, true);
  }
}
void initilizeColorSensor()
{
  if (tcs.begin())
  {
    Serial.println("Da tim thay cam bien");
  }
  else 
  {
    Serial.println("Khong tim thay cam bien");
    while(1); // Dừng tìm nếu không tìm thấy cảm biến
  }
}
// Function
void robotFunction()
{
  ps2x.read_gamepad(0, 0); // Đọc trạng thái của tay cầm PS2
  manualControl(); // Xử lý các tín hiệu từ driver đến robot
  automaticControl(); // Xử lý các chức năng tự động của robot
}
void manualControl()
{
  triangleButtonHandler();
  squareButtonHandler();
  crossButtonHandler();
  circleButtonHandler();
  movementHandler();
}
void automaticControl()
{
  ballIntakeRunner();
  ballSortingHandler();
}
void triangleButtonHandler()
{
  if (ps2x.Button(PSB_GREEN)) // Nút xanh lá (nút tam giác) mở / đóng cổng bắn bóng
  {
    ballLaunchGateState = !ballLaunchGateState;
    pwm.setPWM(5, 0, ballLaunchGateState ? 410 : 0); // Mở cổng nếu đang đóng và đóng cổng nếu đang mở
  }
}
void squareButtonHandler()
{
  if (ps2x.Button(PSB_PINK)) // Nút hồng (nút vuông) mở / đóng cổng thả bóng
  {
    ballDropGateState = !ballDropGateState;
    int pwmValue = ballDropGateState ? 410 : 0;
    pwm.setPWM(BLACK_BALL_SERVO_PIN_A, 0, pwmValue); // Đóng mở servo 1
    pwm.setPWM(BLACK_BALL_SERVO_PIN_B, 0, pwmValue); // Đóng mở servo 2
  }
}
void crossButtonHandler()
{
  if (ps2x.Button(PSB_BLUE)) // Nút xanh dương (nút X) bật / tắt máy bắn bóng
  {
    ballLaunchMotorState = !ballLaunchMotorState
    int motorSpeed = ballLaunchMotorState ? TOP_SPEED : 0;
    pwm.setPWM(SHOOTER_MOTOR_PIN_A, 0, motorSpeed); // Điểu khiển motor
    pwm.setPWM(SHOOTER_MOTOR_PIN_B, 0, 0);
  }
}
void circleButtonHandler()
{
  if (ps2x.Button(PSB_RED)) // Nút đỏ (nút tròn) bật / tắt chế độ bán tự động
  {
    halfAutomaticMode = !halfAutomaticMode;
    if (halfAutomaticMode)
    {
      pwm.setPWM(SORTER_SERVO_PIN, 0, 0) // Đóng cổng
    }
  }
}
void movementHandler()
{
  if (ps2x.Button(PSB_L3) || ps2x.Button(PSB_R3))
  {
    int leftJoystick = ps2x.Analog(PSS_LY); // Đọc giá trị trục y của joystick trái
    int rightJoystick = ps2x.Analog(PSS_RY); // Đọc giá trị trục y của joystick phải
    if (leftJoystick < 480)
    {
      // Chạy động cơ hướng về phía sau
      int speed = map(leftJoystick, 480, 0, 0, TOP_SPEED);
      pwm.setPWM(LEFT_WHEEL_MOTOR_PIN_A, 0, speed);
      pwm.setPWM(RIGHT_WHEEL_MOTOR_PIN_B, 0, 0);
    } 
    else if (leftJoystick > 520)
    {
      // Chạy động cơ hướng về phía trước
      int speed = map(leftJoystick, 520, 1023, 0, TOP_SPEED);
      pwm.setPWM(LEFT_WHEEL_MOTOR_PIN_A, 0, 0);
      pwm.setPWM(LEFT_WHEEL_MOTOR_PIN_B, 0, speed);
    } 
    else 
    {
      // Dừng động cơ
      pwm.setPWM(LEFT_WHEEL_MOTOR_PIN_A, 0, 0);
      pwm.setPWM(LEFT_WHEEL_MOTOR_PIN_B, 0, 0);
    }
    if (rightJoystick < 480)
    {
      // Chạy động cơ hướng về phía sau
      int speed = map(rightJoystick, 480, 0, 0, TOP_SPEED);
      pwm.setPWM(RIGHT_WHEEL_MOTOR_PIN_A, 0, speed);
      pwm.setPWM(RIGHT_WHEEL_MOTOR_PIN_B, 0, 0);
    } 
    else if (rightJoystick > 520)
    {
      // Chạy động cơ hướng về phía trước
      int speed = map(leftJoystick, 520, 1023, 0, TOP_SPEED);
      pwm.setPWM(RIGHT_WHEEL_MOTOR_PIN_A, 0, 0);
      pwm.setPWM(RIGHT_WHEEL_MOTOR_PIN_B, 0, speed);
    } 
    else 
    {
      // Dừng động cơ
      pwm.setPWM(RIGHT_WHEEL_MOTOR_PIN_A, 0, 0);
      pwm.setPWM(RIGHT_WHEEL_MOTOR_PIN_B, 0, 0);
    }
  }
}
void ballIntakeRunner()
{
  pwm.setPWM(INTAKE_MOTOR_PIN_A, 0, TOP_SPEED);
  pwm.setPWM(INTAKE_MOTOR_PIN_B, 0, 0);
}
void ballSortingHandler()
{
if (halfAutomaticMode) return; // Không phân loại bóng nếu chế độ phân loại tự động đang bật
  uint16_t clear, red, green, blue;
  tcs.getRawData(&red, &green, &blue, &clear); // Nhận dữ liệu thô từ cảm biến màu sắc
  
  if (clear < 500 && red < 500 && green < 500 && blue < 500) // Bóng đen
  {
    pwm.setPWM(SORTER_SERVO_PIN, 0, 0); // Đóng servo
  }
  else if (clear > 1000 && red > 1000 && green > 1000 && blue > 1000) // Bóng trắng
  {
    pwm.setPWM(SORTER_SERVO_PIN, 0, 205); // Mở servo tại góc 90 độ
  }
  else 
  {
    pwm.setPWM(SORTER_SERVO_PIN, 0, 0); // Đóng servo
  }
}