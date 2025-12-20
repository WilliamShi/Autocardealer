/*
 * 智能发牌机控制系统
 * 基于Arduino Pro Mini
 * 控制三个HG7881电机，HMC5883L磁力计，红外遥控和LCD1602显示屏
 */

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
#include <IRremote.h>
#include <LiquidCrystal_I2C.h>

// ============== 引脚定义 ==============
// 电机控制引脚
#define MOTOR_LEFT_A 2    // 左轮电机A
#define MOTOR_LEFT_B 3    // 左轮电机B
#define MOTOR_RIGHT_A 4   // 右轮电机A
#define MOTOR_RIGHT_B 5   // 右轮电机B
#define MOTOR_DEAL_A 6    // 发牌电机A
#define MOTOR_DEAL_B 7    // 发牌电机B

// 红外传感器引脚
#define IR_SENSOR_PIN 8   // 红外接收头VC1838
#define IR_DETECT_PIN 9   // 红外检测发牌（普通红外发射接收对管）

// 按钮引脚（可选）
#define BUTTON_START 10   // 开始按钮
#define BUTTON_STOP 11    // 停止按钮

// ============== 全局变量 ==============
// 参数设置
int playerCount = 4;      // 玩家人数（默认4人）
int deckCount = 1;        // 牌副数（默认1副）
int remainCards = 0;      // 剩余牌数（默认0张）
int totalCards = 0;       // 总牌数（自动计算）

// 发牌状态
bool isDealing = false;   // 是否正在发牌
int currentPlayer = 0;    // 当前发牌玩家
int cardsDealt = 0;       // 已发牌数
int cardsPerPlayer = 0;   // 每人应发牌数

// 方向控制
float currentHeading = 0;     // 当前方向
float targetHeading = 0;      // 目标方向
float rotationAngle = 90;     // 旋转角度（根据人数计算）

// 红外遥控
IRrecv irrecv(IR_SENSOR_PIN);
decode_results results;

// HMC5883L磁力计
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);

// LCD1602显示屏（I2C地址通常是0x27或0x3F）
LiquidCrystal_I2C lcd(0x27, 16, 2);

// ============== 电机控制函数 ==============
void setupMotors() {
  pinMode(MOTOR_LEFT_A, OUTPUT);
  pinMode(MOTOR_LEFT_B, OUTPUT);
  pinMode(MOTOR_RIGHT_A, OUTPUT);
  pinMode(MOTOR_RIGHT_B, OUTPUT);
  pinMode(MOTOR_DEAL_A, OUTPUT);
  pinMode(MOTOR_DEAL_B, OUTPUT);
  
  stopAllMotors();
}

void stopAllMotors() {
  digitalWrite(MOTOR_LEFT_A, LOW);
  digitalWrite(MOTOR_LEFT_B, LOW);
  digitalWrite(MOTOR_RIGHT_A, LOW);
  digitalWrite(MOTOR_RIGHT_B, LOW);
  digitalWrite(MOTOR_DEAL_A, LOW);
  digitalWrite(MOTOR_DEAL_B, LOW);
}

void moveForward() {
  // 两个轮子相反方向旋转，实现底座旋转
  digitalWrite(MOTOR_LEFT_A, HIGH);
  digitalWrite(MOTOR_LEFT_B, LOW);
  digitalWrite(MOTOR_RIGHT_A, LOW);
  digitalWrite(MOTOR_RIGHT_B, HIGH);
}

void moveBackward() {
  digitalWrite(MOTOR_LEFT_A, LOW);
  digitalWrite(MOTOR_LEFT_B, HIGH);
  digitalWrite(MOTOR_RIGHT_A, HIGH);
  digitalWrite(MOTOR_RIGHT_B, LOW);
}

void stopBase() {
  digitalWrite(MOTOR_LEFT_A, LOW);
  digitalWrite(MOTOR_LEFT_B, LOW);
  digitalWrite(MOTOR_RIGHT_A, LOW);
  digitalWrite(MOTOR_RIGHT_B, LOW);
}

void startDealingMotor() {
  digitalWrite(MOTOR_DEAL_A, HIGH);
  digitalWrite(MOTOR_DEAL_B, LOW);
  delay(100); // 启动延迟
}

void stopDealingMotor() {
  digitalWrite(MOTOR_DEAL_A, LOW);
  digitalWrite(MOTOR_DEAL_B, LOW);
}

// ============== HMC5883L磁力计函数 ==============
void setupCompass() {
  if(!mag.begin()) {
    lcd.setCursor(0, 0);
    lcd.print("Compass Error!");
    while(1);
  }
}

float getHeading() {
  sensors_event_t event; 
  mag.getEvent(&event);
  
  // 计算角度（0-360度）
  float heading = atan2(event.magnetic.y, event.magnetic.x);
  
  // 转换为度数
  heading = heading * 180 / M_PI;
  
  // 校正角度到0-360范围
  if (heading < 0) {
    heading += 360;
  }
  
  return heading;
}

void rotateToAngle(float targetAngle) {
  // 获取当前方向
  currentHeading = getHeading();
  
  // 计算需要旋转的角度（最短路径）
  float angleDiff = targetAngle - currentHeading;
  
  // 处理角度差超过180度的情况
  if (angleDiff > 180) {
    angleDiff -= 360;
  } else if (angleDiff < -180) {
    angleDiff += 360;
  }
  
  lcd.clear();
  lcd.print("Rotating...");
  lcd.setCursor(0, 1);
  lcd.print("Diff: ");
  lcd.print(angleDiff);
  lcd.print(" deg");
  
  // 开始旋转
  if (angleDiff > 0) {
    // 顺时针旋转
    moveForward();
  } else {
    // 逆时针旋转
    moveBackward();
  }
  
  // 监控旋转过程
  float startTime = millis();
  while (abs(angleDiff) > 3 && (millis() - startTime < 5000)) { // 3度容差，5秒超时
    currentHeading = getHeading();
    angleDiff = targetAngle - currentHeading;
    
    // 处理角度差超过180度的情况
    if (angleDiff > 180) {
      angleDiff -= 360;
    } else if (angleDiff < -180) {
      angleDiff += 360;
    }
    
    delay(50);
  }
  
  // 停止旋转
  stopBase();
  
  lcd.clear();
  lcd.print("Rotation Done");
  delay(500);
}

// ============== 红外遥控处理函数 ==============
void setupIR() {
  irrecv.enableIRIn();
}

void handleIRCommand(unsigned long value) {
  // 根据红外遥控器按键值处理命令
  // 注意：这些值需要根据实际遥控器调整
  switch(value) {
    case 0xFF6897: // 按键1 - 增加玩家人数
      playerCount++;
      if (playerCount > 8) playerCount = 2; // 循环2-8人
      calculateParameters();
      updateDisplay();
      break;
      
    case 0xFF9867: // 按键2 - 增加牌副数
      deckCount++;
      if (deckCount > 4) deckCount = 1; // 循环1-4副
      calculateParameters();
      updateDisplay();
      break;
      
    case 0xFFB04F: // 按键3 - 增加剩余牌数
      remainCards += 5;
      if (remainCards > 50) remainCards = 0; // 最大剩余50张
      calculateParameters();
      updateDisplay();
      break;
      
    case 0xFF30CF: // 按键4 - 重置玩家人数
      playerCount = 4;
      calculateParameters();
      updateDisplay();
      break;
      
    case 0xFF18E7: // 按键5 - 重置牌副数
      deckCount = 1;
      calculateParameters();
      updateDisplay();
      break;
      
    case 0xFF7A85: // 按键6 - 重置剩余牌数
      remainCards = 0;
      calculateParameters();
      updateDisplay();
      break;
      
    case 0xFF10EF: // 按键0 - 开始发牌
      if (!isDealing) {
        startDealing();
      }
      break;
      
    case 0xFF38C7: // 按键* - 停止/重置
      stopDealing();
      break;
      
    case 0xFF5AA5: // 按键# - 校准方向
      calibrateDirection();
      break;
  }
}

// ============== 参数计算函数 ==============
void calculateParameters() {
  // 计算总牌数（假设每副牌54张）
  totalCards = deckCount * 54 - remainCards;
  
  // 计算旋转角度
  rotationAngle = 360.0 / playerCount;
  
  // 计算每人应发牌数
  cardsPerPlayer = totalCards / playerCount;
}

// ============== LCD显示函数 ==============
void setupLCD() {
  lcd.init();
  lcd.backlight();
  lcd.clear();
  
  lcd.setCursor(0, 0);
  lcd.print("Smart Card Dealer");
  lcd.setCursor(0, 1);
  lcd.print("Initializing...");
  delay(2000);
}

void updateDisplay() {
  lcd.clear();
  
  // 第一行：显示基本参数
  lcd.setCursor(0, 0);
  lcd.print("P:");
  lcd.print(playerCount);
  lcd.print(" D:");
  lcd.print(deckCount);
  lcd.print(" R:");
  lcd.print(remainCards);
  
  // 第二行：显示计算参数
  lcd.setCursor(0, 1);
  lcd.print("T:");
  lcd.print(totalCards);
  lcd.print(" C:");
  lcd.print(cardsPerPlayer);
  lcd.print("/P");
}

void showStatus(String message) {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(message);
}

// ============== 发牌控制函数 ==============
void startDealing() {
  if (playerCount < 2) {
    showStatus("Min 2 players");
    delay(2000);
    updateDisplay();
    return;
  }
  
  calculateParameters();
  isDealing = true;
  currentPlayer = 0;
  cardsDealt = 0;
  
  showStatus("Starting deal...");
  delay(1000);
  
  // 主发牌循环
  while (cardsDealt < totalCards && isDealing) {
    // 计算当前玩家的目标角度
    targetHeading = (currentPlayer * rotationAngle);
    if (targetHeading >= 360) targetHeading -= 360;
    
    // 旋转到底座到目标位置
    rotateToAngle(targetHeading);
    
    // 显示当前发牌状态
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Player ");
    lcd.print(currentPlayer + 1);
    lcd.print(" of ");
    lcd.print(playerCount);
    lcd.setCursor(0, 1);
    lcd.print("Card ");
    lcd.print(cardsDealt + 1);
    lcd.print(" of ");
    lcd.print(totalCards);
    
    // 发一张牌
    dealOneCard();
    
    // 更新计数
    cardsDealt++;
    currentPlayer++;
    if (currentPlayer >= playerCount) {
      currentPlayer = 0;
    }
    
    // 检查红外停止信号
    if (!isDealing) break;
    
    // 短暂延迟
    delay(500);
  }
  
  if (cardsDealt >= totalCards) {
    showStatus("Dealing complete!");
    delay(3000);
  }
  
  updateDisplay();
  isDealing = false;
}

void dealOneCard() {
  // 启动发牌电机
  startDealingMotor();
  
  // 等待红外传感器检测到牌
  bool cardDetected = false;
  unsigned long startTime = millis();
  
  while (!cardDetected && (millis() - startTime < 5000)) { // 5秒超时
    if (digitalRead(IR_DETECT_PIN) == LOW) { // 假设有牌时输出低电平
      cardDetected = true;
    }
    
    // 检查停止信号
    if (!isDealing) break;
    
    delay(10);
  }
  
  // 停止发牌电机
  stopDealingMotor();
  
  if (!cardDetected) {
    showStatus("Card jam error!");
    delay(2000);
    isDealing = false;
  }
  
  // 短暂延迟，确保牌完全离开
  delay(300);
}

void stopDealing() {
  isDealing = false;
  stopAllMotors();
  showStatus("Stopped");
  delay(1000);
  updateDisplay();
}

void calibrateDirection() {
  showStatus("Calibrating...");
  delay(1000);
  
  // 获取当前方向作为参考
  currentHeading = getHeading();
  
  showStatus("Calibration OK");
  lcd.setCursor(0, 1);
  lcd.print("Ref: ");
  lcd.print(currentHeading);
  lcd.print(" deg");
  
  delay(2000);
  updateDisplay();
}

// ============== 初始化函数 ==============
void setup() {
  // 初始化串口（调试用）
  Serial.begin(9600);
  
  // 初始化各模块
  setupLCD();
  setupMotors();
  setupCompass();
  setupIR();
  
  // 设置红外检测引脚
  pinMode(IR_DETECT_PIN, INPUT_PULLUP);
  
  // 设置按钮引脚（可选）
  pinMode(BUTTON_START, INPUT_PULLUP);
  pinMode(BUTTON_STOP, INPUT_PULLUP);
  
  // 计算初始参数
  calculateParameters();
  
  // 显示初始状态
  updateDisplay();
  
  // 显示欢迎信息
  showStatus("Ready for setup");
  delay(2000);
  updateDisplay();
}

// ============== 主循环 ==============
void loop() {
  // 处理红外遥控信号
  if (irrecv.decode(&results)) {
    Serial.println(results.value, HEX); // 调试：打印红外码值
    handleIRCommand(results.value);
    irrecv.resume(); // 接收下一个信号
  }
  
  // 检查按钮输入（可选）
  if (digitalRead(BUTTON_START) == LOW && !isDealing) {
    delay(50); // 防抖
    if (digitalRead(BUTTON_START) == LOW) {
      startDealing();
    }
  }
  
  if (digitalRead(BUTTON_STOP) == LOW && isDealing) {
    delay(50); // 防抖
    if (digitalRead(BUTTON_STOP) == LOW) {
      stopDealing();
    }
  }
  
  // 如果正在发牌，更新状态显示
  if (isDealing && (millis() % 1000 < 50)) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Dealing: ");
    lcd.print(cardsDealt);
    lcd.print("/");
    lcd.print(totalCards);
    lcd.setCursor(0, 1);
    lcd.print("Player ");
    lcd.print(currentPlayer + 1);
  }
  
  delay(50); // 主循环延迟
}