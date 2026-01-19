#include <IRremote.hpp>
#include <LiquidCrystal.h>
#include <Wire.h>

// ==================== 调试控制 ====================
#define DEBUG 1  // 启用调试信息

// ==================== 引脚定义 ====================
#define RECV_PIN 10
#define OBSTACLE_PIN 7
#define OBSTACLE_LED 13

// L9110电机驱动引脚
#define MOTOR_A_IA 8      // 电机A控制A (平台旋转)
#define MOTOR_A_IB 9      // 电机A控制B (平台旋转)
#define MOTOR_B_IA A3     // 电机B控制A (发牌轮)
#define MOTOR_B_IB A2     // 电机B控制B (发牌轮)

// ==================== 模块初始化 ====================
LiquidCrystal lcd(12, 11, 5, 4, 3, 2);  // LCD引脚

// ==================== 系统状态 ====================
enum SystemState {
  STATE_IDLE,           // 空闲状态
  STATE_B_RUNNING,      // B电机正在运行，等待避障
  STATE_B_WAITING,      // B电机等待避障（已启动但未检测到）
  STATE_A_RUNNING,      // A电机正在运行
  STATE_B_TIMEOUT       // B电机超时
};

// ==================== 全局变量 ====================
// 游戏设置
uint8_t playerCount = 4;
uint8_t deckCount = 3;
uint8_t remainCards = 0;
uint8_t hasJokers = 1;

// 牌数计算
uint16_t totalCards = 0;
uint16_t dealtCards = 0;

// 系统状态
SystemState currentState = STATE_IDLE;
uint8_t isRunning = 0;

// 时间控制
unsigned long motorStartTime = 0;
unsigned long lastObstacleTime = 0;
unsigned long obstacleDebounce = 0;
unsigned long lastDebugTime = 0;

// 避障状态
int lastObstacleState = LOW;
int obstacleState = LOW;

// 调试计数器
unsigned long bMotorStartCount = 0;
unsigned long obstacleDetectCount = 0;

// 常量
const unsigned long MOTOR_A_TIMEOUT = 400;
const unsigned long MOTOR_B_TIMEOUT = 7000;
const unsigned long OBSTACLE_DEBOUNCE = 100;
const unsigned long OBSTACLE_COOLDOWN = 1000;
const unsigned long DEBUG_INTERVAL = 1000;

// ==================== 函数声明 ====================
void updateDisplay();
void checkObstacle();
void handleIRCommand(IRData* results);
void controlMotorA(uint8_t direction);
void controlMotorB(uint8_t state);
void stopAllMotors();
void calculateTotalCards();
void startDealing();
void stopDealing();
void updateDealtCards();
void showStatusMessage(const char* message);
void handleMotorState();
void changeState(SystemState newState);
void debugInfo();

// ==================== SETUP函数 ====================
void setup() {
  // 初始化引脚
  pinMode(MOTOR_A_IA, OUTPUT);
  pinMode(MOTOR_A_IB, OUTPUT);
  pinMode(MOTOR_B_IA, OUTPUT);
  pinMode(MOTOR_B_IB, OUTPUT);
  pinMode(OBSTACLE_PIN, INPUT);
  pinMode(OBSTACLE_LED, OUTPUT);
  
  // 立即停止所有电机
  stopAllMotors();
  digitalWrite(OBSTACLE_LED, LOW);
  
  // 初始化LCD
  lcd.begin(16, 2);
  lcd.clear();
  lcd.print("Card Dealer v4.3");
  lcd.setCursor(0, 1);
  lcd.print("B Motor Fix");
  
  // 初始化红外接收
  IrReceiver.begin(RECV_PIN, ENABLE_LED_FEEDBACK);
  
  // 计算总牌数
  calculateTotalCards();
  
  // 初始显示避障状态
  obstacleState = digitalRead(OBSTACLE_PIN);
  lastObstacleState = obstacleState;
  
  delay(1500);
  updateDisplay();
}

// ==================== LOOP函数 ====================
void loop() {
  // 检查红外遥控信号
  if (IrReceiver.decode()) {
    IRData* results = IrReceiver.read();
    handleIRCommand(results);
    IrReceiver.resume();
  }
  
  // 检查避障模块
  checkObstacle();
  
  // 处理电机状态
  if (isRunning) {
    handleMotorState();
  }
  
  // 调试信息
  #if DEBUG
  if (millis() - lastDebugTime > DEBUG_INTERVAL) {
    debugInfo();
    lastDebugTime = millis();
  }
  #endif
  
  delay(10);
}

// ==================== 状态处理函数 ====================
void handleMotorState() {
  switch (currentState) {
    case STATE_B_RUNNING:
      // B电机正在运行
      // 检查B电机是否超时
      if (millis() - motorStartTime > MOTOR_B_TIMEOUT) {
        changeState(STATE_B_TIMEOUT);
        showStatusMessage("B Timeout!");
        delay(1000);
        stopDealing();
      }
      break;
      
    case STATE_A_RUNNING:
      // A电机正在运行
      // 检查A电机是否超时
      if (millis() - motorStartTime > MOTOR_A_TIMEOUT) {
        // A电机超时，切换到B电机
        stopAllMotors();
        changeState(STATE_B_RUNNING);
        motorStartTime = millis();
        bMotorStartCount++;
        
        // 启动B电机
        controlMotorB(1);
        showStatusMessage("B Running");
        
        #if DEBUG
        Serial.print("B Motor Started: ");
        Serial.println(bMotorStartCount);
        #endif
      }
      break;
      
    case STATE_B_TIMEOUT:
    case STATE_IDLE:
    default:
      // 其他状态不处理
      break;
  }
}

// ==================== 避障检测函数 ====================
void checkObstacle() {
  // 读取避障传感器状态
  obstacleState = digitalRead(OBSTACLE_PIN);
  
  // 只有在B电机运行时才检测避障
  if (currentState != STATE_B_RUNNING && currentState != STATE_B_WAITING) {
    lastObstacleState = obstacleState;
    return;
  }
  
  // 检测状态变化（上升沿检测）
  if (obstacleState == HIGH && lastObstacleState == LOW) {
    // 防抖处理
    if (millis() - obstacleDebounce > OBSTACLE_DEBOUNCE) {
      obstacleDebounce = millis();
      
      // 冷却时间检查
      if (millis() - lastObstacleTime > OBSTACLE_COOLDOWN) {
        lastObstacleTime = millis();
        obstacleDetectCount++;
        
        // 只有在B电机运行时才处理避障
        if (currentState == STATE_B_RUNNING) {
          // 检测到物体通过
          
          // 1. 停止B电机
          stopAllMotors();
          
          // 2. 更新已发牌数
          dealtCards++;
          if (dealtCards > totalCards) dealtCards = totalCards;
          updateDealtCards();
          
          // 3. 切换到A电机
          changeState(STATE_A_RUNNING);
          motorStartTime = millis();
          
          // 4. 启动A电机
          controlMotorA(1);  // 顺时针转动
          
          // 5. LED指示
          digitalWrite(OBSTACLE_LED, HIGH);
          delay(50);
          digitalWrite(OBSTACLE_LED, LOW);
          
          showStatusMessage("Card Detected");
          
          #if DEBUG
          Serial.print("Obstacle Detected: ");
          Serial.println(obstacleDetectCount);
          #endif
          
          // 6. 检查是否所有牌已发完
          if (dealtCards >= totalCards && totalCards > 0) {
            stopDealing();
            showStatusMessage("All Done!");
            delay(1000);
            updateDisplay();
          }
        }
      }
    }
  }
  
  // 更新上次状态
  lastObstacleState = obstacleState;
}

// ==================== 状态切换函数 ====================
void changeState(SystemState newState) {
  currentState = newState;
}

// ==================== 电机控制函数 ====================
void controlMotorA(uint8_t direction) {
  if (direction == 1) {
    // 顺时针转动
    digitalWrite(MOTOR_A_IA, HIGH);
    digitalWrite(MOTOR_A_IB, LOW);
  } else {
    // 逆时针转动
    digitalWrite(MOTOR_A_IA, LOW);
    digitalWrite(MOTOR_A_IB, HIGH);
  }
  
  #if DEBUG
  Serial.print("Motor A: ");
  Serial.println(direction == 1 ? "Forward" : "Reverse");
  #endif
}

void controlMotorB(uint8_t state) {
  if (state) {
    // 启动B电机
    digitalWrite(MOTOR_B_IA, HIGH);
    digitalWrite(MOTOR_B_IB, LOW);
    #if DEBUG
    Serial.println("Motor B: ON");
    #endif
  } else {
    // 停止B电机
    digitalWrite(MOTOR_B_IA, LOW);
    digitalWrite(MOTOR_B_IB, LOW);
    #if DEBUG
    Serial.println("Motor B: OFF");
    #endif
  }
}

void stopAllMotors() {
  // 停止A电机
  digitalWrite(MOTOR_A_IA, LOW);
  digitalWrite(MOTOR_A_IB, LOW);
  
  // 停止B电机
  digitalWrite(MOTOR_B_IA, LOW);
  digitalWrite(MOTOR_B_IB, LOW);
  
  #if DEBUG
  Serial.println("All Motors Stopped");
  #endif
}

// ==================== 游戏控制函数 ====================
void startDealing() {
  if (totalCards <= 0) {
    showStatusMessage("No Cards!");
    delay(1000);
    updateDisplay();
    return;
  }
  
  // 重置状态
  isRunning = 1;
  dealtCards = 0;
  bMotorStartCount = 0;
  obstacleDetectCount = 0;
  
  // 确保所有电机停止
  stopAllMotors();
  
  // 开始从B电机运行
  changeState(STATE_B_RUNNING);
  motorStartTime = millis();
  
  // 启动B电机
  controlMotorB(1);
  bMotorStartCount++;
  
  // 显示开始信息
  showStatusMessage("Start Dealing");
  
  #if DEBUG
  Serial.println("=== Start Dealing ===");
  Serial.print("Total Cards: ");
  Serial.println(totalCards);
  #endif
  
  delay(500);
}

void stopDealing() {
  isRunning = 0;
  changeState(STATE_IDLE);
  stopAllMotors();
  
  // 更新显示
  updateDisplay();
  
  #if DEBUG
  Serial.println("=== Stop Dealing ===");
  Serial.print("B Motor Start Count: ");
  Serial.println(bMotorStartCount);
  Serial.print("Obstacle Detect Count: ");
  Serial.println(obstacleDetectCount);
  Serial.print("Cards Dealt: ");
  Serial.println(dealtCards);
  #endif
}

void calculateTotalCards() {
  uint8_t cardsPerDeck = hasJokers ? 54 : 52;
  totalCards = deckCount * cardsPerDeck;
  
  if (remainCards > 0 && totalCards > remainCards) {
    totalCards -= remainCards;
  }
  
  if (!isRunning) {
    dealtCards = 0;
  }
}

// ==================== 显示函数 ====================
void updateDisplay() {
  lcd.clear();
  
  // 第一行: 设置信息
  lcd.setCursor(0, 0);
  lcd.print("P");
  lcd.print(playerCount);
  lcd.print(" D");
  lcd.print(deckCount);
  lcd.print(" ");
  lcd.print(hasJokers ? "Y" : "N");
  lcd.print(" ");
  
  // 显示已发牌数
  updateDealtCards();
  
  // 第二行: 牌数信息和状态
  lcd.setCursor(0, 1);
  lcd.print("T:");
  lcd.print(totalCards);
  lcd.print(" D:");
  lcd.print(dealtCards);
  lcd.print(" ");
  
  // 显示运行状态
  if (isRunning) {
    switch (currentState) {
      case STATE_B_RUNNING:
        lcd.print("B-RUN");
        break;
      case STATE_A_RUNNING:
        lcd.print("A-RUN");
        break;
      case STATE_B_TIMEOUT:
        lcd.print("B-TO");
        break;
      default:
        lcd.print("RUN");
    }
  } else {
    lcd.print("STOP");
  }
  
  // 显示避障状态
  lcd.setCursor(15, 1);
  lcd.print(obstacleState == HIGH ? "H" : "L");
}

void updateDealtCards() {
  lcd.setCursor(10, 0);
  lcd.print("D:");
  
  // 格式化显示已发牌数
  if (dealtCards < 10) {
    lcd.print("000");
    lcd.print(dealtCards);
  } else if (dealtCards < 100) {
    lcd.print("00");
    lcd.print(dealtCards);
  } else if (dealtCards < 1000) {
    lcd.print("0");
    lcd.print(dealtCards);
  } else {
    lcd.print(dealtCards);
  }
}

void showStatusMessage(const char* message) {
  lcd.clear();
  
  // 显示消息
  lcd.print(message);
  
  // 在第二行显示进度
  lcd.setCursor(0, 1);
  lcd.print("D:");
  lcd.print(dealtCards);
  lcd.print("/");
  lcd.print(totalCards);
  
  // 短暂延迟以确保消息可见
  delay(300);
}

// ==================== 调试函数 ====================
void debugInfo() {
  #if DEBUG
  // 初始化串口
  static bool serialInitialized = false;
  if (!serialInitialized) {
    Serial.begin(9600);
    serialInitialized = true;
  }
  
  Serial.println("=== Debug Info ===");
  Serial.print("State: ");
  switch (currentState) {
    case STATE_IDLE: Serial.println("IDLE"); break;
    case STATE_B_RUNNING: Serial.println("B_RUNNING"); break;
    case STATE_A_RUNNING: Serial.println("A_RUNNING"); break;
    case STATE_B_TIMEOUT: Serial.println("B_TIMEOUT"); break;
    default: Serial.println("UNKNOWN");
  }
  
  Serial.print("Obstacle: ");
  Serial.println(obstacleState == HIGH ? "HIGH" : "LOW");
  
  Serial.print("B Motor Runs: ");
  Serial.println(bMotorStartCount);
  
  Serial.print("Obstacle Detects: ");
  Serial.println(obstacleDetectCount);
  
  Serial.print("Cards Dealt: ");
  Serial.print(dealtCards);
  Serial.print("/");
  Serial.println(totalCards);
  
  if (isRunning) {
    unsigned long elapsed = millis() - motorStartTime;
    Serial.print("Motor Running Time: ");
    Serial.print(elapsed);
    Serial.println(" ms");
  }
  
  Serial.println();
  #endif
}

// ==================== 红外命令处理函数 ====================
void handleIRCommand(IRData* results) {
  // 在LCD第二行显示红外代码
  lcd.setCursor(12, 1);
  if (results->command < 0x10) lcd.print("0");
  lcd.print(results->command, HEX);
  
  // 根据红外命令执行相应操作
  switch(results->command) {
    case 0xDC:  // POWR: 显示欢迎信息
      lcd.clear();
      lcd.print("Card Dealer v4.3");
      lcd.setCursor(0, 1);
      lcd.print("Debug Mode");
      delay(2000);
      updateDisplay();
      break;
      
    case 0x99:  // LEFT: 设置玩家人数
      playerCount++;
      if (playerCount > 8) playerCount = 2;
      calculateTotalCards();
      updateDisplay();
      break;
      
    case 0xC1:  // RIGHT: 设置牌副数
      deckCount++;
      if (deckCount > 3) deckCount = 1;
      calculateTotalCards();
      updateDisplay();
      break;
      
    case 0xCA:  // UP: 是否带王
      hasJokers = !hasJokers;
      calculateTotalCards();
      updateDisplay();
      break;
      
    case 0xD2:  // DOWN: 设置剩余牌数
      remainCards += playerCount;
      if (remainCards > playerCount * 4) remainCards = 0;
      calculateTotalCards();
      updateDisplay();
      break;
      
    case 0xCE:  // OK: 开始发牌
      if (!isRunning) {
        startDealing();
      }
      break;
      
    case 0xC5:  // RETURN: 恢复默认值
      playerCount = 4;
      deckCount = 3;
      hasJokers = 1;
      remainCards = 0;
      stopDealing();
      calculateTotalCards();
      updateDisplay();
      break;
      
    case 0x88:  // HOME: 测试避障模块
      {
        lcd.clear();
        lcd.print("Test Obstacle");
        lcd.setCursor(0, 1);
        
        int testState = digitalRead(OBSTACLE_PIN);
        if (testState == HIGH) {
          lcd.print("HIGH - Blocked");
        } else {
          lcd.print("LOW - Clear");
        }
        delay(2000);
        updateDisplay();
      }
      break;
      
    case 0x82:  // MENU: 停止所有电机
      if (isRunning) {
        stopDealing();
        showStatusMessage("Stopped");
        delay(1000);
        updateDisplay();
      } else {
        stopAllMotors();
        changeState(STATE_IDLE);
        showStatusMessage("Force Stop");
        delay(1000);
        updateDisplay();
      }
      break;
      
    case 0x81:  // VMINUS: A顺时针转测试
      if (!isRunning) {
        controlMotorA(1);
        showStatusMessage("A+ Test");
        delay(1000);
        stopAllMotors();
        updateDisplay();
      }
      break;
      
    case 0x80:  // VPLUS: A逆时针转测试
      if (!isRunning) {
        controlMotorA(0);
        showStatusMessage("A- Test");
        delay(1000);
        stopAllMotors();
        updateDisplay();
      }
      break;
      
    case 0x98:  // B: B电机测试
      if (!isRunning) {
        controlMotorB(1);
        showStatusMessage("B Test");
        delay(2000);
        stopAllMotors();
        updateDisplay();
      }
      break;
  }
}