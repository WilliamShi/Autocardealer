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
int lastObstacleState = HIGH;  // 初始化为高电平
int obstacleState = HIGH;
bool obstacleActive = false;    // 当前是否有障碍物
bool obstacleTriggered = false; // 是否已经触发过一次

// 调试计数器
unsigned long bMotorStartCount = 0;
unsigned long obstacleDetectCount = 0;

// 常量
const unsigned long MOTOR_A_TIMEOUT = 400;
const unsigned long MOTOR_B_TIMEOUT = 7000;
const unsigned long OBSTACLE_DEBOUNCE = 50;    // 防抖时间
const unsigned long OBSTACLE_COOLDOWN = 500;   // 冷却时间
const unsigned long DEBUG_INTERVAL = 1000;
const unsigned long OBSTACLE_RESET_TIME = 1000; // 障碍物消失后重置时间

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
  
  // 重要：红外避障模块引脚设置为输入模式（不使用上拉）
  pinMode(OBSTACLE_PIN, INPUT);  // 模块自身应该输出高电平
  
  pinMode(OBSTACLE_LED, OUTPUT);
  
  // 立即停止所有电机
  stopAllMotors();
  digitalWrite(OBSTACLE_LED, LOW);
  
  // 初始化LCD
  lcd.begin(16, 2);
  lcd.clear();
  lcd.print("Card Dealer v4.5");
  lcd.setCursor(0, 1);
  lcd.print("IR Module Test");
  
  // 初始化红外接收
  IrReceiver.begin(RECV_PIN, ENABLE_LED_FEEDBACK);
  
  // 计算总牌数
  calculateTotalCards();
  
  // 初始读取避障状态并等待稳定
  delay(100);  // 等待模块稳定
  obstacleState = digitalRead(OBSTACLE_PIN);
  lastObstacleState = obstacleState;
  
  #if DEBUG
  Serial.begin(9600);
  Serial.println("=== System Initialized ===");
  Serial.print("Obstacle Pin Mode: INPUT (no pullup)");
  Serial.print("Initial Obstacle State: ");
  Serial.println(obstacleState == HIGH ? "HIGH" : "LOW");
  
  // 测试读取几次
  Serial.println("Testing obstacle readings:");
  for (int i = 0; i < 5; i++) {
    int testRead = digitalRead(OBSTACLE_PIN);
    Serial.print("Read ");
    Serial.print(i);
    Serial.print(": ");
    Serial.println(testRead == HIGH ? "HIGH" : "LOW");
    delay(100);
  }
  #endif
  
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
        
        #if DEBUG
        Serial.print("B Motor Started (after A timeout): ");
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
  int newState = digitalRead(OBSTACLE_PIN);
  
  // 更新LED显示当前状态（反向显示，低电平时LED亮）
  digitalWrite(OBSTACLE_LED, newState == LOW ? HIGH : LOW);
  
  // 只有在B电机运行时才检测避障
  if (currentState != STATE_B_RUNNING) {
    lastObstacleState = newState;
    obstacleState = newState;
    obstacleActive = false;
    obstacleTriggered = false;
    return;
  }
  
  // 状态变化检测
  if (newState != lastObstacleState) {
    // 状态发生变化，重置防抖计时器
    obstacleDebounce = millis();
    
    #if DEBUG
    Serial.print("Obstacle state changed from ");
    Serial.print(lastObstacleState == HIGH ? "HIGH" : "LOW");
    Serial.print(" to ");
    Serial.print(newState == HIGH ? "HIGH" : "LOW");
    Serial.print(" at ");
    Serial.println(millis());
    #endif
  }
  
  // 防抖处理：状态稳定一段时间后才处理
  if (millis() - obstacleDebounce > OBSTACLE_DEBOUNCE) {
    // 状态已经稳定
    if (newState != obstacleState) {
      // 更新当前状态
      obstacleState = newState;
      
      #if DEBUG
      Serial.print("Obstacle state stabilized to: ");
      Serial.println(obstacleState == HIGH ? "HIGH" : "LOW");
      #endif
    }
    
    // 根据稳定后的状态处理
    if (obstacleState == LOW) {
      // 检测到低电平（有物体）
      if (!obstacleActive) {
        obstacleActive = true;
        #if DEBUG
        Serial.println("Obstacle detected (LOW)");
        #endif
      }
      
      // 如果之前没有触发过，且冷却时间已过，则触发动作
      if (!obstacleTriggered && (millis() - lastObstacleTime > OBSTACLE_COOLDOWN)) {
        // 触发避障事件
        handleObstacleEvent();
      }
    } else {
      // 高电平（无障碍物）
      if (obstacleActive) {
        obstacleActive = false;
        obstacleTriggered = false; // 重置触发标志，允许下次检测
        
        #if DEBUG
        Serial.println("Obstacle cleared (HIGH)");
        #endif
      }
    }
  }
  
  // 更新上次状态
  lastObstacleState = newState;
}

// 处理避障事件
void handleObstacleEvent() {
  // 记录触发时间
  lastObstacleTime = millis();
  obstacleTriggered = true;
  obstacleDetectCount++;
  
  // 检测到物体通过
  #if DEBUG
  Serial.print("=== Obstacle Event #");
  Serial.print(obstacleDetectCount);
  Serial.print(" at ");
  Serial.println(millis());
  #endif
  
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
  
  // 短暂闪烁LED
  for (int i = 0; i < 3; i++) {
    digitalWrite(OBSTACLE_LED, HIGH);
    delay(50);
    digitalWrite(OBSTACLE_LED, LOW);
    delay(50);
  }
  
  showStatusMessage("Card Detected");
  
  #if DEBUG
  Serial.print("Card detected, switching to A motor. Cards dealt: ");
  Serial.println(dealtCards);
  #endif
  
  // 5. 检查是否所有牌已发完
  if (dealtCards >= totalCards && totalCards > 0) {
    stopDealing();
    showStatusMessage("All Done!");
    delay(1000);
    updateDisplay();
  }
}

// ==================== 状态切换函数 ====================
void changeState(SystemState newState) {
  #if DEBUG
  const char* stateNames[] = {"IDLE", "B_RUNNING", "A_RUNNING", "B_TIMEOUT"};
  Serial.print("State Change: ");
  Serial.print(stateNames[currentState]);
  Serial.print(" -> ");
  Serial.println(stateNames[newState]);
  #endif
  
  // 重置避障相关状态
  if (newState != STATE_B_RUNNING) {
    obstacleTriggered = false;
    obstacleActive = false;
  }
  
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
  static uint8_t lastDirection = 0;
  if (lastDirection != direction) {
    Serial.print("Motor A: ");
    Serial.println(direction == 1 ? "Clockwise" : "Counter-Clockwise");
    lastDirection = direction;
  }
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
  static unsigned long lastStopPrint = 0;
  if (millis() - lastStopPrint > 1000) {
    Serial.println("All Motors Stopped");
    lastStopPrint = millis();
  }
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
  obstacleActive = false;
  obstacleTriggered = false;
  
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
  Serial.println("\n=== Start Dealing ===");
  Serial.print("Total Cards: ");
  Serial.println(totalCards);
  Serial.print("Players: ");
  Serial.println(playerCount);
  Serial.print("B motor started at: ");
  Serial.println(millis());
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
  Serial.println("=== Debug Info ===");
  Serial.print("State: ");
  switch (currentState) {
    case STATE_IDLE: Serial.println("IDLE"); break;
    case STATE_B_RUNNING: Serial.println("B_RUNNING"); break;
    case STATE_A_RUNNING: Serial.println("A_RUNNING"); break;
    case STATE_B_TIMEOUT: Serial.println("B_TIMEOUT"); break;
    default: Serial.println("UNKNOWN");
  }
  
  Serial.print("Obstacle State: ");
  Serial.print(obstacleState == HIGH ? "HIGH" : "LOW");
  Serial.print(" (Active: ");
  Serial.print(obstacleActive ? "Yes" : "No");
  Serial.print(", Triggered: ");
  Serial.print(obstacleTriggered ? "Yes" : "No");
  Serial.println(")");
  
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
    
    if (currentState == STATE_B_RUNNING) {
      Serial.print("Time until B timeout: ");
      Serial.print(MOTOR_B_TIMEOUT - elapsed);
      Serial.println(" ms");
    }
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
      lcd.print("Card Dealer v4.5");
      lcd.setCursor(0, 1);
      lcd.print("IR Module Test");
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
        
        // 连续读取多次以获得准确状态
        int highCount = 0;
        int lowCount = 0;
        
        for (int i = 0; i < 20; i++) {
          int testState = digitalRead(OBSTACLE_PIN);
          if (testState == HIGH) {
            highCount++;
          } else {
            lowCount++;
          }
          delay(10);
        }
        
        if (highCount > lowCount) {
          lcd.print("HIGH - Clear");
        } else {
          lcd.print("LOW - Blocked");
        }
        
        #if DEBUG
        Serial.print("Obstacle test - High: ");
        Serial.print(highCount);
        Serial.print(", Low: ");
        Serial.println(lowCount);
        #endif
        
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