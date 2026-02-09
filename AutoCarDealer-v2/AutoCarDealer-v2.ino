// ==================== 系统配置宏 ====================
#define TURNAROUND_MODE 0      // 0=超时模式, 1=罗盘模式
#define ENABLE_INFRA 1
#define ENABLE_KEYBOARD 0
#define DEBUG 1
#define IR_DEBUG 1  // 专门控制红外调试
//#define MOTOR_B_TIMEOUT_MS 8000

// ==================== 系统变量 ====================
float initialHeading = 0;
float targetHeading = 0;
float currentHeading = 0;
float angleTolerance = 8.0;
float anglePerPlayer = 90.0;

#include <LiquidCrystal.h>
#include <Wire.h>
#include <QMC5883LCompass.h>

#if ENABLE_INFRA
#include <IRremote.hpp>
#define IR_RECEIVE_PIN A1
#endif

// ==================== TB6612FNG 引脚定义 ====================
#define MOTOR_A_IN1 6
#define MOTOR_A_IN2 7
#define MOTOR_A_PWM 9
#define MOTOR_B_IN1 A3
#define MOTOR_B_IN2 A2
#define MOTOR_B_PWM 10
#define MOTOR_STBY 8
#define OBSTACLE_PIN A0

// ==================== 模块初始化 ====================
LiquidCrystal lcd(12, 11, 5, 4, 3, 2);
QMC5883LCompass compass;

#if ENABLE_INFRA
IRrecv irrecv(IR_RECEIVE_PIN);
decode_results results;
#endif

// ==================== 系统状态 ====================
enum SystemState {
  STATE_IDLE,
  STATE_B_RUNNING,
  STATE_A_RUNNING,
  STATE_B_TIMEOUT
};

// ==================== 全局变量 ====================
uint8_t playerCount = 4;
uint8_t deckCount = 3;
uint8_t remainCards = 0;
uint8_t hasJokers = 1;
uint16_t totalCards = 0;
uint16_t dealtCards = 0;
SystemState currentState = STATE_IDLE;
bool isRunning = 0;

unsigned long motorStartTime = 0;
unsigned long lastObstacleTime = 0;
unsigned long obstacleDebounce = 0;
unsigned long lastDebugTime = 0;
unsigned long aMotorTimeoutStart = 0;
unsigned long motorBTimeout = 8000;
bool compassInitialized = false;
bool calibrationDone = false;
uint8_t lastObstacleState = HIGH;
uint8_t obstacleState = HIGH;
bool obstacleActive = false;
bool obstacleTriggered = false;

// ==================== 关键修正：手动校准的电机参数 ====================
// 通过实验手动调整这些值，直到旋转角度准确
unsigned long MANUAL_CIRCLE_TIME = 3050;  // 手动设置的一圈时间（单位：ms）
unsigned long motorATimeoutPerPlayer = 765;  // MANUAL_CIRCLE_TIME / playerCount

// 运行时模式切换
bool runtimeTurnaroundMode = TURNAROUND_MODE;

// ==================== 罗盘系统 ====================
float lastStableHeading = 0.0;
float filteredHeading = 0.0;
float headingSamples[3];
int sampleIndex = 0;
bool samplesReady = false;
unsigned long lastCompassUpdate = 0;
const int COMPASS_UPDATE_INTERVAL = 30;

float lastValidHeading = 0.0;
unsigned long lastHeadingChangeTime = 0;
bool compassResponding = true;
int noChangeCount = 0;
const int MAX_NO_CHANGE = 15;

float virtualHeading = 0.0;
const float MAGNETIC_DECLINATION = 5.0;

// ==================== 电机控制参数 ====================
unsigned long lastMotorUpdate = 0;
const int MOTOR_CONTROL_INTERVAL = 20;

// ==================== 电机A旋转状态跟踪 ====================
float rotationStartHeading = 0.0;
unsigned long rotationStartTime = 0;
float lastRotationAngle = 0.0;
bool rotationInProgress = false;

// ==================== 串口输入缓冲区 ====================
const int SERIAL_BUFFER_SIZE = 32;
char serialBuffer[SERIAL_BUFFER_SIZE];
int serialBufferIndex = 0;
unsigned long lastSerialCharTime = 0;
const unsigned long SERIAL_TIMEOUT = 100;

// ==================== TB6612FNG 电机速度参数 ====================
const int MOTOR_A_SPEED = 150;     // 固定速度150
const int MOTOR_B_SPEED = 255;
const int MOTOR_A_CAL_SPEED = 200;

// ==================== 红外遥控键码定义 ====================
#if ENABLE_INFRA
// 红外键码定义（NEC格式）
#define IR_DEC_PLAYER 0xBA45FF00      // 减少玩家数
#define IR_RESET 0xB946FF00           // 重置为默认
#define IR_INC_PLAYER 0xB847FF00      // 增加玩家数
#define IR_INC_DECK 0xBB44FF00        // 增加牌组数
#define IR_TOGGLE_JOKER 0xBF40FF00    // 切换鬼牌
#define IR_START 0xBC43FF00           // 开始发牌
#define IR_DEC_CIRCLE_TIME 0xF807FF00 // 减少圈时间100ms
#define IR_INC_CIRCLE_TIME 0xEA15FF00 // 增加圈时间100ms
#define IR_STOP 0xF609FF00            // 停止发牌
#define IR_REMAIN_CARDS 0xE916FF00    // 调整剩余牌数
#define IR_TEST_MOTOR_A 0xE619FF00    // 测试电机A旋转90度
#define IR_TEST_MOTOR_B 0xF20DFF00    // 测试电机B
#define IR_CALIBRATE 0xF30CFF00       // 校准罗盘
#define IR_TOGGLE_MODE 0xE718FF00     // 切换模式（罗盘/超时）
#define IR_SYSTEM_RESET 0xA15EFF00    // 系统重置
#define IR_VERSION 0xF708FF00         // 版本显示
#define IR_BTM 0xE31CFF00             //减小电机b测试超时时长
#define IR_BTP 0xA55AFF00             //增加电机b测试超时时长
#endif

// ==================== 角度处理函数 ====================
float normalizeAngle(float angle) {
  while (angle >= 360.0) angle -= 360.0;
  while (angle < 0.0) angle += 360.0;
  return angle;
}

float getSimpleAngleDiff(float current, float target) {
  current = normalizeAngle(current);
  target = normalizeAngle(target);
  float diff = target - current;
  
  if (diff > 180.0) diff -= 360.0;
  else if (diff < -180.0) diff += 360.0;
  
  return diff;
}

// ==================== TB6612FNG 电机控制函数 ====================
void enableMotorDriver() {
  digitalWrite(MOTOR_STBY, HIGH);
  delay(10);
}

void disableMotorDriver() {
  digitalWrite(MOTOR_STBY, LOW);
}

void stopAllMotors() {
  digitalWrite(MOTOR_A_IN1, HIGH);
  digitalWrite(MOTOR_A_IN2, HIGH);
  analogWrite(MOTOR_A_PWM, 255);
  
  digitalWrite(MOTOR_B_IN1, HIGH);
  digitalWrite(MOTOR_B_IN2, HIGH);
  analogWrite(MOTOR_B_PWM, 255);
  
  delay(20);
  analogWrite(MOTOR_A_PWM, 0);
  analogWrite(MOTOR_B_PWM, 0);
  
  rotationInProgress = false;
}

void controlMotorA(bool enable) {
  if (enable) {
    digitalWrite(MOTOR_A_IN1, LOW);
    digitalWrite(MOTOR_A_IN2, HIGH);
    analogWrite(MOTOR_A_PWM, MOTOR_A_SPEED);
    rotationInProgress = true;
  } else {
    stopAllMotors();
  }
}

void controlMotorAFullSpeed() {
  digitalWrite(MOTOR_A_IN1, LOW);
  digitalWrite(MOTOR_A_IN2, HIGH);
  analogWrite(MOTOR_A_PWM, MOTOR_A_CAL_SPEED);
  rotationInProgress = true;
}

void controlMotorB(uint8_t state) {
  if (state) {
    digitalWrite(MOTOR_B_IN1, LOW);
    digitalWrite(MOTOR_B_IN2, HIGH);
    analogWrite(MOTOR_B_PWM, MOTOR_B_SPEED);
  } else {
    digitalWrite(MOTOR_B_IN1, HIGH);
    digitalWrite(MOTOR_B_IN2, HIGH);
    analogWrite(MOTOR_B_PWM, 255);
    delay(20);
    analogWrite(MOTOR_B_PWM, 0);
  }
}

// ==================== 计算超时时间 ====================
void calculateMotorATimeout() {
  motorATimeoutPerPlayer = MANUAL_CIRCLE_TIME / playerCount;
  
  if (motorATimeoutPerPlayer < 100) {
    motorATimeoutPerPlayer = 100;
  }
  
  #if DEBUG
  Serial.print(F("Circle time: "));
  Serial.print(MANUAL_CIRCLE_TIME);
  Serial.print(F("ms, Timeout: "));
  Serial.print(motorATimeoutPerPlayer);
  Serial.print(F("ms"));
  #endif
}

// ==================== 罗盘读取函数 ====================
bool readCompassHeading(float &heading) {
  if (!compassInitialized) return false;
  
  compass.read();
  int rawHeading = compass.getAzimuth();
  
  if (rawHeading < 0) rawHeading += 360;
  
  if (rawHeading >= 0 && rawHeading <= 360) {
    heading = normalizeAngle(rawHeading + MAGNETIC_DECLINATION);
    return true;
  }
  
  return false;
}

// ==================== 虚拟角度跟踪 ====================
float updateVirtualHeadingDuringRotation() {
  if (!rotationInProgress) return virtualHeading;
  
  unsigned long currentTime = millis();
  unsigned long elapsed = currentTime - rotationStartTime;
  
  float rotationSpeed = 360.0 / (MANUAL_CIRCLE_TIME / 1000.0);
  float expectedRotation = (elapsed / 1000.0) * rotationSpeed;
  virtualHeading = normalizeAngle(rotationStartHeading - expectedRotation);
  
  return virtualHeading;
}

// ==================== 罗盘更新函数 ====================
void updateCompassHeading() {
  if (!compassInitialized) return;
  
  if (millis() - lastCompassUpdate < COMPASS_UPDATE_INTERVAL) return;
  
  float newHeading;
  if (readCompassHeading(newHeading)) {
    float change = fabs(newHeading - lastValidHeading);
    
    if (change > 0.5) {
      compassResponding = true;
      noChangeCount = 0;
      lastHeadingChangeTime = millis();
    } else {
      noChangeCount++;
      if (noChangeCount > MAX_NO_CHANGE) {
        compassResponding = false;
      }
    }
    
    currentHeading = newHeading;
    lastValidHeading = newHeading;
    virtualHeading = newHeading;
    headingSamples[sampleIndex] = newHeading;
    sampleIndex = (sampleIndex + 1) % 3;
    
    if (!samplesReady && sampleIndex == 0) samplesReady = true;
    
    if (samplesReady) {
      float sum = 0;
      for (int i = 0; i < 3; i++) sum += headingSamples[i];
      filteredHeading = normalizeAngle(sum / 3.0);
    } else {
      filteredHeading = newHeading;
    }
  }
  
  lastCompassUpdate = millis();
}

// ==================== 获取当前航向 ====================
float getCurrentHeading() {
  if (compassInitialized && compassResponding) {
    return filteredHeading;
  } else {
    return updateVirtualHeadingDuringRotation();
  }
}

// ==================== QMC5883L初始化 ====================
bool initCompassUltimate() {
  #if DEBUG
  Serial.println(F("Init Compass..."));
  #endif
  
  lcd.clear();
  lcd.print(F("Init Compass..."));
  
  stopAllMotors();
  disableMotorDriver();
  delay(100);
  
  Wire.end();
  delay(50);
  Wire.begin();
  Wire.setClock(100000);
  delay(200);
  
  bool i2cConnected = false;
  for (int attempt = 1; attempt <= 3; attempt++) {
    Wire.beginTransmission(0x0D);
    byte error = Wire.endTransmission();
    if (error == 0) {
      i2cConnected = true;
      break;
    }
    delay(100);
  }
  
  if (!i2cConnected) {
    lcd.clear();
    lcd.print(F("I2C Fail"));
    delay(2000);
    return false;
  }
  
  compass.init();
  delay(100);
  
  bool dataValid = false;
  for (int i = 0; i < 5; i++) {
    compass.read();
    int x = compass.getX();
    int y = compass.getY();
    int z = compass.getZ();
    
    if (x != 0 || y != 0 || z != 0) {
      dataValid = true;
      break;
    }
    delay(50);
  }
  
  compassInitialized = dataValid;
  calibrationDone = false;
  
  if (compassInitialized) {
    float initialReading = 0.0;
    float headingSum = 0;
    int validHeadings = 0;
    
    for (int i = 0; i < 5; i++) {
      compass.read();
      int rawHeading = compass.getAzimuth();
      if (rawHeading >= 0 && rawHeading <= 360) {
        validHeadings++;
        headingSum += rawHeading;
      }
      delay(50);
    }
    
    if (validHeadings > 0) {
      initialReading = normalizeAngle(headingSum / validHeadings + MAGNETIC_DECLINATION);
    }
    
    currentHeading = initialReading;
    targetHeading = initialReading;
    filteredHeading = initialReading;
    virtualHeading = initialReading;
    lastValidHeading = initialReading;
    
    for (int i = 0; i < 3; i++) headingSamples[i] = initialReading;
    samplesReady = true;
    compassResponding = true;
    noChangeCount = 0;
    
    #if DEBUG
    Serial.print(F("Compass OK: "));
    Serial.println(initialReading, 1);
    #endif
    
    lcd.clear();
    lcd.print(F("Compass OK"));
    delay(500);
  }
  
  return compassInitialized;
}

// ==================== 校准函数 ====================
void calibrateCompass() {
  if (!compassInitialized) {
    lcd.clear();
    lcd.print(F("Compass Not Ready"));
    delay(1000);
    return;
  }
  
  lcd.clear();
  lcd.print(F("Calibrating..."));
  
  stopAllMotors();
  delay(200);
  enableMotorDriver();
  
  float startHeading = 0.0;
  for (int i = 0; i < 5; i++) {
    updateCompassHeading();
    delay(100);
  }
  startHeading = getCurrentHeading();
  
  controlMotorAFullSpeed();
  delay(5000);
  
  stopAllMotors();
  delay(500);
  
  float finalHeading = 0.0;
  for (int i = 0; i < 5; i++) {
    updateCompassHeading();
    delay(100);
  }
  finalHeading = getCurrentHeading();
  
  float avgHeading = normalizeAngle((startHeading + finalHeading) / 2.0);
  
  initialHeading = avgHeading;
  currentHeading = avgHeading;
  targetHeading = avgHeading;
  filteredHeading = avgHeading;
  virtualHeading = avgHeading;
  lastValidHeading = avgHeading;
  calibrationDone = true;
  
  for (int i = 0; i < 3; i++) headingSamples[i] = avgHeading;
  
  #if DEBUG
  Serial.print(F("Calibration done: "));
  Serial.println(avgHeading, 1);
  #endif
  
  lcd.clear();
  lcd.print(F("Calibration OK"));
  lcd.setCursor(0, 1);
  lcd.print(F("H:"));
  lcd.print((int)avgHeading);
  lcd.print(F("°"));
  
  delay(1500);
}

// ==================== 核心：旋转控制函数 ====================
bool rotateToAngle() {
  if (millis() - lastMotorUpdate < MOTOR_CONTROL_INTERVAL) return false;
  
  lastMotorUpdate = millis();
  unsigned long elapsed = millis() - aMotorTimeoutStart;
  
  if (runtimeTurnaroundMode == 1 && compassInitialized && calibrationDone) {
    float current = getCurrentHeading();
    float angleDiff = getSimpleAngleDiff(current, targetHeading);
    
    if (fabs(angleDiff) <= angleTolerance) {
      stopAllMotors();
      return true;
    }
    
    if (elapsed >= motorATimeoutPerPlayer) {
      stopAllMotors();
      return true;
    }
    
    controlMotorA(true);
    return false;
  }
  else {
    if (elapsed >= motorATimeoutPerPlayer) {
      stopAllMotors();
      return true;
    }
    
    float expectedRotation = (elapsed / (float)MANUAL_CIRCLE_TIME) * 360.0;
    float targetRotation = fabs(getSimpleAngleDiff(rotationStartHeading, targetHeading));
    
    if (expectedRotation >= targetRotation * 0.95 && elapsed > motorATimeoutPerPlayer * 0.8) {
      stopAllMotors();
      return true;
    }
    
    controlMotorA(true);
    return false;
  }
}

// ==================== 障碍事件处理 ====================
void handleObstacleEvent() {
  lastObstacleTime = millis();
  
  stopAllMotors();
  dealtCards++;
  if (dealtCards > totalCards) dealtCards = totalCards;
  
  #if DEBUG
  Serial.print(F("Card dealt: "));
  Serial.println(dealtCards);
  #endif
  
  uint8_t currentPlayerIndex = dealtCards % playerCount;
  float targetAngle = initialHeading - (currentPlayerIndex * anglePerPlayer);
  targetHeading = normalizeAngle(targetAngle);
  
  rotationStartHeading = getCurrentHeading();
  rotationStartTime = millis();
  virtualHeading = rotationStartHeading;
  
  changeState(STATE_A_RUNNING);
  aMotorTimeoutStart = millis();
  lastMotorUpdate = millis();
  
  controlMotorA(true);
  
  compassResponding = true;
  noChangeCount = 0;
  lastHeadingChangeTime = millis();
  
  showStatusMessage("Rotating...");
  
  if (dealtCards >= totalCards && totalCards > 0) {
    stopDealing();
    showStatusMessage("All Done!");
    delay(1000);
    updateDisplay();
  }
}

// ==================== 状态切换 ====================
void changeState(SystemState newState) {
  #if DEBUG
  const char* stateNames[] = {"IDLE", "B_RUNNING", "A_RUNNING", "B_TIMEOUT"};
  if (currentState != newState) {
    Serial.print(F("State: "));
    Serial.print(stateNames[currentState]);
    Serial.print(F(" -> "));
    Serial.println(stateNames[newState]);
  }
  #endif
  
  if (newState != STATE_B_RUNNING) {
    obstacleTriggered = false;
    obstacleActive = false;
  }
  
  currentState = newState;
}

// ==================== 游戏控制函数 ====================
void startDealing() {
  if (totalCards <= 0) {
    showStatusMessage("No Cards!");
    delay(1000);
    updateDisplay();
    return;
  }
  
  isRunning = 1;
  dealtCards = 0;
  
  stopAllMotors();
  enableMotorDriver();
  
  if (runtimeTurnaroundMode == 1 && compassInitialized && calibrationDone) {
    targetHeading = initialHeading;
    virtualHeading = initialHeading;
  } else {
    initialHeading = 0;
    currentHeading = 0;
    virtualHeading = 0;
    targetHeading = 0;
  }
  
  changeState(STATE_B_RUNNING);
  motorStartTime = millis();
  controlMotorB(1);
  
  showStatusMessage("Start Dealing");
  delay(300);
}

void stopDealing() {
  isRunning = 0;
  changeState(STATE_IDLE);
  stopAllMotors();
  updateDisplay();
}

// ==================== 避障检测 ====================
void checkObstacle() {
  uint8_t newState = digitalRead(OBSTACLE_PIN);
  
  if (currentState != STATE_B_RUNNING) {
    lastObstacleState = newState;
    obstacleState = newState;
    return;
  }
  
  if (newState != lastObstacleState) {
    obstacleDebounce = millis();
  }
  
  if (millis() - obstacleDebounce > 50) {
    if (newState != obstacleState) {
      obstacleState = newState;
    }
    
    if (obstacleState == LOW && (millis() - lastObstacleTime > 500)) {
      handleObstacleEvent();
    }
  }
  
  lastObstacleState = newState;
}

// ==================== 状态处理函数 ====================
void handleMotorState() {
  switch (currentState) {
    case STATE_B_RUNNING:
      if (millis() - motorStartTime > 5000) {
        changeState(STATE_B_TIMEOUT);
        showStatusMessage("B Timeout!");
        delay(500);
        stopDealing();
      }
      break;
      
    case STATE_A_RUNNING:
      if (rotateToAngle()) {
        stopAllMotors();
        
        if (dealtCards >= totalCards && totalCards > 0) {
          stopDealing();
          showStatusMessage("All Done!");
          delay(1000);
          updateDisplay();
        } else {
          changeState(STATE_B_RUNNING);
          motorStartTime = millis();
          controlMotorB(1);
        }
      }
      break;
  }
}

// ==================== 系统重启函数 ====================
void resetSystem() {
  #if DEBUG
  Serial.println(F("System Reset"));
  #endif
  
  stopAllMotors();
  disableMotorDriver();
  
  isRunning = 0;
  dealtCards = 0;
  currentState = STATE_IDLE;
  obstacleTriggered = false;
  obstacleActive = false;
  
  if (compassInitialized) {
    float current = getCurrentHeading();
    currentHeading = current;
    targetHeading = current;
    virtualHeading = current;
    lastValidHeading = current;
    filteredHeading = current;
  }
  
  serialBufferIndex = 0;
  serialBuffer[0] = '\0';
  
  lcd.clear();
  lcd.print(F("System Reset"));
  lcd.setCursor(0, 1);
  lcd.print(F("Mode: "));
  lcd.print(runtimeTurnaroundMode == 1 ? "Compass" : "Timeout");
  
  delay(1000);
  updateDisplay();
}

// ==================== 红外遥控处理 ====================
#if ENABLE_INFRA
void processInfraredInput() {
  if (IrReceiver.decode()) {
    unsigned long irValue = IrReceiver.decodedIRData.decodedRawData;
    
    #if IR_DEBUG
    // 始终打印接收到的红外键码，便于调试
    Serial.print(F("IR Received: 0x"));
    Serial.println(irValue, HEX);
    #endif
    
    // 处理重复码
    if (irValue == 0xFFFFFFFF) {
      irValue = IrReceiver.decodedIRData.decodedRawData;
      #if IR_DEBUG
      Serial.println(F("IR Repeat"));
      #endif
    }
    
    switch(irValue) {
      case IR_DEC_PLAYER:  // 减少玩家数 (FFA25D)
        if (!isRunning) {
          if (playerCount > 2) playerCount--;
          else playerCount = 8;
          anglePerPlayer = 360.0 / playerCount;
          calculateMotorATimeout();
          updateDisplay();
        }
        break;
        
      case IR_RESET:  // 重置为默认 (FF629D)
        if (!isRunning) {
          playerCount = 4;
          deckCount = 3;
          hasJokers = 1;
          remainCards = 0;
          totalCards = deckCount * (hasJokers ? 54 : 52);
          anglePerPlayer = 360.0 / playerCount;
          calculateMotorATimeout();
          updateDisplay();
        }
        break;
        
      case IR_INC_PLAYER:  // 增加玩家数 (FFE21D)
        if (!isRunning) {
          playerCount++;
          if (playerCount > 8) playerCount = 2;
          anglePerPlayer = 360.0 / playerCount;
          calculateMotorATimeout();
          updateDisplay();
        }
        break;
        
      case IR_INC_DECK:  // 增加牌组数 (FF22DD)
        if (!isRunning) {
          deckCount++;
          if (deckCount > 3) deckCount = 1;
          totalCards = deckCount * (hasJokers ? 54 : 52);
          updateDisplay();
        }
        break;
        
      case IR_TOGGLE_JOKER:  // 切换鬼牌 (FF02FD)
        if (!isRunning) {
          hasJokers = !hasJokers;
          totalCards = deckCount * (hasJokers ? 54 : 52);
          updateDisplay();
        }
        break;
        
      case IR_START:  // 开始发牌 (FFC23D)
        if (!isRunning) {
          startDealing();
        } else {
          showStatusMessage("Already Running");
        }
        break;
        
      case IR_DEC_CIRCLE_TIME:  // 减少圈时间100ms (FFE01F)
        if (!isRunning && MANUAL_CIRCLE_TIME > 200) {
          MANUAL_CIRCLE_TIME -= 100;
          calculateMotorATimeout();
          lcd.clear();
          lcd.print(F("Circle Time -100"));
          lcd.setCursor(0, 1);
          lcd.print(F("T="));
          lcd.print(MANUAL_CIRCLE_TIME);
          lcd.print(F("ms"));
          delay(1000);
          updateDisplay();
        }
        break;
        
      case IR_INC_CIRCLE_TIME:  // 增加圈时间100ms (FFA857)
        if (!isRunning) {
          MANUAL_CIRCLE_TIME += 100;
          calculateMotorATimeout();
          lcd.clear();
          lcd.print(F("Circle Time +100"));
          lcd.setCursor(0, 1);
          lcd.print(F("T="));
          lcd.print(MANUAL_CIRCLE_TIME);
          lcd.print(F("ms"));
          delay(1000);
          updateDisplay();
        }
        break;
        
      case IR_STOP:  // 停止发牌 (FF906F)
        if (isRunning) {
          stopDealing();
        } else {
          lcd.clear();
          lcd.print(F("Not Running"));
          delay(1000);
          updateDisplay();
        }
        break;
        
      case IR_REMAIN_CARDS:  // 调整剩余牌数 (FF6897)
        if (!isRunning) {
          remainCards += playerCount;
          if (remainCards > playerCount * 4) remainCards = 0;
          totalCards = deckCount * (hasJokers ? 54 : 52);
          if (remainCards > 0 && totalCards > remainCards) totalCards -= remainCards;
          updateDisplay();
        }
        break;
        
      case IR_TEST_MOTOR_A:  // 测试电机A旋转90度 (FF9867)
        if (!isRunning) {
          enableMotorDriver();
          float startAngle = getCurrentHeading();
          float testTarget = normalizeAngle(startAngle - 90);
          
          lcd.clear();
          lcd.print(F("Test 90 Rotation"));
          lcd.setCursor(0, 1);
          lcd.print(F("Mode: "));
          lcd.print(runtimeTurnaroundMode == 1 ? "Compass" : "Timeout");
          
          rotationStartHeading = startAngle;
          rotationStartTime = millis();
          aMotorTimeoutStart = millis();
          targetHeading = testTarget;
          
          unsigned long expectedTime = (90.0 / 360.0) * MANUAL_CIRCLE_TIME;
          
          #if DEBUG
          Serial.print(F("90 test: "));
          Serial.print(expectedTime);
          Serial.println(F("ms"));
          #endif
          
          controlMotorA(true);
          
          unsigned long testStart = millis();
          bool testComplete = false;
          
          while (!testComplete && millis() - testStart < 10000) {
            if (rotateToAngle()) {
              testComplete = true;
            }
            delay(10);
          }
          
          if (!testComplete) {
            stopAllMotors();
          }
          
          float endAngle = getCurrentHeading();
          float actualRotation = getSimpleAngleDiff(startAngle, endAngle);
          
          lcd.clear();
          lcd.print(F("Test Complete"));
          lcd.setCursor(0, 1);
          lcd.print(F("Rotated: "));
          lcd.print(actualRotation, 0);
          lcd.print(F("°"));
          
          delay(2000);
          updateDisplay();
        }
        break;
        
      case IR_TEST_MOTOR_B:  // 测试电机B (FFB04F)
        if (!isRunning) {
          enableMotorDriver();
          controlMotorB(1);
          lcd.clear();
          lcd.print(F("Motor B Test"));
          delay(1500);
          stopAllMotors();
          updateDisplay();
        }
        break;
        
      case IR_CALIBRATE:  // 校准罗盘 (FF30CF)
        if (!isRunning && compassInitialized) {
          calibrateCompass();
          updateDisplay();
        }
        break;
        
      case IR_TOGGLE_MODE:  // 切换模式（罗盘/超时） (FF18E7)
        runtimeTurnaroundMode = !runtimeTurnaroundMode;
        lcd.clear();
        lcd.print(F("Mode Changed"));
        lcd.setCursor(0, 1);
        lcd.print(F("Now: "));
        lcd.print(runtimeTurnaroundMode == 1 ? "Compass" : "Timeout");
        delay(1000);
        updateDisplay();
        
        #if DEBUG
        Serial.print(F("Mode: "));
        Serial.println(runtimeTurnaroundMode == 1 ? "Compass" : "Timeout");
        #endif
        break;
        
      case IR_SYSTEM_RESET:  // 系统重置 (FF7A85)
        resetSystem();
        break;
        
      case IR_VERSION:  // 版本显示 (FF10EF)
        lcd.clear();
        lcd.print(F("Card Dealer v28.0"));
        lcd.setCursor(0, 1);
        lcd.print(F("Manual Calibration"));
        delay(1000);
        updateDisplay();
        break;
      case IR_BTP:  // 自定义键码
        if (!isRunning) {
          motorBTimeout += 1000;  // 增加1秒
          if (motorBTimeout > 15000) motorBTimeout = 8000;  // 最大30秒，最小3秒
          lcd.clear();
          lcd.print(F("B Timeout: "));
          lcd.print(motorBTimeout / 1000);
          lcd.print(F("s"));
          delay(1000);
          updateDisplay();
        }
        break;
        case IR_NTM:  // 自定义键码
        if (!isRunning) {
          motorBTimeout -= 1000;  // 减少1秒
          if (motorBTimeout > 15000) motorBTimeout = 8000;  // 最大30秒，最小3秒
          lcd.clear();
          lcd.print(F("B Timeout: "));
          lcd.print(motorBTimeout / 1000);
          lcd.print(F("s"));
          delay(1000);
          updateDisplay();
        }
        break;
      default:
        #if IR_DEBUG
        Serial.print(F("Unknown IR: 0x"));
        Serial.println(irValue, HEX);
        #endif
        break;
    }
    
    IrReceiver.resume();
  }
}
#endif

// ==================== 串口输入处理 ====================
void processSerialInput() {
  #if ENABLE_KEYBOARD
  while (Serial.available() > 0) {
    char c = Serial.read();
    
    if (c == '\n' || c == '\r' || (millis() - lastSerialCharTime > SERIAL_TIMEOUT && serialBufferIndex > 0)) {
      if (serialBufferIndex > 0) {
        serialBuffer[serialBufferIndex] = '\0';
        
        #if DEBUG
        Serial.print(F("Cmd: "));
        Serial.println(serialBuffer);
        #endif
        
        handleSerialCommand(serialBuffer);
        
        serialBufferIndex = 0;
        serialBuffer[0] = '\0';
      }
    } 
    else if (serialBufferIndex < SERIAL_BUFFER_SIZE - 1 && c >= 32) {
      serialBuffer[serialBufferIndex++] = c;
      lastSerialCharTime = millis();
    }
  }
  #endif
}

// ==================== 串口命令处理函数 ====================
void handleSerialCommand(const char* command) {
  #if ENABLE_KEYBOARD
  // 处理数字命令（设置圈时间）
  if (command[0] >= '0' && command[0] <= '9') {
    long timeValue = atol(command);
    if (timeValue >= 100 && timeValue <= 10000) {
      MANUAL_CIRCLE_TIME = timeValue;
      calculateMotorATimeout();
      
      lcd.clear();
      lcd.print(F("Circle Time Set"));
      lcd.setCursor(0, 1);
      lcd.print(F("T="));
      lcd.print(MANUAL_CIRCLE_TIME);
      lcd.print(F("ms"));
      
      #if DEBUG
      Serial.print(F("Circle time: "));
      Serial.print(MANUAL_CIRCLE_TIME);
      Serial.println(F("ms"));
      #endif
      
      delay(1500);
      updateDisplay();
      return;
    }
  }
  
  switch(command[0]) {
    case 'v': case 'V':
      lcd.clear();
      lcd.print(F("Card Dealer v28.0"));
      lcd.setCursor(0, 1);
      lcd.print(F("Manual Calibration"));
      delay(1000);
      updateDisplay();
      break;
      
    case 'p': case 'P':
      playerCount++;
      if (playerCount > 8) playerCount = 2;
      anglePerPlayer = 360.0 / playerCount;
      calculateMotorATimeout();
      updateDisplay();
      break;
      
    case 'd': case 'D':
      deckCount++;
      if (deckCount > 3) deckCount = 1;
      totalCards = deckCount * (hasJokers ? 54 : 52);
      updateDisplay();
      break;
      
    case 'j': case 'J':
      hasJokers = !hasJokers;
      totalCards = deckCount * (hasJokers ? 54 : 52);
      updateDisplay();
      break;
      
    case 'r': case 'R':
      remainCards += playerCount;
      if (remainCards > playerCount * 4) remainCards = 0;
      totalCards = deckCount * (hasJokers ? 54 : 52);
      if (remainCards > 0 && totalCards > remainCards) totalCards -= remainCards;
      updateDisplay();
      break;
      
    case 's': case 'S':
      if (!isRunning) startDealing();
      else showStatusMessage("Already Running");
      break;
      
    case 'c': case 'C':
      playerCount = 4;
      deckCount = 3;
      hasJokers = 1;
      remainCards = 0;
      stopDealing();
      totalCards = deckCount * (hasJokers ? 54 : 52);
      anglePerPlayer = 360.0 / playerCount;
      calculateMotorATimeout();
      updateDisplay();
      break;
      
    case 't': case 'T':
      if (isRunning) stopDealing();
      else {
        stopAllMotors();
        disableMotorDriver();
        changeState(STATE_IDLE);
        lcd.clear();
        lcd.print(F("Force Stop"));
        delay(1000);
        updateDisplay();
      }
      break;
      
    case 'a': case 'A':
      if (!isRunning) {
        enableMotorDriver();
        float startAngle = getCurrentHeading();
        float testTarget = normalizeAngle(startAngle - 90);
        
        lcd.clear();
        lcd.print(F("Test 90 Rotation"));
        lcd.setCursor(0, 1);
        lcd.print(F("Mode: "));
        lcd.print(runtimeTurnaroundMode == 1 ? "Compass" : "Timeout");
        
        rotationStartHeading = startAngle;
        rotationStartTime = millis();
        aMotorTimeoutStart = millis();
        targetHeading = testTarget;
        
        unsigned long expectedTime = (90.0 / 360.0) * MANUAL_CIRCLE_TIME;
        
        #if DEBUG
        Serial.print(F("90 test: "));
        Serial.print(expectedTime);
        Serial.println(F("ms"));
        #endif
        
        controlMotorA(true);
        
        unsigned long testStart = millis();
        bool testComplete = false;
        
        while (!testComplete && millis() - testStart < 10000) {
          if (rotateToAngle()) {
            testComplete = true;
          }
          delay(10);
        }
        
        if (!testComplete) {
          stopAllMotors();
        }
        
        float endAngle = getCurrentHeading();
        float actualRotation = getSimpleAngleDiff(startAngle, endAngle);
        
        lcd.clear();
        lcd.print(F("Test Complete"));
        lcd.setCursor(0, 1);
        lcd.print(F("Rotated: "));
        lcd.print(actualRotation, 0);
        lcd.print(F("°"));
        
        delay(2000);
        updateDisplay();
      }
      break;
      
    case 'b': case 'B':
      if (!isRunning) {
        enableMotorDriver();
        controlMotorB(1);
        lcd.clear();
        lcd.print(F("Motor B Test"));
        delay(8000);
        stopAllMotors();
        updateDisplay();
      }
      break;
      
    case 'l': case 'L':
      if (!isRunning && compassInitialized) {
        calibrateCompass();
        updateDisplay();
      }
      break;
      
    case 'm': case 'M':
      runtimeTurnaroundMode = !runtimeTurnaroundMode;
      lcd.clear();
      lcd.print(F("Mode Changed"));
      lcd.setCursor(0, 1);
      lcd.print(F("Now: "));
      lcd.print(runtimeTurnaroundMode == 1 ? "Compass" : "Timeout");
      delay(1000);
      updateDisplay();
      
      #if DEBUG
      Serial.print(F("Mode: "));
      Serial.println(runtimeTurnaroundMode == 1 ? "Compass" : "Timeout");
      #endif
      break;
      
    case 'z': case 'Z':
      resetSystem();
      break;
      
    case '+':
      MANUAL_CIRCLE_TIME += 100;
      calculateMotorATimeout();
      lcd.clear();
      lcd.print(F("Circle Time +100"));
      lcd.setCursor(0, 1);
      lcd.print(F("T="));
      lcd.print(MANUAL_CIRCLE_TIME);
      lcd.print(F("ms"));
      delay(1000);
      updateDisplay();
      break;
      
    case '-':
      if (MANUAL_CIRCLE_TIME > 200) {
        MANUAL_CIRCLE_TIME -= 100;
        calculateMotorATimeout();
        lcd.clear();
        lcd.print(F("Circle Time -100"));
        lcd.setCursor(0, 1);
        lcd.print(F("T="));
        lcd.print(MANUAL_CIRCLE_TIME);
        lcd.print(F("ms"));
        delay(1000);
        updateDisplay();
      }
      break;
      
    case 'h': case 'H':
      Serial.println(F("=== Card Dealer Commands ==="));
      Serial.println(F("V - Version"));
      Serial.println(F("P - Increase players"));
      Serial.println(F("D - Increase decks"));
      Serial.println(F("J - Toggle jokers"));
      Serial.println(F("R - Adjust remaining"));
      Serial.println(F("S - Start dealing"));
      Serial.println(F("C - Reset to default"));
      Serial.println(F("T - Stop dealing"));
      Serial.println(F("A - Test Motor A 90°"));
      Serial.println(F("B - Test Motor B"));
      Serial.println(F("L - Calibrate compass"));
      Serial.println(F("M - Toggle mode"));
      Serial.println(F("Z - System reset"));
      Serial.println(F("+/- - Adjust circle time"));
      Serial.println(F("100-10000 - Set circle time"));
      Serial.println(F("H - This help"));
      Serial.print(F("Circle time: "));
      Serial.print(MANUAL_CIRCLE_TIME);
      Serial.println(F("ms"));
      Serial.println(F("========================"));
      break;
      
    default:
      lcd.clear();
      lcd.print(F("Unknown Cmd"));
      delay(1000);
      updateDisplay();
      break;
  }
  #endif
}

// ==================== 显示函数 ====================
void updateDisplay() {
  lcd.clear();
  
  // 第一行：设置信息
  lcd.setCursor(0, 0);
  lcd.print(F("P"));
  lcd.print(playerCount);
  lcd.print(F(" D"));
  lcd.print(deckCount);
  lcd.print(F(" "));
  lcd.print(hasJokers ? F("J") : F("N"));
  lcd.print(F(" M:"));
  lcd.print(runtimeTurnaroundMode == 1 ? "C" : "T");
  
  // 显示圈时间
  lcd.setCursor(10, 0);
  if (MANUAL_CIRCLE_TIME < 1000) lcd.print(F("0"));
  lcd.print(MANUAL_CIRCLE_TIME);
  
  // 第二行：状态信息
  lcd.setCursor(0, 1);
  lcd.print(F("T:"));
  lcd.print(totalCards);
  lcd.print(F(" D:"));
  lcd.print(dealtCards);
  lcd.print(F(" "));
  
  if (isRunning) {
    switch (currentState) {
      case STATE_B_RUNNING: lcd.print(F("B")); break;
      case STATE_A_RUNNING: lcd.print(F("A")); break;
      case STATE_B_TIMEOUT: lcd.print(F("TO")); break;
      default: lcd.print(F("R"));
    }
  } else {
    lcd.print(F("S"));
  }
  
  lcd.print(F(" "));
  lcd.print(obstacleState == HIGH ? F("H") : F("L"));
  
  // 显示当前航向
  lcd.setCursor(12, 1);
  float displayAngle = getCurrentHeading();
  int intAngle = (int)displayAngle;
  if (intAngle < 10) lcd.print(F("00"));
  else if (intAngle < 100) lcd.print(F("0"));
  lcd.print(intAngle);
}

void showStatusMessage(const char* message) {
  lcd.clear();
  lcd.print(message);
  lcd.setCursor(0, 1);
  lcd.print(F("D:"));
  lcd.print(dealtCards);
  lcd.print(F("/"));
  lcd.print(totalCards);
  delay(500);
}

// ==================== SETUP函数 ====================
void setup() {
  delay(500);
  
  // 初始化引脚
  pinMode(MOTOR_A_IN1, OUTPUT);
  pinMode(MOTOR_A_IN2, OUTPUT);
  pinMode(MOTOR_A_PWM, OUTPUT);
  pinMode(MOTOR_B_IN1, OUTPUT);
  pinMode(MOTOR_B_IN2, OUTPUT);
  pinMode(MOTOR_B_PWM, OUTPUT);
  pinMode(MOTOR_STBY, OUTPUT);
  pinMode(OBSTACLE_PIN, INPUT_PULLUP);
  
  disableMotorDriver();
  
  lcd.begin(16, 2);
  lcd.clear();
  lcd.print(F("Card Dealer v28.0"));
  lcd.setCursor(0, 1);
  lcd.print(F("Manual Calibration"));
  
  #if DEBUG
  Serial.begin(9600);
  delay(500);
  Serial.println(F("=== System Startup ==="));
  Serial.println(F("Manual Calibration System"));
  Serial.print(F("Initial mode: "));
  Serial.println(runtimeTurnaroundMode == 1 ? "Compass" : "Timeout");
  Serial.print(F("Circle time: "));
  Serial.print(MANUAL_CIRCLE_TIME);
  Serial.println(F("ms"));
  Serial.println(F("Use 'H' for help"));
  #endif
  
  #if ENABLE_INFRA
  IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK);
  #endif
  
  Wire.begin();
  delay(100);
  
  // 初始化罗盘
  compassInitialized = false;
  for (int attempt = 1; attempt <= 2; attempt++) {
    lcd.clear();
    lcd.print(F("Init Compass"));
    lcd.setCursor(0, 1);
    lcd.print(F("Attempt "));
    lcd.print(attempt);
    
    compassInitialized = initCompassUltimate();
    if (compassInitialized) break;
    delay(1000);
  }
  
  if (compassInitialized) {
    delay(500);
    calibrateCompass();
    
    if (!calibrationDone) {
      lcd.clear();
      lcd.print(F("Calibration"));
      lcd.setCursor(0, 1);
      lcd.print(F("Using Timeout"));
      delay(1000);
    }
  } else {
    lcd.clear();
    lcd.print(F("No Compass"));
    lcd.setCursor(0, 1);
    lcd.print(F("Timeout Mode"));
    delay(1000);
    
    initialHeading = 0;
    currentHeading = 0;
    virtualHeading = 0;
    targetHeading = 0;
    calibrationDone = false;
  }
  
  totalCards = deckCount * (hasJokers ? 54 : 52);
  anglePerPlayer = 360.0 / playerCount;
  calculateMotorATimeout();
  
  obstacleState = digitalRead(OBSTACLE_PIN);
  lastObstacleState = obstacleState;
  
  compassResponding = true;
  noChangeCount = 0;
  
  serialBufferIndex = 0;
  serialBuffer[0] = '\0';
  
  updateDisplay();
  
  #if DEBUG
  Serial.print(F("Players: "));
  Serial.println(playerCount);
  Serial.print(F("Cards: "));
  Serial.println(totalCards);
  Serial.print(F("Compass: "));
  Serial.println(compassInitialized ? "YES" : "NO");
  Serial.println(F("========================"));
  #endif
}

// ==================== LOOP函数 ====================
void loop() {
  processSerialInput();
  
  #if ENABLE_INFRA
  processInfraredInput();
  #endif
  
  checkObstacle();
  
  if (isRunning) {
    handleMotorState();
  }
  
  if (runtimeTurnaroundMode == 1) {
    updateCompassHeading();
  }
  
  #if DEBUG
  // 减少状态打印频率到每3秒一次
  if (millis() - lastDebugTime > 3000) {
    Serial.print(F("Cards: "));
    Serial.print(dealtCards);
    Serial.print(F("/"));
    Serial.print(totalCards);
    Serial.print(F(" Mode: "));
    Serial.print(runtimeTurnaroundMode == 1 ? "Compass" : "Timeout");
    
    if (compassInitialized) {
      Serial.print(F(" Heading: "));
      Serial.print(getCurrentHeading(), 1);
    }
    
    Serial.println();
    
    lastDebugTime = millis();
  }
  #endif
  
  delay(10);
}