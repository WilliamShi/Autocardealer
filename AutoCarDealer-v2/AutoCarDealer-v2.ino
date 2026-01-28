// ==================== 系统变量 ====================
float initialHeading = 0;     // 校准时的初始航向角度，作为系统参考零点
float targetHeading = 0;      // 电机A需要旋转到的目标角度
float currentHeading = 0;     // 当前电子罗盘的实时航向角度
float angleTolerance = 10.0;  // 角度容差范围10°
float anglePerPlayer = 90.0;  // 每个玩家之间的角度间隔，初始为4个玩家(360°/4=90°)

#include <LiquidCrystal.h>
#include <Wire.h>
#include <QMC5883LCompass.h>  // 使用Arduino自带的QMC5883LCompass库

// ==================== 输入控制宏 ====================
#define ENABLE_INFRA 0        // 启用红外输入
#define ENABLE_KEYBOARD 1     // 启用键盘（串口）输入

#if ENABLE_INFRA
#include <IRremote.hpp>
#endif

// ==================== 调试控制 ====================
#define DEBUG 1  // 启用调试信息

// ==================== 引脚定义 ====================
#if ENABLE_INFRA
#define RECV_PIN 10
#endif
#define OBSTACLE_PIN 7

// L9110电机驱动引脚
#define MOTOR_A_IA 8
#define MOTOR_A_IB 9
#define MOTOR_B_IA A3
#define MOTOR_B_IB A2

// ==================== 角度转换宏 ====================
#define DEG_TO_RAD 0.017453292519943295769236907684886
#define RAD_TO_DEG 57.295779513082320876798154814105

// ==================== 模块初始化 ====================
LiquidCrystal lcd(12, 11, 5, 4, 3, 2);
QMC5883LCompass compass;  // 使用新的库对象

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

bool compassInitialized = false;
bool calibrationDone = false;

uint8_t lastObstacleState = HIGH;
uint8_t obstacleState = HIGH;
bool obstacleActive = false;
bool obstacleTriggered = false;

// ==================== 电机A超时控制 ====================
unsigned long TIME_A_CIRCLE = 15000;  // 电机A旋转一圈的时间（毫秒），增加到15秒
unsigned long motorATimeoutPerPlayer = 1500;  // 初始值：6000/4=1500ms

// ==================== 改进的罗盘系统 ====================
float lastStableHeading = 0.0;        // 最后一次稳定的罗盘读数
float filteredHeading = 0.0;          // 滤波后的航向
float headingSamples[3];              // 采样数组
int sampleIndex = 0;
bool samplesReady = false;
unsigned long lastCompassUpdate = 0;
const int COMPASS_UPDATE_INTERVAL = 30;  // 更新间隔

// ==================== 罗盘响应检测 ====================
float lastValidHeading = 0.0;
unsigned long lastHeadingChangeTime = 0;
bool compassResponding = true;
int noChangeCount = 0;
const int MAX_NO_CHANGE = 15;  // 连续15次无变化认为罗盘卡住

// ==================== 虚拟角度备份系统 ====================
float virtualHeading = 0.0;           // 虚拟航向，用于罗盘失效时

// ==================== 上海地区磁偏角修正 ====================
const float MAGNETIC_DECLINATION = 5.0;  // 上海地区的磁偏角（西偏为正）

// ==================== 电机控制参数 ====================
unsigned long lastMotorUpdate = 0;       // 上次电机控制更新时间
const int MOTOR_CONTROL_INTERVAL = 50;   // 电机控制更新间隔(ms)，约20Hz

// ==================== 电机A旋转状态跟踪 ====================
float rotationStartHeading = 0.0;        // 旋转开始时的角度
unsigned long rotationStartTime = 0;     // 旋转开始时间
float lastRotationAngle = 0.0;           // 上次旋转时的角度
bool rotationInProgress = false;         // 旋转是否在进行中

// ==================== 串口输入缓冲区 ====================
const int SERIAL_BUFFER_SIZE = 32;
char serialBuffer[SERIAL_BUFFER_SIZE];
int serialBufferIndex = 0;
unsigned long lastSerialCharTime = 0;
const unsigned long SERIAL_TIMEOUT = 100;  // 串口输入超时时间

// ==================== 电机A旋转检测 ====================
float motorA_lastHeading = 0.0;
unsigned long motorA_lastCheckTime = 0;
const unsigned long MOTOR_A_CHECK_INTERVAL = 500;  // 每500ms检查一次
int motorA_stuckCount = 0;
const int MOTOR_A_STUCK_THRESHOLD = 3;  // 连续3次无变化认为电机卡住

// ==================== I2C看门狗 ====================
unsigned long lastI2CCheck = 0;
const unsigned long I2C_CHECK_INTERVAL = 30000;  // 每30秒检查一次
int i2cErrorCount = 0;
const int I2C_ERROR_THRESHOLD = 3;  // 连续3次错误触发恢复

// ==================== 关键函数修正 ====================

// ==================== 角度处理函数 ====================
float normalizeAngle(float angle) {
  // 将角度规范化到0-360度
  while (angle >= 360.0) {
    angle -= 360.0;
  }
  while (angle < 0.0) {
    angle += 360.0;
  }
  return angle;
}

// 计算角度差（带符号，正数表示需要逆时针旋转的角度）
float getSimpleAngleDiff(float current, float target) {
  // 计算最短路径角度差
  current = normalizeAngle(current);
  target = normalizeAngle(target);
  
  float diff = target - current;
  
  // 确保角度差在±180度范围内
  if (diff > 180.0) {
    diff -= 360.0;
  } else if (diff < -180.0) {
    diff += 360.0;
  }
  
  return diff;
}

// ==================== 电机控制函数 ====================
void stopAllMotors() {
  digitalWrite(MOTOR_A_IA, LOW);
  digitalWrite(MOTOR_A_IB, LOW);
  digitalWrite(MOTOR_B_IA, LOW);
  digitalWrite(MOTOR_B_IB, LOW);
}

// ==================== 电机A控制 ====================
void controlMotorA(bool enable) {
  if (enable) {
    // 电机A永远逆时针旋转
    digitalWrite(MOTOR_A_IA, LOW);
    analogWrite(MOTOR_A_IB, 220);  // 较高速度
    rotationInProgress = true;
  } else {
    digitalWrite(MOTOR_A_IA, LOW);
    digitalWrite(MOTOR_A_IB, LOW);
    rotationInProgress = false;
  }
}

void controlMotorB(uint8_t state) {
  if (state) {
    analogWrite(MOTOR_B_IA, 200);  // 中等速度
    digitalWrite(MOTOR_B_IB, LOW);
  } else {
    digitalWrite(MOTOR_B_IA, LOW);
    digitalWrite(MOTOR_B_IB, LOW);
  }
}

// ==================== 计算超时时间 ====================
void calculateMotorATimeout() {
  // 计算每个玩家角度的超时时间 = 一圈时间 / 玩家数
  motorATimeoutPerPlayer = TIME_A_CIRCLE / playerCount;
  
  // 确保最小超时时间
  if (motorATimeoutPerPlayer < 1500) {
    motorATimeoutPerPlayer = 1500;
  }
  
  #if DEBUG
  Serial.print(F("Recalculated motorA timeout: "));
  Serial.print(motorATimeoutPerPlayer);
  Serial.print(F("ms (Circle time: "));
  Serial.print(TIME_A_CIRCLE);
  Serial.print(F("ms / Players: "));
  Serial.print(playerCount);
  Serial.println(F(")"));
  #endif
}

// ==================== 罗盘读取函数 ====================
bool readCompassHeading(float &heading) {
  if (!compassInitialized) {
    return false;
  }
  
  compass.read();
  int rawHeading = compass.getAzimuth();
  
  // 修正：处理负值航向
  if (rawHeading < 0) {
    rawHeading += 360;
  }
  
  if (rawHeading >= 0 && rawHeading <= 360) {
    heading = normalizeAngle(rawHeading + MAGNETIC_DECLINATION);
    return true;
  }
  
  return false;
}

// ==================== 改进的罗盘更新函数 ====================
void updateCompassHeading() {
  if (!compassInitialized) return;
  
  if (millis() - lastCompassUpdate < COMPASS_UPDATE_INTERVAL) {
    return;
  }
  
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
    headingSamples[sampleIndex] = newHeading;
    sampleIndex = (sampleIndex + 1) % 3;
    
    if (!samplesReady && sampleIndex == 0) {
      samplesReady = true;
    }
    
    if (samplesReady) {
      float sum = 0;
      for (int i = 0; i < 3; i++) {
        sum += headingSamples[i];
      }
      filteredHeading = normalizeAngle(sum / 3.0);
    } else {
      filteredHeading = newHeading;
    }
    
    virtualHeading = filteredHeading;
  }
  
  lastCompassUpdate = millis();
}

// ==================== 获取当前航向 ====================
float getCurrentHeading() {
  if (compassInitialized && compassResponding) {
    return filteredHeading;
  } else {
    return virtualHeading;
  }
}

// ==================== 终极版QMC5883L初始化 ====================
bool initCompassUltimate() {
  #if DEBUG
  Serial.println(F("=== QMC5883L ULTIMATE INITIALIZATION ==="));
  #endif
  
  lcd.clear();
  lcd.print(F("Init Compass..."));
  
  stopAllMotors();
  delay(100);
  
  Wire.end();
  delay(50);
  pinMode(A4, INPUT_PULLUP);
  pinMode(A5, INPUT_PULLUP);
  delay(50);
  Wire.begin();
  Wire.setClock(100000);
  delay(200);
  
  bool i2cConnected = false;
  for (int attempt = 1; attempt <= 5; attempt++) {
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
    lcd.setCursor(0, 1);
    lcd.print(F("Check Wiring"));
    delay(2000);
    return false;
  }
  
  // 深度复位
  Wire.end();
  delay(10);
  Wire.begin();
  Wire.setClock(100000);
  
  for (int i = 0; i < 3; i++) {
    Wire.beginTransmission(0x0D);
    Wire.write(0x0A);
    Wire.write(0x80);
    Wire.endTransmission();
    delay(10);
  }
  
  delay(100);
  
  compass.init();
  delay(100);
  
  bool dataValid = false;
  int successfulReadings = 0;
  
  for (int i = 0; i < 5; i++) {
    compass.read();
    int x = compass.getX();
    int y = compass.getY();
    int z = compass.getZ();
    
    if (x != 0 || y != 0 || z != 0) {
      successfulReadings++;
      dataValid = true;
    }
    delay(50);
  }
  
  compassInitialized = dataValid && (successfulReadings >= 3);
  calibrationDone = false;
  
  if (compassInitialized) {
    float initialReading = 0.0;
    float headingSum = 0;
    int validHeadings = 0;
    
    for (int i = 0; i < 10; i++) {
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
    
    for (int i = 0; i < 3; i++) {
      headingSamples[i] = initialReading;
    }
    samplesReady = true;
    
    compassResponding = true;
    noChangeCount = 0;
    
    #if DEBUG
    Serial.print(F("Compass initialized successfully! Initial heading: "));
    Serial.print(initialReading, 1);
    Serial.println(F("°"));
    #endif
    
    lcd.clear();
    lcd.print(F("Compass OK"));
    delay(500);
  }
  
  return compassInitialized;
}

// ==================== 改进的校准函数（15秒全速旋转） ====================
void calibrateCompass() {
  if (!compassInitialized) {
    #if DEBUG
    Serial.println(F("Cannot calibrate - compass not initialized"));
    #endif
    lcd.clear();
    lcd.print(F("Compass Not Ready"));
    delay(1000);
    return;
  }
  
  lcd.clear();
  lcd.print(F("Calibrating..."));
  lcd.setCursor(0, 1);
  lcd.print(F("15s Full Speed"));
  
  #if DEBUG
  Serial.println(F("Starting compass calibration..."));
  Serial.println(F("Motor A will rotate at full speed for 15 seconds"));
  #endif
  
  stopAllMotors();
  delay(200);
  
  unsigned long originalCircleTime = TIME_A_CIRCLE;
  
  // 设置全速旋转
  TIME_A_CIRCLE = 15000;  // 15秒一圈
  calculateMotorATimeout();
  
  // 记录开始角度
  float startHeading = 0.0;
  for (int i = 0; i < 5; i++) {
    updateCompassHeading();
    delay(100);
  }
  startHeading = getCurrentHeading();
  
  #if DEBUG
  Serial.print(F("Start heading: "));
  Serial.println(startHeading, 1);
  #endif
  
  // 启动电机A全速旋转
  digitalWrite(MOTOR_A_IA, LOW);
  analogWrite(MOTOR_A_IB, 255);  // 全速
  
  unsigned long calibrationStartTime = millis();
  unsigned long rotationDuration = 15000;  // 15秒校准时间
  unsigned long lastSampleTime = 0;
  
  int samples = 0;
  int validSamples = 0;
  float headingSum = 0;
  float minHeading = 360.0;
  float maxHeading = 0.0;
  
  while (millis() - calibrationStartTime < rotationDuration) {
    updateCompassHeading();
    
    if (millis() - lastSampleTime >= 50) {
      lastSampleTime = millis();
      float currentHeading = getCurrentHeading();
      samples++;
      
      if (samples > 5) {
        validSamples++;
        headingSum += currentHeading;
        
        if (currentHeading < minHeading) minHeading = currentHeading;
        if (currentHeading > maxHeading) maxHeading = currentHeading;
        
        #if DEBUG
        if (validSamples % 60 == 0) {
          Serial.print(F("Calibration progress: "));
          Serial.print((millis() - calibrationStartTime) * 100 / rotationDuration);
          Serial.print(F("%"));
          Serial.print(F(", Samples: "));
          Serial.print(validSamples);
          Serial.print(F(", Range: "));
          float range = (maxHeading - minHeading > 180) ? (360 - minHeading + maxHeading) : (maxHeading - minHeading);
          Serial.print(range, 1);
          Serial.println(F("°"));
        }
        #endif
        
        if (validSamples % 10 == 0) {
          lcd.setCursor(0, 1);
          lcd.print(F("S:"));
          lcd.print(validSamples);
          float range = (maxHeading - minHeading > 180) ? (360 - minHeading + maxHeading) : (maxHeading - minHeading);
          lcd.print(F(" R:"));
          lcd.print((int)range);
          lcd.print(F("°"));
        }
      }
    }
  }
  
  stopAllMotors();
  delay(500);
  
  float finalRange = 0.0;
  if (maxHeading - minHeading > 180) {
    finalRange = 360 - minHeading + maxHeading;
  } else {
    finalRange = maxHeading - minHeading;
  }
  
  #if DEBUG
  Serial.print(F("Calibration complete. Valid samples: "));
  Serial.println(validSamples);
  Serial.print(F("Min heading: "));
  Serial.print(minHeading, 1);
  Serial.print(F("°, Max heading: "));
  Serial.print(maxHeading, 1);
  Serial.print(F("°, Range: "));
  Serial.print(finalRange, 1);
  Serial.println(F("°"));
  #endif
  
  if (validSamples >= 100 && finalRange > 300.0) {
    float sumSin = 0.0;
    float sumCos = 0.0;
    
    for (int i = 0; i < 10; i++) {
      updateCompassHeading();
      float h = getCurrentHeading() * DEG_TO_RAD;
      sumSin += sin(h);
      sumCos += cos(h);
      delay(100);
    }
    
    float avgRad = atan2(sumSin, sumCos);
    float avgHeading = avgRad * RAD_TO_DEG;
    if (avgHeading < 0) avgHeading += 360.0;
    
    initialHeading = normalizeAngle(avgHeading);
    currentHeading = initialHeading;
    targetHeading = initialHeading;
    filteredHeading = initialHeading;
    virtualHeading = initialHeading;
    lastValidHeading = initialHeading;
    calibrationDone = true;
    
    for (int i = 0; i < 3; i++) {
      headingSamples[i] = initialHeading;
    }
    
    #if DEBUG
    Serial.print(F("Calibration successful! Samples: "));
    Serial.println(validSamples);
    Serial.print(F("Average heading: "));
    Serial.print(avgHeading, 1);
    Serial.println(F("°"));
    Serial.print(F("Initial heading set to: "));
    Serial.println(initialHeading, 1);
    #endif
    
    lcd.clear();
    lcd.print(F("Calibration OK!"));
    lcd.setCursor(0, 1);
    lcd.print(F("H:"));
    lcd.print((int)initialHeading);
    lcd.print(F(" R:"));
    lcd.print((int)finalRange);
    lcd.print(F("°"));
  } else {
    calibrationDone = false;
    
    #if DEBUG
    Serial.print(F("Calibration failed - valid samples: "));
    Serial.print(validSamples);
    Serial.print(F(", range: "));
    Serial.print(finalRange, 1);
    Serial.println(F("° (need >300°)"));
    #endif
    
    lcd.clear();
    lcd.print(F("Calibration FAIL"));
  }
  
  TIME_A_CIRCLE = originalCircleTime;
  calculateMotorATimeout();
  delay(1500);
}

// ==================== 智能旋转控制 ====================
bool rotateToAngle() {
  if (millis() - lastMotorUpdate < MOTOR_CONTROL_INTERVAL) {
    return false;
  }
  
  lastMotorUpdate = millis();
  float current = getCurrentHeading();
  float angleDiff = getSimpleAngleDiff(current, targetHeading);
  unsigned long elapsed = millis() - aMotorTimeoutStart;
  
  #if DEBUG
  static unsigned long lastRotateDebug = 0;
  if (millis() - lastRotateDebug > 300) {
    Serial.print(F("Rotation: current="));
    Serial.print(current, 1);
    Serial.print(F("°, target="));
    Serial.print(targetHeading, 1);
    Serial.print(F("°, diff="));
    Serial.print(angleDiff, 1);
    Serial.print(F("°, elapsed="));
    Serial.print(elapsed);
    Serial.print(F("ms"));
    
    if (compassInitialized && compassResponding && calibrationDone) {
      Serial.print(F(", Mode: Compass"));
    } else {
      Serial.print(F(", Mode: Timeout"));
    }
    
    Serial.println();
    lastRotateDebug = millis();
  }
  #endif
  
  bool shouldStop = false;
  
  // 模式1：罗盘模式
  if (compassInitialized && compassResponding && calibrationDone) {
    // 条件1：角度差小于容差
    if (fabs(angleDiff) <= angleTolerance && elapsed > 1000) {
      shouldStop = true;
      #if DEBUG
      Serial.print(F("✓ Target reached! Angle diff="));
      Serial.print(fabs(angleDiff), 1);
      Serial.println(F("°"));
      #endif
    }
  } 
  
  // 条件2：超时
  if (elapsed >= motorATimeoutPerPlayer) {
    shouldStop = true;
    #if DEBUG
    Serial.print(F("⏱ Rotation timeout after "));
    Serial.print(elapsed);
    Serial.print(F("ms (expected: "));
    Serial.print(motorATimeoutPerPlayer);
    Serial.println(F("ms)"));
    #endif
  }
  
  if (shouldStop) {
    stopAllMotors();
    
    #if DEBUG
    float finalAngle = getCurrentHeading();
    float actualRotation = getSimpleAngleDiff(rotationStartHeading, finalAngle);
    Serial.print(F("Rotation complete: started at "));
    Serial.print(rotationStartHeading, 1);
    Serial.print(F("°, ended at "));
    Serial.print(finalAngle, 1);
    Serial.print(F("°, rotated "));
    Serial.print(actualRotation, 1);
    Serial.print(F("° in "));
    Serial.print(elapsed);
    Serial.println(F("ms"));
    #endif
    
    return true;
  }
  
  // 继续逆时针旋转
  controlMotorA(true);
  return false;
}

// ==================== 关键修正：障碍事件处理 ====================
void handleObstacleEvent() {
  lastObstacleTime = millis();
  
  stopAllMotors();
  dealtCards++;
  if (dealtCards > totalCards) dealtCards = totalCards;
  
  #if DEBUG
  Serial.print(F("Obstacle detected, dealt: "));
  Serial.println(dealtCards);
  #endif
  
  // 计算目标角度 - 关键修正！
  uint8_t currentPlayerIndex = dealtCards % playerCount;
  
  // 关键修改：罗盘是顺时针增加角度，但电机是逆时针旋转
  // 所以目标角度应该是递减：initialHeading - playerIndex * anglePerPlayer
  float targetAngle = initialHeading - (currentPlayerIndex * anglePerPlayer);
  
  // 规范化角度到0-360度
  targetHeading = normalizeAngle(targetAngle);
  
  #if DEBUG
  Serial.print(F("Player index: "));
  Serial.print(currentPlayerIndex);
  Serial.print(F(", New target heading: "));
  Serial.println(targetHeading, 1);
  Serial.print(F("Calculation: "));
  Serial.print(initialHeading, 1);
  Serial.print(F(" - ("));
  Serial.print(currentPlayerIndex);
  Serial.print(F(" * "));
  Serial.print(anglePerPlayer, 1);
  Serial.print(F(") = "));
  Serial.print(targetAngle, 1);
  Serial.print(F(" -> Normalized: "));
  Serial.println(targetHeading, 1);
  #endif
  
  // 记录旋转开始时的角度
  for (int i = 0; i < 3; i++) {
    updateCompassHeading();
    delay(50);
  }
  rotationStartHeading = getCurrentHeading();
  
  changeState(STATE_A_RUNNING);
  aMotorTimeoutStart = millis();
  lastMotorUpdate = millis();
  
  // 开始旋转
  motorA_lastHeading = getCurrentHeading();
  motorA_lastCheckTime = millis();
  controlMotorA(true);
  
  // 重置罗盘响应状态
  compassResponding = true;
  noChangeCount = 0;
  lastHeadingChangeTime = millis();
  
  showStatusMessage("Card Detected");
  
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
    Serial.print(F("State change: "));
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
  
  if (compassInitialized && calibrationDone) {
    // 等待罗盘稳定
    for (int i = 0; i < 10; i++) {
      updateCompassHeading();
      delay(50);
    }
    
    // 使用校准时的初始航向，不重新获取
    targetHeading = initialHeading;
    virtualHeading = initialHeading;
    
    #if DEBUG
    Serial.print(F("Start dealing with calibrated heading: "));
    Serial.println(initialHeading, 1);
    Serial.print(F("Motor timeout per player: "));
    Serial.print(motorATimeoutPerPlayer);
    Serial.println(F("ms"));
    #endif
  } else {
    // 使用超时模式
    initialHeading = 0;
    currentHeading = 0;
    virtualHeading = 0;
    targetHeading = 0;
    
    #if DEBUG
    Serial.println(F("Start dealing in timeout mode"));
    Serial.print(F("Timeout per player: "));
    Serial.print(motorATimeoutPerPlayer);
    Serial.println(F("ms"));
    #endif
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
  
  #if DEBUG
  Serial.println(F("Stop dealing"));
  #endif
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
      if (millis() - motorStartTime > 5000) {  // B电机超时5秒
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

// ==================== 串口输入处理 ====================
void processSerialInput() {
  #if ENABLE_KEYBOARD
  while (Serial.available() > 0) {
    char c = Serial.read();
    
    if (c == '\n' || c == '\r' || (millis() - lastSerialCharTime > SERIAL_TIMEOUT && serialBufferIndex > 0)) {
      if (serialBufferIndex > 0) {
        serialBuffer[serialBufferIndex] = '\0';
        
        #if DEBUG
        Serial.print(F("Received command: "));
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
  lcd.setCursor(14, 1);
  if (command[0] < 0x10) lcd.print(F("0"));
  lcd.print(command[0], HEX);
  
  switch(command[0]) {
    case 'v':  // 版本显示
    case 'V':
      lcd.clear();
      lcd.print(F("Card Dealer v24.2"));
      lcd.setCursor(0, 1);
      lcd.print(F("Fixed Compass"));
      delay(1000);
      updateDisplay();
      break;
      
    case 'p':  // 增加玩家数量
    case 'P':
      playerCount++;
      if (playerCount > 8) playerCount = 2;
      anglePerPlayer = 360.0 / playerCount;
      calculateMotorATimeout();
      updateDisplay();
      break;
      
    case 'd':  // 增加牌组数量
    case 'D':
      deckCount++;
      if (deckCount > 3) deckCount = 1;
      totalCards = deckCount * (hasJokers ? 54 : 52);
      if (remainCards > 0 && totalCards > remainCards) totalCards -= remainCards;
      updateDisplay();
      break;
      
    case 'j':  // 切换是否有王
    case 'J':
      hasJokers = !hasJokers;
      totalCards = deckCount * (hasJokers ? 54 : 52);
      if (remainCards > 0 && totalCards > remainCards) totalCards -= remainCards;
      updateDisplay();
      break;
      
    case 'r':  // 调整剩余牌数
    case 'R':
      remainCards += playerCount;
      if (remainCards > playerCount * 4) remainCards = 0;
      totalCards = deckCount * (hasJokers ? 54 : 52);
      if (remainCards > 0 && totalCards > remainCards) totalCards -= remainCards;
      updateDisplay();
      break;
      
    case 's':  // 开始发牌
    case 'S':
      if (!isRunning) {
        startDealing();
      } else {
        showStatusMessage("Already Running");
        delay(500);
        updateDisplay();
      }
      break;
      
    case 'c':  // 重置到默认
    case 'C':
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
      
    case 't':  // 停止发牌
    case 'T':
      if (isRunning) {
        stopDealing();
        delay(500);
        updateDisplay();
      } else {
        stopAllMotors();
        changeState(STATE_IDLE);
        lcd.clear();
        lcd.print(F("Force Stop"));
        delay(1000);
        updateDisplay();
      }
      break;
      
    case 'a':  // 测试电机A旋转90度
    case 'A':
      if (!isRunning) {
        float startAngle = getCurrentHeading();
        float testTarget = normalizeAngle(startAngle - 90);  // 逆时针旋转
        
        lcd.clear();
        lcd.print(F("Test 90° Rotation"));
        lcd.setCursor(0, 1);
        lcd.print(F("Start:"));
        lcd.print((int)startAngle);
        lcd.print(F("->"));
        lcd.print((int)testTarget);
        
        #if DEBUG
        Serial.println(F("=== 90° Rotation Test ==="));
        Serial.print(F("Start angle: "));
        Serial.println(startAngle, 1);
        Serial.print(F("Target angle: "));
        Serial.println(testTarget, 1);
        #endif
        
        rotationStartHeading = startAngle;
        aMotorTimeoutStart = millis();
        targetHeading = testTarget;
        
        digitalWrite(MOTOR_A_IA, LOW);
        analogWrite(MOTOR_A_IB, 220);
        
        unsigned long testStart = millis();
        bool testComplete = false;
        
        while (!testComplete && millis() - testStart < 10000) {
          float current = getCurrentHeading();
          float diff = getSimpleAngleDiff(current, testTarget);
          
          if (fabs(diff) <= angleTolerance || (millis() - aMotorTimeoutStart >= motorATimeoutPerPlayer)) {
            testComplete = true;
            stopAllMotors();
            
            float endAngle = getCurrentHeading();
            float actualRotation = getSimpleAngleDiff(startAngle, endAngle);
            
            #if DEBUG
            Serial.print(F("Test complete! Final angle: "));
            Serial.print(endAngle, 1);
            Serial.print(F("°, Total rotation: "));
            Serial.print(actualRotation, 1);
            Serial.print(F("°, Time: "));
            Serial.print(millis() - testStart);
            Serial.println(F("ms"));
            #endif
            
            lcd.clear();
            lcd.print(F("Test Complete"));
            lcd.setCursor(0, 1);
            lcd.print(F("Rotated: "));
            lcd.print(actualRotation, 0);
            lcd.print(F("°"));
          }
          
          updateCompassHeading();
          delay(100);
        }
        
        delay(2000);
        updateDisplay();
      }
      break;
      
    case 'b':  // 测试电机B
    case 'B':
      if (!isRunning) {
        controlMotorB(1);
        lcd.clear();
        lcd.print(F("Motor B ON Test"));
        delay(1500);
        stopAllMotors();
        updateDisplay();
      }
      break;
      
    case 'l':  // 校准电子罗盘
    case 'L':
      if (!isRunning && compassInitialized) {
        calibrateCompass();
        updateDisplay();
      }
      break;
      
    case 'h':  // 显示帮助信息
    case 'H':
      Serial.println(F("=== Card Dealer Commands v24.2 ==="));
      Serial.println(F("V - Version display"));
      Serial.println(F("P - Increase player count"));
      Serial.println(F("D - Increase deck count"));
      Serial.println(F("J - Toggle jokers"));
      Serial.println(F("R - Adjust remaining cards"));
      Serial.println(F("S - Start dealing"));
      Serial.println(F("C - Reset to default"));
      Serial.println(F("T - Stop dealing"));
      Serial.println(F("A - Test Motor A 90° rotation"));
      Serial.println(F("B - Test Motor B"));
      Serial.println(F("L - Calibrate compass"));
      Serial.println(F("H - This help"));
      Serial.println(F("=========================="));
      break;
      
    default:
      lcd.clear();
      lcd.print(F("Unknown Cmd:"));
      lcd.print(command[0]);
      delay(1000);
      updateDisplay();
      break;
  }
  #endif
}

// ==================== 显示函数 ====================
void updateDisplay() {
  lcd.clear();
  
  // 第一行
  lcd.setCursor(0, 0);
  lcd.print(F("P"));
  lcd.print(playerCount);
  lcd.print(F(" D"));
  lcd.print(deckCount);
  lcd.print(F(" "));
  lcd.print(hasJokers ? F("J") : F("N"));
  lcd.print(F(" "));
  
  // 显示角度间隔
  if (anglePerPlayer < 10) {
    lcd.print(F("00"));
    lcd.print((int)anglePerPlayer);
  } else if (anglePerPlayer < 100) {
    lcd.print(F("0"));
    lcd.print((int)anglePerPlayer);
  } else {
    lcd.print((int)anglePerPlayer);
  }
  lcd.write(223);  // 度符号
  
  // 显示已发牌数
  lcd.setCursor(11, 0);
  lcd.print(F("D:"));
  
  if (dealtCards < 10) {
    lcd.print(F("000"));
    lcd.print(dealtCards);
  } else if (dealtCards < 100) {
    lcd.print(F("00"));
    lcd.print(dealtCards);
  } else if (dealtCards < 1000) {
    lcd.print(F("0"));
    lcd.print(dealtCards);
  } else {
    lcd.print(dealtCards);
  }
  
  // 第二行
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
  lcd.setCursor(13, 0);
  float displayAngle = getCurrentHeading();
  int intAngle = (int)displayAngle;
  if (intAngle < 10) {
    lcd.print(F("00"));
    lcd.print(intAngle);
  } else if (intAngle < 100) {
    lcd.print(F("0"));
    lcd.print(intAngle);
  } else {
    lcd.print(intAngle);
  }
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
  
  pinMode(MOTOR_A_IA, OUTPUT);
  pinMode(MOTOR_A_IB, OUTPUT);
  pinMode(MOTOR_B_IA, OUTPUT);
  pinMode(MOTOR_B_IB, OUTPUT);
  pinMode(OBSTACLE_PIN, INPUT_PULLUP);
  
  stopAllMotors();
  
  lcd.begin(16, 2);
  lcd.clear();
  lcd.print(F("Card Dealer v24.2"));
  lcd.setCursor(0, 1);
  lcd.print(F("Fixed Rotation"));
  
  #if DEBUG
  Serial.begin(9600);
  delay(500);
  Serial.println(F("System Startup v24.2"));
  Serial.println(F("=========================="));
  Serial.println(F("MOTOR A: CCW ONLY MODE"));
  Serial.print(F("Magnetic Declination: "));
  Serial.print(MAGNETIC_DECLINATION);
  Serial.println(F("° (Shanghai)"));
  Serial.println(F("FIXED: CCW rotation = initial - angle"));
  #endif
  
  Wire.begin();
  delay(100);
  
  int maxAttempts = 2;
  compassInitialized = false;
  
  for (int attempt = 1; attempt <= maxAttempts; attempt++) {
    #if DEBUG
    Serial.print(F("Compass initialization attempt "));
    Serial.println(attempt);
    #endif
    
    lcd.clear();
    lcd.print(F("Init Compass"));
    lcd.setCursor(0, 1);
    lcd.print(F("Attempt "));
    lcd.print(attempt);
    
    compassInitialized = initCompassUltimate();
    
    if (compassInitialized) {
      break;
    }
    
    if (attempt < maxAttempts) {
      delay(1000);
    }
  }
  
  if (compassInitialized) {
    delay(500);
    calibrateCompass();
    
    if (!calibrationDone) {
      lcd.clear();
      lcd.print(F("Cal FAIL"));
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
    targetHeading = 0;
    virtualHeading = 0;
    filteredHeading = 0;
    calibrationDone = false;
  }
  
  uint8_t cardsPerDeck = hasJokers ? 54 : 52;
  totalCards = deckCount * cardsPerDeck;
  
  if (remainCards > 0 && totalCards > remainCards) {
    totalCards -= remainCards;
  }
  
  anglePerPlayer = 360.0 / playerCount;
  calculateMotorATimeout();
  
  obstacleState = digitalRead(OBSTACLE_PIN);
  lastObstacleState = obstacleState;
  
  lastHeadingChangeTime = millis();
  compassResponding = true;
  noChangeCount = 0;
  
  serialBufferIndex = 0;
  serialBuffer[0] = '\0';
  lastSerialCharTime = millis();
  
  updateDisplay();
  
  #if DEBUG
  Serial.print(F("Angle per player: "));
  Serial.println(anglePerPlayer, 1);
  Serial.print(F("Motor A timeout per step: "));
  Serial.print(motorATimeoutPerPlayer);
  Serial.println(F(" ms"));
  Serial.print(F("Player count: "));
  Serial.println(playerCount);
  Serial.print(F("Total cards: "));
  Serial.println(totalCards);
  Serial.println(F("Setup complete"));
  Serial.print(F("Compass: "));
  Serial.println(compassInitialized ? "YES" : "NO");
  Serial.print(F("Calibration: "));
  Serial.println(calibrationDone ? "YES" : "NO");
  Serial.println(F("=========================="));
  #endif
}

// ==================== LOOP函数 ====================
void loop() {
  processSerialInput();
  checkObstacle();
  
  if (isRunning) {
    handleMotorState();
  }
  
  updateCompassHeading();
  
  #if DEBUG
  if (millis() - lastDebugTime > 1000) {
    float current = getCurrentHeading();
    
    Serial.print(F("State: "));
    Serial.print(currentState);
    Serial.print(F(" Cards: "));
    Serial.print(dealtCards);
    Serial.print(F("/"));
    Serial.print(totalCards);
    
    if (compassInitialized) {
      Serial.print(F(" Heading: "));
      Serial.print(current, 1);
      Serial.print(F(" ("));
      Serial.print(compassResponding ? "Active" : "Stuck");
      Serial.print(F(")"));
    } else {
      Serial.print(F(" Virtual: "));
      Serial.print(current, 1);
    }
    
    Serial.print(F(" Target: "));
    Serial.print(targetHeading, 1);
    float diff = getSimpleAngleDiff(current, targetHeading);
    Serial.print(F(" Diff: "));
    Serial.print(diff, 1);
    
    if (compassInitialized && compassResponding && calibrationDone) {
      Serial.print(F(" [Compass Mode]"));
    } else {
      Serial.print(F(" [Timeout Mode]"));
    }
    
    Serial.println();
    
    lastDebugTime = millis();
  }
  #endif
  
  delay(10);
}