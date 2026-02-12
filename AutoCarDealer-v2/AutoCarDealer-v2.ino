// ==================== 系统配置宏 ====================
#define TURNAROUND_MODE 0      // 0=超时模式, 1=罗盘模式
#define ENABLE_INFRA 1
#define ENABLE_KEYBOARD 0
#define DEBUG 1
#define IR_DEBUG 1

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
unsigned long motorBTimeout = 8000;          // 电机B超时（毫秒）
bool compassInitialized = false;
bool calibrationDone = false;
uint8_t lastObstacleState = HIGH;
uint8_t obstacleState = HIGH;
bool obstacleActive = false;
bool obstacleTriggered = false;

// ==================== 关键修正：手动校准的电机参数 ====================
unsigned long MANUAL_CIRCLE_TIME = 3050;      // 整圈时间（ms）
unsigned long motorATimeoutPerPlayer = 765;   // 每张牌超时时间

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
const int MOTOR_A_SPEED = 150;
const int MOTOR_B_SPEED = 255;
const int MOTOR_A_CAL_SPEED = 200;

// ==================== 红外遥控键码定义 ====================
#if ENABLE_INFRA
#define IR_DEC_PLAYER     0xBA45FF00
#define IR_RESET          0xB946FF00
#define IR_INC_PLAYER     0xB847FF00
#define IR_INC_DECK       0xBB44FF00
#define IR_TOGGLE_JOKER   0xBF40FF00
#define IR_START          0xBC43FF00
#define IR_DEC_CIRCLE_TIME 0xF807FF00
#define IR_INC_CIRCLE_TIME 0xEA15FF00
#define IR_STOP           0xF609FF00
#define IR_REMAIN_CARDS   0xE916FF00
#define IR_TEST_MOTOR_A   0xE619FF00
#define IR_TEST_MOTOR_B   0xF20DFF00
#define IR_CALIBRATE      0xF30CFF00
#define IR_TOGGLE_MODE    0xE718FF00
#define IR_SYSTEM_RESET   0xA15EFF00
#define IR_VERSION        0xF708FF00
#define IR_BTM            0xE31CFF00
#define IR_BTP            0xA55AFF00
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

// ==================== 计算超时时间（四舍五入） ====================
void calculateMotorATimeout() {
  // [FIX] 浮点除法 + 四舍五入，消除截断误差
  motorATimeoutPerPlayer = (unsigned long)((float)MANUAL_CIRCLE_TIME / playerCount + 0.5);
  if (motorATimeoutPerPlayer < 100) motorATimeoutPerPlayer = 100;
  #if DEBUG
  Serial.print(F("Circle: "));
  Serial.print(MANUAL_CIRCLE_TIME);
  Serial.print(F("ms, Timeout/player: "));
  Serial.print(motorATimeoutPerPlayer);
  Serial.println(F("ms"));
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
  unsigned long elapsed = millis() - rotationStartTime;
  float rotationSpeed = 360.0 / (MANUAL_CIRCLE_TIME / 1000.0);
  float expectedRotation = (elapsed / 1000.0) * rotationSpeed;
  virtualHeading = normalizeAngle(rotationStartHeading - expectedRotation);
  return virtualHeading;
}

// ==================== 获取当前航向（增强版） ====================
float getCurrentHeading() {
  // [FIX] 罗盘模式下，若电机正在旋转，直接返回虚拟航向（无滞后）
  if (runtimeTurnaroundMode == 1 && rotationInProgress) {
    return updateVirtualHeadingDuringRotation();
  }
  // 否则按原逻辑：罗盘可用则返回滤波值，否则虚拟航向
  if (compassInitialized && compassResponding) {
    return filteredHeading;
  } else {
    return updateVirtualHeadingDuringRotation();
  }
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
      if (noChangeCount > MAX_NO_CHANGE) compassResponding = false;
    }
    currentHeading = newHeading;
    lastValidHeading = newHeading;
    virtualHeading = newHeading;   // 罗盘有效时同步虚拟航向
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

// ==================== QMC5883L初始化 ====================
bool initCompassUltimate() {
  #if DEBUG
  Serial.println(F("Init Compass..."));
  #endif
  lcd.clear();
  lcd.print(F("Init Comp..."));   // [精简]
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
    if (error == 0) { i2cConnected = true; break; }
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
    int x = compass.getX(), y = compass.getY(), z = compass.getZ();
    if (x != 0 || y != 0 || z != 0) { dataValid = true; break; }
    delay(50);
  }
  compassInitialized = dataValid;
  calibrationDone = false;
  if (compassInitialized) {
    float initialReading = 0.0, headingSum = 0;
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
    lcd.print(F("Compass N/A"));
    delay(1000);
    return;
  }
  lcd.clear();
  lcd.print(F("Calib..."));     // [精简]
  stopAllMotors();
  delay(200);
  enableMotorDriver();
  float startHeading = 0.0;
  for (int i = 0; i < 5; i++) { updateCompassHeading(); delay(100); }
  startHeading = getCurrentHeading();
  controlMotorAFullSpeed();
  delay(5000);
  stopAllMotors();
  delay(500);
  float finalHeading = 0.0;
  for (int i = 0; i < 5; i++) { updateCompassHeading(); delay(100); }
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
  Serial.print(F("Cal done: "));
  Serial.println(avgHeading, 1);
  #endif
  lcd.clear();
  lcd.print(F("Cal OK "));      // [精简]
  lcd.print((int)avgHeading);
  lcd.print(F("°"));
  delay(1000);
}

// ==================== 核心：旋转控制函数（修正版） ====================
bool rotateToAngle() {
  if (millis() - lastMotorUpdate < MOTOR_CONTROL_INTERVAL) return false;
  lastMotorUpdate = millis();
  unsigned long elapsed = millis() - aMotorTimeoutStart;

  if (runtimeTurnaroundMode == 1 && compassInitialized && calibrationDone) {
    // ---------- 罗盘模式：闭环角度控制 + 长超时保护 ----------
    float current = getCurrentHeading();   // 旋转时返回虚拟航向，响应迅速
    float angleDiff = getSimpleAngleDiff(current, targetHeading);
    if (fabs(angleDiff) <= angleTolerance) {
      stopAllMotors();
      return true;
    }
    // [FIX] 超时设为整圈时间2倍，仅作防死锁
    if (elapsed >= MANUAL_CIRCLE_TIME * 2) {
      stopAllMotors();
      return true;
    }
    controlMotorA(true);
    return false;
  }
  else {
    // ---------- 超时模式：纯开环定时，移除所有角度依赖 ----------
    if (elapsed >= motorATimeoutPerPlayer) {
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
  // [FIX] 目标角度始终以 initialHeading 为基准（逆时针减小）
  float targetAngle = initialHeading - (currentPlayerIndex * anglePerPlayer);
  targetHeading = normalizeAngle(targetAngle);

  // 记录旋转起点
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

  showStatusMessage("Rotate...");   // [精简]

  if (dealtCards >= totalCards && totalCards > 0) {
    stopDealing();
    showStatusMessage("Done!");
    delay(500);                    // [精简]
    updateDisplay();
  }
}

// ==================== 状态切换 ====================
void changeState(SystemState newState) {
  #if DEBUG
  const char* stateNames[] = {"IDLE", "B_RUN", "A_RUN", "B_TO"};
  if (currentState != newState) {
    Serial.print(F("State: "));
    Serial.print(stateNames[currentState]);
    Serial.print(F("->"));
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
    delay(500);
    updateDisplay();
    return;
  }
  isRunning = 1;
  dealtCards = 0;

  stopAllMotors();
  enableMotorDriver();

  if (runtimeTurnaroundMode == 1 && compassInitialized && calibrationDone) {
    // [FIX] 发牌开始时，以当前航向作为基准 initialHeading
    initialHeading = getCurrentHeading();
    targetHeading = initialHeading;
    virtualHeading = initialHeading;
  } else {
    // 超时模式：强制使用虚拟坐标系，initialHeading=0
    initialHeading = 0;
    currentHeading = 0;
    virtualHeading = 0;
    targetHeading = 0;
  }

  changeState(STATE_B_RUNNING);
  motorStartTime = millis();
  controlMotorB(1);

  showStatusMessage("Start");
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
      // [FIX] 使用可调节的 motorBTimeout
      if (millis() - motorStartTime > motorBTimeout) {
        changeState(STATE_B_TIMEOUT);
        showStatusMessage("B Timeout");
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
          delay(500);
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
  lcd.print(F("Reset"));
  lcd.setCursor(0,1);
  lcd.print(runtimeTurnaroundMode==1?F("Compass"):F("Timeout"));
  delay(800);
  updateDisplay();
}

// ==================== 红外遥控处理 ====================
#if ENABLE_INFRA
void processInfraredInput() {
  if (IrReceiver.decode()) {
    unsigned long irValue = IrReceiver.decodedIRData.decodedRawData;
    #if IR_DEBUG
    Serial.print(F("IR: 0x"));
    Serial.println(irValue, HEX);
    #endif
    // [FIX] 忽略重复码
    if (irValue == 0xFFFFFFFF) {
      IrReceiver.resume();
      return;
    }
    switch(irValue) {
      case IR_DEC_PLAYER:
        if (!isRunning) {
          if (playerCount > 2) playerCount--; else playerCount = 8;
          anglePerPlayer = 360.0 / playerCount;
          calculateMotorATimeout();
          updateDisplay();
        }
        break;
      case IR_RESET:
        if (!isRunning) {
          playerCount = 4; deckCount = 3; hasJokers = 1; remainCards = 0;
          totalCards = deckCount * (hasJokers ? 54 : 52);
          anglePerPlayer = 360.0 / playerCount;
          calculateMotorATimeout();
          updateDisplay();
        }
        break;
      case IR_INC_PLAYER:
        if (!isRunning) {
          playerCount++; if (playerCount > 8) playerCount = 2;
          anglePerPlayer = 360.0 / playerCount;
          calculateMotorATimeout();
          updateDisplay();
        }
        break;
      case IR_INC_DECK:
        if (!isRunning) {
          deckCount++; if (deckCount > 3) deckCount = 1;
          totalCards = deckCount * (hasJokers ? 54 : 52);
          updateDisplay();
        }
        break;
      case IR_TOGGLE_JOKER:
        if (!isRunning) {
          hasJokers = !hasJokers;
          totalCards = deckCount * (hasJokers ? 54 : 52);
          updateDisplay();
        }
        break;
      case IR_START:
        if (!isRunning) startDealing();
        else showStatusMessage("Running");
        break;
      case IR_DEC_CIRCLE_TIME:
        if (!isRunning && MANUAL_CIRCLE_TIME > 200) {
          MANUAL_CIRCLE_TIME -= 100;
          calculateMotorATimeout();
          lcd.clear();
          lcd.print(F("Circle-100"));
          lcd.setCursor(0,1);
          lcd.print(MANUAL_CIRCLE_TIME);
          lcd.print(F("ms"));
          delay(800);
          updateDisplay();
        }
        break;
      case IR_INC_CIRCLE_TIME:
        if (!isRunning) {
          MANUAL_CIRCLE_TIME += 100;
          calculateMotorATimeout();
          lcd.clear();
          lcd.print(F("Circle+100"));
          lcd.setCursor(0,1);
          lcd.print(MANUAL_CIRCLE_TIME);
          lcd.print(F("ms"));
          delay(800);
          updateDisplay();
        }
        break;
      case IR_STOP:
        if (isRunning) stopDealing();
        else {
          lcd.clear();
          lcd.print(F("Idle"));
          delay(800);
          updateDisplay();
        }
        break;
      case IR_REMAIN_CARDS:
        if (!isRunning) {
          remainCards += playerCount;
          if (remainCards > playerCount * 4) remainCards = 0;
          totalCards = deckCount * (hasJokers ? 54 : 52);
          if (remainCards > 0 && totalCards > remainCards) totalCards -= remainCards;
          updateDisplay();
        }
        break;
      case IR_TEST_MOTOR_A:
        if (!isRunning) {
          enableMotorDriver();
          float startAngle = getCurrentHeading();
          float testTarget = normalizeAngle(startAngle - 90);
          lcd.clear();
          lcd.print(F("90° Test"));
          lcd.setCursor(0,1);
          lcd.print(runtimeTurnaroundMode==1?F("Compass"):F("Timeout"));
          rotationStartHeading = startAngle;
          rotationStartTime = millis();
          aMotorTimeoutStart = millis();
          targetHeading = testTarget;
          #if DEBUG
          unsigned long expectedTime = (90.0/360.0)*MANUAL_CIRCLE_TIME;
          Serial.print(F("90 test exp="));
          Serial.print(expectedTime);
          Serial.println(F("ms"));
          #endif
          controlMotorA(true);
          unsigned long testStart = millis();
          bool testComplete = false;
          while (!testComplete && millis() - testStart < 10000) {
            if (rotateToAngle()) testComplete = true;
            delay(10);
          }
          if (!testComplete) stopAllMotors();
          float endAngle = getCurrentHeading();
          float actualRotation = getSimpleAngleDiff(startAngle, endAngle);
          lcd.clear();
          lcd.print(F("Rot:"));
          lcd.print((int)actualRotation);
          lcd.print(F("°"));
          delay(500);           // [FIX] 2000ms -> 500ms
          updateDisplay();
        }
        break;
      case IR_TEST_MOTOR_B:
        // [FIX] 重写为等待避障触发
        if (!isRunning) {
          enableMotorDriver();
          controlMotorB(1);
          lcd.clear();
          lcd.print(F("MotorB Test"));
          lcd.setCursor(0,1);
          lcd.print(F("Wait Obstacle..."));
          unsigned long testStart = millis();
          bool obstacleDetected = false;
          while (millis() - testStart < 5000) {  // 5秒超时
            uint8_t pinState = digitalRead(OBSTACLE_PIN);
            if (pinState == LOW) {
              delay(50);  // 简单消抖
              if (digitalRead(OBSTACLE_PIN) == LOW) {
                obstacleDetected = true;
                break;
              }
            }
            delay(10);
          }
          stopAllMotors();
          lcd.clear();
          if (obstacleDetected) lcd.print(F("B Test OK"));
          else lcd.print(F("B Timeout"));
          delay(800);
          updateDisplay();
        }
        break;
      case IR_CALIBRATE:
        if (!isRunning && compassInitialized) {
          calibrateCompass();
          updateDisplay();
        }
        break;
      case IR_TOGGLE_MODE:
        runtimeTurnaroundMode = !runtimeTurnaroundMode;
        lcd.clear();
        lcd.print(F("Mode:"));
        lcd.print(runtimeTurnaroundMode==1?F("C"):F("T"));
        delay(800);
        updateDisplay();
        #if DEBUG
        Serial.print(F("Mode: "));
        Serial.println(runtimeTurnaroundMode==1?F("Compass"):F("Timeout"));
        #endif
        break;
      case IR_SYSTEM_RESET:
        resetSystem();
        break;
      case IR_VERSION:
        lcd.clear();
        lcd.print(F("Dealer v28.0"));
        lcd.setCursor(0,1);
        lcd.print(F("Manual Cal"));
        delay(800);
        updateDisplay();
        break;
      case IR_BTP:
        if (!isRunning) {
          motorBTimeout += 1000;
          if (motorBTimeout > 15000) motorBTimeout = 8000;
          lcd.clear();
          lcd.print(F("B Tout:"));
          lcd.print(motorBTimeout/1000);
          lcd.print(F("s"));
          delay(800);
          updateDisplay();
        }
        break;
      case IR_BTM:
        if (!isRunning) {
          if (motorBTimeout > 2000) motorBTimeout -= 1000; else motorBTimeout = 8000;
          lcd.clear();
          lcd.print(F("B Tout:"));
          lcd.print(motorBTimeout/1000);
          lcd.print(F("s"));
          delay(800);
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
    } else if (serialBufferIndex < SERIAL_BUFFER_SIZE - 1 && c >= 32) {
      serialBuffer[serialBufferIndex++] = c;
      lastSerialCharTime = millis();
    }
  }
  #endif
}

// ==================== 串口命令处理函数 ====================
void handleSerialCommand(const char* command) {
  #if ENABLE_KEYBOARD
  if (command[0] >= '0' && command[0] <= '9') {
    long timeValue = atol(command);
    if (timeValue >= 100 && timeValue <= 10000) {
      MANUAL_CIRCLE_TIME = timeValue;
      calculateMotorATimeout();
      lcd.clear();
      lcd.print(F("Circle Set"));
      lcd.setCursor(0,1);
      lcd.print(MANUAL_CIRCLE_TIME);
      lcd.print(F("ms"));
      #if DEBUG
      Serial.print(F("Circle: "));
      Serial.print(MANUAL_CIRCLE_TIME);
      Serial.println(F("ms"));
      #endif
      delay(800);
      updateDisplay();
      return;
    }
  }
  switch(command[0]) {
    case 'v': case 'V':
      lcd.clear();
      lcd.print(F("Dealer v28.0"));
      lcd.setCursor(0,1);
      lcd.print(F("Manual Cal"));
      delay(800);
      updateDisplay();
      break;
    case 'p': case 'P':
      playerCount++; if (playerCount > 8) playerCount = 2;
      anglePerPlayer = 360.0 / playerCount;
      calculateMotorATimeout();
      updateDisplay();
      break;
    case 'd': case 'D':
      deckCount++; if (deckCount > 3) deckCount = 1;
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
      else showStatusMessage("Running");
      break;
    case 'c': case 'C':
      playerCount = 4; deckCount = 3; hasJokers = 1; remainCards = 0;
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
        lcd.print(F("Stop"));
        delay(800);
        updateDisplay();
      }
      break;
    case 'a': case 'A':
      if (!isRunning) {
        enableMotorDriver();
        float startAngle = getCurrentHeading();
        float testTarget = normalizeAngle(startAngle - 90);
        lcd.clear();
        lcd.print(F("90° Test"));
        lcd.setCursor(0,1);
        lcd.print(runtimeTurnaroundMode==1?F("C"):F("T"));
        rotationStartHeading = startAngle;
        rotationStartTime = millis();
        aMotorTimeoutStart = millis();
        targetHeading = testTarget;
        #if DEBUG
        unsigned long expectedTime = (90.0/360.0)*MANUAL_CIRCLE_TIME;
        Serial.print(F("90 test exp="));
        Serial.print(expectedTime);
        Serial.println(F("ms"));
        #endif
        controlMotorA(true);
        unsigned long testStart = millis();
        bool testComplete = false;
        while (!testComplete && millis() - testStart < 10000) {
          if (rotateToAngle()) testComplete = true;
          delay(10);
        }
        if (!testComplete) stopAllMotors();
        float endAngle = getCurrentHeading();
        float actualRotation = getSimpleAngleDiff(startAngle, endAngle);
        lcd.clear();
        lcd.print(F("Rot:"));
        lcd.print((int)actualRotation);
        lcd.print(F("°"));
        delay(500);
        updateDisplay();
      }
      break;
    case 'b': case 'B':
      // [FIX] 串口电机B测试也改为避障触发
      if (!isRunning) {
        enableMotorDriver();
        controlMotorB(1);
        lcd.clear();
        lcd.print(F("MotorB Test"));
        lcd.setCursor(0,1);
        lcd.print(F("Wait Ob..."));
        unsigned long testStart = millis();
        bool obstacleDetected = false;
        while (millis() - testStart < 5000) {
          if (digitalRead(OBSTACLE_PIN) == LOW) {
            delay(50);
            if (digitalRead(OBSTACLE_PIN) == LOW) {
              obstacleDetected = true;
              break;
            }
          }
          delay(10);
        }
        stopAllMotors();
        lcd.clear();
        if (obstacleDetected) lcd.print(F("B OK"));
        else lcd.print(F("B TO"));
        delay(800);
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
      lcd.print(F("Mode:"));
      lcd.print(runtimeTurnaroundMode==1?F("C"):F("T"));
      delay(800);
      updateDisplay();
      #if DEBUG
      Serial.print(F("Mode: "));
      Serial.println(runtimeTurnaroundMode==1?F("Compass"):F("Timeout"));
      #endif
      break;
    case 'z': case 'Z':
      resetSystem();
      break;
    case '+':
      MANUAL_CIRCLE_TIME += 100;
      calculateMotorATimeout();
      lcd.clear();
      lcd.print(F("Circle+100"));
      lcd.setCursor(0,1);
      lcd.print(MANUAL_CIRCLE_TIME);
      lcd.print(F("ms"));
      delay(800);
      updateDisplay();
      break;
    case '-':
      if (MANUAL_CIRCLE_TIME > 200) {
        MANUAL_CIRCLE_TIME -= 100;
        calculateMotorATimeout();
        lcd.clear();
        lcd.print(F("Circle-100"));
        lcd.setCursor(0,1);
        lcd.print(MANUAL_CIRCLE_TIME);
        lcd.print(F("ms"));
        delay(800);
        updateDisplay();
      }
      break;
    case 'h': case 'H':
      Serial.println(F("=== Card Dealer Cmds ==="));
      Serial.println(F("V/D/J/R/S/C/T/A/B/L/M/Z/+/-"));
      Serial.print(F("Circle: "));
      Serial.print(MANUAL_CIRCLE_TIME);
      Serial.println(F("ms"));
      break;
    default:
      lcd.clear();
      lcd.print(F("?"));
      delay(800);
      updateDisplay();
      break;
  }
  #endif
}

// ==================== 显示函数 ====================
void updateDisplay() {
  lcd.clear();
  // 第一行：Px Dx J/N M:C/T 圈时间
  lcd.setCursor(0,0);
  lcd.print(F("P"));
  lcd.print(playerCount);
  lcd.print(F(" D"));
  lcd.print(deckCount);
  lcd.print(hasJokers?F(" J"):F(" N"));
  lcd.print(F(" M:"));
  lcd.print(runtimeTurnaroundMode==1?F("C"):F("T"));
  lcd.setCursor(10,0);
  lcd.print(MANUAL_CIRCLE_TIME);
  // 第二行：总牌/已发 状态 航向
  lcd.setCursor(0,1);
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
  lcd.print(obstacleState==HIGH?F("H"):F("L"));
  lcd.setCursor(12,1);
  int intAngle = (int)getCurrentHeading();
  if (intAngle < 100) lcd.print(F("0"));
  if (intAngle < 10) lcd.print(F("0"));
  lcd.print(intAngle);
}

void showStatusMessage(const char* message) {
  lcd.clear();
  lcd.print(message);
  lcd.setCursor(0,1);
  lcd.print(F("D:"));
  lcd.print(dealtCards);
  lcd.print(F("/"));
  lcd.print(totalCards);
  delay(400);
}

// ==================== SETUP函数 ====================
void setup() {
  delay(500);
  pinMode(MOTOR_A_IN1, OUTPUT);
  pinMode(MOTOR_A_IN2, OUTPUT);
  pinMode(MOTOR_A_PWM, OUTPUT);
  pinMode(MOTOR_B_IN1, OUTPUT);
  pinMode(MOTOR_B_IN2, OUTPUT);
  pinMode(MOTOR_B_PWM, OUTPUT);
  pinMode(MOTOR_STBY, OUTPUT);
  pinMode(OBSTACLE_PIN, INPUT_PULLUP);
  disableMotorDriver();
  lcd.begin(16,2);
  lcd.clear();
  lcd.print(F("Dealer v28.0"));
  lcd.setCursor(0,1);
  lcd.print(F("Manual Cal"));
  #if DEBUG
  Serial.begin(9600);
  delay(500);
  Serial.println(F("=== System Startup ==="));
  Serial.println(F("Manual Calibration"));
  Serial.print(F("Init mode: "));
  Serial.println(runtimeTurnaroundMode==1?F("Compass"):F("Timeout"));
  Serial.print(F("Circle: "));
  Serial.print(MANUAL_CIRCLE_TIME);
  Serial.println(F("ms"));
  Serial.println(F("Use 'H' help"));
  #endif
  #if ENABLE_INFRA
  IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK);
  #endif
  Wire.begin();
  delay(100);
  compassInitialized = false;
  for (int attempt = 1; attempt <= 2; attempt++) {
    lcd.clear();
    lcd.print(F("Init Comp"));
    lcd.setCursor(0,1);
    lcd.print(F("Try "));
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
      lcd.print(F("Cal Fail"));
      lcd.setCursor(0,1);
      lcd.print(F("Use Timeout"));
      delay(1000);
    }
  } else {
    lcd.clear();
    lcd.print(F("No Compass"));
    lcd.setCursor(0,1);
    lcd.print(F("Timeout Mode"));
    delay(1000);
    initialHeading = 0; currentHeading = 0; virtualHeading = 0; targetHeading = 0;
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
  Serial.println(compassInitialized?F("YES"):F("NO"));
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
  if (millis() - lastDebugTime > 3000) {
    Serial.print(F("Cards: "));
    Serial.print(dealtCards);
    Serial.print(F("/"));
    Serial.print(totalCards);
    Serial.print(F(" Mode: "));
    Serial.print(runtimeTurnaroundMode==1?F("Compass"):F("Timeout"));
    if (compassInitialized) {
      Serial.print(F(" Hdg:"));
      Serial.print(getCurrentHeading(),1);
    }
    Serial.println();
    lastDebugTime = millis();
  }
  #endif
  delay(10);
}