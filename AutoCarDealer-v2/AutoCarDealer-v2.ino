// ==================== 系统变量 ====================
float initialHeading = 0;     // 校准时的初始航向角度，作为系统参考零点
float targetHeading = 0;      // 电机A需要旋转到的目标角度
float currentHeading = 0;     // 当前电子罗盘的实时航向角度
float angleTolerance = 10.0;  // 角度容差范围10°
float anglePerPlayer = 90.0;  // 每个玩家之间的角度间隔，初始为4个玩家(360°/4=90°)

#include <LiquidCrystal.h>
#include <Wire.h>
#include "QMC5883L.h"  // 使用您的库文件

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

// ==================== 模块初始化 ====================
LiquidCrystal lcd(12, 11, 5, 4, 3, 2);
QMC5883L compass;

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
unsigned long TIME_A_CIRCLE = 12000;  // 电机A旋转一圈的时间（毫秒），可调节
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

// ==================== 新增：QMC5883L硬件复位函数 ====================
void hardwareResetQMC5883L() {
  #if DEBUG
  Serial.println(F("Performing hardware reset of QMC5883L..."));
  #endif
  
  // QMC5883L软复位：向控制寄存器2（0x0B）写入0x80
  Wire.beginTransmission(0x0D);
  Wire.write(0x0B);  // 控制寄存器2地址
  Wire.write(0x80);  // 软复位命令
  Wire.endTransmission();
  
  delay(50);  // 等待复位完成
  
  // 再次确认复位
  Wire.beginTransmission(0x0D);
  Wire.write(0x0B);
  Wire.write(0x01);  // 设置为连续测量模式
  Wire.endTransmission();
  
  delay(100);  // 等待传感器稳定
}

// ==================== 改进的I2C初始化 ====================
void resetI2C() {
  // 重置I2C总线
  Wire.end();
  delay(50);
  pinMode(SDA, INPUT_PULLUP);
  pinMode(SCL, INPUT_PULLUP);
  delay(50);
  Wire.begin();
  delay(200);  // 延长等待时间
  
  #if DEBUG
  Serial.println(F("I2C bus reset complete"));
  #endif
}

// ==================== 电机控制函数 ====================
void stopAllMotors() {
  digitalWrite(MOTOR_A_IA, LOW);
  digitalWrite(MOTOR_A_IB, LOW);
  digitalWrite(MOTOR_B_IA, LOW);
  digitalWrite(MOTOR_B_IB, LOW);
}

// ==================== 电机A控制 - 永远逆时针 ====================
void controlMotorA(bool enable) {
  if (enable) {
    // 电机A永远逆时针旋转
    digitalWrite(MOTOR_A_IA, LOW);
    analogWrite(MOTOR_A_IB, 220);  // 较高速度确保旋转角度
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

float getSimpleAngleDiff(float current, float target) {
  // 计算最短路径角度差
  current = normalizeAngle(current);
  target = normalizeAngle(target);
  
  float diff = target - current;
  if (diff > 180.0) {
    diff -= 360.0;
  } else if (diff < -180.0) {
    diff += 360.0;
  }
  
  return diff;
}

// ==================== 核心改进：计算超时时间 ====================
void calculateMotorATimeout() {
  // 计算每个玩家角度的超时时间 = 一圈时间 / 玩家数
  motorATimeoutPerPlayer = TIME_A_CIRCLE / playerCount;
  
  // 确保最小超时时间
  if (motorATimeoutPerPlayer < 1000) {
    motorATimeoutPerPlayer = 1000;
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

// ==================== 简化的罗盘读取函数 ====================
bool readCompassHeading(float &heading) {
  if (!compassInitialized) {
    return false;
  }
  
  // 使用库的ready()函数检查数据就绪
  if (!compass.ready()) {
    // 等待一小段时间（最多50ms）
    for (int i = 0; i < 5; i++) {
      delay(10);
      if (compass.ready()) break;
    }
    
    if (!compass.ready()) {
      return false;
    }
  }
  
  // 读取航向（库会自动进行校准）
  int rawHeading = compass.readHeading();
  
  if (rawHeading >= 0 && rawHeading <= 360) {
    // 应用磁偏角修正
    heading = normalizeAngle(rawHeading + MAGNETIC_DECLINATION);
    return true;
  }
  
  return false;
}

// ==================== 改进的罗盘更新函数 ====================
void updateCompassHeading() {
  if (!compassInitialized) return;
  
  // 控制更新频率
  if (millis() - lastCompassUpdate < COMPASS_UPDATE_INTERVAL) {
    return;
  }
  
  float newHeading;
  if (readCompassHeading(newHeading)) {
    // 检查罗盘是否响应
    float change = fabs(newHeading - lastValidHeading);
    
    // 如果角度变化大于阈值，认为罗盘在响应
    if (change > 0.5) {
      compassResponding = true;
      noChangeCount = 0;
      lastHeadingChangeTime = millis();
    } else {
      noChangeCount++;
      // 如果连续多次无变化，认为罗盘可能卡住
      if (noChangeCount > MAX_NO_CHANGE) {
        compassResponding = false;
        #if DEBUG
        if (noChangeCount == MAX_NO_CHANGE + 1) {  // 只报告一次
          Serial.print(F("Compass possibly stuck at: "));
          Serial.println(lastValidHeading, 1);
        }
        #endif
      }
    }
    
    // 更新当前航向
    currentHeading = newHeading;
    lastValidHeading = newHeading;
    
    // 更新滤波数组
    headingSamples[sampleIndex] = newHeading;
    sampleIndex = (sampleIndex + 1) % 3;
    
    if (!samplesReady && sampleIndex == 0) {
      samplesReady = true;
    }
    
    // 计算滤波后的航向
    if (samplesReady) {
      float sum = 0;
      for (int i = 0; i < 3; i++) {
        sum += headingSamples[i];
      }
      filteredHeading = normalizeAngle(sum / 3.0);
    } else {
      filteredHeading = newHeading;
    }
    
    // 更新虚拟航向作为备份
    virtualHeading = filteredHeading;
  } else {
    // 读取失败
    #if DEBUG
    static unsigned long lastErrorTime = 0;
    if (millis() - lastErrorTime > 2000) {
      Serial.println(F("Warning: Failed to read compass heading"));
      lastErrorTime = millis();
    }
    #endif
  }
  
  lastCompassUpdate = millis();
}

// ==================== 获取当前航向（优先使用罗盘） ====================
float getCurrentHeading() {
  if (compassInitialized && compassResponding) {
    return filteredHeading;
  } else {
    // 罗盘不响应时，使用虚拟航向
    return virtualHeading;
  }
}

// ==================== 新增：罗盘恢复函数 ====================
void recoverCompass() {
  #if DEBUG
  Serial.println(F("Attempting compass recovery..."));
  #endif
  
  lcd.clear();
  lcd.print(F("Recovering"));
  lcd.setCursor(0, 1);
  lcd.print(F("Compass..."));
  
  // 1. 停止所有电机（减少电磁干扰）
  stopAllMotors();
  delay(100);
  
  // 2. 重置I2C总线
  resetI2C();
  
  // 3. 硬件复位传感器
  hardwareResetQMC5883L();
  
  // 4. 重新初始化罗盘
  bool success = initCompass();
  
  if (success) {
    // 5. 重新校准
    calibrateCompass();
    
    #if DEBUG
    Serial.println(F("Compass recovery successful"));
    #endif
    
    lcd.clear();
    lcd.print(F("Recovery OK"));
    delay(1000);
  } else {
    #if DEBUG
    Serial.println(F("Compass recovery failed"));
    #endif
    
    lcd.clear();
    lcd.print(F("Recovery Failed"));
    lcd.setCursor(0, 1);
    lcd.print(F("Using Timeout"));
    delay(1000);
  }
}

// ==================== 完全重构的罗盘初始化 ====================
bool initCompass() {
  #if DEBUG
  Serial.println(F("=== COMPASS INITIALIZATION ==="));
  Serial.println(F("Using QMC5883L library with auto-calibration"));
  #endif
  
  lcd.clear();
  lcd.print(F("Init Compass..."));
  
  // 1. 重置I2C总线 - 确保从干净状态开始
  resetI2C();
  
  // 2. 尝试硬件复位QMC5883L（针对软复位后的问题）
  hardwareResetQMC5883L();
  
  // 3. 检查I2C设备是否存在
  #if DEBUG
  Serial.println(F("1. Checking I2C device..."));
  #endif
  
  Wire.beginTransmission(0x0D);
  byte error = Wire.endTransmission();
  
  if (error != 0) {
    #if DEBUG
    Serial.print(F("I2C device not found at 0x0D (error: "));
    Serial.print(error);
    Serial.println(F(")"));
    Serial.println(F("Trying alternative addresses..."));
    #endif
    
    // 尝试其他可能的地址
    byte addresses[] = {0x0C, 0x1E, 0x1F};
    bool found = false;
    
    for (byte i = 0; i < sizeof(addresses); i++) {
      Wire.beginTransmission(addresses[i]);
      error = Wire.endTransmission();
      if (error == 0) {
        #if DEBUG
        Serial.print(F("Found device at 0x"));
        Serial.println(addresses[i], HEX);
        #endif
        found = true;
        break;
      }
    }
    
    if (!found) {
      #if DEBUG
      Serial.println(F("No I2C device found!"));
      #endif
      lcd.clear();
      lcd.print(F("No I2C Device"));
      lcd.setCursor(0, 1);
      lcd.print(F("Check Wiring"));
      delay(2000);
      return false;
    }
  } else {
    #if DEBUG
    Serial.println(F("I2C device found at 0x0D"));
    #endif
  }
  
  // 4. 初始化罗盘对象 - 使用新实例避免内存残留问题
  static QMC5883L newCompass;  // 使用静态确保每次都是新实例
  compass = newCompass;        // 赋值给全局变量
  
  #if DEBUG
  Serial.println(F("2. Initializing QMC5883L..."));
  #endif
  
  // 重要：先调用init()，它会调用reset()进行默认配置
  compass.init();
  
  // 5. 配置罗盘参数（使用库函数）
  #if DEBUG
  Serial.println(F("3. Configuring compass parameters..."));
  #endif
  
  // 设置50Hz采样率
  compass.setSamplingRate(50);
  
  // 设置2高斯量程
  compass.setRange(2);
  
  // 设置128次过采样
  compass.setOversampling(128);
  
  // 6. 重置校准数据（重要！）
  #if DEBUG
  Serial.println(F("4. Resetting calibration..."));
  #endif
  compass.resetCalibration();
  
  delay(100);  // 等待配置生效
  
  // 7. 测试罗盘就绪状态 - 改进的重试机制
  #if DEBUG
  Serial.println(F("5. Testing sensor readiness..."));
  #endif
  
  // 增加重试次数和延迟
  int readyAttempts = 0;
  bool sensorReady = false;
  
  while (readyAttempts < 30 && !sensorReady) {
    readyAttempts++;
    
    // 尝试读取状态寄存器
    if (compass.ready()) {
      sensorReady = true;
    } else {
      // 每5次尝试后重置I2C
      if (readyAttempts % 5 == 0) {
        #if DEBUG
        Serial.print(F("Attempt "));
        Serial.print(readyAttempts);
        Serial.println(F(", resetting I2C..."));
        #endif
        resetI2C();
      }
      delay(100);  // 增加延迟
    }
  }
  
  #if DEBUG
  Serial.print(F("Ready check attempts: "));
  Serial.println(readyAttempts);
  Serial.print(F("Sensor ready: "));
  Serial.println(sensorReady ? "YES" : "NO");
  #endif
  
  if (!sensorReady) {
    #if DEBUG
    Serial.println(F("Sensor not ready after 30 attempts, trying emergency reset..."));
    #endif
    
    // 紧急重置：完全重新初始化
    resetI2C();
    hardwareResetQMC5883L();
    delay(500);
    
    // 再次检查
    readyAttempts = 0;
    while (readyAttempts < 10 && !sensorReady) {
      readyAttempts++;
      if (compass.ready()) {
        sensorReady = true;
      }
      delay(100);
    }
    
    if (!sensorReady) {
      #if DEBUG
      Serial.println(F("Sensor still not ready after emergency reset"));
      #endif
      lcd.clear();
      lcd.print(F("Sensor Not Ready"));
      lcd.setCursor(0, 1);
      lcd.print(F("Try Power Cycle"));
      delay(2000);
      return false;
    }
  }
  
  // 8. 读取原始数据验证传感器工作 - 改进验证
  #if DEBUG
  Serial.println(F("6. Testing raw data readings..."));
  #endif
  
  int16_t x, y, z, t;
  int validRawReadings = 0;
  bool dataValid = false;
  
  for (int i = 0; i < 20; i++) {  // 增加尝试次数
    if (compass.ready()) {
      if (compass.readRaw(&x, &y, &z, &t)) {
        validRawReadings++;
        
        // 检查数据是否合理（排除全0或异常值）
        if (abs(x) > 10 || abs(y) > 10 || abs(z) > 10) {
          dataValid = true;
        }
        
        #if DEBUG
        if (i == 0) {
          Serial.print(F("First raw reading: X="));
          Serial.print(x);
          Serial.print(F(" Y="));
          Serial.print(y);
          Serial.print(F(" Z="));
          Serial.print(z);
          Serial.print(F(" T="));
          Serial.println(t);
        }
        #endif
      }
    }
    delay(100);
  }
  
  #if DEBUG
  Serial.print(F("Valid raw readings: "));
  Serial.print(validRawReadings);
  Serial.println(F("/20"));
  #endif
  
  if (!dataValid) {
    #if DEBUG
    Serial.println(F("Data looks invalid (possible sensor lockup)"));
    #endif
    lcd.clear();
    lcd.print(F("Invalid Data"));
    lcd.setCursor(0, 1);
    lcd.print(F("Move Sensor"));
    delay(2000);
  }
  
  // 9. 读取航向测试 - 增加采样数量
  #if DEBUG
  Serial.println(F("7. Testing heading readings..."));
  #endif
  
  int validHeadingReadings = 0;
  float sumHeading = 0;
  
  for (int i = 0; i < 50; i++) {  // 增加样本数
    if (compass.ready()) {
      int heading = compass.readHeading();
      
      if (heading >= 0 && heading <= 360) {
        validHeadingReadings++;
        sumHeading += heading;
        
        #if DEBUG
        if (validHeadingReadings == 1) {
          Serial.print(F("First valid heading: "));
          Serial.println(heading);
        }
        #endif
      }
    }
    delay(50);  // 缩短延迟以加快测试
  }
  
  #if DEBUG
  Serial.print(F("Valid heading readings: "));
  Serial.print(validHeadingReadings);
  Serial.println(F("/50"));
  #endif
  
  if (validHeadingReadings < 15) {  // 降低阈值
    #if DEBUG
    Serial.println(F("Insufficient valid heading readings"));
    #endif
    lcd.clear();
    lcd.print(F("Heading Data Low"));
    lcd.setCursor(0, 1);
    lcd.print(F("Move for Cal"));
    delay(2000);
  }
  
  // 10. 计算平均航向并设置初始值
  float avgHeading = 0;
  if (validHeadingReadings > 0) {
    avgHeading = sumHeading / validHeadingReadings;
  }
  
  float initialReading = normalizeAngle(avgHeading + MAGNETIC_DECLINATION);
  
  // 初始化滤波数组
  for (int i = 0; i < 3; i++) {
    headingSamples[i] = initialReading;
  }
  
  filteredHeading = initialReading;
  virtualHeading = initialReading;
  lastValidHeading = initialReading;
  samplesReady = (validHeadingReadings >= 3);
  
  // 11. 设置系统状态
  compassInitialized = sensorReady && dataValid;
  calibrationDone = false;  // 需要用户校准
  compassResponding = true;
  noChangeCount = 0;
  
  #if DEBUG
  if (compassInitialized) {
    Serial.print(F("Compass initialized successfully! Average heading: "));
    Serial.print(avgHeading, 1);
    Serial.print(F("°, With declination: "));
    Serial.print(initialReading, 1);
    Serial.print(F("°, Valid readings: "));
    Serial.println(validHeadingReadings);
    Serial.println(F("=== INITIALIZATION COMPLETE ==="));
  } else {
    Serial.println(F("Compass initialization partially successful - limited functionality"));
  }
  #endif
  
  if (compassInitialized) {
    lcd.clear();
    lcd.print(F("Compass OK"));
    lcd.setCursor(0, 1);
    lcd.print(F("Ready to Calibrate"));
  } else {
    lcd.clear();
    lcd.print(F("Compass Limited"));
    lcd.setCursor(0, 1);
    lcd.print(F("Timeout Mode Only"));
  }
  
  delay(1000);
  
  return compassInitialized;
}

// ==================== I2C扫描函数 ====================
void scanI2C() {
  Serial.println(F("=== I2C SCAN ==="));
  
  byte error, address;
  int nDevices = 0;
  
  for(address = 1; address < 127; address++ ) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    
    if (error == 0) {
      Serial.print(F("I2C device found at address 0x"));
      if (address < 16) Serial.print("0");
      Serial.print(address, HEX);
      
      // 已知设备
      if (address == 0x0D) Serial.print(F(" (QMC5883L)"));
      else if (address == 0x1E || address == 0x1F) Serial.print(F(" (HMC5883L)"));
      else if (address == 0x68) Serial.print(F(" (MPU6050)"));
      else if (address == 0x27 || address == 0x3F) Serial.print(F(" (LCD)"));
      else Serial.print(F(" (Unknown)"));
      
      Serial.println();
      nDevices++;
    }
  }
  
  if (nDevices == 0) {
    Serial.println(F("No I2C devices found"));
  } else {
    Serial.print(F("Found "));
    Serial.print(nDevices);
    Serial.println(F(" device(s)"));
  }
  Serial.println(F("=== END SCAN ==="));
}

// ==================== 改进的校准函数 ====================
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
  lcd.print(F("Rotate 360°"));
  
  #if DEBUG
  Serial.println(F("Starting compass calibration..."));
  #endif
  
  // 重置校准数据
  compass.resetCalibration();
  
  unsigned long startTime = millis();
  int samples = 0;
  float headingSum = 0;
  
  // 收集数据用于校准
  while (millis() - startTime < 5000) {
    if (compass.ready()) {
      int rawHeading = compass.readHeading();
      
      if (rawHeading >= 0 && rawHeading <= 360) {
        samples++;
        float heading = normalizeAngle(rawHeading + MAGNETIC_DECLINATION);
        headingSum += heading;
        
        #if DEBUG
        if (samples % 20 == 0) {
          Serial.print(F("Calibration sample "));
          Serial.print(samples);
          Serial.print(F(": "));
          Serial.println(heading, 1);
        }
        #endif
        
        // 更新LCD显示
        if (samples % 10 == 0) {
          lcd.setCursor(0, 1);
          lcd.print(F("Samples: "));
          lcd.print(samples);
          lcd.print(F("   "));
        }
      }
    }
    delay(50);
  }
  
  if (samples >= 20) {
    float avgHeading = headingSum / samples;
    
    // 设置初始航向
    initialHeading = normalizeAngle(avgHeading);
    currentHeading = initialHeading;
    targetHeading = initialHeading;
    filteredHeading = initialHeading;
    virtualHeading = initialHeading;
    lastValidHeading = initialHeading;
    calibrationDone = true;
    
    // 重置滤波数组
    for (int i = 0; i < 3; i++) {
      headingSamples[i] = initialHeading;
    }
    
    #if DEBUG
    Serial.print(F("Calibration complete! Samples: "));
    Serial.println(samples);
    Serial.print(F("Average heading: "));
    Serial.print(avgHeading, 1);
    Serial.println(F("°"));
    Serial.print(F("Initial heading set to: "));
    Serial.println(initialHeading, 1);
    #endif
    
    lcd.clear();
    lcd.print(F("Calibration OK"));
    lcd.setCursor(0, 1);
    lcd.print(F("H:"));
    lcd.print((int)initialHeading);
    lcd.print(F(" S:"));
    lcd.print(samples);
  } else {
    calibrationDone = false;
    
    #if DEBUG
    Serial.print(F("Calibration failed - only "));
    Serial.print(samples);
    Serial.println(F(" samples collected"));
    #endif
    
    lcd.clear();
    lcd.print(F("Calibration FAIL"));
    lcd.setCursor(0, 1);
    lcd.print(F("Need more samples"));
  }
  
  delay(1000);
}

// ==================== 关键改进：双模式旋转控制 ====================
bool rotateToAngle() {
  // 限制电机控制更新频率
  if (millis() - lastMotorUpdate < MOTOR_CONTROL_INTERVAL) {
    return false;
  }
  
  lastMotorUpdate = millis();
  
  // 获取当前航向
  float current = getCurrentHeading();
  
  // 计算角度差
  float angleDiff = getSimpleAngleDiff(current, targetHeading);
  
  // 计算已旋转的时间
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
    
    // 显示当前旋转模式
    if (compassInitialized && compassResponding && calibrationDone) {
      Serial.print(F(", Mode: Compass"));
    } else {
      Serial.print(F(", Mode: Timeout"));
    }
    
    Serial.println();
    lastRotateDebug = millis();
  }
  #endif
  
  // 模式1：罗盘模式（罗盘初始化成功、响应且已校准）
  if (compassInitialized && compassResponding && calibrationDone) {
    // 条件1：角度差小于容差
    if (fabs(angleDiff) <= angleTolerance) {
      stopAllMotors();
      
      #if DEBUG
      Serial.print(F("✓ Target reached! Angle diff="));
      Serial.print(fabs(angleDiff), 1);
      Serial.println(F("°"));
      #endif
      
      return true;
    }
    
    // 继续逆时针旋转
    controlMotorA(true);
    return false;
  } 
  // 模式2：超时模式（罗盘不可用）
  else {
    // 条件2：超时
    if (elapsed >= motorATimeoutPerPlayer) {
      stopAllMotors();
      
      #if DEBUG
      Serial.print(F("⏱ Rotation timeout after "));
      Serial.print(elapsed);
      Serial.print(F("ms (expected: "));
      Serial.print(motorATimeoutPerPlayer);
      Serial.println(F("ms)"));
      
      // 超时后，强制更新虚拟航向为目标角度
      virtualHeading = targetHeading;
      currentHeading = targetHeading;
      #endif
      
      return true;
    }
    
    // 继续逆时针旋转
    controlMotorA(true);
    return false;
  }
}

// ==================== 串口输入处理 ====================
void processSerialInput() {
  #if ENABLE_KEYBOARD
  while (Serial.available() > 0) {
    char c = Serial.read();
    
    // 如果收到换行符或超时，处理完整命令
    if (c == '\n' || c == '\r' || (millis() - lastSerialCharTime > SERIAL_TIMEOUT && serialBufferIndex > 0)) {
      if (serialBufferIndex > 0) {
        serialBuffer[serialBufferIndex] = '\0';  // 终止字符串
        
        #if DEBUG
        Serial.print(F("Received command: "));
        Serial.println(serialBuffer);
        #endif
        
        // 处理命令
        handleSerialCommand(serialBuffer);
        
        // 清空缓冲区
        serialBufferIndex = 0;
        serialBuffer[0] = '\0';
      }
    } 
    // 否则，将字符添加到缓冲区
    else if (serialBufferIndex < SERIAL_BUFFER_SIZE - 1 && c >= 32) {  // 只接受可打印字符
      serialBuffer[serialBufferIndex++] = c;
      lastSerialCharTime = millis();
    }
  }
  #endif
}

// ==================== 新增：定期罗盘健康检查 ====================
void checkCompassHealth() {
  static unsigned long lastHealthCheck = 0;
  static int consecutiveFailures = 0;
  
  // 每30秒检查一次
  if (millis() - lastHealthCheck > 30000) {
    lastHealthCheck = millis();
    
    if (compassInitialized) {
      float testHeading;
      bool readingOK = readCompassHeading(testHeading);
      
      if (!readingOK) {
        consecutiveFailures++;
        #if DEBUG
        Serial.print(F("Compass health check failed #"));
        Serial.println(consecutiveFailures);
        #endif
        
        if (consecutiveFailures >= 3) {
          // 连续3次失败，尝试恢复
          recoverCompass();
          consecutiveFailures = 0;
        }
      } else {
        consecutiveFailures = 0;  // 重置失败计数
        
        // 检查数据是否合理
        float change = fabs(testHeading - lastValidHeading);
        if (change > 0.5) {
          // 数据变化正常
          lastHeadingChangeTime = millis();
          compassResponding = true;
        }
      }
    }
  }
}

// ==================== 串口命令处理函数 ====================
void handleSerialCommand(const char* command) {
  #if ENABLE_KEYBOARD
  // 显示命令
  lcd.setCursor(14, 1);
  if (command[0] < 0x10) lcd.print(F("0"));
  lcd.print(command[0], HEX);
  
  // 根据命令字符执行相应操作
  switch(command[0]) {
    case 'v':  // 版本显示
    case 'V':
      lcd.clear();
      lcd.print(F("Card Dealer v24.1"));
      lcd.setCursor(0, 1);
      lcd.print(F("Dual Mode Control"));
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
        float testTarget = normalizeAngle(startAngle - 90);
        
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
        Serial.print(F("Timeout: "));
        Serial.print(motorATimeoutPerPlayer);
        Serial.println(F("ms"));
        #endif
        
        // 开始旋转
        rotationStartHeading = startAngle;
        aMotorTimeoutStart = millis();
        targetHeading = testTarget;
        controlMotorA(true);
        
        // 等待旋转完成
        unsigned long testStart = millis();
        bool testComplete = false;
        
        while (!testComplete && millis() - testStart < 10000) {
          // 使用相同的旋转逻辑
          float current = getCurrentHeading();
          float diff = getSimpleAngleDiff(current, testTarget);
          
          bool angleCondition = (fabs(diff) <= angleTolerance);
          bool timeoutCondition = (millis() - aMotorTimeoutStart >= motorATimeoutPerPlayer);
          
          if (angleCondition || timeoutCondition) {
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
            
            if (timeoutCondition) {
              Serial.println(F("Stopped by timeout"));
            } else {
              Serial.println(F("Stopped by angle condition"));
            }
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
      
    case 'm':  // 显示模式信息
    case 'M':
      lcd.clear();
      lcd.print(F("Dual Mode Control"));
      lcd.setCursor(0, 1);
      if (compassInitialized && compassResponding && calibrationDone) {
        lcd.print(F("Mode: Compass"));
      } else {
        lcd.print(F("Mode: Timeout"));
      }
      delay(1000);
      updateDisplay();
      break;
      
    case 'i':  // 调整电机A一圈时间
    case 'I':
      if (!isRunning) {
        TIME_A_CIRCLE += 500;
        if (TIME_A_CIRCLE > 12000) TIME_A_CIRCLE = 3000;
        calculateMotorATimeout();
        lcd.clear();
        lcd.print(F("Circle Time:"));
        lcd.setCursor(0, 1);
        lcd.print(TIME_A_CIRCLE);
        lcd.print(F("ms"));
        delay(1000);
        updateDisplay();
      }
      break;
      
    case 'u':  // 显示系统状态
    case 'U':
      lcd.clear();
      lcd.print(F("Dual Mode Control"));
      lcd.setCursor(0, 1);
      lcd.print(F("T:"));
      lcd.print(motorATimeoutPerPlayer);
      lcd.print(F("ms P:"));
      lcd.print(playerCount);
      delay(2000);
      updateDisplay();
      break;
      
    case 'g':  // 简单测试电机A
    case 'G':
      if (!isRunning) {
        #if DEBUG
        Serial.println(F("Testing motor A rotation..."));
        Serial.print(F("Timeout set to: "));
        Serial.print(motorATimeoutPerPlayer);
        Serial.println(F("ms"));
        #endif
        
        lcd.clear();
        lcd.print(F("Motor A Test"));
        lcd.setCursor(0, 1);
        lcd.print(F("Timeout: "));
        lcd.print(motorATimeoutPerPlayer);
        lcd.print(F("ms"));
        
        controlMotorA(true);
        delay(motorATimeoutPerPlayer);
        stopAllMotors();
        
        #if DEBUG
        Serial.println(F("Motor test complete"));
        #endif
        
        updateDisplay();
      }
      break;
      
    case 'y':  // 显示超时信息
    case 'Y':
      lcd.clear();
      lcd.print(F("Per Player: "));
      lcd.print(motorATimeoutPerPlayer);
      lcd.print(F("ms"));
      lcd.setCursor(0, 1);
      lcd.print(F("Circle: "));
      lcd.print(TIME_A_CIRCLE);
      lcd.print(F("ms"));
      delay(2000);
      updateDisplay();
      break;
      
    case 'x':  // I2C扫描
    case 'X':
      if (!isRunning) {
        scanI2C();
        updateDisplay();
      }
      break;
      
    case 'w':  // 测试罗盘原始数据
    case 'W':
      if (!isRunning && compassInitialized) {
        lcd.clear();
        lcd.print(F("Testing Raw Data"));
        
        int16_t rawX, rawY, rawZ, rawT;
        int successCount = 0;
        
        for (int i = 0; i < 10; i++) {
          if (compass.ready()) {
            if (compass.readRaw(&rawX, &rawY, &rawZ, &rawT)) {
              successCount++;
              
              Serial.print(F("Raw["));
              Serial.print(i);
              Serial.print(F("]: X="));
              Serial.print(rawX);
              Serial.print(F(" Y="));
              Serial.print(rawY);
              Serial.print(F(" Z="));
              Serial.print(rawZ);
              Serial.print(F(" T="));
              Serial.println(rawT);
              
              lcd.setCursor(0, 1);
              lcd.print(F("X:"));
              lcd.print(rawX);
              lcd.print(F(" Y:"));
              lcd.print(rawY);
            }
          }
          delay(200);
        }
        
        Serial.print(F("Success rate: "));
        Serial.print(successCount);
        Serial.println(F("/10"));
        
        delay(2000);
        updateDisplay();
      }
      break;
      
    case 'o':  // 优化版重新初始化罗盘
    case 'O':
      if (!isRunning) {
        lcd.clear();
        lcd.print(F("Reinit Compass"));
        lcd.setCursor(0, 1);
        lcd.print(F("Optimized..."));
        
        // 完全重置
        compassInitialized = false;
        calibrationDone = false;
        
        // 硬件复位
        hardwareResetQMC5883L();
        delay(200);
        
        // 重新初始化
        bool success = initCompass();
        
        if (success) {
          lcd.clear();
          lcd.print(F("Reinit OK"));
        } else {
          lcd.clear();
          lcd.print(F("Reinit FAIL"));
          lcd.setCursor(0, 1);
          lcd.print(F("Try Power Cycle"));
        }
        
        delay(1000);
        updateDisplay();
      }
      break;
      
    case 'z':  // 重启系统
    case 'Z':
      lcd.clear();
      lcd.print(F("System Reset"));
      lcd.setCursor(0, 1);
      lcd.print(F("Please wait..."));
      
      #if DEBUG
      Serial.println(F("System resetting..."));
      #endif
      
      // 在软复位前重置罗盘传感器
      if (compassInitialized) {
        hardwareResetQMC5883L();
        delay(100);
      }
      
      delay(1000);
      
      // 软件重启
      asm volatile ("jmp 0");
      break;
      
    case 'h':  // 显示帮助信息
    case 'H':
      Serial.println(F("=== Card Dealer Commands v24.1 ==="));
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
      Serial.println(F("M - Display mode info"));
      Serial.println(F("I - Adjust motor circle time"));
      Serial.println(F("U - Display system status"));
      Serial.println(F("G - Simple motor A test"));
      Serial.println(F("Y - Display timeout info"));
      Serial.println(F("X - I2C scan"));
      Serial.println(F("W - Test compass raw data"));
      Serial.println(F("O - Reinitialize compass"));
      Serial.println(F("Z - System reset"));
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

// ==================== SETUP函数 ====================
void setup() {
  // 增加启动延迟，确保系统稳定
  delay(500);
  
  // 初始化引脚
  pinMode(MOTOR_A_IA, OUTPUT);
  pinMode(MOTOR_A_IB, OUTPUT);
  pinMode(MOTOR_B_IA, OUTPUT);
  pinMode(MOTOR_B_IB, OUTPUT);
  pinMode(OBSTACLE_PIN, INPUT_PULLUP);
  
  stopAllMotors();
  
  // 初始化LCD
  lcd.begin(16, 2);
  lcd.clear();
  lcd.print(F("Card Dealer v24.1"));
  lcd.setCursor(0, 1);
  lcd.print(F("Dual Mode Control"));
  
  // 初始化串口
  #if DEBUG
  Serial.begin(9600);
  delay(500);
  Serial.println(F("System Startup v24.1"));
  Serial.println(F("=========================="));
  Serial.println(F("MOTOR A: CCW ONLY MODE"));
  Serial.print(F("Magnetic Declination: "));
  Serial.print(MAGNETIC_DECLINATION);
  Serial.println(F("° (Shanghai)"));
  Serial.println(F("DUAL MODE CONTROL:"));
  Serial.println(F("1. Compass mode (if available)"));
  Serial.println(F("2. Timeout mode (fallback)"));
  
  #if ENABLE_INFRA
  Serial.println(F("Infrared input: ENABLED"));
  #else
  Serial.println(F("Infrared input: DISABLED"));
  #endif
  
  #if ENABLE_KEYBOARD
  Serial.println(F("Keyboard input: ENABLED"));
  Serial.println(F("Type 'H' for help"));
  #else
  Serial.println(F("Keyboard input: DISABLED"));
  #endif
  #endif
  
  // 初始化红外接收
  #if ENABLE_INFRA
  IrReceiver.begin(RECV_PIN, DISABLE_LED_FEEDBACK);
  #endif
  
  // 初始化罗盘
  lcd.clear();
  lcd.print(F("Init Compass..."));
  
  // 初始化I2C总线
  Wire.begin();
  delay(100);
  
  // 尝试初始化罗盘
  compassInitialized = initCompass();
  
  if (compassInitialized) {
    #if DEBUG
    Serial.println(F("Compass initialized successfully"));
    #endif
    
    // 校准罗盘
    calibrateCompass();
    
    if (!calibrationDone) {
      #if DEBUG
      Serial.println(F("Calibration failed, using timeout mode"));
      #endif
      lcd.clear();
      lcd.print(F("Cal FAIL"));
      lcd.setCursor(0, 1);
      lcd.print(F("Using Timeout"));
      delay(1000);
    }
  } else {
    #if DEBUG
    Serial.println(F("Compass initialization failed"));
    Serial.println(F("System will use timeout mode only"));
    #endif
    
    lcd.clear();
    lcd.print(F("No Compass"));
    lcd.setCursor(0, 1);
    lcd.print(F("Timeout Mode"));
    delay(1000);
    
    // 设置初始值
    initialHeading = 0;
    currentHeading = 0;
    targetHeading = 0;
    virtualHeading = 0;
    filteredHeading = 0;
    calibrationDone = false;
  }
  
  // 计算总牌数
  uint8_t cardsPerDeck = hasJokers ? 54 : 52;
  totalCards = deckCount * cardsPerDeck;
  
  if (remainCards > 0 && totalCards > remainCards) {
    totalCards -= remainCards;
  }
  
  // 计算角度间隔和超时时间
  anglePerPlayer = 360.0 / playerCount;
  calculateMotorATimeout();
  
  // 读取初始障碍物状态
  obstacleState = digitalRead(OBSTACLE_PIN);
  lastObstacleState = obstacleState;
  
  // 初始化变量
  lastHeadingChangeTime = millis();
  compassResponding = true;
  noChangeCount = 0;
  
  // 初始化串口缓冲区
  serialBufferIndex = 0;
  serialBuffer[0] = '\0';
  lastSerialCharTime = millis();
  
  // 更新显示
  updateDisplay();
  
  #if DEBUG
  Serial.print(F("Angle per player: "));
  Serial.println(anglePerPlayer, 1);
  Serial.print(F("Motor A timeout per step: "));
  Serial.print(motorATimeoutPerPlayer);
  Serial.println(F(" ms"));
  Serial.print(F("Time per circle: "));
  Serial.print(TIME_A_CIRCLE);
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
  Serial.print(F("Angle tolerance: "));
  Serial.print(angleTolerance, 1);
  Serial.println(F("°"));
  Serial.println(F("=========================="));
  #endif
}

// ==================== LOOP函数 ====================
void loop() {
  // 处理红外输入
  #if ENABLE_INFRA
  if (IrReceiver.decode()) {
    handleIRCommand(IrReceiver.decodedIRData.command);
    IrReceiver.resume();
  }
  #endif
  
  // 处理串口输入
  processSerialInput();
  
  checkObstacle();
  
  if (isRunning) {
    handleMotorState();
  }
  
  // 更新罗盘读数
  updateCompassHeading();
  
  // 定期检查罗盘健康状态
  checkCompassHealth();
  
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
    
    // 显示当前模式
    if (compassInitialized && compassResponding && calibrationDone) {
      Serial.print(F(" [Compass Mode]"));
    } else {
      Serial.print(F(" [Timeout Mode]"));
    }
    
    Serial.println();
    
    lastDebugTime = millis();
  }
  #endif
  
  delay(10);  // 主循环延迟
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

// ==================== 障碍事件处理 ====================
void handleObstacleEvent() {
  lastObstacleTime = millis();
  
  stopAllMotors();
  dealtCards++;
  if (dealtCards > totalCards) dealtCards = totalCards;
  
  #if DEBUG
  Serial.print(F("Obstacle detected, dealt: "));
  Serial.println(dealtCards);
  #endif
  
  // 计算目标角度 - 按照您的要求：initialHeading - i * anglePerPlayer
  uint8_t currentPlayerIndex = dealtCards % playerCount;
  targetHeading = normalizeAngle(initialHeading - (currentPlayerIndex * anglePerPlayer));
  
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
  Serial.println(targetHeading, 1);
  #endif
  
  changeState(STATE_A_RUNNING);
  aMotorTimeoutStart = millis();
  lastMotorUpdate = millis();
  
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
    
    // 获取稳定航向作为初始航向
    initialHeading = getCurrentHeading();
    targetHeading = initialHeading;
    
    // 重置虚拟航向
    virtualHeading = initialHeading;
    
    #if DEBUG
    Serial.print(F("Start dealing, initial heading: "));
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
  
  // 发牌结束后重置罗盘状态
  if (compassInitialized) {
    // 如果不是由罗盘模式完成的，可能需要重置
    if (!compassResponding || !calibrationDone) {
      #if DEBUG
      Serial.println(F("Resetting compass after dealing..."));
      #endif
      recoverCompass();
    }
  }
  
  updateDisplay();
  
  #if DEBUG
  Serial.println(F("Stop dealing"));
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

// ==================== 红外命令处理 ====================
#if ENABLE_INFRA
void handleIRCommand(uint8_t command) {
  lcd.setCursor(14, 1);
  if (command < 0x10) lcd.print(F("0"));
  lcd.print(command, HEX);
  
  switch(command) {
    case 0xDC:  // 版本显示
      lcd.clear();
      lcd.print(F("Card Dealer v24.1"));
      lcd.setCursor(0, 1);
      lcd.print(F("Dual Mode Control"));
      delay(1000);
      updateDisplay();
      break;
      
    case 0x99:  // 增加玩家数量
      playerCount++;
      if (playerCount > 8) playerCount = 2;
      anglePerPlayer = 360.0 / playerCount;
      calculateMotorATimeout();
      updateDisplay();
      break;
      
    case 0xC1:  // 增加牌组数量
      deckCount++;
      if (deckCount > 3) deckCount = 1;
      totalCards = deckCount * (hasJokers ? 54 : 52);
      if (remainCards > 0 && totalCards > remainCards) totalCards -= remainCards;
      updateDisplay();
      break;
      
    case 0xCA:  // 切换是否有王
      hasJokers = !hasJokers;
      totalCards = deckCount * (hasJokers ? 54 : 52);
      if (remainCards > 0 && totalCards > remainCards) totalCards -= remainCards;
      updateDisplay();
      break;
      
    case 0xD2:  // 调整剩余牌数
      remainCards += playerCount;
      if (remainCards > playerCount * 4) remainCards = 0;
      totalCards = deckCount * (hasJokers ? 54 : 52);
      if (remainCards > 0 && totalCards > remainCards) totalCards -= remainCards;
      updateDisplay();
      break;
      
    case 0xCE:  // 开始发牌
      if (!isRunning) {
        startDealing();
      } else {
        showStatusMessage("Already Running");
        delay(500);
        updateDisplay();
      }
      break;
      
    case 0xC5:  // 重置到默认
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
      
    case 0x82:  // 停止发牌
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
      
    case 0x81:  // 测试电机A旋转90度
      if (!isRunning) {
        float startAngle = getCurrentHeading();
        float testTarget = normalizeAngle(startAngle - 90);
        
        lcd.clear();
        lcd.print(F("Test 90° Rotation"));
        lcd.setCursor(0, 1);
        lcd.print(F("Start:"));
        lcd.print((int)startAngle);
        lcd.print(F("->"));
        lcd.print((int)testTarget);
        
        // 开始旋转
        rotationStartHeading = startAngle;
        aMotorTimeoutStart = millis();
        targetHeading = testTarget;
        controlMotorA(true);
        
        // 等待旋转完成
        unsigned long testStart = millis();
        bool testComplete = false;
        
        while (!testComplete && millis() - testStart < 10000) {
          float current = getCurrentHeading();
          float diff = getSimpleAngleDiff(current, testTarget);
          
          bool angleCondition = (fabs(diff) <= angleTolerance);
          bool timeoutCondition = (millis() - aMotorTimeoutStart >= motorATimeoutPerPlayer);
          
          if (angleCondition || timeoutCondition) {
            testComplete = true;
            stopAllMotors();
            
            float endAngle = getCurrentHeading();
            float actualRotation = getSimpleAngleDiff(startAngle, endAngle);
            
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
      
    case 0x98:  // 测试电机B
      if (!isRunning) {
        controlMotorB(1);
        lcd.clear();
        lcd.print(F("Motor B ON Test"));
        delay(1500);
        stopAllMotors();
        updateDisplay();
      }
      break;
      
    case 0x94:  // 校准电子罗盘
      if (!isRunning && compassInitialized) {
        calibrateCompass();
        updateDisplay();
      }
      break;
      
    case 0x90:  // 显示模式信息
      lcd.clear();
      lcd.print(F("Dual Mode Control"));
      lcd.setCursor(0, 1);
      if (compassInitialized && compassResponding && calibrationDone) {
        lcd.print(F("Mode: Compass"));
      } else {
        lcd.print(F("Mode: Timeout"));
      }
      delay(1000);
      updateDisplay();
      break;
      
    case 0x8C:  // 调整电机A一圈时间
      if (!isRunning) {
        TIME_A_CIRCLE += 500;
        if (TIME_A_CIRCLE > 12000) TIME_A_CIRCLE = 3000;
        calculateMotorATimeout();
        lcd.clear();
        lcd.print(F("Circle Time:"));
        lcd.setCursor(0, 1);
        lcd.print(TIME_A_CIRCLE);
        lcd.print(F("ms"));
        delay(1000);
        updateDisplay();
      }
      break;
      
    default:
      lcd.clear();
      lcd.print(F("Unknown Cmd:"));
      lcd.print(command, HEX);
      delay(1000);
      updateDisplay();
      break;
  }
}
#endif