// ==================== 系统配置宏 ====================
#define TURNAROUND_MODE 0          // 默认超时模式 (0=超时, 1=罗盘)
#define ENABLE_INFRA 1            // 启用红外遥控
#define ENABLE_KEYBOARD 0         // 启用串口键盘指令
#define DEBUG 1                  // 调试信息输出
#define IR_DEBUG 0               // 红外详细调试（关闭减少串口干扰）

// ==================== 库包含 ====================
#include <LiquidCrystal.h>
#include <Wire.h>
#include <QMC5883LCompass.h>

#if ENABLE_INFRA
#define EXCLUDE_EXOTIC_PROTOCOLS   // 屏蔽多余红外协议，减小代码
#define NO_LED_FEEDBACK_CODE
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

// ==================== 模块实例化 ====================
LiquidCrystal lcd(12, 11, 5, 4, 3, 2);
QMC5883LCompass compass;

#if ENABLE_INFRA
IRrecv irrecv(IR_RECEIVE_PIN);
decode_results results;
#endif

// ==================== 系统状态枚举 ====================
enum SystemState {
    STATE_IDLE,
    STATE_B_RUNNING,
    STATE_A_RUNNING,
    STATE_B_TIMEOUT
};

// ==================== 全局变量 ====================
// ---- 游戏参数 ----
uint8_t playerCount = 4;
uint8_t deckCount = 3;
uint8_t remainCards = 0;
uint8_t hasJokers = 1;
uint16_t totalCards = 54 * 3;
uint16_t dealtCards = 0;
SystemState currentState = STATE_IDLE;
bool isRunning = 0;

// ---- 电机控制时间参数 ----
unsigned long MANUAL_CIRCLE_TIME = 3050;
unsigned long motorATimeoutPerPlayer = 763;
unsigned long motorBTimeout = 8000;

// ---- 调试定时器 ----
unsigned long lastDebugTime = 0;

// ---- 电机运行状态 ----
unsigned long motorStartTime = 0;
unsigned long aMotorTimeoutStart = 0;
unsigned long lastMotorUpdate = 0;
const int MOTOR_CONTROL_INTERVAL = 20;

// ---- 障碍传感器 ----
unsigned long lastObstacleTime = 0;
unsigned long obstacleDebounce = 0;
uint8_t lastObstacleState = HIGH;
uint8_t obstacleState = HIGH;
bool obstacleActive = false;
bool obstacleTriggered = false;

// ---- 罗盘系统 ----
bool compassInitialized = false;
bool calibrationDone = false;
unsigned long lastCompassUpdate = 0;
const int COMPASS_UPDATE_INTERVAL = 30;
const float MAGNETIC_DECLINATION = 5.0;

float currentHeading = 0.0;
float filteredHeading = 0.0;
float virtualHeading = 0.0;
float lastValidHeading = 0.0;
unsigned long lastHeadingChangeTime = 0;
bool compassResponding = true;
int noChangeCount = 0;
const int MAX_NO_CHANGE = 15;

// 罗盘滤波缓冲区（7点）
const int FILTER_SIZE = 7;
float headingBuffer[FILTER_SIZE];
int bufferIndex = 0;
bool bufferFilled = false;

// ---- 角度控制参数（罗盘模式）----
float targetHeading = 0.0;
float anglePerPlayer = 90.0;
float angleTolerance = 6.0;

// ---- 旋转状态跟踪 ----
float rotationStartHeading = 0.0;
unsigned long rotationStartTime = 0;
bool rotationInProgress = false;

// ---- 模式切换 ----
bool runtimeTurnaroundMode = TURNAROUND_MODE;

// ---- 超时模式动态补偿用基准（仅超时模式）----
float timeoutInitialHeading = 0.0;   // 超时模式下的虚拟基准

// ---- 串口缓冲区 ----
const int SERIAL_BUFFER_SIZE = 32;
char serialBuffer[SERIAL_BUFFER_SIZE];
int serialBufferIndex = 0;
unsigned long lastSerialCharTime = 0;
const unsigned long SERIAL_TIMEOUT = 100;

// ==================== 电机速度参数 ====================
const int MOTOR_A_SPEED = 150;
const int MOTOR_A_CAL_SPEED = 200;
const int MOTOR_B_SPEED = 255;

// ==================== 红外键码定义 ====================
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

// ==================== 函数声明 ====================
float normalizeAngle(float angle);
float getSimpleAngleDiff(float current, float target);
float getCCWDistance(float current, float target);
void enableMotorDriver();
void disableMotorDriver();
void stopAllMotors();
void controlMotorA(bool enable);
void controlMotorAFullSpeed();
void controlMotorB(uint8_t state);
void calculateMotorATimeout();
bool readCompassHeading(float &heading);
float updateVirtualHeadingDuringRotation();
float getCurrentHeading();
void updateCompassHeading();
bool initCompassUltimate();
void i2c_release_bus();          // 新增：释放I2C总线
void calibrateCompass();
void autoCalibrateCircleTime();
bool rotateToAngle();
void handleObstacleEvent();
void changeState(SystemState newState);
void startDealing();
void stopDealing();
void checkObstacle();
void handleMotorState();
void resetSystem();
#if ENABLE_INFRA
void processInfraredInput();
#endif
void processSerialInput();
void handleSerialCommand(const char* command);
void updateDisplay();
void showStatusMessage(const char* message);

// ==================== 角度工具函数 ====================
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

float getCCWDistance(float current, float target) {
    current = normalizeAngle(current);
    target = normalizeAngle(target);
    if (current >= target) {
        return current - target;
    } else {
        return current + 360.0 - target;
    }
}

// ==================== 电机控制函数 ====================
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

// ==================== 超时时间计算 ====================
void calculateMotorATimeout() {
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

// ==================== 罗盘原始读取 ====================
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

// ==================== 虚拟航向更新 ====================
float updateVirtualHeadingDuringRotation() {
    if (!rotationInProgress) return virtualHeading;
    unsigned long elapsed = millis() - rotationStartTime;
    float rotationSpeed = 360.0 / ((float)MANUAL_CIRCLE_TIME / 1000.0);
    float expectedRotation = (elapsed / 1000.0) * rotationSpeed;
    virtualHeading = normalizeAngle(rotationStartHeading - expectedRotation);
    return virtualHeading;
}

// ==================== 获取当前航向（模式感知）====================
float getCurrentHeading() {
    if (runtimeTurnaroundMode == 1) {
        if (currentState == STATE_A_RUNNING) {
            return updateVirtualHeadingDuringRotation();
        } else {
            return filteredHeading;
        }
    } else {
        return updateVirtualHeadingDuringRotation();
    }
}

// ==================== 罗盘更新（7点滑动平均+静止同步）====================
void updateCompassHeading() {
    if (!compassInitialized) return;
    if (millis() - lastCompassUpdate < COMPASS_UPDATE_INTERVAL) return;

    float rawHeading;
    if (!readCompassHeading(rawHeading)) {
        lastCompassUpdate = millis();
        return;
    }

    // 响应性检测
    float change = fabs(rawHeading - lastValidHeading);
    if (change > 0.5) {
        compassResponding = true;
        noChangeCount = 0;
        lastHeadingChangeTime = millis();
    } else {
        noChangeCount++;
        if (noChangeCount > MAX_NO_CHANGE) compassResponding = false;
    }
    lastValidHeading = rawHeading;

    // 7点滑动平均
    headingBuffer[bufferIndex] = rawHeading;
    bufferIndex = (bufferIndex + 1) % FILTER_SIZE;
    if (bufferIndex == 0) bufferFilled = true;

    if (bufferFilled) {
        float sum = 0;
        for (int i = 0; i < FILTER_SIZE; i++) sum += headingBuffer[i];
        filteredHeading = normalizeAngle(sum / FILTER_SIZE);
    } else {
        filteredHeading = rawHeading;
    }

    // 罗盘模式且电机静止时，同步虚拟航向
    bool isMotorARunning = (currentState == STATE_A_RUNNING);
    if (runtimeTurnaroundMode == 1 && !isMotorARunning) {
        virtualHeading = filteredHeading;
    }
    // 罗盘不响应且静止时强制同步
    if (!compassResponding && !isMotorARunning) {
        virtualHeading = filteredHeading;
    }

    currentHeading = filteredHeading;
    lastCompassUpdate = millis();
}

// ==================== 释放I2C总线（产生STOP信号）====================
void i2c_release_bus() {
    // 使用默认的SDA/SCL引脚（Arduino Uno: A4=SDA, A5=SCL）
    pinMode(SDA, OUTPUT);
    pinMode(SCL, OUTPUT);
    digitalWrite(SDA, HIGH);
    digitalWrite(SCL, HIGH);
    delayMicroseconds(10);
    digitalWrite(SCL, LOW);
    delayMicroseconds(10);
    digitalWrite(SCL, HIGH);
    delayMicroseconds(10);
    digitalWrite(SDA, LOW);
    delayMicroseconds(10);
    digitalWrite(SDA, HIGH);
    delayMicroseconds(10);
    pinMode(SDA, INPUT_PULLUP);
    pinMode(SCL, INPUT_PULLUP);
}

// ==================== QMC5883L 终极初始化（带超时保护）====================
bool initCompassUltimate() {
    unsigned long startTime = millis();
    const unsigned long timeout = 2000;  // 总超时2秒

#if DEBUG
    Serial.println(F("Init Compass..."));
#endif
    lcd.clear();
    lcd.print(F("Init Comp..."));
    stopAllMotors();
    disableMotorDriver();
    delay(100);
    Wire.end();
    delay(50);

    // 释放总线，避免卡死
    i2c_release_bus();

    Wire.begin();
    Wire.setClock(100000);
    delay(200);

    bool i2cConnected = false;
    for (int attempt = 1; attempt <= 3; attempt++) {
        if (millis() - startTime > timeout) {
#if DEBUG
            Serial.println(F("I2C timeout"));
#endif
            return false;
        }
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
        if (millis() - startTime > timeout) {
            return false;
        }
        compass.read();
        int x = compass.getX(), y = compass.getY(), z = compass.getZ();
        if (x != 0 || y != 0 || z != 0) {
            dataValid = true;
            break;
        }
        delay(50);
    }

    compassInitialized = dataValid;
    calibrationDone = false;

    if (compassInitialized) {
        float headingSum = 0;
        int validHeadings = 0;
        for (int i = 0; i < 5; i++) {
            if (millis() - startTime > timeout) {
                return false;
            }
            compass.read();
            int rawHeading = compass.getAzimuth();
            if (rawHeading >= 0 && rawHeading <= 360) {
                validHeadings++;
                headingSum += rawHeading;
            }
            delay(50);
        }
        if (validHeadings > 0) {
            float initialReading = normalizeAngle(headingSum / validHeadings + MAGNETIC_DECLINATION);
            currentHeading = initialReading;
            targetHeading = initialReading;
            filteredHeading = initialReading;
            virtualHeading = initialReading;
            lastValidHeading = initialReading;
            for (int i = 0; i < FILTER_SIZE; i++) headingBuffer[i] = initialReading;
            bufferFilled = true;
        }
        compassResponding = true;
        noChangeCount = 0;
#if DEBUG
        Serial.print(F("Compass OK: "));
        Serial.println(filteredHeading, 1);
#endif
        lcd.clear();
        lcd.print(F("Compass OK"));
        delay(500);
    }
    return compassInitialized;
}

// ==================== 罗盘校准（8字形）====================
void calibrateCompass() {
    if (!compassInitialized) {
        lcd.clear();
        lcd.print(F("Compass N/A"));
        delay(1000);
        return;
    }
    lcd.clear();
    lcd.print(F("Calib..."));
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
    currentHeading = avgHeading;
    targetHeading = avgHeading;
    filteredHeading = avgHeading;
    virtualHeading = avgHeading;
    lastValidHeading = avgHeading;
    calibrationDone = true;
    for (int i = 0; i < FILTER_SIZE; i++) headingBuffer[i] = avgHeading;
    bufferFilled = true;

#if DEBUG
    Serial.print(F("Cal done: "));
    Serial.println(avgHeading, 1);
#endif
    lcd.clear();
    lcd.print(F("Cal OK "));
    lcd.print((int)avgHeading);
    lcd.print(F("°"));
    delay(1000);
}

// ==================== 自动校准整圈时间 ====================
void autoCalibrateCircleTime() {
    if (!compassInitialized || !calibrationDone) {
        lcd.clear();
        lcd.print(F("Compass first!"));
        delay(1000);
        return;
    }
    lcd.clear();
    lcd.print(F("Cal Circle..."));
    stopAllMotors();
    delay(500);
    enableMotorDriver();

    for (int i = 0; i < 5; i++) { updateCompassHeading(); delay(50); }
    float startHeading = filteredHeading;
    rotationStartHeading = startHeading;
    virtualHeading = startHeading;
    controlMotorAFullSpeed();
    unsigned long startTime = millis();

    bool crossed = false;
    unsigned long timeout = millis() + 10000;
    while (!crossed && millis() < timeout) {
        updateCompassHeading();
        float current = filteredHeading;
        float diff = getSimpleAngleDiff(startHeading, current);
        if (fabs(diff) >= 350.0) crossed = true;
        delay(20);
    }
    unsigned long endTime = millis();
    stopAllMotors();

    if (crossed) {
        MANUAL_CIRCLE_TIME = endTime - startTime;
        if (MANUAL_CIRCLE_TIME < 1000) MANUAL_CIRCLE_TIME = 1000;
        calculateMotorATimeout();
        lcd.clear();
        lcd.print(F("Circle Cal OK"));
        lcd.setCursor(0, 1);
        lcd.print(MANUAL_CIRCLE_TIME);
        lcd.print(F("ms"));
    } else {
        lcd.clear();
        lcd.print(F("Cal Failed"));
    }
    delay(1500);
    updateDisplay();
}

// ==================== 旋转控制函数（双模式隔离，增加实际航向反馈）====================
bool rotateToAngle() {
    if (millis() - lastMotorUpdate < MOTOR_CONTROL_INTERVAL) return false;
    lastMotorUpdate = millis();

    // ---------- 罗盘模式：闭环角度控制（逆时针距离）----------
    if (runtimeTurnaroundMode == 1 && compassInitialized && calibrationDone) {
        unsigned long elapsed = millis() - aMotorTimeoutStart;
        float currentVirtual = updateVirtualHeadingDuringRotation();
        float currentActual = filteredHeading; // 使用滤波后的实际航向
        float ccwDistVirtual = getCCWDistance(currentVirtual, targetHeading);
        float ccwDistActual = getCCWDistance(currentActual, targetHeading);

#if DEBUG
        static unsigned long lastDebugPrint = 0;
        if (millis() - lastDebugPrint > 200) {
            Serial.print(F("Cv:")); Serial.print(currentVirtual, 1);
            Serial.print(F(" Ca:")); Serial.print(currentActual, 1);
            Serial.print(F(" T:")); Serial.print(targetHeading, 1);
            Serial.print(F(" CCWv:")); Serial.print(ccwDistVirtual, 1);
            Serial.print(F(" CCWa:")); Serial.println(ccwDistActual, 1);
            lastDebugPrint = millis();
        }
#endif

        // 如果实际航向已到达目标，立即停止
        if (ccwDistActual <= angleTolerance || (360.0 - ccwDistActual) <= angleTolerance) {
            stopAllMotors();
            return true;
        }

        // 如果虚拟航向到达目标，也停止
        if (ccwDistVirtual <= angleTolerance || (360.0 - ccwDistVirtual) <= angleTolerance) {
            stopAllMotors();
            return true;
        }

        // 超时保护
        if (elapsed >= MANUAL_CIRCLE_TIME * 2) {
            stopAllMotors();
            return true;
        }

        controlMotorA(true);
        return false;
    }
    // ---------- 超时模式：开环定时 ----------
    else {
        unsigned long elapsed = millis() - aMotorTimeoutStart;
        if (elapsed >= motorATimeoutPerPlayer) {
            stopAllMotors();
            return true;
        }
        controlMotorA(true);
        return false;
    }
}

// ==================== 障碍事件处理（核心修改）====================
void handleObstacleEvent() {
    lastObstacleTime = millis();
    stopAllMotors();
    dealtCards++;
    if (dealtCards > totalCards) dealtCards = totalCards;

#if DEBUG
    Serial.print(F("Card dealt: "));
    Serial.println(dealtCards);
#endif

    // ---------- 罗盘模式：目标 = 当前航向 - 90°（固定逆时针90°）----------
    if (runtimeTurnaroundMode == 1 && compassInitialized) {
        // 读取当前真实航向（多次平均）
        float sum = 0;
        int count = 0;
        for (int i = 0; i < 5; i++) {
            updateCompassHeading();
            sum += filteredHeading;
            count++;
            delay(20);
        }
        float realHeading = normalizeAngle(sum / count);
        rotationStartHeading = realHeading;
        virtualHeading = realHeading;

        // 目标 = 当前航向 - 90°
        targetHeading = normalizeAngle(realHeading - anglePerPlayer);
    }
    // ---------- 超时模式：沿用原有逻辑（基于虚拟基准）----------
    else {
        uint8_t currentPlayerIndex = dealtCards % playerCount;
        float targetAngle = timeoutInitialHeading - (currentPlayerIndex * anglePerPlayer);
        targetHeading = normalizeAngle(targetAngle);
        rotationStartHeading = getCurrentHeading();
        virtualHeading = rotationStartHeading;
    }

    rotationStartTime = millis();
    aMotorTimeoutStart = millis();
    lastMotorUpdate = millis();

    changeState(STATE_A_RUNNING);
    controlMotorA(true);

    if (runtimeTurnaroundMode == 1) {
        compassResponding = true;
        noChangeCount = 0;
        lastHeadingChangeTime = millis();
    }

    showStatusMessage("Rotate...");

    if (dealtCards >= totalCards && totalCards > 0) {
        stopDealing();
        showStatusMessage("Done!");
        delay(500);
        updateDisplay();
    }
}

// ==================== 状态切换 ====================
void changeState(SystemState newState) {
#if DEBUG
    const char* stateNames[] = { "IDLE", "B_RUN", "A_RUN", "B_TO" };
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

// ==================== 游戏控制 ====================
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

    // ---------- 罗盘模式：无需基准，只需同步当前航向 ----------
    if (runtimeTurnaroundMode == 1 && compassInitialized && calibrationDone) {
        updateCompassHeading();
        virtualHeading = filteredHeading;
        rotationStartHeading = filteredHeading;
        targetHeading = filteredHeading;   // 第一张牌无需旋转（直接推牌）
    }
    // ---------- 超时模式：设置虚拟基准为0 ----------
    else {
        timeoutInitialHeading = 0;
        virtualHeading = 0;
        targetHeading = 0;
        rotationStartHeading = 0;
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

// ==================== 状态机处理 ====================
void handleMotorState() {
    switch (currentState) {
        case STATE_B_RUNNING:
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

                // ---------- 超时模式动态补偿（利用罗盘修正）---------
                if (runtimeTurnaroundMode == 0 && compassInitialized && calibrationDone) {
                    float realHeading = getCurrentHeading();
                    float expectedHeading = normalizeAngle(timeoutInitialHeading - (dealtCards % playerCount) * anglePerPlayer);
                    float angleError = getSimpleAngleDiff(realHeading, expectedHeading);
                    const float MS_PER_DEG = (float)MANUAL_CIRCLE_TIME / 360.0;
                    long correction = (long)(angleError * MS_PER_DEG);
                    correction = constrain(correction, -50, 50);
                    int newTimeout = (int)motorATimeoutPerPlayer - correction;
                    motorATimeoutPerPlayer = (unsigned long)constrain(newTimeout, 100, 1000);
#if DEBUG
                    Serial.print(F("Angle err: ")); Serial.print(angleError, 1);
                    Serial.print(F("°, Corr: ")); Serial.print(correction);
                    Serial.print(F("ms, New TO: ")); Serial.println(motorATimeoutPerPlayer);
#endif
                }
                // ----------------------------------------------------

                // 发完所有牌？
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

// ==================== 系统重启 ====================
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
    lcd.setCursor(0, 1);
    lcd.print(runtimeTurnaroundMode == 1 ? F("Compass") : F("Timeout"));
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
        if (irValue == 0xFFFFFFFF) {
            IrReceiver.resume();
            return;
        }
        switch (irValue) {
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
                    lcd.setCursor(0, 1);
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
                    lcd.setCursor(0, 1);
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
                    lcd.setCursor(0, 1);
                    lcd.print(runtimeTurnaroundMode == 1 ? F("Compass") : F("Timeout"));
                    rotationStartHeading = startAngle;
                    rotationStartTime = millis();
                    aMotorTimeoutStart = millis();
                    targetHeading = testTarget;
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
            case IR_TEST_MOTOR_B:
                if (!isRunning) {
                    enableMotorDriver();
                    controlMotorB(1);
                    lcd.clear();
                    lcd.print(F("MotorB Test"));
                    lcd.setCursor(0, 1);
                    lcd.print(F("Wait Obstacle..."));
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
                lcd.print(runtimeTurnaroundMode == 1 ? F("C") : F("T"));
                delay(800);
                updateDisplay();
#if DEBUG
                Serial.print(F("Mode: "));
                Serial.println(runtimeTurnaroundMode == 1 ? F("Compass") : F("Timeout"));
#endif
                break;
            case IR_SYSTEM_RESET:
                resetSystem();
                break;
            case IR_VERSION:
                lcd.clear();
                lcd.print(F("Dealer v32.0"));
                lcd.setCursor(0, 1);
                lcd.print(F("FixCCW"));
                delay(800);
                updateDisplay();
                break;
            case IR_BTP:
                if (!isRunning) {
                    motorBTimeout += 1000;
                    if (motorBTimeout > 15000) motorBTimeout = 8000;
                    lcd.clear();
                    lcd.print(F("B Tout:"));
                    lcd.print(motorBTimeout / 1000);
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
                    lcd.print(motorBTimeout / 1000);
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

// ==================== 串口命令处理 ====================
void handleSerialCommand(const char* command) {
#if ENABLE_KEYBOARD
    if (command[0] >= '0' && command[0] <= '9') {
        long timeValue = atol(command);
        if (timeValue >= 100 && timeValue <= 10000) {
            MANUAL_CIRCLE_TIME = timeValue;
            calculateMotorATimeout();
            lcd.clear();
            lcd.print(F("Circle Set"));
            lcd.setCursor(0, 1);
            lcd.print(MANUAL_CIRCLE_TIME);
            lcd.print(F("ms"));
            delay(800);
            updateDisplay();
            return;
        }
    }
    switch (command[0]) {
        case 'v': case 'V':
            lcd.clear();
            lcd.print(F("Dealer v32.0"));
            lcd.setCursor(0, 1);
            lcd.print(F("FixCCW"));
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
                lcd.setCursor(0, 1);
                lcd.print(runtimeTurnaroundMode == 1 ? F("C") : F("T"));
                rotationStartHeading = startAngle;
                rotationStartTime = millis();
                aMotorTimeoutStart = millis();
                targetHeading = testTarget;
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
            if (!isRunning) {
                enableMotorDriver();
                controlMotorB(1);
                lcd.clear();
                lcd.print(F("MotorB Test"));
                lcd.setCursor(0, 1);
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
            lcd.print(runtimeTurnaroundMode == 1 ? F("C") : F("T"));
            delay(800);
            updateDisplay();
#if DEBUG
            Serial.print(F("Mode: "));
            Serial.println(runtimeTurnaroundMode == 1 ? F("Compass") : F("Timeout"));
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
            lcd.setCursor(0, 1);
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
                lcd.setCursor(0, 1);
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

// ==================== LCD 显示 ====================
void updateDisplay() {
    lcd.clear();
    // 第一行：Px Dx J/N M:C/T 圈时间
    lcd.setCursor(0, 0);
    lcd.print(F("P"));
    lcd.print(playerCount);
    lcd.print(F(" D"));
    lcd.print(deckCount);
    lcd.print(hasJokers ? F(" J") : F(" N"));
    lcd.print(F(" M:"));
    lcd.print(runtimeTurnaroundMode == 1 ? F("C") : F("T"));
    lcd.setCursor(10, 0);
    lcd.print(MANUAL_CIRCLE_TIME);
    // 第二行：总牌/已发 状态 航向
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
    lcd.setCursor(12, 1);
    int intAngle = (int)getCurrentHeading();
    if (intAngle < 100) lcd.print(F("0"));
    if (intAngle < 10) lcd.print(F("0"));
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
    delay(400);
}

// ==================== SETUP ====================
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

    lcd.begin(16, 2);
    lcd.clear();
    lcd.print(F("Dealer v32.0"));
    lcd.setCursor(0, 1);
    lcd.print(F("FixCCW"));

#if DEBUG
    Serial.begin(9600);
    delay(500);
    Serial.println(F("=== System Startup ==="));
    Serial.println(F("Manual Calibration"));
    Serial.print(F("Init mode: "));
    Serial.println(runtimeTurnaroundMode == 1 ? F("Compass") : F("Timeout"));
    Serial.print(F("Circle: "));
    Serial.print(MANUAL_CIRCLE_TIME);
    Serial.println(F("ms"));
    Serial.println(F("Use 'H' help"));
#endif

#if ENABLE_INFRA
    IrReceiver.begin(IR_RECEIVE_PIN);
#endif

    Wire.begin();
    delay(100);

    compassInitialized = false;
    for (int attempt = 1; attempt <= 2; attempt++) {
        lcd.clear();
        lcd.print(F("Init Comp"));
        lcd.setCursor(0, 1);
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
            lcd.setCursor(0, 1);
            lcd.print(F("Use Timeout"));
            delay(1000);
        }
    } else {
        lcd.clear();
        lcd.print(F("No Compass"));
        lcd.setCursor(0, 1);
        lcd.print(F("Timeout Mode"));
        delay(1000);
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
    Serial.println(compassInitialized ? F("YES") : F("NO"));
    Serial.println(F("========================"));
#endif
}

// ==================== LOOP ====================
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
        Serial.print(runtimeTurnaroundMode == 1 ? F("Compass") : F("Timeout"));
        if (compassInitialized) {
            Serial.print(F(" Hdg:"));
            Serial.print(getCurrentHeading(), 1);
        }
        Serial.println();
        lastDebugTime = millis();
    }
#endif
    delay(10);
}