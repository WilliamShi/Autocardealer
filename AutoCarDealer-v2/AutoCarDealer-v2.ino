// ==================== 系统配置宏 ====================
#define ENABLE_INFRA 1            // 启用红外遥控
#define ENABLE_KEYBOARD 0         // 启用串口键盘指令
#define DEBUG 1                    // 调试信息输出
#define IR_DEBUG 0                 // 红外详细调试（关闭减少串口干扰）

// ==================== 库包含 ====================
#include <LiquidCrystal.h>

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

// ==================== 光电开关引脚定义（根据实际接线修正）====================
#define PHOTO_A_PIN A4        // 电机A计数光电开关（实际接在A4）
#define PHOTO_B_PIN A0        // 电机B发牌检测光电开关（实际接在A0）

// ==================== 模块实例化 ====================
LiquidCrystal lcd(12, 11, 5, 4, 3, 2);

#if ENABLE_INFRA
IRrecv irrecv(IR_RECEIVE_PIN);
decode_results results;
#endif

// ==================== 系统状态枚举 ====================
enum SystemState {
    STATE_IDLE,
    STATE_B_RUNNING,   // 发牌电机运行
    STATE_A_RUNNING    // 旋转电机运行
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
uint8_t isRunning = 0;          // 0:停止 1:运行 2:暂停

// ---- 电机控制参数 ----
const int MOTOR_A_SPEED = 255;
const int MOTOR_B_SPEED = 255;
unsigned long motorBTimeout = 8000;      // 电机B超时保护（8秒）
unsigned long motorATimeout = 10000;      // 电机A超时保护（10秒）

// ---- 旋转方向 ----
bool clockwise = true;                    // true = 顺时针，false = 逆时针

// ---- 光电开关A（旋转计数）相关 ----
volatile uint16_t photoACount = 0;        // 当前旋转过程中累计的遮挡次数（从0开始）
uint16_t nextStopCount = 0;               // 下一次应该停止的计数目标
uint8_t lastPhotoAState = HIGH;            // 上次读取的光电A状态
unsigned long photoADebounce = 0;          // 防抖动时间
unsigned long motorAStartTime = 0;         // 电机A开始旋转的时间（用于超时）

// ---- 光电开关B（发牌检测）相关 ----
uint8_t lastPhotoBState = HIGH;
unsigned long photoBTriggerTime = 0;       // 检测到上升沿的时间
bool photoBTriggered = false;              // 是否已检测到牌离开
unsigned long photoBDebounce = 0;
const unsigned long PHOTO_B_DELAY = 200;    // 牌离开后延迟停止时间

// ---- 电机B超时保护 ----
unsigned long motorBStartTime = 0;

// ---- 调试定时器 ----
unsigned long lastDebugTime = 0;

// ---- 串口缓冲区 ----
const int SERIAL_BUFFER_SIZE = 32;
char serialBuffer[SERIAL_BUFFER_SIZE];
int serialBufferIndex = 0;
unsigned long lastSerialCharTime = 0;
const unsigned long SERIAL_TIMEOUT = 100;

// ==================== 基础位置表（第0圈）====================
// 索引为玩家数，每个数组存储对应玩家数的第0圈位置（顺序与玩家顺序一致）
const uint8_t basePositions[9][8] = {
    {0},           // 占位，不使用
    {0},           // 玩家数1（未使用）
    {0, 4},        // 玩家2
    {0, 2, 5},     // 玩家3
    {0, 2, 4, 6},  // 玩家4
    {0, 2, 4, 5, 6}, // 玩家5
    {0, 2, 3, 4, 6, 7}, // 玩家6
    {0, 1, 2, 3, 4, 5, 6}, // 玩家7
    {0, 1, 2, 3, 4, 5, 6, 7}  // 玩家8
};

// ==================== 函数声明 ====================
void enableMotorDriver();
void disableMotorDriver();
void stopAllMotors();
void controlMotorA(bool enable);
void controlMotorB(bool enable);
void startDealing();
void pauseDealing();
void resumeDealing();
void stopDealing();
void updatePhotoA();                 // 检测光电A边沿并计数
void updatePhotoB();                 // 检测光电B上升沿
void handleMotorState();
void resetSystem();
void performInitialHoming();          // 上电归位：寻找第一个下降沿
#if ENABLE_INFRA
void processInfraredInput();
#endif
void processSerialInput();
void handleSerialCommand(const char* command);
void updateDisplay();
void showStatusMessage(const char* message);
void updateNextStopCount();           // 根据当前已发牌数计算下一个停止计数值

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
}

void controlMotorA(bool enable) {
    if (enable) {
        // 根据旋转方向设置电机A的转向
        if (clockwise) {
            // 顺时针方向
            digitalWrite(MOTOR_A_IN1, LOW);
            digitalWrite(MOTOR_A_IN2, HIGH);
        } else {
            // 逆时针方向
            digitalWrite(MOTOR_A_IN1, HIGH);
            digitalWrite(MOTOR_A_IN2, LOW);
        }
        analogWrite(MOTOR_A_PWM, MOTOR_A_SPEED);
        motorAStartTime = millis();   // 记录开始时间，用于超时
    } else {
        stopAllMotors();
    }
}

void controlMotorB(bool enable) {
    if (enable) {
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

// ==================== 光电开关A（旋转计数）检测 ====================
void updatePhotoA() {
    // 只在电机A旋转时进行计数，避免误触发
    if (currentState != STATE_A_RUNNING) return;

    uint8_t currentStateA = digitalRead(PHOTO_A_PIN);
    // 检测下降沿（从HIGH到LOW，对应遮挡物离开）
    if (currentStateA == LOW && lastPhotoAState == HIGH) {
        if (millis() - photoADebounce > 20) { // 简单防抖
            photoADebounce = millis();
            photoACount++;
#if DEBUG
            Serial.print(F("PhotoA count: "));
            Serial.println(photoACount);
#endif
        }
    }
    lastPhotoAState = currentStateA;
}

// ==================== 光电开关B（发牌检测）检测 ====================
void updatePhotoB() {
    // 只在电机B运行时检测
    if (currentState != STATE_B_RUNNING) return;

    uint8_t currentStateB = digitalRead(PHOTO_B_PIN);
    // 检测上升沿（从遮挡到未遮挡，即牌离开）
    if (currentStateB == LOW && lastPhotoBState == HIGH) {
        if (millis() - photoBDebounce > 20) {
            photoBDebounce = millis();
            // 记录触发时间，稍后停止电机
            photoBTriggered = true;
            photoBTriggerTime = millis();
#if DEBUG
            Serial.println(F("PhotoB triggered (card left)"));
#endif
        }
    }
    lastPhotoBState = currentStateB;
}

// ==================== 上电归位：寻找第一个下降沿 ====================
void performInitialHoming() {
    enableMotorDriver();
    delay(100);

    // 电机A归位：旋转直到第一个下降沿
    lcd.clear();
    lcd.print("Homing MotorA...");
    controlMotorA(true);
    unsigned long startTime = millis();
    uint8_t lastState = digitalRead(PHOTO_A_PIN);
    bool found = false;
    while (millis() - startTime < 10000) { // 超时10秒
        uint8_t curState = digitalRead(PHOTO_A_PIN);
        if (curState == LOW && lastState == HIGH) { // 下降沿
            found = true;
            break;
        }
        lastState = curState;
        delay(10);
    }
    controlMotorA(false);
    lcd.setCursor(0, 1);
    if (found) {
        lcd.print("A OK");
    } else {
        lcd.print("A TO");
    }
    delay(500);
/*
    // 电机B归位：旋转直到第一个下降沿
    lcd.clear();
    lcd.print("Homing MotorB...");
    controlMotorB(true);
    startTime = millis();
    lastState = digitalRead(PHOTO_B_PIN);
    found = false;
    while (millis() - startTime < 10000) {
        uint8_t curState = digitalRead(PHOTO_B_PIN);
        if (curState == LOW && lastState == HIGH) { // 下降沿
            found = true;
            break;
        }
        lastState = curState;
        delay(10);
    }
    controlMotorB(false);
    lcd.setCursor(0, 1);
    if (found) {
        lcd.print("B OK");
    } else {
        lcd.print("B TO");
    }
    delay(500);
*/
    lcd.clear();
}

// ==================== 计算下一次停止计数（精确查表）====================
void updateNextStopCount() {
    if (playerCount < 2 || playerCount > 8) return; // 安全保护

    // 下一张牌的索引 = 当前已发牌数（因为下一张就是第dealtCards张，从0开始）
    uint16_t nextIndex = dealtCards;
    uint8_t circle = nextIndex / playerCount;          // 第几圈（每圈有playerCount张牌）
    uint8_t posInCircle = nextIndex % playerCount;     // 圈内位置
    nextStopCount = basePositions[playerCount][posInCircle] + circle * 8;

#if DEBUG
    Serial.print(F("Next stop: "));
    Serial.print(nextStopCount);
    Serial.print(F(" (index="));
    Serial.print(nextIndex);
    Serial.print(F(", circle="));
    Serial.print(circle);
    Serial.print(F(", pos="));
    Serial.print(posInCircle);
    Serial.println(F(")"));
#endif
}

// ==================== 暂停发牌 ====================
void pauseDealing() {
    if (isRunning == 1) {          // 仅在运行状态可暂停
        stopAllMotors();            // 停止电机，保留所有计数变量
        isRunning = 2;              // 进入暂停状态
#if DEBUG
        Serial.println(F("Paused"));
#endif
        updateDisplay();
    }
}

// ==================== 恢复发牌 ====================
void resumeDealing() {
    if (isRunning == 2) {          // 仅在暂停状态可恢复
        enableMotorDriver();        // 确保驱动已使能
        // 根据之前的状态恢复电机
        if (currentState == STATE_B_RUNNING) {
            if (photoBTriggered) {
                // 已在等待延迟，不需要启动电机B，但需重置超时计时防止误触发
                motorBStartTime = 0;  // 超时检查将失效，由延迟逻辑接管
            } else {
                // 重新启动电机B
                lastPhotoBState = digitalRead(PHOTO_B_PIN); // 刷新当前电平
                controlMotorB(true);
                motorBStartTime = millis();
            }
        } else if (currentState == STATE_A_RUNNING) {
            // 重新启动电机A
            lastPhotoAState = digitalRead(PHOTO_A_PIN); // 刷新当前电平
            controlMotorA(true);
            motorAStartTime = millis();
        } else {
            // 意外状态，默认回到发牌阶段
            currentState = STATE_B_RUNNING;
            controlMotorB(true);
            motorBStartTime = millis();
        }
        isRunning = 1;              // 恢复运行
#if DEBUG
        Serial.println(F("Resumed"));
#endif
        updateDisplay();
    }
}

// ==================== 开始发牌（多态：开始/暂停/恢复）====================
void startDealing() {
    if (isRunning == 0) {
        // 全新开始
        if (totalCards <= 0) {
            showStatusMessage("No Cards!");
            delay(500);
            updateDisplay();
            return;
        }
        isRunning = 1;
        dealtCards = 0;
        photoACount = 0;
        nextStopCount = 0;           // 第一张牌不旋转，目标为0

        stopAllMotors();
        enableMotorDriver();

        // 第一张牌：直接启动电机B
        currentState = STATE_B_RUNNING;
        motorBStartTime = millis();
        controlMotorB(true);
        showStatusMessage("Start");
        delay(300);
    } else if (isRunning == 1) {
        // 运行中 -> 暂停
        pauseDealing();
    } else if (isRunning == 2) {
        // 暂停中 -> 恢复
        resumeDealing();
    }
}

// ==================== 完全停止发牌（重置所有状态）====================
void stopDealing() {
    isRunning = 0;
    currentState = STATE_IDLE;
    photoACount = 0;
    nextStopCount = 0;
    dealtCards = 0;                  // 重置已发牌数
    photoBTriggered = false;
    stopAllMotors();
    // 可选：disableMotorDriver();  // 如需省电可取消注释
    updateDisplay();
}

// ==================== 状态机处理 ====================
void handleMotorState() {
    switch (currentState) {
        case STATE_B_RUNNING:
            // 检测光电B上升沿
            updatePhotoB();

            // 如果已检测到牌离开，延迟后停止电机B
            if (photoBTriggered && (millis() - photoBTriggerTime >= PHOTO_B_DELAY)) {
                controlMotorB(false);          // 停止电机B
                photoBTriggered = false;
                dealtCards++;
                if (dealtCards > totalCards) dealtCards = totalCards;

#if DEBUG
                Serial.print(F("Card dealt: "));
                Serial.println(dealtCards);
#endif

                // 检查是否发完所有牌
                if (dealtCards >= totalCards && totalCards > 0) {
                    stopDealing();
                    showStatusMessage("All Done!");
                    delay(500);
                    updateDisplay();
                    break;
                }

                // 计算下一次应该停止的计数目标（精确查表）
                updateNextStopCount();
                currentState = STATE_A_RUNNING;
                controlMotorA(true);
                showStatusMessage("Rotate...");
            }

            // 超时保护（如果一直没有牌发出）-> 暂停而非停止
            if (millis() - motorBStartTime > motorBTimeout && isRunning == 1) {
                pauseDealing();
                showStatusMessage("B Timeout");
                delay(500);
                updateDisplay();
            }
            break;

        case STATE_A_RUNNING:
            updatePhotoA();   // 更新旋转计数（下降沿计数）

            // 检查是否达到目标计数
            if (photoACount >= nextStopCount) {
                controlMotorA(false);   // 停止电机A
                currentState = STATE_B_RUNNING;
                motorBStartTime = millis();
                controlMotorB(true);
                showStatusMessage("Deal...");
                break;
            }

            // 电机A超时保护（防止计数不准导致无限旋转）-> 暂停
            if (millis() - motorAStartTime > motorATimeout && isRunning == 1) {
                controlMotorA(false);
                pauseDealing();
                showStatusMessage("A Timeout");
                delay(500);
                updateDisplay();
            }
            break;
    }
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
        // 非空闲状态（运行或暂停）只响应停止键和发牌键
        if (isRunning != 0) {
            if (irValue == 0xF609FF00) { // 停止
                stopDealing();
            } else if (irValue == 0xBC43FF00) { // 发牌键（开始/暂停/恢复）
                startDealing();
            }
            IrReceiver.resume();
            return;
        }

        // 空闲状态处理所有键
        switch (irValue) {
            case 0xBA45FF00: // 减玩家
                if (playerCount > 2) playerCount--; else playerCount = 8;
                updateDisplay();
                break;
            case 0xB946FF00: // 重置参数
                playerCount = 4; deckCount = 3; hasJokers = 1; remainCards = 0;
                totalCards = deckCount * (hasJokers ? 54 : 52);
                updateDisplay();
                break;
            case 0xB847FF00: // 加玩家
                playerCount++; if (playerCount > 8) playerCount = 2;
                updateDisplay();
                break;
            case 0xBB44FF00: // 加副数
                deckCount++; if (deckCount > 3) deckCount = 1;
                totalCards = deckCount * (hasJokers ? 54 : 52);
                updateDisplay();
                break;
            case 0xBF40FF00: // 切换有无王
                hasJokers = !hasJokers;
                totalCards = deckCount * (hasJokers ? 54 : 52);
                updateDisplay();
                break;
            case 0xBC43FF00: // 开始（空闲时正常开始）
                startDealing();
                break;
            case 0xF807FF00: // 减整圈时间（无效）
            case 0xEA15FF00: // 加整圈时间
                lcd.clear();
                lcd.print(F("No circle time"));
                delay(800);
                updateDisplay();
                break;
            case 0xF609FF00: // 停止（空闲时显示空闲）
                lcd.clear();
                lcd.print(F("Idle"));
                delay(800);
                updateDisplay();
                break;
            case 0xE916FF00: // 设置剩余牌
                remainCards += playerCount;
                if (remainCards > playerCount * 4) remainCards = 0;
                totalCards = deckCount * (hasJokers ? 54 : 52);
                if (remainCards > 0 && totalCards > remainCards) totalCards -= remainCards;
                updateDisplay();
                break;
            case 0xE619FF00: // 测试电机A
                enableMotorDriver();
                lcd.clear();
                lcd.print(F("MotorA Test"));
                lcd.setCursor(0, 1);
                lcd.print(F("Wait count..."));
                photoACount = 0;
                nextStopCount = 2; // 简单测试，转两个遮挡停
                controlMotorA(true);
                unsigned long testStart = millis();
                bool testComplete = false;
                while (!testComplete && millis() - testStart < 10000) {
                    updatePhotoA();
                    if (photoACount >= nextStopCount) {
                        controlMotorA(false);
                        testComplete = true;
                    }
                    delay(10);
                }
                if (!testComplete) stopAllMotors();
                lcd.clear();
                lcd.print(F("Test done"));
                delay(800);
                updateDisplay();
                break;
            case 0xF20DFF00: // 测试电机B
                enableMotorDriver();
                controlMotorB(true);
                lcd.clear();
                lcd.print(F("MotorB Test"));
                lcd.setCursor(0, 1);
                lcd.print(F("Wait card..."));
                unsigned long testStartB = millis();
                bool cardDetected = false;
                photoBTriggered = false;
                while (millis() - testStartB < 5000) {
                    updatePhotoB();
                    if (photoBTriggered) {
                        cardDetected = true;
                        break;
                    }
                    delay(10);
                }
                controlMotorB(false);
                lcd.clear();
                if (cardDetected) lcd.print(F("B OK"));
                else lcd.print(F("B TO"));
                delay(800);
                updateDisplay();
                break;
            case 0xE718FF00: // 切换旋转方向
                clockwise = !clockwise;
                lcd.clear();
                lcd.print(F("Direction:"));
                lcd.setCursor(0, 1);
                lcd.print(clockwise ? F("CW") : F("CCW"));
                delay(800);
                updateDisplay();
                break;
            case 0xA15EFF00: // 系统复位
                resetSystem();
                break;
            case 0xF708FF00: // 版本信息
                lcd.clear();
                lcd.print(F("Dealer v39.4"));
                lcd.setCursor(0, 1);
                lcd.print(F("Homing"));
                delay(800);
                updateDisplay();
                break;
            case 0xE31CFF00: // 减B超时
                if (motorBTimeout > 2000) motorBTimeout -= 1000; else motorBTimeout = 8000;
                lcd.clear();
                lcd.print(F("B Tout:"));
                lcd.print(motorBTimeout / 1000);
                lcd.print(F("s"));
                delay(800);
                updateDisplay();
                break;
            case 0xA55AFF00: // 加B超时
                motorBTimeout += 1000;
                if (motorBTimeout > 15000) motorBTimeout = 8000;
                lcd.clear();
                lcd.print(F("B Tout:"));
                lcd.print(motorBTimeout / 1000);
                lcd.print(F("s"));
                delay(800);
                updateDisplay();
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
        lcd.clear();
        lcd.print(F("Not supported"));
        delay(800);
        updateDisplay();
        return;
    }
    switch (command[0]) {
        case 'v': case 'V':
            lcd.clear();
            lcd.print(F("Dealer v39.4"));
            lcd.setCursor(0, 1);
            lcd.print(F("Homing"));
            delay(800);
            updateDisplay();
            break;
        case 'p': case 'P':
            playerCount++; if (playerCount > 8) playerCount = 2;
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
            startDealing();
            break;
        case 'c': case 'C':
            playerCount = 4; deckCount = 3; hasJokers = 1; remainCards = 0;
            stopDealing();
            totalCards = deckCount * (hasJokers ? 54 : 52);
            updateDisplay();
            break;
        case 't': case 'T':
            stopDealing();
            break;
        case 'a': case 'A':
            if (!isRunning) {
                enableMotorDriver();
                lcd.clear();
                lcd.print(F("MotorA Test"));
                lcd.setCursor(0, 1);
                lcd.print(F("Wait count..."));
                photoACount = 0;
                nextStopCount = 2;
                controlMotorA(true);
                unsigned long testStart = millis();
                bool testComplete = false;
                while (!testComplete && millis() - testStart < 10000) {
                    updatePhotoA();
                    if (photoACount >= nextStopCount) {
                        controlMotorA(false);
                        testComplete = true;
                    }
                    delay(10);
                }
                if (!testComplete) stopAllMotors();
                lcd.clear();
                lcd.print(F("Test done"));
                delay(800);
                updateDisplay();
            }
            break;
        case 'b': case 'B':
            if (!isRunning) {
                enableMotorDriver();
                controlMotorB(true);
                lcd.clear();
                lcd.print(F("MotorB Test"));
                lcd.setCursor(0, 1);
                lcd.print(F("Wait card..."));
                unsigned long testStart = millis();
                bool cardDetected = false;
                photoBTriggered = false;
                while (millis() - testStart < 5000) {
                    updatePhotoB();
                    if (photoBTriggered) {
                        cardDetected = true;
                        break;
                    }
                    delay(10);
                }
                controlMotorB(false);
                lcd.clear();
                if (cardDetected) lcd.print(F("B OK"));
                else lcd.print(F("B TO"));
                delay(800);
                updateDisplay();
            }
            break;
        case 'm': case 'M':   // 切换方向
            clockwise = !clockwise;
            lcd.clear();
            lcd.print(F("Direction:"));
            lcd.setCursor(0, 1);
            lcd.print(clockwise ? F("CW") : F("CCW"));
            delay(800);
            updateDisplay();
            break;
        case 'z': case 'Z':
            resetSystem();
            break;
        case '+': case '-':
            lcd.clear();
            lcd.print(F("No circle adj"));
            delay(800);
            updateDisplay();
            break;
        case 'h': case 'H':
            Serial.println(F("=== Card Dealer Cmds (Homing) ==="));
            Serial.println(F("P/D/J/R/S/C/T/A/B/M/Z/H"));
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
    photoACount = 0;
    nextStopCount = 0;
    photoBTriggered = false;
    serialBufferIndex = 0;
    serialBuffer[0] = '\0';
    lcd.clear();
    lcd.print(F("Reset"));
    lcd.setCursor(0, 1);
    lcd.print(F("Homing"));
    delay(800);
    updateDisplay();
}

// ==================== LCD 显示 ====================
void updateDisplay() {
    lcd.clear();
    // 第一行：Px Dx J 剩余牌数 方向
    lcd.setCursor(0, 0);
    lcd.print(F("P"));
    lcd.print(playerCount);
    lcd.print(F(" D"));
    lcd.print(deckCount);
    lcd.print(F(" "));
    lcd.print(hasJokers ? F("J") : F("N"));
    lcd.print(F(" R"));
    lcd.print(totalCards - dealtCards);
    lcd.print(F(" "));
    lcd.print(clockwise ? F("CW") : F("CC"));

    // 第二行：已发/总数 状态 计数
    lcd.setCursor(0, 1);
    lcd.print(F("D:"));
    lcd.print(dealtCards);
    lcd.print(F("/"));
    lcd.print(totalCards);
    lcd.print(F(" "));
    if (isRunning == 1) {
        switch (currentState) {
            case STATE_B_RUNNING: lcd.print(F("B")); break;
            case STATE_A_RUNNING: lcd.print(F("A")); break;
            default: lcd.print(F("R"));
        }
    } else if (isRunning == 2) {
        lcd.print(F("P"));  // 暂停状态
    } else {
        lcd.print(F("S"));
    }
    lcd.print(F(" C:"));
    lcd.print(photoACount);
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
    pinMode(PHOTO_A_PIN, INPUT_PULLUP);
    pinMode(PHOTO_B_PIN, INPUT_PULLUP);
    disableMotorDriver();

    lcd.begin(16, 2);
    lcd.clear();
    lcd.print(F("Dealer v39.4"));
    lcd.setCursor(0, 1);
    lcd.print(F("Homing"));

#if DEBUG
    Serial.begin(9600);
    delay(500);
    Serial.println(F("=== System Startup (Homing) ==="));
    Serial.println(F("Players: 2-8, Direction: CW (default)"));
#endif

#if ENABLE_INFRA
    IrReceiver.begin(IR_RECEIVE_PIN);
#endif

    // 初始化参数
    totalCards = deckCount * (hasJokers ? 54 : 52);

    lastPhotoAState = digitalRead(PHOTO_A_PIN);
    lastPhotoBState = digitalRead(PHOTO_B_PIN);

    serialBufferIndex = 0;
    serialBuffer[0] = '\0';

    // 执行上电归位
    performInitialHoming();

    updateDisplay();

#if DEBUG
    Serial.print(F("Players: "));
    Serial.println(playerCount);
    Serial.print(F("Cards: "));
    Serial.println(totalCards);
    Serial.println(F("========================"));
#endif
}

// ==================== LOOP ====================
void loop() {
    processSerialInput();
#if ENABLE_INFRA
    processInfraredInput();
#endif
    if (isRunning == 1) {   // 仅在运行状态处理电机
        handleMotorState();
    }
#if DEBUG
    if (millis() - lastDebugTime > 3000) {
        Serial.print(F("Cards: "));
        Serial.print(dealtCards);
        Serial.print(F("/"));
        Serial.print(totalCards);
        Serial.print(F(" State:"));
        Serial.print(currentState == STATE_B_RUNNING ? "B" : (currentState == STATE_A_RUNNING ? "A" : "I"));
        Serial.print(F(" Dir:"));
        Serial.print(clockwise ? "CW" : "CC");
        Serial.print(F(" PhotoA cnt:"));
        Serial.print(photoACount);
        Serial.print(F(" NextStop:"));
        Serial.print(nextStopCount);
        Serial.print(F(" isRunning:"));
        Serial.println(isRunning);
        lastDebugTime = millis();
    }
#endif
    delay(10);
}