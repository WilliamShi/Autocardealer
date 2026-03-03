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
bool isRunning = 0;

// ---- 电机控制参数 ----
const int MOTOR_A_SPEED = 150;
const int MOTOR_B_SPEED = 255;
unsigned long motorBTimeout = 8000;      // 电机B超时保护（8秒）

// ---- 旋转方向 ----
bool clockwise = true;                    // true = 顺时针，false = 逆时针

// ---- 光电开关A（旋转计数）相关 ----
volatile uint16_t photoACount = 0;        // 当前旋转过程中累计的遮挡次数（从0开始）
uint16_t nextStopCount = 0;               // 下一次应该停止的计数目标
float cardsPerRevolution = 2.0;            // 每发一张牌对应的遮挡次数增量（8/玩家数）
float accumulatedTarget = 0.0;             // 累积的目标遮挡次数（浮点）
uint8_t lastPhotoAState = HIGH;            // 上次读取的光电A状态
unsigned long photoADebounce = 0;          // 防抖动时间

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

// ==================== 函数声明 ====================
void enableMotorDriver();
void disableMotorDriver();
void stopAllMotors();
void controlMotorA(bool enable);
void controlMotorB(bool enable);
void startDealing();
void stopDealing();
void updatePhotoA();                 // 检测光电A边沿并计数
void updatePhotoB();                 // 检测光电B上升沿
void handleMotorState();
void resetSystem();
#if ENABLE_INFRA
void processInfraredInput();
#endif
void processSerialInput();
void handleSerialCommand(const char* command);
void updateDisplay();
void showStatusMessage(const char* message);

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
    // 检测下降沿（遮挡发生）
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
    if (currentStateB == HIGH && lastPhotoBState == LOW) {
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
    accumulatedTarget = 0.0;          // 重置累积目标
    photoACount = 0;
    nextStopCount = 0;

    stopAllMotors();
    enableMotorDriver();

    // 第一张牌：直接启动电机B
    currentState = STATE_B_RUNNING;
    motorBStartTime = millis();
    controlMotorB(true);
    showStatusMessage("Start");
    delay(300);
}

void stopDealing() {
    isRunning = 0;
    currentState = STATE_IDLE;
    stopAllMotors();
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

                // 更新累积目标，准备旋转到下一位置
                accumulatedTarget += cardsPerRevolution;
                nextStopCount = (uint16_t)(accumulatedTarget + 0.999); // ceil
                photoACount = 0;                 // 重置计数，开始新一圈计数
                currentState = STATE_A_RUNNING;
                controlMotorA(true);
                showStatusMessage("Rotate...");
            }

            // 超时保护（如果一直没有牌发出）
            if (millis() - motorBStartTime > motorBTimeout) {
                stopDealing();
                showStatusMessage("B Timeout");
                delay(500);
                updateDisplay();
            }
            break;

        case STATE_A_RUNNING:
            updatePhotoA();   // 更新旋转计数

            // 检查是否达到目标计数
            if (photoACount >= nextStopCount) {
                controlMotorA(false);   // 停止电机A
                currentState = STATE_B_RUNNING;
                motorBStartTime = millis();
                controlMotorB(true);
                showStatusMessage("Deal...");
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
        switch (irValue) {
            case 0xBA45FF00: // 减玩家
                if (!isRunning) {
                    if (playerCount > 2) playerCount--; else playerCount = 8;
                    cardsPerRevolution = 8.0 / playerCount;
                    updateDisplay();
                }
                break;
            case 0xB946FF00: // 重置参数
                if (!isRunning) {
                    playerCount = 4; deckCount = 3; hasJokers = 1; remainCards = 0;
                    totalCards = deckCount * (hasJokers ? 54 : 52);
                    cardsPerRevolution = 8.0 / playerCount;
                    updateDisplay();
                }
                break;
            case 0xB847FF00: // 加玩家
                if (!isRunning) {
                    playerCount++; if (playerCount > 8) playerCount = 2;
                    cardsPerRevolution = 8.0 / playerCount;
                    updateDisplay();
                }
                break;
            case 0xBB44FF00: // 加副数
                if (!isRunning) {
                    deckCount++; if (deckCount > 3) deckCount = 1;
                    totalCards = deckCount * (hasJokers ? 54 : 52);
                    updateDisplay();
                }
                break;
            case 0xBF40FF00: // 切换有无王
                if (!isRunning) {
                    hasJokers = !hasJokers;
                    totalCards = deckCount * (hasJokers ? 54 : 52);
                    updateDisplay();
                }
                break;
            case 0xBC43FF00: // 开始
                if (!isRunning) startDealing();
                else showStatusMessage("Running");
                break;
            case 0xF807FF00: // 减整圈时间（无效，保留占位）
            case 0xEA15FF00: // 加整圈时间
                if (!isRunning) {
                    // 新系统中圈时间不再使用，但可忽略或显示提示
                    lcd.clear();
                    lcd.print(F("No circle time"));
                    delay(800);
                    updateDisplay();
                }
                break;
            case 0xF609FF00: // 停止
                if (isRunning) stopDealing();
                else {
                    lcd.clear();
                    lcd.print(F("Idle"));
                    delay(800);
                    updateDisplay();
                }
                break;
            case 0xE916FF00: // 设置剩余牌
                if (!isRunning) {
                    remainCards += playerCount;
                    if (remainCards > playerCount * 4) remainCards = 0;
                    totalCards = deckCount * (hasJokers ? 54 : 52);
                    if (remainCards > 0 && totalCards > remainCards) totalCards -= remainCards;
                    updateDisplay();
                }
                break;
            case 0xE619FF00: // 测试电机A
                if (!isRunning) {
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
                }
                break;
            case 0xF20DFF00: // 测试电机B
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
            case 0xE718FF00: // 原模式切换键 -> 现改为切换旋转方向
                if (!isRunning) {
                    clockwise = !clockwise; // 切换方向
                    lcd.clear();
                    lcd.print(F("Direction:"));
                    lcd.setCursor(0, 1);
                    lcd.print(clockwise ? F("CW") : F("CCW"));
                    delay(800);
                    updateDisplay();
                }
                break;
            case 0xA15EFF00: // 系统复位
                resetSystem();
                break;
            case 0xF708FF00: // 版本信息
                lcd.clear();
                lcd.print(F("Dealer v36.0"));
                lcd.setCursor(0, 1);
                lcd.print(F("DirSwitch"));
                delay(800);
                updateDisplay();
                break;
            case 0xE31CFF00: // 减B超时
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
            case 0xA55AFF00: // 加B超时
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
        // 数字命令不再用于设置圈时间
        lcd.clear();
        lcd.print(F("Not supported"));
        delay(800);
        updateDisplay();
        return;
    }
    switch (command[0]) {
        case 'v': case 'V':
            lcd.clear();
            lcd.print(F("Dealer v36.0"));
            lcd.setCursor(0, 1);
            lcd.print(F("DirSwitch"));
            delay(800);
            updateDisplay();
            break;
        case 'p': case 'P':
            playerCount++; if (playerCount > 8) playerCount = 2;
            cardsPerRevolution = 8.0 / playerCount;
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
            cardsPerRevolution = 8.0 / playerCount;
            updateDisplay();
            break;
        case 't': case 'T':
            if (isRunning) stopDealing();
            else {
                stopAllMotors();
                disableMotorDriver();
                currentState = STATE_IDLE;
                lcd.clear();
                lcd.print(F("Stop"));
                delay(800);
                updateDisplay();
            }
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
        case 'm': case 'M':   // 原模式切换 -> 现改为切换方向
            if (!isRunning) {
                clockwise = !clockwise;
                lcd.clear();
                lcd.print(F("Direction:"));
                lcd.setCursor(0, 1);
                lcd.print(clockwise ? F("CW") : F("CCW"));
                delay(800);
                updateDisplay();
            }
            break;
        case 'z': case 'Z':
            resetSystem();
            break;
        case '+': case '-':
            // 圈时间调整无效
            lcd.clear();
            lcd.print(F("No circle adj"));
            delay(800);
            updateDisplay();
            break;
        case 'h': case 'H':
            Serial.println(F("=== Card Dealer Cmds (Photo ver) ==="));
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
    accumulatedTarget = 0.0;
    photoBTriggered = false;
    serialBufferIndex = 0;
    serialBuffer[0] = '\0';
    lcd.clear();
    lcd.print(F("Reset"));
    lcd.setCursor(0, 1);
    lcd.print(F("Photo Mode"));
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
    if (isRunning) {
        switch (currentState) {
            case STATE_B_RUNNING: lcd.print(F("B")); break;
            case STATE_A_RUNNING: lcd.print(F("A")); break;
            default: lcd.print(F("R"));
        }
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
    lcd.print(F("Dealer v36.0"));
    lcd.setCursor(0, 1);
    lcd.print(F("DirSwitch"));

#if DEBUG
    Serial.begin(9600);
    delay(500);
    Serial.println(F("=== System Startup (Photo Switch + Direction) ==="));
    Serial.println(F("Players: 2-8, Direction: CW (default)"));
#endif

#if ENABLE_INFRA
    IrReceiver.begin(IR_RECEIVE_PIN);
#endif

    // 初始化参数
    totalCards = deckCount * (hasJokers ? 54 : 52);
    cardsPerRevolution = 8.0 / playerCount;

    lastPhotoAState = digitalRead(PHOTO_A_PIN);
    lastPhotoBState = digitalRead(PHOTO_B_PIN);

    serialBufferIndex = 0;
    serialBuffer[0] = '\0';

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
    if (isRunning) {
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
        Serial.println(nextStopCount);
        lastDebugTime = millis();
    }
#endif
    delay(10);
}