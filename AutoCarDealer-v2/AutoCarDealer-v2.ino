// ==================== 系统配置宏 ====================
#define ENABLE_INFRA 1            // 启用红外遥控
#define ENABLE_INFRA2 1           // 启用第二个红外遥控（默认关闭，需手动设为1）
#define ENABLE_KEYBOARD 0         // 启用串口键盘指令
#define DEBUG 0                    // 调试信息输出
#define IR_DEBUG 0                 // 红外详细调试（开启以便观察干扰）

// ==================== 脉冲宽度检测阈值（ms）====================
#define MIN_CARD_PULSE 20    // 最小有效脉冲宽度，小于此值视为干扰
#define MAX_CARD_PULSE 150   // 最大单牌脉冲宽度，超过此值可能为多张粘连（报警但只计一张）

// ==================== 红外遥控键码定义（第一遥控器）====================
#if ENABLE_INFRA
#define IR_DEC_PLAYER     0xBA45FF00 //ch-
#define IR_RESET          0xB946FF00 //ch
#define IR_INC_PLAYER     0xB847FF00 //ch+
#define IR_INC_DECK       0xBB44FF00 //prev
#define IR_TOGGLE_JOKER   0xBF40FF00 //next
#define IR_START          0xBC43FF00 //play
#define IR_STOP           0xF609FF00 //EQ
#define IR_REMAIN_CARDS   0xE916FF00 //0
#define IR_TEST_MOTOR_A   0xE619FF00 //100+
#define IR_TEST_MOTOR_B   0xF20DFF00 //200+
#define IR_CALIBRATE      0xF30CFF00 //1
#define IR_TOGGLE_MODE    0xE718FF00 //2
#define IR_SYSTEM_RESET   0xA15EFF00 //3
#define IR_VERSION        0xF708FF00 //4
#define IR_REBOOT         0xB54AFF00 //重启键（按钮9）

// 新增比分调整键（已用IR_DEBUG获取实际值）
#define IR_SCORE_X_INC    0xE31CFF00 // 按钮5：X加1
#define IR_SCORE_X_DEC    0xA55AFF00 // 按钮6：X减1
#define IR_SCORE_Y_INC    0xBD42FF00 // 按钮7：Y加1
#define IR_SCORE_Y_DEC    0xAD52FF00 // 按钮8：Y减1
#endif

// ==================== 第二个红外遥控器按键定义 ====================
#if ENABLE_INFRA2
#define IR2_START        0xFE017F80 //发牌，暂停，恢复，等同于IR_START
#define IR2_STOP         0xED127F80 //停止发牌，等同于 IR_STOP
#define IR2_INC_PLAYER   0xFA057F80 //等同于IR_INC_PLAYER
#define IR2_REMAIN_CARDS 0xE41B7F80 //等同于IR_REMAIN_CARDS
#define IR2_INC_DECK     0xEE117F80 //等同于IR_INC_DECK
#define IR2_TOGGLE_JOKER 0xFC037F80 //等同于IR_TOGGLE_JOKER
#define IR2_TOGGLE_MODE  0xF6097F80 //等同于IR_TOGGLE_MODE
#endif

// ==================== 库包含 ====================
#include <LiquidCrystal.h>

#if ENABLE_INFRA || ENABLE_INFRA2
#define EXCLUDE_EXOTIC_PROTOCOLS
#define NO_LED_FEEDBACK_CODE
#include <IRremote.hpp>
#define IR_RECEIVE_PIN A1
#endif

// ==================== TB6612FNG 引脚定义 ====================
#define MOTOR_A_IN1 6
#define MOTOR_A_IN2 7
#define MOTOR_A_PWM 9
#define MOTOR_B_IN1 A2
#define MOTOR_B_IN2 A3
#define MOTOR_B_PWM 10
#define MOTOR_STBY 8

// ==================== 光电开关引脚定义 ====================
#define PHOTO_A_PIN A4        // 电机A计数光电开关
#define PHOTO_B_PIN A0        // 电机B发牌检测光电开关

// ==================== 模块实例化 ====================
LiquidCrystal lcd(12, 11, 5, 4, 3, 2);

#if ENABLE_INFRA || ENABLE_INFRA2
IRrecv irrecv(IR_RECEIVE_PIN);
decode_results results;
#endif

// ==================== 系统状态枚举 ====================
enum SystemState {
    STATE_IDLE,
    STATE_B_RUNNING,   // 发牌电机运行（等待出牌）
    STATE_A_RUNNING    // 旋转电机运行
};

// ==================== 全局变量 ====================
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
unsigned long motorBTimeout = 6000;      // 电机B超时保护（6秒，基于最后出牌时间）
unsigned long motorATimeout = 3000;      // 电机A超时保护（3秒）

// ---- 旋转方向（true=顺时针，false=逆时针）----
bool clockwise = true;  // 缺省方向改为顺时针

// ---- 光电开关A相关 ----
volatile uint16_t photoACount = 0;
uint16_t nextStopCount = 0;
uint8_t lastPhotoAState = HIGH;
unsigned long photoADebounce = 0;
unsigned long motorAStartTime = 0;
unsigned long lastPhotoATime = 0;         // 最后一次光电A计数时间，用于停滞检测

// ---- 光电开关B相关 ----
uint8_t lastPhotoBState = HIGH;
unsigned long photoBStartTime = 0;        // 遮挡开始时间（下降沿）
bool photoBBlocked = false;               // 是否处于遮挡状态
unsigned long photoBDebounce = 0;
unsigned long lastCardTime = 0;
unsigned long cardIgnoreUntil = 0;        // 发牌后消隐窗口结束时间

// ---- 调试定时器 ----
unsigned long lastDebugTime = 0;

// ---- 串口缓冲区 ----
const int SERIAL_BUFFER_SIZE = 32;
char serialBuffer[SERIAL_BUFFER_SIZE];
int serialBufferIndex = 0;
unsigned long lastSerialCharTime = 0;
const unsigned long SERIAL_TIMEOUT = 100;

// ==================== 红外防抖变量 ====================
unsigned long lastIRCommandTime = 0;
unsigned long lastIRCommandValue = 0;
const unsigned long IR_DEBOUNCE_TIME = 300;

// ==================== 新增比分变量 ====================
uint8_t scoreX = 2;   // 默认02
uint8_t scoreY = 2;   // 默认02

// ==================== 软件复位函数指针 ====================
void (*resetFunc)(void) = 0;  // 指向地址0，调用即复位

// ==================== 基于16计数每圈的发牌位置表 ====================
const uint8_t basePositions16[9][8] = {
    {0},       // 0人（未用）
    {0},       // 1人（未用）
    {0, 8},    // 2人
    {0, 5, 11}, // 3人
    {0, 4, 8, 12}, // 4人
    {0, 3, 7, 10, 13}, // 5人
    {0, 3, 6, 8, 11, 14}, // 6人
    {0, 3, 5, 7, 9, 11, 13}, // 7人
    {0, 2, 4, 6, 8, 10, 12, 14} // 8人
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
void updatePhotoA();
void updatePhotoB();
void handleMotorState();
void resetSystem();
void performInitialHoming();
void calibrateMotorA();
void resetDealCounts();
#if ENABLE_INFRA || ENABLE_INFRA2
void processInfraredInput();
#endif
void processSerialInput();
void handleSerialCommand(const char* command);
void updateDisplay();
void showStatusMessage(const char* message);
void updateNextStopCount();

// ==================== 电机控制函数（优化快速停止）====================
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
    analogWrite(MOTOR_A_PWM, 0);
    digitalWrite(MOTOR_B_IN1, HIGH);
    digitalWrite(MOTOR_B_IN2, HIGH);
    analogWrite(MOTOR_B_PWM, 0);
    delay(10);  // 略微等待电机惯性停止
}

void controlMotorA(bool enable) {
    if (enable) {
        digitalWrite(MOTOR_A_IN1, HIGH);
        digitalWrite(MOTOR_A_IN2, HIGH);
        analogWrite(MOTOR_A_PWM, 255);
        delayMicroseconds(500);
        if (clockwise) {
            digitalWrite(MOTOR_A_IN1, LOW);
            digitalWrite(MOTOR_A_IN2, HIGH);
        } else {
            digitalWrite(MOTOR_A_IN1, HIGH);
            digitalWrite(MOTOR_A_IN2, LOW);
        }
        analogWrite(MOTOR_A_PWM, MOTOR_A_SPEED);
        motorAStartTime = millis();
        lastPhotoATime = millis();
        delayMicroseconds(500);
    } else {
        digitalWrite(MOTOR_A_IN1, HIGH);
        digitalWrite(MOTOR_A_IN2, HIGH);
        analogWrite(MOTOR_A_PWM, 0);
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
        analogWrite(MOTOR_B_PWM, 0);
    }
}

// ==================== 光电开关A计数（双边沿检测）====================
void updatePhotoA() {
    if (currentState != STATE_A_RUNNING) return;

    uint8_t currentStateA = digitalRead(PHOTO_A_PIN);
    if (currentStateA != lastPhotoAState) {
        if (millis() - photoADebounce > 10) {
            photoADebounce = millis();
            photoACount++;
            lastPhotoATime = millis();
#if DEBUG
            Serial.print(F("PhotoA count: "));
            Serial.println(photoACount);
#endif
        }
    }
    lastPhotoAState = currentStateA;
}

// ==================== 光电开关B检测（脉冲宽度检测）====================
void updatePhotoB() {
    if (isRunning != 1 || currentState != STATE_B_RUNNING) return;
    if (millis() < cardIgnoreUntil) return;

    uint8_t currentStateB = digitalRead(PHOTO_B_PIN);

    if (currentStateB == LOW && lastPhotoBState == HIGH) {
        if (millis() - photoBDebounce > 50) {
            photoBDebounce = millis();
            photoBStartTime = millis();
            photoBBlocked = true;
#if DEBUG
            Serial.println(F("PhotoB start"));
#endif
        }
    }

    if (currentStateB == HIGH && lastPhotoBState == LOW) {
        if (millis() - photoBDebounce > 50) {
            photoBDebounce = millis();
            if (photoBBlocked) {
                unsigned long blockDuration = millis() - photoBStartTime;
                photoBBlocked = false;

#if DEBUG
                Serial.print(F("PhotoB end, duration="));
                Serial.println(blockDuration);
#endif

                if (blockDuration < MIN_CARD_PULSE) {
#if DEBUG
                    Serial.println(F("Ignore short pulse"));
#endif
                } else if (blockDuration > MAX_CARD_PULSE) {
#if DEBUG
                    Serial.println(F("Warning: long pulse (multiple cards?)"));
#endif
                    processCard();
                } else {
                    processCard();
                }
            }
        }
    }

    lastPhotoBState = currentStateB;
}

// ==================== 处理一张牌发出（优化：电机B不停止）====================
void processCard() {
    cardIgnoreUntil = millis() + 300;  // 消隐窗口300ms
    dealtCards++;
    lastCardTime = millis();

#if DEBUG
    Serial.print(F("Card dealt: "));
    Serial.println(dealtCards);
#endif

    if (dealtCards >= totalCards && totalCards > 0) {
        stopDealing();
        showStatusMessage("All Done!");
        delay(500);
        updateDisplay();
        return;
    }

    // 电机B保持转动，不停止
    updateNextStopCount();

    // 如果需要旋转，启动电机A（电机B继续转）
    if (nextStopCount > photoACount) {
        controlMotorA(true);
        currentState = STATE_A_RUNNING;
    } else {
        // 不需要旋转，电机B继续转，状态保持B_RUNNING
        currentState = STATE_B_RUNNING;
    }
}

// ==================== 上电归位 ====================
void performInitialHoming() {
    enableMotorDriver();
    delay(100);

    lcd.clear();
    lcd.print("Homing MotorA...");
    controlMotorA(true);
    unsigned long startTime = millis();
    uint8_t lastState = digitalRead(PHOTO_A_PIN);
    bool found = false;
    while (millis() - startTime < 10000) {
        uint8_t curState = digitalRead(PHOTO_A_PIN);
        if (curState == LOW && lastState == HIGH) {
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
    lcd.clear();
}

// ==================== 电机A校准归位 ====================
void calibrateMotorA() {
    stopAllMotors();
    enableMotorDriver();
    delay(100);

    lcd.setCursor(0, 1);
    lcd.print("Calib A...      ");

    controlMotorA(true);
    unsigned long startTime = millis();
    uint8_t lastState = digitalRead(PHOTO_A_PIN);
    bool found = false;
    while (millis() - startTime < 10000) {
        uint8_t curState = digitalRead(PHOTO_A_PIN);
        if (curState == LOW && lastState == HIGH) {
            found = true;
            break;
        }
        lastState = curState;
        delay(10);
    }
    controlMotorA(false);

    lcd.setCursor(0, 1);
    if (found) {
        lcd.print("A OK           ");
    } else {
        lcd.print("A TO           ");
    }
    delay(800);
    updateDisplay();
}

// ==================== 重置已发牌数和计数 ====================
void resetDealCounts() {
    dealtCards = 0;
    photoACount = 0;
    nextStopCount = 0;
    cardIgnoreUntil = 0;
    if (isRunning != 0) {
        stopDealing();
    }
    updateDisplay();
    lcd.setCursor(0, 1);
    lcd.print(F("Counts reset"));
    delay(500);
    updateDisplay();
}

// ==================== 计算下一次停止计数 ====================
void updateNextStopCount() {
    if (playerCount < 2 || playerCount > 8) return;

    uint16_t nextIndex = dealtCards;
    uint8_t circle = nextIndex / playerCount;
    uint8_t posInCircle = nextIndex % playerCount;
    nextStopCount = basePositions16[playerCount][posInCircle] + circle * 16;

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

// ==================== 暂停/恢复 ====================
void pauseDealing() {
    if (isRunning == 1) {
        stopAllMotors();
        isRunning = 2;
#if DEBUG
        Serial.println(F("Paused"));
#endif
        updateDisplay();
    }
}

void resumeDealing() {
    if (isRunning == 2) {
        enableMotorDriver();
        // 恢复时先启动电机B（常转）
        controlMotorB(true);
        if (currentState == STATE_A_RUNNING) {
            // 如果之前是在旋转状态，也要启动电机A
            controlMotorA(true);
        }
        isRunning = 1;
#if DEBUG
        Serial.println(F("Resumed"));
#endif
        updateDisplay();
    }
}

// ==================== 开始发牌 ====================
void startDealing() {
    if (isRunning == 0) {
        if (totalCards <= 0) {
            showStatusMessage("No Cards!");
            delay(500);
            updateDisplay();
            return;
        }
        isRunning = 1;
        dealtCards = 0;
        photoACount = 0;
        nextStopCount = 0;
        cardIgnoreUntil = 0;

        stopAllMotors();
        enableMotorDriver();

        // 启动电机B并保持运转
        controlMotorB(true);
        lastCardTime = millis();
        currentState = STATE_B_RUNNING;
        showStatusMessage("Start");
        delay(300);
        updateDisplay();
    } else if (isRunning == 1) {
        pauseDealing();
    } else if (isRunning == 2) {
        resumeDealing();
    }
}

// ==================== 完全停止 ====================
void stopDealing() {
    isRunning = 0;
    currentState = STATE_IDLE;
    photoACount = 0;
    nextStopCount = 0;
    dealtCards = 0;
    cardIgnoreUntil = 0;
    stopAllMotors();
    updateDisplay();
}

// ==================== 状态机处理（电机B常转）====================
void handleMotorState() {
    // 电机B超时检测（基于最后出牌时间）
    if (isRunning == 1 && (millis() - lastCardTime > motorBTimeout)) {
        pauseDealing();
        showStatusMessage("B Timeout");
        delay(500);
        updateDisplay();
        return;
    }

    switch (currentState) {
        case STATE_B_RUNNING:
            updatePhotoB();  // 检测出牌
            break;

        case STATE_A_RUNNING:
            updatePhotoA();  // 检测旋转计数

            // 检查是否达到目标
            if (photoACount >= nextStopCount) {
                controlMotorA(false);  // 停止电机A
                if (dealtCards < totalCards) {
                    // 电机B继续转，状态切回B_RUNNING
                    currentState = STATE_B_RUNNING;
                } else {
                    stopDealing();  // 发完牌停止
                }
            }

            // 电机A停滞检测
            if (millis() - lastPhotoATime > 1000 && photoACount < nextStopCount) {
                controlMotorA(false);
                pauseDealing();
                showStatusMessage("A Stuck");
                delay(500);
                updateDisplay();
            }

            // 电机A超时保护
            if (millis() - motorAStartTime > motorATimeout && isRunning == 1) {
                controlMotorA(false);
                pauseDealing();
                showStatusMessage("A Timeout");
                delay(500);
                updateDisplay();
            }
            break;

        default:
            break;
    }
}

// ==================== 红外遥控处理（增加防抖和比分调整）====================
#if ENABLE_INFRA || ENABLE_INFRA2
void processInfraredInput() {
    if (IrReceiver.decode()) {
        unsigned long irValue = IrReceiver.decodedIRData.decodedRawData;
#if IR_DEBUG
        Serial.print(F("IR Received: 0x"));
        Serial.println(irValue, HEX);
#endif
        if (irValue == 0xFFFFFFFF) {
            IrReceiver.resume();
            return;
        }

        // 红外防抖
        unsigned long now = millis();
        if (irValue == lastIRCommandValue && (now - lastIRCommandTime < IR_DEBOUNCE_TIME)) {
#if IR_DEBUG
            Serial.println(F("IR debounce: ignored"));
#endif
            IrReceiver.resume();
            return;
        }
        lastIRCommandTime = now;
        lastIRCommandValue = irValue;

        // 非空闲状态（运行或暂停）只响应停止、发牌和比分调整
        if (isRunning != 0) {
            bool isStop = false, isStart = false;
#if ENABLE_INFRA
            if (irValue == IR_STOP) isStop = true;
            if (irValue == IR_START) isStart = true;
#endif
#if ENABLE_INFRA2
            if (irValue == IR2_STOP) isStop = true;
            if (irValue == IR2_START) isStart = true;
#endif
            if (isStop) {
                stopDealing();
            } else if (isStart) {
                startDealing();
            }
            // 即使运行中也允许调整比分
#if ENABLE_INFRA
            else if (irValue == IR_SCORE_X_INC) {
                scoreX++; if (scoreX > 14) scoreX = 2;
                updateDisplay();
            } else if (irValue == IR_SCORE_X_DEC) {
                if (scoreX == 2) scoreX = 14; else scoreX--;
                updateDisplay();
            } else if (irValue == IR_SCORE_Y_INC) {
                scoreY++; if (scoreY > 14) scoreY = 2;
                updateDisplay();
            } else if (irValue == IR_SCORE_Y_DEC) {
                if (scoreY == 2) scoreY = 14; else scoreY--;
                updateDisplay();
            }
#endif
            // 其他键忽略
            IrReceiver.resume();
            return;
        }

        // 空闲状态处理
#if ENABLE_INFRA
        if (irValue == IR_DEC_PLAYER) {
            if (playerCount > 2) playerCount--; else playerCount = 8;
            resetDealCounts();
            updateDisplay();
        } else
#endif
#if ENABLE_INFRA
        if (irValue == IR_RESET) {
            playerCount = 4; deckCount = 3; hasJokers = 1; remainCards = 0;
            totalCards = deckCount * (hasJokers ? 54 : 52);
            resetDealCounts();
            updateDisplay();
        } else
#endif
#if defined(IR_INC_PLAYER) || defined(IR2_INC_PLAYER)
        if (
#if ENABLE_INFRA
            irValue == IR_INC_PLAYER ||
#endif
#if ENABLE_INFRA2
            irValue == IR2_INC_PLAYER ||
#endif
            false) {
            playerCount++; if (playerCount > 8) playerCount = 2;
            resetDealCounts();
            updateDisplay();
        } else
#endif
#if defined(IR_INC_DECK) || defined(IR2_INC_DECK)
        if (
#if ENABLE_INFRA
            irValue == IR_INC_DECK ||
#endif
#if ENABLE_INFRA2
            irValue == IR2_INC_DECK ||
#endif
            false) {
            deckCount++; if (deckCount > 3) deckCount = 1;
            totalCards = deckCount * (hasJokers ? 54 : 52);
            resetDealCounts();
            updateDisplay();
        } else
#endif
#if defined(IR_TOGGLE_JOKER) || defined(IR2_TOGGLE_JOKER)
        if (
#if ENABLE_INFRA
            irValue == IR_TOGGLE_JOKER ||
#endif
#if ENABLE_INFRA2
            irValue == IR2_TOGGLE_JOKER ||
#endif
            false) {
            hasJokers = !hasJokers;
            totalCards = deckCount * (hasJokers ? 54 : 52);
            resetDealCounts();
            updateDisplay();
        } else
#endif
#if defined(IR_START) || defined(IR2_START)
        if (
#if ENABLE_INFRA
            irValue == IR_START ||
#endif
#if ENABLE_INFRA2
            irValue == IR2_START ||
#endif
            false) {
            startDealing();
        } else
#endif
#if defined(IR_STOP) || defined(IR2_STOP)
        if (
#if ENABLE_INFRA
            irValue == IR_STOP ||
#endif
#if ENABLE_INFRA2
            irValue == IR2_STOP ||
#endif
            false) {
            lcd.clear();
            lcd.print(F("Idle"));
            delay(800);
            updateDisplay();
        } else
#endif
#if defined(IR_REMAIN_CARDS) || defined(IR2_REMAIN_CARDS)
        if (
#if ENABLE_INFRA
            irValue == IR_REMAIN_CARDS ||
#endif
#if ENABLE_INFRA2
            irValue == IR2_REMAIN_CARDS ||
#endif
            false) {
            remainCards += playerCount;
            if (remainCards > playerCount * 4) remainCards = 0;
            totalCards = deckCount * (hasJokers ? 54 : 52);
            if (remainCards > 0 && totalCards > remainCards) totalCards -= remainCards;
            resetDealCounts();
            updateDisplay();
        } else
#endif
#if ENABLE_INFRA
        if (irValue == IR_TEST_MOTOR_A) {
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
            while (!testComplete && millis() - testStart < 3000) {
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
        } else
#endif
#if ENABLE_INFRA
        if (irValue == IR_TEST_MOTOR_B) {
            enableMotorDriver();
            controlMotorB(true);
            lcd.clear();
            lcd.print(F("MotorB Test"));
            lcd.setCursor(0, 1);
            lcd.print(F("Wait card..."));
            {
                unsigned long testStartB = millis();
                bool cardDetected = false;
                lastPhotoBState = digitalRead(PHOTO_B_PIN);
                while (millis() - testStartB < 5000) {
                    uint8_t cur = digitalRead(PHOTO_B_PIN);
                    if (cur == LOW && lastPhotoBState == HIGH) {
                        cardDetected = true;
                        break;
                    }
                    lastPhotoBState = cur;
                    delay(10);
                }
                controlMotorB(false);
                lcd.clear();
                if (cardDetected) lcd.print(F("B OK"));
                else lcd.print(F("B TO"));
            }
            delay(800);
            updateDisplay();
        } else
#endif
#if ENABLE_INFRA
        if (irValue == IR_CALIBRATE) {
            calibrateMotorA();
        } else
#endif
#if defined(IR_TOGGLE_MODE) || defined(IR2_TOGGLE_MODE)
        if (
#if ENABLE_INFRA
            irValue == IR_TOGGLE_MODE ||
#endif
#if ENABLE_INFRA2
            irValue == IR2_TOGGLE_MODE ||
#endif
            false) {
            clockwise = !clockwise;
            resetDealCounts();
            lcd.clear();
            lcd.print(F("Direction:"));
            lcd.setCursor(0, 1);
            lcd.print(clockwise ? F("CCW") : F("CW"));
            lcd.setCursor(0, 1);
            lcd.print(F("Please Calib A"));
            delay(1500);
            updateDisplay();
        } else
#endif
#if ENABLE_INFRA
        if (irValue == IR_SYSTEM_RESET) {
            resetSystem();
        } else
#endif
#if ENABLE_INFRA
        // 重启键 IR_REBOOT 调用 resetFunc()
        if (irValue == IR_REBOOT) {
#if IR_DEBUG
            Serial.println(F("Rebooting via resetFunc..."));
#endif
            delay(100);
            resetFunc();
        } else
#endif
#if ENABLE_INFRA
        if (irValue == IR_VERSION) {
            lcd.clear();
            lcd.print(F("Dealer v40.4"));
            lcd.setCursor(0, 1);
            lcd.print(F("16-edge mode"));
            delay(800);
            updateDisplay();
        } else
#endif
        // 空闲状态下也处理比分调整
#if ENABLE_INFRA
        if (irValue == IR_SCORE_X_INC) {
            scoreX++; if (scoreX > 14) scoreX = 2;
            updateDisplay();
        } else if (irValue == IR_SCORE_X_DEC) {
            if (scoreX == 2) scoreX = 14; else scoreX--;
            updateDisplay();
        } else if (irValue == IR_SCORE_Y_INC) {
            scoreY++; if (scoreY > 14) scoreY = 2;
            updateDisplay();
        } else if (irValue == IR_SCORE_Y_DEC) {
            if (scoreY == 2) scoreY = 14; else scoreY--;
            updateDisplay();
        } else
#endif
        {
#if IR_DEBUG
            Serial.print(F("Unknown IR: 0x"));
            Serial.println(irValue, HEX);
#endif
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
            lcd.print(F("Dealer v40.4"));
            lcd.setCursor(0, 1);
            lcd.print(F("16-edge mode"));
            delay(800);
            updateDisplay();
            break;
        case 'p': case 'P':
            playerCount++; if (playerCount > 8) playerCount = 2;
            resetDealCounts();
            updateDisplay();
            break;
        case 'd': case 'D':
            deckCount++; if (deckCount > 3) deckCount = 1;
            totalCards = deckCount * (hasJokers ? 54 : 52);
            resetDealCounts();
            updateDisplay();
            break;
        case 'j': case 'J':
            hasJokers = !hasJokers;
            totalCards = deckCount * (hasJokers ? 54 : 52);
            resetDealCounts();
            updateDisplay();
            break;
        case 'r': case 'R':
            remainCards += playerCount;
            if (remainCards > playerCount * 4) remainCards = 0;
            totalCards = deckCount * (hasJokers ? 54 : 52);
            if (remainCards > 0 && totalCards > remainCards) totalCards -= remainCards;
            resetDealCounts();
            updateDisplay();
            break;
        case 's': case 'S':
            startDealing();
            break;
        case 'c': case 'C':
            playerCount = 4; deckCount = 3; hasJokers = 1; remainCards = 0;
            stopDealing();
            totalCards = deckCount * (hasJokers ? 54 : 52);
            resetDealCounts();
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
                while (!testComplete && millis() - testStart < 3000) {
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
                lastPhotoBState = digitalRead(PHOTO_B_PIN);
                while (millis() - testStart < 5000) {
                    uint8_t cur = digitalRead(PHOTO_B_PIN);
                    if (cur == LOW && lastPhotoBState == HIGH) {
                        cardDetected = true;
                        break;
                    }
                    lastPhotoBState = cur;
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
        case 'm': case 'M':
            clockwise = !clockwise;
            resetDealCounts();
            lcd.clear();
            lcd.print(F("Direction:"));
            lcd.setCursor(0, 1);
            lcd.print(clockwise ? F("CCW") : F("CW"));
            lcd.setCursor(0, 1);
            lcd.print(F("Calib A"));
            delay(1500);
            updateDisplay();
            break;
        case 'z': case 'Z':
            resetSystem();
            break;
        case 'x': case 'X':
            resetFunc();
            break;
        case '+': case '-':
            lcd.clear();
            lcd.print(F("No circle adj"));
            delay(800);
            updateDisplay();
            break;
        case 'h': case 'H':
            Serial.println(F("=== Card Dealer Cmds ==="));
            Serial.println(F("P/p: Player++  D/d: Deck++  J/j: Joker toggle"));
            Serial.println(F("R/r: RemainCards  S/s: Start/Pause  C/c: Clear settings"));
            Serial.println(F("T/t: Stop  A/a: Test motorA  B/b: Test motorB"));
            Serial.println(F("M/m: Direction toggle  Z/z: System reset  X/x: Reboot"));
            Serial.println(F("H/h: Help"));
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

// ==================== 系统重启（保留用于其他场景）====================
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
    cardIgnoreUntil = 0;
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
    // 第一行：Px Dx J 比分 方向
    lcd.setCursor(0, 0);
    lcd.print(F("P"));
    lcd.print(playerCount);
    lcd.print(F(" D"));
    lcd.print(deckCount);
    lcd.print(F(" "));
    lcd.print(hasJokers ? F("J") : F("N"));
    lcd.print(F(" "));
    // 显示比分，两位数字带前导零
    if (scoreX < 10) lcd.print('0');
    lcd.print(scoreX);
    lcd.print(':');
    if (scoreY < 10) lcd.print('0');
    lcd.print(scoreY);
    lcd.print(F(" "));
    lcd.print(clockwise ? F("CCW") : F("CW"));

    // 第二行：D:已发/总数 状态 C:计数
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
        lcd.print(F("P"));
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
    lcd.print(F("Dealer v40.4"));
    lcd.setCursor(0, 1);
    lcd.print(F("Homing"));

#if DEBUG
    Serial.begin(9600);
    delay(500);
    Serial.println(F("=== System Startup (16-edge mode) ==="));
    Serial.println(F("Players: 2-8, Direction: CCW (default)"));
#endif

#if ENABLE_INFRA || ENABLE_INFRA2
    IrReceiver.begin(IR_RECEIVE_PIN);
#endif

    totalCards = deckCount * (hasJokers ? 54 : 52);

    lastPhotoAState = digitalRead(PHOTO_A_PIN);
    lastPhotoBState = digitalRead(PHOTO_B_PIN);

    serialBufferIndex = 0;
    serialBuffer[0] = '\0';

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
#if ENABLE_INFRA || ENABLE_INFRA2
    processInfraredInput();
#endif
    if (isRunning == 1) {
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
        Serial.print(clockwise ? "CCW" : "CW");
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