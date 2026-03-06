// ==================== 系统配置宏 ====================
#define ENABLE_INFRA 1            // 启用红外遥控
#define ENABLE_INFRA2 1           // 启用第二个红外遥控（默认关闭，需手动设为1）
#define ENABLE_KEYBOARD 1         // 启用串口键盘指令
#define DEBUG 1                    // 调试信息输出
#define IR_DEBUG 1                 // 红外详细调试（关闭减少串口干扰）

// ==================== 红外遥控键码定义（第一遥控器）====================
#if ENABLE_INFRA
#define IR_DEC_PLAYER     0xBA45FF00 //ch-
#define IR_RESET          0xB946FF00 //ch
#define IR_INC_PLAYER     0xB847FF00 //ch+
#define IR_INC_DECK       0xBB44FF00 //prev
#define IR_TOGGLE_JOKER   0xBF40FF00 //next
#define IR_START          0xBC43FF00 //play
//#define IR_DEC_CIRCLE_TIME 0xF807FF00 //-
//#define IR_INC_CIRCLE_TIME 0xEA15FF00 //+
#define IR_STOP           0xF609FF00 //EQ
#define IR_REMAIN_CARDS   0xE916FF00 //0
#define IR_TEST_MOTOR_A   0xE619FF00 //100+
#define IR_TEST_MOTOR_B   0xF20DFF00 //200+
#define IR_CALIBRATE      0xF30CFF00 //1
#define IR_TOGGLE_MODE    0xE718FF00 //2
#define IR_SYSTEM_RESET   0xA15EFF00 //3
#define IR_VERSION        0xF708FF00 //4
//#define IR_BTM            0xE31CFF00 //5
//#define IR_BTP            0xA55AFF00 //6
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
#define EXCLUDE_EXOTIC_PROTOCOLS   // 屏蔽多余红外协议，减小代码
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
    STATE_B_RUNNING,   // 发牌电机运行（常转）
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
unsigned long motorBTimeout = 8000;      // 电机B超时保护（8秒，基于最后出牌时间）
unsigned long motorATimeout = 10000;      // 电机A超时保护（10秒）
unsigned long maxRotateTime = 2000;       // 电机A最大允许旋转时间（根据实测调整）

// ---- 旋转方向 ----
bool clockwise = true;                    // true = 顺时针，false = 逆时针

// ---- 光电开关A（旋转计数）相关 ----
volatile uint16_t photoACount = 0;        // 当前旋转过程中累计的遮挡次数
uint16_t nextStopCount = 0;               // 下一次应该停止的计数目标
uint8_t lastPhotoAState = HIGH;            // 上次读取的光电A状态
unsigned long photoADebounce = 0;          // 防抖动时间
unsigned long motorAStartTime = 0;         // 电机A开始旋转的时间（用于超时）

// ---- 光电开关B（发牌检测）相关 ----
uint8_t lastPhotoBState = HIGH;
unsigned long photoBDebounce = 0;
unsigned long lastCardTime = 0;            // 最后一张牌发出的时间（用于电机B超时）

// ---- 调试定时器 ----
unsigned long lastDebugTime = 0;

// ---- 串口缓冲区 ----
const int SERIAL_BUFFER_SIZE = 32;
char serialBuffer[SERIAL_BUFFER_SIZE];
int serialBufferIndex = 0;
unsigned long lastSerialCharTime = 0;
const unsigned long SERIAL_TIMEOUT = 100;

// ==================== 基础位置表（第0圈）====================
const uint8_t basePositions[9][8] = {
    {0},           // 占位，不使用
    {0},           // 玩家数1
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
void updatePhotoB();                 // 检测光电B上升沿（立即处理发牌）
void handleMotorState();
void resetSystem();
void performInitialHoming();          // 上电归位（清屏）
void calibrateMotorA();               // 电机A校准归位（下降沿，不清屏）
#if ENABLE_INFRA || ENABLE_INFRA2
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

// 优化电机A控制：极短刹车和启动延时（500微秒），提升响应速度
void controlMotorA(bool enable) {
    if (enable) {
        // 先刹车再启动，消除不确定状态
        digitalWrite(MOTOR_A_IN1, HIGH);
        digitalWrite(MOTOR_A_IN2, HIGH);
        analogWrite(MOTOR_A_PWM, 255);
        delayMicroseconds(500);          // 极短刹车
        if (clockwise) {
            digitalWrite(MOTOR_A_IN1, LOW);
            digitalWrite(MOTOR_A_IN2, HIGH);
        } else {
            digitalWrite(MOTOR_A_IN1, HIGH);
            digitalWrite(MOTOR_A_IN2, LOW);
        }
        analogWrite(MOTOR_A_PWM, MOTOR_A_SPEED);
        motorAStartTime = millis();       // 记录开始时间，用于超时
        delayMicroseconds(500);            // 极短启动延时
    } else {
        // 停止电机A（刹车）
        digitalWrite(MOTOR_A_IN1, HIGH);
        digitalWrite(MOTOR_A_IN2, HIGH);
        analogWrite(MOTOR_A_PWM, 255);
        delay(5);
        analogWrite(MOTOR_A_PWM, 0);
    }
}

void controlMotorB(bool enable) {
    if (enable) {
        digitalWrite(MOTOR_B_IN1, LOW);
        digitalWrite(MOTOR_B_IN2, HIGH);
        analogWrite(MOTOR_B_PWM, MOTOR_B_SPEED);
    } else {
        // 停止电机B（刹车）
        digitalWrite(MOTOR_B_IN1, HIGH);
        digitalWrite(MOTOR_B_IN2, HIGH);
        analogWrite(MOTOR_B_PWM, 255);
        delay(5);
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
        if (millis() - photoADebounce > 10) { // 防抖时间缩短为10ms
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
// 检测到牌离开（上升沿）时立即处理发牌计数和电机A控制
void updatePhotoB() {
    // 只在电机B运行时检测（即系统运行中）
    if (isRunning != 1) return;

    uint8_t currentStateB = digitalRead(PHOTO_B_PIN);
    // 检测上升沿（从遮挡到未遮挡，即牌离开）
    if (currentStateB == LOW && lastPhotoBState == HIGH) {
        if (millis() - photoBDebounce > 10) { // 防抖10ms
            photoBDebounce = millis();

            // 牌已发出
            dealtCards++;
            lastCardTime = millis();   // 更新最后出牌时间

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
                return;
            }

            // 计算下一次应该停止的计数目标（精确查表）
            updateNextStopCount();

            // 控制电机A：如果当前计数未达到目标，则启动电机A；否则保持停止
            if (photoACount < nextStopCount) {
                if (currentState != STATE_A_RUNNING) {
                    controlMotorA(true);
                    currentState = STATE_A_RUNNING;
                }
            } else {
                // 已经到位，确保电机A停止
                if (currentState == STATE_A_RUNNING) {
                    controlMotorA(false);
                    currentState = STATE_B_RUNNING;
                }
            }
        }
    }
    lastPhotoBState = currentStateB;
}

// ==================== 上电归位（清屏）====================
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

    // 电机B归位（可选，如果机械位置需要）
    // 此处省略，根据需要可加入类似归位

    lcd.clear();
}

// ==================== 电机A校准归位（下降沿，不清屏）====================
void calibrateMotorA() {
    stopAllMotors();                 // 确保电机停止
    enableMotorDriver();
    delay(100);

    // 临时在第二行显示校准信息（不擦除第一行）
    lcd.setCursor(0, 1);
    lcd.print("Calib A...      ");   // 清空第二行

    controlMotorA(true);
    unsigned long startTime = millis();
    uint8_t lastState = digitalRead(PHOTO_A_PIN);
    bool found = false;
    while (millis() - startTime < 10000) {
        uint8_t curState = digitalRead(PHOTO_A_PIN);
        // 检测下降沿 (HIGH -> LOW)
        if (curState == LOW && lastState == HIGH) {
            found = true;
            break;
        }
        lastState = curState;
        delay(10);
    }
    controlMotorA(false);

    // 在第二行显示结果
    lcd.setCursor(0, 1);
    if (found) {
        lcd.print("A OK           ");
    } else {
        lcd.print("A TO           ");
    }
    delay(800);

    // 恢复完整显示
    updateDisplay();
}

// ==================== 计算下一次停止计数 ====================
void updateNextStopCount() {
    if (playerCount < 2 || playerCount > 8) return;

    uint16_t nextIndex = dealtCards;   // 下一张牌索引（当前已发牌数，因为下一张就是第dealtCards张）
    uint8_t circle = nextIndex / playerCount;
    uint8_t posInCircle = nextIndex % playerCount;
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
    if (isRunning == 1) {
        stopAllMotors();            // 停止电机A和B
        isRunning = 2;
#if DEBUG
        Serial.println(F("Paused"));
#endif
        updateDisplay();
    }
}

// ==================== 恢复发牌 ====================
void resumeDealing() {
    if (isRunning == 2) {
        enableMotorDriver();
        // 电机B常转
        controlMotorB(true);
        // 如果之前状态是A运行，则启动电机A，否则只启动B
        if (currentState == STATE_A_RUNNING) {
            controlMotorA(true);
        } else {
            currentState = STATE_B_RUNNING;
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

        // 电机B常转
        controlMotorB(true);
        lastCardTime = millis();      // 初始化超时计时
        currentState = STATE_B_RUNNING; // 初始状态为B运行（等待第一张牌）
        showStatusMessage("Start");
        delay(300);
    } else if (isRunning == 1) {
        pauseDealing();
    } else if (isRunning == 2) {
        resumeDealing();
    }
}

// ==================== 完全停止发牌 ====================
void stopDealing() {
    isRunning = 0;
    currentState = STATE_IDLE;
    photoACount = 0;
    nextStopCount = 0;
    dealtCards = 0;
    stopAllMotors();
    updateDisplay();
}

// ==================== 状态机处理 ====================
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
            // 持续检测新牌（电机B常转）
            updatePhotoB();
            break;

        case STATE_A_RUNNING:
            updatePhotoA();   // 旋转计数
            updatePhotoB();   // 同时检测新牌（电机B仍在转）

            // 检查是否达到目标计数
            if (photoACount >= nextStopCount) {
                controlMotorA(false);
                currentState = STATE_B_RUNNING;
            }

            // 电机A超时保护（防止计数不准导致无限旋转）
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

// ==================== 红外遥控处理 ====================
#if ENABLE_INFRA || ENABLE_INFRA2
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
            bool isStop = false;
            bool isStart = false;
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
            IrReceiver.resume();
            return;
        }

        // 空闲状态处理所有键
        // 减玩家（只有第一遥控器有此功能）
#if ENABLE_INFRA
        if (irValue == IR_DEC_PLAYER) {
            if (playerCount > 2) playerCount--; else playerCount = 8;
            updateDisplay();
        } else
#endif
        // 重置参数（只有第一遥控器）
#if ENABLE_INFRA
        if (irValue == IR_RESET) {
            playerCount = 4; deckCount = 3; hasJokers = 1; remainCards = 0;
            totalCards = deckCount * (hasJokers ? 54 : 52);
            updateDisplay();
        } else
#endif
        // 加玩家（两个遥控器都有）
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
            updateDisplay();
        } else
#endif
        // 加副数（两个遥控器都有）
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
            updateDisplay();
        } else
#endif
        // 切换有无王（两个遥控器都有）
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
            updateDisplay();
        } else
#endif
        // 发牌/暂停/恢复（两个遥控器都有）
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
        // 停止（两个遥控器都有，空闲时仅显示信息）
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
        // 设置剩余牌（两个遥控器都有）
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
            updateDisplay();
        } else
#endif
        // 测试电机A（只有第一遥控器）
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
        // 测试电机B（只有第一遥控器）
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
        // 电机A校准归位（下降沿，不清屏）
#if ENABLE_INFRA
        if (irValue == IR_CALIBRATE) {
            calibrateMotorA();
        } else
#endif
        // 切换旋转方向（两个遥控器都有）
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
            lcd.clear();
            lcd.print(F("Direction:"));
            lcd.setCursor(0, 1);
            lcd.print(clockwise ? F("CW") : F("CCW"));
            delay(800);
            updateDisplay();
        } else
#endif
        // 系统复位（只有第一遥控器）
#if ENABLE_INFRA
        if (irValue == IR_SYSTEM_RESET) {
            resetSystem();
        } else
#endif
        // 版本信息（只有第一遥控器）
#if ENABLE_INFRA
        if (irValue == IR_VERSION) {
            lcd.clear();
            lcd.print(F("Dealer v39.5"));
            lcd.setCursor(0, 1);
            lcd.print(F("Homing"));
            delay(800);
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
            lcd.print(F("Dealer v39.5"));
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
            Serial.println(F("=== Card Dealer Cmds ==="));
            Serial.println(F("Player++/Deck++/Joker/Remaining/Start/Csettingreset/Tstop/A/B/Mclockwise/Zsystemreset/Help"));
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
    lcd.print(F("Dealer v39.5"));
    lcd.setCursor(0, 1);
    lcd.print(F("Homing"));

#if DEBUG
    Serial.begin(9600);
    delay(500);
    Serial.println(F("=== System Startup (Continuous Mode) ==="));
    Serial.println(F("Players: 2-8, Direction: CW (default)"));
#endif

#if ENABLE_INFRA || ENABLE_INFRA2
    IrReceiver.begin(IR_RECEIVE_PIN);
#endif

    // 初始化参数
    totalCards = deckCount * (hasJokers ? 54 : 52);

    lastPhotoAState = digitalRead(PHOTO_A_PIN);
    lastPhotoBState = digitalRead(PHOTO_B_PIN);

    serialBufferIndex = 0;
    serialBuffer[0] = '\0';

    // 执行上电归位（会清屏）
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