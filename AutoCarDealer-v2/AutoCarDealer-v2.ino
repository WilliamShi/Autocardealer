// ==================== 系统配置宏 ====================
#define ENABLE_INFRA 1            // 启用红外遥控
#define ENABLE_INFRA2 1           // 启用第二个红外遥控（默认关闭，需手动设为1）
#define ENABLE_KEYBOARD 0         // 启用串口键盘指令
#define DEBUG 0                    // 调试信息输出
#define IR_DEBUG 0                 // 红外详细调试（开启以便观察干扰）

// ==================== 优化参数配置 ====================
// 电机速度（已经是最大255）
#define MOTOR_A_SPEED 255
#define MOTOR_B_SPEED 230

// 超时时间（进一步减少以加快错误恢复）
#define MOTOR_B_TIMEOUT 8000       // 从4000ms减少到3000ms
#define MOTOR_A_TIMEOUT 1500       // 从2000ms减少到1500ms

// 消隐窗口（进一步减少以提高发牌频率）
#define CARD_IGNORE_TIME 70        // 从150ms减少到80ms

// 防抖时间（进一步减少以提高响应速度）
#define PHOTO_A_DEBOUNCE 8         // 3ms 偏小，A码盘抖动会多计，改为8ms
#define PHOTO_B_DEBOUNCE 10        // 从25ms减少到10ms

// 循环延迟（进一步减少以提高响应速度）
#define LOOP_DELAY 1               // 从2ms减少到1ms

// 红外防抖（进一步减少）
#define IR_DEBOUNCE_TIME 150       // 从200ms减少到150ms

// 电机A停滞检测时间（进一步减少）
#define MOTOR_A_STUCK_TIME 500     // 从800ms减少到500ms

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
#define IR2_START        0xFE017F80 //play,发牌，暂停，恢复，等同于IR_START
#define IR2_STOP         0xED127F80 //up,停止发牌，等同于 IR_STOP
#define IR2_INC_PLAYER   0xFA057F80 //+等同于IR_INC_PLAYER
#define IR2_REMAIN_CARDS 0xE41B7F80 //-等同于IR_REMAIN_CARDS
#define IR2_INC_DECK     0xEE117F80 //left等同于IR_INC_DECK
#define IR2_TOGGLE_JOKER 0xFC037F80 //right等同于IR_TOGGLE_JOKER
#define IR2_TOGGLE_MODE  0xF6097F80 //down等同于IR_TOGGLE_MODE
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
#define MOTOR_B_IN1 A3
#define MOTOR_B_IN2 A2
#define MOTOR_B_PWM 10
#define MOTOR_STBY 8

// ==================== 光电开关引脚定义 ====================
#define PHOTO_A_PIN A4        // 电机A计数光电开关
#define PHOTO_B_PIN A0        // 电机B发牌检测光电开关

// ==================== LED 引脚定义 ====================
#define LED_RED_PIN 13    // 红色LED控制引脚（D13）
#define LED_GREEN_PIN A5  // 绿色LED控制引脚（A5作为数字引脚使用）

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
const int MOTOR_A_SPEED_VAL = MOTOR_A_SPEED;
const int MOTOR_B_SPEED_VAL = MOTOR_B_SPEED;
unsigned long motorBTimeout = MOTOR_B_TIMEOUT;
unsigned long motorATimeout = MOTOR_A_TIMEOUT;

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
unsigned long photoBDebounce = 0;
unsigned long lastCardTime = 0;
unsigned long cardIgnoreUntil = 0;        // 发牌后消隐窗口结束时间
bool waitingCardClear = false;            // 等待旧牌离开光电，防止B重启后旧牌抖动被多计
uint8_t lastPhotoBInA = HIGH;             // A转动期间跟踪光电B状态
bool bActivityInA = false;                // A转动期间检测到光电B活动，说明B惯性带出牌

// ---- 非阻塞 LED 闪烁状态 ----
bool blinkActive = false;
unsigned long blinkStartMs = 0;
unsigned long blinkDurationMs = 0;
uint8_t blinkLastPhase = 0;

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
const unsigned long IR_DEBOUNCE_TIME_VAL = IR_DEBOUNCE_TIME;

// ==================== 新增比分变量 ====================
uint8_t scoreX = 2;   // 默认02
uint8_t scoreY = 2;   // 默认02

// ==================== 软件复位函数指针 ====================
void (*resetFunc)(void) = 0;  // 指向地址0，调用即复位

// ==================== 上次显示的值（用于局部刷新）====================
uint8_t lastPlayerCount = 0;
uint8_t lastDeckCount = 0;
uint8_t lastHasJokers = 0;
uint8_t lastScoreX = 0;
uint8_t lastScoreY = 0;
bool lastClockwise = true;
uint16_t lastDealtCards = 0;
uint16_t lastTotalCards = 0;
uint8_t lastIsRunning = 0;
SystemState lastCurrentState = STATE_IDLE;
uint16_t lastPhotoACount = 0;

// ==================== 全局刷新标志 ====================
bool fullRefresh = true;   // 初始需要全刷新

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

// ==================== 预计算位置缓存 ====================
uint16_t nextStopCache[256];  // 缓存最多256张牌的位置
bool cacheInitialized = false;

// ==================== 函数声明 ====================
void enableMotorDriver();
void disableMotorDriver();
void stopAllMotors();
void controlMotorA(bool enable);
void controlMotorB(bool enable);
void controlMotorB_SlowStop();
void startDealing();
void pauseDealing();
void resumeDealing();
void stopDealing();
void updatePhotoA();
void updatePhotoB();
void updateDealtFast();
void updateStateFast();
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
void setLED(uint8_t red, uint8_t green);
void startBlink(unsigned long duration);
void updateBlink();
void updateLEDState();

// ==================== 初始化位置缓存 ====================
void initPositionCache() {
    if (playerCount < 2 || playerCount > 8) {
        // 如果playerCount无效，用默认值初始化
        for (int i = 0; i < 256; i++) {
            nextStopCache[i] = 0;
        }
        cacheInitialized = true;
        return;
    }

    for (int i = 0; i < 256; i++) {
        uint8_t circle = i / playerCount;
        uint8_t posInCircle = i % playerCount;
        nextStopCache[i] = basePositions16[playerCount][posInCircle] + circle * 16;
    }
    cacheInitialized = true;
}

// ==================== LED 控制函数 ====================
void setLED(uint8_t red, uint8_t green) {
    digitalWrite(LED_RED_PIN, red ? HIGH : LOW);
    digitalWrite(LED_GREEN_PIN, green ? HIGH : LOW);
}

// 非阻塞 LED 闪烁：设置状态，由 loop 每轮更新
void startBlink(unsigned long duration) {
    blinkActive = true;
    blinkStartMs = millis();
    blinkDurationMs = duration;
    blinkLastPhase = 0;
}

// 在 loop 中调用，非阻塞更新 LED 闪烁
void updateBlink() {
    if (!blinkActive) return;
    unsigned long now = millis();
    if (now - blinkStartMs >= blinkDurationMs) {
        blinkActive = false;
        updateLEDState();
        return;
    }
    unsigned long elapsed = now - blinkStartMs;
    uint8_t phase = (elapsed / 20) & 1;
    if (phase != blinkLastPhase) {
        blinkLastPhase = phase;
        setLED(phase ? 0 : 1, phase ? 1 : 0);
    }
}

void updateLEDState() {
    if (isRunning == 0) {
        // 停止状态：红色
        setLED(1, 0);
    } else if (isRunning == 1) {
        // 运行状态：绿色
        setLED(0, 1);
    } else if (isRunning == 2) {
        // 暂停状态：红色+绿色（黄色）
        setLED(1, 1);
    }
}

// ==================== 电机控制函数（优化快速停止）====================
void enableMotorDriver() {
    digitalWrite(MOTOR_STBY, HIGH);
    delay(2);  // 进一步减少延迟
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
    delay(2);  // 进一步减少延迟
}

void controlMotorA(bool enable) {
    if (enable) {
        digitalWrite(MOTOR_A_IN1, HIGH);
        digitalWrite(MOTOR_A_IN2, HIGH);
        analogWrite(MOTOR_A_PWM, 255);
        delayMicroseconds(100);  // 减少到100微秒
        if (clockwise) {
            digitalWrite(MOTOR_A_IN1, LOW);
            digitalWrite(MOTOR_A_IN2, HIGH);
        } else {
            digitalWrite(MOTOR_A_IN1, HIGH);
            digitalWrite(MOTOR_A_IN2, LOW);
        }
        analogWrite(MOTOR_A_PWM, MOTOR_A_SPEED_VAL);
        motorAStartTime = millis();
        lastPhotoATime = millis();
        // 重置A光电基准状态，防止A启动时因状态与上次停止时不同而多计
        lastPhotoAState = digitalRead(PHOTO_A_PIN);
        photoADebounce = millis() - PHOTO_A_DEBOUNCE - 1;
        delayMicroseconds(100);  // 减少到100微秒
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
        analogWrite(MOTOR_B_PWM, MOTOR_B_SPEED_VAL);
    } else {
        digitalWrite(MOTOR_B_IN1, HIGH);
        digitalWrite(MOTOR_B_IN2, HIGH);
        analogWrite(MOTOR_B_PWM, 0);
    }
}

// ==================== B电机平滑停止 ====================
// 先降速到100运行8ms，再短路制动。
// 这样动能显著减小，惯性带出第二张牌的概率降低；
// 总停止时间与原 delay(8) 基本相同，不增加发牌周期。
void controlMotorB_SlowStop() {
    // 先减速：保持方向，PWM 降到 100
    digitalWrite(MOTOR_B_IN1, LOW);
    digitalWrite(MOTOR_B_IN2, HIGH);
    analogWrite(MOTOR_B_PWM, 100);
    delay(8);  // 减速阶段
    // 短路制动
    digitalWrite(MOTOR_B_IN1, HIGH);
    digitalWrite(MOTOR_B_IN2, HIGH);
    analogWrite(MOTOR_B_PWM, 0);
}

// ==================== 光电开关A计数（双边沿检测）====================
void updatePhotoA() {
    if (currentState != STATE_A_RUNNING) return;

    uint8_t currentStateA = digitalRead(PHOTO_A_PIN);
    if (currentStateA != lastPhotoAState) {
        unsigned long now = millis();
        if (now - photoADebounce > PHOTO_A_DEBOUNCE) {
            photoADebounce = now;
            photoACount++;
            lastPhotoATime = now;
#if DEBUG
            Serial.print(F("PhotoA count: "));
            Serial.println(photoACount);
#endif
        }
        lastPhotoAState = currentStateA;
    }
}

// ==================== 光电开关B检测 ====================
// 关键修复：
// 1. 消隐窗口内不计数，但仍然同步 lastPhotoBState，防止窗口结束时旧状态误触发
// 2. 触发后重新读取光电状态，避免 processCard 改变状态后用旧值覆盖
// 3. waitingCardClear：A到位启动B时若旧牌仍在光电处，先等它稳定HIGH再开始消隐计数，
//    彻底防止旧牌在B重启后被推走过程中的抖动被当成新牌多计
void updatePhotoB() {
    if (isRunning != 1 || currentState != STATE_B_RUNNING) return;

    uint8_t currentStateB = digitalRead(PHOTO_B_PIN);
    unsigned long now = millis();

    // 等待旧牌离开光电
    if (waitingCardClear) {
        if (currentStateB == HIGH) {
            waitingCardClear = false;
            cardIgnoreUntil = now + CARD_IGNORE_TIME;
            lastPhotoBState = HIGH;
        } else {
            lastPhotoBState = LOW;
        }
        return;
    }

    if (now >= cardIgnoreUntil) {
        if (currentStateB == LOW && lastPhotoBState == HIGH) {
            if (now - photoBDebounce > PHOTO_B_DEBOUNCE) {
                photoBDebounce = now;
                processCard();
                // processCard 可能启动A或暂停，重新读取实际光电状态，避免旧值覆盖
                lastPhotoBState = digitalRead(PHOTO_B_PIN);
                return;
            }
        }
    }

    // 无论是否在消隐窗口，都同步最新光电状态
    lastPhotoBState = currentStateB;
}

// ==================== 轻量刷新：已发牌数（每张调用）====================
// 只刷新第二行第2~4列，格式与 updateDisplay() 的 "%-3d" 一致
void updateDealtFast() {
    uint8_t h = dealtCards / 100;
    uint8_t t = (dealtCards / 10) % 10;
    uint8_t o = dealtCards % 10;

    lcd.setCursor(2, 1);
    if (h > 0) {
        lcd.write('0' + h);
        lcd.write('0' + t);
        lcd.write('0' + o);
    } else if (t > 0) {
        lcd.write('0' + t);
        lcd.write('0' + o);
        lcd.write(' ');
    } else {
        lcd.write('0' + o);
        lcd.write(' ');
        lcd.write(' ');
    }

    lastDealtCards = dealtCards;   // 同步 last，避免下次 updateDisplay 重复写
}

// ==================== 轻量刷新：状态字符（B/A/P/S）====================
// 只刷新第二行第10列，状态从 B 切 A 或 A 切 B 时保持显示正确
void updateStateFast() {
    uint8_t stateChar;
    if (isRunning == 1) {
        switch (currentState) {
            case STATE_B_RUNNING: stateChar = 'B'; break;
            case STATE_A_RUNNING: stateChar = 'A'; break;
            default:              stateChar = 'R'; break;
        }
    } else if (isRunning == 2) {
        stateChar = 'P';
    } else {
        stateChar = 'S';
    }

    lcd.setCursor(10, 1);
    lcd.write(stateChar);

    lastIsRunning = isRunning;
    lastCurrentState = currentState;
}

// ==================== 处理一张牌发出（B平滑停止，先启电机后刷LCD）====================
void processCard() {
    // 平滑停止电机B：先减速再制动，减小惯性，防止带出第二张
    controlMotorB_SlowStop();

    cardIgnoreUntil = millis() + CARD_IGNORE_TIME;  // 使用优化的消隐窗口
    dealtCards++;
    lastCardTime = millis();

#if DEBUG
    Serial.print(F("Card dealt: "));
    Serial.println(dealtCards);
#endif

    if (dealtCards >= totalCards && totalCards > 0) {
        stopDealing();
        showStatusMessage("All Done!");
        delay(150);  // 进一步减少延迟
        updateDisplay();
        return;
    }

    // 先计算下一次停止计数
    updateNextStopCount();

    // 先启动电机，避免 LCD 刷新阻塞电机启动
    if (nextStopCount > photoACount) {
        controlMotorA(true);
        currentState = STATE_A_RUNNING;
        motorAStartTime = millis();
        lastPhotoATime = millis();
        // A阶段时快照B光电状态，跟踪A期间B是否还有惯性活动
        lastPhotoBInA = digitalRead(PHOTO_B_PIN);
        bActivityInA = false;
    } else {
        // 异常：photoACount 多计，A 已越过目标位置。
        // 不能跳过 A 转动直接重启 B，否则会连续发牌并错位。
        controlMotorA(false);
        pauseDealing();
        showStatusMessage("A Count Err");
        delay(150);
        updateDisplay();
        return;
    }

    // 电机启动后再做轻量 LCD 刷新；每50张全刷新一次，自愈轻微错位
    delay(2);
    if ((dealtCards % 50) == 0) {
        fullRefresh = true;
        updateDisplay();
    } else {
        updateDealtFast();
        updateStateFast();
    }
}

// ==================== 上电归位 ====================
void performInitialHoming() {
    enableMotorDriver();
    delay(30);  // 进一步减少延迟

    lcd.clear();
    lcd.print("Homing MotorA...");
    controlMotorA(true);
    unsigned long startTime = millis();
    uint8_t lastState = digitalRead(PHOTO_A_PIN);
    bool found = false;
    while (millis() - startTime < 5000) {  // 减少到5秒
        uint8_t curState = digitalRead(PHOTO_A_PIN);
        if (curState == LOW && lastState == HIGH) {
            found = true;
            break;
        }
        lastState = curState;
        delay(2);  // 进一步减少延迟
    }
    controlMotorA(false);
    lcd.setCursor(0, 1);
    if (found) {
        lcd.print("A OK");
    } else {
        lcd.print("A TO");
    }
    delay(150);  // 进一步减少延迟
    lcd.clear();
}

// ==================== 电机A校准归位 ====================
void calibrateMotorA() {
    stopAllMotors();
    enableMotorDriver();
    delay(30);  // 进一步减少延迟

    lcd.setCursor(0, 1);
    lcd.print("Calib A...      ");

    controlMotorA(true);
    unsigned long startTime = millis();
    uint8_t lastState = digitalRead(PHOTO_A_PIN);
    bool found = false;
    while (millis() - startTime < 5000) {  // 减少到5秒
        uint8_t curState = digitalRead(PHOTO_A_PIN);
        if (curState == LOW && lastState == HIGH) {
            found = true;
            break;
        }
        lastState = curState;
        delay(2);  // 进一步减少延迟
    }
    controlMotorA(false);

    lcd.setCursor(0, 1);
    if (found) {
        lcd.print("A OK           ");
    } else {
        lcd.print("A TO           ");
    }
    delay(300);  // 进一步减少延迟
    // 临时消息覆盖了第二行，需要全刷新恢复固定字符
    fullRefresh = true;
    updateDisplay();
}

// ==================== 重置已发牌数和计数 ====================
void resetDealCounts() {
    dealtCards = 0;
    photoACount = 0;
    nextStopCount = 0;
    cardIgnoreUntil = 0;
    waitingCardClear = false;
    bActivityInA = false;
    lastPhotoBInA = HIGH;
    if (isRunning != 0) {
        stopDealing();
    }
    updateDisplay();                 // 先更新显示
    lcd.setCursor(0, 1);
    lcd.print(F("Counts reset"));    // 覆盖第二行
    delay(150);  // 进一步减少延迟
    fullRefresh = true;               // 需要全刷新恢复固定字符
    updateDisplay();
}

// ==================== 计算下一次停止计数 ====================
void updateNextStopCount() {
    if (playerCount < 2 || playerCount > 8) return;

    // 如果缓存未初始化或playerCount可能变化，确保缓存有效
    if (!cacheInitialized) {
        initPositionCache();
    }

    // 使用缓存提高速度
    if (dealtCards < 256) {
        nextStopCount = nextStopCache[dealtCards];
    } else {
        uint16_t nextIndex = dealtCards;
        uint8_t circle = nextIndex / playerCount;
        uint8_t posInCircle = nextIndex % playerCount;
        nextStopCount = basePositions16[playerCount][posInCircle] + circle * 16;
    }

#if DEBUG
    Serial.print(F("Next stop: "));
    Serial.print(nextStopCount);
    Serial.print(F(" (index="));
    Serial.print(dealtCards);
    Serial.println(F(")"));
#endif
}

// ==================== 暂停/恢复 ====================
// 暂停时电机已停止，等电源稳定后重新初始化LCD，修复可能的移位/乱码
void pauseDealing() {
    if (isRunning == 1) {
        stopAllMotors();
        isRunning = 2;
        waitingCardClear = false;
        bActivityInA = false;

        // 等电机停止、电源稳定，再重新初始化 LCD
        delay(20);
        lcd.begin(16, 2);
        delay(5);

        fullRefresh = true;
        updateDisplay();
        updateLEDState();

#if DEBUG
        Serial.println(F("Paused"));
#endif
    }
}

void resumeDealing() {
    if (isRunning == 2) {
        enableMotorDriver();

        // 重置所有与超时相关的计时器，防止恢复后立即再次超时
        lastCardTime = millis();               // 电机B超时计时
        if (currentState == STATE_A_RUNNING) {
            motorAStartTime = millis();        // 电机A超时计时
            lastPhotoATime = millis();          // 电机A停滞检测计时
            controlMotorA(true);
            waitingCardClear = false;
            cardIgnoreUntil = millis() + CARD_IGNORE_TIME;
            lastPhotoBState = digitalRead(PHOTO_B_PIN);
            lastPhotoBInA = lastPhotoBState;
            bActivityInA = false;
        } else if (currentState == STATE_B_RUNNING) {
            lastPhotoBState = digitalRead(PHOTO_B_PIN);
            controlMotorB(true);
            if (lastPhotoBState == LOW) {
                // 旧牌还在光电处，等它离开再计数，防止恢复后旧牌抖动被多计
                waitingCardClear = true;
                cardIgnoreUntil = millis() + 2000;
            } else {
                waitingCardClear = false;
                cardIgnoreUntil = millis() + CARD_IGNORE_TIME;
            }
            bActivityInA = false;
        }

        isRunning = 1;

        // 电机刚启动，等电流冲击过去再刷 LCD；此时不做 lcd.begin，避免电源不稳时初始化失败
        delay(10);
        fullRefresh = true;
        updateDisplay();
        updateLEDState();

#if DEBUG
        Serial.println(F("Resumed"));
#endif
    }
}

// ==================== 开始发牌 ====================
void startDealing() {
    if (isRunning == 0) {
        if (totalCards <= 0) {
            showStatusMessage("No Cards!");
            delay(150);  // 进一步减少延迟
            updateDisplay();
            return;
        }
        isRunning = 1;
        dealtCards = 0;
        photoACount = 0;
        nextStopCount = 0;

        stopAllMotors();
        enableMotorDriver();

        // 启动电机B开始发牌，先同步光电B状态
        lastPhotoBState = digitalRead(PHOTO_B_PIN);
        controlMotorB(true);
        if (lastPhotoBState == LOW) {
            // 启动时光电已有牌遮挡，等它离开再计数
            waitingCardClear = true;
            cardIgnoreUntil = millis() + 2000;
        } else {
            waitingCardClear = false;
            cardIgnoreUntil = millis() + CARD_IGNORE_TIME;
        }
        lastCardTime = millis();
        currentState = STATE_B_RUNNING;
        bActivityInA = false;
        showStatusMessage("Start");
        delay(80);  // 进一步减少延迟
        updateDisplay();
        updateLEDState();  // 更新LED为运行状态（绿）- 直接更新，不闪烁
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
    waitingCardClear = false;
    bActivityInA = false;
    lastPhotoBInA = HIGH;
    stopAllMotors();

    // 等电机停止、电源稳定后重新初始化 LCD
    delay(20);
    lcd.begin(16, 2);
    delay(10);
    fullRefresh = true;
    updateDisplay();
    updateLEDState();  // 更新LED为停止状态（红）- 直接更新，不闪烁
}

// ==================== 状态机处理 ====================
void handleMotorState() {
    switch (currentState) {
        case STATE_B_RUNNING:
            updatePhotoB();  // 检测出牌
            // 电机B超时检测（基于最后出牌时间）
            if (millis() - lastCardTime > motorBTimeout) {
                // pauseDealing 内部会停电机、delay 20ms、lcd.begin 修复移位/乱码
                pauseDealing();
                showStatusMessage("B Timeout");
            }
            break;

        case STATE_A_RUNNING:
            updatePhotoA();  // 检测旋转计数

            // ★ A转动期间跟踪B光电，检测B惯性是否带出牌
            {
                uint8_t bNow = digitalRead(PHOTO_B_PIN);
                if (bNow != lastPhotoBInA) {
                    bActivityInA = true;
                }
                lastPhotoBInA = bNow;
            }

            // 检查是否达到目标
            if (photoACount >= nextStopCount) {
                controlMotorA(false);  // 停止电机A
                if (dealtCards < totalCards) {
                    // 重启B前同步光电B状态；若旧牌还在光电处，进入等待清除模式
                    lastPhotoBState = digitalRead(PHOTO_B_PIN);
                    controlMotorB(true);
                    currentState = STATE_B_RUNNING;
                    lastCardTime = millis();
                    if (lastPhotoBState == LOW) {
                        // 旧牌仍在光电处，B重启后先等它稳定HIGH，再开始消隐计数
                        waitingCardClear = true;
                        cardIgnoreUntil = millis() + 2000;
                    } else {
                        waitingCardClear = false;
                        cardIgnoreUntil = millis() + CARD_IGNORE_TIME;
                    }
                    bActivityInA = false;
                    delay(2);
                    updateStateFast();         // 状态 A -> B，立即刷新
                } else {
                    stopDealing();  // 发完牌停止
                }
            }

            // 电机A停滞检测
            if (millis() - lastPhotoATime > MOTOR_A_STUCK_TIME && photoACount < nextStopCount) {
                controlMotorA(false);
                pauseDealing();
                showStatusMessage("A Stuck");
                delay(150);
                updateDisplay();
            }

            // 电机A超时保护
            if (millis() - motorAStartTime > motorATimeout && isRunning == 1) {
                controlMotorA(false);
                pauseDealing();
                showStatusMessage("A Timeout");
                delay(150);
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
        Serial.print(F("IR Received: 0x"));
        Serial.println(irValue, HEX);
#endif
        if (irValue == 0xFFFFFFFF) {
            IrReceiver.resume();
            return;
        }

        // 红外防抖
        unsigned long now = millis();
        if (irValue == lastIRCommandValue && (now - lastIRCommandTime < IR_DEBOUNCE_TIME_VAL)) {
#if IR_DEBUG
            Serial.println(F("IR debounce: ignored"));
#endif
            IrReceiver.resume();
            return;
        }

        // 收到有效红外信号，非阻塞启动 LED 闪烁
        // ★ 用 startBlink 替代原来的 blinkLEDBoth(200)，不再阻塞主循环，
        //   避免电机干扰触发红外误码时把光电采样窗口堵死导致漏检
        startBlink(150);

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
            initPositionCache();  // 更新缓存
            resetDealCounts();
            updateDisplay();
        } else
#endif
#if ENABLE_INFRA
        if (irValue == IR_RESET) {
            playerCount = 4; deckCount = 3; hasJokers = 1; remainCards = 0;
            totalCards = deckCount * (hasJokers ? 54 : 52);
            initPositionCache();  // 更新缓存
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
            initPositionCache();  // 更新缓存
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
            fullRefresh = true;
            lcd.print(F("Idle"));
            delay(300);  // 进一步减少延迟
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
            remainCards += 1;   // 每次增加1，而非playerCount
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
            fullRefresh = true;
            lcd.print(F("MotorA Test"));
            lcd.setCursor(0, 1);
            lcd.print(F("Wait count..."));
            photoACount = 0;
            nextStopCount = 2;
            controlMotorA(true);
            unsigned long testStart = millis();
            bool testComplete = false;
            while (!testComplete && millis() - testStart < 1500) {  // 从2000ms减少到1500ms
                updatePhotoA();
                if (photoACount >= nextStopCount) {
                    controlMotorA(false);
                    testComplete = true;
                }
                delay(2);  // 进一步减少延迟
            }
            if (!testComplete) stopAllMotors();
            lcd.clear();
            fullRefresh = true;
            lcd.print(F("Test done"));
            delay(300);  // 进一步减少延迟
            updateDisplay();
        } else
#endif
#if ENABLE_INFRA
        if (irValue == IR_TEST_MOTOR_B) {
            enableMotorDriver();
            controlMotorB(true);
            lcd.clear();
            fullRefresh = true;
            lcd.print(F("MotorB Test"));
            lcd.setCursor(0, 1);
            lcd.print(F("Wait card..."));
            {
                unsigned long testStartB = millis();
                bool cardDetected = false;
                lastPhotoBState = digitalRead(PHOTO_B_PIN);
                while (millis() - testStartB < 2000) {  // 从3000ms减少到2000ms
                    uint8_t cur = digitalRead(PHOTO_B_PIN);
                    if (cur == LOW && lastPhotoBState == HIGH) {
                        cardDetected = true;
                        break;
                    }
                    lastPhotoBState = cur;
                    delay(2);  // 进一步减少延迟
                }
                controlMotorB(false);
                lcd.clear();
                fullRefresh = true;
                if (cardDetected) lcd.print(F("B OK"));
                else lcd.print(F("B TO"));
            }
            delay(300);  // 进一步减少延迟
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
            fullRefresh = true;
            lcd.print(F("Direction:"));
            lcd.setCursor(0, 1);
            lcd.print(clockwise ? F("CCW") : F("CW"));
            lcd.setCursor(0, 1);
            lcd.print(F("Please Calib A"));
            delay(600);  // 进一步减少延迟
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
            delay(30);  // 进一步减少延迟
            resetFunc();
        } else
#endif
#if ENABLE_INFRA
        if (irValue == IR_VERSION) {
            lcd.clear();
            fullRefresh = true;
            lcd.print(F("Dealer v40.4"));
            lcd.setCursor(0, 1);
            lcd.print(F("16-edge mode"));
            delay(300);  // 进一步减少延迟
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
        fullRefresh = true;
        lcd.print(F("Not supported"));
        delay(300);  // 进一步减少延迟
        updateDisplay();
        return;
    }
    switch (command[0]) {
        case 'v': case 'V':
            lcd.clear();
            fullRefresh = true;
            lcd.print(F("Dealer v40.4"));
            lcd.setCursor(0, 1);
            lcd.print(F("16-edge mode"));
            delay(300);  // 进一步减少延迟
            updateDisplay();
            break;
        case 'p': case 'P':
            playerCount++; if (playerCount > 8) playerCount = 2;
            initPositionCache();  // 更新缓存
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
            remainCards += 1;   // 每次增加1，而非playerCount
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
            initPositionCache();  // 更新缓存
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
                fullRefresh = true;
                lcd.print(F("MotorA Test"));
                lcd.setCursor(0, 1);
                lcd.print(F("Wait count..."));
                photoACount = 0;
                nextStopCount = 2;
                controlMotorA(true);
                unsigned long testStart = millis();
                bool testComplete = false;
                while (!testComplete && millis() - testStart < 1500) {  // 从2000ms减少到1500ms
                    updatePhotoA();
                    if (photoACount >= nextStopCount) {
                        controlMotorA(false);
                        testComplete = true;
                    }
                    delay(2);  // 进一步减少延迟
                }
                if (!testComplete) stopAllMotors();
                lcd.clear();
                fullRefresh = true;
                lcd.print(F("Test done"));
                delay(300);  // 进一步减少延迟
                updateDisplay();
            }
            break;
        case 'b': case 'B':
            if (!isRunning) {
                enableMotorDriver();
                controlMotorB(true);
                lcd.clear();
                fullRefresh = true;
                lcd.print(F("MotorB Test"));
                lcd.setCursor(0, 1);
                lcd.print(F("Wait card..."));
                unsigned long testStart = millis();
                bool cardDetected = false;
                lastPhotoBState = digitalRead(PHOTO_B_PIN);
                while (millis() - testStart < 2000) {  // 从3000ms减少到2000ms
                    uint8_t cur = digitalRead(PHOTO_B_PIN);
                    if (cur == LOW && lastPhotoBState == HIGH) {
                        cardDetected = true;
                        break;
                    }
                    lastPhotoBState = cur;
                    delay(2);  // 进一步减少延迟
                }
                controlMotorB(false);
                lcd.clear();
                fullRefresh = true;
                if (cardDetected) lcd.print(F("B OK"));
                else lcd.print(F("B TO"));
                delay(300);  // 进一步减少延迟
                updateDisplay();
            }
            break;
        case 'm': case 'M':
            clockwise = !clockwise;
            resetDealCounts();
            lcd.clear();
            fullRefresh = true;
            lcd.print(F("Direction:"));
            lcd.setCursor(0, 1);
            lcd.print(clockwise ? F("CCW") : F("CW"));
            lcd.setCursor(0, 1);
            lcd.print(F("Calib A"));
            delay(600);  // 进一步减少延迟
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
            fullRefresh = true;
            lcd.print(F("No circle adj"));
            delay(300);  // 进一步减少延迟
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
            fullRefresh = true;
            lcd.print(F("?"));
            delay(300);  // 进一步减少延迟
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
    cardIgnoreUntil = 0;
    waitingCardClear = false;
    bActivityInA = false;
    lastPhotoBInA = HIGH;
    serialBufferIndex = 0;
    serialBuffer[0] = '\0';

    // 等电机停止、电源稳定后重新初始化 LCD
    delay(20);
    lcd.begin(16, 2);
    delay(10);
    fullRefresh = true;
    lcd.clear();
    lcd.print(F("Reset"));
    lcd.setCursor(0, 1);
    lcd.print(F("Homing"));
    delay(300);

    initPositionCache();  // 重置缓存

    updateDisplay();
    setLED(1, 0);  // 复位后红色
}

// ==================== LCD 显示（局部刷新）====================
void updateDisplay() {
    if (fullRefresh) {
        // 全刷新：用空格覆盖两行，避免 lcd.clear() 指令被干扰后整屏错位
        lcd.setCursor(0, 0);
        lcd.print(F("                "));
        lcd.setCursor(0, 1);
        lcd.print(F("                "));

        // ---- 第一行固定字符 ----
        lcd.setCursor(0, 0); lcd.print('P');
        lcd.setCursor(2, 0); lcd.print(' ');
        lcd.setCursor(3, 0); lcd.print('D');
        lcd.setCursor(5, 0); lcd.print(' ');
        lcd.setCursor(7, 0); lcd.print(' ');
        lcd.setCursor(10,0); lcd.print(':');
        lcd.setCursor(13,0); lcd.print(' ');

        // ---- 第二行固定字符 ----
        lcd.setCursor(0, 1); lcd.print('D');
        lcd.setCursor(1, 1); lcd.print(':');
        lcd.setCursor(5, 1); lcd.print('/');
        lcd.setCursor(9, 1); lcd.print(' ');

        // 更新所有变量
        lcd.setCursor(1, 0); lcd.print(playerCount);
        lcd.setCursor(4, 0); lcd.print(deckCount);
        lcd.setCursor(6, 0); lcd.print(hasJokers ? 'J' : 'N');
        lcd.setCursor(8, 0); if (scoreX < 10) lcd.print('0'); lcd.print(scoreX);
        lcd.setCursor(11,0); if (scoreY < 10) lcd.print('0'); lcd.print(scoreY);
        lcd.setCursor(14,0);
        if (clockwise) lcd.print("CCW");
        else lcd.print("CW ");

        char buf[6];
        lcd.setCursor(2, 1); sprintf(buf, "%-3d", dealtCards); lcd.print(buf);
        lcd.setCursor(6, 1); sprintf(buf, "%-3d", totalCards); lcd.print(buf);

        uint8_t stateChar;
        if (isRunning == 1) {
            switch (currentState) {
                case STATE_B_RUNNING: stateChar = 'B'; break;
                case STATE_A_RUNNING: stateChar = 'A'; break;
                default: stateChar = 'R';
            }
        } else if (isRunning == 2) {
            stateChar = 'P';
        } else {
            stateChar = 'S';
        }
        lcd.setCursor(10, 1); lcd.print(stateChar);

#if DEBUG
        lcd.setCursor(14, 1); sprintf(buf, "%-3d", photoACount); lcd.print(buf);
#endif

        // 更新所有 last_* 变量
        lastPlayerCount = playerCount;
        lastDeckCount = deckCount;
        lastHasJokers = hasJokers;
        lastScoreX = scoreX;
        lastScoreY = scoreY;
        lastClockwise = clockwise;
        lastDealtCards = dealtCards;
        lastTotalCards = totalCards;
        lastIsRunning = isRunning;
        lastCurrentState = currentState;
        lastPhotoACount = photoACount;

        fullRefresh = false;
    } else {
        // 局部刷新：只更新变化的部分
        if (playerCount != lastPlayerCount) {
            lcd.setCursor(1, 0); lcd.print(playerCount);
            lastPlayerCount = playerCount;
        }
        if (deckCount != lastDeckCount) {
            lcd.setCursor(4, 0); lcd.print(deckCount);
            lastDeckCount = deckCount;
        }
        if (hasJokers != lastHasJokers) {
            lcd.setCursor(6, 0); lcd.print(hasJokers ? 'J' : 'N');
            lastHasJokers = hasJokers;
        }
        if (scoreX != lastScoreX) {
            lcd.setCursor(8, 0); if (scoreX < 10) lcd.print('0'); lcd.print(scoreX);
            lastScoreX = scoreX;
        }
        if (scoreY != lastScoreY) {
            lcd.setCursor(11,0); if (scoreY < 10) lcd.print('0'); lcd.print(scoreY);
            lastScoreY = scoreY;
        }
        if (clockwise != lastClockwise) {
            lcd.setCursor(14,0);
            if (clockwise) lcd.print("CCW");
            else lcd.print("CW ");
            lastClockwise = clockwise;
        }

        if (dealtCards != lastDealtCards) {
            // 手动写字符，避免 sprintf 在电机运行时拉长 LCD 受干扰窗口
            uint8_t h = dealtCards / 100;
            uint8_t t = (dealtCards / 10) % 10;
            uint8_t o = dealtCards % 10;
            lcd.setCursor(2, 1);
            if (h > 0) {
                lcd.write('0' + h);
                lcd.write('0' + t);
                lcd.write('0' + o);
            } else if (t > 0) {
                lcd.write('0' + t);
                lcd.write('0' + o);
                lcd.write(' ');
            } else {
                lcd.write('0' + o);
                lcd.write(' ');
                lcd.write(' ');
            }
            lastDealtCards = dealtCards;
        }
        if (totalCards != lastTotalCards) {
            uint8_t h = totalCards / 100;
            uint8_t t = (totalCards / 10) % 10;
            uint8_t o = totalCards % 10;
            lcd.setCursor(6, 1);
            if (h > 0) {
                lcd.write('0' + h);
                lcd.write('0' + t);
                lcd.write('0' + o);
            } else if (t > 0) {
                lcd.write('0' + t);
                lcd.write('0' + o);
                lcd.write(' ');
            } else {
                lcd.write('0' + o);
                lcd.write(' ');
                lcd.write(' ');
            }
            lastTotalCards = totalCards;
        }

        uint8_t stateChar;
        if (isRunning == 1) {
            switch (currentState) {
                case STATE_B_RUNNING: stateChar = 'B'; break;
                case STATE_A_RUNNING: stateChar = 'A'; break;
                default: stateChar = 'R';
            }
        } else if (isRunning == 2) {
            stateChar = 'P';
        } else {
            stateChar = 'S';
        }
        if (isRunning != lastIsRunning || currentState != lastCurrentState) {
            lcd.setCursor(10, 1); lcd.print(stateChar);
            lastIsRunning = isRunning;
            lastCurrentState = currentState;
        }

#if DEBUG
        if (photoACount != lastPhotoACount) {
            char buf[6];
            lcd.setCursor(14, 1); sprintf(buf, "%-3d", photoACount); lcd.print(buf);
            lastPhotoACount = photoACount;
        }
#endif
    }
}

// ==================== 显示临时消息 ====================
void showStatusMessage(const char* message) {
    // 用空格覆盖两行，避免 lcd.clear() 指令被干扰
    lcd.setCursor(0, 0);
    lcd.print(F("                "));
    lcd.setCursor(0, 1);
    lcd.print(F("                "));
    lcd.setCursor(0, 0);
    lcd.print(message);
    lcd.setCursor(0, 1);
    lcd.print(F("D:"));
    lcd.print(dealtCards);
    lcd.print(F("/"));
    lcd.print(totalCards);
    delay(100);
    fullRefresh = true;
    updateDisplay();
}

// ==================== SETUP ====================
void setup() {
    delay(150);  // 进一步减少延迟
    pinMode(MOTOR_A_IN1, OUTPUT);
    pinMode(MOTOR_A_IN2, OUTPUT);
    pinMode(MOTOR_A_PWM, OUTPUT);
    pinMode(MOTOR_B_IN1, OUTPUT);
    pinMode(MOTOR_B_IN2, OUTPUT);
    pinMode(MOTOR_B_PWM, OUTPUT);
    pinMode(MOTOR_STBY, OUTPUT);
    pinMode(PHOTO_A_PIN, INPUT_PULLUP);
    pinMode(PHOTO_B_PIN, INPUT_PULLUP);
    pinMode(LED_RED_PIN, OUTPUT);
    pinMode(LED_GREEN_PIN, OUTPUT);
    disableMotorDriver();

    lcd.begin(16, 2);
    lcd.clear();
    lcd.print(F("Dealer v40.4"));
    lcd.setCursor(0, 1);
    lcd.print(F("Homing"));

    setLED(1, 0);  // 上电红色

#if DEBUG
    Serial.begin(9600);
    delay(150);  // 进一步减少延迟
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

    // 初始化位置缓存
    initPositionCache();

    // 初始化上次显示值，确保第一次 updateDisplay() 全刷新
    lastPlayerCount = 0;
    lastDeckCount = 0;
    lastHasJokers = 0;
    lastScoreX = 0;
    lastScoreY = 0;
    lastClockwise = !clockwise;
    lastDealtCards = 0xFFFF;
    lastTotalCards = 0;
    lastIsRunning = 0xFF;
    lastCurrentState = (SystemState)0xFF;
    lastPhotoACount = 0xFFFF;

    performInitialHoming();

    fullRefresh = true;   // 归位后需要全刷新
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
    // 非阻塞 LED 闪烁更新
    updateBlink();

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
    delay(LOOP_DELAY);  // 使用优化的循环延迟
}