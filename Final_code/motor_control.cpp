#include "motor_control.h"

// ================== Motor Pins 实例定义 ==================
MotorPins FL_pins = {42, 41, 40, 2, 1};
MotorPins FR_pins = {37, 39, 38, 20, 19};
MotorPins RL_pins = {6, 7, 15, 5, 4};
MotorPins RR_pins = {18, 16, 17, 14, 13};

// ================== PWM 参数 ==================
static const int pwm_freq = 5000;
static const int pwm_res  = 8;

// ================== 编码器参数 ==================
static const int PPR = 7;
static const int reduction_ratio = 150;
static const int PPR_WHEEL = PPR * reduction_ratio * 4;

// ================== 防抖 ==================
static const unsigned long debounce_us = 0;

// ================== PID 启用/禁用 (⭐ 唯一的新增) ==================
bool usePID = true;

// ================== 编码器变量 ==================
volatile long L_encoderCount = 0;
volatile long R_encoderCount = 0;
volatile long L2_encoderCount = 0;
volatile long R2_encoderCount = 0;

volatile unsigned long L_lastPulse = 0;
volatile unsigned long R_lastPulse = 0;
volatile unsigned long L2_lastPulse = 0;
volatile unsigned long R2_lastPulse = 0;

// ================== 目标转速 ==================
float targetL  = 0.0, targetR  = 0.0;
float targetL2 = 0.0, targetR2 = 0.0;

// ================== 当前转速 ==================
float rpmL  = 0.0, rpmR  = 0.0;
float rpmL2 = 0.0, rpmR2 = 0.0;

// ================== PID 中间量 ==================
float errorL = 0, errorR = 0;
float errorL2 = 0, errorR2 = 0;

float prevErrorL = 0, prevErrorR = 0;
float prevErrorL2 = 0, prevErrorR2 = 0;

float errorSumL = 0, errorSumR = 0;
float errorSumL2 = 0, errorSumR2 = 0;

int dutyL = 0, dutyR = 0;
int dutyL2 = 0, dutyR2 = 0;

const float ERROR_SUM_MAX = 30.0;
float Kp = 0.6, Ki = 1.0, Kd = 0.02;

// ================== Timing ==================
unsigned long lastPIDTime = 0;
unsigned long lastRPMTime = 0;

// ISR 声明
void IRAM_ATTR L_ISR_A(); void IRAM_ATTR L_ISR_B();
void IRAM_ATTR R_ISR_A(); void IRAM_ATTR R_ISR_B();
void IRAM_ATTR L2_ISR_A(); void IRAM_ATTR L2_ISR_B();
void IRAM_ATTR R2_ISR_A(); void IRAM_ATTR R2_ISR_B();

// =======================================================
//  电机输出（内部使用）
// =======================================================
static void applyMotorRaw(int dutyL, int dutyR, int dutyL2, int dutyR2)
{
    int maxDuty = (1 << pwm_res) - 1;
    int pwmL  = map(abs(dutyL),  0, 100, 0, maxDuty);
    int pwmR  = map(abs(dutyR),  0, 100, 0, maxDuty);
    int pwmL2 = map(abs(dutyL2), 0, 100, 0, maxDuty);
    int pwmR2 = map(abs(dutyR2), 0, 100, 0, maxDuty);

    // === 左前 ===
    if (dutyL > 0) {
        digitalWrite(FL_pins.pinF, HIGH); digitalWrite(FL_pins.pinB, LOW);
    } else if (dutyL < 0) {
        digitalWrite(FL_pins.pinF, LOW); digitalWrite(FL_pins.pinB, HIGH);
    } else {
        digitalWrite(FL_pins.pinF, LOW); digitalWrite(FL_pins.pinB, LOW);
    }
    ledcWrite(FL_pins.pinPWM, pwmL);

    // === 右前 ===
    if (dutyR > 0) {
        digitalWrite(FR_pins.pinF, HIGH); digitalWrite(FR_pins.pinB, LOW);
    } else if (dutyR < 0) {
        digitalWrite(FR_pins.pinF, LOW); digitalWrite(FR_pins.pinB, HIGH);
    } else {
        digitalWrite(FR_pins.pinF, LOW); digitalWrite(FR_pins.pinB, LOW);
    }
    ledcWrite(FR_pins.pinPWM, pwmR);

    // === 左后 ===
    if (dutyL2 > 0) {
        digitalWrite(RL_pins.pinF, HIGH); digitalWrite(RL_pins.pinB, LOW);
    } else if (dutyL2 < 0) {
        digitalWrite(RL_pins.pinF, LOW); digitalWrite(RL_pins.pinB, HIGH);
    } else {
        digitalWrite(RL_pins.pinF, LOW); digitalWrite(RL_pins.pinB, LOW);
    }
    ledcWrite(RL_pins.pinPWM, pwmL2);

    // === 右后 ===
    if (dutyR2 > 0) {
        digitalWrite(RR_pins.pinF, HIGH); digitalWrite(RR_pins.pinB, LOW);
    } else if (dutyR2 < 0) {
        digitalWrite(RR_pins.pinF, LOW); digitalWrite(RR_pins.pinB, HIGH);
    } else {
        digitalWrite(RR_pins.pinF, LOW); digitalWrite(RR_pins.pinB, LOW);
    }
    ledcWrite(RR_pins.pinPWM, pwmR2);
}

// =======================================================
//  初始化
// =======================================================
void initMotors()
{
    MotorPins* M[4] = { &FL_pins, &FR_pins, &RL_pins, &RR_pins };

    for (int i = 0; i < 4; i++)
    {
        pinMode(M[i]->pinF, OUTPUT);
        pinMode(M[i]->pinB, OUTPUT);
        pinMode(M[i]->pinA, INPUT_PULLUP);
        pinMode(M[i]->pinB_enc, INPUT_PULLUP);
        ledcAttach(M[i]->pinPWM, pwm_freq, pwm_res);
    }

    attachInterrupt(digitalPinToInterrupt(FL_pins.pinA), L_ISR_A, CHANGE);
    attachInterrupt(digitalPinToInterrupt(FL_pins.pinB_enc), L_ISR_B, CHANGE);
    attachInterrupt(digitalPinToInterrupt(FR_pins.pinA), R_ISR_A, CHANGE);
    attachInterrupt(digitalPinToInterrupt(FR_pins.pinB_enc), R_ISR_B, CHANGE);
    attachInterrupt(digitalPinToInterrupt(RL_pins.pinA), L2_ISR_A, CHANGE);
    attachInterrupt(digitalPinToInterrupt(RL_pins.pinB_enc), L2_ISR_B, CHANGE);
    attachInterrupt(digitalPinToInterrupt(RR_pins.pinA), R2_ISR_A, CHANGE);
    attachInterrupt(digitalPinToInterrupt(RR_pins.pinB_enc), R2_ISR_B, CHANGE);
}

// =======================================================
//  STOP：强电子刹车
// =======================================================
void brakeAll()
{
    // ⭐ 为了防止刹车时 PID 积分继续累积，暂时禁用 PID 逻辑
    // 但这个不是永久禁用，releaseBrake 会恢复它
    // usePID = false; (实际上不需要这一行，只要 target=0 即可)

    digitalWrite(FL_pins.pinF, HIGH); digitalWrite(FL_pins.pinB, HIGH);
    digitalWrite(FR_pins.pinF, HIGH); digitalWrite(FR_pins.pinB, HIGH);
    digitalWrite(RL_pins.pinF, HIGH); digitalWrite(RL_pins.pinB, HIGH);
    digitalWrite(RR_pins.pinF, HIGH); digitalWrite(RR_pins.pinB, HIGH);

    ledcWrite(FL_pins.pinPWM, 0);
    ledcWrite(FR_pins.pinPWM, 0);
    ledcWrite(RL_pins.pinPWM, 0);
    ledcWrite(RR_pins.pinPWM, 0);

    targetL = targetR = targetL2 = targetR2 = 0;
    errorSumL = errorSumR = errorSumL2 = errorSumR2 = 0;
}

// =======================================================
//  STOP 解除
// =======================================================
void releaseBrake()
{
    // 恢复常态（具体由 updateMotorControl 接管）
    digitalWrite(FL_pins.pinF, LOW); digitalWrite(FL_pins.pinB, LOW);
    digitalWrite(FR_pins.pinF, LOW); digitalWrite(FR_pins.pinB, LOW);
    digitalWrite(RL_pins.pinF, LOW); digitalWrite(RL_pins.pinB, LOW);
    digitalWrite(RR_pins.pinF, LOW); digitalWrite(RR_pins.pinB, LOW);
}

// =======================================================
//  设置目标 RPM
// =======================================================
void setTargetRPM(float fl, float fr, float rl, float rr)
{
    targetL  = fl;
    targetR  = fr;
    targetL2 = rl;
    targetR2 = rr;
}

// =======================================================
//  主控 PID 更新 (⭐ 核心修改：增加 if-else 切换)
// =======================================================
void updateMotorControl()
{
    // ⭐ 模式 B：Wall Following 开环控制
    if (!usePID) {
        // 直接把 target 当作占空比 (-100 ~ 100)
        // 这里的逻辑就是你发给我的 Wall Following 代码里的逻辑
        dutyL  = constrain((int)targetL,  -100, 100);
        dutyR  = constrain((int)targetR,  -100, 100);
        dutyL2 = constrain((int)targetL2, -100, 100);
        dutyR2 = constrain((int)targetR2, -100, 100);
        
        applyMotorRaw(dutyL, dutyR, dutyL2, dutyR2);
        return;
    }

    // ⭐ 模式 A：PID 闭环控制 (完全保留你的原始代码)
    unsigned long now = millis();

    // ======== 更新 RPM (每 200ms) ========
    if (now - lastRPMTime >= 200)
    {
        float dt = (now - lastRPMTime) / 1000.0;
        lastRPMTime = now;

        static long lastCountL = 0, lastCountR = 0, lastCountL2 = 0, lastCountR2 = 0;

        noInterrupts();
        long cL = L_encoderCount;
        long cR = R_encoderCount;
        long cL2 = L2_encoderCount;
        long cR2 = R2_encoderCount;
        interrupts();

        rpmL  = ( (cL  - lastCountL ) / (float)PPR_WHEEL ) * (60.0 / dt);
        rpmR  = ( (cR  - lastCountR ) / (float)PPR_WHEEL ) * (60.0 / dt);
        rpmL2 = ( (cL2 - lastCountL2) / (float)PPR_WHEEL ) * (60.0 / dt);
        rpmR2 = ( (cR2 - lastCountR2) / (float)PPR_WHEEL ) * (60.0 / dt);

        lastCountL  = cL;
        lastCountR  = cR;
        lastCountL2 = cL2;
        lastCountR2 = cR2;
    }

    // ======== 执行 PID (每 50ms) ========
    if (now - lastPIDTime >= 50)
    {
        float dt = (now - lastPIDTime) / 1000.0;
        lastPIDTime = now;

        errorL  = targetL  - rpmL;
        errorR  = targetR  - rpmR;
        errorL2 = targetL2 - rpmL2;
        errorR2 = targetR2 - rpmR2;

        const float Kff = 0.5;
        float uLff  = Kff * targetL;
        float uRff  = Kff * targetR;
        float uL2ff = Kff * targetL2;
        float uR2ff = Kff * targetR2;

        errorSumL  = constrain(errorSumL  + errorL  * dt, -ERROR_SUM_MAX, ERROR_SUM_MAX);
        errorSumR  = constrain(errorSumR  + errorR  * dt, -ERROR_SUM_MAX, ERROR_SUM_MAX);
        errorSumL2 = constrain(errorSumL2 + errorL2 * dt, -ERROR_SUM_MAX, ERROR_SUM_MAX);
        errorSumR2 = constrain(errorSumR2 + errorR2 * dt, -ERROR_SUM_MAX, ERROR_SUM_MAX);

        float dL  = (errorL  - prevErrorL)  / dt;
        float dR  = (errorR  - prevErrorR)  / dt;
        float dL2 = (errorL2 - prevErrorL2) / dt;
        float dR2 = (errorR2 - prevErrorR2) / dt;

        prevErrorL  = errorL;
        prevErrorR  = errorR;
        prevErrorL2 = errorL2;
        prevErrorR2 = errorR2;

        float uL  = uLff  + Kp * errorL  + Ki * errorSumL  + Kd * dL;
        float uR  = uRff  + Kp * errorR  + Ki * errorSumR  + Kd * dR;
        float uL2 = uL2ff + Kp * errorL2 + Ki * errorSumL2 + Kd * dL2;
        float uR2 = uR2ff + Kp * errorR2 + Ki * errorSumR2 + Kd * dR2;

        dutyL  = constrain((int)uL,  -100, 100);
        dutyR  = constrain((int)uR,  -100, 100);
        dutyL2 = constrain((int)uL2, -100, 100);
        dutyR2 = constrain((int)uR2, -100, 100);

        applyMotorRaw(dutyL, dutyR, dutyL2, dutyR2);
    }
}

// ================== ISR 部分 (保持不变) ==================
void IRAM_ATTR L_ISR_A() {
    unsigned long now = micros(); if (now - L_lastPulse < debounce_us) return;
    L_lastPulse = now;
    int a = digitalRead(FL_pins.pinA);
    int b = digitalRead(FL_pins.pinB_enc);
    if (a == b) L_encoderCount++; else L_encoderCount--;
}
void IRAM_ATTR L_ISR_B() {
    unsigned long now = micros(); if (now - L_lastPulse < debounce_us) return;
    L_lastPulse = now;
    int a = digitalRead(FL_pins.pinA);
    int b = digitalRead(FL_pins.pinB_enc);
    if (a != b) L_encoderCount++; else L_encoderCount--;
}

void IRAM_ATTR R_ISR_A() {
    unsigned long now = micros(); if (now - R_lastPulse < debounce_us) return;
    R_lastPulse = now;
    int a = digitalRead(FR_pins.pinA);
    int b = digitalRead(FR_pins.pinB_enc);
    if (a == b) R_encoderCount++; else R_encoderCount--;
}
void IRAM_ATTR R_ISR_B() {
    unsigned long now = micros(); if (now - R_lastPulse < debounce_us) return;
    R_lastPulse = now;
    int a = digitalRead(FR_pins.pinA);
    int b = digitalRead(FR_pins.pinB_enc);
    if (a != b) R_encoderCount++; else R_encoderCount--;
}

void IRAM_ATTR L2_ISR_A() {
    unsigned long now = micros(); if (now - L2_lastPulse < debounce_us) return;
    L2_lastPulse = now;
    int a = digitalRead(RL_pins.pinA);
    int b = digitalRead(RL_pins.pinB_enc);
    if (a == b) L2_encoderCount++; else L2_encoderCount--;
}
void IRAM_ATTR L2_ISR_B() {
    unsigned long now = micros(); if (now - L2_lastPulse < debounce_us) return;
    L2_lastPulse = now;
    int a = digitalRead(RL_pins.pinA);
    int b = digitalRead(RL_pins.pinB_enc);
    if (a != b) L2_encoderCount++; else L2_encoderCount--;
}

void IRAM_ATTR R2_ISR_A() {
    unsigned long now = micros(); if (now - R2_lastPulse < debounce_us) return;
    R2_lastPulse = now;
    int a = digitalRead(RR_pins.pinA);
    int b = digitalRead(RR_pins.pinB_enc);
    if (a == b) R2_encoderCount++; else R2_encoderCount--;
}
void IRAM_ATTR R2_ISR_B() {
    unsigned long now = micros(); if (now - R2_lastPulse < debounce_us) return;
    R2_lastPulse = now;
    int a = digitalRead(RR_pins.pinA);
    int b = digitalRead(RR_pins.pinB_enc);
    if (a != b) R2_encoderCount++; else R2_encoderCount--;
}