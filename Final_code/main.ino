#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ESP32Servo.h>
#include <Wire.h>

#include "vive_tracking.h"
#include "motor_control.h"
#include "webpage_vive_points.h"
#include "tof_sensors.h" // ⭐ 包含 ToF

// ============================================================
//                WiFi AP 配置
// ============================================================
const char* AP_SSID = "Robot-Car-Final-Ultimate";
const char* AP_PASS = "12345678";
WebServer server(80);

// ============================================================
//           I2C / TopHat Health System
// ============================================================
#define I2C_SLAVE_ADDR 0x28
#define SDA_PIN 8
#define SCL_PIN 9
// ⭐ 必须保持 40kHz 以配合 TopHat
#define I2C_FREQ 40000 
const unsigned long I2C_PERIOD_MS = 500;
unsigned long lastI2CTime = 0;
uint8_t wifi_packets = 0;
uint8_t tophat_health = 255; 
bool isDead = false; 
bool is_blind_forward = false;          // 是否处于 1.5s 盲走期
unsigned long blind_forward_start = 0;  // 开始盲走的时间戳
const unsigned long BLIND_FORWARD_MS = 1500; // 前进时长 1.5秒

// ========================= I2C 工具函数 =========================
void send_I2C_byte(uint8_t data) {
  Wire.beginTransmission(I2C_SLAVE_ADDR);
  Wire.write(data);
  Wire.endTransmission();
}

uint8_t receive_I2C_byte() {
  uint8_t count = Wire.requestFrom(I2C_SLAVE_ADDR, (uint8_t)1);
  if (count > 0 && Wire.available()) {
    return Wire.read();
  } else {
    return tophat_health; // 读不到保持原值
  }
}

// ============================================================
//                导航参数
// ============================================================
float FINAL_X = 0.0, FINAL_Y = 0.0;
float TARGET_X = 0.0, TARGET_Y = 0.0;
float TARGET_HEADING = 0.0f;

const float POS_TOL   = 50.0f;
const float ANGLE_TOL = 0.04f;
const int MOVE_RPM        = 100;
const int CORRECTION_GAIN = 10;

// ============================================================
//                障碍与目标参数
// ============================================================

const float TOWER_X_MIN=4280.0, TOWER_X_MAX=4988.0;
const float TOWER_Y_MIN=3612.0, TOWER_Y_MAX=4369.0, TOWER_BYPASS_X=5226.0;
const float NEXUS_Y_LOW_MAX=1659.0, NEXUS_Y_HIGH_MIN=6415.0;
const float LOWER_RAMP_X_MAX=3697.0, LOWER_RAMP_Y_MAX=1930.0;
const float UPPER_RAMP_X_MAX=3697.0, UPPER_RAMP_Y_MIN=6210.0;
const float MID_X=4071.0, LOWER_MID_Y=1782.0, UPPER_MID_Y=6415.0;
const float RED_NEXUS_X=4530.0, RED_NEXUS_Y=1954.0;
const float BLUE_NEXUS_X=4553.0, BLUE_NEXUS_Y=6276.0;
const float LOW_TOWER_BLUE_X=4618.0, LOW_TOWER_BLUE_Y=3560.0;
const float LOW_TOWER_RED_X=4618.0, LOW_TOWER_RED_Y=4640.0;
const float HIGH_TOWER_BLUE_X=3200.0, HIGH_TOWER_BLUE_Y=3423.0;
const float HIGH_TOWER_RED_X=3200.0, HIGH_TOWER_RED_Y=4597.0;
// ============================================================
//                状态机与任务
// ============================================================
enum State { STATE_ALIGN_Y, STATE_ALIGN_X, STATE_STOP };
enum PathMode { PATH_Y_THEN_X, PATH_X_THEN_Y, PATH_LOWER_RAMP, PATH_UPPER_RAMP };
enum HighLevelTask {
  TASK_NONE, TASK_GOTO_POINT,
  TASK_ATTACK_RED, TASK_ATTACK_BLUE,
  TASK_CAPTURE_LOW_TOWER_BLUE, TASK_CAPTURE_LOW_TOWER_RED,
  TASK_CAPTURE_HIGH_TOWER_BLUE, TASK_CAPTURE_HIGH_TOWER_RED,
  TASK_SEQUENCE_ATTACK_ALL,
  TASK_WALL_FOLLOW // ⭐ 新增
};

State currentState = STATE_STOP;
PathMode currentPath = PATH_Y_THEN_X;
HighLevelTask currentTask = TASK_NONE;

bool path_decided = false;
bool brake_released = false;
bool has_target = false;
int rampPhase = 0;
bool towerBypassActive = false;
bool is_intermediate_move = false;
int sequenceStep = 0;

// ============================================================
//                Attack & Capture 状态
// ============================================================
enum AttackState { ATTACK_IDLE, ATTACK_FORWARD, ATTACK_BACKWARD, ATTACK_PAUSE, ATTACK_DONE };
AttackState attackState = ATTACK_IDLE;
int attackCount = 0;
int targetAttackHits = 4;
unsigned long attackPhaseStart = 0;

const int ATTACK_RPM = 40;
const unsigned long ATTACK_FORWARD_TIME_RED  = 650;
const unsigned long ATTACK_BACKWARD_TIME_RED = 450;
const unsigned long ATTACK_FORWARD_TIME_BLUE = 650;
const unsigned long ATTACK_BACKWARD_TIME_BLUE = 450;
const unsigned long ATTACK_PAUSE_TIME   = 1000;
const int           ATTACK_HITS_TOTAL   = 4;

enum CaptureState { CAPTURE_IDLE, CAPTURE_FORWARD, CAPTURE_HOLD, CAPTURE_BACKWARD, CAPTURE_DONE };
enum CaptureAxis { CAPTURE_AXIS_Y, CAPTURE_AXIS_X };
CaptureState captureState = CAPTURE_IDLE;
CaptureAxis captureAxis = CAPTURE_AXIS_Y;
int captureDirToward = 0;
unsigned long capturePhaseStart = 0;

const int CAPTURE_RPM = 40;
const unsigned long CAPTURE_FORWARD_TIME  = 4000;
const unsigned long CAPTURE_BACKWARD_TIME = 450;
const unsigned long CAPTURE_HOLD_TIME     = 15000;

// ============================================================
//                Wall Follow Params (⭐ 新增)
// ============================================================
const int LEFT_TOF_INDEX = 1;
const int FRONT_TOF_INDEX = 0;
const float D_TARGET_MM = 110.0f;
const float BASE_DUTY = 40.0f; 
const float KP_WALL = 0.5f; 
const float KD_WALL = 0.15f;
const float MAX_TURN_DUTY = 30.0f;
const float MIN_FORWARD_DUTY = 20.0f;
const float FRONT_ENTER_THRESH_MM = 230.0f;
const float FRONT_CLEAR_THRESH_MM = 230.0f;
const float TURN_IN_PLACE_DUTY = 65.0f;

enum WallState { FOLLOW_WALL_S, TURN_CORNER_S };
WallState wallState = FOLLOW_WALL_S;
float prevError_WALL = 0.0f;
unsigned long lastFollowTime = 0;

// ============================================================
//                Servo & Manual
// ============================================================
const int SERVO_PIN = 36;
const int SERVO_MIN_DEG = 30;
const int SERVO_MAX_DEG = 150;
const int SERVO_STEP_DEG = 5;
const unsigned long SERVO_INTERVAL_MS = 30;

Servo attackServo;
enum ServoMode { SERVO_OFF, SERVO_MANUAL, SERVO_AUTO };
ServoMode servoMode = SERVO_OFF;
float servoCurDeg = 90.0f;
int   lastServoDeg = -1; // 记录上次角度，防抖动
unsigned long lastServoUpdate = 0;
int servoDir = +1;

String curMode = "stop";
int curRPM = 0;
bool manualMode = false;

static inline bool ieq(const String& a, const String& b) { return a.equalsIgnoreCase(b); }

// ============================================================
//                辅助函数
// ============================================================
float angNorm(float a) {
  while (a > 3.1415926f) a -= 6.2831852f;
  while (a < -3.1415926f) a += 6.2831852f;
  return a;
}
float computeHeading() { return angNorm(atan2(vive2_y - vive1_y, vive2_x - vive1_x)); }
float getRobotX() { return (vive1_x + vive2_x) / 2.0f; }
float getRobotY() { return (vive1_y + vive2_y) / 2.0f; }

void setMecanumSpeed(int vx, int vy, int w) {
  setTargetRPM(vy + vx - w, vy - vx + w, vy - vx - w, vy + vx + w);
}

// ---------------- Tower 碰撞检查 ----------------
bool verticalHitsTower(float x, float y1, float y2) {
  if (x < TOWER_X_MIN || x > TOWER_X_MAX) return false;
  float ym = min(y1, y2), yM = max(y1, y2);
  return !(yM < TOWER_Y_MIN || ym > TOWER_Y_MAX);
}
bool horizontalHitsTower(float y, float x1, float x2) {
  if (y < TOWER_Y_MIN || y > TOWER_Y_MAX) return false;
  float xm = min(x1, x2), xM = max(x1, x2);
  return !(xM < TOWER_X_MIN || xm > TOWER_X_MAX);
}
bool pathYthenX_hitsTower(float x0, float y0, float xt, float yt) {
  return verticalHitsTower(x0, y0, yt) || horizontalHitsTower(yt, x0, xt);
}
bool pathXthenY_hitsTower(float x0, float y0, float xt, float yt) {
  return horizontalHitsTower(y0, x0, xt) || verticalHitsTower(xt, y0, yt);
}
bool targetInNexusBand() { return (FINAL_Y <= NEXUS_Y_LOW_MAX) || (FINAL_Y >= NEXUS_Y_HIGH_MIN); }
bool lowerRampTarget() { return (FINAL_X < LOWER_RAMP_X_MAX) && (FINAL_Y < LOWER_RAMP_Y_MAX); }
bool upperRampTarget() { return (FINAL_X < UPPER_RAMP_X_MAX) && (FINAL_Y > UPPER_RAMP_Y_MIN); }
bool inTowerBandX(float x) { return (x > TOWER_X_MIN) && (x < TOWER_X_MAX); }
bool crossTowerY(float y1, float y2) { return (y1 < TOWER_Y_MIN && y2 > TOWER_Y_MAX) || (y1 > TOWER_Y_MAX && y2 < TOWER_Y_MIN); }

void resetNavigation(float x, float y) {
  FINAL_X = x; FINAL_Y = y;
  TARGET_X = x; TARGET_Y = y;
  has_target = true; path_decided = false; is_intermediate_move = false;
  rampPhase = 0; towerBypassActive = false;
  currentState = STATE_ALIGN_Y;
  brakeAll(); brake_released = false;
  is_blind_forward = true; 
  blind_forward_start = millis();
}

// ============================================================
//                Servo 逻辑
// ============================================================
void applyServo() {
  int target = (int)servoCurDeg;
  if (target != lastServoDeg) {
    attackServo.write(target);
    lastServoDeg = target;
  }
}

void updateServoAuto() {
  if (servoMode != SERVO_AUTO) return;
  unsigned long now = millis();
  if (now - lastServoUpdate < SERVO_INTERVAL_MS) return;
  lastServoUpdate = now;

  servoCurDeg += servoDir * SERVO_STEP_DEG;
  if (servoCurDeg >= SERVO_MAX_DEG) { servoCurDeg = SERVO_MAX_DEG; servoDir = -1; }
  else if (servoCurDeg <= SERVO_MIN_DEG) { servoCurDeg = SERVO_MIN_DEG; servoDir = +1; }
  
  applyServo(); 
}

void handleServo() {
  wifi_packets++;
  String mode = server.arg("mode");
  if (ieq(mode, "manual")) {
    int ang = server.arg("angle").toInt();
    servoCurDeg = constrain(ang, 0, 180);
    servoMode = SERVO_MANUAL;
    applyServo(); 
  } else if (ieq(mode, "auto")) {
    servoMode = SERVO_AUTO;
  } else if (ieq(mode, "stop")) {
    servoMode = SERVO_OFF;
  }
  server.send(200, "text/plain", "OK");
}

// ============================================================
//                HTTP Handlers (Manual & Tasks)
// ============================================================
void setTargetsFromModeRPM(const String& mode, int val) {
  curMode = mode; curRPM = val;
  if (ieq(mode, "stop")) { 
    brakeAll(); 
    setTargetRPM(0,0,0,0); 
    manualMode = false; 
    usePID = true; // ⭐ Stop时恢复PID
    return; 
  }
  manualMode = true; 
  usePID = true; // ⭐ 手动模式开启PID
  releaseBrake();
  float rpm = constrain(val, 0, 200);
  if (ieq(mode, "forward")) setTargetRPM(rpm,rpm,rpm,rpm);
  else if (ieq(mode, "back")) setTargetRPM(-rpm,-rpm,-rpm,-rpm);
  else if (ieq(mode, "left")) setTargetRPM(-rpm,rpm,rpm,-rpm);
  else if (ieq(mode, "right")) setTargetRPM(rpm,-rpm,-rpm,rpm);
  else if (ieq(mode, "cw")) setTargetRPM(rpm,-rpm,rpm,-rpm);
  else if (ieq(mode, "ccw")) setTargetRPM(-rpm,rpm,-rpm,rpm);
}

void handleCmd() {
  wifi_packets++;
  setTargetsFromModeRPM(server.arg("mode"), server.arg("rpm").toInt());
  server.send(200, "text/plain", "OK");
}

void handleSetTarget() {
  wifi_packets++;
  currentTask = TASK_GOTO_POINT; 
  usePID = true; manualMode = false;
  resetNavigation(server.arg("x").toFloat(), server.arg("y").toFloat());
  server.send(200, "text/plain", "Target Set");
}
void handleAttackRed() {
  wifi_packets++; currentTask = TASK_ATTACK_RED; targetAttackHits = 4;
  attackState = ATTACK_IDLE; attackCount = 0; manualMode = false; usePID = true;
  resetNavigation(RED_NEXUS_X, RED_NEXUS_Y);
  server.send(200, "text/plain", "Attack RED");
}
void handleAttackBlue() {
  wifi_packets++; currentTask = TASK_ATTACK_BLUE; targetAttackHits = 4;
  attackState = ATTACK_IDLE; attackCount = 0; manualMode = false; usePID = true;
  resetNavigation(BLUE_NEXUS_X, BLUE_NEXUS_Y);
  server.send(200, "text/plain", "Attack BLUE");
}
void handleAttackSequence() {
  wifi_packets++; currentTask = TASK_SEQUENCE_ATTACK_ALL; manualMode = false; usePID = true;
  sequenceStep = 1; targetAttackHits = 1; attackState = ATTACK_IDLE; attackCount = 0;
  resetNavigation(LOW_TOWER_BLUE_X, LOW_TOWER_BLUE_Y);
  server.send(200, "text/plain", "Seq Started");
}
void handleCaptureLowBlue() {
  wifi_packets++; currentTask = TASK_CAPTURE_LOW_TOWER_BLUE; captureState = CAPTURE_IDLE; manualMode = false; usePID = true;
  resetNavigation(LOW_TOWER_BLUE_X, LOW_TOWER_BLUE_Y);
  server.send(200, "text/plain", "Cap Low Blue");
}
void handleCaptureLowRed() {
  wifi_packets++; currentTask = TASK_CAPTURE_LOW_TOWER_RED; captureState = CAPTURE_IDLE; manualMode = false; usePID = true;
  resetNavigation(LOW_TOWER_RED_X, LOW_TOWER_RED_Y);
  server.send(200, "text/plain", "Cap Low Red");
}
void handleCaptureHighBlue() {
  wifi_packets++; currentTask = TASK_CAPTURE_HIGH_TOWER_BLUE; captureState = CAPTURE_IDLE; manualMode = false; usePID = true;
  resetNavigation(HIGH_TOWER_BLUE_X, HIGH_TOWER_BLUE_Y);
  server.send(200, "text/plain", "Cap High Blue");
}
void handleCaptureHighRed() {
  wifi_packets++; currentTask = TASK_CAPTURE_HIGH_TOWER_RED; captureState = CAPTURE_IDLE; manualMode = false; usePID = true;
  resetNavigation(HIGH_TOWER_RED_X, HIGH_TOWER_RED_Y);
  server.send(200, "text/plain", "Cap High Red");
}

// ⭐ 新增 Wall Follow 路由
void handleWallFollow() {
  wifi_packets++;
  currentTask = TASK_WALL_FOLLOW;
  usePID = false; // ⭐ 关闭 PID，切到开环模式
  manualMode = false; has_target = false;
  
  wallState = FOLLOW_WALL_S; 
  lastFollowTime = millis();
  prevError_WALL = 0.0f;
  
  releaseBrake(); brake_released = true;
  server.send(200, "text/plain", "Wall Follow Started");
}

void handleStop() {
  wifi_packets++; brakeAll(); setTargetRPM(0,0,0,0);
  brake_released = false; has_target = false; path_decided = false;
  towerBypassActive = false; currentState = STATE_STOP; currentTask = TASK_NONE;
  attackState = ATTACK_IDLE; captureState = CAPTURE_IDLE; manualMode = false;
  usePID = true; // ⭐ Stop时恢复PID
  server.send(200, "text/plain", "STOP OK");
}

// ============================================================
//                Logic Loops
// ============================================================
void updateAttack() {
  if (attackState == ATTACK_IDLE || attackState == ATTACK_DONE) return;
  unsigned long now = millis();
  bool isRed = (currentTask == TASK_ATTACK_RED || currentTask == TASK_CAPTURE_LOW_TOWER_RED || currentTask == TASK_CAPTURE_HIGH_TOWER_RED);
  if (currentTask == TASK_SEQUENCE_ATTACK_ALL) isRed = false;
  int vy = isRed ? -ATTACK_RPM : ATTACK_RPM;
  unsigned long fwdT = (currentTask == TASK_ATTACK_RED) ? ATTACK_FORWARD_TIME_RED : ATTACK_FORWARD_TIME_BLUE;
  unsigned long bwdT = (currentTask == TASK_ATTACK_RED) ? ATTACK_BACKWARD_TIME_RED : ATTACK_BACKWARD_TIME_BLUE;

  switch (attackState) {
    case ATTACK_FORWARD:
      if (!brake_released) { releaseBrake(); brake_released = true; }
      setMecanumSpeed(0, vy, 0);
      if (now - attackPhaseStart >= fwdT) { attackPhaseStart = now; attackState = ATTACK_BACKWARD; }
      break;
    case ATTACK_BACKWARD:
      if (!brake_released) { releaseBrake(); brake_released = true; }
      setMecanumSpeed(0, -vy, 0);
      if (now - attackPhaseStart >= bwdT) { attackPhaseStart = now; attackState = ATTACK_PAUSE; attackCount++; brakeAll(); brake_released = false; }
      break;
    case ATTACK_PAUSE:
      brakeAll(); brake_released = false;
      if (attackCount >= targetAttackHits) {
        attackState = ATTACK_DONE;
        if (currentTask != TASK_SEQUENCE_ATTACK_ALL) { currentTask = TASK_NONE; has_target = false; }
      } else if (now - attackPhaseStart >= ATTACK_PAUSE_TIME) {
        attackPhaseStart = now; attackState = ATTACK_FORWARD;
      }
      break;
    default: brakeAll(); break;
  }
}

void updateCapture() {
  if (captureState == CAPTURE_IDLE || captureState == CAPTURE_DONE) return;
  unsigned long now = millis();
  int vx=0, vy=0;
  if(captureAxis==CAPTURE_AXIS_Y) vy = captureDirToward*CAPTURE_RPM; else vx = captureDirToward*CAPTURE_RPM;

  switch (captureState) {
    case CAPTURE_FORWARD:
      if(!brake_released){releaseBrake();brake_released=true;}
      setMecanumSpeed(vx, vy, 0);
      if(now - capturePhaseStart >= CAPTURE_FORWARD_TIME) { brakeAll(); brake_released=false; capturePhaseStart=now; captureState=CAPTURE_HOLD; }
      break;
    case CAPTURE_HOLD:
      brakeAll(); brake_released=false;
      if(now - capturePhaseStart >= CAPTURE_HOLD_TIME) { capturePhaseStart=now; captureState=CAPTURE_BACKWARD; }
      break;
    case CAPTURE_BACKWARD:
      if(!brake_released){releaseBrake();brake_released=true;}
      setMecanumSpeed(-vx, -vy, 0);
      if(now - capturePhaseStart >= CAPTURE_BACKWARD_TIME) { brakeAll(); brake_released=false; captureState=CAPTURE_DONE; currentTask=TASK_NONE; has_target=false; }
      break;
    default: brakeAll(); break;
  }
}

// ⭐ Wall Follow Logic
void updateWallFollow() {
  updateToF(); // 读取距离
  float d_left = tof[LEFT_TOF_INDEX];
  float d_front = tof[FRONT_TOF_INDEX];

  switch (wallState) {
    case FOLLOW_WALL_S: {
      unsigned long now = millis();
      float dt = (now - lastFollowTime) / 1000.0f;
      lastFollowTime = now;

      // 前方遇障碍 -> 转弯
      if (d_front > 0 && d_front < FRONT_ENTER_THRESH_MM) {
        wallState = TURN_CORNER_S;
        prevError_WALL = 0.0f;
        break;
      }

      // PD 控制
      float error = d_left - D_TARGET_MM;
      float d_term = 0.0f;
      if (dt > 0) d_term = KD_WALL * ((error - prevError_WALL) / dt);
      prevError_WALL = error;

      float turn = KP_WALL * error + d_term;
      if (turn > MAX_TURN_DUTY) turn = MAX_TURN_DUTY;
      if (turn < -MAX_TURN_DUTY) turn = -MAX_TURN_DUTY;

      float dL = BASE_DUTY - turn;
      float dR = BASE_DUTY + turn;
      
      // 限制幅度
      if (dL >= 0) dL = max(dL, MIN_FORWARD_DUTY);
      if (dR >= 0) dR = max(dR, MIN_FORWARD_DUTY);
      dL = constrain(dL, -100.0f, 100.0f);
      dR = constrain(dR, -100.0f, 100.0f);

      // 写入 target (此时作为 Duty)
      targetL = dL; targetL2 = dL;
      targetR = dR; targetR2 = dR;
      break;
    }
    case TURN_CORNER_S: {
      targetL = TURN_IN_PLACE_DUTY; targetL2 = TURN_IN_PLACE_DUTY;
      targetR = -TURN_IN_PLACE_DUTY; targetR2 = -TURN_IN_PLACE_DUTY;
      
      if (d_front > FRONT_CLEAR_THRESH_MM) {
        wallState = FOLLOW_WALL_S;
        lastFollowTime = millis();
      }
      break;
    }
  }
}

// ============================================================
//                SETUP & LOOP
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(300);
  
  // 初始化 Servo
  ESP32PWM::allocateTimer(3);
  attackServo.setPeriodHertz(50);
  attackServo.attach(SERVO_PIN, 500, 2500);
  attackServo.write(90);
  lastServoDeg = 90;

  // ⭐ I2C 初始化 (40kHz)
  Wire.begin(SDA_PIN, SCL_PIN, I2C_FREQ);
  
  initVives();
  initToF(); // 内部不再调用 Wire.begin
  initMotors();
  brakeAll();

  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);

  server.on("/", [](){ server.send_P(200, "text/html", PAGE_VIVE_POINTS); });
  server.on("/set_target", handleSetTarget);
  server.on("/attack_red", handleAttackRed);
  server.on("/attack_blue", handleAttackBlue);
  server.on("/attack_sequence", handleAttackSequence);
  server.on("/wall_follow", handleWallFollow); // ⭐ 注册
  server.on("/capture_low_blue", handleCaptureLowBlue);
  server.on("/capture_low_red", handleCaptureLowRed);
  server.on("/capture_high_blue", handleCaptureHighBlue);
  server.on("/capture_high_red", handleCaptureHighRed);
  server.on("/stop", handleStop);
  server.on("/cmd", handleCmd);
  server.on("/servo", handleServo);
  server.begin();

  Serial.println("System Ready.");
}

void loop() {
  server.handleClient();

  // =================== I2C Health Check ======================
  if (millis() - lastI2CTime >= I2C_PERIOD_MS) {
    lastI2CTime = millis();
    send_I2C_byte(wifi_packets);
    tophat_health = receive_I2C_byte();
    wifi_packets = 0;

    // 💀 死亡判定
    if (tophat_health == 0) {
      if (!isDead) {
        Serial.println("💀 DEAD!");
        isDead = true;
        brakeAll(); brake_released = false; servoMode = SERVO_OFF;
        currentTask = TASK_NONE; currentState = STATE_STOP;
        manualMode = false; has_target = false;
        attackState = ATTACK_IDLE; captureState = CAPTURE_IDLE;
        usePID = true; // 死的时候恢复 PID
      }
    } else {
      if (isDead) { Serial.println("✨ ALIVE!"); isDead = false; }
    }
  }

  // ⭐ “死亡锁”
  if (isDead) { 
    brakeAll(); 
    return; 
  }

  // =================== 正常控制 ======================
  updateVives();
  updateMotorControl();
  updateServoAuto();

  if (manualMode) return; 
  if (is_blind_forward) {
    // 检查是否还在 1.5秒内
    if (millis() - blind_forward_start < BLIND_FORWARD_MS) {
      // 1. 松刹车
      if (!brake_released) { releaseBrake(); brake_released = true; }
      
      // 2. 强制前进 (使用 MOVE_RPM 或你自己定义的速度，这里用 100)
      // 参数顺序: FL, FR, RL, RR
      setTargetRPM(MOVE_RPM, MOVE_RPM, MOVE_RPM, MOVE_RPM); 
      
      // 3. 必须调用这个让电机动起来
      updateMotorControl(); 
      updateVives(); // 顺便读一下坐标，保持数据新鲜
      
      return; // ⛔ 只要在盲走期，就直接下一轮循环，不执行后面的导航
    } else {
      // 时间到了！
      // 稍微刹车一下，让后面逻辑接管，或者直接平滑过渡
      brakeAll(); 
      brake_released = false;
      is_blind_forward = false; // 关闭标志位
    }
  }
  // 核心：根据 currentTask 决定调用谁
  if (currentTask == TASK_WALL_FOLLOW) {
    updateWallFollow(); 
    return; // 沿墙模式下，不要执行下面的 Vive 导航
  }

  if (attackState != ATTACK_IDLE && attackState != ATTACK_DONE) { updateAttack(); return; }
  if (captureState != CAPTURE_IDLE && captureState != CAPTURE_DONE) { updateCapture(); return; }

  if (!has_target) { brakeAll(); return; }

  float cur_x = getRobotX();
  float cur_y = getRobotY();
  float cur_theta = computeHeading();

  if (fabsf(cur_x) < 1.0f && fabsf(cur_y) < 1.0f) return;

  // ==========================================
  // ⭐ 核心逻辑：路径规划与预处理 (无省略)
  // ==========================================
  if (!path_decided) {
    is_intermediate_move = false;
    TARGET_X = FINAL_X; TARGET_Y = FINAL_Y;
    
    // 1. 低区脱困
    if (cur_y < 1883.0f) { 
      TARGET_X = cur_x; TARGET_Y = 2100.0f; 
      currentState = STATE_ALIGN_Y; is_intermediate_move = true;
    } 
    // 2. 塔区防横穿
    else if ((cur_y >= 3766.0f && cur_y <= 4518.0f) && (FINAL_Y >= 3766.0f && FINAL_Y <= 4518.0f)) {
      TARGET_X = cur_x; TARGET_Y = 3600.0f;
      currentState = STATE_ALIGN_Y; is_intermediate_move = true;
    } 
    // 3. 常规路径
    else { 
      if (!towerBypassActive && inTowerBandX(cur_x) && inTowerBandX(FINAL_X) && crossTowerY(cur_y, FINAL_Y)) {
        towerBypassActive = true; currentState = STATE_ALIGN_X; currentPath = PATH_X_THEN_Y;
      } else if (upperRampTarget() || currentTask == TASK_CAPTURE_HIGH_TOWER_BLUE || currentTask == TASK_CAPTURE_HIGH_TOWER_RED) {
        currentPath = PATH_UPPER_RAMP; currentState = STATE_ALIGN_X;
      } else if (lowerRampTarget()) {
        currentPath = PATH_LOWER_RAMP; currentState = STATE_ALIGN_X;
      } else {
         if (targetInNexusBand()) {
           if (!pathXthenY_hitsTower(cur_x, cur_y, FINAL_X, FINAL_Y)) { currentPath = PATH_X_THEN_Y; currentState = STATE_ALIGN_X; }
           else { currentPath = PATH_Y_THEN_X; currentState = STATE_ALIGN_Y; }
         } else {
           if (!pathYthenX_hitsTower(cur_x, cur_y, FINAL_X, FINAL_Y)) { currentPath = PATH_Y_THEN_X; currentState = STATE_ALIGN_Y; }
           else { currentPath = PATH_X_THEN_Y; currentState = STATE_ALIGN_X; }
         }
      }
    }
    path_decided = true; rampPhase = 0;
  }

  // 状态机执行移动
  float theta_err = angNorm(TARGET_HEADING - cur_theta);
  int w_correction = (fabsf(theta_err) > ANGLE_TOL) ? (theta_err > 0 ? CORRECTION_GAIN : -CORRECTION_GAIN) : 0;

  switch(currentState) {
    case STATE_ALIGN_Y: {
      float y_dest = TARGET_Y;
      if (!is_intermediate_move) {
         if ((currentPath==PATH_LOWER_RAMP||currentPath==PATH_UPPER_RAMP)) {
            if(rampPhase==1) y_dest=(currentPath==PATH_LOWER_RAMP)?LOWER_MID_Y:UPPER_MID_Y;
            else if(rampPhase==3) y_dest=TARGET_Y;
         }
      }
      float dy = y_dest - cur_y;
      if (fabsf(dy) < POS_TOL) {
        brakeAll(); brake_released=false; delay(50);
        if(is_intermediate_move) { path_decided=false; is_intermediate_move=false; }
        else if (currentPath==PATH_Y_THEN_X) currentState=STATE_ALIGN_X;
        else if (currentPath==PATH_LOWER_RAMP||currentPath==PATH_UPPER_RAMP) {
           if(rampPhase==1) {rampPhase=2; currentState=STATE_ALIGN_X;} else if(rampPhase==3) currentState=STATE_STOP;
        } else currentState=STATE_STOP;
      } else {
        if(!brake_released){releaseBrake();brake_released=true;}
        setMecanumSpeed(0, (dy>0)?MOVE_RPM:-MOVE_RPM, w_correction);
      }
    } break;

    case STATE_ALIGN_X: {
      float x_dest = TARGET_X;
      if (!is_intermediate_move) {
         if (currentPath==PATH_LOWER_RAMP||currentPath==PATH_UPPER_RAMP) { if(rampPhase==0) x_dest=MID_X; else if(rampPhase==2) x_dest=TARGET_X; }
         if (towerBypassActive) x_dest=TOWER_BYPASS_X;
      }
      float dx = x_dest - cur_x;
      if (fabsf(dx) < POS_TOL) {
        brakeAll(); brake_released=false; delay(50);
        if(towerBypassActive) {towerBypassActive=false; path_decided=false;}
        else if(currentPath==PATH_LOWER_RAMP||currentPath==PATH_UPPER_RAMP) {
          if(rampPhase==0) {rampPhase=1; currentState=STATE_ALIGN_Y;} else if(rampPhase==2) {rampPhase=3; currentState=STATE_ALIGN_Y;}
        } else {
          if (currentPath==PATH_X_THEN_Y) currentState=STATE_ALIGN_Y; else currentState=STATE_STOP;
        }
      } else {
        if(!brake_released){releaseBrake();brake_released=true;}
        setMecanumSpeed((dx>0)?MOVE_RPM:-MOVE_RPM, 0, w_correction);
      }
    } break;

    case STATE_STOP:
      brakeAll(); setTargetRPM(0,0,0,0); brake_released=false; has_target=false; path_decided=false;
      
      // 任务触发
      if (currentTask == TASK_ATTACK_RED || currentTask == TASK_ATTACK_BLUE) { attackState = ATTACK_FORWARD; attackPhaseStart = millis(); }
      else if (currentTask == TASK_SEQUENCE_ATTACK_ALL) {
        if (attackState == ATTACK_IDLE) { attackState = ATTACK_FORWARD; attackPhaseStart = millis(); }
        else if (attackState == ATTACK_DONE) {
           if (sequenceStep == 1) { sequenceStep=2; targetAttackHits=1; attackState=ATTACK_IDLE; resetNavigation(BLUE_NEXUS_X, BLUE_NEXUS_Y); }
           else if (sequenceStep == 2) { sequenceStep=3; targetAttackHits=1; attackState=ATTACK_IDLE; resetNavigation(HIGH_TOWER_BLUE_X, HIGH_TOWER_BLUE_Y); }
           else if (sequenceStep == 3) { currentTask=TASK_NONE; attackState=ATTACK_IDLE; }
        }
      }
      else if (currentTask >= TASK_CAPTURE_LOW_TOWER_BLUE && currentTask <= TASK_CAPTURE_HIGH_TOWER_RED) {
        if(currentTask==TASK_CAPTURE_LOW_TOWER_BLUE || currentTask==TASK_CAPTURE_LOW_TOWER_RED) {
           captureAxis=CAPTURE_AXIS_Y; captureDirToward=(currentTask==TASK_CAPTURE_LOW_TOWER_BLUE)?1:-1;
        } else { captureAxis=CAPTURE_AXIS_X; captureDirToward=-1; }
        captureState=CAPTURE_FORWARD; capturePhaseStart=millis();
      } else currentTask=TASK_NONE;
      break;
  }
}