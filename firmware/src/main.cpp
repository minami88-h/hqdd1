/* ============================================================
 *  SimpleFOC 電流モード(current) FOC — hqdd_1 custom board
 *  MCU    : STM32G431KBUx (UFQFPN32)
 *  Driver : DRV8311H (基板上, 3x PWM + nSLEEP) + 低サイド電流センス
 *  Sensor : AS5600L (I2C)
 *  Motor  : ジンバル(磁石14 -> pole_pairs=7)
 *
 *  ベースは Nucleo-G474RE + DRV8311Hモジュール版のFOC設定(別プロジェクト g4_foc/src/main.cpp)。
 *  モーター本体・FOCの各種ゲインは変更していない。変更したのは
 *  ピン配置とボード固有の初期化(Serial)のみ。
 *
 *  ---- ピン対応 (hardware/mcu.kicad_sch から実機配線を解析して確定) ----
 *    DRV8311 INHA/INHB/INHC <- PA8/PA9/PA10 (TIM1, 低サイドCS同期に必須)
 *    DRV8311 nSLEEP         <- PB0  (enable, アクティブHigh)
 *    DRV8311 SOA/SOB        -> PA0/PA1 (ADC1_IN1/IN2, 電流センス)
 *    DRV8311 GAIN            = +3.3V 直結 (2V/A)  -> CSENSE_VpA=2.0
 *    DRV8311 INLA/INLB/INLC  = +3.3V 直結 (3xPWMモードの定石通り)
 *    DRV8311 MODE            = NC (Hi-Z, 9A OCP)
 *    AS5600L SDA/SCL        <- PB7/PB8 (I2C1, AF4)
 *    SWDIO/SWCLK            <- PA13/PA14 (J2, 外付けST-Linkで書き込み)
 *    デバッグSerial(USART2)  : TX=PA2 / RX=PA15 (J3, 外付けUSB-TTL)
 *      ※回路図のネット名は "UART_RX"=PA2 / "UART_TX"=PA15 (外部機器視点の
 *        命名)。MCUのハードウェアUSART2としてはPA2=TX/PA15=RXでないと
 *        動かない。USB-TTLアダプタの TXD を基板PA2に、RXDを基板PA15に
 *        接続すること。(platformio.iniでPIN_SERIAL_TX/RXを固定済み)
 *
 *  ---- CAN通信(FDCAN1) / ボタンID / 負荷LED ----
 *    CAN (FDCAN1)  : TX=PA12 / RX=PA11 (TCAN330経由、1Mbps)
 *      TCAN330のSTANDBY(S)/SHDNピンはジャンパ線で解決済み。
 *      受信ID=0x100+id、送信(テレメトリ)ID=0x200+id。詳細はcan_link.h/.cpp。
 *    SET_ID(押しボタン)=PA3 : 2秒長押しでID変更モード、押下回数(1〜8)で
 *      id=回数-1を確定・フラッシュ保存。詳細はui.h/.cpp、device_id.h/.cpp。
 *    LED1=PA5 : 現在IDを(id+1)回点滅で表示。LED0=PA4 : 負荷検出(|Iq|閾値)。
 * ============================================================ */
#include <SimpleFOC.h>
#include "device_id.h"
#include "can_link.h"
#include "ui.h"
#include "settings_store.h"

// ====== 設定 (Nucleo-G474RE版から変更なし) ======
#define POLE_PAIRS      7
#define SUPPLY_VOLTAGE  8.0f
#define CURRENT_LIMIT   1.0f     // [A]
#define CSENSE_VpA      2.0f     // GAIN=AVDD相当(+3.3V直結) -> 2 V/A

TwoWire I2Cbus = TwoWire(PB7, PB8);
MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);
BLDCDriver3PWM driver = BLDCDriver3PWM(PA8, PA9, PA10, PB0);
LowsideCurrentSense currentSense = LowsideCurrentSense(CSENSE_VpA, 1.0f, PA0, PA1);
BLDCMotor motor = BLDCMotor(POLE_PAIRS);

Commander command = Commander(Serial);
// 積分・フィルタ状態を捨てる(モード切替/停止時に、溜まった積分で回り続けるのを防ぐ)
void resetControllers() {
  motor.PID_velocity.reset();
  motor.PID_current_q.reset();
  motor.PID_current_d.reset();
}
static bool isTorqueDomain(MotionControlType t) {
  return t == MotionControlType::torque || t == MotionControlType::angle_nocascade;
}
// 'M<値>' のようなターゲット値指定は出力軸基準の入力として扱い、減速比を掛けて
// モーター軸基準に変換してから適用する(Serial/CAN共通。CAN経由のコマンドも
// can_link.cppからcommand.run()->ここに来るので、ここ1箇所の変換で両方に効く)。
// モード切替・PIDチューニング等、値そのもの以外のコマンドは従来通りCommanderに渡す。
static void syncD();
void doMotor(char* cmd) {
  MotionControlType prev = motor.controller;
  bool isValue = isDigit(cmd[0]) || cmd[0] == '-' || cmd[0] == '+';
  bool isGet = cmd[0] == '\0' || cmd[0] == '\n' || cmd[0] == '\r';
  if (isValue) {
    float outputVal = atof(cmd);
    motor.target = gearOutputToMotor(outputVal, isTorqueDomain(motor.controller));
    Serial.print(F("Target: ")); Serial.println(outputVal, 4);
  } else if (isGet) {
    Serial.println(gearMotorToOutput(motor.target, isTorqueDomain(motor.controller)), 4);
  } else {
    command.motor(&motor, cmd);
    syncD();   // MQP/MQI等でqが変わったらdも追従
  }
  if (motor.controller != prev) {
    resetControllers();
    // モード切替直後に古い目標値へ飛ばないようにする。位置モードに入った時は
    // 目標=現在位置(その場保持)、それ以外は0。クライアント側の送信順に依存しない。
    bool angleTarget = motor.controller == MotionControlType::angle ||
                       motor.controller == MotionControlType::angle_nocascade;
    motor.target = angleTarget ? motor.shaft_angle : 0;
  }
}

// ---- パラメータ表: 読み出し('?VP')・設定('P')・保存('W')・出荷状態('F')・Serial行頭ID ----
// 設定可能な値を1つの表で管理する。出荷状態 = 起動時にコードが設定した値(下のdefs)。
float g_gear_ratio = 29.16f;       // 減速比
static float g_il = CURRENT_LIMIT;  // 電流上限 [A]
static float g_vl = 60.0f;          // 速度上限 [rad/s, モーター軸]
static float g_ul = 6.0f;           // 電圧上限 [V] (電流PIDの出力上限)
static float g_sv = SUPPLY_VOLTAGE; // 電源電圧 [V] (PWM変調の基準。バッテリーに合わせる)
static float g_av = 5.0f;           // 起動時アライメント電圧 [V] (次回起動から有効)

struct Param { const char* name; float* p; float lo; float hi; float def; };
// 並びは settings_store の保存順。名前は Commander の 'M' 以降と同じ2文字(VP/VI/...)。
static Param paramTable[SETTINGS_NPARAM] = {
  {"VP", &motor.PID_velocity.P, 0, 1e4f, 0},  {"VI", &motor.PID_velocity.I, 0, 1e4f, 0},
  {"VD", &motor.PID_velocity.D, 0, 1e4f, 0},  {"VF", &motor.LPF_velocity.Tf, 0, 1.0f, 0},
  {"AP", &motor.P_angle.P, 0, 1e4f, 0},
  {"QP", &motor.PID_current_q.P, 0, 1e4f, 0}, {"QI", &motor.PID_current_q.I, 0, 1e5f, 0},
  {"GR", &g_gear_ratio, 0.1f, 1000.0f, 0},
  {"IL", &g_il, 0.05f, 5.0f, 0},   {"VL", &g_vl, 1.0f, 300.0f, 0},
  {"UL", &g_ul, 0.5f, 17.0f, 0},   {"SV", &g_sv, 5.0f, 17.0f, 0},
  {"AV", &g_av, 0.5f, 12.0f, 0},
};
static Param* findParam(const char* n) {
  for (Param& e : paramTable) if (n[0] == e.name[0] && n[1] == e.name[1]) return &e;
  return nullptr;
}
// 電流dは常に電流qと同じ値(q/d共通)。qのゲインが変わりうる所で必ず呼ぶ。
static void syncD() {
  motor.PID_current_d.P = motor.PID_current_q.P;
  motor.PID_current_d.I = motor.PID_current_q.I;
}
// 制限値・電源電圧をモーター/ドライバへ反映(いつ呼んでも安全)。
// 電源電圧が変わるとPWMのデューティ換算(V→duty)が変わる。電圧[V]で指令する電流PIDは
// 電源電圧を正しく設定すれば、ゲインを変えずにLiPo 2セル(2S)/3セル(3S)どちらでも同じ特性になる。
static void applyParams() {
  driver.voltage_power_supply = g_sv;
  driver.voltage_limit = g_sv;
  motor.updateVoltageLimit(min(g_ul, g_sv));
  motor.updateCurrentLimit(g_il);
  motor.updateVelocityLimit(g_vl);
}
static void captureDefaults() { for (Param& e : paramTable) e.def = *e.p; }

bool gain_get(const char* n, float* out) {
  if (n[0] == 'M' && n[1] == 'C') { *out = (float)(int)motor.controller; return true; }  // 現在の制御モード(UIの同期用)
  Param* e = findParam(n);
  if (!e) return false;
  *out = *e->p;
  return true;
}
// 複数台が同じUARTバスに乗るため、Serialに出す行は "#<id> " で始める。
static void printIdPrefix() { Serial.print('#'); Serial.print(device_id_get()); Serial.print(' '); }
static void printGain(char a, char b, float v) {
  printIdPrefix(); Serial.print(F("GAIN ")); Serial.print(a); Serial.print(b);
  Serial.print(' '); Serial.println(v, 5);
}
void doQuery(char* c) {
  float v;
  if (c[0] && c[1] && gain_get(c, &v)) printGain(c[0], c[1], v);
  else { printIdPrefix(); Serial.println(F("GAIN ?")); }
}
// 'P<名前2文字><値>': 表のパラメータを設定(範囲外は無視)。応答は現在値。
void doParam(char* c) {
  Param* e = (c[0] && c[1]) ? findParam(c) : nullptr;
  if (!e) { printIdPrefix(); Serial.println(F("GAIN ?")); return; }
  if (isDigit(c[2]) || c[2] == '.') {
    float v = atof(&c[2]);
    if (v >= e->lo && v <= e->hi) { *e->p = v; applyParams(); }
  }
  printGain(e->name[0], e->name[1], *e->p);
}
// 'G<減速比>': 'PGR..'の別名(従来互換)。
void doGear(char* c) {
  char buf[20]; snprintf(buf, sizeof(buf), "GR%s", c);
  doParam(buf);
}
// 'W': 表の全値をフラッシュへ保存。書き込み中は数十msCPUが止まるので、
// 回転中(|ω|>2rad/s)は拒否する。
int settings_save_now() {   // 1=成功 0=失敗 2=回転中で拒否
  if (fabsf(motor.shaft_velocity) > 2.0f) return 2;
  float v[SETTINGS_NPARAM];
  for (int i = 0; i < SETTINGS_NPARAM; i++) v[i] = *paramTable[i].p;
  return settings_save(v) ? 1 : 0;
}
void doSave(char*) {
  int r = settings_save_now();
  printIdPrefix();
  Serial.println(r == 1 ? F("SAVED") : r == 2 ? F("SAVE BUSY (回転中)") : F("SAVE FAILED"));
}
// 'F': 全パラメータを出荷状態(コードの初期値)に戻し、保存内容も消す。
int settings_reset_now() {  // 1=成功 0=失敗 2=回転中で拒否
  if (fabsf(motor.shaft_velocity) > 2.0f) return 2;
  for (Param& e : paramTable) *e.p = e.def;
  syncD();
  applyParams();
  resetControllers();
  return settings_erase() ? 1 : 0;
}
void doFactory(char*) {
  int r = settings_reset_now();
  printIdPrefix();
  Serial.println(r == 1 ? F("RESET OK") : r == 2 ? F("RESET BUSY (回転中)") : F("RESET FAILED"));
}
// 'T1'/'T0': Serialの周期テレメトリのON/OFF。複数台が1本のバスに乗る時、
// 問い合わせ対象以外を黙らせるために使う(CANには影響しない)。
static bool g_serialTelem = true;
void doTelem(char* c) { g_serialTelem = (c[0] != '0'); }

// Serial行の宛先振り分け。"@<id> <cmd>"=そのIDだけ、"@* <cmd>"=全台(S/Tのみ許可)、
// 先頭が'@'でない行は従来通り(単体で端末から操作する用)。
static void serialPoll() {
  static char line[48];
  static uint8_t n = 0;
  static bool overflow = false;
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\r') continue;
    if (ch != '\n') {
      if (n < sizeof(line) - 1) line[n++] = ch; else overflow = true;
      continue;
    }
    line[n] = 0; n = 0;
    if (overflow) { overflow = false; continue; }
    char* p = line;
    if (*p == '@') {
      p++;
      if (*p == '*') {
        p++;
        while (*p == ' ') p++;
        if (*p != 'S' && *p != 'T') continue;   // 一斉送信は停止・テレメトリ設定のみ
      } else {
        if (!isDigit(*p)) continue;
        int id = 0;
        while (isDigit(*p)) id = id * 10 + (*p++ - '0');
        if (id != device_id_get()) continue;
        while (*p == ' ') p++;
      }
    }
    if (!*p) continue;
    // '?'はCommanderではコマンド一覧表示に予約されているので、ここで先に横取りする
    if (*p == '?') { doQuery(p + 1); continue; }
    char buf[48];
    snprintf(buf, sizeof(buf), "%s\n", p);
    command.run(buf);
  }
}

// 'Z': 現在位置を出力軸の0位置にする。整流に使う電気角はsensor->getMechanicalAngle()
// から作られ、sensor_offsetは位置制御側(shaftAngle)にだけ効くので、モーターの
// 通電位相には影響しない。位置モード中は目標も0にして、その場に保持する。
void doZero(char*) {
  motor.sensor_offset += motor.shaft_angle;
  motor.shaft_angle = 0;
  resetControllers();
  if (motor.controller == MotionControlType::angle ||
      motor.controller == MotionControlType::angle_nocascade) {
    motor.target = 0;
  }
}
// 'S': 速度0指令で減速→停止(フリーラン防止)。その後 'MC0' 等で好きなモードへ
void doStop(char*) {
  resetControllers();
  motor.controller = MotionControlType::velocity;
  motor.target = 0;
}

// ---- 高速キャプチャ(デバッグ用): 'L' で開始、溜まったらCSV出力 ----
#define CAP_N 600
static float cap[CAP_N][4];   // v, Iq, Vq, sa
static uint32_t cap_t[CAP_N];
static int cap_i = CAP_N;     // CAP_N = idle
void doCap(char*) { cap_i = 0; }

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println(F("=== hqdd_1 (STM32G431KBUx) CURRENT-mode FOC ==="));
  SimpleFOCDebug::enable(&Serial);

  // センサー
  I2Cbus.begin();
  I2Cbus.setClock(400000);
  // I2Cスキャン: AS5600(0x36)が見えるか
  Serial.println(F("I2C scan:"));
  { uint8_t found=0;
    for (uint8_t a=1;a<127;a++){ I2Cbus.beginTransmission(a);
      if (I2Cbus.endTransmission()==0){ Serial.print(F("  found 0x")); Serial.println(a,HEX); found++; } }
    if(!found) Serial.println(F("  !! NO I2C device (AS5600配線/3V3/プルアップ確認)")); }
  sensor.init(&I2Cbus);
  // 速度推定ノイズ対策: Δt最小値を5msに (Nucleo-G474RE版と同じ理由)
  sensor.min_elapsed_time = 0.005f;
  motor.linkSensor(&sensor);
  Serial.println(F("sensor check (手で軸を回すと値が変わるはず):"));
  for(int i=0;i<5;i++){ sensor.update(); Serial.print(F("  ang=")); Serial.println(sensor.getAngle(),3); delay(120); }

  // ドライバ
  driver.voltage_power_supply = g_sv;
  driver.voltage_limit = g_sv;
  if (!driver.init()) { Serial.println(F("driver init failed!")); return; }
  motor.linkDriver(&driver);

  // 電流センス: ドライバ有効化→0V出力で静定→その後オフセット校正、の順を厳守
  driver.enable();
  motor.voltage_limit = g_ul;
  motor.voltage_sensor_align = g_av;
  motor.init();
  motor.enable();
  motor.setPhaseVoltage(0, 0, 0);
  delay(300);

  currentSense.linkDriver(&driver);
  if (currentSense.init() != 1)
    Serial.println(F("current sense init failed!"));
  // 自動アライメントはスキップし、検証済みの正しい構成を固定
  currentSense.skip_align = true;
  motor.linkCurrentSense(&currentSense);

  // 制御モード: 電流FOC
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor.torque_controller = TorqueControlType::foc_current;
  motor.controller = MotionControlType::torque;

  // 電流PID (q/d 共通)
  // 内側ループ: 速度ループより十分速く。LPFを短くして遅れを減らし、Iを上げて
  // 電圧飽和後の巻き戻りも速くする。発振/ブーン音が出たらPかTfを戻す。
  motor.PID_current_q.P = 0.8f;  motor.PID_current_q.I = 60.0f;
  motor.PID_current_d.P = 0.8f;  motor.PID_current_d.I = 60.0f;
  motor.LPF_current_q.Tf = 0.001f;
  motor.LPF_current_d.Tf = 0.001f;
  motor.updateTorqueControlType(TorqueControlType::foc_current);
  motor.updateCurrentLimit(g_il);   // PID_velocity.limit も電流上限に揃う

  // 速度PID (電流モード: 出力は電流指令[A])
  // 実機計測で決定: このジンバルは慣性が極小(Iq 0.1Aで~900rad/s^2)で、P>=0.04で
  // 11Hz付近の発振、I>=0.05で3-5Hzの発振が出る。コギング(12スロット)由来の
  // ~20Hzリップルは残る。位置応答高速化のため P=0.03/I=0.03/Tf=8ms、
  // P_angle=30、velocity_limit=60 に更新(1rad t90~45ms, 6rad t90~110ms, OS<7%)。
  motor.PID_velocity.P = 0.03f;
  motor.PID_velocity.I = 0.03f;
  motor.PID_velocity.D = 0.0f;
  motor.PID_velocity.output_ramp = 0;
  motor.LPF_velocity.Tf = 0.008f;
  // 最高速: 高速域は電気角の位相遅れで不安定になるので抑える
  motor.updateVelocityLimit(g_vl);

  // 位置P (出力は目標速度[rad/s])。速度ループ帯域の1/5以下 -> 減速が間に合う
  motor.P_angle.P = 30.0f;

  motor.useMonitoring(Serial);

  // 電気角アライメント(電流センスalignはskip_alignで無効)
  // ここまでのコード上の値が「出荷状態」。控えてから、保存済みの設定があれば上書きする。
  syncD();
  captureDefaults();
  { float v[SETTINGS_NPARAM];
    if (settings_load(v)) {
      for (int i = 0; i < SETTINGS_NPARAM; i++)
        if (v[i] >= paramTable[i].lo && v[i] <= paramTable[i].hi) *paramTable[i].p = v[i];
      syncD();
      Serial.println(F("settings loaded from flash"));
    } }
  applyParams();
  motor.voltage_sensor_align = min(g_av, min(g_ul, g_sv));   // initFOCのアライメントに使う

  if (!motor.initFOC()) { Serial.println(F("initFOC failed!")); return; }

  motor.target = 0.0f;
  command.add('M', doMotor, "motor");
  command.add('S', doStop, "stop");
  command.add('L', doCap, "capture");
  command.add('?', doQuery, "gain query");
  command.add('G', doGear, "gear ratio");
  command.add('P', doParam, "set parameter");
  command.add('F', doFactory, "factory reset");
  command.add('W', doSave, "save settings");
  command.add('T', doTelem, "serial telemetry on/off");
  command.add('Z', doZero, "zero position");

  device_id_init();
  can_link_init(device_id_get());
  ui_init();

  Serial.println(F("ready. CURRENT-torque: 'M0.3'=0.3A 'M0'=0."));
  Serial.println(F("  速度モード:'MC1'後 'M10'=10rad/s / 位置:'MC2'後 'M<rad>'"));
}

void loop() {
  // ビープ中はui.cppが位相電圧を直接制御するので、通常のFOC出力は止める。
  if (!ui_is_beeping()) {
    motor.loopFOC();
    motor.move();
  }
  serialPoll();
  can_link_update();
  ui_update();
  static uint32_t lastcap = 0;
  if (cap_i < CAP_N && micros() - lastcap >= 1000) {
    lastcap = micros();
    cap_t[cap_i] = lastcap;
    cap[cap_i][0] = motor.shaft_velocity; cap[cap_i][1] = motor.current.q;
    cap[cap_i][2] = motor.voltage.q;      cap[cap_i][3] = motor.shaft_angle;
    if (++cap_i == CAP_N) {
      Serial.println(F("CAPBEGIN"));
      for (int i = 0; i < CAP_N; i++) {
        Serial.print(cap_t[i] - cap_t[0]); Serial.print(',');
        Serial.print(cap[i][0], 2); Serial.print(',');
        Serial.print(cap[i][1], 3); Serial.print(',');
        Serial.print(cap[i][2], 2); Serial.print(',');
        Serial.println(cap[i][3], 4);
      }
      Serial.println(F("CAPEND"));
    }
  }
  if (cap_i < CAP_N) return;   // キャプチャ中は他の出力を止める
  static unsigned long t0 = 0;
  static unsigned long iters = 0;
  iters++;
  if (millis() - t0 > 200) {
    // v/tgt/sa は出力軸基準(減速比換算後)で表示。Iq/Id/Vqはモーター軸の生の
    // 電気量なので変換しない(電流PIDチューニング等で従来通り使えるように)。
    bool torqueDomain = isTorqueDomain(motor.controller);
    if (g_serialTelem) {
    printIdPrefix();
    Serial.print(F("v=")); Serial.print(gearMotorToOutput(motor.shaft_velocity, false), 2);
    Serial.print(F(" tgt=")); Serial.print(gearMotorToOutput(motor.target, torqueDomain), 2);
    Serial.print(F(" Iq=")); Serial.print(motor.current.q, 2);
    Serial.print(F(" Id=")); Serial.print(motor.current.d, 2);
    Serial.print(F(" Vq=")); Serial.print(motor.voltage.q, 2);
    Serial.print(F(" sa=")); Serial.print(gearMotorToOutput(motor.shaft_angle, false), 2);
    Serial.print(F(" trq=")); Serial.print(motor.current.q * MOTOR_KT_NM_PER_A * GEAR_RATIO, 4);
    Serial.print(F(" loopHz=")); Serial.println((float)iters * 1000.0f / (millis() - t0), 0);
    }
    t0 = millis(); iters = 0;
  }
}
