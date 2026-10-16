#include "can_link.h"
#include <Arduino.h>
#include <string.h>
#include <SimpleFOC.h>

// ---- FDCANドライバ(pierremolinaro/ACANFD_STM32からG431向けだけを取り込み) --
// 元ライブラリはヘッダ内で対応ボードを "#ifdef ARDUINO_NUCLEO_xxx" で判定して
// おり、未知のボード(hqdd_1はNucleoではない)だと#errorになる。G431向けの
// 分岐自体は実際にはMCU依存(FDCANの固定RAMレイアウト)だけでNucleo固有の
// 処理は無いが、ビルド全体でARDUINO_NUCLEO_G431KBを定義すると、STM32duino
// コア側の無関係なvariant_NUCLEO_G431KB.cpp(別基板用、本来は不使用)まで
// 有効化されてしまい、variant_generic.cppとdigitalPin/analogInputPinテーブルが
// 二重定義になりリンクエラーになる(実際に発生を確認して分かった)。
// そのため、必要なファイルだけ src/can_driver/ に取り込み、ボード判定の
// 分岐を「G431固定」に書き換えて使っている(詳細はcan_driver内の変更コメント参照)。
#include "can_driver/ACANFD_STM32_Settings.h"
#include "can_driver/ACANFD-STM32-fixed-ram-sections.h"

extern BLDCMotor motor;
extern Commander command;

static const uint32_t CAN_CMD_ID_BASE = 0x100;  // 受信: 0x100 + id
static const uint32_t CAN_FB_ID_BASE = 0x200;   // 送信: 0x200 + id
static const uint32_t CAN_REPLY_ID_BASE = 0x300;  // 送信: ゲイン問い合わせの応答 0x300 + id
static const uint32_t FB_PERIOD_MS = 20;        // Web UIのグラフ用に50Hz(1Mbpsなら負荷は無視できる)

static const std::initializer_list<ACANFD_STM32::PinPort> fdcan1_tx_pins{
    ACANFD_STM32::PinPort(PA_12, 9),  // hqdd_1基板の配線: CAN TX = PA12
};
static const std::initializer_list<ACANFD_STM32::PinPort> fdcan1_rx_pins{
    ACANFD_STM32::PinPort(PA_11, 9),  // hqdd_1基板の配線: CAN RX = PA11
};

static ACANFD_STM32 fdcan1(FDCAN1, SRAMCAN_BASE,
                            ACANFD_STM32::IRQs(FDCAN1_IT0_IRQn, FDCAN1_IT1_IRQn),
                            fdcan1_tx_pins, fdcan1_rx_pins);

extern "C" void FDCAN1_IT0_IRQHandler(void) { fdcan1.isr0(); }
extern "C" void FDCAN1_IT1_IRQHandler(void) { fdcan1.isr1(); }

static uint8_t s_deviceId = 0;
static uint32_t s_lastFbMs = 0;

static void startWithFilter(uint8_t deviceId) {
  ACANFD_STM32_Settings settings(1000UL * 1000, DataBitRateFactor::x1);
  ACANFD_STM32_StandardFilters filters;
  filters.addSingle((uint16_t)(CAN_CMD_ID_BASE + deviceId), ACANFD_STM32_FilterAction::FIFO0);
  settings.mNonMatchingStandardFrameReception = ACANFD_STM32_FilterAction::REJECT;
  fdcan1.beginFD(settings, filters);
}

void can_link_init(uint8_t deviceId) {
  s_deviceId = deviceId;
  startWithFilter(s_deviceId);
}

void can_link_set_id(uint8_t deviceId) {
  s_deviceId = deviceId;
  fdcan1.end();
  startWithFilter(s_deviceId);
}

// Serialと全く同じコマンド書式(例 "M0.3" "MC1" "ME0")をそのまま流用する。
// command.run()は先頭文字で登録済みハンドラ('M'/'S'/'L')へディスパッチする
// public APIで、CAN用の別パーサーを新たに書く必要がない。
// "?VP" のようなゲイン問い合わせ。応答: ID=0x300+id、data[0..1]=名前、data[2]=1(成功)/0、
// data[4..7]=float(リトルエンディアン)。
static void replyGain(const uint8_t* payload, uint8_t len) {
  CANFDMessage r;
  r.id = CAN_REPLY_ID_BASE + s_deviceId;
  r.ext = false;
  r.type = CANFDMessage::CAN_DATA;
  r.len = 8;
  memset(r.data, 0, 8);
  float v = 0;
  bool ok = len >= 3 && gain_get((const char*)&payload[1], &v);
  if (len >= 3) { r.data[0] = payload[1]; r.data[1] = payload[2]; }
  r.data[2] = ok ? 1 : 0;
  memcpy(&r.data[4], &v, 4);
  fdcan1.tryToSendReturnStatusFD(r);
}

static void dispatchCommand(const uint8_t* payload, uint8_t len) {
  if (len >= 1 && payload[0] == '?') { replyGain(payload, len); return; }
  if (len >= 1 && (payload[0] == 'W' || payload[0] == 'F')) {   // 保存/出荷状態: 応答 data[0..1]="WS"/"FR", data[2]=結果
    bool save = payload[0] == 'W';
    int r = save ? settings_save_now() : settings_reset_now();
    CANFDMessage a;
    a.id = CAN_REPLY_ID_BASE + s_deviceId;
    a.ext = false; a.type = CANFDMessage::CAN_DATA; a.len = 8;
    memset(a.data, 0, 8);
    a.data[0] = save ? 'W' : 'F'; a.data[1] = save ? 'S' : 'R'; a.data[2] = (uint8_t)r;
    fdcan1.tryToSendReturnStatusFD(a);
    return;
  }
  char buf[10];
  uint8_t n = len > 8 ? 8 : len;
  memcpy(buf, payload, n);
  buf[n] = '\n';
  buf[n + 1] = '\0';
  command.run(buf);
}

static void sendFeedback() {
  float angle_out = motor.shaft_angle / GEAR_RATIO;
  float vel_out = motor.shaft_velocity / GEAR_RATIO;
  float iq = motor.current.q;
  float torque_out = iq * MOTOR_KT_NM_PER_A * GEAR_RATIO;

  CANFDMessage frame;
  frame.id = CAN_FB_ID_BASE + s_deviceId;
  frame.ext = false;
  frame.type = CANFDMessage::CAN_DATA;
  frame.len = 8;
  frame.data_s16[0] = (int16_t)constrain(angle_out * 1000.0f, -32767.0f, 32767.0f);
  frame.data_s16[1] = (int16_t)constrain(vel_out * 100.0f, -32767.0f, 32767.0f);
  frame.data_s16[2] = (int16_t)constrain(iq * 1000.0f, -32767.0f, 32767.0f);
  frame.data_s16[3] = (int16_t)constrain(torque_out * 1000.0f, -32767.0f, 32767.0f);
  fdcan1.tryToSendReturnStatusFD(frame);
}

void can_link_update() {
  CANFDMessage frame;
  if (fdcan1.receiveFD0(frame)) {
    dispatchCommand(frame.data, frame.len);
  }

  uint32_t now = millis();
  if (now - s_lastFbMs >= FB_PERIOD_MS) {
    s_lastFbMs = now;
    sendFeedback();
  }
}
