#include "ui.h"
#include <Arduino.h>
#include <SimpleFOC.h>
#include "device_id.h"
#include "can_link.h"

extern BLDCMotor motor;

static const int PIN_BTN = PA3;
static const int PIN_LED_ID = PA5;    // 物理LED1 -> ID表示
static const int PIN_LED_LOAD = PA4;  // 物理LED0 -> 負荷検出

// ---- ボタン デバウンス ----------------------------------------------------
static const uint32_t DEBOUNCE_MS = 25;
static bool s_rawPressed = false;
static bool s_stablePressed = false;
static uint32_t s_rawChangeMs = 0;

// ---- ID設定 状態機械 -------------------------------------------------------
static const uint32_t LONG_PRESS_MS = 2000;
static const uint32_t IDLE_TIMEOUT_MS = 2000;
static const uint32_t PRESS_FLASH_MS = 300;
static const uint32_t ERROR_BLINK_MS = 600;  // 100ms on/off x3

enum class Mode { NORMAL, WAIT_RELEASE_AFTER_LONGPRESS, ID_SET };
static Mode s_mode = Mode::NORMAL;
static uint32_t s_pressStartMs = 0;
static uint32_t s_lastActivityMs = 0;
static uint8_t s_pressCount = 0;
static uint32_t s_flashUntilMs = 0;
static bool s_errorActive = false;
static uint32_t s_errorStartMs = 0;

// ---- 負荷検出LED(ヒステリシス) --------------------------------------------
// ★実機計測: このジンバルモーターは慣性・摩擦が極小で、8rad/s巡航中でも
//   Iqは0.01〜0.03A程度にしかならない(当初の閾値0.3A/0.08Aでは実動作中
//   ほぼ届かず、LEDが点かなかった真因はこれ)。電流だけでは「動作中」を
//   安定して検出できないため、(1)targetが非ゼロ(=モーターに指令中)、
//   (2)|Iq|が閾値超(=target 0でも外力で電流が出ている=真の負荷)
//   のどちらかでON、両方とも収まったらOFFにする。
static const float LOAD_IQ_ON_A = 0.02f;
static const float LOAD_IQ_OFF_A = 0.01f;
static const float LOAD_TARGET_EPS = 0.01f;
static bool s_loadOn = false;

// ---- ビープ(d軸電圧トグル、ノンブロッキング) --------------------------------
// ★音量アップ: 振幅を0.3V->1.0Vへ(d軸電圧なので理想的にはトルクを生まない。
//   位相抵抗が数Ωとしても1A未満でDRV8311のOCP(5A)に対して十分余裕がある)。
//   周波数も人の耳が最も感度の高い2kHz付近へ変更。
static const float BEEP_VOLT = 1.0f;
static const uint32_t BEEP_TONE_MS = 150;
static const uint32_t BEEP_GAP_MS = 100;
static const uint32_t BEEP_HALF_PERIOD_US = 250;  // ~2kHz
static bool s_beepActive = false;
static bool s_beepInTone = false;
static uint8_t s_beepRemaining = 0;
static uint32_t s_beepPhaseMs = 0;
static uint32_t s_beepToggleUs = 0;
static bool s_beepSign = false;
static float s_beepAngle = 0;

static void startBeep(uint8_t count) {
  if (count == 0) return;
  s_beepActive = true;
  s_beepInTone = true;
  s_beepRemaining = count;
  s_beepPhaseMs = millis();
  s_beepToggleUs = micros();
  s_beepSign = true;
  s_beepAngle = motor.electrical_angle;
}

static void updateBeep() {
  if (!s_beepActive) return;
  uint32_t nowMs = millis();
  if (s_beepInTone) {
    if (micros() - s_beepToggleUs >= BEEP_HALF_PERIOD_US) {
      s_beepToggleUs = micros();
      s_beepSign = !s_beepSign;
    }
    motor.setPhaseVoltage(0.0f, s_beepSign ? BEEP_VOLT : -BEEP_VOLT, s_beepAngle);
    if (nowMs - s_beepPhaseMs >= BEEP_TONE_MS) {
      motor.setPhaseVoltage(0.0f, 0.0f, s_beepAngle);
      s_beepRemaining--;
      if (s_beepRemaining == 0) {
        s_beepActive = false;
      } else {
        s_beepInTone = false;
        s_beepPhaseMs = nowMs;
      }
    }
  } else {
    if (nowMs - s_beepPhaseMs >= BEEP_GAP_MS) {
      s_beepInTone = true;
      s_beepPhaseMs = nowMs;
      s_beepToggleUs = micros();
      s_beepSign = true;
    }
  }
}

bool ui_is_beeping() { return s_beepActive; }

// ---- ID表示LED(millis()だけから計算する、状態を持たないパターン) -------------
static void updateLedId() {
  uint32_t now = millis();

  if (s_errorActive) {
    if (now - s_errorStartMs >= ERROR_BLINK_MS) {
      s_errorActive = false;
    } else {
      bool on = ((now - s_errorStartMs) % 200) < 100;
      digitalWrite(PIN_LED_ID, on ? HIGH : LOW);
      return;
    }
  }

  if (s_mode == Mode::ID_SET || s_mode == Mode::WAIT_RELEASE_AFTER_LONGPRESS) {
    bool on = (now < s_flashUntilMs) || ((now % 200) < 100);
    digitalWrite(PIN_LED_ID, on ? HIGH : LOW);
    return;
  }

  // 通常時: (id+1)回の短点滅 -> 長い消灯 -> 繰り返し
  uint8_t id = device_id_get();
  uint32_t blinkTotalMs = (uint32_t)(id + 1) * 300;
  uint32_t cycleMs = blinkTotalMs + 700;
  uint32_t t = now % cycleMs;
  bool on = (t < blinkTotalMs) && ((t % 300) < 150);
  digitalWrite(PIN_LED_ID, on ? HIGH : LOW);
}

static void updateLedLoad() {
  float iq = fabsf(motor.current.q);
  bool commanded = fabsf(motor.target) > LOAD_TARGET_EPS;
  if (!s_loadOn && (commanded || iq > LOAD_IQ_ON_A)) {
    s_loadOn = true;
  } else if (s_loadOn && !commanded && iq < LOAD_IQ_OFF_A) {
    s_loadOn = false;
  }
  digitalWrite(PIN_LED_LOAD, s_loadOn ? HIGH : LOW);
}

// ---- ボタン読み取り + ID確定処理 -------------------------------------------
static void finalizeIdSet() {
  if (s_pressCount >= 1 && s_pressCount <= 8) {
    uint8_t newId = s_pressCount - 1;
    if (newId != device_id_get()) {
      device_id_set(newId);
      can_link_set_id(newId);
    }
    startBeep(newId + 1);
  } else {
    // 範囲外(0回 or 9回以上): 誤って別IDに変わる事故を避けるため、
    // クランプせず無視する。エラー点滅で知らせる。
    s_errorActive = true;
    s_errorStartMs = millis();
  }
  s_mode = Mode::NORMAL;
  s_pressCount = 0;
}

static void updateButton() {
  bool raw = digitalRead(PIN_BTN) == LOW;  // INPUT_PULLUPでアクティブLow想定
  uint32_t now = millis();

  if (raw != s_rawPressed) {
    s_rawPressed = raw;
    s_rawChangeMs = now;
  } else if ((now - s_rawChangeMs) >= DEBOUNCE_MS && s_stablePressed != s_rawPressed) {
    bool wasPressed = s_stablePressed;
    s_stablePressed = s_rawPressed;

    if (!wasPressed && s_stablePressed) {
      // 立ち上がり(押下確定)
      s_pressStartMs = now;
      if (s_mode == Mode::NORMAL) {
        // 長押し判定はupdateButton末尾のタイマーで行う
      } else if (s_mode == Mode::ID_SET) {
        s_pressCount++;
        s_lastActivityMs = now;
        s_flashUntilMs = now + PRESS_FLASH_MS;
      }
    } else if (wasPressed && !s_stablePressed) {
      // 立ち下がり(解放確定)
      if (s_mode == Mode::WAIT_RELEASE_AFTER_LONGPRESS) {
        s_mode = Mode::ID_SET;
        s_pressCount = 0;
        s_lastActivityMs = now;
      }
    }
  }

  // 長押し検出(NORMAL中、押しっぱなしのまま2秒経過)
  if (s_mode == Mode::NORMAL && s_stablePressed && (now - s_pressStartMs) >= LONG_PRESS_MS) {
    s_mode = Mode::WAIT_RELEASE_AFTER_LONGPRESS;
  }

  // ID_SETタイムアウト
  if (s_mode == Mode::ID_SET && (now - s_lastActivityMs) >= IDLE_TIMEOUT_MS) {
    finalizeIdSet();
  }
}

void ui_init() {
  pinMode(PIN_BTN, INPUT_PULLUP);
  pinMode(PIN_LED_ID, OUTPUT);
  pinMode(PIN_LED_LOAD, OUTPUT);
  digitalWrite(PIN_LED_ID, LOW);
  digitalWrite(PIN_LED_LOAD, LOW);
  // 起動時に現在IDをビープで知らせる(id+1回)
  startBeep(device_id_get() + 1);
}

void ui_update() {
  updateButton();
  updateBeep();
  updateLedId();
  if (!s_beepActive) updateLedLoad();
}
