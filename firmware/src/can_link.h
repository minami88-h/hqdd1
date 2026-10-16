#pragma once
#include <stdint.h>

// CAN通信(FDCAN1、クラシックCAN互換、1Mbps)。
// 受信: ID = 0x100 + deviceId、ペイロードはSerialと同じコマンド文字列(例 "M0.3")を
//       そのまま command.run() に渡す(main.cppのdoMotor()経由で減速比変換も適用される)。
// 送信: ID = 0x200 + deviceId、200ms周期で出力軸換算のテレメトリを送る。
//
// 減速比は外部とのやり取りの境界(Serial/CANどちらの'M'ターゲット値・テレメトリも)で
// 適用し、内部のPID制御自体はモーター軸基準のまま変更しない。
// 5.4f or 29.16fに変更すればそちらの減速比に切り替わる。
// 実行時に変更可能(UIから'G'コマンド、settings_storeでフラッシュ保存)。既定値はmain.cppの
// g_gear_ratio初期値。外部とのやり取りの境界で適用する点は変わらない。
extern float g_gear_ratio;
#define GEAR_RATIO (g_gear_ratio)

// ジンバル2204 260KVのトルク定数。メーカー公開値なし(汎用品)のため、
// Kt[Nm/A] = 60/(2π・KV) の理論式から算出(実測ではないので±10-20%程度の誤差を見込む)。
constexpr float MOTOR_KT_NM_PER_A = 0.0367f;

// 出力軸基準値 <-> モーター軸基準値の変換。
// トルク(電流)モードは減速機でトルクが増幅される分、モーター側は小さくて済むので
// 向きが角度/速度と逆になる(掛け算/割り算が入れ替わる)。
inline float gearOutputToMotor(float outputVal, bool isTorqueMode) {
  return isTorqueMode ? outputVal / GEAR_RATIO : outputVal * GEAR_RATIO;
}
inline float gearMotorToOutput(float motorVal, bool isTorqueMode) {
  return isTorqueMode ? motorVal * GEAR_RATIO : motorVal / GEAR_RATIO;
}

void can_link_init(uint8_t deviceId);
// ボタンでIDが変わった時に呼ぶ。受信フィルタを即座に再設定する。
void can_link_set_id(uint8_t deviceId);
// loop()から毎回呼ぶ(ノンブロッキング)。受信コマンド処理+周期フィードバック送信。
void can_link_update();

// PIDゲイン読み出し(main.cppで定義)。nameは"VP"等の2文字。
bool gain_get(const char* name, float* out);
// ゲイン・減速比のフラッシュ保存(main.cppで定義)。1=成功 0=失敗 2=回転中で拒否
int settings_save_now();
// 全パラメータを出荷状態に戻し保存内容も消す(main.cppで定義)。戻り値は同上
int settings_reset_now();
