#pragma once
#include <stdint.h>

// UIから変更したPIDゲイン・減速比・制限値のフラッシュ永続化(電源を切っても保持)。
// 保存先はIDページ(63)の1つ前のページ62。プログラムを書き込み直すと消える(=既定値に戻る)。
// フラッシュ書き込み中はCPUが数十ms止まるため、保存は停止中に明示操作('W')で行う。

constexpr int SETTINGS_NPARAM = 13;  // 並びは main.cpp の paramTable と一致させる

// 保存済みの有効な設定があれば true を返し、値を埋める。
bool settings_load(float v[SETTINGS_NPARAM]);
bool settings_save(const float v[SETTINGS_NPARAM]);
// 保存内容を消す(次回起動から出荷状態=コード上の値)。
bool settings_erase();
