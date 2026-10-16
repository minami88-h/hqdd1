#pragma once
#include <stdint.h>

// ボタン(PA3, SET_ID)によるCAN ID設定 + LED(ID表示/負荷検出)+ ビープ。
// 全てノンブロッキング(millis()ベースの状態機械)。loop()から毎回 ui_update() を呼ぶ。

void ui_init();
void ui_update();
// ビープ中はmain.cpp側でmotor.loopFOC()/move()を止め、位相電圧をui.cppに
// 委譲してもらう必要があるため公開する。
bool ui_is_beeping();
