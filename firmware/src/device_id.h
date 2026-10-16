#pragma once
#include <stdint.h>

// CANバス用デバイスID(0〜7)のフラッシュ永続化。
// 起動時に device_id_init() で読み出し、変更時のみ device_id_set() で書き込む。

// プログラムを書き込み直した直後のID(ボタンで変えたIDは書き込みで既定値に戻る)。
// 複数台を書き込むときは、台ごとにここを変えてビルドするか、書き込み後にボタンで設定する。
#define DEFAULT_DEVICE_ID 0

void device_id_init();
uint8_t device_id_get();
// id を保存する。実際に値が変化した時だけフラッシュへ書き込む(摩耗軽減)。
void device_id_set(uint8_t id);
