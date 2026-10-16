#pragma once
#include <stdint.h>

// ビルドごとに変わる識別子(scripts/gen_build_id.py が build_id_gen.cpp を生成)。
// フラッシュ保存データに付けて、書き込み直したら古い保存内容を無効にするために使う。
extern const uint32_t BUILD_ID;
