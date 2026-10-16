# ビルドごとに固有のIDを持つ src/build_id_gen.cpp を生成する(PlatformIOのpreスクリプト)。
# フラッシュに保存した設定(PIDゲイン・制限値・デバイスID)はこのIDと一緒に記録され、
# 書き込み直してIDが変わると無効になり、コード上の値が使われる。
Import("env")
import os, time

path = os.path.join(env["PROJECT_DIR"], "src", "build_id_gen.cpp")
bid = int(time.time()) & 0xFFFFFFFF
with open(path, "w") as f:
    f.write('// 自動生成(scripts/gen_build_id.py)。編集しない・コミットしない。\n'
            '#include "build_id.h"\n'
            f'const uint32_t BUILD_ID = 0x{bid:08X}UL;\n')
