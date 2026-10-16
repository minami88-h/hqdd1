# HQDD1 — 小型 QDD(準ダイレクトドライブ)アクチュエータ

ジンバルモーター(2204 / 260KV)+ 減速機を、自作の制御基板 **hqdd_1** で駆動する QDD アクチュエータの、基板設計・ファームウェア・操作用 Web UI をまとめたリポジトリです。

*A small quasi-direct-drive actuator: a custom STM32G431 + DRV8311H FOC driver board (KiCad files included), SimpleFOC-based firmware with CAN/UART control, and a browser UI (Web Serial / SocketCAN). Documentation is mostly in Japanese.*

| | |
|---|---|
| MCU | STM32G431KBUx(UFQFPN32、128 KB flash / 32 KB RAM) |
| モータードライバ | TI DRV8311H(3 × PWM、低サイド電流センス) |
| 角度センサー | AS5600L(I2C、12 bit) |
| CAN トランシーバ | TCAN330(FDCAN1、1 Mbps) |
| モーター | ジンバル 2204-260KV(極対数 7)、減速比 29.16 または 5.4 |
| 制御 | SimpleFOC による電流 FOC(トルク / 速度 / 位置モード) |
| 入力電圧 | LiPo バッテリー(2 セル直列 = 2S で動作確認)。3 セル直列(3S)は、UI で電源電圧を 12 V に設定すれば使える想定(実機では未確認) |

## 構成

```
hqdd1/
├── firmware/   PlatformIO プロジェクト(STM32G431 + SimpleFOC、CAN/UART 通信)
├── webui/      操作用 Web UI(ブラウザ + 任意のバックエンド server.py)
├── hardware/   KiCad プロジェクト(回路図・PCB・3D モデル・製造データ)
└── docs/       通信仕様書(Markdown / PDF)、アクチュエータの仕様サマリー
```

## 主な機能

- **電流 FOC** — トルク(電流)/ 速度 / 位置の 3 モード。モード切替時は目標を自動で安全な値にする(位置モードは現在位置で保持)。
- **CAN(FDCAN1)と UART の両方で制御** — 同じコマンド文字列を使います。CAN は最大 8 台を同一バスに接続でき、UART も `@<ID>` で宛先を指定できます。仕様は [docs/communication_spec.md](docs/communication_spec.md)([PDF](docs/communication_spec.pdf))。
- **パラメータの調整・保存** — PID ゲイン、電流/速度/電圧の上限、電源電圧、減速比を通信経由で変更でき、フラッシュに保存して電源を切っても保持。出荷状態(コード上の値)へ戻すコマンドもあります。
- **Web UI** — リアルタイムグラフ(項目を選んで増やせる)、運転操作、PID・制限値の調整、高速キャプチャ(ステップ応答の記録)。
- **ボタンとLED** — 基板上のボタンでデバイス ID を設定し、LED で ID と負荷を表示。

## はじめに

### 1. ファームウェアを書き込む

必要なもの: [PlatformIO](https://platformio.org/)、ST-Link(基板に ST-Link は載っていないので、J2 の SWDIO/SWCLK に外付けします)。

```bash
cd firmware
pio run                 # ビルド
pio run -t upload       # ST-Link で書き込み
```

> **新しい基板に初めて書き込むときは、先に下の「オプションバイトの修正」を一度だけ実行してください。** しないと、書き込みは成功しても毎回ブートローダに入り、動きません。

**書き込みの動作について。** ビルドごとに固有の番号を保存データに付けているため、**プログラムを書き込み直すと、保存済みのパラメータとデバイス ID はコード上の値に戻ります**。調整した値を残したい場合は、コードの初期値(`firmware/src/main.cpp`)に反映してから書き込んでください。ID の既定値は `firmware/src/device_id.h` の `DEFAULT_DEVICE_ID` です。

### 2. 配線

| 機能 | 内容 |
|---|---|
| 電源 | LiPo バッテリー。2 セル直列(2S、公称 7.4 V)で動作確認。UI の「電源電圧」を 2S = 8 V / 3S(3 セル直列、公称 11.1 V)= 12 V に合わせる。3S は実機未確認で、電源まわりの部品の耐圧は各自で確認すること |
| UART(J3) | USB-TTL アダプタの TXD → 基板 PA15、RXD → 基板 PA2、GND(3.3 V TTL、115200 bps) |
| CAN | CAN_H / CAN_L、バスの両端に 120 Ω の終端(1 Mbps) |
| 書き込み(J2) | ST-Link の SWDIO / SWCLK / GND |

### 3. Web UI を使う

**UART だけなら、サーバ不要でブラウザだけで使えます。** Chrome または Edge で UI を開き、「接続」を押してシリアルポートを選びます(Web Serial を使うため、Firefox / Safari は非対応)。

- 公開ページ: <https://minami88-h.github.io/hqdd1/>(GitHub Pages)
- ローカルで開く場合: `cd webui/static && python3 -m http.server 8099` → <http://localhost:8099>

**CAN を使う場合**は、PC 側のバックエンドが必要です(Linux の SocketCAN を使います)。

```bash
# CAN アダプタを can0 として立ち上げる(例: gs_usb 系)
sudo ip link set can0 up type can bitrate 1000000
# slcan ファームのアダプタの場合: sudo slcand -o -c -s8 /dev/ttyACM0 can0 && sudo ip link set can0 up

pip install aiohttp pyserial
python3 webui/server.py          # -> http://localhost:8080
```

UART もサーバ経由で使えます(UI のヘッダーで「経路」を切り替え)。

## 通信

CAN と UART の仕様(フレーム形式、コマンド一覧、パラメータ表、複数台の運用)は **[docs/communication_spec.md](docs/communication_spec.md)** にまとめています。簡単な例:

```bash
cansend can0 101#4D4331      # ID=1 に "MC1"(速度モード)
cansend can0 101#4D32        # "M2"(2 rad/s)
candump can0                 # 0x201 に 20 ms 周期でテレメトリ
```

```
@1 MC1                       # UART: ID=1 を速度モードに
@1 M2                        # 2 rad/s
@1 ?VP                       # -> #1 GAIN VP 0.03000
```

## ハードウェア

KiCad のプロジェクトは [hardware/](hardware/) にあります(KiCad 10 で保存。4 層基板)。製造用データ(ガーバー、BOM、実装位置)は `hardware/production/` です。詳しくは [hardware/README.md](hardware/README.md)。

### ピン対応

回路図(`hardware/mcu.kicad_sch`)から確定したものです。

| 機能 | MCU ピン | 備考 |
|---|---|---|
| PWM A/B/C → DRV8311 INHA/B/C | PA8 / PA9 / PA10 | TIM1 CH1/2/3 |
| DRV8311 nSLEEP(enable) | PB0 | アクティブ High |
| 電流センス SOA / SOB | PA0 / PA1 | ADC1。GAIN は 3.3 V 直結(2 V/A) |
| AS5600L I2C | SDA = PB7、SCL = PB8 | I2C1(AF4) |
| デバッグ UART(USART2) | TX = PA2、RX = PA15 | J3 |
| CAN(FDCAN1) | TX = PA12、RX = PA11 | TCAN330 経由 |
| SWD | PA13 / PA14 | J2 |
| LED0 / LED1 | PA4 / PA5 | 負荷検出 / ID 表示 |
| SET_ID(ボタン) | PA3 | 2 秒長押しで ID 設定モード |

回路図のネット名 `UART_TX` = PA15、`UART_RX` = PA2 は、外部機器から見た向きの名前です(MCU から見ると PA2 が送信、PA15 が受信)。**配線は回路図のラベル通り**(アダプタの TXD → `UART_TX`、RXD → `UART_RX`)で動作します。

## 重要な注意点

### オプションバイトの修正(チップごとに 1 回だけ)

STM32G431 は、出荷設定(`nSWBOOT0 = 1`)だと BOOT0 を **PB8 ピンの電圧**で判定します。この基板は PB8 を AS5600 の I2C SCL に使い、4.7 kΩ で 3.3 V にプルアップしているため、**リセットのたびに BOOT0 = High と判定され、毎回システムブートローダに入ってしまいます**(書き込み・ベリファイ自体は成功します)。`nSWBOOT0` をクリアして、常にメインフラッシュから起動するようにします。新品のチップ、または基板を新規製作したときに、ST-Link で 1 回だけ実行してください。

```bash
OOCD=~/.platformio/packages/tool-openocd/bin/openocd
SCRIPTS=~/.platformio/packages/tool-openocd/openocd/scripts
$OOCD -s $SCRIPTS -f interface/stlink.cfg -f target/stm32g4x.cfg \
  -c "init" -c "halt" \
  -c "stm32l4x option_write 0 0x20 0x00000000 0x04000000" \
  -c "stm32l4x option_load 0" \
  -c "shutdown"
```

`option_load` でリセットがかかるため "option load failed" と表示されることがありますが、正常です。設定はチップに恒久保存されます。

### デバッグ用シリアルのピン指定

`firmware/platformio.ini` で `PIN_SERIAL_TX=PA2_ALT1` を指定しています。素の `PA2` だと、STM32duino のピンマップ上で LPUART1 の TX に解決され、USART2 の RX(PA15)と食い違って `Serial` が応答しなくなります。

### 電源電圧の設定

基板には電源電圧を測る回路がありません。LiPo バッテリーを替えたら(2 セル = 2S ⇔ 3 セル = 3S)、UI の「電源電圧」を合わせてください(2S = 8 V、3S = 12 V)。ゲインの変更は不要です。合っていないと電流ループの実効ゲインがずれ、発振することがあります。

### 通信が途切れたときの動作

基板は通信断を検出して停止する機能を持ちません。最後に受けた指令のまま動き続けます。上位側で、定期的な送信と停止手順を用意してください。

## 既知の課題

- **TCAN330 の STANDBY(S)/SHDN ピン**が、基板パターンでは未接続です。現在の実機ではジャンパ線で対処しています(基板の修正が必要です)。
- **DRV8311 の AGND/PGND に接続されているネット名が `VDD`** になっています(`power.kicad_sch` の電源レールと同名)。GND であるべき箇所と思われますが、実害がないため未検証です。基板実体の配線を確認してください。
- CAN の受信はメインループ 1 周につき 1 フレームです(約 2 kHz)。連続して送る場合は 1 ms 以上の間隔を空けてください。

## ライセンス

未設定です(`LICENSE` を追加してください)。`firmware/src/can_driver/` は [ACANFD_STM32](https://github.com/pierremolinaro/ACANFD_STM32)(MIT)から取り込んだもので、ライセンスは同ディレクトリの `LICENSE-ACANFD_STM32` を参照してください。ファームウェアは [SimpleFOC](https://simplefoc.com/)(MIT)を使用しています。

## 謝辞

[SimpleFOC](https://simplefoc.com/)、[ACANFD_STM32](https://github.com/pierremolinaro/ACANFD_STM32)
