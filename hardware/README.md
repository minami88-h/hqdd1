# hqdd_1 基板(KiCad)

STM32G431KBUx + DRV8311H + AS5600L + TCAN330 の制御基板です。KiCad 10 で保存されています(4 層基板)。古い版では開けません。

## ファイル

| ファイル | 内容 |
|---|---|
| `hqdd_1.kicad_pro` | プロジェクト |
| `hqdd_1.kicad_sch` | ルートの回路図(`mcu` と `power` の 2 シートを持つ) |
| `mcu.kicad_sch` | MCU、DRV8311H、AS5600L、TCAN330、コネクタ類 |
| `power.kicad_sch` | 電源まわり |
| `hqdd_1.kicad_pcb` | PCB |
| `hqdd_1.step` | 3D モデル(PCB から出力) |
| `production/` | 製造用データ(下記) |
| `fabrication-toolkit-options.json` | Fabrication Toolkit(KiCad プラグイン)の出力設定 |

### `production/`

Fabrication Toolkit で出力した発注用データです。

| ファイル | 内容 |
|---|---|
| `hqdd_1.zip` | ガーバーとドリルデータ |
| `bom.csv` | 部品表 |
| `positions.csv` | 部品の実装位置 |
| `designators.csv` | リファレンス一覧 |
| `netlist.ipc` | IPC ネットリスト |

## 既知の課題

- TCAN330 の STANDBY(S)/SHDN ピンが基板パターンで未接続です(ジャンパ線で対処)。
- DRV8311 の AGND/PGND に接続されているネット名が `VDD` になっています。GND であるべき箇所と思われますが、未検証です。

回路の詳細、ピン対応、通信は、ルートの [README](../README.md) を参照してください。
