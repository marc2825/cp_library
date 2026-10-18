# 共通基盤 F-01〜F-07 の図解

[index.html](index.html)を直接開き、モードとJSON入力を選んで実行します。先頭・戻る・次へ・最後・再生で計算過程を追えます。整数はJavaScript Numberへ変換せず、文字列とBigIntで扱います。

| モード | 観察できること |
|---|---|
| static_modint | 負数の正規化・和・積・逆元の有無 |
| dynamic_modint | 構築時の世代・法変更・古い値の使用拒否 |
| barrett_reduction | ceil逆数・高位積・商の過大評価と補正 |
| montgomery_modint | 表現への変換・積・下位bit消去・通常表現への復元 |
| modint61 | 61bitでの分割・折り畳み・法の減算 |
| fast_io | バッファ境界・数値の蓄積・トークン確定・エラー・EOF |
| safe_integer | 型の範囲・数学的な和と積・検査／飽和・floor/ceil |

## 図の範囲

演算原理のモデルで、C++をブラウザ内で実行するものではありません。世代は単一IDの2回の設定を表示し、set_modを省略した既定法は表示しません。modintの共通APIすべての操作を列挙する画面ではありません。MontgomeryはBigIntでRの逆元を求めて最後のdecodeを示し、C++のNewton初期化ループは省略します。

FastIOは整数入力のモデルで、128bitまで、ASCII120文字・バッファ1〜16バイトに表示を制限します。C++のFILEエラー・浮動小数点・出力バッファはC++テストの対象です。安全整数は和・積・丸めの表示で、減算・番兵・上限判定は個別説明とC++テストを参照します。図ではBigIntで数学的な結果を計算し、C++ではbuiltinを用いてオーバーフローを避けます。

## 検証

```sh
node verify/visualizations/foundation_model_test.js
node verify/visualizations/foundation_browser_test.mjs --browser /path/to/chromium
```

7サンプルを`verify/foundation/visualization_expected.json`に記録し、C++ bridgeとJavaScriptの両方で照合します。モデルの追加6,000比較、入力拒否・丸め・バッファ境界も確認。ブラウザでは全7モードの入力編集・再生・移動・エラー・リンクと390px幅を検証します。`--screenshot /tmp/foundation.png`でdesktop/mobileのPNGを保存できます。

[API・使用例・検証](../../docs/basic_utility/foundation.md) / [共通図解一覧](../index.html)
