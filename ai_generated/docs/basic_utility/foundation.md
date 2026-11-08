# F-01〜F-07 共通基盤の整備

配置は3.1の`basic_utility/`、共通入口は`basic_utility/all.hpp`です。C++17、GCC/Clangの128bit整数・検査付き演算builtinを使用します。

| ID | 機能・API | 実装 | 操作 |
|---|---|---|---|
| F-01 | [固定法modint](static_modint.md) | [hpp](../../basic_utility/static_modint.hpp) | [図解](../../visualizations/foundation/index.html#static_modint) |
| F-02 | [実行時法modint](dynamic_modint.md) | [hpp](../../basic_utility/dynamic_modint.hpp) | [図解](../../visualizations/foundation/index.html#dynamic_modint) |
| F-03 | [Barrett Reduction](barrett_reduction.md) | [hpp](../../basic_utility/barrett_reduction.hpp) | [図解](../../visualizations/foundation/index.html#barrett_reduction) |
| F-04 | [Montgomery Reduction・64bit modint](montgomery_modint.md) | [hpp](../../basic_utility/montgomery_modint.hpp) | [図解](../../visualizations/foundation/index.html#montgomery_modint) |
| F-05 | [2^61−1 modint](modint61.md) | [hpp](../../basic_utility/modint61.hpp) | [図解](../../visualizations/foundation/index.html#modint61) |
| F-06 | [共通FastIO](fast_io.md) | [hpp](../../basic_utility/fast_io.hpp) | [図解](../../visualizations/foundation/index.html#fast_io) |
| F-07 | [安全な整数演算](safe_integer.md) | [hpp](../../basic_utility/safe_integer.hpp) | [図解](../../visualizations/foundation/index.html#safe_integer) |

## 選び方と対応範囲

法が固定で32bitならStaticModInt、実行時法ならDynamicModIntを使います。DynamicModIntは複数IDに対応し、法変更前の値を世代で検出します。BarrettReductionはuint32法のuint64剰余を単独でも使えます。Montgomeryは奇数法1以上2^63未満に限定し、実行時reducerと固定法modintを提供します。ModInt61は2^61−1専用で、部分文字列ハッシュへの接続例があります。

FastIOは非所有のFILE*とインスタンスごとのバッファを使います。旧FastIO2種は保持し、新しい共通入口への移行方法を説明しています。安全な整数演算は検査付き・飽和・上限判定・非負番兵加算・符号付きfloor/ceil除算を提供します。既存parser内Fpや検証用ModInt、カテゴリ内detailの一括置換は行いません。

## ローカル検証

```sh
make verify-foundation
python3 verify/run_foundation.py --sanitize
node verify/visualizations/foundation_model_test.js
node verify/visualizations/foundation_browser_test.mjs --browser /path/to/chromium
```

- 公開7ヘッダ・内部2ヘッダの単独／重複include、Debug/Release、全カテゴリall.hppとの混在・複数翻訳単位を検証します。
- 7個の説明内コードとexamplesを一致させ、出力も検証します。
- 固定法・動的法・Montgomery・2^61−1、法1・偶数／合成数・32/63bit境界、逆元なし、128bitからの正規化、巨大10進入力、uint64指数を確認します。
- reducerの10万組を広い整数の剰余と比較。符号付き／なし8bitの加減乗算全131,072組を全探索し、floor/ceilと番兵・上限判定も比較します。
- Python任意精度整数による12,196入力の差分比較をDebug/Releaseのそれぞれで実施します。任意のuint32法、REDC入力、64/128bitの検査・飽和・丸めを含みます。
- FastIOは1バイトのバッファ、20万文字のトークン／空白、極小／極大値、末尾改行なし、浮動小数点往復、不正入力、EOF、Linuxの/dev/full・write-only streamによるI/Oエラーを確認します。
- A+B、AOJ NTL_1_Bの2ジャッジ形式を各32入力、Debug/Releaseでローカル比較します。**オンライン提出・ACは未確認です。**
- 図解7モードはC++の参照出力と一致させ、モデル6,000比較、ブラウザの入力編集・履歴移動・再生・例外表示・390px幅を検証します。

[カテゴリ索引](README.md) / [一次資料と設計判断](references.md) / [図解の範囲](../../visualizations/foundation/README.md)
