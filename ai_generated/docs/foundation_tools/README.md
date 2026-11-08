# 共通基盤 F-12〜F-17

2026-10-19。基盤型の残り6項目をまとめて整備しました。F-11のBigInt、既存Grid BFSと行列・関数モノイドを共有します。

| ID | API・使用例 | ヘッダ | 図解 |
|---|---|---|---|
| F-12 | [多倍長有理数](../basic_utility/big_rational.md) | [hpp](../../basic_utility/big_rational.hpp) | [操作](../../visualizations/foundation_tools/index.html#big_rational) |
| F-13 | [型付きArena](../basic_utility/arena.md) | [hpp](../../basic_utility/arena.hpp) | [操作](../../visualizations/foundation_tools/index.html#arena) |
| F-14 | [ビット演算・固定popcount列挙](../basic_utility/bit_operations.md) | [hpp](../../basic_utility/bit_operations.hpp) | [操作](../../visualizations/foundation_tools/index.html#bit_operations) |
| F-15 | [グリッド共通処理](../basic_utility/grid.md) | [hpp](../../basic_utility/grid.hpp) | [操作](../../visualizations/foundation_tools/index.html#grid) |
| F-16 | [Rollback配列](../basic_utility/rollback_array.md) | [hpp](../../basic_utility/rollback_array.hpp) | [操作](../../visualizations/foundation_tools/index.html#rollback_array) |
| F-17 | [安定基数ソート](../basic_utility/radix_sort.md) | [hpp](../../basic_utility/radix_sort.hpp) | [操作](../../visualizations/foundation_tools/index.html#radix_sort) |

## 検証

```sh
make verify-foundation-tools
python3 verify/run_foundation_tools.py --sanitize
node verify/visualizations/foundation_tools_model_test.js
node verify/visualizations/foundation_tools_browser_test.mjs --browser /path/to/chromium
```

6新規ヘッダと共通化したGrid BFSの単独・重複include、全カテゴリall.hpp、複数翻訳単位、6使用例を検証します。

- 有理数73,984組、Python Fraction 1,200入力、符号・0・約分・ストリーム・Matrix／関数合成。
- Arenaの128byte整列、構築例外・破棄順、20万要素、401版の永続木の内容。
- 16bit全数のビット位置・個数、n≤16の固定popcount全列挙と64bit境界。
- 全512壁配置×9始点のBFSをFloyd–Warshallと照合。回転・転置・近傍・空と不正入力。
- Rollbackの1万操作を全コピー状態と比較し、失われた枝・所有者・bool・例外を検証。
- 10整数型×150ケースの安定ソート、極値、projection／コピー例外、20万要素。
- Pythonの独立比較はFraction・bit・signed128bit整列の計1,916出力。Debug/Releaseで実行。

オンライン提出・ACは未確認です。図解は6モードのC++参照値とモデルの独立比較を使います。

[残作業](../remaining_work.md) / [一次資料](references.md) / [図の範囲](../../visualizations/foundation_tools/README.md)
