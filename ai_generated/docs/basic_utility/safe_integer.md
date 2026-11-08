---
title: 安全な整数演算
documentation_of: //basic_utility/safe_integer.hpp
---

# 安全な整数演算

数学的な結果が型の範囲に収まるかを確認し、オーバーフローによる未定義動作を避ける共通APIです。カテゴリ内の既存detailを一括置換せず、利用者が明示的に選べる公開部品を追加しました。

## 使用例

```cpp
#include <iostream>
#include <limits>
#include "basic_utility/safe_integer.hpp"
int main() {
    const auto high = std::numeric_limits<long long>::max();
    std::cout << cp::checked_add(high, 1LL).has_value() << " " << cp::saturating_add(high, 1LL) << "\n";
    std::cout << cp::floor_div(-7, 3) << " " << cp::ceil_div(-7, 3) << "\n";
    std::cout << cp::infinity_add(7, 8, 10) << " " << cp::multiply_leq(7, 8, 50) << "\n";
}
```

出力:

```text
0 9223372036854775807
-3 -2
10 0
```

## APIと使用条件

すべて`namespace cp`。引数は同じ整数型Tで、bool以外の組み込み型（符号付き／なし128bit含む）。混合型の場合は呼び出し側で型を揃えます。GCC/Clangの検査付き演算builtinを使います。

| API | 結果・条件 |
|---|---|
| `checked_add/sub/mul(T a,T b)` | 結果が表現可能なら`optional<T>`、それ以外は`nullopt` |
| `saturating_add/sub/mul(a,b)` | 数学的な結果を`[lowest,max]`へ飽和。unsignedの減算underflowは0 |
| `add_leq(a,b,limit)` / `multiply_leq(a,b,limit)` | 非負入力で数学的な和／積がlimit以下か。積・和を溢れさせずに判定 |
| `infinity_add(a,b,infinity)` | `0<=a,b<=infinity`で`min(a+b,infinity)`。infinityを吸収する番兵として扱う |
| `floor_div(a,b)` / `ceil_div(a,b)` | 数学的な商の床／天井。負の分母も可 |

非負条件や番兵の範囲を満たさない入力は`invalid_argument`。割り算の0除算も`invalid_argument`、符号付き極小値/-1は`overflow_error`です。**番兵加算は非負の費用用**で、負の辺を含む最短路用の意味は付けません。`multiply_leq(a,0,0)`はaが巨大でもtrue。`ceil_div`は`a+b-1`を作らず、商と余りから補正します。全操作O(1)時間・O(1)領域（固定幅整数の演算コストを単位時間とする）。

## 使用する問題と検証

整数の上限判定、非負距離・DPの無限大番兵、負数を含む区間端点のfloor/ceil。符号付き／なし8bitの全組合せと、64/128bitの極値・乱数をPython任意精度で照合します。

`python3 verify/run_foundation.py` / `--sanitize`（`make verify-foundation`）で単独・重複include、全カテゴリとの混在、複数翻訳単位、Debug/Release、使用例、独立した比較を検証します。オンライン提出・ACは未確認です。[検証範囲](foundation.md)。

[操作できる図解](../../visualizations/foundation/index.html#safe_integer) / [図解の範囲](../../visualizations/foundation/README.md) / [カテゴリ索引](README.md) / [一次資料・設計判断](references.md)
