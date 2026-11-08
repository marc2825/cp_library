---
title: GCD・LCM
documentation_of: //number_theory_basic/gcd_lcm.hpp
---

# GCD・LCM

符号を取り除いた大きさにEuclidの互除法を適用し、最大公約数と最小公倍数を求めます。LCMは先にGCDで割り、最後の積が表現できるか確認します。

[カテゴリ索引](README.md) / [操作できる図解](../../visualizations/number_theory_basic/index.html#gcd) / [実装](../../number_theory_basic/gcd_lcm.hpp) / [参考資料](references.md)

## usage

```cpp
#include <iostream>
#include "number_theory_basic/gcd_lcm.hpp"
int main() {
    std::cout << cp::gcd(-18, 24) << '\n';
    if (auto value = cp::lcm(-18, 24)) std::cout << *value << '\n';
}
```

[動く使用例](../../examples/number_theory_basic/gcd_lcm.cpp)。リポジトリのルートをincludeパスへ通します。

## API・使用条件・計算量

`gcd(long long a, long long b) -> unsigned long long`

`lcm(long long a, long long b) -> optional<unsigned long long>`

- 負数と `LLONG_MIN` を含む、すべての `long long` を受け取ります。
- GCDは非負、`gcd(0,0)=0`。LCMはどちらかが0なら0です。
- `|LLONG_MIN|` が符号付き型に入らないため、結果は符号なし型です。符号付き型へ戻すときは範囲を確認してください。
- LCMが `ULLONG_MAX` を超える場合だけ `nullopt`。0と失敗を区別できます。
- 引数は `long long` です。`LLONG_MAX` を超える符号なし入力を渡すAPIではありません。

単語長を固定した演算としてO(log(max(|a|,|b|)+1))時間、O(1)空間。`std::gcd` は絶対値が共通型で表現できることを要求するため、このAPIでは符号なしの大きさを明示的に扱います。

## 使用する問題・検証範囲

[使用する問題](https://judge.u-aizu.ac.jp/onlinejudge/description.jsp?id=ALDS1_1_B)：GCDはAOJ ALDS1_1_B、LCMはAOJ NTL_1_C。

40,401個の符号付き小整数対を約数走査と比較し、整数極値、LCMの符号付き範囲超えと符号なし範囲超え、Python任意精度整数で照合します。

[検証入口](../../verify/run_number_theory_basic.py)は通常／ASan・UBSan、debug/release、単独include・混在include・複数翻訳単位、使用例、独立オラクル、ジャッジ形式の入出力を確認します。オンラインACは未確認です。
