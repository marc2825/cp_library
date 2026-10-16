---
title: 約数列挙
documentation_of: //number_theory_basic/divisors.hpp
---

# 約数列挙

dとn/dを対にして正の約数を集めます。小さい約数の列と大きい約数の逆順の列を連結するため、ソートせずに昇順へ揃えます。

[カテゴリ索引](README.md) / [操作できる図解](../../visualizations/number_theory_basic/index.html#divisors) / [実装](../../number_theory_basic/divisors.hpp) / [参考資料](references.md)

## usage

```cpp
#include <iostream>
#include "number_theory_basic/divisors.hpp"
int main() {
    for (long long d : cp::divisors(36)) std::cout << d << ' ';
    std::cout << '\n';
}
```

[動く使用例](../../examples/number_theory_basic/divisors.cpp)。リポジトリのルートをincludeパスへ通します。

## API・使用条件・計算量

`divisors(long long n) -> vector<long long>`

- `n>=1`。0・負数は `invalid_argument`。
- 結果は正の約数だけを重複なく昇順に並べた列。1の結果は `{1}`。
- 完全平方数では中央の約数を一度だけ追加します。負の約数は含みません。
- `d<=n/d` により平方のオーバーフローと浮動小数点の丸めを避けます。

O(√n+τ(n))時間、O(τ(n))空間。τ(n)は約数の個数です。64ビットの大きなnでは試し割りの走査が長くなります。

## 使用する問題・検証範囲

[使用する問題](https://judge.u-aizu.ac.jp/onlinejudge/description.jsp?id=ITP1_3_D)：AOJ ITP1_3_D — How Many Divisors?。

2〜10,000を全dの走査と照合し、1、平方数、昇順・重複なし、10^12の169個の約数、0と負数の拒否を確認します。

[検証入口](../../verify/run_number_theory_basic.py)は通常／ASan・UBSan、debug/release、単独include・混在include・複数翻訳単位、使用例、独立オラクル、ジャッジ形式の入出力を確認します。オンラインACは未確認です。
