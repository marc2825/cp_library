---
title: Catalan数
documentation_of: //combinatorics/catalan.hpp
---

# Catalan数

$$
C_n=\binom{2n}{n}-\binom{2n}{n+1}
$$

[カテゴリ索引](README.md) / [操作できる図解](../../visualizations/combinatorics/index.html#catalan) / [実装](../../combinatorics/catalan.hpp) / [参考資料](references.md)

## usage

```cpp
#include <iostream>
#include "combinatorics/catalan.hpp"
int main() {
    cp::FactorialTable table(10);
    for (int n = 0; n <= 5; ++n) std::cout << cp::catalan(table, n) << ' ';
    std::cout << '\n';
    cp::PascalBinomial composite(10, 12);
    std::cout << cp::catalan(composite, 4) << '\n';
}
```

[動く使用例](../../examples/combinatorics/catalan.cpp)。ルートディレクトリをincludeパスへ通します。

## API・使用条件

`catalan(const Table& table, long long n) -> long long`

TableはFactorialTableかPascalBinomial。n<0は0、n>=0なら `2*n<=table.max_n()` が必要です。超過は `out_of_range`。2*nの乗算前に範囲判定するので、n=LLONG_MAXでもあふれません。

C_0=1 mod m。PascalBinomialなら任意の正の法が使え、n+1の逆元がない法でも計算できます。

## 原理・計算量

n個の開き括弧とn個の閉じ括弧の全配置から、途中で閉じ括弧の数が多くなる配置を除きます。最初に違反するまでの部分を反転すると、除く配置と二項係数C(2n,n+1)の配置が対応します。

C(2n,n)/(n+1)の形もありますが、modの下でn+1で割るには条件が必要です。実装は二項係数の差を使います。括弧列・二分木等の数を表します。

二項係数を2回問い合わせる費用。階乗表では一般法O(log m)、m<=2^32でO(1)、Pascal表ではO(1)。前計算は2nまで必要です。

## 使用する問題・検証範囲

[使用する問題](https://cses.fi/problemset/task/2064)：Bracket Sequences I。長さが奇数なら0、偶数ならn/2番目のCatalan数です。

小さい部分集合・順序付きの選択・弱合成・括弧列・集合族を全列挙し、個数を独立に比較します。合成数法・法1・64ビットの法、0・不正な引数・前計算超過・加算前の範囲判定、旧APIも確認します。200万の階乗表・1500行のPascal表・22集合の包除も含みます。

[検証入口](../../verify/run_combinatorics.py)：通常／ASan・UBSan、debug/release、単独・混在include、複数翻訳単位、使用例、Python任意精度整数、6ジャッジ形式・384ケース。オンラインACは未確認です。
