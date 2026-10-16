---
title: 重複組合せ
documentation_of: //combinatorics/multiset_combination.hpp
---

# 重複組合せ

$$
{}_nH_k=\binom{n+k-1}{k}
$$

[カテゴリ索引](README.md) / [操作できる図解](../../visualizations/combinatorics/index.html#multiset) / [実装](../../combinatorics/multiset_combination.hpp) / [参考資料](references.md)

## usage

```cpp
#include <iostream>
#include "combinatorics/multiset_combination.hpp"
int main() {
    cp::FactorialTable table(10);
    std::cout << cp::multiset_combination(table, 3, 4) << '\n';
    std::cout << cp::multiset_combination(table, 0, 0) << ' '
              << cp::multiset_combination(table, 0, 4) << '\n';
}
```

[動く使用例](../../examples/combinatorics/multiset_combination.cpp)。ルートディレクトリをincludeパスへ通します。

## API・使用条件

`multiset_combination(const Table& table, long long n, long long k) -> long long`

n種類から、各種類を何回でも使ってk個選ぶ数です。TableはFactorialTableかPascalBinomial。

- n<0・k<0は0。
- k=0なら1 mod m。n=0も含み、この場合は前計算範囲を要求しません。
- n=0,k>0なら0。
- n>0,k>0ならn+k-1まで前計算が必要。超過は `out_of_range`。

n+k-1を計算する前に上限を比較するので、LLONG_MAXの加算もあふれません。

## 原理・計算量

種類ごとの個数x_1,…,x_nを並べると、非負整数の和 `x_1+…+x_n=k` を満たす解の個数です。k個の星とn-1本の仕切りを並べる配置と一対一に対応します。

種類には区別があり、同じ種類の品物には区別がありません。各種類に個数上限がある問題は、そのままではこの式を使えません。

二項係数を1回問い合わせる費用。階乗表では一般法O(log m)、m<=2^32でO(1)、Pascal表ではO(1)。k=0やn=0の処理はO(1)。

## 使用する問題・検証範囲

[使用する問題](https://cses.fi/problemset/task/1716)：Distributing Apples。子供を種類、配るリンゴの個数をkとみなします。

小さい部分集合・順序付きの選択・弱合成・括弧列・集合族を全列挙し、個数を独立に比較します。合成数法・法1・64ビットの法、0・不正な引数・前計算超過・加算前の範囲判定、旧APIも確認します。200万の階乗表・1500行のPascal表・22集合の包除も含みます。

[検証入口](../../verify/run_combinatorics.py)：通常／ASan・UBSan、debug/release、単独・混在include、複数翻訳単位、使用例、Python任意精度整数、6ジャッジ形式・384ケース。オンラインACは未確認です。
