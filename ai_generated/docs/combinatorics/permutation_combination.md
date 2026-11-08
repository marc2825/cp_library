---
title: 順列・組合せ
documentation_of: //combinatorics/permutation_combination.hpp
---

# 順列・組合せ

$$
{}_nP_k=\frac{n!}{(n-k)!},\qquad {}_nC_k=\frac{{}_nP_k}{k!}
$$

[カテゴリ索引](README.md) / [操作できる図解](../../visualizations/combinatorics/index.html#permutation) / [実装](../../combinatorics/permutation_combination.hpp) / [参考資料](references.md)

## usage

```cpp
#include <iostream>
#include "combinatorics/permutation_combination.hpp"
int main() {
    cp::FactorialTable table(10);
    std::cout << cp::permutations(table, 5, 3) << '\n';
    std::cout << cp::combinations(table, 5, 3) << '\n';
}
```

[動く使用例](../../examples/combinatorics/permutation_combination.cpp)。ルートディレクトリをincludeパスへ通します。

## API・使用条件

`permutations(const FactorialTable& table, long long n, long long k) -> long long`

`combinations(const Table& table, long long n, long long k) -> long long`

順列は異なるn個から重複なくk個を選んで並べる数、組合せは同じ選択で順序を無視する数です。combinationsはbinomialと同じAPI条件・結果で、TableはFactorialTableかPascalBinomialです。

n<0・k<0・k>nは0。有効なnが表の上限を超えると `out_of_range`。0個を選ぶ場合も、有効なnは上限内を要求します。

## 原理・計算量

順列では最初の位置にn通り、次にn-1通り、…と選びます。同じk個の集合からはk!種類の並べ方ができるので、順列をk!で割ると組合せになります。

この関数は個数だけを返します。具体的な順列や組合せを列挙する関数ではありません。

順列・階乗表による組合せの問い合わせは一般法O(log m)、m<=2^32でO(1)。Pascal表の組合せはO(1)。前計算は各表の費用です。

## 使用する問題・検証範囲

[使用する問題](https://atcoder.jp/contests/abc172/tasks/abc172_e)：ABC172 E — NEQ。位置の一致を個数ごとに包除し、順列・組合せを使います。

小さい部分集合・順序付きの選択・弱合成・括弧列・集合族を全列挙し、個数を独立に比較します。合成数法・法1・64ビットの法、0・不正な引数・前計算超過・加算前の範囲判定、旧APIも確認します。200万の階乗表・1500行のPascal表・22集合の包除も含みます。

[検証入口](../../verify/run_combinatorics.py)：通常／ASan・UBSan、debug/release、単独・混在include、複数翻訳単位、使用例、Python任意精度整数、6ジャッジ形式・384ケース。オンラインACは未確認です。
