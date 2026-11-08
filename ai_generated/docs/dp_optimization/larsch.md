---
title: LARSCH
documentation_of: //dp_optimization/larsch.hpp
---

# LARSCH

[カテゴリ索引](README.md) / [B項目の対応表](../b_12_15/README.md) / [操作できる図解](../../visualizations/b_12_15/index.html#larsch) / [実装](../../dp_optimization/larsch.hpp) / [一次資料と設計](../b_12_15/references.md)

下三角の全単調行列から最左の行最小位置を、行番号順にオンラインで取得します。行縮約と列縮約を再帰的に交互適用します。

## usage

```cpp
#include <iostream>
#include "dp_optimization/larsch.hpp"
int main() {
    using E=std::pair<bool,long long>;
    cp::Larsch<E> s(3,[](int i,int j)->E{if(j>i) return {true,0}; return {false,1LL*(i-j)*(i-j)};});
    for(int i=0;i<3;++i) { if(i) std::cout << ' '; std::cout << s.next_argmin(); }
    std::cout << '\n';
}
```

出力:

```text
0 1 2
```

## API・使用条件

`Larsch<T>(n,std::function<T(int,int)> f)`、`next_argmin(),reset(),size(),processed_rows()`。行iの候補は列0..i。f(i,j)のj>iは全有限値より大きくし、f(i,j+1)は行jの答えを取得した後に評価可能であること。Tは<が必要。

## 仕組み・境界条件

noshi91のCC0版を移植し、0行・reset・行数超過のout_of_rangeを追加しました。fは未来の行を評価することがありますが、参照する列の依存は上記の順序を守ります。上三角の扱いを含むtotally monotoneの前提を利用者が満たしてください。単なる行argminの単調性だけでは不足です。

## 計算量

全N行でO(N)回のf評価、時間・領域O(N)。縮約の深さO(log N)。 標準整数の演算をO(1)として数えています。64bitの汎用mod乗算は最大O(log m)の加算を使うため、それを使う処理は別途その係数が掛かります。

## 使用する問題・検証

先に確定したdp[j]を含むMonge DP。 [動く小問題](../../examples/dp_optimization/larsch_b.cpp)。

`python3 verify/run_b_12_15.py` / `--sanitize`。[独立した比較方法](../b_12_15/README.md#ローカル検証)に従い、全列挙・定義からの直接計算・Python任意精度と比較します。使用例は文書中のコードと一致を検査してコンパイル・実行します。オンライン提出・ACは未確認です。図解の小入力化と代替計算は[図解の範囲](../../visualizations/b_12_15/README.md)に明記しています。
