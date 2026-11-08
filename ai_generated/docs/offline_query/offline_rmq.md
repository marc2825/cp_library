---
title: Offline RMQ
documentation_of: //offline_query/offline_rmq.hpp
---

# Offline RMQ

静的な全区間最小値問い合わせを右端順にまとめて処理。

## API・使用条件

offline_rmq(vector<T>,vector<pair<int,int>>,Compare=less<T>) -> vector<int>。同値なら最左argmin、空区間は-1。

0<=l<=r<=N。greaterで最左argmax。値そのものではなく元配列の添字を返す。空入力でも空区間を照会可。

C++17、namespace cp。特記以外は0始まり・半開区間。前提条件は呼出し側が守り、添字/範囲の違反はdebug assertで検査します。整数の全中間値とコンテナの添字が使用型に収まることが必要です。計算量は要素の演算/比較/コピーをO(1)としたものです。

## Usage

```cpp
#include <iostream>
#include "offline_query/offline_rmq.hpp"
int main() {
    auto a=cp::offline_rmq(std::vector<int>{4,2,2,7},{{0,4},{2,4},{1,1}});
    for(int x:a) { std::cout<<x<<" "; } std::cout<<"\n";
}
```

出力: `1 2 -1`。

## 計算量

保守的にO((N+Q) log(N+1))時間、O(N+Q)領域。単調スタックは各位置1回、親リンクに経路圧縮を使う。


## 問題例・検証・参考資料

用途に対応する[問題例と一次資料](../a_priority/references.md)を参照。`python3 verify/run_a_priority.py`（`--sanitize`対応）で単独/重複include、複数翻訳単位、上記使用例と独立した素朴解の差分検証を行います。[検証範囲](../a_priority/README.md)。オンラインACは未確認です。

[操作できる図解](../../visualizations/a_priority/index.html#offline_rmq) / [図の範囲](../../visualizations/a_priority/README.md) / [カテゴリ索引](README.md) / [動く例](../../examples/offline_query/offline_rmq.cpp)
