---
title: 二次元BIT（座圧版）
documentation_of: //offline_query/compressed_fenwick_tree_2d.hpp
---

# 二次元BIT（座圧版）

更新点だけ登録する疎な二次元BIT。登録後の点加算と長方形和はオンライン。

## API・使用条件

CompressedFenwickTree2D<T,Coord=long long>(vector<pair<Coord,Coord>>)、add(x,y,w)、sum(x,y)、sum(l,d,r,u)。

同一点の重複登録可、初期重み0。更新は登録点だけ、違反invalid_argumentで変更なし。閾値は未登録可、sum(x,y)は両座標が厳密に未満。空/逆長方形0。Tは加法可換群、Coordは<だけでよい。

C++17、namespace cp。特記以外は0始まり・半開区間。前提条件は呼出し側が守り、添字/範囲の違反はdebug assertで検査します。整数の全中間値とコンテナの添字が使用型に収まることが必要です。計算量は要素の演算/比較/コピーをO(1)としたものです。

## Usage

```cpp
#include <iostream>
#include "offline_query/compressed_fenwick_tree_2d.hpp"
int main() {
    cp::CompressedFenwickTree2D<long long> s({{1,2},{100,200}}); s.add(1,2,7);
    std::cout<<s.sum(0,0,50,50)<<"\n";
}
```

出力: `7`。

## 計算量

P登録点の構築O(P log²(P+1))、領域O(P log(P+1))、操作O(log²(P+1))。


## 問題例・検証・参考資料

用途に対応する[問題例と一次資料](../a_priority/references.md)を参照。`python3 verify/run_a_priority.py`（`--sanitize`対応）で単独/重複include、複数翻訳単位、上記使用例と独立した素朴解の差分検証を行います。[検証範囲](../a_priority/README.md)。オンラインACは未確認です。

[操作できる図解](../../visualizations/a_priority/index.html#compressed_fenwick_tree_2d) / [図の範囲](../../visualizations/a_priority/README.md) / [カテゴリ索引](README.md) / [動く例](../../examples/offline_query/compressed_fenwick_tree_2d.cpp)
