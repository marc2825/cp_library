---
title: Monge Shortest Path
documentation_of: //dp_optimization/monge_shortest_path.hpp
---

# Monge Shortest Path

[カテゴリ索引](README.md) / [B項目の対応表](../b_12_15/README.md) / [操作できる図解](../../visualizations/b_12_15/index.html#monge_shortest_path) / [実装](../../dp_optimization/monge_shortest_path.hpp) / [一次資料と設計](../b_12_15/references.md)

前向き完全DAGのDPを、オンライン最小位置探索のLARSCHで解きます。距離と直前頂点、経路を復元します。

## usage

```cpp
#include <iostream>
#include "dp_optimization/monge_shortest_path.hpp"
int main() {
    auto r=cp::monge_shortest_path(4,[](int i,int j){return 1LL*(j-i)*(j-i);});
    std::cout << r.distance[4] << ' ' << r.path_to(4).size() << '\n';
}
```

出力:

```text
4 5
```

## API・使用条件

`monge_shortest_path(n,cost(i,j))`。頂点0..n、辺は全i<j、全費用は有限long long。結果distance,parent,path_to(v)。n>=0、始点0。

## 仕組み・境界条件

Monge条件cost(i,j)+cost(k,l)<=cost(i,l)+cost(k,j)（i<k<j<l）を前提にします。dp[i]を列へ加えても性質を保つため、先のdpを参照せず行を確定できます。負辺可。有限値と無効な上三角をpairで区別し、LLONG_MAXを番兵にしません。距離の加算超過はoverflow_error。

## 計算量

O(N)回のcost評価、O(N)時間・領域。Nは終点番号。 標準整数の演算をO(1)として数えています。64bitの汎用mod乗算は最大O(log m)の加算を使うため、それを使う処理は別途その係数が掛かります。

## 使用する問題・検証

二乗区間費用による任意個数の分割や、Monge辺費用の最短路。 [動く小問題](../../examples/dp_optimization/monge_shortest_path_b.cpp)。

`python3 verify/run_b_12_15.py` / `--sanitize`。[独立した比較方法](../b_12_15/README.md#ローカル検証)に従い、全列挙・定義からの直接計算・Python任意精度と比較します。使用例は文書中のコードと一致を検査してコンパイル・実行します。オンライン提出・ACは未確認です。図解の小入力化と代替計算は[図解の範囲](../../visualizations/b_12_15/README.md)に明記しています。
