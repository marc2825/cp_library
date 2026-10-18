---
title: Incremental Bridge Connectivity
documentation_of: //union_find/incremental_bridge_connectivity.hpp
---

# Incremental Bridge Connectivity

橋を辺とする縮約森を維持します。異なる木を結ぶ辺は橋を一つ増やし、同じ木に閉路を作る辺は経路上の橋を縮約します。

## 使用例

```cpp
#include <iostream>
#include "union_find/incremental_bridge_connectivity.hpp"
int main() {
    cp::IncrementalBridgeConnectivity d(3); d.add_edge(0,1); d.add_edge(1,2); d.add_edge(2,0);
    std::cout << d.bridge_count() << " " << d.two_edge_connected(0,2) << "\n";
}
```

出力: `0 1`。

## APIと使用条件

`IncrementalBridgeConnectivity(n)`、add_edge(u,v)、bridge_count()、same(u,v)、two_edge_connected(u,v)。

無向の辺追加だけ。自己ループ、多重辺可。個々の橋の辺ID一覧は保持せず、橋数・通常連結性・二重辺連結性を返します。探索と圧縮は反復で行い長い鎖の再帰を避けます。

## 計算量

N頂点M追加に対してO((N+M) log(N+1))の償却上界、O(N)領域。 演算・比較・値のコピーが O(1) の場合です。文字列などを格納すると、その処理時間・領域が加わります。

## 使用する問題と検証

区間同士の同一視、オンライン辺追加、頂点群への一括加算、辺の削除を含む連結性などで使います。成分ラベルの直接更新、全辺を除いたBFSと比較します。

`python3 verify/run_b_1_6.py`（`--sanitize` 対応）で単独／重複include、debug/releaseの比較検証、上の使用例を確認します。検証範囲の詳細は[追加分の一覧](../b_1_6/README.md)。オンライン提出・ACは未確認です。

[操作できる図解](../../visualizations/b_1_6/index.html#incremental_bridge_connectivity) / [カテゴリ索引](README.md) / [参考資料と設計判断](../b_1_6/references.md#dsu) / [図解の簡略化](../../visualizations/b_1_6/README.md)
