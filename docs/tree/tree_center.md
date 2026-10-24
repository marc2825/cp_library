---
title: 木の中心
documentation_of: //tree/tree_center.hpp
---

# 木の中心（T-01）

最も遠い頂点までの距離（離心度）が最小の頂点を求めます。重心分解で使う「頂点数を均等に分ける重心」とは異なります。直径の両端からの距離の最大値が離心度になることを使います。

## API・条件

- `tree_center(Graph<T>) -> TreeCenterResult`：重みを無視した中心（1個または2個の頂点、昇順）と整数の半径 `radius`。
- `weighted_tree_center(Graph<T>) -> WeightedTreeCenterResult<T>`：非負・有限の重み。`vertices` は頂点に限定した中心全件、`vertex_radius` はその離心度。零重みでは中心が3頂点以上になることもあります。
- 連続な距離空間の中心は直径の中点です。`vertex >= 0` ならその頂点。辺内部なら `edge_id` は元の辺番号、`from -> to` は直径上の向き、`offset_twice / 2` が `from` からの距離です。半径は `radius_twice / 2`。整数の割り算を先に行わず、2倍表現のまま使えます。零重みで複数の表現が同じ点を指すときは代表1個を返します。
- 空の木は頂点一覧が空、半径0、位置のIDはすべて-1。非空では無向の連結な木が必要で、森・閉路・多重辺・自己ループ・有向グラフは `invalid_argument`。負辺・NaN・無限大も拒否します。
- 重みの型は通常の組み込み整数・浮動小数点です。全距離は `T` の最大値未満。既存の `RootedTree` と同じ加算検査を共有し、超過は `overflow_error`。2倍表現を求める際に `2 * diameter` は計算しません。
- 浮動小数点の中心判定は厳密な `==` です。丸め誤差による同点差が必要なら整数へスケールしてください。整数の半整数中心は厳密です。

## 計算量・用途

時間・追加空間ともに O(n)。重みなしは既存の直径を共有し、重み付きは直径両端の距離を求めます。最悪の通信距離を最小化する施設配置、木の最小離心度、直径中点の位置の復元に使います。

## 検証

840個の木について全頂点から独立に距離を計算し、中心全件・離心度・連続中心からの最大距離を照合。零重み、巨大整数の半整数位置、小数、負辺・NaN、空、森・閉路、20万頂点の鎖を検査しています。

## 使用例

```cpp
#include <iostream>
#include "../../tree/tree_center.hpp"
int main() {
    cp::Graph<long long> g(3);
    g.add_edge(0, 1, 2); g.add_edge(1, 2, 5);
    auto c = cp::tree_center(g);
    std::cout << c.vertices[0] << '\n';
    auto w = cp::weighted_tree_center(g);
    std::cout << w.vertices[0] << ' ' << w.vertex_radius << '\n';
    std::cout << w.edge_id << ' ' << w.offset_twice << ' ' << w.radius_twice << '\n';
}
```

出力:

```text
1
1 5
1 7 7
```

[操作できる図解](../../visualizations/tree_extensions/index.html#tree_center) / [検証範囲・コマンド](../tree_extensions/README.md) / [一次資料と設計判断](../tree_extensions/references.md) / [木カテゴリ](README.md)
