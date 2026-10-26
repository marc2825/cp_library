---
documentation_of: //dp_optimization/frontier_dp.hpp
---

# Frontier DP・連結性の境界状態

[P-01の対応表・検証](../dp_extensions/README.md) / [操作できる図解](../../visualizations/dp_extensions/index.html#frontier_dp) / [ヘッダ](../../dp_optimization/frontier_dp.hpp)

`frontier_connected_degree_count<Count>(n, edges, degree)` は、各頂点の次数が指定値0〜2に一致し、使用頂点が連結な無向辺部分集合を数えます。既定CountはBigInt、modintも使用可能。多重辺は辺IDで区別し、自己ループは拒否します。次数0の頂点は連結条件から除外し、全て0なら空部分集合を1個とします。

`frontier_hamiltonian_path_count(n,edges,s,t)` は全頂点を通るs-tパス、`frontier_hamiltonian_cycle_count(n,edges)` は全頂点を通る閉路です。無向の向き・閉路の開始点は重複計数しません。平行2辺による2頂点閉路は1個。空グラフの閉路は0、n=1かつs=tのパスは空経路1個、それ以外のs=tは0。

各辺の採用/不採用を試し、最後の接続辺を処理した頂点を境界から除きます。次数不一致、途中で消える別成分、完成した成分への追加を除外します。`FrontierConnectivityState` は `degree`、`component`（未使用は-1）、`finished` を持ち、`normalize()` が成分ラベルを初出順0,1,…へ揃えます。同じ正規化状態の個数をmapで合算します。

最大境界幅w、同時状態数S、辺数mなら、比較を含め時間O(m S w (log S+log(w+1)))、領域O(n+m+S w)。Sはwに指数的です。辺順序は入力順で、細長いグリッドは列ごとに並べると幅を抑えられます。n頂点部分集合DPとの置換ではなく、幅が小さいグラフ用です。返値は個数であり、全解の辺集合を保持しません。

用途: 細長い盤面のHamiltonian経路/一周路、次数指定の配線計数。5頂点以下の全グラフ・平行辺・1000頂点の梯子を検証します。

## 使用例

```cpp
#include <iostream>
#include "dp_optimization/frontier_dp.hpp"
int main() {
    std::vector<std::pair<int,int>> edges{{0,1},{1,2},{2,3},{3,0}};
    std::cout << cp::frontier_hamiltonian_path_count(4, edges, 0, 3) << ' '
              << cp::frontier_hamiltonian_cycle_count(4, edges) << '\n';
}
```

出力:

```text
1 1
```

## 検証・参考資料

`make verify-dp-extensions` で単独・重複include、Debug/NDEBUG、使用例、独立した素朴解との比較を実行します。詳細と実行記録は[14項目の検証](../dp_extensions/README.md#検証)、理論・既存実装との関係は[一次資料・設計判断](../dp_extensions/references.md)を参照。オンライン提出ACは未確認です。
