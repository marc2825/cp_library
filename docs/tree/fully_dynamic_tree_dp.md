---
title: Fully Dynamic Tree DP
documentation_of: //tree/fully_dynamic_tree_dp.hpp
---

# Fully Dynamic Tree DP（T-23）

`FullyDynamicTreeDP<Spec>` は [Dynamic Top Tree](dynamic_top_tree.md) を共有し、`query(root)` で根付き木DPの最終結果を返します。辺のlink/cutと頂点値の変更を両方扱い、問い合わせごとの木全体の再計算はしません。

## API・条件

通常の `Info/Path/Point` と5つの結合関数に加え、`Spec::finish(Path) -> Result` を定義します。

- `query(root) -> Result`：その成分をrootで根付けた答え。
- `subtree_query(root,v) -> optional<Result>`：同じ根でv以下だけを解く。非連結はnullopt。
- `link/cut/connected/make_root/set_vertex/get_vertex` とcluster取得APIは共通。

Specはpreferred-path分解に依存しないDPを表すこと。任意の木DPを自動的に高速化するものではなく、定数サイズのrake/compress情報へまとめられるものが対象です。構築O(n)、各操作ならしO(log n)、空間O(n)に、Specの演算・コピー費用を掛けます。

## 付属のDP

`FullyDynamicIndependentSet<T>` は `IndependentSetTreeDP<T>` の別名です。頂点のInfoは重みw、答えは独立集合Iについての `Σ ∏ w[v]`。全重みを1にすれば独立集合数です。Tは可換半環（0,1,+,*）で、逆元は使いません。零や合成数modでも動作します。

子の白/黒状態をW/Bとすると、点clusterは `any=∏(W+B), white=∏W`。頂点を加えると2×2行列 `[[any,any],[w*white,0]]` になり、spine上では上から行列積を取ります。下端の外側には `[1,0]` を入れるので、上端のW/Bは積の第0列です。rakeは点clusterの成分ごとの積です。EDPC Pの木DPを、辺を差し替えても更新できる形にした例です。

`FullyDynamicDistanceSum<T>` は `DistanceSumTreeDP<T>` の別名です。単位辺長に対して `Σ_v w[v]*dist(root,v)` を返します。根ごとに答えが変わるDPの例です。Pathはspineの頂点数、総重み、上端/下端からの距離和を持ち、compressで他方の総重み×spine長を加えます。辺コスト付きの距離和ではありません。

組み込み整数では途中の和・積が型内に収まること。個数が大きい場合はmodintやBigIntを選べます。

## 検証

50個の9頂点の森で合計15000段階の辺追加・削除・頂点更新・根変更を行い、独立集合は全512部分集合、距離和は独立BFSと照合。mod 12と重み0を含め、逆元に依存しないことを検査します。文字列の方向付きcompressも別のSpecで検査しています。オンラインACは未確認です。

## 使用例

```cpp
#include <iostream>
#include "../../tree/fully_dynamic_tree_dp.hpp"
#include "../../basic_utility/static_modint.hpp"
int main() {
    using Mint = cp::StaticModInt<998244353>;
    cp::FullyDynamicIndependentSet<Mint> dp({1, 1, 1, 1});
    dp.link(0, 1); dp.link(1, 2); dp.link(1, 3);
    std::cout << dp.query(0).val() << '\n';
    dp.cut(1, 3); dp.link(2, 3);
    std::cout << dp.query(3).val() << '\n';
    dp.set_vertex(1, 0);
    std::cout << dp.query(0).val() << '\n';
}
```

出力:

```text
9
8
6
```

[操作できる図解](../../visualizations/tree_extensions/index.html#fully_dynamic_tree_dp) / [検証範囲・コマンド](../tree_extensions/README.md) / [一次資料と設計判断](../tree_extensions/references.md) / [木カテゴリ](README.md)
