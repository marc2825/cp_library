---
title: Rank-Maximal Matching
documentation_of: //graph_flow/rank_maximal_matching.hpp
---

# Rank-Maximal Matching

[カテゴリ索引](README.md) / [G残り15項目](../graph_advanced/README.md) / [操作できる図解](../../visualizations/graph_advanced/index.html#rank_maximal_matching) / [実装](../../graph_flow/rank_maximal_matching.hpp) / [一次資料・設計判断](../graph_advanced/references.md)

二部グラフで、第1順位の辺数、第2順位の辺数、…を辞書式最大化します。

## usage

```cpp
#include <iostream>
#include "graph_flow/rank_maximal_matching.hpp"
int main() {
    auto r=cp::rank_maximal_matching(2,2,{{0,0,1},{0,1,2},{1,0,2}});
    std::cout << r.profile[0] << ' ' << r.profile[1] << '\n';
}
```

出力:

```text
1 0
```

## API・使用条件

`rank_maximal_matching(left,right,vector<RankedBipartiteEdge>)`。辺は `{left,right,rank}`。結果の `left_match/right_match` は未対応−1、`edge_ids`は元入力順ID、`ranks`は存在する順位の昇順、`profile`がその順位ごとの本数。

rankは正int、小さいほど優先。欠けた順位は圧縮し、同順位を許す。最大本数を先に求める目的ではありません。平行候補は最小rank（同値なら先頭）を保持。サイズ・端点・rank不正はinvalid_argument。BigIntのBoost依存。

## 仕組み

基数B=min(L,R)+1の位取りで順位を多倍長整数へ厳密変換します。未対応用dummy列を加えた矩形Hungarianで重み和最大の割当を作ります。Bより本数が小さいので下位順位からの桁上がりがなく、辞書式比較と一致します。

## 計算量

順位数r、K=L+RとしてO(L²K)回のBigInt加減比較に、重み配列の構築を加えます。整数長O(r log(min(L,R)+1))bit、領域O(LR+(L+R+r)r log(min(L,R)+1))bit程度。順位数が多いと通常整数matchingより高コストです。

## 使用する問題・検証

希望順位を最優先する応募者と枠の割当、優先レベル付き資源配分。

500小二部グラフの全matchingを列挙し順位profileと比較。第1順位1本が第2順位2本に優先する例を含む。 [実行できる使用例](../../examples/graph_flow/rank_maximal_matching.cpp)も文書とコードの一致を検査します。

`make verify-graph-advanced` / `python3 verify/run_graph_advanced.py --sanitize` / `python3 -m tools.cp_tools.suite --stdlib-debug --runners graph_advanced`。[検証コード](../../verify/graph_advanced/)と[検証範囲](../graph_advanced/README.md#検証)を参照。オンライン提出ACは未確認です。図解の教材モデルとC++の違いは[図解の説明](../../visualizations/graph_advanced/README.md)に記載しています。
