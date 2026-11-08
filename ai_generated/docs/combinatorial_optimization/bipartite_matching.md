---
title: 二部最大マッチング（共通API）
documentation_of: //combinatorial_optimization/bipartite_matching.hpp
---
# 二部最大マッチング（共通API）

[カテゴリ・使用条件](README.md) / [操作できる図解](../../visualizations/combinatorial_optimization/index.html#bipartite) / [参考資料](references.md)

`BipartiteMatching(left_size,right_size)`と`add_edge(left,right)`、`maximum_matching()`は[3.9の共通実装](../graph_flow/bipartite_matching.md)です。このカテゴリのヘッダはその実装をincludeし、重複した別実装は作りません。

左右それぞれ0始まり。結果はcardinality、left_match、right_match（未対応−1）、左頂点順のpairsです。全ての行を埋める必要はなく、費用も扱わず、組数だけを最大化します。重複辺と空の片側を許します。呼ぶたびに空の対応から解きます。

この既存APIの次元・頂点番号はassertによる使用条件です。NDEBUGで無効になるため、利用者が条件を守ります。新しいGeneralMatchingとMinCostFlowの例外による境界検査とは契約が異なります。詳細は3.9を参照してください。

## usage

```cpp
#include <iostream>
#include <vector>
#include "combinatorial_optimization/bipartite_matching.hpp"
int main(){
    cp::BipartiteMatching graph(2,3);
    graph.add_edge(0,0);graph.add_edge(0,1);graph.add_edge(1,0);
    auto r=graph.maximum_matching();
    std::cout<<r.cardinality<<'\n';
    for(auto [a,b]:r.pairs)std::cout<<a<<' '<<b<<'\n';
}
```

[実行できる使用例](../../examples/combinatorial_optimization/bipartite_matching.cpp)。

## 計算量・数値の条件

O((V+E)√(V+1)) 時間・O(V+E) 領域。nは行数、mは列数（全行を割り当てるときn≤m）、Vは頂点数、Eは登録辺数です。この二部版ヘッダ自体は標準C++17のみで使えます。費用のある新実装はGCC/Clangの検査付き128ビット演算を使用します。返す総費用はlong longまたはCostに収まる必要があり、超過はoverflow_errorです。[共通仕様](README.md#共通仕様)を参照してください。

## 問題・検証

[Library Checker Bipartite Matching](https://judge.yosupo.jp/problem/bipartitematching)形式のローカル検証コードを用意しました。独立した全割当・頂点部分集合DP・全整数フローと比較し、境界、数値極値と復元も確認します。[検証入口](../../verify/run_combinatorial_optimization.py)と[全検証範囲](README.md#ローカル検証)を参照。オンラインACは未確認です。
