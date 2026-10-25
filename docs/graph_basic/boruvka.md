---
title: Borůvka・暗黙MST・XOR-MST
documentation_of: //graph_basic/boruvka.hpp
---

# Borůvka・暗黙MST・XOR-MST

各成分の最小外向き辺をまとめて採用するBorůvkaです。辺集合を持たずに最小辺を返すoracleも渡せます。XOR-MSTは64bit trieを使って成分外の最小XOR相手を探します。

[操作できる図解](../../visualizations/graph_extensions/index.html#boruvka) / [15項目の検証](../graph_extensions/README.md) / [実装](../../graph_basic/boruvka.hpp)

## usage

```cpp
#include <iostream>
#include "graph_basic/boruvka.hpp"
int main() {
    auto r=cp::xor_minimum_spanning_tree({0,1,4,5});
    std::cout << cp::optimization_to_string(r.cost) << '\n';
}
```

出力:

```text
6
```

## API・使用条件

`boruvka_msf<T>(n,oracle)`、`boruvka_msf(g)`、`xor_minimum_spanning_tree(vector<uint64_t>)`。oracleは`BoruvkaComponents`（各頂点の連続成分IDと成分頂点列）を受け、成分数と同じ長さの`vector<optional<BoruvkaEdge<T>>>`を返します。Edgeはfrom/to/cost/id。結果は128bitのcost、components、採用edges。

oracleは常に同じ無向グラフを表し、各成分に接する最小外向き辺、なければnulloptを返す契約。返却サイズ・端点・成分外条件は検査しますが、最小性は呼出側の責任。明示版は負重み・多重辺可、loop無視。型Tは64bit以下整数、bool不可。XOR版は同値頂点も区別し、全64bitを処理。暗黙辺のidは−1、端点から特定します。

## 計算量

高々O(log(n+1))フェーズ。oracle時間をQとするとO((n+Q) log(n+1))、明示版O((n+m) log(n+1))、XOR版O(64n log(n+1))時間・O(64n)空間。

## 使用する問題・検証

全辺を列挙できない距離グラフ、XORを辺費用とする完全グラフ。

[動く使用例](../../examples/graph_basic/boruvka.cpp)と[独立比較テスト](../../verify/graph_extensions/)を`make verify-graph-extensions`で実行します。小規模全探索、復元した頂点・辺ID、最適値、境界・不正入力を確認します。オンラインACは未確認です。

[一次資料・設計判断](../graph_extensions/references.md) / [カテゴリ索引](README.md)。
