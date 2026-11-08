---
title: Matroid Union・Partition
documentation_of: //combinatorial_optimization/matroid_union.hpp
---

# O-03 Matroid Union・Partition

[カテゴリ](README.md) / [実装](../../combinatorial_optimization/matroid_union.hpp) / [操作できる図解](../../visualizations/optimization_extensions/index.html#matroid_union) / [一次資料](references.md)

未割当要素からBFSで交換先を探し、増加路に沿って所属する集合を組み替える。

## API・条件

`MatroidOracle` は `function<bool(const vector<int>&)>`。既存の[GraphicMatroid](../combinatorial_optimization/graphic_matroid.md)・[PartitionMatroid](../combinatorial_optimization/partition_matroid.md)を格納でき、異なる種類のマトロイドを混在させられます。台集合は共通の0..n−1です。渡される集合は重複なしですが、添字順とは限りません。oracleは順序によらず、同じ集合へ常に同じ独立性を返すことが前提です。

`matroid_union` は選択要素をk個の独立集合へ分配して選択数を最大化します。`matroid_partition` は同じ実体で、completeを見て全台集合を分割できたか判定できます。容量グループを表すPartitionMatroid自体とは別操作です。

結果のassignment[e]は担当マトロイドの添字、未選択は−1。sets[j]はその独立集合の昇順添字列、cardinalityは合計、completeはcardinality=n、oracle_callsは判定回数です。マトロイドが0個なら選択0、要素も0個ならcompleteです。

未割当要素から、追加できる集合または既存要素との交換をBFSで探します。見つかった最短の交換経路に沿って所属を更新し、選択数を1増やします。最大重みのunionや最少の同型マトロイド数はこのAPIの目的ではありません。必要なら外側で個数を調整して再実行します。

## 計算量

最大選択数r、マトロイド数k、oracle費用FでO((r+1)(k+1)(n+1)²(F+n+1))時間、O(n+k)追加領域。

## 使用例

```cpp
#include "combinatorial_optimization/matroid_union.hpp"
#include <iostream>
using namespace cp;
int main() {
    std::vector<int> groups{0, 0, 1, 1, 1};
    std::vector<MatroidOracle> oracles;
    for (auto capacities : std::vector<std::vector<int>>{{1, 2}, {1, 1}})
        oracles.emplace_back([groups, capacities](const std::vector<int> &s) {
            auto left = capacities;
            for (int e : s)
                if (--left[groups[e]] < 0)
                    return false;
            return true;
        });
    auto r = matroid_partition(5, oracles);
    std::cout << r.cardinality << " " << r.complete << "\n";
}
```

出力:

```text
5 1
```

## 使用する問題・検証

辺の複数森林への分配、異なる独立条件を持つ担当先への割当。 オンラインACは未確認です。

[単体・全探索比較](../../verify/optimization_extensions/unit_test.cpp)、[独立Python比較](../../verify/run_optimization_extensions.py)、[使用例と参照値](../../verify/optimization_extensions/manifest.json)で検証します。検証コマンドと図解との相違は[カテゴリREADME](README.md)を参照してください。
