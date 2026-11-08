---
title: 差分制約
documentation_of: //graph_basic/difference_constraints.hpp
---

# 差分制約

x[to]−x[from]≤boundという不等式を蓄積し、全成分の実行可能性を判定します。全頂点へ重み0の仮想始点を置くBellman–Fordで、解か矛盾する負閉路を返します。

[操作できる図解](../../visualizations/graph_extensions/index.html#difference_constraints) / [15項目の検証](../graph_extensions/README.md) / [実装](../../graph_basic/difference_constraints.hpp)

## usage

```cpp
#include <iostream>
#include "graph_basic/difference_constraints.hpp"
int main() {
    cp::DifferenceConstraints<> system(3);
    system.add_constraint(0,1,-2); system.add_constraint(1,2,1); system.add_constraint(0,2,0);
    auto r=system.solve();
    for (int v=0;v<3;++v) std::cout << cp::optimization_to_string(r.assignment[v]) << (v==2?'\n':' ');
    system.add_constraint(2,0,0);
    std::cout << !system.solve().feasible() << '\n';
}
```

出力:

```text
0 -2 -1
1
```

## API・使用条件

`DifferenceConstraints<T>(n)`、`add_constraint(from,to,bound)`（制約IDを返す）、`solve()`、`graph()`。結果の`feasible()`、128bitの`assignment`、optionalな`contradiction`を利用します。

Tはbool以外64bit以下整数。可解時は全変数の解、不可解時はassignmentが空でcontradictionのedge_idsが制約ID。閉路の不等式を足すと0≤負数となり矛盾を証明できます。解は一意でなく、全変数への同じ定数の加算も解。solveは状態を変えず追加後に再実行可。変数数負値はinvalid_argument、範囲外はout_of_range。

## 計算量

各solveはO(V(V+E))時間、O(V+E)空間。負閉路抽出と共通coreを使います。

## 使用する問題・検証

日程の時刻差、順序の間隔制約、ポテンシャルの実行可能性。等式は逆向きの2制約で表現できます。

[動く使用例](../../examples/graph_basic/difference_constraints.cpp)と[独立比較テスト](../../verify/graph_extensions/)を`make verify-graph-extensions`で実行します。小規模全探索、復元した頂点・辺ID、最適値、境界・不正入力を確認します。オンラインACは未確認です。

[一次資料・設計判断](../graph_extensions/references.md) / [カテゴリ索引](README.md)。
