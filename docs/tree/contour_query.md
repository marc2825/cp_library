---
title: Contour Query
documentation_of: //tree/contour_query.hpp
---

# Contour Query

[カテゴリ索引](README.md) / [B項目一覧](../b_7_11/README.md) / [操作できる図解](../../visualizations/b_7_11/index.html#contour_query) / [実装](../../tree/contour_query.hpp) / [一次資料と設計](../b_7_11/references.md)

重心祖先ごとの距離BITと枝別BITを持ち、指定頂点から距離帯に入る頂点だけの寄与を包除で集計します。

## usage

```cpp
#include <iostream>
#include "tree/contour_query.hpp"
int main() {
    cp::Graph<long long> g(3); g.add_edge(0,1,2); g.add_edge(1,2,3);
    cp::ContourQuery<> q(g); q.add(0,5); q.add(2,7);
    std::cout << q.sum(1,1,2) << '\n';
}
```

出力:

```text
12
```

## API・使用条件

`ContourQuery<T>(g)`のadd(v,x),set(v,x),get(v),sum(v,l,r)。`ContourAddPointGet<T>(g)`のadd(v,l,r,x),get(v)。無向森、重み無視、距離は辺数、半開区間[l,r)。Tは可換加法群。

頂点・辺IDは0始まり。グラフを受ける関数は構築時点の入力に対する計算です。重みや値の型は特記した制約に従います。

## 仕組みと境界

点加算/距離帯和と距離帯加算/点取得の両方向を提供します。同じ枝にいる対を引いて二重計数を防ぎます。l>=rは空、負の距離・LLONG_MIN/MAXの境界も安全にclampします。異なる成分は寄与しません。初期値は全0。

## 計算量

構築O(V log(V+1))、更新・検索O(log²(V+1))、領域O(V log(V+1))。 Vは頂点数、Eは論理辺数。テンプレートの比較・加減乗算・Specの各演算はO(1)として数えています。

## 使用する問題・検証

想定問題: 重心祖先ごとの距離BITと枝別BITを持ち、指定頂点から距離帯に入る頂点だけの寄与を包除で集計します。 [実行できる小問題](../../examples/tree/contour_query_b.cpp)。

Contour Queryを必要とする3.11の最適化・構造判定・更新クエリで利用します。上の使用例は実際にコンパイル・実行し、文書中のコードとの一致も検査します。

`python3 verify/run_b_7_11.py` / `--sanitize`。[検証コード](../../verify/b_7_11/)には定義からの全列挙、独立した素朴解、乱択比較と復元の整合性検査を置いています。[比較方法](../b_7_11/README.md#ローカル検証)を参照してください。オンライン提出・ACは未確認です。図解は小入力の教材モデルで、C++との差異は[図解の説明](../../visualizations/b_7_11/README.md)へ明記しています。
