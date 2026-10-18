---
title: C4の個数
documentation_of: //graph_combinatorics/count_four_cycles.hpp
---

# C4の個数

[カテゴリ索引](README.md) / [B項目一覧](../b_7_11/README.md) / [操作できる図解](../../visualizations/b_7_11/index.html#count_four_cycles) / [実装](../../graph_combinatorics/count_four_cycles.hpp) / [一次資料と設計](../b_7_11/references.md)

次数と番号の順序で一番大きい頂点を固定し、その下の2辺walkを同じ終点ごとに組み合わせます。

## usage

```cpp
#include <iostream>
#include "graph_combinatorics/count_four_cycles.hpp"
int main() {
    cp::Graph<int> g(4); for(int u=0;u<4;++u) for(int v=u+1;v<4;++v) g.add_edge(u,v);
    std::cout << cp::count_four_cycles(g) << '\n';
}
```

出力:

```text
3
```

## API・使用条件

`count_four_cycles(g)`→uint64_t。単純無向、ループ・平行辺はinvalid_argument。重み無視。

頂点・辺IDは0始まり。グラフを受ける関数は構築時点の入力に対する計算です。重みや値の型は特記した制約に従います。

## 仕組みと境界

4頂点集合そのものではなく、異なる長さ4の閉路を数えます。K4には3個。対角線があっても数えます。結果がuint64を超える場合はoverflow_error。

## 計算量

時間O(V+E sqrt(E+1))（前処理sortを含む）、領域O(V+E)。 Vは頂点数、Eは論理辺数。テンプレートの比較・加減乗算・Specの各演算はO(1)として数えています。

## 使用する問題・検証

想定問題: 次数と番号の順序で一番大きい頂点を固定し、その下の2辺walkを同じ終点ごとに組み合わせます。 [実行できる小問題](../../examples/graph_combinatorics/count_four_cycles_b.cpp)。

C4の個数を必要とする3.10の最適化・構造判定・更新クエリで利用します。上の使用例は実際にコンパイル・実行し、文書中のコードとの一致も検査します。

`python3 verify/run_b_7_11.py` / `--sanitize`。[検証コード](../../verify/b_7_11/)には定義からの全列挙、独立した素朴解、乱択比較と復元の整合性検査を置いています。[比較方法](../b_7_11/README.md#ローカル検証)を参照してください。オンライン提出・ACは未確認です。図解は小入力の教材モデルで、C++との差異は[図解の説明](../../visualizations/b_7_11/README.md)へ明記しています。
