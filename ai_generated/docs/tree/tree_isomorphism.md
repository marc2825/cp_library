---
title: 木の同型判定
documentation_of: //tree/tree_isomorphism.hpp
---

# 木の同型判定

[カテゴリ索引](README.md) / [B項目一覧](../b_7_11/README.md) / [操作できる図解](../../visualizations/b_7_11/index.html#tree_isomorphism) / [実装](../../tree/tree_isomorphism.hpp) / [一次資料と設計](../b_7_11/references.md)

重心を根とする木の子コード列を辞書で完全一致させるAHU型の判定です。対応する頂点の写像を復元します。

## usage

```cpp
#include <iostream>
#include "tree/tree_isomorphism.hpp"
int main() {
    cp::Graph<int> a(3),b(3); a.add_edge(0,1); a.add_edge(1,2); b.add_edge(2,0); b.add_edge(0,1);
    std::cout << cp::tree_isomorphism(a,b).isomorphic << '\n';
}
```

出力:

```text
1
```

## API・使用条件

`tree_isomorphism(a,b)`→TreeIsomorphismResult{isomorphic,mapping}。無向の連結木または空、重み無視。森・閉路はinvalid_argument。

頂点・辺IDは0始まり。グラフを受ける関数は構築時点の入力に対する計算です。重みや値の型は特記した制約に従います。

## 仕組みと境界

ハッシュは使用せず、ソートした子コード列を2本の木で同じ辞書へ登録します。各木の重心は高々2個なので根の組を高々4組試します。写像は入力a→b。同型でない場合mappingは空です。

## 計算量

キー列比較も含む保守的上界O(V² log(V+1))、領域O(V)。 Vは頂点数、Eは論理辺数。テンプレートの比較・加減乗算・Specの各演算はO(1)として数えています。

## 使用する問題・検証

想定問題: 重心を根とする木の子コード列を辞書で完全一致させるAHU型の判定です。対応する頂点の写像を復元します。 [実行できる小問題](../../examples/tree/tree_isomorphism_b.cpp)。

木の同型判定を必要とする3.11の最適化・構造判定・更新クエリで利用します。上の使用例は実際にコンパイル・実行し、文書中のコードとの一致も検査します。

`python3 verify/run_b_7_11.py` / `--sanitize`。[検証コード](../../verify/b_7_11/)には定義からの全列挙、独立した素朴解、乱択比較と復元の整合性検査を置いています。[比較方法](../b_7_11/README.md#ローカル検証)を参照してください。オンライン提出・ACは未確認です。図解は小入力の教材モデルで、C++との差異は[図解の説明](../../visualizations/b_7_11/README.md)へ明記しています。
