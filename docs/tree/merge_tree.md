---
title: 木上のマージ過程
documentation_of: //tree/merge_tree.hpp
---

# 木上のマージ過程

[カテゴリ索引](README.md) / [B項目一覧](../b_7_11/README.md) / [操作できる図解](../../visualizations/b_7_11/index.html#merge_tree) / [実装](../../tree/merge_tree.hpp) / [一次資料と設計](../b_7_11/references.md)

既存MergeTreeを木カテゴリから共有し、成功した成分併合ごとに二分木の内部頂点を作って履歴を保存します。

## usage

```cpp
#include <iostream>
#include "tree/merge_tree.hpp"
int main() {
    cp::MergeTree t(3); int a=t.merge(0,1); int b=t.merge(1,2);
    std::cout << a << ' ' << b << ' ' << t.parent[a] << '\n';
}
```

出力:

```text
3 4 4
```

## API・使用条件

`MergeTree(n)`、`merge(u,v),same,leader,component_root,roots`、公開parent/children。mergeの引数は元の葉ID。葉は0..n-1、新内部頂点はn以降。

頂点・辺IDは0始まり。グラフを受ける関数は構築時点の入力に対する計算です。重みや値の型は特記した制約に従います。

## 仕組みと境界

同成分の再併合では節点を増やさず既存の成分根を返します。parent=-1が履歴森の根、葉のchildrenは{-1,-1}。時刻や重みは新内部IDに対応する外部配列へ保存し、完成後にLCA等と組み合わせます。

## 計算量

merge/sameはならしO(α(V+1))、領域O(V)、rootsは履歴頂点数O(V)。 Vは頂点数、Eは論理辺数。テンプレートの比較・加減乗算・Specの各演算はO(1)として数えています。

## 使用する問題・検証

想定問題: 既存MergeTreeを木カテゴリから共有し、成功した成分併合ごとに二分木の内部頂点を作って履歴を保存します。 [実行できる小問題](../../examples/tree/merge_tree_b.cpp)。

木上のマージ過程を必要とする3.11の最適化・構造判定・更新クエリで利用します。上の使用例は実際にコンパイル・実行し、文書中のコードとの一致も検査します。

`python3 verify/run_b_7_11.py` / `--sanitize`。[検証コード](../../verify/b_7_11/)には定義からの全列挙、独立した素朴解、乱択比較と復元の整合性検査を置いています。[比較方法](../b_7_11/README.md#ローカル検証)を参照してください。オンライン提出・ACは未確認です。図解は小入力の教材モデルで、C++との差異は[図解の説明](../../visualizations/b_7_11/README.md)へ明記しています。
