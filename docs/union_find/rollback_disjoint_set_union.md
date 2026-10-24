---
title: Rollback DSU
documentation_of: //union_find/rollback_disjoint_set_union.hpp
---

# Rollback DSU

[カテゴリ索引](README.md) / [図解](../../visualizations/advanced/index.html#rollback) / [実装](../../union_find/rollback_disjoint_set_union.hpp) / [一次資料・設計判断](../advanced/references.md)

## usage

```cpp
#include <iostream>
#include "union_find/rollback_disjoint_set_union.hpp"
int main() {
    cp::RollbackDisjointSetUnion d(3); auto saved = d.snapshot();
    d.merge(0, 1); d.merge(1, 2); std::cout << d.component_size(0) << ' ';
    d.rollback(saved); std::cout << d.component_count() << '\n';
}
```

[実行できる使用例](../../examples/union_find/rollback_disjoint_set_union.cpp)。C++17、namespace cp。ライブラリルートをincludeパスに追加してください。

## API・使用条件

`RollbackDisjointSetUnion(n)`、`leader/same/component_size/component_count`、`merge(u,v)->bool`、`snapshot()->size_t`、`undo()`、`rollback(snapshot)`。merge失敗も履歴を1個積むため、undoは直前のmerge呼出しを取り消します。snapshotは同一オブジェクトの現在の履歴の祖先に限ります。空履歴のundo・将来や破棄した枝のsnapshot・不正添字は契約外。

## 計算量・原理

leader/same/mergeはO(log n)、undoはO(1)、rollbackは取り消す回数に比例。格納O(n+履歴長)。

サイズの小さい木を大きい木へつなぎ、変更した根の対だけを保存します。経路圧縮を行わず、巻戻し時に親とサイズと成分数を復元します。

## 使用する問題・検証

[関連問題](https://judge.yosupo.jp/problem/unionfind)。関連問題は用途の例であり、このAPIだけでそのまま全て解けるという意味ではありません。特に動的連結性の関連問題は追加の成分和が必要です。

[追加分の検証入口](../../verify/run_advanced.py)で、素朴解との比較、空・重複・境界、非可換集約、履歴分岐、繰返し実行、20万要素の主要構造を検査します。実際の範囲は[検証一覧](../advanced/README.md)を参照。オンライン提出のACは未確認です。

図解は学習用の小さい入力で内部状態・候補・論理的な操作を追えます。[簡略化の範囲](../../visualizations/advanced/README.md)を参照してください。

2026-10-24にmerge(u,v,callback)と[任意Dataを同時に戻すラッパー](rollback_disjoint_set_union_with_data.md)を追加しました。元クラスのsnapshotは引き続き履歴長です。
