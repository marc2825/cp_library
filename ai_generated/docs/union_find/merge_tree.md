---
title: マージ履歴木
documentation_of: //union_find/merge_tree.hpp
---

# マージ履歴木

成功した成分併合ごとに二つの旧根を子にした新頂点を作る。

## API・使用条件

MergeTree(n)、merge(u,v)->履歴頂点ID、component_root(v)、roots()、parent、children。元頂点0..n-1。

葉のchildrenは{-1,-1}、根のparentは-1。同成分mergeは新頂点を作らず現在根を返す。時刻/重みラベルは返ったIDに利用者が保存できる。森と空入力に対応。

C++17、namespace cp。特記以外は0始まり・半開区間。前提条件は呼出し側が守り、添字/範囲の違反はdebug assertで検査します。整数の全中間値とコンテナの添字が使用型に収まることが必要です。計算量は要素の演算/比較/コピーをO(1)としたものです。

## Usage

```cpp
#include <iostream>
#include "union_find/merge_tree.hpp"
int main() {
    cp::MergeTree d(3); int a=d.merge(0,1), b=d.merge(1,2);
    std::cout<<a<<" "<<b<<" "<<d.parent[0]<<"\n";
}
```

出力: `3 4 3`。

## 計算量

構築O(N)、merge/根照会ならしO(α(N))、rootsはO(N)、領域O(N)。


## 問題例・検証・参考資料

用途に対応する[問題例と一次資料](../a_priority/references.md)を参照。`python3 verify/run_a_priority.py`（`--sanitize`対応）で単独/重複include、複数翻訳単位、上記使用例と独立した素朴解の差分検証を行います。[検証範囲](../a_priority/README.md)。オンラインACは未確認です。

[操作できる図解](../../visualizations/a_priority/index.html#merge_tree) / [図の範囲](../../visualizations/a_priority/README.md) / [カテゴリ索引](README.md) / [動く例](../../examples/union_find/merge_tree.cpp)
