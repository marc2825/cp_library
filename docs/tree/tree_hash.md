---
title: 木のハッシュ
documentation_of: //tree/tree_hash.hpp
---

# 木のハッシュ

子の順序・頂点番号・辺重みに依存しない、根付き部分木と非根付き木の二重ハッシュ。

## API・使用条件

rooted_tree_hash(Graph<T>,root=0,seed=既定) -> RootedTreeHashResult{hash,parent,height,roots}、unrooted_tree_hash(Graph<T>,seed=既定)->TreeHash{size,value[2]}。TreeHashは比較演算対応。

無向、重みとラベルは無視。根付き版は森可、非根付き版は連結木（違反invalid_argument）、空は可。閉路/自己ループ/多重辺は拒否。同じseedの値同士を比較する。二重modでも衝突はあり得て、等値は同型性の厳密証明ではない。再現可能な固定seedが既定。

C++17、namespace cp、0始まり。頂点・辺IDと結果の公開配列は読み取り用です。入力を変えたら再構築/再計算してください。前提条件は呼出し側が守り、添字・符号などはdebug assertで検査します。整数の全中間値は使用型に収まる必要があります。

## Usage

```cpp
#include <iostream>
#include "tree/tree_hash.hpp"
int main() {
    cp::Graph<int> g(3);g.add_edge(0,1);g.add_edge(0,2);auto a=cp::rooted_tree_hash(g);
    std::cout<<(a.hash[1]==a.hash[2])<<" "<<cp::unrooted_tree_hash(g).size<<"\n";
}
```

出力: `1 3`。

## 原理・計算量

高さごとのsaltをSplitMix64で作り、各頂点でΠ(salt[height]+child_hash)をmod 1000000007/1000000009で計算。部分木サイズも比較に含む。非根付き木は重心を根とし、二重心なら二つの値の小さい方を採用する。整数積はuint64内で標準C++17のみ。

O(N)時間・領域。反復走査で20万頂点のパスも対応。

## 問題例・検証・参考資料

[一次資料・対象ごとの問題例](../graph_a/references.md)。`python3 verify/run_graph_a.py` / `--sanitize` で単独/重複include、複数翻訳単位、使用例、独立した全探索/素朴解と比較します。[検証範囲](../graph_a/README.md)。オンラインACは未確認です。

[操作できる図解](../../visualizations/graph_a/index.html#tree_hash) / [図の範囲](../../visualizations/graph_a/README.md) / [動く例](../../examples/tree/tree_hash.cpp) / [カテゴリ索引](README.md)
