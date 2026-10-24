---
title: Segment Tree Cover
documentation_of: //range_query/segment_tree_cover.hpp
---

# Segment Tree Cover（D-08）

半開区間を、完全二分木の互いに重ならない標準ノードへ左から右に分解します。時間軸セグ木が共有し、今後の区間辺グラフにも使える番号規約です。

## API・使用条件

`SegmentTreeCover(n)`、`cover(l,r)`、`for_each(l,r,visit)`、`interval(node)`、`leaf(p)`、`base()`、`node_capacity()`、`size()`。

base は max(1,n) 以上の最小2冪。根=1、子=2k/2k+1、葉=base+p、配列容量=2*base です。番号0は未使用。ID は size_t。n は非負の int、2*base が size_t に収まる環境で使用します。

cover は vector<size_t>、for_each は同じ順序で callback(node) を呼びます。**列挙順は左から右**なので非可換の順序にも利用できます。interval は padding を [0,n) に切り詰め、padding だけのノードには [n,n) を返します。空区間は空の分解。n=0 でも根の区間は [0,0) です。状態を持つ実体の木や値配列は確保しません。

C++17、namespace cp、0始まり・半開区間。添字・範囲の前提は debug assert で検査します。比較・コピー・集約演算を O(1) として計算量を示します。整数の途中値は使用型に収まることが必要です。

## Usage

```cpp
#include <iostream>
#include "range_query/segment_tree_cover.hpp"
int main() {
    cp::SegmentTreeCover layout(8);
    for (auto k : layout.cover(1,7)) {
        auto [l,r]=layout.interval(k);
        std::cout << k << ": [" << l << "," << r << ")\n";
    }
}
```

```text
9: [1,2)
5: [2,4)
6: [4,6)
14: [6,7)
```

## 計算量

構築・cover・for_each・interval は O(log(n+1))、base/leaf/node_capacity/size は O(1)。オブジェクト本体 O(1)、列挙の補助・結果は O(log(n+1))。

## 問題例・検証・参考資料

例題: 区間辺を標準ノード経由に変換するグラフ、時間軸への区間登録。n=0〜96 の全区間で、順序・被覆・重複なしを直接確認。padding、葉、INT_MAX の座標域も検証します。

[共通の検証入口・対応範囲](../data_structure_extensions/README.md) / [一次資料と設計判断](../data_structure_extensions/references.md)。オンライン AC は未確認です。

[操作できる図解](../../visualizations/data_structure_extensions/index.html#segment_tree_cover) / [図解の範囲](../../visualizations/data_structure_extensions/README.md) / [動く例](../../examples/range_query/segment_tree_cover.cpp) / [カテゴリ索引](README.md)
