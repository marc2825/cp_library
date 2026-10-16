---
title: Segment Tree Beats
documentation_of: //range_query/segment_tree_beats.hpp
---

# Segment Tree Beats

[カテゴリ索引](README.md) / [図解](../../visualizations/advanced/index.html#beats) / [実装](../../range_query/segment_tree_beats.hpp) / [一次資料・設計判断](../advanced/references.md)

## usage

```cpp
#include <iostream>
#include "range_query/segment_tree_beats.hpp"
int main() {
    cp::SegmentTreeBeats s(std::vector<long long>{5, 1, 7});
    s.range_chmin(0, 3, 4); s.range_add(1, 3, 2); s.range_chmax(0, 2, 4);
    std::cout << s.range_sum(0, 3) << ' ' << s.range_min(0, 3) << ' ' << s.range_max(0, 3) << '\n';
}
```

[実行できる使用例](../../examples/range_query/segment_tree_beats.cpp)。C++17、namespace cp。ライブラリルートをincludeパスに追加してください。

## API・使用条件

`SegmentTreeBeats(vector<long long>)` または `(int n)`（初期値0）。`size/get(p)`、`range_chmin/chmax/add(l,r,x)`、`range_sum/min/max(l,r)`。0始まり・半開区間。空区間の和は0、最小は `INF`、最大は `-INF`、`INF=2^60`。境界はassert。

全要素は常に厳密に `(-INF,INF)` 内、各和・差・積・遅延加算値を含む全中間演算はlong long内が前提です。超過検査による例外保証はありません。コピーは独立。問い合わせも遅延を伝播するため非const。

## 計算量・原理

構築・格納O(n)。加算・集約はO(log n)、chmin/chmaxを含む更新列はならしO(log² n)/操作の標準的上界。単一のchmin/chmaxは最悪O(n)。

最大・第2最大・最大の個数、および最小側の同じ情報を保持します。第2最大 < x < 最大なら最大要素だけをまとめてchminし、条件が合わないときだけ子へ降ります。加算を先に伝播し、その後に親の上下限を子へ伝えます。

## 使用する問題・検証

[関連問題](https://judge.yosupo.jp/problem/range_chmin_chmax_add_range_sum)。関連問題は用途の例であり、このAPIだけでそのまま全て解けるという意味ではありません。特に動的連結性の関連問題は追加の成分和が必要です。

[追加分の検証入口](../../verify/run_advanced.py)で、素朴解との比較、空・重複・境界、非可換集約、履歴分岐、繰返し実行、20万要素の主要構造を検査します。実際の範囲は[検証一覧](../advanced/README.md)を参照。オンライン提出のACは未確認です。

図解は学習用の小さい入力で内部状態・候補・論理的な操作を追えます。[簡略化の範囲](../../visualizations/advanced/README.md)を参照してください。
