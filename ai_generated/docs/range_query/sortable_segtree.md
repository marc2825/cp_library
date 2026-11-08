---
title: Sortable Segtree
documentation_of: //range_query/sortable_segtree.hpp
---

# Sortable Segtree

ソート済みrunをkey値域の疎な木で表し、区間境界でrank分割、sort時に破壊的meldをします。runの順／逆方向の積を持ち、外側のSegtreeで列順に集約します。

## 使用例

```cpp
#include <iostream>
#include "range_query/sortable_segtree.hpp"
long long op(long long a,long long b) { return a+b; }
long long e() { return 0; }
int main() {
    cp::SortableSegtree<long long,op,e> s({{2,10},{0,20},{1,30}},3); s.sort(0,3);
    std::cout << s.get(0).second << " " << s.prod(0,2) << "\n";
}
```

出力: `20 50`。

## APIと使用条件

`SortableSegtree<S,op,e>(vector<pair<int,S>>,int universe)`、sort(l,r,descending=false)、prod(l,r)、get(p)はkeyとpayload、set(p,key,value)、size。

keyは一意、[0,U)のint、U>0。重複・値域外はinvalid_argument。sortとget/prodはrun境界を変更します。payloadは非可換可。setは現在の一要素のkey・payloadを変更できます。他要素と同じkeyへの変更はinvalid_argumentで、論理列は変えません。列の長さは固定です。

## 計算量

N初期要素Q操作、値域Uに対し総時間O((N+Q) log(N+1) log(U+1))の償却上界、領域O((N+Q) log(U+1)+N)。 演算・比較・値のコピーが O(1) の場合です。文字列などを格納すると、その処理時間・領域が加わります。

## 使用する問題と検証

[Library Checker: Point Set Range Sort Range Composite](https://judge.yosupo.jp/problem/point_set_range_sort_range_composite)。key順のsortをstd::sortと比較し、payloadの文字列連結で順序を確認します。

`python3 verify/run_b_1_6.py`（`--sanitize` 対応）で単独／重複include、debug/releaseの比較検証、上の使用例を確認します。検証範囲の詳細は[追加分の一覧](../b_1_6/README.md)。オンライン提出・ACは未確認です。

[操作できる図解](../../visualizations/b_1_6/index.html#sortable_segtree) / [カテゴリ索引](README.md) / [参考資料と設計判断](../b_1_6/references.md#range) / [図解の簡略化](../../visualizations/b_1_6/README.md)
