---
title: Kinetic Segment Tree
documentation_of: //range_query/kinetic_segtree.hpp
---

# Kinetic Segment Tree（D-22）

各位置に線形関数 `a*t+b` を置き、単調に進む整数時刻で区間最小値を答えます。子の勝者が逆転する最初の整数時刻をcertificateとして保存し、失効した部分だけを再計算します。

[カテゴリ一覧](README.md) / [D-20〜D-35一覧](../data_structure_advanced/README.md) / [操作できる図解](../../visualizations/data_structure_advanced/index.html#kinetic_segtree)

## API・使用条件

`KineticSegtree(vector<Line>, time=0)`、`Line{slope,intercept}` はlong long。

- `advance(t)` は現在以上の時刻へ進めます。
- `set(p,line)` は現在時刻で線形関数を変更。
- `minimum(l,r)` → `optional<Minimum>{index,value}`。同値は小さいindex、空区間は空。
- `next_event()` は全certificateで最初の失効時刻、存在しなければ空。`size()`, `time()`。

時刻は0以上のlong long、逆行はinvalid_argument。値と交点の計算はGCC/Clangの符号付き128bit。傾き・切片のlong long全域を扱います。添字はassert。

## 計算量・保存領域

構築 O(n)、点変更・区間最小 O(log n)、時刻更新は失効ノード数 k に対し O(k+1)、最悪 O(n)。領域 O(n)。時刻の飛び越しは中間イベントを逐次再生しません。

## 使用例

[コンパイル可能な使用例](../../examples/range_query/kinetic_segtree.cpp)と同じコードです。

```cpp
#include <iostream>
#include "../../range_query/kinetic_segtree.hpp"
int main(){
    cp::KineticSegtree tree({{1,0},{-1,10}});
    std::cout<<*tree.next_event()<<'\n';
    tree.advance(6); auto result=*tree.minimum(0,2);
    std::cout<<result.index<<' '<<static_cast<long long>(result.value)<<'\n';
}
```

```text
6
1 4
```

## 使用する問題

時間で線形に変わるコストの区間最小。物体ごとの線形式が途中で変更されるイベント処理。

## 検証

`python3 verify/run_data_structure_advanced.py` でヘッダ単独・重複include、全カテゴリ混在2翻訳単位、この使用例、独立した素朴解との比較を実行します。範囲・実行構成・証拠は[共通検証記録](../data_structure_advanced/README.md#検証)を参照してください。オンライン提出ACは未確認です。

## 参考資料・設計

[Basch–Guibas–Hershberger, kinetic data structures](https://doi.org/10.1145/258533.258573) のcertificateの考え方。整数時刻・同値時のindex優先を明示しています。 [共通の出典記録](../data_structure_advanced/references.md)にも設計上の選択をまとめています。
