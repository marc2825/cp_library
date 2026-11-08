---
title: Implicit Treap
documentation_of: //ordered_structure/implicit_treap.hpp
---

# Implicit Treap

[カテゴリ索引](README.md) / [図解](../../visualizations/advanced/index.html#treap) / [実装](../../ordered_structure/implicit_treap.hpp) / [一次資料・設計判断](../advanced/references.md)

## usage

```cpp
#include <iostream>
#include "ordered_structure/implicit_treap.hpp"
long long op(long long a, long long b) { return a + b; }
long long e() { return 0; }
int main() {
    cp::ImplicitTreap<long long, op, e> t(std::vector<long long>{1, 2, 3});
    t.reverse(0, 3); t.insert(1, 9); t.erase(2); t.move(0, 1, 2);
    for (auto x : t.to_vector()) std::cout << x << ' ';
    std::cout << t.prod(0, 3) << '\n';
}
```

[実行できる使用例](../../examples/ordered_structure/implicit_treap.cpp)。C++17、namespace cp。ライブラリルートをincludeパスに追加してください。

## API・使用条件

`ImplicitTreap<S,op,e>(seed=固定値)` または `(vector<S>,seed)`。`insert(p,x)`、`erase(p)`、`set/get(p)`、`prod(l,r)`、`all_prod()`、`reverse(l,r)`、`move(l,r,p)`、`to_vector()`、`size/empty/clear`。moveは[l,r)を取り出した後の列のpへ挿入します。pは0..size−(r−l)。

非可換モノイドも反転できます。乱数優先度は固定seedで再現可能ですが、適応的入力に対する最悪時間保証はありません。削除済ノードもclearまで保持。添字・ノード数はint範囲、演算の中間値は利用側の責任。問い合わせは内部木を再編するので非const。

## 計算量・原理

構築は逐次挿入で期待O(n log(n+1))、各列操作は期待O(log n)、all_prodはO(1)、to_vectorはO(n)。格納は過去の挿入数に比例。再帰の深さは期待O(log n)・最悪O(n)。

部分木サイズを暗黙の添字としてsplit/mergeし、優先度のヒープ条件で平衡を保ちます。反転時は左右の子と順方向／逆方向の積を交換し、反転タグを伝えます。

## 使用する問題・検証

[関連問題](https://judge.yosupo.jp/problem/dynamic_sequence_reverse_range_sum)。関連問題は用途の例であり、このAPIだけでそのまま全て解けるという意味ではありません。特に動的連結性の関連問題は追加の成分和が必要です。

[追加分の検証入口](../../verify/run_advanced.py)で、素朴解との比較、空・重複・境界、非可換集約、履歴分岐、繰返し実行、20万要素の主要構造を検査します。実際の範囲は[検証一覧](../advanced/README.md)を参照。オンライン提出のACは未確認です。

図解は学習用の小さい入力で内部状態・候補・論理的な操作を追えます。[簡略化の範囲](../../visualizations/advanced/README.md)を参照してください。
