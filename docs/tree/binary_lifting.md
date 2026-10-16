---
title: Binary Lifting・一意な遷移の繰り返し
documentation_of: //tree/binary_lifting.hpp
---

# Binary Lifting・一意な遷移の繰り返し

各頂点に一つの遷移先があるとき、k 回遷移した先を前計算で高速に求めます。木の親だけでなく、自己ループや閉路を含む functional graph にも使えます。

[カテゴリ索引](README.md) / [操作できる図解](../../visualizations/tree/index.html#lifting) / [実装](../../tree/binary_lifting.hpp) / [参考資料](references.md)

## usage

```cpp
#include <iostream>
#include "tree/binary_lifting.hpp"
int main() {
    cp::BinaryLifting next({1, 2, 0, -1});
    std::cout << next.jump(0, 1000000000000000000ULL) << ' ' << next.jump(3, 1) << '\n';
}
```

[動く使用例](../../examples/tree/binary_lifting.cpp)。リポジトリのルートを include パスに指定します。

## API・使用条件

| API | 意味 |
|---|---|
| `BinaryLifting(next,max_steps=UINT64_MAX)` | `vector<int>` の遷移表をコピーして構築 |
| `jump(v,k)` | k 回先の頂点。k は uint64_t。途中で尽きれば `-1` |
| `size()` | 頂点数 |
| `max_steps()` | 構築時に指定した問い合わせ上限 |
| `up[b][v]` | 2^b 回先。不存在は `-1` |

next[v] は `[0,N)` または `-1`（終端）。この範囲と始点は assert で検査します。k=0 では始点自身。k>max_steps は release でも out_of_range。負数を uint64_t に変換して渡さないでください。

既定は64段で uint64_t の全範囲を扱えます。問題の制約に合う max_steps を明示するとメモリを減らせます。max_steps=0 でも1段は保持します。空の next は構築可能で、問い合わせはできません。元の next の変更・破棄には影響されません。

## 原理・計算量・注意点

L=max(1, max_steps のビット長) として、構築・空間 O(NL)、jump は O(max(1, k のビット長))、最大64段です。

`up[b][v] = up[b−1][up[b−1][v]]`。途中が -1 なら、以降も -1 にします。問い合わせは k の立ったビットだけ適用します。64ビットの最大値でも 64 以上のシフトをしません。

祖先・LCA・木上のパスも必要なら [DoublingLCA](doubling_lca.md) を使います。このクラスは遷移回数を制限するだけで、グラフの深さ・LCAや辺上の集約は持ちません。

## 使用する問題・検証

[CSES Planets Queries I](https://cses.fi/problemset/task/1750)。

1〜4頂点の終端を含む遷移表700個を全探索、0〜37回を素朴遷移と比較。3周期・自己ループ・終端・2^63・UINT64_MAX・上限0／超過・20万頂点。

共通の [検証入口](../../verify/run_tree.py) は C++17・警告エラー・debug/release・単独 include・複数翻訳単位・使用例・独立オラクル・ジャッジ入出力を確認します。`--sanitize` で ASan/UBSan。**オンライン AC は未確認**です。
