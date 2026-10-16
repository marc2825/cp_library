---
title: Fenwick Tree・累積和 lower_bound
documentation_of: //range_query/fenwick_tree.hpp
---

# Fenwick Tree

一点加算と区間和を扱います。頻度表の累積和から k 番目の要素を探す `lower_bound` も O(log n) です。

[実装](../../range_query/fenwick_tree.hpp) / [操作できる図解](../../visualizations/range_query/index.html#fenwick) / [カテゴリ一覧](README.md)

## API・条件

`cp::FenwickTree<T>`。`T(0)` を加算の単位元とし、`+=` と引き算を定義した可換な加算構造を使います。整数・modint などに対応します。大小比較は `lower_bound` を呼ぶ場合だけ必要です。

| API | 意味 | 計算量 |
|---|---|---|
| `FenwickTree(n=0)` | n 要素を 0 で初期化 | O(n) |
| `FenwickTree(vector<T>)` | 配列から構築 | O(n) |
| `size()` | 論理要素数 | O(1) |
| `add(p,x)` | `a[p] += x` | O(log n) |
| `set(p,x)` / `get(p)` | 一点代入／取得 | O(log n) |
| `sum(r)` | `[0,r)` の和 | O(log n) |
| `sum(l,r)` | `[l,r)` の和 | O(log n) |
| `lower_bound(w)` | `sum(i+1) >= w` となる最小の**要素添字 i** | O(log n) |

メモリ O(n)。`lower_bound` は `w<=0` なら 0、該当要素がなければ n を返します。空配列では常に 0 です。**すべての要素が非負**で、加算と順序が整合することが探索条件です。通常の `add/sum` は負数でも使えます。剰余型や丸め誤差のある浮動小数点数での探索は対象外です。

## usage

```cpp
#include "range_query/fenwick_tree.hpp"
#include <vector>
cp::FenwickTree<long long> fw(std::vector<long long>{0, 2, 0, 3});
fw.add(2, 1);                   // {0,2,1,3}
auto total = fw.sum(1, 4);      // 6
int index = fw.lower_bound(3);  // 2（累積和は {0,2,3,6}）
int absent = fw.lower_bound(7); // 4 == size()
```

[コンパイルできる使用例](../../examples/range_query/fenwick.cpp)。頻度を数える場合、`lower_bound(k)` が 1 始まりの k 番目の値の添字になります。削除時にも頻度を負にしないでください。

## 原理・注意点

内部の `bit[i]` は 1 始まりの位置 i に対応する `[i-lowbit(i),i)` の和です。取得は `i-=lowbit(i)`、更新は `i+=lowbit(i)` で O(log n) 個を辿ります。配列構築時は各ノードから親へ一度ずつ足します。

`lower_bound` は 2 の冪幅で候補を試し、目標未満のブロックだけ飛ばします。返す値は prefix の右端ではなく要素の添字なので、累積区間の右端が必要なら `i<n` を確認して `i+1` にします。

旧 `BIT<T>` は `n` に要素数+1 を保持し、`get` は O(1) でした。新実装は余分な元配列を持たず、`size()` が要素数、`get` は O(log n) です。旧公開メンバを読むコードには旧ヘッダを残しています。`lower_bound(w<=0)==0` の仕様は引き継いでいます。

## 使用する問題・検証

- [Library Checker: Point Add Range Sum](https://judge.yosupo.jp/problem/point_add_range_sum) — [ローカル検証用プログラム](../../verify/range_query/point_add_range_sum.test.cpp)。
- [AtCoder practice2 B](https://atcoder.jp/contests/practice2/tasks/practice2_b) — 一点加算・区間和。
- [AtCoder ARC033 C](https://atcoder.jp/contests/arc033/tasks/arc033_3) — 頻度を管理して k 番目を取り出す用途。

素朴配列との比較で add/set/get/sum、0 の連続、目標以下／総和超過、空配列、20 万要素を確認します。オンライン AC の記録ではありません。

参考: [ACL Fenwick Tree](https://atcoder.github.io/ac-library/production/document_ja/fenwicktree.html)、[KACTL FenwickTree](https://github.com/kth-competitive-programming/kactl/blob/main/content/data-structures/FenwickTree.h)。本実装の非正目標の戻り値は KACTL の -1 と異なります。
