---
title: 座標圧縮＋BIT による Order Statistics
documentation_of: //ordered_structure/compressed_order_statistics.hpp
---

# 座標圧縮＋BIT による Order Statistics

挿入候補が事前に分かる多重集合について、個数・順位・k 番目・前後検索を O(log M) で扱います。座標ごとの頻度を [3.3 の Fenwick Tree](../range_query/fenwick_tree.md) に保持します。

[実装](../../ordered_structure/compressed_order_statistics.hpp) / [頻度・BIT・探索の図解](../../visualizations/ordered_structure/index.html#statistics) / [一覧](README.md)

## API・条件

`cp::CompressedOrderStatistics<T,Compare=std::less<T>>`。M は登録した異なる座標数。比較器順での多重集合を管理します。以下の大小関係は既定の昇順の表記です。greater なら順位・前後・区間は降順で解釈します。

| API | 意味 | 計算量 |
|---|---|---|
| `CompressedOrderStatistics(universe={},compare={})` | 候補を圧縮し、空の多重集合を作る | O(N log N + M) |
| `size()` / `empty()` | 重複を含む総数 long long／空判定 | O(1) |
| `universe_size()` / `compression()` | 候補の種類数／圧縮器の const 参照 | O(1) |
| `insert(x,c=1)` | x を c 個追加。成功なら true | O(log M) |
| `erase(x,c=1)` | x を c 個削除。成功なら true | O(log M) |
| `erase_all(x)` | x を全削除し、削除個数を返す | O(log M) |
| `count(x)` | x の現在の個数。未登録も 0 | O(log M) |
| `order_of_key(x)` | x 未満の個数 | O(log M) |
| `count_less_equal(x)` | x 以下の個数 | O(log M) |
| `count_range(l,r)` | 値域 `[l,r)` の個数。逆順／空なら 0 | O(log M) |
| `kth(k)` | **0 始まり**の k 番目。範囲外なら nullopt | O(log M) |
| `lower_bound(x)` / `upper_bound(x)` | 最小の x 以上／x より大きい値、optional | O(log M) |
| `predecessor(x,inclusive=false)` | 最大の x 未満、true なら x 以下、optional | O(log M) |
| `successor(x,inclusive=false)` | 最小の x より大きい値、true なら x 以上、optional | O(log M) |
| `clear()` | 個数を 0 にする。登録座標は保持 | O(M) |

更新・問い合わせに追加メモリ O(1)、保持する BIT・頻度は O(M) です。圧縮器の確保容量は元の候補列長 N にも依存します。返す optional<T> は座標の代表値をコピーします。

**挿入・削除する値は事前登録が必要**です。count と、順位・前後・区間の**問い合わせ閾値は未登録でも構いません**。初期要素と将来の insert の値を集めて構築します。座標を後から増やす場合は、新しいインスタンスを作ります。

## usage

```cpp
#include "ordered_structure/compressed_order_statistics.hpp"
int main() {
    cp::CompressedOrderStatistics<long long> values({-5,20,100}); // まだ空
    values.insert(20,2); values.insert(100); values.insert(-5);
    auto x = values.kth(2);                  // 20
    auto rank = values.order_of_key(20);     // 1
    auto count = values.count_range(0,100);  // 2（閾値 0 は未登録でもよい）
    bool unknown = values.insert(7);         // false
    bool shortage = values.erase(20,3);      // false、2 個のまま
    return x != 20 || rank != 1 || count != 2 || unknown || shortage || values.size() != 4;
}
```

[動く使用例](../../examples/ordered_structure/order_statistics.cpp)。set として使う場合は `if (!values.count(x)) values.insert(x)` とします。1 始まりの順位 k が入力される問題では `kth(k-1)` に変換します。

## 頻度を守る契約

負の c、未登録値、削除個数不足、追加後の総数が LLONG_MAX を超える場合は false で、論理状態を変更しません。c=0 は登録済みなら true、未登録なら false です。erase_all は未登録・0 個なら 0 を返します。

頻度を外部から直接変更する API は設けません。個数が非負なので累積和は単調になり、BIT の lower_bound を安全に使えます。

## 原理・注意点

値 x の厳密な順位は `BIT.sum(compression.lower_bound(x))`。k 番目は、累積和が **k+1** 以上になる最初の座標を BIT 上で探索し、元の値へ戻します。各座標に複数個ある場合、複数の k が同じ値を指します。

この構造は元の値が負でも使えます。探索の非負条件が必要なのは**頻度**です。総数と頻度の両方を検査するため、BIT 内の部分和も long long に収まります。標準 set で distance/next を使う O(n) の順位処理とは異なります。

## 使用する問題・検証

- [Library Checker: Ordered Set](https://judge.yosupo.jp/problem/ordered_set) — [検証プログラム](../../verify/ordered_structure/ordered_set.test.cpp)。ジャッジの k は 1 始まり。
- [Library Checker: Predecessor Problem](https://judge.yosupo.jp/problem/predecessor_problem) — [圧縮版](../../verify/ordered_structure/predecessor_compressed.test.cpp)。標準 set 版と同じ入力で検証。
- [AtCoder ARC033 C](https://atcoder.jp/contests/arc033/tasks/arc033_3) — [k 番目の取り出し](../../verify/ordered_structure/kth_extract_arc033.test.cpp)。

頻度の全探索、ランダム更新と全順位／境界の比較、失敗時の状態維持、LLONG_MAX、文字列・独自比較器・20 万座標を検証します。オンライン AC の記録ではありません。

参考: [KACTL FenwickTree の累積和探索](https://github.com/kth-competitive-programming/kactl/blob/main/content/data-structures/FenwickTree.h)、[ACL Fenwick Tree](https://atcoder.github.io/ac-library/production/document_ja/fenwicktree.html)。
