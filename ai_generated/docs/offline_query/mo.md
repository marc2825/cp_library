---
title: Mo's Algorithm
documentation_of: //offline_query/mo.hpp
---

# Mo's Algorithm

静的な配列に対する区間クエリを並べ替え、現在の区間の両端を動かして答えます。種類数・頻度から計算できる値・区間の転倒数などに使います。両端の追加と削除を実装できることが条件です。

[操作できる図解](../../visualizations/offline_query/index.html#mo) は種類数を例に、区間・頻度・問い合わせの移動順を表示します。

## API

| API | 契約 |
|---|---|
| `cp::Mo(n)` | 長さ n。n≥0。配列や集計値そのものは保持しない |
| `add_query(l,r)` | `[l,r)` を登録して 0 始まり ID を返す。`0≤l≤r≤n`、空区間可 |
| `size()`, `query_count()` | 配列長、登録数 |
| `query_order(block_size=0)` | 処理順の ID 列。0 なら幅を自動選択、正数ならその幅 |
| `run(add_left,add_right,remove_left,remove_right,answer,block_size=0)` | 指定した処理で全問い合わせを解く |
| `run(add,remove,answer,block_size=0)` | 左右を区別しない集計用 |

コールバックには**配列添字**を渡します。`answer(id)` は元の登録 ID を一度ずつ受け取り、呼出順は Mo の処理順です。結果を元の順に出すには `ans[id]` に保存します。コールバックは値渡しなので、外の状態は参照キャプチャ等で共有します。

各 `run` は `[0,0)` から始まります。呼ぶ前に、利用者が集計状態を空へ初期化してください。終了時に空へ戻す処理はしません。再 `run` の前にも再初期化が必要です。登録クエリを変更せず、`run` 中に `add_query` を呼ばないでください。Q=0 の場合はコールバックを呼びません。

## usage

```cpp
#include <iostream>
#include <vector>
#include "offline_query/mo.hpp"
#include "ordered_structure/coordinate_compression.hpp"

int main() {
    std::vector<int> a{4, 1, 4, 2, 1};
    cp::CoordinateCompression<int> values(a);
    for (auto& x : a) x = *values.index(x);
    cp::Mo mo(int(a.size()));
    mo.add_query(0, 3);
    mo.add_query(1, 5);
    mo.add_query(2, 2);
    std::vector<int> count(values.size()), answer(mo.query_count());
    int distinct = 0;
    mo.run([&](int i) { if (count[a[i]]++ == 0) ++distinct; },
           [&](int i) { if (--count[a[i]] == 0) --distinct; },
           [&](int id) { answer[id] = distinct; });
    for (int x : answer) std::cout << x << '\n'; // 2, 3, 0
}
```

[動く例](../../examples/offline_query/mo_distinct.cpp) / [左右を区別する転倒数の例](../../verify/offline_query/range_inversions.test.cpp)

## 原理・注意点

左端のブロック番号を第一キーにし、右端は偶数ブロックで昇順・奇数で降順にします。区間間を移動する際は、左を広げ、右を広げ、左を縮め、右を縮める順です。離れた区間や空区間も、存在する要素だけを削除できます。

転倒数では、左から追加する値 x は「区間内で x より小さい個数」、右から追加する x は「x より大きい個数」の寄与を持ちます。左右共通のコールバックにしてはいけません。削除は追加の逆操作です。通常の Mo は更新を含む入力をそのまま扱えません。

## 計算量

N=配列長、Q=問い合わせ数、B=幅。1 回の追加／削除が T、回答処理が A のとき:

- 登録: 償却 O(1)。処理順のソート: O(Q log(Q+1))。
- 移動: O(N²/B + QB + N) 回、全体 O(Q log(Q+1) + (N²/B+QB+N)T + QA)。
- 自動幅 `max(1, floor(N/√max(1,Q)))` では、移動は O(N√Q+Q+N) 回。
- エンジンの追加メモリ O(Q)。配列・頻度等の利用者の状態は別途必要。

## 問題・検証・参考

[Library Checker: Static Range Inversions Query](https://judge.yosupo.jp/problem/static_range_inversions_query) に [ジャッジ形式](../../verify/offline_query/range_inversions.test.cpp) を用意しています。全探索・方向別処理・空区間・大入力と、転倒数の Python 素朴解比較を実施します。検証全体は [カテゴリ索引](README.md) を参照してください。

左右別の処理とクエリ ID の渡し方は [Luzhiled's Library の Mo](https://ei1333.github.io/library/other/mo.hpp.html) を参考にしました。幅と移動回数の考え方は [CP-Algorithms の Mo](https://cp-algorithms.com/data_structures/sqrt_decomposition.html#mos-algorithm) も参照しています。[設計の違い](references.md)。
