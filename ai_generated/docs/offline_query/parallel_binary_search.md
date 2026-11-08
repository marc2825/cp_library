---
title: Parallel Binary Search（並列二分探索）
documentation_of: //offline_query/parallel_binary_search.hpp
---

# Parallel Binary Search

更新を最初から t 回適用した状態について、各問い合わせが初めて成立する t をまとめて求めます。各判定は `false … false, true … true` と単調である必要があります。「並列」は探索をまとめる意味で、スレッドや並行実行は使いません。

[操作できる図解](../../visualizations/offline_query/index.html#pbs) は辺の追加に対する初回連結時刻を例に、各 Q の low/mid/high と毎ラウンドの再構築を表示します。

## API・境界

```text
cp::parallel_binary_search(updates, queries, reset, apply, check)
  -> vector<int>（問い合わせ ID 順）
```

M=`updates` は `0≤M<INT_MAX`、Q=`queries` は非負。実際に調べる状態は **0..M** です。

| コールバック | 役割 |
|---|---|
| `reset()` | 更新を一つも適用していない初期状態に戻す。毎ラウンド一度 |
| `apply(i)` | 0 始まりの更新 i を適用。そのラウンドで 0,1,… と順に呼ぶ |
| `check(q)` | 現在の状態で Qq が成立するか bool を返す。判定で表す状態を変えない |

`check` が Union-Find の経路圧縮を行うことは可能です。判定を繰り返した際の論理的な状態が変わらないことが条件です。途中に更新を追加したり、問い合わせごとに更新を消費したりしません。コールバックは値渡しなので、外部状態は参照で共有します。

答えは初期状態で成立すれば **0**、更新を t 回行って初めて成立すれば **t**、最後まで成立しなければ **M+1**。M=0 でも初期状態を判定します。Q=0 なら空の結果を返し、コールバックは一度も呼びません。終了後の利用者の状態は最後のラウンドで到達した状態であり、M 回適用済みや初期状態になる保証はありません。

## usage

```cpp
#include <iostream>
#include <utility>
#include <vector>
#include "offline_query/parallel_binary_search.hpp"
#include "union_find/disjoint_set_union.hpp"

int main() {
    std::vector<std::pair<int, int>> edges{{0, 1}, {2, 3}, {1, 2}};
    std::vector<std::pair<int, int>> queries{{0, 0}, {0, 3}, {0, 4}};
    cp::DisjointSetUnion dsu(5);
    auto answer = cp::parallel_binary_search(int(edges.size()), int(queries.size()),
        [&] { dsu.reset(5); },
        [&](int i) { dsu.merge(edges[i].first, edges[i].second); },
        [&](int q) { return dsu.same(queries[q].first, queries[q].second); });
    for (int t : answer) std::cout << t << '\n'; // 0, 3, 4 (4 means absent)
}
```

[動く例](../../examples/offline_query/first_connected.cpp)

## 原理・計算量

各 Q に未成立の番兵 `low=-1` と成立の番兵 `high=M+1` を置きます。番兵は実際の `check` には渡しません。未確定 Q の mid を時刻別に集め、初期化後に更新を順に適用し、各 mid で判定して探索範囲を半分にします。M 個の更新を Q 回繰り返す必要がなくなります。

reset の時間 R、1 更新の時間 U、1 判定の時間 C とすると、時間は **O((M+Q+R+MU+QC) log(M+2))**、追加メモリ **O(M+Q)**（利用者のデータ構造を除く）。バケツは整数の連結リストにして、各時刻ごとの vector 確保を避けています。各ラウンドは必要な最大 mid まで適用しますが、バケツ初期化は O(M) です。

単調性は検査しません。例えば負の増分を含む「累積値が閾値以上」や、追加と削除が混在する「連結か」は前提を満たすとは限りません。

## 問題・検証・参考

[AGC002 D: Stamp Rally](https://atcoder.jp/contests/agc002/tasks/agc002_d) の [ジャッジ形式](../../verify/offline_query/stamp_rally.test.cpp) では、二つの頂点の連結成分の和集合サイズを判定します。同じ成分ならサイズを二度足さないようにします。ローカルの Python BFS と比較しています。

M=0..80 の全成立境界、初期 true／不成立、Q=0、コールバックの更新順・回数、200 グラフ履歴、20 万状態を検証します。[検証範囲](README.md)。

応用の参考は [robert1003 の Parallel Binary Search 解説](https://robert1003.github.io/2020/02/05/parallel-binary-search.html)。この実装は毎ラウンド reset するバケツ方式で書いており、解説の再帰・巻き戻し方式をそのまま取り込んではいません。[参考資料](references.md)。
