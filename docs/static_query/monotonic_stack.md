---
title: Monotonic Stack・最近傍の大小関係
documentation_of: //static_query/monotonic_stack.hpp
---

# Monotonic Stack

配列を順番に読み、直前・直後の「自分より小さい／大きい」要素の添字を O(n) で求めます。ヒストグラムの境界、区間の最小値に寄与する範囲などに使えます。

[実装](../../static_query/monotonic_stack.hpp) / [候補削除の図解](../../visualizations/static_query/index.html#stack) / [一覧](README.md)

## API・条件

`cp::MonotonicStack<T,Compare=std::less<T>>` は、左から読む候補スタックです。比較器は厳密弱順序とし、構築時の `strict=true` なら `<`、false なら `<=` に対応します。greater にすると `>` / `>=` です。

| API | 意味 | 計算量 |
|---|---|---|
| `MonotonicStack(strict=true,compare={})` | 空で初期化 | O(1) |
| `push(value)` | 新しい要素を追加し、**追加前の最近傍の適格な添字**を返す | ならし O(1) |
| `top()` | 最後の候補の `Entry{index,value}` | O(1) |
| `size()` / `empty()` | 保持している候補数／空判定 | O(1) |
| `processed_count()` | clear 以降の push 数 | O(1) |
| `candidates()` | 候補列への const 参照、底→頂上 | O(1) |
| `clear()` | 候補を破棄し、次の添字を 0 に戻す | O(候補数) |

一括で使う場合:

```cpp
previous_better_indices(a, strict=true, compare={}); // 左側の最近傍
next_better_indices(a, strict=true, compare={});     // 右側の最近傍
```

両方 O(n) 時間・O(n) メモリ。該当なしは -1、空配列には空の vector を返します。添字は元の配列の 0 始まりです。

## usage

```cpp
#include "static_query/monotonic_stack.hpp"
#include <functional>
#include <vector>
int main() {
    std::vector<int> a{3, 1, 1, 2};
    auto less = cp::previous_better_indices(a);        // {-1,-1,-1,2}
    auto less_equal = cp::previous_better_indices(a, false); // {-1,-1,1,2}
    auto greater = cp::next_better_indices(a, true, std::greater<int>{});
    cp::MonotonicStack<int> stack;
    stack.push(3);             // -1
    int previous = stack.push(4); // 0
    return previous != 0 || less[3] != 2 || less_equal[2] != 1 || greater[1] != 3;
}
```

[動く使用例](../../examples/static_query/sparse_and_nearest.cpp)。ヒストグラムの幅を求める例は [検証プログラム](../../verify/static_query/largest_rectangle_aoj.test.cpp) にあります。

## 原理・注意点

新しい値 x より良くない末尾候補を取り除きます。残った末尾が最近傍です。厳密な `<` の場合、等値も取り除きます。`<=` の場合は等値を残します。各添字が一度入って高々一度出るため全体 O(n) です。

これは入力をすべて保持する通常の LIFO スタックではなく、将来の検索に必要な候補だけを残す構造です。手動 pop や入力の巻き戻しは提供しません。size は入力数ではありません。破棄した候補を復元せずに処理を戻すことはできません。

値への参照が引数でも、push は値をコピーしてから処理します。vector の容量は clear 後も再利用できます。区間最小値の寄与を左右で数える場合は、同値を重複計上しないよう片側を厳密、もう片側を非厳密にする等の境界設計が必要です。

## 使用する問題・検証

[AOJ DPL_3_C: Largest Rectangle in a Histogram](https://judge.u-aizu.ac.jp/onlinejudge/description.jsp?id=DPL_3_C) — 左右の厳密な小さい境界から幅を求める [検証プログラム](../../verify/static_query/largest_rectangle_aoj.test.cpp)。旧 `misc/Largest_Rectangle.hpp` もありますが、既存実装はそのまま保持しています。

全探索配列で min/max、厳密／非厳密、左右の最近傍を走査による素朴解と比較します。オンライン AC の記録ではありません。[出典・設計判断](references.md)。
