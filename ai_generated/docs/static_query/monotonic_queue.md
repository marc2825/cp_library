---
title: Monotonic Queue
documentation_of: //static_query/monotonic_queue.hpp
---

# Monotonic Queue

右端への追加と左端からの削除を繰り返しながら、区間内の最小／最大値と、その元の添字を取得します。

[実装](../../static_query/monotonic_queue.hpp) / [論理キューと候補の図解](../../visualizations/static_query/index.html#queue) / [一覧](README.md)

## API・条件

`cp::MonotonicQueue<T,Compare=std::less<T>>`。less は最小、greater は最大を優先します。比較器は厳密弱順序です。同値の最良候補は最も古い添字を返します。

| API | 意味 | 計算量 |
|---|---|---|
| `MonotonicQueue(compare={})` | 空で初期化 | O(1) |
| `push(value)` | 論理キューの末尾へ追加 | ならし O(1)、最悪 O(候補数) |
| `pop()` | 論理キューの最も古い要素を削除 | O(1) |
| `best()` | 最良値への const 参照 | O(1) |
| `best_index()` | 最良値の通算 push 添字（clear から 0 始まり） | O(1) |
| `size()` / `empty()` | **論理キューの長さ**／空判定 | O(1) |
| `candidates()` | 単調な候補 deque の const 参照 | O(1) |
| `clear()` | 候補・添字カウンタをリセット | O(候補数) |

メモリ O(現在の候補数)、最悪 O(現在の論理長)。空の pop/best/best_index は呼べません。通常の FIFO の先頭値は保持していない場合があり、front() は提供しません。

## usage

```cpp
#include "static_query/monotonic_queue.hpp"
int main() {
    cp::MonotonicQueue<int> queue;
    queue.push(4); queue.push(2); queue.push(2);
    queue.pop(); // 論理要素 4 を削除。最良候補 2 は消えない
    int value = queue.best();       // 2
    int index = queue.best_index(); // 1（最も左の 2）
    queue.pop();                    // 添字 1 の 2 を削除
    return value != 2 || index != 1 || queue.best_index() != 2;
}
```

[動く使用例](../../examples/static_query/window_minimum.cpp)。固定長なら [Sliding Window Minimum](sliding_window_minimum.md) を直接使えます。

## 原理・注意点

新しい要素より悪い候補を後ろから破棄します。その候補は新要素より先に窓の外へ出るため、今後の最良値に不要です。等値は残し、左側を先に返します。論理的な左端が候補の添字を通過したときだけ、候補の前を取り除きます。

`pop()` は最良候補の削除を意味しません。入力全体を保存する deque でもありません。可変長の窓にも使えますが、左端・右端を後戻りさせる操作には対応しません。内部の添字は削除のたびに詰め直さず、clear まで通算です。

## 使用する問題・検証

[AOJ DSL_3_D: Sliding Minimum Elements](https://judge.u-aizu.ac.jp/onlinejudge/description.jsp?id=DSL_3_D) — 逐次入力で固定長の窓を管理する [検証プログラム](../../verify/static_query/monotonic_queue_aoj.test.cpp)。

ランダムな push/pop/clear を素朴な deque の min/max と比較し、同値の左優先、候補から既に消えた要素の pop、カスタム比較器、20 万要素を確認します。オンライン AC の記録ではありません。

参考: [CP-Algorithms Minimum Queue・添字で削除する方法](https://cp-algorithms.com/data_structures/stack_queue_modification.html#queue-modification-method-2)。
