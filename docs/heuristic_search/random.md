---
title: 乱数生成器
documentation_of: //heuristic_search/random.hpp
---

# 乱数生成器

[カテゴリ](README.md) / [実装](../../heuristic_search/random.hpp) / [使用例](../../examples/heuristic_search/random.cpp) / [図解](../../visualizations/heuristic_search/index.html#random) / [一次資料](references.md)

SplitMix64の64ビット状態をオブジェクトごとに保持します。seedを省略すると0。アルゴリズムと範囲変換を固定し、標準ライブラリの分布実装の差に依存しない整数列・shuffleを得ます。時計からの自動seedはありません。

## usage

```cpp
#include <algorithm>
#include <iostream>
#include <vector>
#include "heuristic_search/random.hpp"
int main() {
    cp::Random random(0);
    std::cout << random() << '\n' << random() << '\n';
    std::cout << random.integer(-7, -7) << '\n';
    std::vector<int> order{0, 1, 2, 3};
    random.shuffle(order.begin(), order.end());
    std::sort(order.begin(), order.end());
    std::cout << (order == std::vector<int>{0, 1, 2, 3}) << '\n';
}
```

先頭2値は16294208416658607535、7960286522194355700です。

| API | 内容 |
|---|---|
| Random(seed=0)、seed(value=0) | 初期化・再初期化。全uint64_tを許す |
| operator()() | uint64_tのrawを1個生成 |
| uniform(bound) | [0,bound)。bound=0はinvalid_argument |
| integer<Int>(lo,hi) | 両端を含む[lo,hi]。同じ整数型を渡す。逆区間はinvalid_argument |
| real() | 上位53ビットからdoubleの[0,1)上の2^-53刻みを生成 |
| shuffle(first,last) | ランダムアクセスの有効な[first,last)をFisher–Yatesで並べ替える |
| result_type、min()、max() | C++のURBGとして使用可能 |

integerはbool以外の64ビット以下の標準整数型。符号付きの負値、INT64_MIN〜INT64_MAX、uint64_t全範囲も使えます。符号の復元で範囲外の符号付きキャストに頼りません。全範囲はraw1回、長さ1の範囲もrawを消費します。uniformはraw < 2^64 mod boundを棄却し、残りを剰余へ写します。realはraw1回、shuffleは末尾からuniform(i)を使います。

コピーすると同じ内部状態になります。共有する1オブジェクトを複数スレッドから同時変更する設計ではありません。暗号用途には使いません。rawと整数演算は再現可能ですが、SAのexp等や実時間による打ち切りまで環境間で同じになる保証はありません。

## 計算量・使いどころ・検証

raw/real/seedはO(1)時間・O(1)領域。uniform/integerは一様なrawを仮定して期待O(1)、期待raw回数は2未満。shuffleは長さnに対して期待O(n)時間・O(1)追加領域です。反復リスタート、近傍選択、入力の並べ替えに使用します。

既知ベクトル、再seed、コピー、8整数型の範囲、64ビット全域、棄却率の大きい上限、realの格子、空を含むshuffle、URBG互換と不正型を検証。Python任意精度の定義とも比較します。乱数の統計品質の新しい証明は行っていません。問題例は[AtCoder Introduction to Heuristics](https://atcoder.jp/contests/intro-heuristics/tasks/intro_heuristics_a)の近傍選択です。
