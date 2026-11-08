---
title: 桁DP・上限付きの状態遷移
documentation_of: //dp_optimization/digit_dp.hpp
---

# 桁DP・上限付きの状態遷移

十進整数0〜upperを数えます。利用者が有限状態の遷移と最終受理条件を指定するテンプレートです。上限は文字列なので64ビットを超える桁数も扱えます。

[カテゴリ索引](README.md) / [操作できる図解](../../visualizations/dp_optimization/index.html#digit) / [実装](../../dp_optimization/digit_dp.hpp) / [参考資料](references.md)

## usage

```cpp
#include <iostream>
#include "dp_optimization/digit_dp.hpp"
int main() {
    // 1〜1000 のうち桁和が3の倍数。0はstarted=falseで除く。
    auto count = cp::digit_dp("1000", 3, 0,
        [](int state,int digit,bool,bool) { return (state+digit)%3; },
        [](int state,bool started) { return state==0 && started; });
    std::cout << count << '\n';
}
```

[動く使用例](../../examples/dp_optimization/digit_dp.cpp)。リポジトリのルートをincludeパスへ通します。

## API・使用条件

```text
digit_dp<Count=long long>(upper,state_count,initial_state,transition,accept) -> Count
transition(state,digit,started,next_started) -> 次の状態 または -1
accept(state,started) -> bool
```

upperは空でない十進数字列。先頭0を許容し、非数字／空はinvalid_argument。state_count>0、initial_stateは有効な状態番号。遷移先は-1（拒否）か[0,state_count)が条件でassert検査。

startedは既に非0桁を選んだか、next_startedは今回を含むフラグです。**先頭0もtransitionへ渡す**ので、桁に関する状態を変えたくなければ利用者がフラグで分岐します。整数0は最後までstarted=false。0を含めるかはacceptで選びます。各整数をupperと同じ桁数へ0埋めした経路として1回ずつ数えます。

CountはCount(0)、Count(1)、コピー、==、+=を持つ型。標準整数は自動の超過検査（bool不可）。法を取る場合は+=等を定義した型を渡します。[ジャッジ形式](../../verify/dp_optimization/digit_dp.test.cpp)にmod 10^9+7の例があります。

## 原理・計算量・注意点

桁数D、状態数Sとして、遷移・Count操作がO(1)なら時間O(10DS)、空間O(S)。tight（上限一致）×startedの4状態をローリングします。コールバックの呼出し順や回数に依存しないでください。

Countの全中間個数・最終の合計が型に収まる必要があります。受理されない状態も数えるので、最終答えだけが収まっても途中でoverflow_errorになる場合があります。mod型等の独自型の算術はその型の責任です。

[L,R]なら、利用者が十進文字列のL−1を計算し、F(R)−F(L−1)を取る形が基本です。L=0の場合の下端や法の減算は問題側で処理します。

## 使用する問題・検証範囲

[EDPC S - Digit Sum](https://atcoder.jp/contests/dp/tasks/dp_s)。

上限0〜1000・桁和の法1〜9を全整数と比較。先頭0、0の含む／除く、禁止桁・同じ隣接桁の拒否、符号付き／符号なし極値と超過。mod型で10万桁。

[検証入口](../../verify/run_dp_optimization.py)は通常／ASan・UBSan、debug/release、単独include、混在include、複数翻訳単位、使用例、独立オラクル、ジャッジ形式の入出力を確認します。オンラインACは未確認です。
