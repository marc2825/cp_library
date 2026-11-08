---
title: スコア差分更新とapply/undo
documentation_of: //heuristic_search/incremental_score.hpp
---

# スコア差分更新とapply/undo

Moveだけを提案し、スコア差分とin-placeの状態更新・巻き戻しで探索します。

[カテゴリ索引](README.md) / [操作できる図解](../../visualizations/final_a/index.html#incremental) / [追加APIと検証](../final_a/README.md) / [一次資料](../final_a/references.md)

## API・条件

`try_scored_move(state,score,move,delta,apply,undo,accept) -> bool`。delta(const State&,const Move&)はScoreと同じ型の「新score−旧score」を返します。Scoreは符号付きの64bit以下の算術型。deltaは入力を変更しません。整数の差分加算超過はoverflow_error、非有限値はinvalid_argumentでapply前に拒否します。差分自体の計算範囲は呼出側が保証します。

apply(State&,move)の後、scoreを候補値にし、accept(const State&,old_score,new_score)を呼びます。falseならundoで状態とscoreを元へ戻します。acceptが例外を投げてもundoして例外を再送出します。**applyは強い例外保証、undoは例外を投げず正確に元へ戻す**必要があります。apply/undo内の任意の副作用は自動復元しません。

`simulated_annealing_delta(initial,evaluate,propose,delta,apply,undo,Random&,AnnealingOptions={}) -> SearchResult`。propose(const State&,Random&)がMoveを返します。初期評価1回だけを行い、以降は差分評価です。evaluationsは初期評価+差分評価。既存SAと同じ温度・採否・同点・stop規約を使い、棄却時はundo、最良更新時だけ状態を保存します。Stateコピーは独立に変更可能であることが必要です。

## Usage

```cpp
#include <iostream>
#include "heuristic_search/incremental_score.hpp"
int main() {
    long long state = 4, score = 16;
    auto delta = [](const long long& x, const long long& d) -> long long { return 2 * x * d + d * d; };
    auto apply = [](long long& x, long long d) { x += d; };
    auto undo = [](long long& x, long long d) { x -= d; };
    auto accept = [](const long long&, long long old, long long next) { return next < old; };
    std::cout << cp::try_scored_move(state, score, -1LL, delta, apply, undo, accept) << ' ' << state << ' ' << score << '\n';
    std::cout << cp::try_scored_move(state, score, 2LL, delta, apply, undo, accept) << ' ' << state << ' ' << score << '\n';
    cp::Random random(0); cp::AnnealingOptions options; options.max_iterations = 4;
    auto r = cp::simulated_annealing_delta(4LL, [](const long long& x) { return x * x; },
        [](const long long&, cp::Random&) { return -1LL; }, delta, apply, undo, random, options);
    std::cout << r.current_state << ' ' << r.current_score << ' ' << r.evaluations << '\n';
}
```

出力:

```text
1 3 9
0 3 9
0 0 5
```

[コンパイルできる使用例](../../examples/heuristic_search/incremental_score.cpp)。

## 原理

候補状態を丸ごとコピーして再評価する代わりに、変わった箇所だけから差分を求めます。採用判定までは一時的な状態として扱い、棄却時に逆操作を適用します。SAは差分で得た新旧scoreを既存の温度と乱数による判定へ接続します。

## 計算量

try_scored_moveは差分評価+apply+accept+必要時undoの費用。SAはI回の提案・差分更新の費用と、最良解更新時のStateコピー。score差分がO(1)なら全評価より評価コストを減らせますが、一般のMoveでO(1)を保証するAPIではありません。

## 使用する問題と検証

局所探索・SAの部分更新。二乗誤差の差分を全再計算と照合し、同じ乱数seedの既存SAと採否・最良/現在解・乱数の次の値まで一致させます。

`python3 verify/run_final_a.py`（`--sanitize`対応）で単独include・複数翻訳単位・このusageと出力・独立した素朴解・境界と例外を確認します。[詳しい検証範囲](../final_a/README.md)。オンライン提出・ACは未確認です。
