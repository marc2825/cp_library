---
title: Simulated Annealing
documentation_of: //heuristic_search/simulated_annealing.hpp
---

# Simulated Annealing

[カテゴリ](README.md) / [実装](../../heuristic_search/simulated_annealing.hpp) / [使用例](../../examples/heuristic_search/simulated_annealing.cpp) / [図解](../../visualizations/heuristic_search/index.html#annealing) / [一次資料](references.md)

改善・同点を必ず採用し、悪化幅lossをexp(−loss/T)の確率で採用します。現在解から悪化しても最良解のコピーを残します。実装は反復予算つきの有限探索で、大域最適への収束を保証しません。

## usage

```cpp
#include <iostream>
#include "heuristic_search/simulated_annealing.hpp"
int main() {
    cp::Random random(42);
    cp::AnnealingOptions options;
    options.max_iterations = 12;
    options.start_temperature = 10;
    options.end_temperature = 0.1;
    const auto result = cp::simulated_annealing(8,
        [](int x) { return x * x; },
        [](int x, cp::Random&) { return x > 0 ? x - 1 : x; }, random, options);
    std::cout << result.best_state << ' ' << result.best_score << '\n';
    // Time budget: options.stop = [&] { return timer.expired(); };
    // options.progress = [&] { return timer.progress(); };
}
```

`simulated_annealing(initial,evaluate,propose,random,options={})`はSearchResult<State,Score>を返します。propose(const State&,Random&)は独立した候補を値で返し、元の状態を変更しません。近傍の合法性は利用側で保証します。採用しない場合のundoは不要です。コピーの重い状態を高速に差分更新する用途には利用側で専用ループを組む選択肢があります。

AnnealingOptionsは共通SearchOptionsにstart_temperature=1、end_temperature=0.01、cooling=CoolingSchedule::geometric、任意のprogressコールバックを追加します。温度は有限・正。linearは(1−p)T0+pT1、geometricはexp((1−p)log T0+p log T1)。加熱する設定も許します。annealing_temperature(options,p)を単独でも使えます。pは有限の[0,1]、両端の温度は正確に元の値を返します。

既定のpはstep/(max_iterations−1)。1反復なら開始温度、0反復なら温度を検査して初期解だけ返します。options.progressがあれば、提案・評価後に一度呼んだ値を使います。時間予算ならTimer.progressとTimer.expiredを両方接続してください。

悪化判定はScoreの整数比較を保ち、符号付き両端の差やUINT64_MAX付近の差1もオーバーフロー・桁落ちで同点にしません。差をlong doubleへ変換し、expで確率を計算します。改善・同点では採否用の乱数を消費せず、悪化ではRandom.realを必ず1回消費。提案で使う乱数は別途消費します。確率が0へunderflowした場合は採用しません。

stopは各反復の提案前に確認します。提案・評価の途中には割り込めません。iterationsは完了した採否判定数、evaluationsはiterations+1、acceptedは採用数。終了はiteration_limitまたはstoppedです。最良評価が同点なら先の状態を保持します。

## 計算量・使いどころ・検証

I反復ならI回の提案・評価、最大I回の状態更新と最良解コピー。状態コピー・提案・評価を定数と見ればO(I)時間、現在・候補・最良のO(1)個の状態を保持します。近傍生成や評価のコストは別途必要です。問題例は[AtCoder Introduction to Heuristics](https://atcoder.jp/contests/intro-heuristics/tasks/intro_heuristics_a)のスケジュール変更、[AHC001](https://atcoder.jp/contests/ahc001/tasks/ahc001_a)の長方形調整です。

両目的・両冷却の800シミュレーションで別の温度式によるオラクルと採否・乱数消費を比較。整数極値、最良解と現在解の分離、温度の極小・極大、非有限値、不正設定、反復0/1、Timerとの接続も検査します。Pythonの固定提案列とも照合。実問題でのスコア改善は測定していません。
