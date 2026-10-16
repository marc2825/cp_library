---
title: Timer
documentation_of: //heuristic_search/timer.hpp
---

# Timer

[カテゴリ](README.md) / [実装](../../heuristic_search/timer.hpp) / [使用例](../../examples/heuristic_search/timer.cpp) / [図解](../../visualizations/heuristic_search/index.html#timer) / [一次資料](references.md)

steady_clockの構築時刻から経過した実時間を秒で返します。CPU時間ではありません。有限の時間予算、0秒、制限なしを同じAPIで扱います。

## usage

```cpp
#include <iostream>
#include "heuristic_search/timer.hpp"
int main() {
    cp::Timer timer(1.9); // Seconds, measured from construction by steady_clock.
    std::cout << timer.limit() << '\n';
    // while (!timer.expired()) { ... }  // Cooperative wall-time budget.
    timer.reset(0);
    std::cout << timer.expired() << ' ' << timer.progress() << '\n';
}
```

| API | 内容 |
|---|---|
| Timer(limit_seconds=+∞) | 即座に計測を開始 |
| elapsed()、limit() | 経過秒・設定された制限秒 |
| remaining() | max(0,limit−elapsed) |
| expired() | elapsed≥limit。期限ちょうどでもtrue |
| progress() | [0,1]へ切り詰めたelapsed/limit |
| reset()、reset(seconds) | 予算を保持／指定して開始時刻を現在へ戻す |
| BasicTimer<Clock> | is_steady=trueのClockを注入できるTimerの本体 |

0秒なら開始時点からexpired=true、progress=1。+∞ならexpired=false、remaining=+∞、progress=0。負値・NaN・−∞はinvalid_argumentで、失敗したresetは元の設定を保ちます。ClockはC++のClock要件を満たす単調時計とします。コピーは開始時刻と制限をそのままコピーします。

SA/局所探索/Beamのoptions.stopへ`[&] { return timer.expired(); }`を設定できます。SAのoptions.progressへ`[&] { return timer.progress(); }`を設定すると時間比率で温度を更新します。timerは探索が終わるまで生存させ、入出力時間も含めたい場合はその前に構築します。Timer自体は探索を中断しません。コールバックやソートの時間を考慮して提出先の制限に余裕を設けます。

## 計算量・使いどころ・検証

各操作O(1)時間・O(1)領域。ヒューリスティックの全体予算や各フェーズの予算に使います。問題例は[AtCoder Introduction to Heuristics](https://atcoder.jp/contests/intro-heuristics/tasks/intro_heuristics_a)。

模擬Clockで0秒・期限前・期限ちょうど・期限超過、reset、無限予算、不正値を検証。SAと接続し4ミリ秒相当の4反復を待機せず再現します。実Clockは経過非負のsmoke testだけにし、負荷で不安定になる厳密な実時間比較は行いません。図解も模擬時計です。
