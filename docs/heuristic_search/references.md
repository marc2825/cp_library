# 一次資料・設計判断

[カテゴリ索引](README.md) / [図解](../../visualizations/heuristic_search/index.html)

2026-10-17に確認しました。

| 一次資料 | 参考にした点 |
|---|---|
| [Sebastiano Vigna: SplitMix64参照コード](https://prng.di.unimi.it/splitmix64.c) | 固定増分、混合定数、全seedを許す64ビット生成器。public domainのアルゴリズムに基づきクラス化し、ヘッダにも出典を記載 |
| [M.E. O'Neill: Efficiently Generating a Number in a Range](https://www.pcg-random.org/posts/bounded-rands.html) | 下端の余り部分を棄却して剰余の偏りを除く方法 |
| [C++標準草案: steady_clock](https://eel.is/c++draft/time.clock.steady) | 壁時計時刻の修正に左右されない単調な経過時間 |
| [AtCoder Introduction to Heuristics](https://atcoder.jp/contests/intro-heuristics/tasks_print?lang=ja) | 局所探索、焼きなまし、部分解を複数保持する探索、制限時間内のスコア最適化 |
| [AtCoder Heuristic First-step Vol.1](https://img.atcoder.jp/ahf1-S2wpmcNX/AtCoderHeuristic_FirstStep_Vol1_Extended.html) | seedを固定した近傍選択、時間比率と温度更新、経路改善の擬似コード |
| [AHC021公式コンテストのBeam search解説](https://atcoder.jp/contests/ahc021/editorial/6681) | 層ごとの部分解・評価と幅による選別の適用例 |

## 設計判断

Randomは固定アルゴリズム・seed0を既定にし、非決定的なseedや共有大域状態を避けました。整数の範囲変換とshuffleも固定しています。標準整数だけで実装し、整数の全範囲でも符号付きの加算超過や範囲外キャストに頼りません。

Timerは秒単位のsteady_clock。Clock注入によりテストは実時間を待たずに行います。探索は反復予算を基本にし、stopとSAのprogressで時間予算を指定します。期限は協調的に確認し、厳密な実行時間保証とは区別します。

探索は独立した状態のコピーと純粋な評価を基本とし、undoの失敗を避ける使い方にしました。大きな状態の差分更新や高速な評価更新は利用側の専用実装が適する場合があります。一般性のために状態のハッシュや重複削除、最終状態判定を推測して追加しません。

SAは現在と最良の状態を分け、整数評価をdoubleへ変換してから比較しません。悪化幅は符号なし差分で求めてから確率計算へ変換します。Beamは安定した同点順を持ち、異なる深さのスコアを混ぜません。局所探索は厳密改善だけを採用します。

解品質の改善は単体テストの一致だけでは保証できません。検証は乱数の定義・極値と探索手順の独立シミュレーションに集中し、実問題でのオンラインスコアやACは主張しません。
