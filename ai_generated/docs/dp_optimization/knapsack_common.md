---
title: ナップサックの共通型
documentation_of: //dp_optimization/knapsack_common.hpp
---

# ナップサックの共通型

[カテゴリ索引](README.md) / [0-1版のusage](zero_one_knapsack.md) / [実装](../../dp_optimization/knapsack_common.hpp) / [図解](../../visualizations/dp_optimization/index.html#zero-one)

`KnapsackItem{weight,value}`、`BoundedKnapsackItem{weight,value,count}` と `KnapsackResult` を共有します。重さ・容量はint、価値と個数はlong long。

結果のdpは長さcapacity+1の `vector<optional<long long>>`。初期はdp[0]=0、他はnulloptです。`best_value()`は到達可能な全重さの最大価値をO(W)で求めます。空集合を許すため0以上です。`KnapsackResult(capacity)`自体の構築はO(W)時間・空間。

O(NW)などは正のN,Wでの通常の表記です。空の品物・容量0も含めると、3種類のナップサックは初期化込みO((N+1)(W+1))時間・O(W+1)空間、best_valueはO(W+1)です。

各アルゴリズムは重さちょうどの状態を返すため、負の価値を持つ状態も残します。容量以下の最適値と、重さちょうどを区別してください。LLONG_MIN/MAXを未到達の番兵に使わず、有限値として受け付けます。全ての評価する加算・乗算はlong longの範囲内が条件で、範囲外はoverflow_error。DP関数内の超過時には結果を返しません。

[0-1](zero_one_knapsack.md)は高々1個、[完全](unbounded_knapsack.md)は何個でも（重さ正）、[個数制限](bounded_knapsack.md)はcount以下。入力と個数上限の条件は各説明を参照してください。内部の加算・乗算補助は `detail/arithmetic.hpp` で、グラフの距離用INFとは異なりlong longの最大値も許容します。

使用例・問題・検証は各アルゴリズムの説明にまとめています。[検証入口](../../verify/run_dp_optimization.py)は全dpセルを独立した個数列挙と比較します。オンラインACは未確認です。[参考資料](references.md)。
