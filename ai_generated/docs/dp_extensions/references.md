# P追加項目の一次資料・設計判断

2026-10-25〜26確認。今回の14項目は理論・公開仕様を参照した独立実装です。外部実装のソースは取り込んでいません。既存のBigInt、SMAWK、DP安全演算と品物型を共有します。

| 項目 | 一次資料と採用した内容 |
|---|---|
| Frontier DP | [Kawahara: frontier-based searchの例](https://github.com/junkawahara/frontier_basic_tdzdd)。境界の次数/連結状態と単一路・閉路の完成条件を参考にしました。本実装はZDDを構築せずmapで等価状態の個数を合算し、Hamiltonian条件を次数で指定します。 |
| Bit-parallel LCS | [Allison–Dix (1986), 著者による説明](https://www.cantab.net/users/mmlist/ll/Publications/1986IPL/)。差分bit列、shift・減算borrowのword間伝播。 |
| Myers | [Myers (1999), 論文](https://www.gersteinlab.org/courses/452/09-spring/pdf/Myers.pdf)。§4.2のblock境界の水平差分を用い、単位費用のglobal alignmentとして実装しました。 |
| Exact Cover | [Knuth, Dancing Links](https://arxiv.org/abs/cs/0011047)。最小列選択、cover/uncover、primary/secondary。節のコードを転記せず、int添字の所有vectorで構成しました。 |
| A* | [Hart–Nilsson–Raphael (1968)](https://ai.stanford.edu/~nilsson/OnlinePubs-Nils/PublishedPapers/astar.pdf)。許容下界による最適性。非整合heuristicでも改善時の再展開と不変親履歴を許容します。 |
| IDA* | [Korf (1985)](https://www.sciencedirect.com/science/article/pii/0004370285900840)。最小の閾値超過に従う反復探索。経路上の循環除去と明示stackへ適用します。 |
| SAT | [Eén–Sörensson, An Extensible SAT-solver](https://research.chalmers.se/en/publication/74325) / [著者PDF](https://minisat.se/downloads/MiniSat.pdf)。watched literalsの伝播を参考に、DPLLを独立実装。本実装はCDCLの学習節を持ちません。 |
| Monge積 | [Aggarwal–Klawe–Moran–Shor–Wilber](https://doi.org/10.1145/10515.10546)。全単調行列のargminを既存SMAWKで処理します。任意A×Monge Bの密な長方形出力が今回の範囲です。 |

双方向BFSは層全体の探索と最短合流、復元Knapsackは剰余類単調queueと段階別選択表、解数は添字解集合の和/差、重み付きSlope Trickは既存の単位hingeを重み付きmapへ拡張しました。動的DPはmin-plus行列の結合、少種類重量Knapsackは降順価値prefixの凹性と単調argmaxを使います。これらはAPI条件と計算量を[個別説明](README.md)で定義しています。

Countは既定BigIntです。LCSの数は異なる値列数と一致しないため、添字pair列であることを関数名と説明に明記しました。P-20〜P-23は名称だけでは対象が定まらなかったため、密なMonge積、重み付きhinge、固定状態数の遷移列更新、少種類の重量へ具体化しました。Python版への移植は別計画です。
