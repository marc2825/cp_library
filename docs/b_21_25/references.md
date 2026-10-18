# 一次資料・設計判断

2026-10-18に参照。標準ライブラリと本リポジトリ内の明示的なincludeを使います。出典側のAC表示を、本リポジトリの提出・ACとして扱いません。

- [Bannaiほか: The Runs Theorem](https://arxiv.org/abs/1406.0263)、[hitonanode: Lyndon文字列とRuns](https://hitonanode.github.io/cplib-cpp/string/lyndon.hpp.html)：二つの文字順のLyndon根を伸ばす列挙。今回は次にsuffix順位が小さい位置と、厳密なSA/LCP RMQのLCE/LCSを使い、構築と整列を含むO(N log N)です。
- [Library Checker: Runs仕様](https://github.com/yosupo06/library-checker-problems/blob/master/string/runenumerate/task.md)、[Wildcard仕様](https://github.com/yosupo06/library-checker-problems/blob/master/string/wildcard_pattern_matching/task.md)：ローカルの入出力と極大・最小周期の定義。Wildcardは固定長1byteの照合で、今回は非負の不一致式Σab(a−b)²と正確な整数畳み込みを使います。
- [Rubinchik–Shur: EERTREE](https://arxiv.org/abs/1506.04862)：回文の差分をまとめるseries linkと最小分割のDP。今回の版は最小値の起点も保持して復元します。
- [Auer–Cesa-Bianchi–Fischer: Finite-time Analysis of the Multi-armed Bandit Problem](https://www2.compute.dtu.dk/pubdb/pubs/2088-full.html)：UCB1の出典。任意の非定常な探索報酬に理論保証が移るとは扱いません。
- [KACTL: Minimum Enclosing Circle](https://github.com/kth-competitive-programming/kactl/blob/main/content/geometry/MinimumEnclosingCircle.h)：ランダム増分法の設計比較。今回の版はlong double、固定seed、空入力、共線の3点と非有限値を明示します。
- [AOJ CGL一覧](https://onlinejudge.u-aizu.ac.jp/solutions)：円と直線・円同士の交点の問題への入口。[CGL_7_D](https://onlinejudge.u-aizu.ac.jp/problems/CGL_7_D)、[CGL_7_E](https://onlinejudge.u-aizu.ac.jp/problems/CGL_7_E)の形式をローカル比較に使用します。
- [Matroid Intersectionの講義ノート](https://www.extremalcombinatorics.com/optimization/notes/sec_matroidIntersection.html)：独立性オラクル、交換グラフ、最短増加路と貪欲法の適用範囲。今回は非重み付き交差、Graphic/Partitionのオラクル、1マトロイドの重み付き貪欲法を公開します。
- [hitonanode: Linear Matroid Parity](https://hitonanode.github.io/cplib-cpp/combinatorial_opt/linear_matroid_parity.hpp.html)：交代行列のrankにより最大対数を表す代数的な方法。今回の版はrankの再計算による削除法で、資料の高速な逆行列更新の実装はコピーしていません。全試行回数を有限にし、候補の独立性を検査。小さい入力には別の決定的厳密DFSを用意しました。代数版の最大性はMonte Carloです。

## リポジトリ内の共有先

- 動的ハッシュは[ImplicitTreap](../../ordered_structure/implicit_treap.hpp)の順方向・逆方向集約を共有し、[静的RollingHash](../../string_algorithm/rolling_hash.hpp)と同じbyte+1・base・二つの法に揃えます。衝突を消したとは説明しません。
- Suffix Treeは既存SA/LCPの静的な圧縮trie。回文分割は既存Eertreeを利用し、差分とseries linkを別に保持。複数文字列比較・Runsは共通のLCE内部型を共有します。
- 最遠点対は既存凸包とキャリパー、Manhattan MSTは3.7の実体、半平面交差は既存の有理数/浮動小数点の凸切断。半平面交差は利用者が明示した矩形内の逐次切断でO(H²)、無限領域の分類やO(H log H)のdeque法は実装範囲に含めません。
- Nimberは既存の64bit nim積。グラフゲームは通常ルールのwin/lose/drawをretrogradeで分類し、DAG専用Grundyと区別。Partizanは有限short gameの和・負号・再帰的な部分順序をmemo化し、正規形の自動簡約はしません。
- Multi-Startと近傍管理付きSAは既存のRandom・SearchResult・目標方向・悪化幅・温度計算を共有。温度較正、paired seedの設定比較、UCB1/epsilon-greedy、近傍の重み選択と観測はそれぞれ独立した公開APIです。スコア改善・最適性・実時間の厳密上限は保証しません。

CのWeighted Matroid Intersection、Frobenius/高度なゲーム値、多点並列探索や高度な文字列周期問題へ範囲を広げません。Python版の作成計画も保持し、この追加はC++版です。
