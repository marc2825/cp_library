# 一次資料・設計判断

確認日: 2026-10-16。以下の一次資料を読み、既存カテゴリと同じC++17・namespace cp向けに独自実装しました。外部のコードや説明文を複製して取り込んではいません。

| 資料 | 確認した点と設計 |
|---|---|
| [Luzhiled: Longest Increasing Subsequence](https://ei1333.github.io/library/dp/longest-increasing-subsequence.hpp.html) | 末尾の更新と狭義／広義の境界。新APIでは長さ・添字の復元・各位置で終わる長さを返し、入力を変更しない |
| [CP-Algorithms: LIS](https://cp-algorithms.com/dynamic_programming/longest_increasing_subsequence.html) | 長さごとの末尾・前駆による復元。値の番兵や符号反転は不要とし、任意の比較器を使う |
| [CP-Algorithms: Knapsack](https://cp-algorithms.com/dynamic_programming/knapsack.html) | 0-1／完全の走査方向、個数制限の剰余類・単調キュー。重さちょうどの到達性をoptionalで分離し、負値・重さ0にも仕様を付ける |
| [CP-Algorithms: Submask Enumeration](https://cp-algorithms.com/algebra/all-submasks.html) | (sub−1)&mask、空集合の終了処理とO(3^N)。uint64_tで最上位ビットにも対応 |
| [AOJ ALDS1_10_C](https://judge.u-aizu.ac.jp/onlinejudge/description.jsp?id=ALDS1_10_C) / [DPL_1_E](https://judge.u-aizu.ac.jp/onlinejudge/description.jsp?id=DPL_1_E) | LCS・単位費用の編集距離の入出力。長さのみの省メモリ版と、添字／編集操作の復元を分ける |
| [AOJ DPL_1_B](https://judge.u-aizu.ac.jp/onlinejudge/description.jsp?id=DPL_1_B) / [DPL_1_C](https://judge.u-aizu.ac.jp/onlinejudge/description.jsp?id=DPL_1_C) / [DPL_1_G](https://judge.u-aizu.ac.jp/onlinejudge/description.jsp?id=DPL_1_G) | 価値・重さ・個数の入力順を確認し、3種類のジャッジ形式を用意。公開APIの品物は重さ・価値の順 |
| [CSES Money Sums](https://cses.fi/problemset/task/1745/) | 各品物を1回使う部分和の列挙。Bitsetは件数でなく到達可能性を保持し、語をまたぐシフトを実装 |
| [EDPC N - Slimes](https://atcoder.jp/contests/dp/tasks/dp_n) | 隣接結合の区間DP。cost(l,k,r)を指定する二分割テンプレートと結合順の復元 |
| [EDPC S - Digit Sum](https://atcoder.jp/contests/dp/tasks/dp_s) | 長い上限の十進入力と法を取る個数。汎用版は状態・開始前後のフラグ・拒否遷移・受理条件・Count型を指定 |
| [AOJ DPL_2_A](https://judge.u-aizu.ac.jp/onlinejudge/description.jsp?id=DPL_2_A) | 訪問済み集合と終点のDP、閉じる辺と不存在。辺なしはnullopt、費用の番兵を使わずに巡回路を復元 |
| [AOJ DPL_3_C](https://judge.u-aizu.ac.jp/onlinejudge/description.jsp?id=DPL_3_C) / [DPL_3_B](https://judge.u-aizu.ac.jp/onlinejudge/description.jsp?id=DPL_3_B) | ヒストグラムと0の長方形。既存の3.4単調スタックを再利用し、面積に加えて半開の座標を返す |

## 検証と旧API

既存の `DP/LIS.hpp`、`DP/subset_dp.hpp`、`DP/TSP.hpp`、`misc/Largest_Rectangle.hpp` は保持します。旧LISと長方形はテストでも比較します。既存の貼り付けスニペットを自己完結したヘッダとして使うことは想定せず、新規コードは `dp_optimization/` を利用してください。

C++の検証は更新式をそのまま書き直すだけで済ませず、部分列・品物個数・隣接結合順・巡回順列・全長方形などの独立した探索と比較します。編集距離は実際の文字列間を編集するBFS。図解モデルも更新途中の表、個数制限の候補、十進接頭辞の全列挙、各集合状態を照合します。

PROBLEMは対応する入出力形式を示すものです。オンラインACは未確認です。全てローカルで検証します。

追加A項目の一次資料・使用問題と設計判断は[3.12〜3.15補完の資料一覧](../math_a/references.md)にまとめています。

3.12〜3.15のB追加分は[一次資料・設計判断](../b_12_15/references.md)を参照してください。
