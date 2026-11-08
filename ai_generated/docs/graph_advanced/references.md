# 一次資料・設計判断

確認日: 2026-10-25。G-16・G-20〜G-33の対応範囲は[一覧](README.md)と各APIへ記載しました。

| 資料 | 参照した内容 |
|---|---|
| [OR-Tools min_cost_flow.h](https://github.com/google/or-tools/blob/stable/ortools/graph/min_cost_flow.h) / [LEMON CostScaling](https://github.com/seqan/lemon/blob/master/include/lemon/cost_scaling.h) | epsilon最適性、push/relabel、実行可能性の事前確認、整数費用のスケーリング。G-16はアルゴリズムの説明から独立実装し、両ライブラリのコードは取り込まない。 |
| [Iwata–Yamaguchi: Finding a Shortest Non-zero Path in Group-Labeled Graphs](https://arxiv.org/abs/1906.04062) | v6、§5 FastSUPとLemma 5.3の経路復元。任意群、逆向きの逆元、単純パスとwalkの区別。群演算・heap・DSUを本API向けに実装。 |
| [Hassin–Tamir: On the minimum diameter spanning tree problem](https://www.tau.ac.il/~hassin/diameter_95.pdf) | absolute 1-centerと最小直径木の関係。こちらは全点対距離と辺ごとの切替点整列、半整数の2倍表現を採用。 |
| [OGDF SPQRTree](https://ogdf.github.io/doc/ogdf/classogdf_1_1_s_p_q_r_tree.html) | S/P/R skeleton、実辺と仮想辺の対、Q省略の規約を比較。OGDFコードは取り込まず、線形時間実装の性能も主張しない。 |
| [Boost Graph固定版](https://github.com/boostorg/graph/blob/54a94911e68dccf5812b32d018566566700499f3/include/boost/graph/maximum_weighted_matching.hpp) | Boost 1.89の重み付きBlossom context。O(V³)・O(V+E)、多重辺を事前に集約する条件、4倍費用の範囲。 |
| [Irving et al.: Rank-maximal matchings](https://eprints.gla.ac.uk/13998/) | 順位ごとの本数の辞書式最大化という目的。本実装は多倍長位取り+Hungarianで解き、この論文の順位別グラフ更新法ではない。 |
| [Archer–Gessel–Graves–Liang: Counting acyclic and strong digraphs by descents](https://arxiv.org/abs/1909.01550) | §2.2のWrightの漸化式(2)(3)、ループなし・逆向き辺を同時に許すグラフの定義と小さい係数。 |
| [Jerrum: Two good counting algorithms](https://webspace.maths.qmul.ac.uk/m.jerrum/ETHbook/chapter1.pdf) | 有向Matrix TreeとBESTの計数。根・先頭辺固定と、辺列の回転の規約を区別。 |

## 取り込みとライセンス

G-26だけ外部C++ coreを取り込み、その他はアルゴリズムの定義と既存部品に基づく独立実装です。

`graph_flow/detail/weighted_blossom.hpp` はBoost Graph commit `54a94911e68dccf5812b32d018566566700499f3`（boost-1.89.0）の `maximum_weighted_matching_context` を保持し、公開wrapper・brute-force・入力検査ヘルパーを除いてnamespaceを `cp::weighted_blossom_detail` に隔離しています。上流の著作権表示（Yi Ji / Joris van Rantwijk）と[Boost Software License 1.0本文](../../verification/licenses/Boost-1.0.txt)を保存。[出典台帳](../../verification/provenance.json)に元ファイルhash、固定版、変更点とローカルhashがあります。NDEBUG時の警告対策としてassert専用ループ変数にmaybe_unusedを付け、coreのアルゴリズムは変更していません。

環境のBoost 1.74の旧matchingは試験でblossomの循環所有によるLSanリークを検出したため採用せず、所有権を整理した固定版へ置き換えました。基本グラフ型はシステムBoost 1.74と組み合わせて検証します。BigIntは既存のBoost.Multiprecision依存を共有します。追加のリンクライブラリは不要です。

少種類重みのFIFO、消去集合の木幅DP、cographのunion/join文字列、単位容量フロー修復、連結成分の数え上げ、同型の個別化と精緻化、二端子SP縮約は、各ページに証明の要点と計算量を記載しています。図解は独立の教材モデルで、C++の高速coreの実行トレースではありません。
