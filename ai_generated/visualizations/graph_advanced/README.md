# グラフの残り15項目の図解

[操作画面](index.html) / [API・検証](../../docs/graph_advanced/README.md) / [一次資料](../../docs/graph_advanced/references.md)。JSON編集、再生、前後移動、SVG対象表示と状態表を備えます。外部通信・ビルド不要です。

全般にn<=8、辺数<=16、整数値−100〜100の教材入力。rankは左右<=5/14辺、Eulerは10辺、b-flowは容量−3〜4/組合せ10万以下、数え上げはn<=12、動的操作は30個以下。空グラフは数え上げのみ表示対象とし、C++の空入力対応とは異なります。

| モード | 教材モデル |
|---|---|
| [G-16 Cost Scaling Min-Cost b-Flow](index.html#cost_scaling_b_flow) | 容量内の流量全列挙と保存則・費用。epsilonの操作列は再現しない。 |
| [G-20 少種類重みの最短路](index.html#few_weight_shortest_path) | C++と同じ重み別FIFOで距離と候補列を表示。 |
| [G-21 Shortest Nonzero Path](index.html#shortest_nonzero_path) | 有限巡回加法群だけを単純パス全探索。C++は任意群のFastSUP。 |
| [G-22 Minimum Diameter Spanning Tree](index.html#minimum_diameter_spanning_tree) | 全域木の辺集合を列挙し直径を比較。C++は連続中心。 |
| [G-23 SPQR Tree（分離対探索版）](index.html#spqr_tree) | 分離対とS/P/Rの分割・併合を表示。 |
| [G-24 一般グラフの木幅分解](index.html#tree_decomposition) | 全消去順序で厳密幅を求め、fillとbagを表示。C++はsubset DP。 |
| [G-25 Cographの標準形](index.html#cograph_canonical_form) | union/joinの構造文字列を下から組み立てる。 |
| [G-26 一般グラフ最大重みMatching](index.html#general_weighted_matching) | 全matchingの重みを比較。C++のBlossom回転は再現しない。 |
| [G-27 Rank-Maximal Matching](index.html#rank_maximal_matching) | 全matchingの順位profileを比較。C++は多倍長Hungarian。 |
| [G-28 動的単位容量Max Flow](index.html#dynamic_unit_flow) | 更新後の全カットと最大流値。C++は残余フローを修復。 |
| [G-29 ラベル付き連結グラフ数](index.html#connected_graph_count) | BigIntによる成分サイズの漸化式。 |
| [G-30 強連結有向グラフ数](index.html#strongly_connected_graph_count) | BigIntによるWright漸化式。 |
| [G-31 Euler閉路数（BEST定理）](index.html#euler_circuit_count) | 辺IDのEuler列をDFS全列挙。C++はBESTと行列式。 |
| [G-32 一般グラフの厳密同型判定](index.html#graph_isomorphism) | 頂点置換の全探索。C++は個別化と色精緻化。 |
| [G-33 二端子直並列グラフ分解](index.html#series_parallel_decomposition) | series/parallel縮約の式ノードを表示。 |

`node verify/visualizations/graph_advanced_model_test.js` はC++の15使用例と独立oracleによる1,016トレースを検査。`node verify/visualizations/graph_advanced_browser_test.mjs --browser /path/to/chromium` は15モード、6入力変更、再生・移動・不正入力・リンク・390px表示を確認します。
