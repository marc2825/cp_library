# 3.7〜3.11 B：一次資料・設計判断

2026-10-18にアルゴリズムの仕様・実装例を確認しました。外部ライブラリへの実行依存はありません。[対応表](README.md) / [図解と簡略化](../../visualizations/b_7_11/README.md)。

| 一次資料 | 参照した内容・このライブラリでの判断 |
|---|---|
| [Luzhiled Directed MST](https://ei1333.github.io/library/graph/mst/directed-mst.hpp) | 縮約と復元。今回はheap版の複製をせず、各段で全辺を調べるO(VE)版。全層を保存する領域も明記。 |
| [KACTL ManhattanMST](https://github.com/kth-competitive-programming/kactl/blob/main/content/geometry/ManhattanMST.h) | chilli / Takanori Maeharaの4方向sweep。CC0。ヘッダに帰属とURLを記載し、座標変換は128bitへ拡張。 |
| [Luzhiled Dominator Tree](https://ei1333.github.io/library/graph/others/dominator-tree.hpp) | Lengauer–Tarjanの半支配頂点・link/eval。DFSとevalは非再帰に構成。 |
| [Luzhiled 三重辺連結成分](https://ei1333.github.io/library/graph/connected-components/three-edge-connected-components.hpp) / [ライセンス](https://github.com/ei1333/library/blob/master/LICENSE) | Unlicenseのpath absorptionを移植。橋の前処理、非再帰化、pathのmoveを追加。小多重グラフでは最大流3以上という定義と比較。 |
| [hitonanode DM分解](https://hitonanode.github.io/cplib-cpp/graph/dulmage_mendelsohn_decomposition.hpp.html) | 左右余剰領域と均衡SCCの仕様確認。実装は既存Hopcroft–KarpとSCCを共有。 |
| [maspypy 二部グラフ辺彩色](https://maspypy.github.io/library/graph/bipartite_edge_coloring.hpp) | Δ色という仕様の確認。今回は正則化・分割版ではなく、2色交互路の版を独立に実装。 |
| [Library Checker Minimum Cost b-flow仕様](https://github.com/yosupo06/library-checker-problems/tree/master/graph/min_cost_b_flow) | 下限・上限、流出−流入、双対potentialの出力条件。負費用辺の飽和とSSPを使用。容量依存のため同問題の全制約に対する時間保証・ACは主張しない。 |
| [maspypy Static Top Tree](https://maspypy.github.io/library/graph/ds/static_toptree.hpp.html) / [hitonanode Static Top Tree](https://hitonanode.github.io/cplib-cpp/data_structure/static_toptree.hpp.html) | Point/Pathとrake/compressの役割を確認。この版は各列を中点分割し、高さO(log²V)の上界。辺・頂点のID対応と森を扱う。 |
| [maspypy Dynamic Rerooting Tree DP](https://maspypy.github.io/library/graph/ds/dynamic_rerooting_tree_dp.hpp.html) | 固定topologyで更新し任意根の集約を得る仕様。ここでは外側Contextを明示的に伝える独自APIと距離和Specを用意。 |
| [Tarjan, Dynamic Trees as Search Trees via Euler Tours](https://www.cs.princeton.edu/research/techreps/203) | Euler列の回転・連結・切断から動的な森を表現する考え方。自己tokenの可換集約をimplicit treapへ実装。 |
| [Library Checker 木の距離分布](https://github.com/yosupo06/library-checker-problems/tree/master/tree/frequency_table_of_tree_distance) | 異なる頂点の非順序対を距離ごとに数える出力条件。重心のall²−sub²と既存の整数畳み込みを共有。 |

MST感度解析はcut/cycle交換則とHLD、幅2木分解は次数2消去とfill、chordal判定はMCSと完全消去順序、彩色多項式は独立集合分割DP、Steiner木はDreyfus–Wagner法を実装しています。簡易同型の1-WLは未確定を明示し、木の同型は辞書で子コードを完全一致させます。確率ハッシュ版と混同しません。

外部のソースをそのまま同梱せず、API条件と計算量はこのリポジトリの実装に合わせています。共有できるLink-Cut Tree・Bron–Kerbosch・MergeTree・Assignment・SCC・Dinic・HLD・Fenwick・convolutionは既存実体を使います。外部資料のAC表示を本実装の検証実績として数えません。
