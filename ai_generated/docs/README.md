# 説明・使用例・インタラクティブ解説の構成

[競技での使い方](usage.md) / [グラフ・木のガイド](graph_tree.md) / [ローカル検証](development/testing.md)

[実行・観察ツール V-05〜V-07](development_runtime/README.md) / [3モードの図解](../visualizations/development_runtime/index.html)。`make verify-development-runtime`。

[開発・検証基盤 V-01〜V-04](development/README.md) / [4モードの図解](../visualizations/development/index.html)。`make verify-development`。

[F-12〜F-17のAPI・使用例・検証](foundation_tools/README.md) / [6モードの図解](../visualizations/foundation_tools/index.html)。`make verify-foundation-tools`。

[F-11：多倍長整数のAPI・使用例・性能・検証](basic_utility/big_integer.md) / [4モードの図解](../visualizations/big_integer/index.html)。`make verify-big-integer`。Boostヘッダが必要です。

[共通基盤 F-08〜F-10のAPI・使用例・検証](foundation_structures/README.md) / [5モードの図解](../visualizations/foundation_structures/index.html)。`make verify-foundation-structures`。

[共通基盤 F-01〜F-07のAPI・使用例・検証](basic_utility/foundation.md) / [7モードの図解](../visualizations/foundation/index.html)。`make verify-foundation`。

[3.21〜3.25 B全28項目のAPI・検証](b_21_25/README.md) / [28モードの図解](../visualizations/b_21_25/index.html)。`make verify-b-21-25`。

[3.16〜3.20 B全31項目のAPI・比較検証](b_16_20/README.md) / [31モードの図解](../visualizations/b_16_20/index.html)。`make verify-b-16-20`。

[3.7〜3.11 B全36掲載箇所の整備・検証](b_7_11/README.md) / [36モードの図解](../visualizations/b_7_11/index.html)。`make verify-b-7-11`。

`3.xx` ごとに1つの独立したカテゴリフォルダを用意します。説明はコードと同名の Markdown として、`docs/<カテゴリ>/` に配置します。

```text
union_find/                 # 3.2 の実装
docs/union_find/            # 同じ名前の .md とカテゴリ索引
visualizations/union_find/  # 図を操作できるアルゴリズム解説
examples/union_find/        # コンパイルして動かす使用例
verify/union_find/          # 差分テスト・ジャッジ形式のプログラム
```

以降の各アルゴリズムの説明には、次を揃えます。

1. 何を解くか、使用条件、API。
2. 最小の usage と動く使用例。
3. 構築・操作・メモリの計算量。
4. 使用する問題へのリンク、検証済み範囲。
5. 原理・境界条件・実装上の注意点。
6. 図解を組み合わせたインタラクティブな解説。
7. 参考にした一次資料。

ヘッダ内には短い用途・重要な契約・説明への場所だけを残し、長い説明は移します。実装を Markdown に複製しません。`documentation_of: //カテゴリ/ファイル.hpp` により、公開用ドキュメント生成時に説明と実装を紐付けられます。

インタラクティブ解説の実装言語は問いません。`visualizations/` に独立して置き、各説明・カテゴリ索引・[共通の解説一覧](../visualizations/index.html) から参照できるようにします。現状の HTML はローカルで直接開ける形式です。公開時には `visualizations/` と実装・Markdown を同じルート構成で配置します。

[3.1〜3.6 A項目の補完・検証](a_priority/README.md) / [追加の32モード図解](../visualizations/a_priority/index.html)

[3.7〜3.11 A項目の補完・検証](graph_a/README.md) / [追加の14モード図解](../visualizations/graph_a/index.html)

3.16〜3.20のAは全29掲載箇所が実装済みです。[不足9掲載箇所の補完・検証](transform_a/README.md) / [追加の8モード図解](../visualizations/transform_a/index.html)。`make verify-transform-a` / `python3 verify/run_transform_a.py --sanitize`。

3.21〜3.25のAは全25掲載箇所が実装済みです。[不足10項目の補完・検証](final_a/README.md) / [追加の10モード図解](../visualizations/final_a/index.html)。`make verify-final-a` / `python3 verify/run_final_a.py --sanitize`。

[3.1〜3.6 B全43掲載箇所の整備・検証](b_1_6/README.md) / [42モードの図解](../visualizations/b_1_6/index.html)。

## カテゴリ索引

[高度なアルゴリズム20項目：追加・既存実装の対応表](advanced/README.md) / [追加分の図解](../visualizations/advanced/index.html)

- [3.1 基本ユーティリティ・探索](basic_utility/README.md)
- [3.2 Union-Find・連結性管理](union_find/README.md)
- [3.3 セグメント木・Fenwick Tree](range_query/README.md)
- [3.4 静的クエリ・順序統計・集合管理](static_query/README.md)
- [3.5 平衡二分探索木・永続データ構造](ordered_structure/README.md)
- [3.6 オフラインクエリ・二次元データ構造](offline_query/README.md)
- [3.7 グラフ：基本探索・最短路・MST](graph_basic/README.md)
- [3.8 グラフ：連結成分・構造分解](graph_connectivity/README.md)
- [3.9 グラフ：フロー・マッチング・最適化](graph_flow/README.md)
- [3.10 グラフ：組合せ構造・数え上げ](graph_combinatorics/README.md)
- [3.11 木アルゴリズム](tree/README.md)
- [3.12 DP・最適化](dp_optimization/README.md)
- [3.13 数論：基本整数演算](number_theory_basic/README.md)
- [3.14 数論：乗法的関数・高度な整数問題](number_theory_convolution/README.md)
- [3.15 組合せ論・数え上げ](combinatorics/README.md)
- [3.16 畳み込み・高速変換](convolution/README.md)
- [3.17 多項式・形式的冪級数（FPS）](fps/README.md)
- [3.18 線形漸化式・母関数](linear_recurrence/README.md)
- [3.19 集合関数・Bitmask変換](set_function/README.md)
- [3.20 線形代数](linear_algebra/README.md)
- [3.21 文字列アルゴリズム](string_algorithm/README.md)
- [3.22 計算幾何](computational_geometry/README.md)
- [3.23 組合せ最適化・マトロイド](combinatorial_optimization/README.md)
- [3.24 ゲーム理論](game_theory/README.md)
- [3.25 探索・ヒューリスティック](heuristic_search/README.md)
- [構文解析・有限オートマトン](parsing/README.md)
- [既存のグラフ・木のガイド](graph_tree.md)

既存ドキュメントは順次この構成へ揃えます。

[3.12〜3.15 A項目の補完・検証](math_a/README.md) / [追加の18モード図解](../visualizations/math_a/index.html)

[3.12〜3.15 B全30掲載箇所の整備・検証](b_12_15/README.md) / [30モードの図解](../visualizations/b_12_15/index.html)。`make verify-b-12-15`。

- [D-20〜D-35: 動的・永続・多次元データ構造16項目](data_structure_advanced/README.md)（使用例・比較検証・操作できる図解）。

## 木の追加項目

[T-01〜T-03・T-20〜T-23のAPI・検証](tree_extensions/README.md) / [7モードの図解](../visualizations/tree_extensions/index.html)。木の中心・Prüfer符号・根変更区間と、link/cutを伴うTop Tree・lazy・部分木集約・木DPを追加しました。`make verify-tree-extensions`でまとめて検証できます。

[グラフ追加G-01〜G-15のAPI・検証](graph_extensions/README.md)を追加しました。区間辺・被覆・カット木・st順序・平均閉路・次数列・暗黙/増分MST・なもり・反鎖・辺彩色・差分制約・負閉路・最小費用素パスを扱います。

[G残り15項目のAPI・検証](graph_advanced/README.md)を追加しました。Cost Scaling、少種類重み/群ラベル最短路、最小直径木、SPQR、一般木幅、cograph標準形、重み付き/順位matching、動的単位容量フロー、連結/強連結/Euler計数、厳密同型、二端子直並列分解を扱います。`make verify-graph-advanced`で検証できます。

[P全14項目のAPI・検証](dp_extensions/README.md)を追加しました。Frontier・bit並列LCS/Myers・Exact Cover・A*/IDA*/双方向BFS・SAT・ナップサック復元と個数・Monge積・重み付きSlope Trick・動的DP・少種類重量Knapsack。[14モードの図解](../visualizations/dp_extensions/index.html)。`make verify-dp-extensions`。

[N全19項目のAPI・検証](number_theory_extensions/README.md)を追加しました。区間篩・位数・多項式Floor Sum・逆元・巨大二項係数/階乗・再利用BSGS・mod根・Gaussian整数・連分数/Pell/二平方数和・商prefix・素数和・Min_25・重み付きmod和。[19モードの図解](../visualizations/number_theory_extensions/index.html)。`make verify-number-theory-extensions`。

## 組合せ C全8項目

[C全8項目のAPI・検証](combinatorics_extensions/README.md)。q二項係数、上下境界付き列、小素数法Stirling、ラベルなし木、RSK、Young盤、Hook-Length、Lah/Euler変換と逆変換。[8モードの図解](../visualizations/combinatorics_extensions/index.html)。`make verify-combinatorics-extensions`。
