# 説明・使用例・インタラクティブ解説の構成

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

## カテゴリ索引

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
- [既存のグラフ・木のガイド](graph_tree.md)

既存ドキュメントは順次この構成へ揃えます。

開発手順・現状・新しいセッションへの依頼テンプレートは [引き継ぎ書](../HANDOFF.md) にまとめています。
