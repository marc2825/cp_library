# ヒューリスティック探索E・構文解析X

[探索カテゴリ](../heuristic_search/README.md) / [構文解析カテゴリ](../parsing/README.md) / [11モードの図解](../../visualizations/search_parsing_extensions/index.html) / [一次資料](references.md)

E-01〜E-06・E-20〜E-21の8項目とX-01〜X-03の3項目を整備しました。E-07〜E-19の行はありません。E-21のAlias法・重み付き非復元抽出・正規/指数/切断指数分布はユーザー承認済みの範囲です。C++17・namespace cp、0始まり/半開区間、Eはheuristic_search/all.hpp、Xはparsing/all.hppで公開します。

| ID | API・説明 | 主な用途 |
|---|---|---|
| E-01 | [汎用Undo Manager・スコア検算](undo_manager.md) | 複合近傍の棄却、バックトラック中の履歴、差分scoreの開発時検算。 |
| E-02 | [IndexSet・一様ランダム選択](index_set.md) | 候補集合の追加削除、未使用頂点からの一様選択。 |
| E-03 | [TSPの2-opt・Or-opt](tsp_neighborhood.md) | 巡回セールスマン問題の局所改善・焼きなまし。 |
| E-04 | [SA受理判定の対数テーブル](fast_annealing_acceptance.md) | 悪化moveの採否がボトルネックになる固定回数SA。 |
| E-05 | [差分更新型Beam Search](delta_beam_search.md) | 状態が大きく、各moveの適用/取消しが小さい構築型探索。 |
| E-06 | [Chokudai Search](chokudai_search.md) | 時間内に探索の候補を増やし、深さ別に良い状態を保持する探索。 |
| E-20 | [多点並列探索・並列Multi-Start SA](parallel_search.md) | 独立した開始点・seedでのSAや探索の並列実行。 |
| E-21 | [Alias法・重み付き抽出・連続分布](distribution_sampling.md) | 静的な非一様候補抽出、候補の非復元抽出、近傍生成の連続乱数。 |
| X-01 | [再帰下降parser・構文木](expression_parser.md) | 整数式の評価、構文木を用いる独自の評価器・式変換。 |
| X-02 | [DFA/NFAの共通操作](automata.md) | 禁止パターンの受理条件、言語の積・補集合、桁DPの条件構築。 |
| X-03 | [正規表現からNFA・Thompson構成](regex_nfa.md) | 小さい正規言語の認識、正規表現条件を満たす文字列のDP。 |

## 共通仕様・既存機能との共有

探索Scoreは既存SearchCommonに従うbool以外の64bit以下の算術型です。差分は符号付き型、long doubleは受理温度・許容誤差の計算に使います。

Undoと差分探索は逆操作のnoexcept・参照先の寿命を契約にします。scoreの有限性/overflow、最小化/最大化、停止callback、Random、既存SA/Beam、数値許容誤差、digit_dp、BigIntを共有しています。従来のsimulated_annealing_deltaは同じexp採否・乱数順序を保ち、採否callbackを差し替える入口を追加しました。並列探索は実際にstd::threadを使用し、コンパイル・リンクに-pthreadが必要です。

整数Aliasは確率境界の丸めを避けます。連続分布と対数テーブルは有限精度の近似です。最小DFAはMoore法で、決定化・積には状態上限があります。構文木評価は反復式、parserの再帰には上限があります。探索・状態数・指数の予算は任意callbackやBigIntの1演算時間までは制限しません。

旧parsing_template.hpp/parse_tree.hppは問題専用mainと重複定義を除き、新しい式parserへのinclude入口にしました。旧グローバル関数/変数は新APIへ移行が必要です。[移行先と演算規則](expression_parser.md)を参照してください。

## 検証

```sh
make verify-search-parsing-extensions
python3 -m tools.cp_tools.suite --runners search_parsing_extensions
python3 -m tools.cp_tools.suite --stdlib-debug --runners search_parsing_extensions
python3 -m tools.cp_tools.suite --sanitize --runners search_parsing_extensions
node verify/visualizations/search_parsing_extensions_model_test.js
node verify/visualizations/search_parsing_extensions_browser_test.mjs --browser /path/to/chromium --screenshot /tmp/search-parsing.png
```

単独/重複include・全カテゴリの2翻訳単位・11使用例/文書内C++/図解参照値を確認します。C++はDebug/NDEBUGでUndoの例外・stale snapshot、IndexSet対set、TSP全近傍の再計算、受理確率境界、既存Beamとの比較、Chokudaiの全探索、1/2/4workerと逐次SAの一致・同時実行・join、Aliasの確率質量の完全一致、分布のモーメント・乱数消費、式の評価/失敗位置、独立NFA探索・最小DFAの状態識別・std::regexとの言語比較を行います。

Pythonは別実装の配列・集合・heap・任意精度式・状態×入力位置の探索・正規表現・SplitMix64からの変換と870入力を比較します。図解モデル・Chromium検証は入力変更、全モードの移動/再生/失敗からの復帰、390px幅を確認します。[対数テーブルの実測](benchmark.md)は採否率・乱数消費・前計算を含む条件を記録します。オンラインACとGitHub Actions上の実行は未確認です。
