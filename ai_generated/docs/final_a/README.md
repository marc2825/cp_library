# 3.21〜3.25 A項目の補完

3.21〜3.25の不足10項目を補完し、対象のA全25掲載箇所が実装済みです。3.22・3.23、Aho–Corasick・Eertree、ゲームの直和、Hill Climbingは既存実装を共有します。10公開ヘッダ・10個別説明・10使用例を追加しました。

[実装予定リスト](../implementation_plan.md#3-21) / [操作できる10モード](../../visualizations/final_a/index.html) / [一次資料](references.md)

| 項目 | 公開API |
|---|---|
| [Suffix Automaton](../string_algorithm/suffix_automaton.md) | append、所属・初出・出現数・異なる部分文字列数、状態のconst参照 |
| [Lyndon Factorization](../string_algorithm/lyndon_factorization.md) | lyndon_factorization、unsigned byte順の半開区間 |
| [Longest Common Substring](../string_algorithm/longest_common_substring.md) | 共通SAMで最長の連続区間を復元、同長時の規則あり |
| [2D Rolling Hash](../string_algorithm/rolling_hash_2d.md) | 二つの素数法、矩形get/equal、空の矩形、形とbaseを検査 |
| [Nim Product](../game_theory/nim_product.md) | uint64全域のnim_product、8bit表と二次拡大 |
| [周期的Grundy](../game_theory/periodic_grundy.md) | 観測列の候補検出、有限引き算ゲームの周期証明と巨大添字 |
| [Zobrist Hash](../heuristic_search/zobrist_hash.md) | 固定seedの位置×値の表、hashとchange |
| [Tabu Search](../heuristic_search/tabu_search.md) | 訪問keyの期限、aspiration、悪化手、最良/現在解 |
| [Beam重複除去](../heuristic_search/beam_search_unique.md) | 各深さのkeyごとに最良を残し、その後に幅を制限 |
| [差分スコア](../heuristic_search/incremental_score.md) | try_scored_moveとsimulated_annealing_delta、apply/undo |

## 条件と設計

- 文字列は任意のbyteを扱い、入力へ番兵を混ぜません。SAMの出現数キャッシュは追加で無効化し、cloneと重複出現を扱います。LCSは連続部分文字列です。
- 2D Rolling HashとZobristには衝突があります。同じhashだけでは厳密な等値判定になりません。Beam/Tabuのkeyのhash衝突はequalで比較しますが、64bit Zobrist値自体をkeyにした場合の衝突は利用側が管理します。
- Nim Productは普通の整数積ではなく、nim加算=XORと組み合わせる積です。有限体の逆元・除算は今回のA項目の対象外です。
- 一般の観測列の周期候補を、未観測Grundy値の証明として扱いません。固定の有限引き算ゲームは最大手M個の同一窓でのみ外挿を許可し、検出できなければ指定項数で停止します。
- 既存beam_searchは重複を保持します。新しいbeam_search_uniqueは同じ深さだけで統合し、全候補の評価後に最良代表を選びます。
- Tabuは訪問状態のkeyを禁止します。tenure=0も可、期限を超えて再訪した場合の履歴更新とaspirationを扱います。品質や最適性は保証しません。
- 差分スコアはsignedの同じScore型を使い、加算前に範囲・有限性を検査します。applyは強い例外保証、undoは正確かつ例外を投げずに戻せることが必要です。

## 検証

```sh
python3 verify/run_final_a.py
python3 verify/run_final_a.py --sanitize
make verify-final-a
node verify/visualizations/final_a_model_test.js
node verify/visualizations/final_a_browser_test.mjs --browser /path/to/chromium
```

10公開入口の単独・重複include（debug/release）、全カテゴリ混在の複数翻訳単位、10使用例と説明中usageの完全一致、5ジャッジ形式各24入力を検証します。ヒューリスティックやゲーム固有の証明に架空のPROBLEMは付けません。オンラインAC・スコア改善は未確認です。

Pythonの独立した9,505ケースをdebug/releaseで比較します。全部分文字列と出現位置、全Lyndon分割、最長共通区間の総当たり、任意精度の二変数多項式の直接和、64bitの全4,096組の単項基底のnim積、全引き算手集合、周期候補の全探索、独立SplitMix64表、keyごとの代表選択とTabu履歴、差分更新後の全評価を使います。

C++では400系列のSAM追加・キャッシュ再構築、全256byte、100,000文字、2D矩形と境界、64×64のmex定義、600組の全幅の体の法則、Fermatの2冪、256個の引き算手集合、巨大手による誤った外挿の拒否、2,000回のZobrist差分、hashが常に0のkey、期限切れ・aspiration、200組の既存SAとの完全な乱数再現、棄却・accept例外時のundo・整数超過を確認します。

図解モデルは3,078入力・32,128状態・14拒否条件。各prefixのSAMのendposと状態長、全Lyndon分割、直接矩形ハッシュ、nimの単項基底、周期証明、検索履歴、applyの一時状態とundoの全スコアを独立に比較します。[教材モデルとの差](../../visualizations/final_a/README.md)も明記しています。

既存 `run_string_algorithm.py`、`run_game_theory.py`、`run_heuristic_search.py`、`run_computational_geometry.py`、`run_combinatorial_optimization.py`、`run_advanced.py` を回帰検証します。サニタイザとChromiumは環境のptrace/socket制限により、制限外実行が必要になる場合があります。

検証結果（2026-10-17）：上記の通常・ASan/UBSan/LeakSanitizer（debug/release）、6カテゴリの既存回帰、全図解モデル、ブラウザ10モードと390px表示が成功しました。関連ローカルリンクに欠落なし。デスクトップ・モバイル画像も確認しました。
