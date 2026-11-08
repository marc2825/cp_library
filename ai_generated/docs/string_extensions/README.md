# 文字列・圧縮索引 S全11項目

[統合一覧](../remaining_work.md) / [実装計画](../implementation_plan.md) / [一次資料](references.md) / [11モードの図解](../../visualizations/string_extensions/index.html)

S-01〜S-07・S-20〜S-23の全11項目を11公開ヘッダ・11使用例・11図解で整備しています。全byte・空文字列・半開区間を共通規約にし、既存のSA/LCP、SuffixAutomaton、Z Algorithm、SuccinctBitVectorを共有します。

| ID | API・条件・説明 | ヘッダ | 図解 |
|---|---|---|---|
| S-01 | [部分列オートマトン・異なる部分列数](../string_algorithm/subsequence_automaton.md) | [subsequence_automaton.hpp](../../string_algorithm/subsequence_automaton.hpp) | [図解](../../visualizations/string_extensions/index.html#subsequence) |
| S-02 | [Lempel–Ziv（LZ77）分解](../string_algorithm/lempel_ziv.md) | [lempel_ziv.hpp](../../string_algorithm/lempel_ziv.hpp) | [図解](../../visualizations/string_extensions/index.html#lz77) |
| S-03 | [BWT・逆BWT](../string_algorithm/burrows_wheeler.md) | [burrows_wheeler.hpp](../../string_algorithm/burrows_wheeler.hpp) | [図解](../../visualizations/string_extensions/index.html#bwt) |
| S-04 | [FM-index](../string_algorithm/fm_index.md) | [fm_index.hpp](../../string_algorithm/fm_index.hpp) | [図解](../../visualizations/string_extensions/index.html#fm_index) |
| S-05 | [Prefix-Substring / Semi-local LCS](../string_algorithm/semi_local_lcs.md) | [semi_local_lcs.hpp](../../string_algorithm/semi_local_lcs.hpp) | [図解](../../visualizations/string_extensions/index.html#semi_local) |
| S-06 | [Border Tree・全周期列挙](../string_algorithm/border_tree.md) | [border_tree.hpp](../../string_algorithm/border_tree.hpp) | [図解](../../visualizations/string_extensions/index.html#borders) |
| S-07 | [SAMのk番目・全出現位置](../string_algorithm/suffix_automaton_utilities.md) | [suffix_automaton_utilities.hpp](../../string_algorithm/suffix_automaton_utilities.hpp) | [図解](../../visualizations/string_extensions/index.html#sam_utilities) |
| S-20 | [両端回文木](../string_algorithm/double_ended_eertree.md) | [double_ended_eertree.hpp](../../string_algorithm/double_ended_eertree.hpp) | [図解](../../visualizations/string_extensions/index.html#deque_eertree) |
| S-21 | [Online Z](../string_algorithm/online_z.md) | [online_z.hpp](../../string_algorithm/online_z.hpp) | [図解](../../visualizations/string_extensions/index.html#online_z) |
| S-22 | [Generalized Suffix Automaton](../string_algorithm/generalized_suffix_automaton.md) | [generalized_suffix_automaton.hpp](../../string_algorithm/generalized_suffix_automaton.hpp) | [図解](../../visualizations/string_extensions/index.html#generalized_sam) |
| S-23 | [区間の周期・反復回数](../string_algorithm/substring_period.md) | [substring_period.hpp](../../string_algorithm/substring_period.hpp) | [図解](../../visualizations/string_extensions/index.html#period_queries) |

半局所LCSと既存Static Range LISはseaweedの範囲計数を共通化しました。既存LISの分割統治・unit-Monge積は保持しています。BWTは終端を仮想記号にしてbyte 0も扱い、FM-indexはpacked rankと座標で間引いたSA標本を保存します。

仕様が必要だったS-23は、ユーザーの承認を得て、固定文字列の任意区間についてperiod判定・最小反復周期・反復回数を求める機能に具体化しています。長さを割り切らないperiodと、整数回の反復単位を区別します。Online Zはborder木による厳密なO(log² n)問い合わせです。両端回文木はsurfaceを更新し、過去のnodeをclearまで保持します。各機能の計算量・メモリ上限は個別APIに明記しています。

## 検証

```sh
make verify-string-extensions
python3 -m tools.cp_tools.suite --runners string_extensions
python3 -m tools.cp_tools.suite --stdlib-debug --runners string_extensions
python3 -m tools.cp_tools.suite --sanitize --runners string_extensions
node verify/visualizations/string_extensions_model_test.js
node verify/visualizations/string_extensions_browser_test.mjs --browser /path/to/chromium
```

- [索引](../../verify/string_extensions/index_test.cpp)：355全探索・全byte文字列で部分列集合、greedy LZ77長、直接整列したBWT、逆変換、FM出現とsampling、k番目・全出現位置を照合。90文書の逐次追加で境界を跨がないunion・出現・文書IDを比較。256種byte・uint64最大法・空入力・不正入力・10万byteの繰返しも確認します。
- [構造](../../verify/string_extensions/structure_test.cpp)：511文字列の全prefixについてborder・全period・Z値・出現数、全区間の反復周期を定義と比較。961文字列対で4種類のSemi-localとPrefix-Substring全問い合わせを通常LCSに照合。共通化したStatic Range LISの200列、20万追記のOnline Zも確認します。
- [両端回文木](../../verify/string_extensions/deque_test.cpp)：3万ランダム操作を全回文の直接列挙と比較。1,020種類の追加・削除履歴、同一byteの20万両端更新、4万のnode再利用操作を検証します。
- 全11ヘッダの単独・重複include、全カテゴリ混在2翻訳単位、11使用例を実行。Python任意精度とbyte列の直接比較による96クエリ、Number of Substringsのジャッジ形式24入力も確認します。
- 図解は実C++使用例の11参照値と独立モデル比較、11モードの入力変更・前後移動・再生・エラー復帰・390px表示を検証します。

検証記録は共通suiteから[検証台帳](../../verification/evidence/)へ保存します。fingerprintが異なる過去の成功を現状の証拠としません。オンライン提出AC・GitHub Actionsでの実行は未確認です。

## 今回の実行結果（2026-10-28）

通常48.1秒・libstdc++ debug mode69.5秒・ASan/UBSan/LeakSanitizer86.2秒で、Debug/NDEBUG・11使用例・C++/図解11参照値・独立比較を通過しました。既存string_algorithm（33.1秒）と、共有したStatic Range LISを含むdata_structure_advanced（55.2秒）の通常回帰も成功しています。

[通常の記録](../../verification/evidence/string_extensions/standard.json) / [debug mode](../../verification/evidence/string_extensions/stdlib-debug.json) / [sanitizer](../../verification/evidence/string_extensions/sanitize.json)。5記録は同じソースfingerprint `70cfcfe82497fc1e87e9736124bfa9a5e496e949a5e0019fbe0dcf6b4832df77`（1886ファイル）に一致します。

図解モデル8,050独立比較・全11モードのブラウザ操作・11変更入力・390px表示が成功し、BWT・半局所LCS・両端回文木の画像も確認しました。出典検査・workflow構文・進捗245行の集計も成功。検証台帳は775ヘッダ・358 C++テスト・55runnerです。ChromiumとLeakSanitizerは既知のsocket/プロセス検査制限に従い、承認レビュー付きでローカル検証を実行しました。
