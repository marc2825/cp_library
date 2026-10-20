# 基本ユーティリティ・列挙・探索 U-01〜U-22

[統合一覧](../remaining_work.md) / [カテゴリ](../basic_utility/README.md) / [参考資料](references.md) / [図解の範囲](../../visualizations/utility_search/README.md)

今回の範囲は一覧に掲載されたU-01〜U-11とU-20〜U-22の14項目です。U-12〜U-19という行はありません。U-20は「最新版への点代入・過去版の参照・単調条件が初めて成立する版の検索」と仕様を定め、既存の部分永続Union-Findへの検索アダプタも追加しました。U-21は固定時刻上の操作合成の編集、U-22は回文・数字制限・指定因子の積の昇順列挙と仕様を定めました。全14項目が実装済みです。

| ID | 説明・使用例・条件 | ヘッダ | 図解 |
|---|---|---|---|
| U-01 | [尺取り法](../basic_utility/two_pointers.md) | [two_pointers.hpp](../../basic_utility/two_pointers.hpp) | [図解](../../visualizations/utility_search/index.html#two_pointers) |
| U-02 | [最大部分配列和・区間復元](../basic_utility/maximum_subarray.md) | [maximum_subarray.hpp](../../basic_utility/maximum_subarray.hpp) | [図解](../../visualizations/utility_search/index.html#maximum_subarray) |
| U-03 | [重複付き動的MEX](../basic_utility/dynamic_mex.md) | [dynamic_mex.hpp](../../basic_utility/dynamic_mex.hpp) | [図解](../../visualizations/utility_search/index.html#dynamic_mex) |
| U-04 | [半分全列挙・部分和の個数と復元](../basic_utility/meet_in_the_middle.md) | [meet_in_the_middle.hpp](../../basic_utility/meet_in_the_middle.hpp) | [図解](../../visualizations/utility_search/index.html#meet_in_the_middle) |
| U-05 | [順列の合成・逆・サイクル・符号・冪](../basic_utility/permutation.md) | [permutation.hpp](../../basic_utility/permutation.hpp) | [図解](../../visualizations/utility_search/index.html#permutation) |
| U-06 | [黄金分割探索・整数フィボナッチ探索](../basic_utility/unimodal_search.md) | [unimodal_search.hpp](../../basic_utility/unimodal_search.hpp) | [図解](../../visualizations/utility_search/index.html#unimodal_search) |
| U-07 | [集合・多重集合ハッシュ](../basic_utility/collection_hash.md) | [collection_hash.hpp](../../basic_utility/collection_hash.hpp) | [図解](../../visualizations/utility_search/index.html#collection_hash) |
| U-08 | [順列木・共通区間分解](../basic_utility/permutation_tree.md) | [permutation_tree.hpp](../../basic_utility/permutation_tree.hpp) | [図解](../../visualizations/utility_search/index.html#permutation_tree) |
| U-09 | [括弧列・順序木・辞書順k番目](../basic_utility/parentheses.md) | [parentheses.hpp](../../basic_utility/parentheses.hpp) | [図解](../../visualizations/utility_search/index.html#parentheses) |
| U-10 | [整数分割の列挙](../basic_utility/integer_partitions.md) | [integer_partitions.hpp](../../basic_utility/integer_partitions.hpp) | [図解](../../visualizations/utility_search/index.html#integer_partitions) |
| U-11 | [集合分割・Restricted Growth列挙](../basic_utility/set_partitions.md) | [set_partitions.hpp](../../basic_utility/set_partitions.hpp) | [図解](../../visualizations/utility_search/index.html#set_partitions) |
| U-20 | [部分永続配列と最初の成立時刻の探索](../basic_utility/partial_persistent_search.md) | [partial_persistent_search.hpp](../../basic_utility/partial_persistent_search.hpp) | [図解](../../visualizations/utility_search/index.html#partial_persistent_search) |
| U-21 | [過去の操作を編集する時刻列](../basic_utility/retroactive_operations.md) | [retroactive_operations.hpp](../../basic_utility/retroactive_operations.hpp) | [図解](../../visualizations/utility_search/index.html#retroactive_operations) |
| U-22 | [特殊な整数の昇順列挙](../basic_utility/special_integer_enumeration.md) | [special_integer_enumeration.hpp](../../basic_utility/special_integer_enumeration.hpp) | [図解](../../visualizations/utility_search/index.html#special_integer_enumeration) |

C++17、namespace cp、原則0始まり・半開区間です。黄金分割探索の実数区間だけは端点を含みます。個別ヘッダとbasic_utility/all.hppから利用できます。括弧列・分割数では既存BigInt経由でBoostが必要です。半分全列挙と整数探索はGCC/Clangの128bit整数拡張を使います。

## 検証

```sh
python3 verify/run_utility_search.py
python3 -m tools.cp_tools.suite --runners utility_search
python3 -m tools.cp_tools.suite --stdlib-debug --runners utility_search
python3 -m tools.cp_tools.suite --sanitize --runners utility_search
node verify/visualizations/utility_search_model_test.js
node verify/visualizations/utility_search_browser_test.mjs --browser /path/to/chromium
```

2026-10-20、GCC 11.4・C++17・Boost 1.74で以下を確認しました。

- 通常のDebug/Release、libstdc++ debug modeのDebug/Release、ASan・UBSan・LeakSanitizerのDebug/Releaseで全比較テスト成功。
- 14ヘッダの単独・重複include、全カテゴリ混在・2翻訳単位、14使用例、14件のC++/図解の結果比較、1,610文書リンクに成功。
- Node 22.14のモデル検証、Chromiumの14モード・16変更入力・前後移動・再生・エラー復帰・ローカルリンク・390px表示に成功。desktop5枚とmobile2枚の画像を生成し、desktopとmobileの表示を確認。
- 出典検査は9レコード・11ファイルを追跡。自動台帳の597ヘッダ・307 C++テスト・44runnerへの反映、進捗245行の内訳も確認。

[通常構成の実行記録](../../verification/evidence/utility_search/standard.json) / [stdlib-debugの実行記録](../../verification/evidence/utility_search/stdlib-debug.json) / [sanitizerの実行記録](../../verification/evidence/utility_search/sanitize.json) は現在のソースfingerprintに一致します。共通suiteが結果を保存し、コード変更後は失効します。以前の他分野の記録は今回のソース追加で失効したまま保持しています。今回のテストをCIのlocal jobにも追加しました。GitHub Actions上の実行とオンライン提出は未実施です。

[unit_test.cpp](../../verify/utility_search/unit_test.cpp) はNDEBUGでも消えないチェックを使います。

- 尺取り・最大部分配列和は全区間走査、動的MEXは頻度表と0からの走査と比較。
- 半分全列挙は16要素までの全bitmaskで和・個数・復元・同点を比較。128bitの問い合わせ極値、64bit入力極値も確認。
- 順列演算・順列木はn≤8の全順列。転倒数、反復適用、全共通区間、交差しない強共通区間を独立に計算。順列木は20万要素でも構築。
- 整数探索は左右で傾きの異なる単峰列、最小平坦区間、全64bit幅近くの探索。評価点の範囲と回数もチェック。実数探索は端点を含む解析解と比較。
- 集合hashは頻度表を逆順で再構築して比較。これは衝突しないことの証明ではありません。
- 括弧列は9組まで全ての2^(2n)候補から正規列を抽出してrank/unrankを照合。100組の大きな順位、深さ10万の木の往復を確認。
- 整数分割・集合分割は独立した再帰列挙、個数DP、出力順・早期停止を比較。
- 部分永続は全版のコピー、素朴な連結成分ラベル、各版の線形探索を比較。
- 過去操作編集は[追加の比較テスト](../../verify/utility_search/retroactive_enumeration_test.cpp)で3000回の編集をstd::mapの再生と照合。非可換・非可逆・時刻極値を確認。時刻数は通常／サニタイザ20万、libstdc++ debug modeでは2000（二分探索の前提検査が線形走査になるため）。
- 特殊整数列挙は100万未満の回文、全1024数字集合・256因子集合、600範囲、20桁の回文、uint64上限、空・重複・打切りを独立走査／DPと比較。

14ヘッダの単独・重複includeをDebug/Releaseで検査し、全カテゴリall.hppとの混在・2翻訳単位リンク・文書の例とソースの一致・14使用例の出力・相互リンクも検証します。図解の14サンプルは[実C++ bridge](../../verify/utility_search/visualization_bridge.cpp)と比較し、JavaScript側にも独立比較を置いています。オンライン提出ACは未実施です。
