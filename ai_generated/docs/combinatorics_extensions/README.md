# C追加項目: 組合せ・数え上げ

C-01〜C-04・C-20〜C-23の全8項目。C-05〜C-19の行はありません。C++17、namespace cp、[カテゴリ一括include](../../combinatorics/all.hpp)。C-23は承認されたLah/Euler変換と逆変換です。

| ID | API・説明 | 図解 |
|---|---|---|
| C-01 | [q二項係数・q階乗](../combinatorics/q_binomial.md) | [入力と手順](../../visualizations/combinatorics_extensions/index.html#q_binomial) |
| C-02 | [上下境界付き単調列・格子路](../combinatorics/bounded_monotone.md) | [入力と手順](../../visualizations/combinatorics_extensions/index.html#bounded_monotone) |
| C-03 | [小素数法・巨大引数のStirling数](../combinatorics/stirling_small_prime.md) | [入力と手順](../../visualizations/combinatorics_extensions/index.html#stirling_small_prime) |
| C-04 | [ラベルなし木の同型類数](../combinatorics/unlabeled_trees.md) | [入力と手順](../../visualizations/combinatorics_extensions/index.html#unlabeled_trees) |
| C-20 | [RSK対応と逆対応](../combinatorics/rsk.md) | [入力と手順](../../visualizations/combinatorics_extensions/index.html#rsk) |
| C-21 | [Young図形と標準盤の列挙](../combinatorics/young_tableaux.md) | [入力と手順](../../visualizations/combinatorics_extensions/index.html#young_tableaux) |
| C-22 | [Hook-Length / Hook-Content](../combinatorics/hook_length.md) | [入力と手順](../../visualizations/combinatorics_extensions/index.html#hook_length) |
| C-23 | [Lah・Euler変換と逆変換](../combinatorics/combinatorial_transforms.md) | [入力と手順](../../visualizations/combinatorics_extensions/index.html#combinatorial_transforms) |

[出典・設計判断](references.md) / [動く使用例](../../examples/combinatorics/) / [図解の対応範囲](../../visualizations/combinatorics_extensions/README.md)

## 検証入口

```sh
make verify-combinatorics-extensions
python3 -m tools.cp_tools.suite --runners combinatorics_extensions
python3 -m tools.cp_tools.suite --stdlib-debug --runners combinatorics_extensions
python3 -m tools.cp_tools.suite --sanitize --runners combinatorics_extensions
node verify/visualizations/combinatorics_extensions_model_test.js
node verify/visualizations/combinatorics_extensions_browser_test.mjs --browser /path/to/chromium
```

8ヘッダの単独/重複includeをDebug/Releaseでコンパイルし、全カテゴリ混在2翻訳単位、8個の説明と同一の使用例を検査します。C++/図解の固定参照値は同じmanifestから照合。3つの比較テストはNDEBUGでも有効なCHECKを使います。

- q-Pascal・従来Stirling三角表・巨大引数の漸化式: 978,272比較。
- 単調列: 全境界・ランダム境界17,162比較、2万要素の二項係数/Catalan/一致境界。
- RSK・Young: 全順列/全単語/整数分割等16,252比較、行列の転置対称性、1万行の非再帰列挙。
- ラベルなし木: n≤7のPrüfer列と根付き括弧表現で同型類を集計。
- Hook: 全標準盤・小さい半標準盤の直接列挙、sum f²=n!、合成数法と64bit端。
- 変換: Lah三角表の行列積、Euler積のknapsack、正逆往復、NTTを使う長さ。

実行記録の保存先と集計方法は[検証台帳の使い方](../development_records/verification.md)を参照してください。
