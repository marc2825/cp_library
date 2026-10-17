# 3.21 文字列アルゴリズム

[ライブラリ一覧](../../README.md) / [全図解](../../visualizations/index.html) / [一次資料・設計判断](references.md)

S項目を`string_algorithm/`へまとめています。[all.hpp](../../string_algorithm/all.hpp)が入口です。旧`string/`のファイルは保持し、新しいAPIはnamespace cpに置きます。

| S項目 | API・usage | 意味・条件 | 図解 |
|---|---|---|---|
| Z Algorithm | [z_algorithm](z_algorithm.md) | 先頭と各suffixのLCP、先頭値n | [一致区間](../../visualizations/string_algorithm/index.html#z) |
| Prefix Function | [prefix_function](prefix_function.md) | 各接頭辞の最長の真のborder、先頭値0 | [戻る先](../../visualizations/string_algorithm/index.html#prefix) |
| KMP | [kmp_search](kmp.md) | 重複を含む全一致、空patternは全境界 | [本文を戻さない検索](../../visualizations/string_algorithm/index.html#kmp) |
| Rolling Hash | [RollingHash](rolling_hash.md) | 二つの素数法、区間・連結・LCP、衝突あり | [先頭分を引く](../../visualizations/string_algorithm/index.html#hash) |
| Trie | [Trie](trie.md) | バイト列の多重集合、完全一致・prefix・1個削除 | [接頭辞の共有](../../visualizations/string_algorithm/index.html#trie) |
| Suffix Array | [suffix_array](suffix_array.md) | 空suffixを除いた辞書順、O(n log n)倍増法 | [順位の組](../../visualizations/string_algorithm/index.html#suffix) |
| LCP Array | [lcp_array](lcp_array.md) | 辞書順の隣接suffix間、O(n) | [長さを持ち越す](../../visualizations/string_algorithm/index.html#lcp) |
| Manacher | [manacher](manacher.md) | 奇数／偶数の半径と区間の回文判定 | [鏡像の再利用](../../visualizations/string_algorithm/index.html#manacher) |

## 共通仕様

- C++17、namespace cp、標準ライブラリだけで使えます。0始まり・半開区間[l,r)です。入力を変更せず、計算中だけstring_viewを参照します。
- 文字列は**バイト列**です。UTF-8の文字数や見た目の文字単位には変換しません。辞書順・ハッシュ・Trieのbyteはunsigned charで0..255として扱います。0を含むデータは、長さを持つstringや明示したstring_viewを渡してください。
- 添字・長さの配列はintで、各入力長はINT_MAXまで。超過はlength_error。メモリ確保そのものの失敗は通常のbad_alloc等です。
- Z / Prefix / KMP / Suffix Array / LCP / Manacherにはvector<T>版もあります。等値比較だけの手法は==、Suffix Arrayは<を要求します。両方を組み合わせる場合は順序の同値性と==が一致することが前提です。
- 空入力を許します。Zの先頭はn、Prefixの先頭は0。空patternは0..n。Suffix Arrayは空suffixを含めず、LCPはmax(0,n−1)要素。Manacherはodd/evenに分け、空区間を回文とします。
- Rolling Hashは固定baseの二重ハッシュで、衝突時に等値やLCPが誤ります。baseは全オブジェクトで共通にし、厳密な照合が必要な場面ではKMPやSA/LCP等を選べます。
- Trieは空単語・重複と任意のbyteを扱います。eraseは1個、prefixの個数は重複込みです。個数0の過去のノードはclearまで保持します。

## ローカル検証

```sh
python3 verify/run_string_algorithm.py
python3 verify/run_string_algorithm.py --sanitize
node verify/visualizations/string_algorithm_model_test.js
node verify/visualizations/string_algorithm_browser_test.mjs --browser /path/to/chromium
```

`--compiler clang++`で切り替えられます。10ヘッダ、8使用例、8ジャッジ形式・512ケース。9,841三値文字列・180全バイト列で全配列・全区間の回文・ハッシュを定義から比較し、3,937 KMP組、48,000 Trie操作、6,000異なる文字列間の区間ハッシュ問い合わせを検証。Python任意精度4,126クエリ（うちTrie 2,880操作）、汎用整数列、0/FF・空・重複・base境界・入力拒否、20万文字の同一／交互列と深いTrie、比較回数の線形上限も含みます。通常・サニタイザのdebug/releaseを実行します。オンラインACは未確認です。

図解は8モード。先頭一致区間、borderへの戻り、重複一致、二重ハッシュの各prefixと二分探索、Trieの重複数、suffix順位の倍増、LCPの持越し、回文の鏡像を操作できます。UTF-8とhexを切り替え、0/FFも入力可能。3,096入力・51,386スナップショット・16拒否条件を独立に検証します。

base=257では、バイト列`0d 81 93 6e fc 1f 73 22`と8個の`00`が二つの法で同時に衝突することも検証しています。係数差の多項式が両法の積に等しくなる例です。

## 高度なアルゴリズムの追加

- [Aho–Corasick](aho_corasick.md) / [操作できる図解](../../visualizations/advanced/index.html#aho)
- [Eertree（回文木）](eertree.md) / [操作できる図解](../../visualizations/advanced/index.html#eertree)

追加分の検証は `python3 verify/run_advanced.py`（`--sanitize`対応）。[全追加項目と既存項目の対応表](../advanced/README.md)。

## 3.21〜3.25 A項目の補完

- [Suffix Automaton](suffix_automaton.md) / [操作できる図解](../../visualizations/final_a/index.html#suffix_automaton)
- [Lyndon Factorization](lyndon_factorization.md) / [操作できる図解](../../visualizations/final_a/index.html#lyndon)
- [Longest Common Substring](longest_common_substring.md) / [操作できる図解](../../visualizations/final_a/index.html#common_substring)
- [2D Rolling Hash](rolling_hash_2d.md) / [操作できる図解](../../visualizations/final_a/index.html#hash_2d)

[追加APIの条件・検証一覧](../final_a/README.md)。`python3 verify/run_final_a.py` / `--sanitize` / `make verify-final-a`。
