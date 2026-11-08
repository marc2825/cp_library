# 一次資料・設計判断

2026-10-16に次の一次資料と公式仕様を確認しました。外部コードや説明文の転載はせず、定義式から実装しています。

| 一次資料 | 確認した内容・今回の設計 |
|---|---|
| [ACL: String公式文書](https://atcoder.github.io/ac-library/production/document_en/string.html) | Zの先頭値n、SAの空suffix除外、隣接LCPとvector版の規約。空のLCPも扱い、SAは独立なO(n log n)倍増法を選ぶ |
| [CP-Algorithms: Prefix Function / KMP](https://cp-algorithms.com/string/prefix-function.html) | borderをたどる戻り先と本文を戻さない走査。区切り文字を入れず、重複一致と空patternを明示 |
| [CP-Algorithms: String Hashing](https://cp-algorithms.com/string/string-hashing.html) | 先頭ハッシュから区間を求める式と衝突の限界。byte+1を二つの素数法で計算し、baseの一致を検査 |
| [CP-Algorithms: Suffix Array](https://cp-algorithms.com/string/suffix-array.html) | 順位を倍増する構成とKasaiのLCP。文字列へ番兵を追加せず、外側の順位を−1として計数ソート |
| [CP-Algorithms: Manacher](https://cp-algorithms.com/string/manacher.html) | 奇数と偶数の中心、対称位置の半径の再利用。番兵バイトを使わず、区間の回文判定も提供 |
| [AOJ Dictionary公式問題](https://judge.u-aizu.ac.jp/onlinejudge/description.jsp?id=ALDS1_4_C) | insert/findの辞書操作。新Trieは任意のbyte、重複・prefix個数・削除にも対応し、辺を疎に保持 |
| [AOJ String Search公式問題](https://judge.u-aizu.ac.jp/onlinejudge/description.jsp?id=ALDS1_14_B) | 検索の0始まりの全出現位置。KMPとハッシュのラッパを直接比較 |
| [CSES Finding Borders公式問題](https://cses.fi/problemset/task/1732) | 全体と異なる真のborderを昇順に返す使用例 |
| [Library Checker: Enumerate Palindromes公式仕様](https://raw.githubusercontent.com/yosupo06/library-checker-problems/master/string/enumerate_palindromes/task.md) | 奇数・偶数の中心を交互にした最大回文長への変換 |

旧string/rolling_hash.hppとRunLengthEncoding.hppは保持します。新cp::RollingHashは二重の32ビット素数法で、旧61ビット法とはハッシュ値・型・APIを分けています。特定の記号が入力にないことを前提にしません。

固定baseでのハッシュ一致は厳密な文字列一致を保証しません。二つの法でも独立一様な衝突確率を無条件には主張せず、検証済みの入力で衝突しなかった事実と保証を区別します。CP-Algorithmsの文章はCC BY-SA 4.0。今回外部コードは取り込んでいません。オンラインACは未確認です。

3.21〜3.25のA補完については[追加の一次資料・設計判断](../final_a/references.md)を参照してください。
