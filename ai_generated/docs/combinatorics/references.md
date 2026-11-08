# 一次資料・設計判断

2026-10-16に次の実装・仕様を読み、API・前計算条件・境界を検討しました。外部コード・説明文の転載をせず、原理を参考にこのカテゴリの実装と説明を書いています。

| 一次資料 | 確認した箇所 | 今回の設計 |
|---|---|---|
| [Nyaan: binomial.hpp](https://nyaannyaan.github.io/library/modulo/binomial.hpp.html) | 階乗と逆階乗、nCk・nPk・nHk、前計算の法の境界 | 自動拡張せず上限を固定。実行時の64ビット法、逆元なしの構築拒否、明示的な範囲例外 |
| [CP-Algorithms: Binomial Coefficients](https://cp-algorithms.com/combinatorics/binomial-coefficients.html) | 階乗方式の逆元の条件、加算だけのPascalの三角形 | FastとPascalを同じbinomialの呼び方で使う。合成数法での除算の誤用を避ける |
| [NIST DLMF §26.3](https://dlmf.nist.gov/26.3) | 格子路と二項係数の意味 | 集合の選択・格子路を図解の例にし、単なる数式の表示で終わらせない |
| [NIST DLMF §26.5](https://dlmf.nist.gov/26.5)・[CP-Algorithms: Catalan Numbers](https://cp-algorithms.com/combinatorics/catalan-numbers.html) | 括弧列、反射の対応、二項係数の差 | n+1のmod逆元を使わない式。Pascalを選べば任意の正の法で計算 |
| [CP-Algorithms: Stars and Bars](https://cp-algorithms.com/combinatorics/stars_and_bars.html) | 非負整数の和と重複組合せ | 0種類・0個を独立に定義し、n+k-1は上限確認後に計算 |
| [CP-Algorithms: Inclusion–Exclusion](https://cp-algorithms.com/combinatorics/inclusion-exclusion.html) | 和集合の交互和、補集合、集合数でまとめる使い方 | 和集合／補集合を別の関数にし、mask=0の意味を明記。符号付き途中値・mod型を扱う |

Nyaanのリポジトリは [CC0](https://github.com/NyaanNyaan/library/blob/master/LICENSE)、CP-Algorithmsの文章はCC BY-SA 4.0です。今回はコードや原文の転載はしていません。数式をAPIの範囲・型・例外条件へ合わせて説明しています。

問題の公式仕様として、[CSES Binomial Coefficients](https://cses.fi/problemset/task/1079)、[Creating Strings II](https://cses.fi/problemset/task/1715)、[Distributing Apples](https://cses.fi/problemset/task/1716)、[Bracket Sequences I](https://cses.fi/problemset/task/2064)、[Prime Multiples](https://cses.fi/problemset/task/2185)、[ABC172 E](https://atcoder.jp/contests/abc172/tasks/abc172_e)を確認しました。包除原理の全ての部分集合を列挙する関数を、Nが50万になるABC172 Eへそのまま適用することはせず、ラッパでは一致する位置の個数でまとめています。

旧 `Combination/` の2実装も読み、公開フィールド・既存APIを保持しました。正の小入力は旧実装と比較しますが、旧APIの提出リンクは新APIのAC実績として扱いません。

追加A項目の一次資料・使用問題と設計判断は[3.12〜3.15補完の資料一覧](../math_a/references.md)にまとめています。

3.12〜3.15のB追加分は[一次資料・設計判断](../b_12_15/references.md)を参照してください。
