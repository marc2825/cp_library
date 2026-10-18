# 一次資料・設計判断

2026-10-16に以下の実装・仕様を読み、境界条件と公開APIを検討しました。アルゴリズムの原理を参考に独立に実装し、外部ソースのコードを転載していません。

| 資料 | 参考箇所 | 今回の設計 |
|---|---|---|
| [C++標準草案 numeric.ops.gcd](https://eel.is/c++draft/numeric.ops.gcd) | 絶対値を共通型で表現できるという事前条件 | long longの最小値も扱うため、新GCDは符号なしの大きさと結果を使う |
| [ACL mathの公式仕様](https://atcoder.github.io/ac-library/production/document_ja/math.html) | pow_modの指数・法、inv_modの代表元・法1 | 累乗の法もlong longへ拡大。逆元が存在しない入力はoptionalで表す |
| [ACL internal_math.hpp](https://github.com/atcoder/ac-library/blob/master/atcoder/internal_math.hpp) | safe_mod、inv_gcdの係数と余りの不変条件 | 負の剰余を正規化。拡張Euclidは両係数を返し、最小整数の絶対値は符号なしで保持 |
| [KACTL euclid.h](https://github.com/kth-competitive-programming/kactl/blob/main/content/number-theory/euclid.h) | Bézout係数とmod逆元の関係 | 再帰でなく反復。最終余り0の係数生成を省き、全符号の極値を扱う |
| [KACTL Eratosthenes.h](https://github.com/kth-competitive-programming/kactl/blob/main/content/number-theory/Eratosthenes.h) | 素数の平方から消す篩・素数列 | 固定グローバルbitsetを使わず、上限を含むオブジェクト。0・1にも対応 |
| [CP-Algorithms: Linear Sieve](https://cp-algorithms.com/algebra/prime-sieve-linear.html) | 最小素因数以下の素数で合成数を一度だけ生成する原理 | 乗算前に上限/iと比較。SPFによる分解を同じクラスへ置く |
| [CP-Algorithms: Integer factorization](https://cp-algorithms.com/algebra/factorization.html) | 2と奇数の試し割り・残った素数 | 商による終了判定と(素数,指数)の出力。高速64ビット分解とは用途を区別 |
| [CP-Algorithms: Number of divisors](https://cp-algorithms.com/algebra/divisors.html) | 素因数の指数と約数個数の関係 | 検証の個数確認とSPF使用例に利用。約数本体は対を列挙して昇順連結 |

ACLはCC0、KACTLのEratosthenesはファイル内にCC0の記載があります。KACTL Euclidの当該ファイルには個別ライセンス記載がないため、コードを流用していません。CP-Algorithmsの文章のライセンスはCC BY-SA 4.0です。原文やサンプルコードの転載はしていません。

検証に使う問題の仕様は [AOJ NTL_1_E](https://judge.u-aizu.ac.jp/onlinejudge/description.jsp?id=NTL_1_E&lang=ja)、[AOJ NTL_1_A](https://judge.u-aizu.ac.jp/onlinejudge/description.jsp?id=NTL_1_A&lang=ja)、[ABC186 E](https://atcoder.jp/contests/abc186/tasks/abc186_e)、[ABC179 C](https://atcoder.jp/contests/abc179/tasks/abc179_c)も確認しました。NTL_1_Eの最小係数の選択は、汎用APIではなく問題用ラッパの責務にしています。

旧 `number_theory/` の拡張Euclid・篩・試し割り・約数列挙も読み、公開関数を変更せず新カテゴリを追加しました。既存の提出リンクは新APIのAC履歴として扱いません。

追加A項目の一次資料・使用問題と設計判断は[3.12〜3.15補完の資料一覧](../math_a/references.md)にまとめています。

3.12〜3.15のB追加分は[一次資料・設計判断](../b_12_15/references.md)を参照してください。
