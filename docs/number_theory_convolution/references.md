# 一次資料・設計判断

2026-10-16に以下の資料・実装・問題仕様を読みました。アルゴリズムの原理を参考に独立に実装し、外部コードや説明文を転載していません。

| 一次資料 | 参考箇所 | 今回の設計 |
|---|---|---|
| [noshi91: 高速ゼータ変換の約数版](https://noshi91.hatenablog.com/entry/2018/12/27/121649) | 素因数の指数に沿う累積和、走査方向、O(N log log N) | 素数列は3.13の篩を再利用。空・添字0を定義し、4変換を個別のヘッダにする |
| [Nyaan: 倍数変換・約数変換](https://nyaannyaan.github.io/library/multiplicative-function/divisor-multiple-transform.hpp.html) | 4変換の定義、kの昇順／降順、逆変換 | 逆変換は素数とkの両方を逆順にし、更新列全体の逆として実装。標準整数は超過を検出 |
| [Nyaan: GCD畳み込み](https://nyaannyaan.github.io/library/multiplicative-function/gcd-convolution.hpp.html) | zeta→要素積→mobiusの構成 | 入力長の違いを0で補い、出力0はゼロ。3回の変換で篩を共有 |
| [NIST DLMF §27.5](https://dlmf.nist.gov/27.5) | Dirichlet積、メビウス反転、φの恒等式 | ij=nの直接集計、上限までの出力、idの逆変換の使用例。乗法的関数という入力条件は置かない |
| [Library Checker GCDの問題定義](https://github.com/yosupo06/library-checker-problems/blob/master/convolution/gcd_convolution/task.md)・[パラメータ](https://github.com/yosupo06/library-checker-problems/blob/master/convolution/gcd_convolution/info.toml) | 正の添字1〜N、998244353、N上限100万 | 実装のmodは固定せず、ジャッジ用の小さなmod型で入出力検証。100万の全出力も独立式と比較 |
| [Library Checker LCMの問題定義](https://github.com/yosupo06/library-checker-problems/blob/master/convolution/lcm_convolution/task.md)・[パラメータ](https://github.com/yosupo06/library-checker-problems/blob/master/convolution/lcm_convolution/info.toml) | LCMごとの集計と出力範囲 | Nを超えるLCMを捨てる条件を明記。完全な結果が必要なら入力をゼロ拡張する |

Nyaanのリポジトリは [CC0](https://github.com/NyaanNyaan/library/blob/master/LICENSE) です。今回はコードの転載をせず、共通の更新処理・例外条件・型の演算条件を独自に設計しています。記事の説明文もコピーせず、このAPIと図解に合わせて書きました。

GCDとLCMは同じ添字対に異なる集計先を与えます。約数／倍数のzetaの向きを逆にすると結果も入れ替わるため、往復テストに加えて全添字対の独立オラクルを使います。逆変換のμも定義から独立に計算し、同じ変換実装で期待値を作りません。
