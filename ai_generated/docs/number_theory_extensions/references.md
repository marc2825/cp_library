# N追加項目の一次資料・設計判断

2026-10-26確認。数学的恒等式・公開仕様を参照して独立実装しました。外部ソースコードの転載・移植はありません。BigInt/BigRational、Miller–Rabin/Pollard–Rho、原始根、整数平方根、任意mod畳み込み、安全な環演算は既存の共通部品を使います。

| 対象 | 資料・共有部品・判断 |
|---|---|
| Floor Sum・合同式 | [AtCoder Library公式math仕様](https://atcoder.github.io/ac-library/production/document_ja/math.html)、[ARC182 E公式解説](https://atcoder.jp/contests/arc182/editorial/10674)。床の差から剰余の個数を数えます。新APIはBigIntと最初の実現添字を追加。万能Euclidの水平/垂直交換とmoment平行移動は個別説明の操作列から導出しました。 |
| 大引数階乗 | [Library Checker問題集](https://github.com/yosupo06/library-checker-problems)のFactorial Mod型の用途。今回、個別問題ページの取得はできなかったためオンライン仕様への適合・ACは主張しません。既存の静的法FPSは実行時の素数法を受けないため、積・剰余木に必要な演算を内部実装し、積は既存convolution_modへ委譲します。線形階乗表を巨大化しません。 |
| Min_25・素数和・大規模総和 | [boxによるMin_25と拡張の導出](https://codeforces.com/blog/entry/92703)、[ABC370 G公式解説](https://atcoder.jp/contests/abc370/editorial/10906)。完全乗法的重みの素数篩と、素数を逆順に復元する表計算を採用。任意の乗法的関数には素数prefixを与える必要があり、その構築費用は別です。 |
| 連分数・有理数近似 | [Python公式Fraction.limit_denominator仕様](https://docs.python.org/3/library/fractions.html#fractions.Fraction.limit_denominator)。分母制限下の最小誤差という操作の参考です。コードは転記せず、収束分数と半収束分数の比較から実装し、同点規則を独自に明示。 |
| Gaussian整数・二平方数和 | [Jürgen Müller, Elementary number theory講義](https://www.math.rwth-aachen.de/~Juergen.Mueller/preprints/jm121.pdf)。Euclid領域・二次整数環の理論を参照。Gaussianの丸めとassociate、非負かつ順序を除いた二平方数和を公開契約で固定します。 |
| Pell方程式 | [Manuel Eberl, Pell’s Equation (AFP)](https://isa-afp.org/entries/Pell.html)。基本解とその累乗による解の列の理論。正/負のnormを区別し、連分数の収束分数から最小解を返します。 |
| 既存の64bit演算 | [Pollard–Rho](../number_theory_convolution/pollard_rho.md)、[原始根](../number_theory_basic/primitive_root.md)、[BigInt](../basic_utility/big_integer.md)、[任意mod畳み込み](../convolution/convolution_mod.md)。uint64の積は共通の剰余乗算、無制限の式はBigIntで処理します。 |

区間篩は短区間に残る素因数を持つ配列、乗法位数はCarmichael指数の因子除去、batch inversionはprefix積、素数冪二項係数はpを除いた階乗の周期とCRT、再利用BSGSはgcd縮小列とbaby tableをそれぞれ実装しました。Dirichlet積・逆元は双曲線恒等式と商集合の閉包性から導出しています。

N-28は「Σ i^p·((a*i+b) mod m)^q」の厳密和と任意法への出力に具体化しました。N-20は素数法、N-07は2..2×10^9の実行時素数、N-06は各素数冪の表サイズを予算内に確保できる法が対象です。各APIの条件・予算は個別説明に記載しています。Python版移植とオンライン提出は今回の対象外です。
