# 一次資料と設計判断

2026-10-18に参照しました。外部ライブラリへの実行依存は追加していません。以下はアルゴリズムやAPIを調査した資料であり、本リポジトリのオンラインACを示すものではありません。

- [Nyaan: Relaxed Convolution](https://nyaannyaan.github.io/library/ntt/relaxed-convolution.hpp.html)：ブロックの積を将来の係数へ転送する設計。今回の実装はmin(i,j)の最高位bitによる帯の分割を使い、変換キャッシュは行いません。
- [Nyaan: Convolution Large](https://nyaannyaan.github.io/library/ntt/convolution-large.hpp.html)：NTTの作業長上限を超える畳み込み。今回の版はブロック対ごとの積を重ねる方式で、変換を共有する高速化は行いません。
- [Nyaan: Chirp-Z](https://nyaannyaan.github.io/library/ntt/chirp-z.hpp.html)：三角数の指数を使った等比点評価。今回はstartを第2引数、ratioを第3引数に固定し、全長積の容量条件を明記しました。
- [suisen: Formal Power Series](https://suisen-cp.github.io/cp-library-cpp/library/polynomial/formal_power_series.hpp)：疎な入力からの係数漸化式。今回のSparseFPSは重複次数を加算し、逆数・log・exp・非負整数冪を公開します。
- [Nyaan: FPS Composition](https://nyaannyaan.github.io/library/fps/fps-composition.hpp.html)：高速合成の比較資料。今回のB版はbaby-step/giant-stepで、小さい冪の保存とブロックごとの線形結合を行います。資料のO(N log²N)法を実装したとは扱いません。高度な高速合成はCに保持します。
- [Nyaan: Exp of SPS](https://nyaannyaan.github.io/library/set-function/exp-of-set-power-series.hpp)：変数を一つずつ増やす指数演算と部分集合積。今回の版は高位変数で分割し、同じ考え方からlogとinvも実装しました。
- [Library Checker: FPS composition仕様](https://github.com/yosupo06/library-checker-problems/blob/master/polynomial/composition_of_formal_power_series/task.md)と[SPS exp仕様](https://github.com/yosupo06/library-checker-problems/blob/master/set_power_series/exp_of_set_power_series/task.md)：ローカルのジャッジ形式と数学的定義。全最大制約の時間保証はしていません。
- [LinBox: Wiedemann](https://github.com/linbox-team/linbox/blob/master/linbox/algorithms/wiedemann.h)：行列ベクトル積だけを使う有限体の求解と射影。今回はスカラー射影のBM、固定seedの試行、候補解の残差確認を公開します。
- [Harrison–Johnson–Saunders: Probabilistic analysis of Wiedemann's algorithm](https://arxiv.org/abs/1412.5071)：射影の最小多項式が行列全体の最小多項式と一致するとは限らない点。今回のwiedemann_polynomialは射影列の多項式と明記し、matrix_minimal_polynomial_modは標準基底ごとのKrylov依存関係のlcmによる決定的版です。
- [Higham: Sherman–Morrison–Woodbury](https://nhigham.com/2020/09/29/what-is-the-sherman-morrison-woodbury-formula/)：逆行列の低ランク更新と小さい核行列。今回は素数modのWoodbury式と行列式比を公開し、特異な更新はnullopt/0で返します。

GCD/LCM、Min-Plus、漸化式推定と第N項、Matrix Tree、ranked-zetaによるSubset Convolutionは既存の公開実体を共通利用します。[3.12〜3.15 Bの出典](../b_12_15/references.md)、[3.16〜3.20 Aの出典](../transform_a/references.md)、各カテゴリのreferencesも参照してください。

有限多項式合成・Euclid法・剰余二分累乗・係数履歴付きKrylov消去・部分分数の局所Taylor展開は記載した定義式から構成しました。外部のmodint・NTT・乱数エンジンには依存せず、標準C++17と既存の本リポジトリ実装を使います。
