# Q追加項目の一次資料・設計判断

2026-10-26〜27確認。今回のコードは本リポジトリの型・既存演算へ接続した実装です。外部ライブラリのソースを取り込んでいません。

| 対象 | 一次資料 | 設計・相違点 |
|---|---|---|
| Q-03/Q-28 有限体因数分解 | [NTL ZZ_pXFactoring](https://libntl.org/doc/ZZ_pXFactoring.cpp.html) | square-free、DDF、EDF、根抽出の前提を参照。NTLへの依存なし。DDFは逐次Frobenius、剰余除算は基本実装、小標数と微分0を処理 |
| Q-06/Q-27/Q-29/Q-35 転置・合成 | [Kinoshita–Li原論文](https://arxiv.org/html/2404.05177v2)、[AtCoder解説](https://atcoder.jp/contests/abc387/editorial/11875?lang=en)、[maspypyの公開合成](https://maspypy.github.io/library/poly/composition.hpp) | 二変数Bostan–Moriと転置原理を参照。自然順NTT、Kroneckerの空き領域、転置中間積で独自構成。maspypyのbit-reversed最適化やソースは取り込まない |
| Q-20/Q-21 FFT | [FFTWの定義](https://fftw.org/fftw3_doc/What-FFTW-Really-Computes.html) | 符号・正規化に流儀差があるため既存cp::fftの正符号・逆1/nを統一。Raderの乗法添字置換とBluesteinの二乗位相恒等式から構成 |
| Q-22 64bit素数 | [Plonky2 Goldilocks field](https://github.com/0xPolygonZero/plonky2/blob/main/field/src/goldilocks_field.rs) | p=2^64−2^32+1と専用体の設計を参照。128bit積の上位・下位を2^64≡2^32−1で還元するcp専用実装 |
| Q-37 Reeds–Sloane | [著者公開の原論文](https://neilsloane.com/doc/1218shift.pdf)、[SIAM出版情報](https://epubs.siam.org/doi/10.1137/0214038) | Section IVの多項式対・length・p進付値・archive更新を独立実装。分子次数をlengthへ含めること、CRT、空列、法1、末尾0を明示 |
| 検証用途 | [Library Checker](https://github.com/yosupo06/library-checker-problems)、[公式問題一覧](https://github.com/yosupo06/library-checker-problems/blob/master/categories.toml) | 各個別説明に使用問題を記載。ローカル比較とオンラインACを区別 |

Reeds–Sloaneの調査ではzimpha/algorithmic-libraryの固定commit `804b5859e96ed3240ef7ecac5fabb5fde88e15fc` とLICENSEも確認しましたが、CC BY-NC-NDのためソースを移植していません。上記原論文の数式手順を実装しています。

その他は既存のNTT/FPS・Karatsuba・nimber・Kitamasa・ranked-zetaの出典を引き継ぎます。数式・アルゴリズムの参照とソースの取り込みは区別しています。対応範囲・計算量の限定は各APIの説明を参照してください。
