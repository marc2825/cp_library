# 高度なアルゴリズム：一次資料と設計判断

今回のヘッダは既存のC++17・`namespace cp`・半開区間の規約に合わせた独立実装です。参照元の検証済み表示を、このライブラリのオンラインAC実績として引き継いでいません。既存の標準C++17の剰余乗算、NTT、ゼータ／メビウス、RootedTree／LCAを明示的に再利用しています。

| 項目 | 一次資料・著者の実装 | この実装の選択 |
|---|---|---|
| Beats | [NyaanのSegment Tree Beats](https://nyaannyaan.github.io/library/segment-tree/segment-tree-beats.hpp.html) | 固定long long、chmin/chmax/addと和・上下限、境界と算術条件を明記 |
| Wavelet Matrix | [Claude, Navarro, Ordóñez: The wavelet matrix](https://doi.org/10.1016/j.is.2014.06.002) | 座標圧縮と64bit rank辞書、任意の比較可能な値 |
| Rollback／動的連結性 | [NyaanのOffline Dynamic Connectivity](https://nyaannyaan.github.io/library/graph/offline-dynamic-connectivity.hpp.html) | 多重辺を参照数管理、時間区間木＋独立Rollback DSU |
| 永続木・Treap・Mo | [Library Checkerの問題・公式generator](https://github.com/yosupo06/library-checker-problems) | 根ハンドル、静的prefix頻度木、方向付き積、更新を利用側が管理するMo |
| Virtual Tree・DSU on Tree | [Nyaanの木アルゴリズム](https://github.com/NyaanNyaan/library/tree/master/tree) | 既存RootedTree・DoublingLCAを再利用、森対応、深いDFSは反復 |
| Link-Cut Tree | [NyaanのLink/Cut Tree](https://github.com/NyaanNyaan/library/blob/master/lct/link-cut-tree.hpp) | indexによるsplay、非可換パス積、反復push、辺更新は辺用頂点 |
| Li Chao／CHT | [既存CHTとその出典](../../data_structure/ConvexHullTrick.hpp) | 旧CHTは保持、追加のLi Chaoは事前座標・線分追加・optional・安全な評価 |
| SMAWK | [Aggarwal et al.: Geometric applications of a matrix-searching algorithm](https://doi.org/10.1007/BF01840359) | 全単調性を前提、最左同点、行列評価コールバック |
| Monotone Minima・凸Min-Plus | [suisen-cpのMin-Plus Convolution](https://suisen-cp.github.io/cp-library-cpp/library/convolution/min_plus_convolution.hpp.html) | 両凸O(n+m)、片凸は有効帯を直接絞る分割統治でINF不要 |
| Miller–Rabin | [決定的基底集合の記録](https://miller-rabin.appspot.com/)、[KACTL MillerRabin](https://github.com/kth-competitive-programming/kactl/blob/main/content/number-theory/MillerRabin.h) | 同じ7基底、浮動近似を使わず正確なuint64乗算で全域対応 |
| Pollard–Rho | [Brent: An improved Monte Carlo factorization algorithm](https://doi.org/10.1007/BF01933190) | 長さ倍増・GCDのバッチ化・失敗時再試行、固定seedで再現 |
| Subset Convolution | [Björklund et al.: Fourier meets Möbius](https://arxiv.org/abs/cs/0611101) | rank別ゼータとメビウス、汎用環・途中の整数超過を検査 |
| 任意mod畳み込み | [tko919のArbitrary Mod Convolution](https://tko919.github.io/library/Convolution/arbitrary.hpp.html)、[既存NTTの資料](../convolution/references.md) | 既存3素数のNTTを再利用、復元一意性の係数上界を明記 |
| Aho–Corasick | [Aho, Corasick: Efficient string matching](https://doi.org/10.1145/360825.360855) | 疎なbyte辺、空・重複pattern、failure集約と出力link |
| Eertree | [Rubinchik, Shur: EERTREE](https://arxiv.org/abs/1506.04862) | 2根、オンライン追記、非破壊の出現回数集約 |

上記はアルゴリズム・API・検証方針の参考です。外部コードをビルド時に取得する依存はありません。既存ACL由来のNTT等の出典・ライセンス記載は各カテゴリのreferencesに保持しています。
