# F-08〜F-10の一次資料と設計判断

参照日: 2026-10-19。

- [Sebastiano Vigna: SplitMix64](https://prng.di.unimi.it/splitmix64.c)：ハッシュの拡散に使う処理。public domainの原実装を参照した既存[Random](../../heuristic_search/random.hpp)を共有し、新たに乱数生成器を複製していません。
- [Abseil: Swiss Tables Design Notes](https://abseil.io/about/design/swisstables)：open addressingで空／削除／使用中を区別する設計を参照。今回の実装は線形探索・optionalのslot・負荷率1/2であり、SIMDやSwiss Tableのコードを取り込んでいません。
- [Boost 1.85: Compressed Sparse Row Graph](https://www.boost.org/doc/libs/1_85_0/libs/graph/doc/compressed_sparse_row.html)：offsetと連続arc配列による静的グラフ表現を参照。既存Graphの無向既定・論理辺ID・入力順に合わせて独自実装しました。Boostの実行依存はありません。
- [ACL Segtree](https://atcoder.github.io/ac-library/production/document_ja/segtree.html) / [Lazy Segtree](https://atcoder.github.io/ac-library/production/document_ja/lazysegtree.html)：モノイドと作用の契約・合成順序。既存のACL由来Segtree／LazySegtree（CC0、ヘッダに出典あり）を共有しています。

複合キーhashの結合、CSRの逆arc対応、各Monoidと多項式momentの式は個別説明に記した契約から実装しています。新規ヘッダに外部ライブラリのソースを取り込み・同梱していません。一次分数は体上の可逆変換として条件を限定し、行列積は既存Matrix APIを使用します。上記のリンクは設計資料であり、特定commitを移植したという記録ではありません。

[API・検証](README.md)
