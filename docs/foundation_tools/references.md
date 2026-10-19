# F-12〜F-17の資料・設計判断

参照日: 2026-10-19。

- [Boost Rational](https://www.boost.org/doc/libs/1_74_0/libs/rational/rational.html)：分子・分母の正規形と正確な有理数の契約を確認。今回の演算はF-11のBigInt上に記述し、boost::rationalやそのソースは取り込んでいません。
- [Sean Eron Anderson: Bit Twiddling Hacks](https://graphics.stanford.edu/~seander/bithacks.html#NextBitPermutation)：同じ1の個数を持つ次のbit patternの考え方を参照。公開APIは0と64bit端の終了規約を加えています。
- [Sedgewick / Wayne: LSD radix sort](https://github.com/kevin-wayne/algs4/blob/master/src/main/java/edu/princeton/cs/algs4/LSD.java)：byte単位の安定な数え上げと下位からのpassを参照。ソースはGPLであり取り込みは行わず、符号bit反転・キーの保存・添字の安定ソートとして別に実装しました。

ArenaはC++17のalignas・整列付き配列new・placement new・launderを使う型付き所有コンテナです。外部プールのコードを移植していません。Gridは既存grid_bfsの方向順と探索を共有。Rollbackは配列の旧値と履歴通番による独自のsnapshot契約を持ちます。図解で使用する仮想的なブロック・ログは実アドレスやC++オブジェクトの寿命を再現するものではありません。

[API・検証](README.md)
