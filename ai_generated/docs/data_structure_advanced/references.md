# D-20〜D-35の出典・設計判断

[API一覧・検証](README.md)。独自実装と外部ソースの改変を分け、性能上の選択を記録します。

- D-20: 群制約を `inverse(P[u])*P[v]` と定義し、既存加法DSUと結果enumを共有。付け替える根の重みは `weight[u]*difference*inverse(weight[v])`。
- D-21: 既存EulerTourTreeの全域森を使い、削除後の小成分から代替辺を走査。一般オンライン連結を実現しますがHDTではなく、木辺削除の最悪時間は線形です。
- D-22: [Basch–Guibas–Hershberger](https://doi.org/10.1145/258533.258573) のkinetic certificate。整数時刻・indexで同点処理する独自実装。
- D-23 / D-32: [Martínez–Roura](https://doi.org/10.1145/274787.274812) のサイズ比RBSTと不変ノードの経路コピー。D-23は恒等作用を渡す入口で、実装を複製しません。
- D-24: [jiry_2のBeats解説](https://codeforces.com/blog/entry/57319)。floor除算の商または差分が一様かをmin/maxで判定。任意の更新列に対する対数時間は保証しません。
- D-25: ブロック区間の最頻候補と出現位置の二分探索。多数決候補だけでは最頻値を求められないためRangeMajorityとは別実装。
- D-26: [tko919/library RangeLIS](https://github.com/tko919/library/blob/3925f75892c0705fb22d6e3f892ae1b5b4fac32d/DataStructure/rangelis.hpp) を **CC0-1.0** で改変。2026-10-24に固定commit `3925f75892c0705fb22d6e3f892ae1b5b4fac32d` のコードとLICENSEを取得しました。[ライセンス全文](../../verification/licenses/tko919-CC0.txt)、[出典台帳](../../verification/provenance.json)に取得版とhashを保存。既存WaveletMatrixへ置換し、任意値・同値の逆位置rank・Compare・空入力・const queryを追加。[ko_osagaの一次解説](https://codeforces.com/blog/entry/111807) でunit-Monge積も確認しました。ライセンスを確認できなかった別の実装からは取り込んでいません。
- D-27: 既存のMo/Fenwick完成例を関数へ抽出し、旧ジャッジラッパーも公開APIに接続。
- D-28 / D-29: [Wavelet Matrixの原論文](https://doi.org/10.1016/j.is.2014.06.002) の安定分割を、部分木長と1の数を持つ動的implicit treapへ接続。非圧縮の木なので静的succinct版より大きい領域が必要です。
- D-30: [Hinze–Paterson, Finger Trees](https://www.staff.city.ac.uk/~ross/papers/FingerTree.html)。1〜4要素のdigit、2/3要素のNode、再帰middleを直接実装。著者の訂正は遅延評価の重要性を指摘しており、strict C++で完全永続の両端ならしO(1)を主張しません。
- D-31: [Demaine–Iacono–Langerman, Retroactive Data Structures](https://people.csail.mit.edu/edemaine/papers/Retroactive_TALG/paper.pdf) のSection 4.4。prefix和0のbridge、bridge以降の削除済み最大要素、bridge以前の生存最小要素を使います。登録時刻・LazySegtreeによる独自実装。全履歴再生は検証oracle側だけです。
- D-33 / D-35: [Fenwickの原論文](https://doi.org/10.1002/spe.4380240306)。圧縮Fenwickを次元方向に再帰化し、2^D個のprefixを包除します。D-33は重みと個数で同じ公開実装を共有。
- D-34: [BentleyのKD-tree](https://doi.org/10.1145/361002.361007)。静的KDと更新buffer・世代番号・平方分割での再構築を組み合わせます。浮動小数点距離と最悪線形探索を明示。

図解は各APIの意味・集約量・更新の原理を操作できる参照モデルです。C++のポインタ配置・RBST乱数・Seaweed積・HDTを再現するものではありません。[図解の対応範囲](../../visualizations/data_structure_advanced/README.md)を参照してください。
