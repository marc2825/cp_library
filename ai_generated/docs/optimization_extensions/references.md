# O/Bの一次資料と設計判断

[カテゴリ](README.md) / [図解](../../visualizations/optimization_extensions/README.md)

数式・APIの確認に使用した一次資料です。外部の実装コードは新規移植せず、既存の任意精度・行列・独立性oracleを共有しました。

- [University of Waterloo: Phase 1 Simplex Method](https://www.eng.uwaterloo.ca/~syde05/phase1.pdf)：負の右辺と人工変数による実行可能性の回復。本実装はその後に元目的へ切り替え、主双対・Farkas・非有界方向を復元します。
- [Norbert Zeh: Cycling and Bland’s Rule](https://web.cs.dal.ca/~nzeh/Teaching/4113/book/simplex/cycling.html)：最小添字で入出基底を決める規則と、最悪反復数が多項式ではないこと。浮動版には別途epsilonと予算を設けます。
- [Michel Goemans: Matroid Intersection lecture notes](https://math.mit.edu/~goemans/18453S17/matroid-intersect-notes.pdf)：交換グラフ・増加路・matroid union。既存のvector<int>独立性oracleへ合わせています。
- [Ta-Wei Tu: Subquadratic Weighted Matroid Intersection Under Rank Oracles](https://arxiv.org/abs/2212.00508)：重み付き交換グラフと、最短路の同値時に辺数を最小にする条件。本実装はrank oracleの高速法ではなく、独立性oracleとBellman–Fordを使います。
- [Werner Dinkelbach: On Nonlinear Fractional Programming](https://pubsonline.informs.org/doi/10.1287/mnsc.13.7.492)：比率をパラメータ付き目的へ変換する方法。正の分母下限・大域oracle・停止条件を公開契約にしました。
- [Francis Bach: Learning with Submodular Functions](https://www.di.ens.fr/~fbach/submodular/) / [著者のLovász拡張の講義資料](https://www.di.ens.fr/~fbach/structured_sparsity_fbach.pdf)：基多面体の貪欲法と、[0,1]^n上の凸な拡張の最小化。実装は厳密LPのcutting-planeで、最悪指数時間と上下界を明示します。
- [Berarducci・Mantova: Transseries as germs of surreal functions](https://arxiv.org/abs/1703.01995)：ωを含む超現実数の部分体。本実装の対象はユーザー承認済みの有限有理関数Q(ω)に限定し、指数・対数・一般の無限級数は扱いません。

低次元LPは記号的な箱の頂点を列挙する構成です。最適化の原理は、非空の有界多面体上の線形目的が頂点で最適になることと、形式的なMの係数の順序比較です。整数計画はこのカテゴリの有理LPを緩和に用いる分枝限定法で、近似LPから整数の最適性を断定しません。
