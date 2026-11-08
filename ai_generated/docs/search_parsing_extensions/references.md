# E/Xの一次資料と実装範囲

[一覧](README.md)。2026-10-29確認。原理・定義を参照し、外部ソースコードは転記していません。公開API、予算、例外復元、検算、検証と図解はこのリポジトリの規約に合わせて実装しています。

| 対象 | 一次資料 | 使用した原理・実装上の違い |
|---|---|---|
| E-06 | [chokudai本人による探索の説明](https://chokudai.hatenablog.com/entry/2017/04/12/055515) | 深さ別queueの保持と巡回。固定回数/ノード予算と深さ別の解保持を公開。 |
| E-21 Alias | [Keith Schwarz: Darts, Dice, and Coins](https://www.keithschwarz.com/darts-dice-coins/) | Voseの大小列構成。uint64重みと128bit中間積による整数閾値。 |
| E-21 非復元 | [Efraimidis, Weighted Random Sampling over Data Streams](https://arxiv.org/abs/1012.0256) / [Efraimidis・Spirakis原論文](https://www.sciencedirect.com/science/article/pii/S002001900500298X) | 重み付きkeyを順位付け。対数の指数時計とし、逐次比例抽出と包含確率の違いを明記。 |
| E-21 正規 | [Box・Muller原論文](https://projecteuclid.org/journals/annals-of-mathematical-statistics/volume-29/issue-2/A-Note-on-the-Generation-of-Random-Normal-Deviates/10.1214/aoms/1177706645.full) | 2つの一様乱数を半径と角度へ変換し2つの正規乱数を生成。予備値を保持。 |
| E-21 指数 | [NIST統計ハンドブック](https://itl.nist.gov/div898/handbook/eda/section3/eda3667.htm) | 指数分布のCDF/逆関数。切断CDFは条件付けから導出しexpm1/log1pで計算。 |
| X-01 | [LLVM公式のparser tutorial](https://releases.llvm.org/8.0.1/docs/tutorial/LangImpl02.html) | 再帰下降と優先順位。独自の整数式grammar、postorder AST、位置付きエラー。 |
| X-02 | [Cornell講義: DFA minimization](https://www.cs.cornell.edu/courses/cs412/2008sp/lectures/lec04.pdf) | 受理状態から分割し遷移先の同値類で細分化。Moore法の計算量を明記。 |
| X-03 | [Russ Cox: Regular Expression Matching Can Be Simple And Fast](https://swtch.com/~rsc/regexp/regexp1.html) / [Virtual Machine Approach](https://swtch.com/~rsc/regexp/regexp2.html) | Thompson構成とε閉包simulation。バイトalphabet、文字集合、状態/深さ予算を追加。 |

E-01/E-02は変更履歴とdense/inverse配列の不変条件、E-03は削除/追加する巡回辺の差、E-04は中点CDFの量子化誤差、E-05は共有prefixのDFS、E-20はrun別seedとthread joinを直接設計し、独立実装と比較しています。E-04の誤差式は[個別説明](fast_annealing_acceptance.md)、実測は[benchmark](benchmark.md)に記録しています。最悪計算量や数値近似を高速性・最適性の保証と取り違えないように区別しています。
