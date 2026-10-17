# 一次資料・問題例と設計判断

2026-10-17確認。アルゴリズムの原理・契約を参照し、このリポジトリの既存部品とAPIに合わせて実装しています。下記のライブラリへの実行時依存はありません。

| 対象 | 資料・問題 | 採用した設計 |
|---|---|---|
| D&C/Knuth | [KACTL KnuthDP](https://github.com/kth-competitive-programming/kactl/blob/main/content/various/KnuthDP.h)、[AtCoder DP N](https://atcoder.jp/contests/dp/tasks/dp_n) | 最左argmin、分割の復元。D&Cは既存Monotone Minima、Knuthは既存IntervalDPResultを共有。 |
| CHT/Li Chao DP | [KACTL LineContainer](https://github.com/kth-competitive-programming/kactl/blob/main/content/data-structures/LineContainer.h)、[AtCoder DP Z](https://atcoder.jp/contests/dp/tasks/dp_z) | 旧CHTと新Li Chaoを再実装せず、同じ二乗距離DPで比較。 |
| Slope Trick | [Nyaan実装](https://github.com/NyaanNyaan/library/blob/master/data-structure/slope-trick.hpp) | 単位hinge二heap。オフセット・最小値・区間shift・非破壊mergeを公開。 |
| WQS | [OI Wiki WQS](https://oi-wiki.org/dp/opt/wqs-binary-search/)、[IOI公式2016タスク](https://ioinformatics.org/page/ioi-2016/42) | 離散凸性、整数支持線、同値なら個数が多いoracle、端点のbracketを明記。 |
| φ/μ | [NIST DLMF 27.2](https://dlmf.nist.gov/27.2)、[27.3](https://dlmf.nist.gov/27.3)、[AOJ NTL_1_D](https://judge.u-aizu.ac.jp/onlinejudge/description.jsp?id=NTL_1_D) | 線形SPFを共有した篩と既存64bit因数分解による単一値API。 |
| CRT/floor_sum | [ACL公式仕様](https://atcoder.github.io/ac-library/production/document_ja/math.html)、[実装](https://github.com/atcoder/ac-library/blob/master/atcoder/math.hpp)、[practice2 C](https://atcoder.jp/contests/practice2/tasks/practice2_c) | CRTはoptionalとlcm超過検査、floor_sumは負係数とcheckedな返却値。ACLのfloor_sumのwrap契約とは異なる。 |
| Garner | [既存の固定3法実装](../../convolution/convolution_mod.hpp)、[ACL CRT](https://atcoder.github.io/ac-library/production/document_ja/math.html) | 任意長の混合基数、各法の係数/定数だけを保持し積全体を作らない。出力法に逆元を要求しない。 |
| 原始根 | [原始根の定義と判定の解説（著者記事）](https://exploringnumbertheory.wordpress.com/2013/09/03/defining-primitive-root/)、[Library Checker](https://judge.yosupo.jp/problem/primitive_root) | 素数検査とp-1の因数分解を共有し、最小の正の原始根を返す。 |
| Stirling/Bell | [NIST DLMF 26.8](https://dlmf.nist.gov/26.8)、[26.7](https://dlmf.nist.gov/26.7) | 第1種は既定で符号なし、符号付きも選択可。BellはO(N)領域のBell三角形。 |
| 分割/Montmort | [NIST DLMF 27.14](https://dlmf.nist.gov/27.14)、[26.13](https://dlmf.nist.gov/26.13)、[Partition Function](https://judge.yosupo.jp/problem/partition_function)、[Montmort Number](https://judge.yosupo.jp/problem/montmort_number_mod) | 五角数定理、逆元不要の完全順列漸化式。合成数法も可。 |
| Burnside/Pólya | [CMU K. Sutnerの講義資料](https://www.cs.cmu.edu/~cdm/resources.html) | 固定点平均とcycle indexを共有。群の完全列挙、mod平均の可逆性、重み代入の範囲を明記。 |

参考資料側のAC表示をこの追加コードのACとは数えません。[検証プログラム](../../verify/math_a/)はローカル検証済み範囲として管理します。
