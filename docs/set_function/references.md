# 一次資料・設計判断

2026-10-16に次の一次資料・公式仕様を確認しました。外部コード・説明文を転載せず、既存APIと集合の定義から整えています。

| 一次資料 | 確認した内容・今回の設計 |
|---|---|
| [CP-Algorithms: Submask Enumeration](https://cp-algorithms.com/algebra/all-submasks.html) | 降順列挙、0での終了、全maskとの組の3^n。uint64の全域と空集合の規約を明記 |
| [USACO Guide: SOS DP](https://usaco.guide/plat/dp-sos?lang=cpp) | 処理したビットだけを変える部分集合の分割。加算だけでなく可換・結合的merge版を提供し、単位元を不要にする |
| [Nyaan: Zeta / Moebius Transform](https://nyaannyaan.github.io/library/set-function/zeta-mobius-transform.hpp.html) | 部分集合／上位集合の更新方向と演算。逆変換はビットを逆順にし、整数の途中超過を検査 |
| [Library Checker: AND Convolution公式仕様](https://github.com/yosupo06/library-checker-problems/blob/master/convolution/bitwise_and_convolution/task.md) | 上位集合変換の使用例。部分集合変換も補集合で添字を反転したラッパにより同じ問題へ接続 |
| [CSES SOS Bit Problem](https://cses.fi/problemset/task/1654) | 部分集合・上位集合の頻度と、補集合を使った共通ビットの個数 |
| [Codeforces 165E: Compatible Numbers](https://codeforces.com/problemset/problem/165/E) | maxによるSOSと、補集合内で条件を満たす要素を返す使用例 |

3.12からSubmask・部分集合ゼータ／メビウスの本体を移し、旧includeを保持します。整数の検査は[3.14の内部算術補助](../../number_theory_convolution/detail/arithmetic.hpp)を明示的に再利用し、素数篩や実行時の前計算は必要ありません。

標準整数は途中の加減算も範囲内であることを要求します。打ち消し後の最終値だけが収まる場合を無条件に許しません。自由ビットの数が大きい列挙は指数時間で、変換と使い分けます。非可換なSOSや、max/minの逆変換は対象外です。CP-Algorithmsの文章はCC BY-SA 4.0です。今回外部コードは取り込んでいません。オンラインACは未確認です。
