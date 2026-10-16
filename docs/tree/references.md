# 一次資料と設計判断

確認日: 2026-10-16。既存の木実装を保持し、追加部分は本リポジトリの Graph・API 向けに独自実装しました。以下のコード・説明を読み、考え方と契約を比較しました。外部実装のコードを複製して取り込んではいません。

| 一次資料 | 確認した点 | この実装での判断 |
|---|---|---|
| [CP-Algorithms: LCA / Binary Lifting](https://cp-algorithms.com/graph/lca_binary_lifting.html) | 2のべき乗の祖先表、深さと祖先によるLCA | 既存の DoublingLCA を保持。根の祖先は -1、森・負辺・経路移動も扱う |
| [Luzhiled: Rerooting](https://ei1333.github.io/library/graph/tree/rerooting.hpp.html) | 下向きDPと左右の累積集約、辺を越える変換 | 再帰・専用隣接グラフを使わず、共有Graphで反復処理。頂点追加を独立させる |
| [Nyaan: Rerooting](https://nyaannyaan.github.io/library/tree/rerooting.hpp.html) | 子→親の変換と全根のDP、prefix/suffix | S/Rを分け、辺の from/to を情報の流れに一致させる。親の寄与も元の隣接位置へ置き、非可換演算を保証する |
| [Library Checker: LCA](https://github.com/yosupo06/library-checker-problems/blob/master/tree/lca/task.md) / [Jump on Tree](https://github.com/yosupo06/library-checker-problems/blob/master/tree/jump_on_tree/task.md) | 親配列の入力、パス上の0始まり位置、範囲外 -1 | 同形式の verify ラッパーを用意し、全パスの素朴解と照合 |
| [Library Checker: Tree Diameter](https://github.com/yosupo06/library-checker-problems/blob/master/tree/tree_diameter/task.md) | 重み付き直径、長さと復元パスの出力 | 長さ・端点・頂点列に加えて元辺ID列も保持。0重み・空にも仕様を付ける |
| [Library Checker: Vertex Add Subtree Sum](https://github.com/yosupo06/library-checker-problems/blob/master/tree/vertex_add_subtree_sum/task.md) | 一点加算と部分木和 | Euler Tour の preorder と既存の Fenwick を併用 |
| [CSES: Planets Queries I](https://cses.fi/problemset/task/1750) | 閉路を含む一意な遷移を k 回適用 | BinaryLifting を木とは独立した配列APIに。-1終端・上限指定・uint64全範囲にも対応 |
| [EDPC P: Independent Set](https://atcoder.jp/contests/dp/tasks/dp_p) | 白／黒の2状態の木DP | Tree DP の最小例とジャッジ形式。ラッパーは mod 10^9+7 |
| [EDPC V: Subtree](https://atcoder.jp/contests/dp/tasks/dp_v) | 全頂点の答えと任意の法 | 全方位DPの検証に使用。合成数や0を含む積でも逆元不要の方式を確認 |

## 演算の順序

各頂点の隣接リスト順をDPの定義に含めます。root の変更で親辺の位置が変わっても、全方位の答えは同じ順に集約します。逆向きの suffix 走査でも `merge(incoming[i],suffix)` とし、積の順序を逆転させません。独立した全根の文字列DPで検証しています。

辺のコールバックは「受信側から見た隣接辺」と解釈すると from/to を取り違えやすいため、必ず送信元→受信先としました。無向の cost と idx は両方向で共通です。方向ごとに別の値が必要なら from/to/idx を使って利用者の配列を参照できます。

## 検証の位置付け

`PROBLEM` は検証プログラムが対応する入出力形式を示します。外部ジャッジへの提出やオンライン AC を意味しません。ローカルで独立オラクル・通常／ASan/UBSan・debug/release・20万頂点を確認します。従来のグラフ／木／旧APIの回帰検証も実行します。
