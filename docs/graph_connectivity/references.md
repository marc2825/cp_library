# 参考資料・設計判断

2026-10-15 に以下の一次資料を確認しました。SCC は原理を確認して独自の反復 Kosaraju 法で追加し、Lowlink・連結成分・二部グラフは本リポジトリの既存実装を整理しました。外部ライブラリのコードを取り込んだものではありません。

| 参照先 | 確認した点・設計判断 |
|---|---|
| [AtCoder Library: SCC](https://atcoder.github.io/ac-library/production/document_ja/scc.html) | 成分一覧がトポロジカル順になる契約。こちらは共有 Graph を受け取り、頂点→成分番号と same/size も返す。成分内は昇順、再帰を避ける Kosaraju 法 |
| [CP-Algorithms: SCC / Condensation](https://cp-algorithms.com/graph/strongly-connected-components.html) | 終了順の逆順と逆グラフ、縮約が DAG になる理由。縮約辺は重複集約を既定とし、元辺 ID の代表を保存 |
| [CP-Algorithms: Bridges](https://cp-algorithms.com/graph/bridge-searching.html) | low と ord の厳密比較、多重辺で親辺だけを除外する必要性。論理辺 ID を利用 |
| [CP-Algorithms: Articulation Points](https://cp-algorithms.com/graph/cutpoints.html) | 根以外の ≤ と根の子数の特例。削除で成分数が増える定義と照合 |
| [CP-Algorithms: Bipartite Check](https://cp-algorithms.com/graph/bipartite-check.html) | 全成分を 0/1 彩色。自己ループ・奇閉路の失敗と多重辺の扱い |
| [CP-Algorithms: Connected Components](https://cp-algorithms.com/graph/search-for-connected-components.html) | 未訪問の根ごとに到達範囲を集める原理。既存のスタック探索を維持 |
| [Library Checker: SCC の一次仕様](https://github.com/yosupo06/library-checker-problems/blob/master/graph/scc/task.md) | トポロジカル順で全頂点を一度ずつ出力。独立した相互到達性の検証を用意 |
| [AOJ GRL_3_C](https://judge.u-aizu.ac.jp/onlinejudge/description.jsp?id=GRL_3_C) / [GRL_3_A](https://judge.u-aizu.ac.jp/onlinejudge/description.jsp?id=GRL_3_A) / [GRL_3_B](https://judge.u-aizu.ac.jp/onlinejudge/description.jsp?id=GRL_3_B) | SCC 同成分問い合わせ・関節点・橋の入出力。橋は出力形式に合わせて端点を整列 |
| [AtCoder ABC282 D](https://atcoder.jp/contests/abc282/tasks/abc282_d) | 二部グラフの色と無向連結成分を使う問題。同色ペアを引くのは同じ連結成分内だけ |

共有 Graph と辺 ID は 3.7・木・既存の二重連結分解で共用します。旧 graph/low_link.hpp 等を互換入口とし、根の特例や平行辺の扱いを変更しません。縮約 DAG では重みを集約せず 1 とし、original_edge は最小 ID を保証しません。

図解は SCC の2回の探索、low の伝播、比較条件、仮削除後の探索、彩色の衝突を表します。C++ が返さない途中状態も説明のために表示します。二部グラフの失敗時に奇閉路全体を復元する API は含みません。[検証方法](README.md#検証)。

3.7〜3.11 A補完の[一次資料・問題例・設計判断](../graph_a/references.md)。旧Radix Heap・SCC・Block-Cut・Dinic・BinaryLiftingは共通実装を利用しています。
