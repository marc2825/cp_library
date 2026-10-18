---
title: CSR Graph・静的隣接リスト
documentation_of: //graph/csr_graph.hpp
---

# CSR Graph・静的隣接リスト

静的な隣接リストを、offset配列と連続したarc配列で保持します。元の辺は1回だけ保存し、無向辺の二方向は同じ論理辺IDを共有します。各頂点内のarcは入力順です。

## 使用例

```cpp
#include <iostream>
#include "graph/csr_graph.hpp"
int main() {
    cp::CSRGraph<int> graph(3, {{0, 1, 5}, {1, 1, 2}, {0, 1, 7}});
    std::cout << graph.edge_count() << " " << graph.arc_count() << "\n";
    for (const auto& arc : graph[1]) {
        std::cout << arc.idx << " " << arc.rev << "\n";
    }
    std::cout << graph[2].empty() << "\n";
}
```

出力:

```text
3 6
0 0
1 4
1 3
2 1
1
```

## API・使用条件・計算量

実体は`graph/csr_graph.hpp`、`graph_basic/csr_graph.hpp`も共通入口です。`CSRGraph<T=long long>`、頂点・論理辺ID・arc番号は0始まり。構築後の頂点／辺追加APIはありません。

| API | 意味 |
|---|---|
| `CSRGraph(int n=0, vector<Edge<T>> edges={}, GraphKind kind=Undirected)` | 辺列から構築。既定は無向 |
| `CSRGraph(const Graph<T>&)` | 既存Graphをコピーして圧縮 |
| `size()/edge_count()/arc_count()` | 頂点数／論理辺数／格納arc数 |
| `kind()/directed()` | 向きの規約 |
| `operator[](v)` | 読み取り専用ArcRange。begin/end/size/empty/添字アクセスが可能 |
| `edges()` | 入力順の論理辺列。入力Edge.idxは無視し、[0,m)へ振り直す |
| `arcs()/arc(i)` | 平坦なarc列／1arcへのconst参照 |
| `offsets()` | 長さn+1。頂点vのarcは[offsets[v],offsets[v+1]) |
| `to_graph()` | 既存Graphを新たに構築。頂点内順序・論理辺ID・重みを保持 |

各arcは`CSRArc<T>{from,to,cost,idx,rev}`。idxは論理辺ID、revは**平坦なarc配列**上の逆向きarc番号です。無向辺の自己ループも異なる二つのarcを作り、互いのrevを設定します。有向辺のrevは常に-1で、入力に逆向き辺があっても自動では対応付けません。多重辺を保持します。

負のn・不正なGraphKindはinvalid_argument、範囲外の頂点／arc／ArcRange添字はout_of_range。arc総数はINT_MAX以下（無向なら2m以下）を要求し、超過はlength_errorです。空グラフと孤立点に対応します。重みはT(1)で構築でき、コピー・代入可能な型。重みの符号や値そのものは制限しません。

ビュー・参照は元のCSRGraphの寿命に従います。代入・swap・ムーブで既存ビュー／参照を引き継ぐ保証はありません。コピーは独立し、ムーブ元は空の無向グラフになります。

構築O(n+m)時間・領域、頂点からのrange取得O(1)、全隣接走査O(n+m)、arc参照O(1)。論理辺列と各arcの重みを保持するため、転送先だけを保持する最小CSR表現より領域の定数倍が大きくなります。to_graphはO(n+m)のコピーと追加領域を使います。

既存のbfs/dijkstra等はGraph<T>を受け取るため、CSRGraphを直接渡すことはできません。独自の走査で`for (const auto& arc : graph[v])`を使うか、`cp::bfs(graph.to_graph(), source)`等で明示的に変換します。変換はゼロコピーではありません。

## 使用する問題と検証

静的グラフの反復走査、大きな疎グラフ、逆向きarcを辿る処理。自己ループ・多重辺・空・有向／無向、Graphとの往復、20万頂点の鎖を検証します。

`python3 verify/run_foundation_structures.py` / `--sanitize`（`make verify-foundation-structures`）で単独／重複include、全カテゴリとの混在・複数翻訳単位、Debug/Release、使用例・差分比較を検証します。オンライン提出・ACは未確認です。

[追加範囲と検証](../foundation_structures/README.md) / [参考資料](../foundation_structures/references.md) / [カテゴリ索引](README.md) / [操作できる図解](../../visualizations/foundation_structures/index.html#csr_graph) / [図解の範囲](../../visualizations/foundation_structures/README.md)
