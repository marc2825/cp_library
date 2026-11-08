---
title: 共有ランダムデータ生成器
documentation_of: //tools/cp_tools/random_data.py
---

# 共有ランダムデータ生成器（V-03）

[一覧](README.md) / [図解](../../visualizations/development/index.html#random) / [参考資料](references.md)

## 使用例

```python
from tools.cp_tools.random_data import Random, forest, undirected_graph, dag, simple_polygon

rng = Random(42)
print(forest(rng, 7, 3).text())
print(undirected_graph(rng, 8, 10, components=2).text(one_based=True))
print(dag(rng, 5, 7).edges)
print(simple_polygon(rng, 6, bound=20))
```

リポジトリルートから実行します。同じ[使用例ファイル](../../examples/development/random_data.py)は `python3 -m examples.development.random_data` で動きます。[生成器の外部コマンド例](../../examples/development/generate.py)はseedを第1引数に受け取り、[stress tester](stress.md)へそのまま接続できます。

## API

| API | 条件・結果 |
|---|---|
| `Random(seed)` | seedをmod 2^64で正規化。SplitMix64の状態を保持。 |
| `next_u64()` / `below(n)` / `integer(lo,hi)` | uint64乱数、0以上n未満、両端含む整数。1≤n≤2^64。剰余バイアスを棄却法で除く。 |
| `shuffle(list)` / `sample(values,k)` | in-place Fisher–Yates / コピーから重複なし抽選。 |
| `tree(rng,n)` | n≥0。ラベルをshuffleした後、各頂点をそれ以前のランダムな親へ接続。 |
| `forest(rng,n,components)` | 木からn−components辺を選ぶ。空グラフは(0,0)。 |
| `undirected_graph(rng,n,m,components=None,multi=False,loops=False)` | m辺を生成。components指定なら正確にその成分数、1で連結、2以上で非連結。multiで平行辺、loopsで自己辺を許す。 |
| `dag(rng,n,m)` | ランダムな頂点順で前から後ろへ辺を選ぶ。平行辺・自己辺なし。 |
| `simple_polygon(rng,n,bound=100)` | 3≤n≤2bound+1。整数座標の厳密凸多角形を反時計回りに返す。 |

グラフはimmutableな `Graph(n, edges, directed=False)`。edgesは0-index頂点ペアのtupleです。`text(one_based=False)` は先頭にn,m、以後に各辺を出力します。Graphの直接構築は検証しませんが、生成関数は不可能な頂点・辺・成分数をValueErrorにします。

## 分布・再現性

生成順とPRNGを自前で固定しているため、同じAPI呼び出し列とseedから同じ結果を得ます。全ての木・グラフの一様分布ではありません。成分数指定グラフは1つの大きい成分と残りの孤立頂点に分け、ラベルをshuffleします。この構成で可能な最大辺数まで生成できますが、均等な成分サイズは生成しません。森では木の辺を削除するため多様な成分サイズが得られます。

多角形は異なる整数xを抽選して `(x,x²)` 順に結び、開始点を回転します。凸性により単純多角形ですが、凹多角形を含む分布ではありません。座標は |x|≤bound、0≤y≤bound²。

## 計算量・検証・問題例

木・森はO(n)時間/空間。グラフ・DAGは候補辺を列挙するためO(n²+m)時間、O(n²+m)空間です。多角形はO(bound+n log n)時間、O(bound+n)空間。大量頂点の疎グラフを生成する用途より、小中規模の差分テスト向けです。

120 seedの木/森/単純・多重グラフ/DAGを独立した探索とトポロジカル除去で確認し、小さい全成分数・実現可能辺数も検査します。100多角形は全ての辺に対する他頂点の向きを確認します。SplitMix64既知値、seed正規化、範囲エラー、Python/図解の一致を確認します。問題例は連結成分、木DP、トポロジカル順、凸多角形の面積・包含です。
