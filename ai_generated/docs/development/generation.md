# テストケース生成ライブラリ

[機能検索カタログ](../../FEATURES.md) / [JSON・CLI](generation_json.md) / [実行例22項目](../../examples/development/generation/README.md) / [既存API](random_data.md) / [差分テスト](stress.md)

Python 3.10以上、追加パッケージ不要。以下は `ai_generated/` を作業ディレクトリとして実行します。
別のディレクトリから使う場合は、このディレクトリを `PYTHONPATH` に追加するか、絶対パスの `tools/cplib.py` を実行してください。

```python
from tools.cp_tools.generators import Random, random_int_tuple, random_interval, random_array
from tools.cp_tools.formatting import format_case

rng = Random(42)
n, m, k = random_int_tuple(rng, [(1, 100), (0, 100), (1, 1000)])
l, r = random_interval(rng, 1, n)
a = random_array(rng, n, 1, 1000)
print(format_case({"n": n, "m": m, "k": k, "a": a},
                  [{"line": ["n", "m", "k"]}, {"line": ["a"]}]), end="")
```

## 構成

| 層 | 実装 | 役割 |
|---|---|---|
| 乱数 | [random_data.py](../../tools/cp_tools/random_data.py) | 既存SplitMix64、任意精度整数、整数重み付き選択 |
| 基本生成 | [generators.py](../../tools/cp_tools/generators.py) | スカラー、タプル、配列、文字列、ケースサイズ割当 |
| 構造生成 | [structures.py](../../tools/cp_tools/structures.py) | 行列、グリッド、木、疎グラフ、区間、クエリ |
| 制約・JSON・CLI | [generation.py](../../tools/cp_tools/generation.py) | 依存順生成、式検査、総予算、再生成 |
| 出力 | [formatting.py](../../tools/cp_tools/formatting.py) | 同一行、別行、辺リスト、T付き・EOF形式 |
| 検索 | [feature_catalog.py](../../tools/cp_tools/feature_catalog.py) | 公開機能収集、日英検索、更新・整合性検査 |

## 整数・複数スカラー

`random_int(rng, low, high)` は両端を含み、64bitを超える整数も扱います。
`multiple=2` で偶数、奇数は `2*rng.integer(a,b)+1` などで構成します。
`rng.choice(values)`、`rng.weighted_choice(values, weights)` は候補選択で、重みは非負整数、総和は正が必要です。
`random_float(rng, low, high)` は有限float、`random_fixed(rng, low, high, digits=2)` は指定桁の `Decimal`、
`random_bool`、`random_char` も利用できます。固定小数点の境界を厳密にしたい場合は文字列で渡します。

```python
from tools.cp_tools.generators import Random, random_int, random_ordered_tuple, random_int_tuple, random_interval
from math import gcd
rng = Random(42)
huge = random_int(rng, -(10**100), 10**100)
a, b, c = random_ordered_tuple(rng, 3, 1, 10**9, strict=True)
l, r = random_interval(rng, 1, 100)  # 1 <= l <= r <= 100
x, y = random_int_tuple(rng, [(1, 100)] * 2, predicate=lambda t: gcd(*t) == 1)
```

`random_ordered_tuple(..., strict=False)` は等値を許します。`random_interval(..., half_open=True)` は
**空でない** `[l,r)` で、右端も `high` 以下です。配列添字なら `low=0, high=n` を使います。
`length` は閉区間では要素数、半開区間では `r-l` です。
任意述語付きタプルは最大 `attempts=1000` 回の棄却法です。予算切れは不可能の証明ではありません。
順序・総和のような低確率条件には構成的APIを使ってください。

## 配列・タプル列

```python
from tools.cp_tools.generators import Random, random_array, random_array_with_sum, random_records
rng = Random(42)
a = random_array(rng, 100, 1, 1000, unique=True)
b = random_array_with_sum(rng, 100, 0, 1000, total=5000)
c = random_array_with_sum(rng, 100, -10, 10, sum_min=-100, sum_max=200)
d = random_array(rng, 100, 1, 1000, distinct_count=3)
records = random_records(rng, 10, [lambda r: r.integer(1, 100), lambda r: r.choice("abc")])
```

相異なる配列は値域全体を列挙せずに抽出します。総和は各ステップで残り要素が実現できる範囲を計算し、最後にシャッフルします。
`n*low <= total <= n*high` を検査します。総和APIとunique・順序・distinct_countの同時指定は未対応です。
Pythonでは頻度指定の配列、周期・交互列、等差・等比数列はリスト構築と `rng.shuffle` を組み合わせて作れます。

| mode | 整数 | 配列・文字列 |
|---|---|---|
| uniform / random | 一様整数 | 独立生成。uniqueなら重複なし抽出 |
| minimum / maximum | 許される最小・最大 | 全要素が端点 |
| boundary | 端点とその隣 | 各要素を境界寄りに生成 |
| small / large | 下限・上限の10以内 | 各要素に適用 |
| zero_heavy | 0に重みを追加 | 数値配列向け。文字列ならalphabet先頭を多く選ぶ |
| power2 / power10 | 正負の累乗と±1 | 各要素に適用 |
| all_equal | — | 全要素同一 |
| unique | — | 全要素相異 |
| sorted / reversed | — | 昇順・降順（unique併用で狭義） |
| almost_sorted | — | ソート後に約1%のswap |
| few_distinct | — | 最大3種類。distinct_countで正確な種類数 |
| alternating | — | 最大2種類を交互に配置 |

整数・配列のmodeにリストを渡すと、呼び出しごとに1モードを選びます。全組合せが実現可能とは限らず、
uniqueとall_equalの矛盾などはエラーにします。文字列固有モードとの混合リストは未対応です。

## 文字列

```python
from tools.cp_tools.generators import Random, random_string, random_anagram_pair, random_brackets
import string
rng = Random(42)
s = random_string(rng, 100, string.ascii_lowercase)
p = random_string(rng, 101, "abc", mode="palindrome")
q = random_string(rng, 100, "01", mode="no_adjacent")
c = random_string(rng, 10, "abc", counts={"a": 3, "b": 7})
s, t = random_anagram_pair(rng, 100, "abc")
brackets = random_brackets(rng, 50, mode="nested")
```

alphabetには `string.ascii_uppercase`、`string.ascii_letters + string.digits`、`string.digits`、`"01"`、日本語も使えます。
重複文字は除去します。固有モードは `palindrome / non_palindrome / no_adjacent / periodic`。
periodicは長さ最大3のパターンを繰り返します。固有モードとunique・distinct_countは併用できません。
countsは単独指定です。括弧列のmodeは `random / nested / alternating` で、常に正しい列を返します。
特定部分文字列の禁止・編集距離指定・複数文字種の括弧制約は未対応です。

## 行列・グリッド

```python
from tools.cp_tools.generators import Random
from tools.cp_tools.structures import random_matrix, random_grid
rng = Random(42)
binary = random_matrix(rng, 5, 8, 0, 1, ones=10)
symmetric = random_matrix(rng, 5, 5, -10, 10, mode="symmetric")
row_sums = random_matrix(rng, 3, 5, 0, 10, row_sums=[10, 20, 30])
grid = random_grid(rng, 5, 8, walls=10, connectivity="reachable")
```

行列は `symmetric / identity / diagonal / upper / lower / permutation` の正方構造に対応します。
onesは0/1行列の正確な1の個数、row_sumsは行ごとの総和です。同時の行和・列和指定は未対応です。
文字行列は `random_string` を高さ分呼んで構成します。

グリッドのSは左上、Gは右下、4近傍です。`walls` は常に正確な個数です。
`reachable` は単調経路を予約し、`unreachable` はSの隣を壁にします。
`unique_path` は単調経路以外をすべて壁にするため `walls = H*W-(H+W-1)` が必要です。
実現可能な全グリッドを生成できるとは限りません。例えば reachable の壁数上限は予約経路で制限されます。
`start_goal=False, connectivity="any"` ならS/Gなしの壁・通路グリッドです。
島数、任意S/G位置、一般迷路、到達不能グリッドでの最小壁数探索は未対応です。

## 木・グラフ

```python
from tools.cp_tools.generators import Random
from tools.cp_tools.structures import random_tree, random_graph, random_dag, random_bipartite_graph, weighted_edges, parent_array
rng = Random(42)
t = random_tree(rng, 100, mode="uniform")
parents = parent_array(t, root=0)  # rootは-1
connected = random_graph(rng, 100, 150, connected=True)
disconnected = random_graph(rng, 100, 150, components=3)
dag = random_dag(rng, 100, 150)
bipartite = random_bipartite_graph(rng, 40, 60, 150)
weighted = weighted_edges(rng, connected, -100, 100)
```

木の `random` は既存の再帰木、`uniform` は一様なPrüfer列によるラベル付き木です。
`path / star / binary` はラベルをランダム化した構造です。森は既存 `random_data.forest` を使います。
`Graph(n, edges, directed)`、既存の `tree / forest / undirected_graph / dag / simple_polygon` は維持しています。
旧 `undirected_graph / dag` は小・中規模向けの候補列挙方式なので、大規模には新 `random_graph / random_dag` を使います。

新グラフは整数の辺順位から抽出し、候補辺を全列挙しません。`directed / loops / multi` に対応します。
有向 `connected=True` は弱連結です。componentsは無向のみで、大きな1成分と孤立点から構成します。
空グラフは連結とは主張せず `Graph(0, ())` を返します。完全グラフは最大辺数を指定できます。
強連結、指定SCC数、任意次数列、負閉路の有無を保証する重み付き生成は未対応です。
負重みの指定だけでは負閉路の有無は保証しません。

## 区間・クエリと拡張

```python
from tools.cp_tools.generators import Random
from tools.cp_tools.structures import random_intervals, random_queries, stateful_queries
rng = Random(42)
intervals = random_intervals(rng, 100, 0, 1000, half_open=True)
queries = random_queries(rng, 1000, 100, kind="set")
state = []
custom = stateful_queries(rng, 10, state, lambda st: ["append"],
    lambda st, op, engine: (st.append(1) or (op, len(st))))
```

kindは `point / range / union_find / set / multiset / stack / queue / edges`。
pointは `(0,index,value)` と `(1,index)`、rangeは `(0,l,r,value)` と `(1,l,r)`、
union_findは `(0または1,u,v)` です。添字は0始まり、区間は半開です。
状態付きの操作名は `insert / erase / push / pop` で、空からのpopや存在しない値・辺のeraseを生成しません。
edgesは無向単純グラフの追加・削除です。JSON出力に数値操作コードが必要ならPythonで変換してください。
`stateful_queries` のavailableとtransitionで独自操作を登録でき、transitionが内部状態を更新して出力レコードを返します。
動的文字列や複雑な状態機械は利用者のcallbackで組み立てます。

## レイアウトと複数ケース

`format_case(values, layout)` は乱数を消費しません。同じ辞書を別の行構成で出力できます。

| 操作 | 例 | 出力 |
|---|---|---|
| 同一行 | `{"line": ["N", "M", "K"]}` | `N M K` |
| 配列 | `{"line": ["A"]}` | 空白区切り1行 |
| 別行 | `{"lines": "A"}` | 1要素1行 |
| レコード列・行列・グリッド | `{"rows": "records"}` | 1レコード1行 |
| 辺 | `{"edges": "G", "one_based": true}` | 最初の2列に1を加算、重み列は保持 |
| 番兵 | `{"literal": "0 0"}` | 指定行 |

任意長タプル・整数と文字列の混在を扱います。1行に入れる配列は1段だけ平坦化します。
空配列のlineは空行、空のrowsは0行。空白を含む文字列や改行をトークンにするとエラーです。
`format_cases(..., t_prefixed=False)` はEOF形式、`t_prefixed=True` は先頭にTを出します。
隣接リスト・親配列・疎な(index,value)列・頻度表・ランレングスはPythonでレコードにして同じ出力APIを使えます。

`random_sizes(rng, T, low, high, sum_max=..., sum_min=..., total=...)` でサイズを構成します。
`cost_max=K` は `sum(N**2) <= K`。これはsum_maxとだけ併用可能です。
Tが固定なら `T*low > sum_max` などの不可能条件を検出します。棄却法は使いません。
JSONでの複数サイズ予算・個別再生成は [設定仕様](generation_json.md) を参照してください。

## 再現性・計算量・対応範囲

seedは従来どおり下位64bitを使います。同一バージョン・同一seed・同一API呼び出し順で再現します。
`below` の64bit制限と従来範囲の `integer` の乱数消費順は維持しました。任意精度は `randbelow / bits` を利用します。
新APIのアルゴリズムや呼び出し順の将来変更は結果を変え得ます。長期保存には入力本体・設定・ライブラリ版も保存してください。

| 処理 | 時間 | 追加領域（出力を含む） |
|---|---|---|
| 配列・文字列・総和割当・クエリ | O(n)（orderingはO(n log n)） | O(n) |
| 相異抽出 | 期待O(k)、値域全体の列挙なし | O(k) |
| 行列・グリッド | O(HW) | O(HW) |
| 再帰木・構造木 | O(n) | O(n) |
| Prüfer木 | O(n log n) | O(n) |
| 疎グラフ | O(n + m log(m+1)) | O(n+m) |
| DAG・二部グラフ | 期待O(n+m) | O(n+m) |

整数演算の費用はビット長に依存し、PRNGの棄却は期待定数回です。巨大整数のpowerモードは桁数にも比例します。
出力は現在ケース全体をメモリ上に持ちます。ストリーミング、一般制約ソルバー、制約を保つ縮小器は未実装です。
高度な数論サンプラー・一般幾何配置は未実装で、幾何は既存の放物線上の凸多角形を再利用します。

## 検証

```sh
make verify-generation
python3 tools/cplib.py catalog --check
python3 verify/generation/benchmark.py
```

[独立した性質検査](../../verify/generation/test_generation.py) で、BFSによる連結・到達可能性、
トポロジカルソート、括弧残高、操作列の再実行、総和、seed、CLIとAPIの一致、不可能条件を検査します。
ベンチマークは [実測記録](generation_report.md) を参照してください。全組合せやオンラインACを保証する記録ではありません。
