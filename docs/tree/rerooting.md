---
title: 全方位木DP（Rerooting）
documentation_of: //tree/rerooting.hpp
---

# 全方位木DP（Rerooting）

全ての頂点を根としたときの木DPを求めます。まず子→親の状態を作り、次に親→子へ情報を返します。累積積で一つの寄与を除くため、逆元なしで実行できます。

[カテゴリ索引](README.md) / [操作できる図解](../../visualizations/tree/index.html#rerooting) / [実装](../../tree/rerooting.hpp) / [参考資料](references.md)

## usage

```cpp
#include <iostream>
#include "tree/rerooting.hpp"
struct State { long long count = 0, sum = 0; };
int main() {
    cp::Graph<> g(4);
    g.add_edge(0, 1); g.add_edge(1, 2); g.add_edge(1, 3);
    auto answer = cp::rerooting(g, State{},
        [](State a, State b) { return State{a.count + b.count, a.sum + b.sum}; },
        [](State a, int) { ++a.count; return a; },
        [](State a, const cp::Edge<long long>& e) { a.sum += a.count * e.cost; return a; });
    for (auto s : answer) std::cout << s.sum << ' ';
    std::cout << '\n';
}
```

[動く使用例](../../examples/tree/rerooting.cpp)。リポジトリのルートを include パスに指定します。

## API・使用条件

```text
rerooting(g, identity, merge, add_vertex, lift, root=0) -> vector<R>
```

コールバックと型・辺の向きは [Tree DP](tree_dp.md) と共通です。ただし merge は**結合的**で、identity は左右両方の単位元である必要があります。可換性・逆元は不要です。各頂点で **g[v] の隣接順** を保ち、親側の寄与もその辺の元の位置へ入れます。

戻り値 answer[v] は v を根としたときの状態で、森では v の成分だけを対象にします。root は最初の下向きDPに使う根です。正しいモノイド／コールバックなら answer はこの root に依存しません。

S はコピー・代入可能、R はデフォルト構築・コピー・代入可能。add_vertex は同じ頂点に複数回呼びます。特定の呼び出し回数／順番に依存せず、渡された引数だけから結果を計算してください。入力や渡された状態を変更しないでください。

[RootedTree の入力・距離算術条件](rooted_tree.md)を引き継ぎます。空では空列でコールバックなし。自己ループ・多重辺・閉路・有向入力は invalid_argument。入力の Graph を変更しません。

## 原理・計算量・注意点

各コールバック・状態のコピー／構築が O(1) なら時間 O(N+E)、空間 O(N)。頂点次数 d ごとに incoming[d]・prefix[d+1] を一時的に作り、suffix は右から一つずつ集約します。星型でも各辺の処理回数は定数です。

```text
prefix[0] = identity
prefix[i+1] = merge(prefix[i], incoming[i])
suffix[d] = identity
suffix[i] = merge(incoming[i], suffix[i+1])
exclude(i) = merge(prefix[i], suffix[i+1])
```

除いた状態へ v 自身を足して、v→子 の辺で変換します。suffix では `merge(incoming[i], suffix)` の順序が重要です。

使用例は各頂点からの距離の総和。状態 `{count,sum}` を成分ごとに足し、頂点追加で count を1増やし、辺を越えると sum に count×cost を足します。負辺も扱えます。count×cost と和の全中間値が型に収まるようにします。DP 自身の算術はコールバックの責任です。

## 使用する問題・検証

[EDPC V - Subtree](https://atcoder.jp/contests/dp/tasks/dp_v)。法が合成数でも、積と単位元で処理でき、逆元は使いません。[ジャッジ形式](../../verify/tree/subtree.test.cpp)を用意。

全根から独立に再計算する距離和・順序付き文字列DP、S≠R、親の位置が隣接列の途中のケース、送受信方向、非連結、0／負重み。20万頂点のパス・星型とコールバック回数の線形上限。

共通の [検証入口](../../verify/run_tree.py) は C++17・警告エラー・debug/release・単独 include・複数翻訳単位・使用例・独立オラクル・ジャッジ入出力を確認します。`--sanitize` で ASan/UBSan。**オンライン AC は未確認**です。
