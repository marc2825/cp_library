---
title: Sweep Line による静的長方形和
documentation_of: //offline_query/sweep_line.hpp
---

# Sweep Line: 静的な点の長方形和

平面上の重み付き点と長方形を先に集め、各長方形に入る点の重みの和を求めます。x の条件は走査線で、y の条件は座標圧縮した Fenwick Tree で処理します。

[操作できる図解](../../visualizations/offline_query/index.html#sweep) は点・境界の走査順・圧縮 y・BIT の担当区間・答えの部分和を表示します。

## API

```text
cp::static_rectangle_sum(vector<cp::WeightedPoint<Coord,Weight>> points,
                         const vector<cp::Rectangle<Coord>>& rectangles)
  -> vector<Weight>（長方形の入力順）
```

[型と数値の条件](rectangle.md): 点は `{x,y,weight}`、長方形は `{left,down,right,up}`。**両軸とも半開区間**です。空・逆向きなら 0、点がなければ全て 0、問い合わせがなければ空を返します。同一座標は加算し、負の重みも扱います。

座標 Coord はコピー可能で `<` が全順序を与えること、Weight は `Weight(0)`, `+=`, `-` を持つ正確な加法可換群であることが条件です。逆元・BIT 内部・部分和を含む中間値も型に収めてください。浮動小数の丸め誤差は保証しません。点・長方形はそれぞれ 1 億件以下を前提とします。

点は値渡しして内部でソートします。元の vector をそのまま渡せば元の順序は変えず、不要なら `std::move(points)` で渡せます。長方形は変更しません。依存ヘッダは座標圧縮と Fenwick Tree です。

## usage

```cpp
#include <iostream>
#include <vector>
#include "offline_query/sweep_line.hpp"

int main() {
    std::vector<cp::WeightedPoint<>> points{{1, 2, 4}, {3, 2, 7}, {1, 2, -1}};
    std::vector<cp::Rectangle<>> rectangles{{1, 2, 3, 3}, {0, 0, 4, 3}, {3, 2, 3, 3}};
    auto answer = cp::static_rectangle_sum(points, rectangles);
    for (auto x : answer) std::cout << x << '\n'; // 3, 10, 0
}
```

[動く例](../../examples/offline_query/static_rectangle.cpp)

## 原理・計算量

`G(x,d,u)=Σ(px<x, d≤py<u) weight` と置けば、答えは `G(right,down,up)-G(left,down,up)` です。各長方形を二つの x 境界イベントに分けてソートし、その境界より x が厳密に小さい点だけを BIT に追加します。y の両境界は lower_bound により `[d,u)` の圧縮位置に変換します。

同じ x のイベント間の順番は問いません。同じ x の点を先に追加すると右境界上の点を数えてしまうため、比較は `point.x < event.x` です。`left-1` 等の座標演算は行わず、LLONG_MIN の境界も扱えます。

P=点数、Q=長方形数、Y=点の異なる y 数として、時間 **O(P log(P+1)+Q log(Q+1)+(P+Q) log(Y+1))**、メモリ **O(P+Q)**。座標比較・重み演算は O(1) としています。dense な座標平面の大きさには依存しません。呼出し元が保持する入力は別途必要です。

## 問題・検証・参考

[Library Checker: Rectangle Sum](https://judge.yosupo.jp/problem/rectangle_sum) に [ジャッジ形式](../../verify/offline_query/rectangle_sum.test.cpp) を用意しています。問題 ID は `rectangle_sum` です。点追加を含む場合は [CDQ 版](cdq_rectangle_sum.md) を参照してください。

300 履歴から作った静的点集合を全長方形について素朴和と比較し、625 格子長方形・同一座標・四辺の境界・負の重み・極値座標・空入力・独自型・20 万点を検証します。[検証範囲](README.md)。

[Library Checker の一次仕様](https://github.com/yosupo06/library-checker-problems/blob/master/data_structure/rectangle_sum/task.md) と [時刻を加えて拡張する CDQ の解説](https://robert1003.github.io/2020/01/31/cdq-divide-and-conquer.html) を参照しています。[参考資料](references.md)。
