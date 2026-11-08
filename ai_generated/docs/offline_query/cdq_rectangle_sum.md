---
title: CDQ Divide and Conquer による点追加・長方形和
documentation_of: //offline_query/cdq_rectangle_sum.hpp
---

# CDQ Divide and Conquer: 点追加・長方形和

時系列に点追加と長方形和を記録し、最後に一括で答えます。各問い合わせが見るのは、その登録より**前**に追加した点だけです。同一座標への追加と負の差分を扱えます。オンラインで次の入力を決める問題にはそのまま使えません。

[操作できる図解](../../visualizations/offline_query/index.html#cdq) は時刻分割、左更新 → 右問い合わせの走査、BIT の巻き戻し、部分寄与を表示します。

## API

| API | 契約 |
|---|---|
| `cp::CDQRectangleSum<Coord=long long, Weight=long long>` | 空の操作列で構築 |
| `add_point(x,y,weight)` | その時刻に重みを加算。初期点は最初の問い合わせより前に登録 |
| `add_query(left,down,right,up)` / `add_query(Rectangle<Coord>)` | 半開長方形の和を登録し、0 始まり ID を返す |
| `query_count()` | 登録した問い合わせ数。空長方形も含む |
| `solve() const` | 登録 ID 順に `vector<Weight>` を返す。操作列を変更しない |
| `clear()` | 操作と ID をリセット。確保済み capacity は残る場合がある |

`solve` を繰り返しても答えは変わりません。操作を追記して再 `solve` でき、前の問い合わせに後の点が寄与することもありません。空・逆向きの長方形は 0、問い合わせなしは空、点なしは全て 0 です。[型・数値の条件](rectangle.md) は静的版と共通です。

空でない長方形 1 件につき 4 イベント、点追加 1 件につき 1 イベントを保持します。展開後イベント数・問い合わせ数は各 1 億件以下を前提とします。重みは逆元 `Weight(0)-w` を含む全中間値が型に収まる必要があります。例えば long long の最小値をそのまま重みにすると巻き戻しの逆元が収まりません。

## usage

```cpp
#include <iostream>
#include "offline_query/cdq_rectangle_sum.hpp"

int main() {
    cp::CDQRectangleSum<> queries;
    queries.add_query(0, 0, 4, 4);   // id 0: no points yet
    queries.add_point(1, 2, 4);
    queries.add_query(0, 0, 4, 4);   // id 1: 4
    queries.add_point(1, 2, -1);     // signed delta at the same coordinates
    queries.add_point(3, 2, 7);
    queries.add_query(1, 2, 3, 3);   // id 2: right boundary is excluded
    for (auto x : queries.solve()) std::cout << x << '\n'; // 0, 4, 3
}
```

[動く例](../../examples/offline_query/dynamic_rectangle.cpp)

## CDQ の基本形

`F(x,y)=Σ(px<x,py<y) weight` と置き、長方形を `+F(right,up)-F(left,up)-F(right,down)+F(left,down)` の 4 問い合わせに分解します。

1. 点追加と展開した問い合わせを時刻順に並べ、区間 `[l,r)` を `[l,m)` と `[m,r)` に分割する。
2. 左右を再帰的に解く。
3. 左の点追加 → 右の問い合わせの寄与を集計する。左右を x 順に走査し、厳密に小さい x の左更新を y の BIT に入れ、右問い合わせは `y<境界` の累積和を読む。
4. 今回入れた更新だけ逆元で戻す。左右を x 順にマージして親の分割へ戻る。

左右の順序が x 順に変わっても、各半分の要素は元の時刻区間内に留まります。ある過去更新と未来問い合わせの組は、初めて左右の別々の半分に分かれる分割で一度だけ数えられます。未来更新 → 過去問い合わせは数えません。1 件の長方形が途中で分割されても、4 イベントの間に更新はないため、その時刻の答えは変わりません。

全 BIT を再帰ノードごとに初期化すると大きく遅くなります。実装はそのノードで挿入した更新だけを巻き戻します。座標境界の −1 や、重みへの整数符号の乗算を必要としません。

## 計算量

P=点追加数、Q=問い合わせ数、E=P+4×空でない問い合わせ数、Y=異なる点の y 数。座標比較・重み演算を O(1) として:

- 登録: 償却 O(1)。問い合わせを追加しても即時の答えは返さない。
- `solve`: **O(E log(E+1) log(Y+1) + E log(E+1) + Q)**。最悪 **O(E log²(E+1)+Q)**。
- メモリ: 記録・作業領域・結果込み **O(E+Q)**、再帰深さ O(log(E+1))。共有の添字バッファを使う。
- `clear`: 保持イベントを破棄する O(E)。毎回 solve し直すと、毎回全履歴分の計算が必要。

## 問題・検証・参考

[Library Checker: Point Add Rectangle Sum](https://judge.yosupo.jp/problem/point_add_rectangle_sum) の [ジャッジ形式](../../verify/offline_query/point_add_rectangle_sum.test.cpp) を用意しています。最初の点を先に登録し、全操作を読み終えて solve します。

300 履歴・48,000 操作を時刻付きの素朴点列で比較し、625 格子長方形、同一座標、負の重み、極値座標、先頭問い合わせ、solve の再実行／追記／clear、20 万操作・50 万展開イベントも検証します。[検証範囲](README.md)。

考え方の参考は [robert1003 による CDQ 解説の Mokia](https://robert1003.github.io/2020/01/31/cdq-divide-and-conquer.html#bzoj1176---mokia)。この実装では半開区間・厳密比較、座標圧縮、共有バッファ、再実行可能な操作記録に揃えています。[設計判断](references.md)。
