---
title: SWAG・Sliding Window Aggregation
documentation_of: //static_query/swag.hpp
---

# SWAG（Sliding Window Aggregation）

キューの末尾へ追加し、先頭から削除しながら、全要素の左から右への集約を取得します。和、min/max/gcd、行列積、関数合成などに使え、逆演算・可換性・冪等性は不要です。

[実装](../../static_query/swag.hpp) / [2 スタックと合成方向の図解](../../visualizations/static_query/index.html#swag) / [一覧](README.md)

## API・条件

`cp::SWAG<S,op,e>`。op は結合律を満たし、e() は左右の単位元を返します。

| API | 意味 | 計算量 |
|---|---|---|
| `SWAG()` | 空で初期化 | O(1) |
| `push(x)` | 末尾に追加 | ならし O(1) |
| `pop()` | 先頭を削除 | ならし O(1)、最悪 O(n) |
| `front()` | 先頭値への const 参照 | ならし O(1)、最悪 O(n) |
| `back()` | 末尾値への const 参照 | O(1) |
| `prod()` | 左から右の全体積、空なら e() | O(1) |
| `size()` / `empty()` | 論理要素数／空判定 | O(1) |
| `clear()` | 全要素を破棄 | O(n) |

演算・コピー O(1) の場合です。保持データは O(n)。vector の容量は再利用するため、確保済みメモリは過去の最大長にも依存します。空の pop/front/back は呼べません。front は内部の移し替えを行うため非 const です。

## usage

```cpp
#include "static_query/swag.hpp"
struct F { long long a, b; };
F op(F left, F right) { return {right.a*left.a, right.a*left.b+right.b}; }
F e() { return {1, 0}; }
int main() {
    cp::SWAG<F, op, e> queue;
    queue.push({2,1}); queue.push({3,4}); // 2x+1 の後に 3x+4
    auto f = queue.prod();              // 6x+7
    queue.pop();                       // 2x+1 を削除
    auto g = queue.prod();              // 3x+4
    return f.a != 6 || f.b != 7 || g.a != 3 || g.b != 4;
}
```

[動くアフィン合成の使用例](../../examples/static_query/swag.cpp)。整数の合成では中間値も型に収めてください。ジャッジ用例では 998244353 で剰余を取ります。

## 原理・注意点

追加用 back と削除用 front の 2 スタックを持ちます。back の累積は `op(以前の積,新要素)`、移し替え時の front の累積は `op(新要素,以前の積)` と、向きが逆です。全体は `op(front の積,back の積)` で取得します。

front が空になったときだけ back を逆順に移し替えます。各要素は追加・移動・削除を高々 1 回ずつ行うため、合計 O(n) 操作で O(n) です。1 回の pop が常に O(1) という保証ではありません。

単調キューと違って全要素を保持するため、優劣で捨てられない演算にも対応します。文字列連結を使うと演算もコピーも O(文字列長) になり、累積文字列によるメモリ増加もあるので、大入力には固定サイズのモノイドを使ってください。

## 使用する問題・検証

[Library Checker: Queue Operate All Composite](https://judge.yosupo.jp/problem/queue_operate_all_composite) — [剰余付き検証プログラム](../../verify/static_query/queue_operate_all_composite.test.cpp)。

独立した関数の逐次適用と比較し、front/back 両スタックがある状態、全移し替え、空の積、clear、文字列の順序、非デフォルト構築型と演算回数を確認します。オンライン AC の記録ではありません。

参考: [Nyaan Slide Window Aggregation](https://nyaannyaan.github.io/library/data-structure/slide-window-aggregation.hpp.html)。本 API は `prod()` に揃え、front/back/size/clear を提供します。
