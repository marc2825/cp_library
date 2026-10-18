---
title: 等差数列・多項式加算と区間和
documentation_of: //range_query/polynomial_add_sum.hpp
---

# 等差数列・多項式加算と区間和

各座標の冪の和を集約に保持し、区間へ多項式を加える更新をLazy Segtreeへ接続します。次数1を使えば、任意の左端から始まる等差数列の加算も扱えます。

## 使用例

```cpp
#include <iostream>
#include "range_query/polynomial_add_sum.hpp"
int main() {
    using AP = cp::ArithmeticProgressionAddSumAction<long long>;
    cp::ActedMonoidLazySegtree<AP> arithmetic(AP::values({1, 2, 3, 4}));
    arithmetic.apply(1, 4, AP::progression(1, 3, 2));
    std::cout << arithmetic.all_prod().sum << "\n";
    using Poly = cp::PolynomialAddSumAction<long long, 2>;
    cp::ActedMonoidLazySegtree<Poly> polynomial(Poly::values({1, 2, 3, 4}));
    polynomial.apply(0, 4, {0, 0, 1}); // 絶対添字iに対してi*iを加える。
    std::cout << polynomial.all_prod().sum << "\n";
}
```

出力:

```text
25
24
```

## API・使用条件・計算量

`PolynomialAddSumAction<T,Degree>`は既定なしのコンパイル時次数Dを持ちます。Tは加減乗算ができる型で、整数では中間値を含む表現可能性が必要です。法付き整数にも使え、割り算は不要です。Degree+1と配列領域が表現・確保できる範囲で使用します。

- `S={sum,moments}`：moments[k]は区間内の各**絶対座標**xについてx^kを足した値。k=0は要素数に対応します。
- `F=array<T,Degree+1>`：係数を低次から順に並べる。更新量はf[0]+f[1]x+…+f[D]x^D。
- `leaf(value,coordinate)`：座標付きの1要素。
- `values(vector<T>)`：座標を0,1,…としてleaf列を構築。
- `op`は和・各momentの加算。e()は空区間（momentもすべて0）。
- `mapping(f,s)`は係数とmomentの内積をsumへ加算し、momentを保持。
- `composition(f,g)`は係数ごとの加算、id()は全係数0。

`ArithmeticProgressionAddSumAction<T>`は次数1の別型で、`progression(size_t l,T first,T step)`を追加します。`apply(l,r,progression(l,first,step))`で位置lにfirst、l+1にfirst+step、…を加えます。返す係数は`{first-step*l,step}`で、`[l,r)`は半開区間です。負の初項・公差も型の範囲内なら扱えます。

**n個の値を用意する場合はvaluesで初期化してください。** `LazySegtree(n)`はe()がn個なのでmoment=0のままであり、0値の要素をn個置くこととは異なります。点更新set(p,...)では`leaf(value,T(p))`を渡し、座標を維持してください。任意座標で初期化した場合は、各leafに渡した座標を更新多項式も使います。区間の左端に自動で座標を合わせる動作はありません。

0^0=1としてmomentを作ります。空区間・空配列に対応します。modint利用中はその法を変更しません。全操作は演算条件を満たすTに対して結合則・作用の分配則を満たします。更新前後の値や座標冪が通常整数に収まらない場合は、modintや適切な多倍長型を用いてください。

leaf/op/mapping/compositionはO(D+1)、各S/Fの領域もO(D+1)。初期化O(n(D+1))、区間更新・区間和・点更新O((D+1)log n)、木全体の領域O(n(D+1))。

## 使用する問題と検証

添字依存の加算、等差加算、二次以上の区間加算DP。次数0・1・4、作用の分配と合成・空区間・点更新を検証し、Pythonでは8,000回の二次式操作を素朴配列と比較します。

`python3 verify/run_foundation_structures.py` / `--sanitize`（`make verify-foundation-structures`）で単独／重複include、全カテゴリとの混在・複数翻訳単位、Debug/Release、使用例・差分比較を検証します。オンライン提出・ACは未確認です。

[追加範囲と検証](../foundation_structures/README.md) / [参考資料](../foundation_structures/references.md) / [カテゴリ索引](README.md) / [操作できる図解](../../visualizations/foundation_structures/index.html#polynomial_add_sum) / [図解の範囲](../../visualizations/foundation_structures/README.md)
