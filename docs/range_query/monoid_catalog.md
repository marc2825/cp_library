---
title: Monoid／ActedMonoidカタログ
documentation_of: //range_query/monoid_catalog.hpp
---

# Monoid／ActedMonoidカタログ

既存Segtree／LazySegtreeに渡す演算を共通の型インターフェースでまとめます。sum/add/assign/affine作用は既存のrange_actions.hppを共有し、不足していた順序付き集約を追加します。

## 使用例

```cpp
#include <iostream>
#include <vector>
#include "basic_utility/static_modint.hpp"
#include "range_query/monoid_catalog.hpp"
int main() {
    using Min = cp::MinCountMonoid<int>;
    cp::MonoidSegtree<Min> minimum(std::vector<Min::S>{Min::leaf(3), Min::leaf(1), Min::leaf(1)});
    std::cout << minimum.all_prod().value << " " << minimum.all_prod().count << "\n";
    using Best = cp::MaxSubarrayMonoid<long long>;
    cp::MonoidSegtree<Best> best(std::vector<Best::S>{Best::leaf(-3), Best::leaf(5), Best::leaf(-1)});
    std::cout << best.all_prod().best << "\n";
    using Mint = cp::StaticModInt<101>;
    using Affine = cp::AffineFunctionMonoid<Mint>;
    std::cout << Affine::op({2, 1}, {3, 4})(5) << "\n";
    using Matrix = cp::MatrixProductMonoid<Mint, 2>;
    std::cout << Matrix::op({{1, 1}, {0, 1}}, {{1, 0}, {2, 1}})(0, 0) << "\n";
    using Fraction = cp::LinearFractionalMonoid<Mint>;
    auto reciprocal = Fraction::make(0, 1, 1, 0);
    std::cout << *Fraction::apply(reciprocal, Mint(2)) << " " << Fraction::apply(reciprocal, Mint(0)).has_value() << "\n";
    using Hash = cp::RollingHashMonoid<Mint, 7>;
    std::cout << Hash::op(Hash::op(Hash::leaf(1), Hash::leaf(2)), Hash::leaf(3)).hash << "\n";
}
```

出力:

```text
1 2
5
37
3
51 0
66
```

## API・使用条件・計算量

### 共通インターフェース

Monoidは`S`・`static S op(S,S)`・`static S e()`を持ち、結合則と単位元を満たします。`MonoidSegtree<M>`は既存Segtreeへの別名です。ActedMonoidはさらに`F`・`mapping(F,S)`・`composition(F,F)`・`id()`を持ちます。`ActedMonoidLazySegtree<A>`は既存ActionLazySegtreeへの別名です。

**`composition(f,g)`はgの後にf**。`op`の結合順序とは独立です。mappingはopに分配し、空区間を保つように定義します。サイズ指定のSegtreeはe()で初期化され、値0の要素をn個入れた状態とは限りません。配列から作る場合はleaf／valuesを使用します。添字制約・境界探索の前提は既存[Segtree](segtree.md)／[LazySegtree](lazy_segtree.md)と共通です。

| カタログ | Sと意味 | 主な生成・評価 |
|---|---|---|
| `MinCountMonoid<T>` / `MaxCountMonoid<T>` | `{value,count}`、極値と出現数。count=0は空 | `leaf(value)` |
| `MaxSubarrayMonoid<T,AllowEmpty=false>` | `{sum,prefix,suffix,best,length}`、和と最大prefix/suffix/部分配列和 | `leaf(value)` |
| `MatrixProductMonoid<T,N>` | 既存Matrix<T>のN×N行列、左から右への積A*B | e()は単位行列 |
| `AffineFunctionMonoid<T>` | `{a,b}`でax+b。op(f,g)はg∘f | Sのoperator()(x) |
| `LinearFractionalMonoid<T>` | `{a,b,c,d}`で(ax+b)/(cx+d)。op(f,g)はg∘f | make／apply（下記） |
| `RollingHashMonoid<Mint,Base>` | `{hash,power,length}`。左の列に右の列を連結 | `leaf(code)` |
| 既存`SumMonoid<T>`と`AddSumAction`／`AssignSumAction`／`AffineSumAction` | 和・長さと加算／代入／affine作用 | [既存API](range_actions.md) |
| `PolynomialAddSumAction`／`ArithmeticProgressionAddSumAction` | 絶対座標の多項式加算・等差加算と和 | [専用ヘッダと説明](polynomial_add_sum.md) |

極値モノイドのTは既定構築と全順序比較が必要です。空のvalueは結果として使いません。最大部分配列和のAllowEmpty=falseは全負値なら最大の負数、trueなら0を返せます。**空区間ではlength=0でbest=0**を返すため、非空の最大和として解釈しないでください。区間の位置復元はこの集約には含めません。

行列は既存matrix_multiplyを共有し、サイズがN×Nでなければopでinvalid_argument。積A*Bは列ベクトルへB→Aの順に作用します。一方、関数モノイドは列に並んだf→gの順に作用します。非可換な演算の向きを混同しないでください。

一次分数変換は**体の上の可逆行列**を要求します（例：素数法StaticModInt）。make(a,b,c,d)はad-bc=0ならinvalid_argument。apply(f,optional<T> x)ではnulloptが無限遠点を表し、無限遠点への像もnulloptです。特異なSを直接作った場合の0/0はdomain_error。合成や評価には体の四則演算が必要で、整数の切り捨て除算や逆元のない合成数法は対象外です。逆元での評価O(log mod)等のコストはTに従います。

Hashは`h(left+right)=h(left)*power(right)+h(right)`、powerはBaseの長さ乗です。codeの割当・Base・Mintの法は利用者が固定します。長さも比較し、ハッシュ一致を厳密な同一性とみなさないでください。単一hash・固定baseの衝突耐性は保証しません。動的modintの法は木の生存中に変更しないでください。

整数を使う演算はすべての中間値が型に収まること、count/lengthはsize_tに収まることが前提です。浮動小数点は丸めにより厳密な結合則を満たさない点に注意してください。MatrixProductMonoidの整数積は既存APIの検査に従います。

Tの演算をO(1)とすると、行列以外のop/eはO(1)時間・領域、行列opはO(N³)時間・O(N²)領域。Segtreeの構築はn回程度のop、問い合わせ・更新はO(log n)回のopです。一般のモノイド法則をコンパイル時に証明するものではありません。

## 使用する問題と検証

[Library Checker Point Set Range Composite](https://judge.yosupo.jp/problem/point_set_range_composite)。全1,093個の三値配列を全区間で直接探索し、順序付き関数合成・行列作用・無限遠点を含む一次分数合成・hashを独立に比較します。

`python3 verify/run_foundation_structures.py` / `--sanitize`（`make verify-foundation-structures`）で単独／重複include、全カテゴリとの混在・複数翻訳単位、Debug/Release、使用例・差分比較を検証します。オンライン提出・ACは未確認です。

[追加範囲と検証](../foundation_structures/README.md) / [参考資料](../foundation_structures/references.md) / [カテゴリ索引](README.md) / [操作できる図解](../../visualizations/foundation_structures/index.html#monoid_catalog) / [図解の範囲](../../visualizations/foundation_structures/README.md)
