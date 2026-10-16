---
title: 点の偏角ソート
documentation_of: //computational_geometry/angular_sort.hpp
---
# 点の偏角ソート

[カテゴリ・共通仕様](README.md) / [操作できる図解](../../visualizations/computational_geometry/index.html#angle) / [一次資料・設計判断](references.md)

`angular_sort(points)`は原点を最初、その後は+x軸から反時計回りの[0,2π)に並べたコピーを返します。同じ向きの順番は保持します。距離順への追加ソートはしません。

整数版は半平面と厳密な外積で比較します。浮動小数点版はatan2から得た角度の数値順です。丸めで同じ角度値になった点は安定順であり、厳密な角度比較ではありません。どちらも比較器へEPSを入れません。近似等値を使った非推移的な比較によるstd::sortの前提違反を避けます。

Library Checkerは(−π,π]のため、ジャッジ用コードではy<0の点を前へ移し、APIの順序から変換しています。原点の角度は0として扱います。

## usage

```cpp
#include <iostream>
#include <vector>
#include "computational_geometry/angular_sort.hpp"
namespace e=cp::geometry::exact;
namespace f=cp::geometry::floating;
int main(){
    auto p=e::angular_sort({{-1,0},{0,-1},{0,0},{0,1},{1,0}});
    for(auto q:p)std::cout<<q.x<<' '<<q.y<<'\n';
}
```

[実行できる例](../../examples/computational_geometry/angular_sort.cpp)。整数・浮動小数点の型とnamespaceは混ぜず、使用する方を明示してください。

## 条件・計算量

O(n log n) 時間・O(n) 追加領域。整数はGCC/Clangの検査付き128ビット演算です。有理数の正規化・比較にはO(log M)の整数演算を要し、上の計算量はこれを基本演算一回として数えています。中間値の範囲・浮動小数点の誤差と例外は[共通仕様](README.md#数値と境界の契約)を参照してください。

## 使用する問題・検証

Library Checker: Sort Points by Argument。ローカルでは[検証スクリプト](../../verify/run_computational_geometry.py)から使用例・独立したFraction／総当たりの解・境界と退化ケースを確認します。全体の検証範囲は[カテゴリ索引](README.md#ローカル検証)に記載しています。オンラインACは未確認です。
