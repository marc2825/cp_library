---
title: Largest Rectangle・最大長方形
documentation_of: //dp_optimization/largest_rectangle.hpp
---

# Largest Rectangle・最大長方形

非負ヒストグラム、または障害物付き二値グリッドから、面積最大の長方形とその範囲を返します。

[カテゴリ索引](README.md) / [操作できる図解](../../visualizations/dp_optimization/index.html#rectangle) / [実装](../../dp_optimization/largest_rectangle.hpp) / [参考資料](references.md)

## usage

```cpp
#include <iostream>
#include "dp_optimization/largest_rectangle.hpp"
int main() {
    auto histogram = cp::largest_rectangle_histogram({2,1,4,5,1,3,3});
    std::cout << histogram.area << ' ' << histogram.left << ' ' << histogram.right << '\n';
    auto grid = cp::largest_rectangle_grid({{false,false,true}, {false,false,false}});
    std::cout << grid.area << ' ' << grid.top << ' ' << grid.left << ' ' << grid.bottom << ' ' << grid.right << '\n';
}
```

[動く使用例](../../examples/dp_optimization/largest_rectangle.cpp)。リポジトリのルートをincludeパスへ通します。

## API・使用条件

`largest_rectangle_histogram(const vector<long long>& height) -> HistogramRectangle`

結果は `area,left,right,height`。棒の区間は[left,right)、高さは非負（assert）。全ての棒に含まれる長方形です。

`largest_rectangle_grid(const vector<vector<bool>>& blocked) -> GridRectangle`

**trueは障害物、falseが使えるマス**。結果は `area,top,left,bottom,right`、行[top,bottom)×列[left,right)。raggedな配列はinvalid_argument。

空・0列・全0高さ・全障害物では、面積0・全座標0の空の長方形。同点の辞書順は保証しません。入力長・行数はintに収まることが条件です。面積はlong longで、乗算超過はoverflow_error。

## 原理・計算量・注意点

ヒストグラムはO(N)時間・空間。既存の[単調スタック](../static_query/monotonic_stack.md)で左右の**厳密に小さい**棒を探し、その間の最大幅×高さを比較します。同値の棒は境界にしません。

グリッドは各行まで上向きに続く使用可能マスの高さを更新し、ヒストグラムへ帰着。O(HW)時間・O(W)空間です。

0列の場合は行の形を確認するO(H)時間を使います。空も含む表記ではO(H(W+1)+1)時間・O(W+1)空間です。

[グリッドの図解](../../visualizations/dp_optimization/index.html#grid)もあります。図解の1回のスタック走査とC++の左右境界の方式は同じ最大幅を求めますが、内部命令の一致は意図していません。旧 `misc/Largest_Rectangle.hpp` は変更せず保持します。障害物の意味も同じです。

## 使用する問題・検証範囲

[AOJ DPL_3_C: Histogram](https://judge.u-aizu.ac.jp/onlinejudge/description.jsp?id=DPL_3_C)、[DPL_3_B: Grid](https://judge.u-aizu.ac.jp/onlinejudge/description.jsp?id=DPL_3_B)。

三値高さ9,841配列と5,058二値グリッドを全長方形と比較。座標・高さ・面積・使用可能マスも検査。旧API比較、空・0列・同値・面積の極値／超過・20万棒。

[検証入口](../../verify/run_dp_optimization.py)は通常／ASan・UBSan、debug/release、単独include、混在include、複数翻訳単位、使用例、独立オラクル、ジャッジ形式の入出力を確認します。オンラインACは未確認です。
