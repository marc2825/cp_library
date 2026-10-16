---
title: 部分集合DP・列挙と巡回路
documentation_of: //dp_optimization/subset_dp.hpp
---

# 部分集合DP・列挙と巡回路

集合をビットで表す基本操作と、訪問済み集合×終点のTSPを提供します。部分集合の総和をまとめるゼータ変換・その逆変換も使えます。

[カテゴリ索引](README.md) / [操作できる図解](../../visualizations/dp_optimization/index.html#subset) / [実装](../../dp_optimization/subset_dp.hpp) / [参考資料](references.md)

## usage

```cpp
#include <iostream>
#include <optional>
#include <vector>
#include "dp_optimization/subset_dp.hpp"
int main() {
    std::vector<std::vector<std::optional<long long>>> cost{
        {std::nullopt,2,8}, {4,std::nullopt,3}, {1,6,std::nullopt}};
    auto tour = cp::minimum_hamiltonian_cycle(cost);
    if (tour.exists) {
        std::cout << tour.cost << '\n';
        for (int v : tour.vertices) std::cout << v << ' ';
        std::cout << '\n';
    }
    std::vector<long long> f{1,2,3,4};
    cp::subset_zeta_transform(f);
    for (auto x : f) std::cout << x << ' ';
    std::cout << '\n';
}
```

[動く使用例](../../examples/dp_optimization/subset_dp.cpp)。リポジトリのルートをincludeパスへ通します。

## API・使用条件

| API | 意味 |
|---|---|
| `for_each_submask(uint64_t mask,callback,include_empty=true)` | 部分集合を降順でcallback(sub)へ渡す。空集合も既定で含む |
| `subset_zeta_transform(vector<T>& f)` | in-placeで `f[mask] = Σ_{sub⊆mask} f[sub]` |
| `subset_mobius_transform(vector<T>& f)` | 上の逆変換。Tの-=が必要 |
| `minimum_hamiltonian_cycle(cost,start=0)` | 全頂点を1回ずつ訪問して戻る最小有向巡回路 |

変換配列の長さは正の2冪。違反はinvalid_argument。長さ1は要素を変えません。Tは+=、逆変換では-=を持つ可換な加法の型です。3.19で本体を共通化し、bool以外の標準整数は各加減算の超過を検査してoverflow_errorにします。独自型の算術条件は利用者が管理します。途中の例外では以前の更新が残ります。逆変換はビットの段階を降順に処理します。

TSPのcostは `vector<vector<optional<long long>>>` の正方行列。nulloptは辺なし、0・負の費用は辺です。形の違反はinvalid_argument。20頂点を超えるとreleaseでもlength_error。非空ではstartが有効な頂点であること（assert）。

結果は `exists,cost,vertices`。存在しなければfalse・0・空。空はtrue・0・空。1頂点は移動しない費用0・`{start}`（自己ループ不要）。2頂点以上は長さN+1の頂点列で末尾にstartを重ねます。同点の辞書順保証なし。

## 原理・計算量・注意点

部分集合列挙はk=popcount(mask)としてO(2^k)回のコールバック、追加空間O(1)。0で終了するため無限ループになりません。全maskとそのsubmaskの列挙はO(3^N)。コールバックからの打切り機能はありません。

ゼータ／メビウスは長さ2^NとしてO(N2^N)時間・追加空間O(1)。ジェネリック型の演算費用を掛けて考えます。

TSPはO(N²2^N)時間・O(N2^N)空間。dpの到達性を別配列に持つのでLLONG_MIN/MAXを番兵に予約しません。費用の各加算は範囲を検査し、超過でoverflow_error。一般的な64bit環境で20頂点のDP用配列は約200MiBを使います。上限は実行時間保証ではなく、指数時間が実用的な入力規模で使ってください。

旧 `DP/subset_dp.hpp`、`DP/TSP.hpp` は既存の貼り付け用スニペットのまま保持します。新しい自己完結したヘッダは本ファイルです。

## 使用する問題・検証範囲

[AOJ DPL_2_A: Traveling Salesman Problem](https://judge.u-aizu.ac.jp/onlinejudge/description.jsp?id=DPL_2_A)。

300グラフを全順列と比較し、始点変更・欠辺・0／負辺・全復元経路を確認。部分集合の全列挙、全submask和、逆変換、mod型、uint64の最上位ビット、形・制限超過、18頂点の疎な巡回路。

[検証入口](../../verify/run_dp_optimization.py)は通常／ASan・UBSan、debug/release、単独include、混在include、複数翻訳単位、使用例、独立オラクル、ジャッジ形式の入出力を確認します。オンラインACは未確認です。

Submask列挙と部分集合変換の本体は[3.19 集合関数・Bitmask変換](../set_function/README.md)へ移動しました。このヘッダからも同じAPIを使えます。Supermask列挙・上位集合変換・汎用SOSの説明と図解は3.19を参照してください。
