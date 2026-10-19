---
title: グリッド共通処理
documentation_of: //basic_utility/grid.hpp
---

# グリッド共通処理（F-15）

4／8近傍、境界判定、転置、90度単位の回転を共有します。既存[grid_bfs](../../graph/grid_bfs.hpp)も方向配列と境界判定を使います。BFSの探索自体は既存実装を保持しています。

## 使用例

```cpp
#include <iostream>
#include <string>
#include <vector>
#include "basic_utility/grid.hpp"
#include "graph/grid_bfs.hpp"
int main() {
    std::vector<std::string> grid{"abc", "def"};
    for (const auto& row : cp::grid_rotate_clockwise(grid)) std::cout << row << "\n";
    std::cout << cp::grid_shape(grid).neighbors4(0, 1).size() << "\n";
    std::cout << cp::grid_bfs({"...", ".#."}, 0, 0)[1][2] << "\n";
}
```

出力:

```text
da
eb
fc
3
3
```

## API・条件

- `GridShape(int rows,int cols)`：負の寸法はinvalid_argument。rows()/cols()で寸法、contains(r,c)で範囲判定。
- `neighbors4(r,c)`／`neighbors8(r,c)`：範囲内の近傍座標vector<pair<int,int>>を返す。起点が範囲外ならout_of_range。
- `grid_directions4`：右、下、左、上。`grid_directions8`：右から時計回り（右、右下、下、左下、左、左上、上、右上）。壁は扱いません。
- `grid_shape(vector<Row>)`：矩形性とint以内の寸法を検査。不揃いはinvalid_argument、寸法超過はlength_error。
- `grid_transpose(grid)`：行と列を交換した新しい配列。
- `grid_rotate_clockwise(grid,int turns=1)`：時計回り90度×turns。負数は反時計回り、4の剰余で正規化。

Rowはstd::stringやvector<T>のように、value_type・size・添字・iterator・個数と初期値からの構築を持つ型を使います。要素は既定構築・コピー・代入可能であることが必要です。文字列では文字コード単位ではなく**バイト**を回転します。入力は変更しません。

空行列は許可します。vector<Row>では0行の列数を保持できないため、H×0の転置や90度回転は空配列になり、元のHは復元できません。寸法だけ必要ならGridShapeを別途保持してください。0回転では元の行数を保ちます。

既存grid_bfsは矩形・始点・セル総数を検査するようになりました。非空盤面の範囲外始点はout_of_range、不揃いはinvalid_argument、セル数>INT_MAXはlength_error。空／幅0では距離の空配列を返し、始点を参照しません。壁の始点・未到達は-1、通常の探索結果は従来どおりです。

近傍列挙はO(1)時間・領域。矩形検査O(H)、転置／回転はO(H+W+HW)時間・領域です。0幅では行の管理コストもあるため単にO(HW)だけとはしません。Grid BFSはO(H+HW)時間・領域です。

## 使用する問題・検証

盤面探索、回転による同型判定、画像や表の転置。0〜8の各寸法、座標写像と逆回転、近傍の直接探索を確認。3×3の全512壁配置・全9始点のBFSをFloyd–Warshallと比較します。

`make verify-foundation-tools` / `python3 verify/run_foundation_tools.py --sanitize`でDebug/Release、単独include、全カテゴリ混在、説明内使用例、独立比較を検証します。オンラインACは未確認です。

[追加範囲・検証](../foundation_tools/README.md) / [一次資料](../foundation_tools/references.md) / [カテゴリ索引](README.md) / [図解](../../visualizations/foundation_tools/index.html#grid) / [図の範囲](../../visualizations/foundation_tools/README.md)
