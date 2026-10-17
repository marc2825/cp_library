# 3.24 ゲーム理論

[ライブラリ一覧](../../README.md) / [全図解](../../visualizations/index.html) / [一次資料・設計判断](references.md)

Nim、Misère Nim、Grundy数、DAGゲームのS項目を[game_theory/all.hpp](../../game_theory/all.hpp)へまとめています。通常ルールとmisèreの終局を区別し、勝敗だけでなく勝ち手も復元します。

| 説明・usage | 計算量 | 図解 |
|---|---|---|
| [Nim](nim.md) | O(n) 時間・O(1) 作業領域 | [操作](../../visualizations/game_theory/index.html#nim) |
| [Misère Nim](misere_nim.md) | O(n) 時間・O(1) 作業領域 | [操作](../../visualizations/game_theory/index.html#misere) |
| [Grundy数計算](grundy.md) | mex: O(k) 時間・O(k) 領域、DAG: O(V+E) 時間・O(V) 作業領域 | [操作](../../visualizations/game_theory/index.html#grundy) |
| [DAGゲーム・独立和と勝ち手](dag_game.md) | 構築 O(V+E) 時間・O(V+E) 保持領域 | [操作](../../visualizations/game_theory/index.html#dag) |

## 共通仕様

- 標準C++17、namespace cp。各ヘッダは単独include可能です。Nimは64ビット以下の標準整数型を使い、128ビット拡張には依存しません。入力は変更しません。
- Nim系の入力はvector<Int>、bool以外の標準整数型。符号付きの負値はinvalid_argument、boolや浮動小数点型はコンパイル時に拒否。0を含めてよく、uint64_tの全ビットを使えます。総石数は加算しません。
- NimMoveは元配列のpile添字・remaining・removed。勝勢なら敗勢へ移す手、敗勢ならnullopt。Misèreの終局だけは勝ちでも合法手がないためnulloptです。
- DAGの頂点は局面、辺は一手の遷移。通常ルールの公平な有限完全情報ゲームに使います。全成分がDAGである必要があり、自己ループと到達不能成分の閉路も拒否します。重複辺と孤立点は許します。
- Grundy値はint。頂点数V≤INT_MAX、単体mexの入力長k≤INT_MAX。範囲外頂点はout_of_range、負の値や閉路はinvalid_argument、長さ上限はlength_error。配列確保の失敗は標準のbad_alloc等です。NDEBUGでも検査を残します。
- DAGGameの複数positionsは独立したゲームです。同じ頂点でも別の成分として数え、一手で一成分だけ動かします。相互の占有制約や、同時に複数成分を動かすルールは含めません。
- Graph<Cost>版はDirectedのみ受け、費用を無視します。calculate_grundy(Graph)は隣接リストへコピーするO(V+E)追加領域。リスト版の作業領域はO(V)、DAGGameは移動先を保持するO(V+E)領域です。

| 石／合法手がない局面 | 勝敗 | 勝ち手 |
|---|---|---|
| Nim、通常のDAGゲーム | 手番側の負け | nullopt |
| Misère Nim | 手番側の勝ち（最後を取った側の負け） | nullopt |

Misère Nim専用の戦略と、通常ルールのSprague–GrundyのXORは分けて使ってください。

## ローカル検証

```sh
python3 verify/run_game_theory.py
python3 verify/run_game_theory.py --sanitize
node verify/visualizations/game_theory_model_test.js
node verify/visualizations/game_theory_browser_test.mjs --browser /path/to/chromium
```

`--compiler clang++`で切替可能。5ヘッダの単独・重複include、型のコンパイル時拒否、他カテゴリとの混在・複数翻訳単位、4使用例、4ジャッジ形式256入力と10万石の形式を検証。Pythonの1,028ケースでゲーム木・mex集合・全成分の積ゲーム・64ビットXORを独立比較します。通常・ASan/UBSanのdebug/releaseで実行。オンラインACは未確認です。

C++では6山までの19,531列を通常／misèreの完全ゲーム木と比較。21,845列のmex、5頂点までの順序付きDAG全1,100個＋350並べ替え多重DAG、各グラフの全単体・3成分までの積ゲームを照合します。合法な勝ち手、任意targetへの手、終局、到達不能閉路、重複・負値・64ビット極値、20万山・20万頂点・20万成分・mex、1,500頂点の密DAGと100万重複辺も含みます。

図解は4モード。XORの途中、Misèreの終盤、トポロジカル順とmex、独立な駒のXORと勝ち手を操作できます。64ビットの山はBigIntで保持し、合法手や勝ち手の後を再解析できます。

図解モデルは5,366入力・93,244スナップショット・3,057合法勝ち手、19拒否条件を独立解で検証します。

## 3.21〜3.25 A項目の補完

- [Nim Product](nim_product.md) / [操作できる図解](../../visualizations/final_a/index.html#nim_product)
- [周期的Grundy数列の検出](periodic_grundy.md) / [操作できる図解](../../visualizations/final_a/index.html#periodic_grundy)

[追加APIの条件・検証一覧](../final_a/README.md)。`python3 verify/run_final_a.py` / `--sanitize` / `make verify-final-a`。
