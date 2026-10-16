# 3.23 組合せ最適化・マトロイド

[ライブラリ一覧](../../README.md) / [全図解](../../visualizations/index.html) / [一次資料・設計判断](references.md)

指定されたHungarian Algorithm、最大マッチング、最小費用流、Assignment Problemを独立カテゴリへまとめています。入口は[all.hpp](../../combinatorial_optimization/all.hpp)。C++17、namespace cpです。

| API・usage | 計算量 | 図解 |
|---|---|---|
| [Hungarian Algorithm](hungarian.md) | O(n²m+n+m) 時間・O(n+m) 作業領域（入力・出力を除く） | [操作](../../visualizations/combinatorial_optimization/index.html#hungarian) |
| [Assignment Problem](assignment_problem.md) | 構築 O(nm)・set/forbid/cost O(1)・solve O(n²m+n+m) 時間 | [操作](../../visualizations/combinatorial_optimization/index.html#assignment) |
| [一般グラフの最大マッチング（Blossom）](general_matching.md) | O(V³+E) 時間・O(V+E) 領域 | [操作](../../visualizations/combinatorial_optimization/index.html#matching) |
| [二部最大マッチング（共通API）](bipartite_matching.md) | O((V+E)√(V+1)) 時間・O(V+E) 領域 | [操作](../../visualizations/combinatorial_optimization/index.html#bipartite) |
| [最小費用流](min_cost_flow.md) | 初期化 O(V+VE)、探索 O((A+1)(V+E) log(V+E+1))、保持・作業 O(V+E) | [操作](../../visualizations/combinatorial_optimization/index.html#flow) |

## 選び方

| やりたいこと | 選ぶAPI |
|---|---|
| 費用行列から、全行へ異なる列を割り当てる | hungarian |
| 禁止辺の変更や最小／最大目的を切り替える | AssignmentProblem |
| 左右が決まったグラフで組数を最大化する | BipartiteMatching（3.9と共通） |
| 奇数閉路がある無向グラフで組数を最大化する | GeneralMatching（Blossom） |
| 容量と単位費用のあるネットワークへ送流する | MinCostFlow |

最大組数、全行の最小費用割当、最大重み割当は目的が異なります。一般グラフの重みはGeneralMatchingでは扱いません。費用付き二部割当はHungarianまたは容量1の最小費用流で表せます。

## 共通仕様

- 0始まり、入力は保持・コピーまたはconst参照。solveやmaximum_matchingは毎回再計算し、返した配列は独立した値です。MinCostFlowだけは残余状態を保持し、追加量を返します。
- Hungarian／Assignmentの入力費用はlong long。禁止辺はoptionalのnullopt、巨大な有限値ではありません。全行を埋められなければnullopt。空と長方形も扱います。同値最適解の辞書順最小は保証しません。
- 費用を扱う新実装はGCC/Clangの`__int128`とoverflow検出組込み関数を使います。外部ライブラリは不要ですが、この拡張がないコンパイラは対象外。内部は検査付きOptimizationWideで、表示はoptimization_to_string。
- 入力long longの上下限も受け取れますが、全ての途中値が128ビットに、返す総費用はlong longまたはCostに収まることが条件です。範囲外はoverflow_error。任意精度でも飽和計算でもありません。フローは各増加後の今回費用もCostの範囲が必要です。
- Assignmentのpotentialは最小化でu+v≤cost・v≤0、最大化でu+v≥weight・v≥0。選択辺は等号、未使用列のv=0、Σu+Σv=costの最適性証明を返します。
- GeneralMatchingは無向の重みなし最大組数。自己ループは無視し、平行辺は解くときに除去。二部版は既存APIを共有し、その境界条件はassertです。
- 最小費用流は負辺を許し、初回正送流時に全成分の正容量辺の負閉路を拒否します。負閉路の循環改善は対象外。s,tと辺集合はresetまで固定。全辺の復元とslopeを提供します。
- Assignmentの行・列数はINT_MAX未満、GeneralMatching／MinCostFlowはintの頂点数。過大な配列確保は標準のbad_alloc等になり得ます。新APIの不正入力は例外で検出し、NDEBUGでも検査を残します。

## ローカル検証

```sh
python3 verify/run_combinatorial_optimization.py
python3 verify/run_combinatorial_optimization.py --sanitize
node verify/visualizations/combinatorial_optimization_model_test.js
node verify/visualizations/combinatorial_optimization_browser_test.mjs --browser /path/to/chromium
```

`--compiler clang++`で切替可能。7ヘッダの単独・重複include、既存カテゴリとの混在・複数翻訳単位、5使用例、5ジャッジ形式・320ケースを確認します。Pythonの任意精度1,099ケースで割当・双対証明・マッチング・流量／費用／slopeを独立比較。通常・ASan/UBSanのdebug/releaseを対象にします。オンラインACは未確認です。

C++では禁止辺付き行列4,458種類と800ランダム割当、6頂点までの単純グラフ全33,868個、1,400シャッフル多重グラフ、二部版との100比較、1,130ネットワークの全整数フローを検証します。最適性の双対証明、負費用・逆辺の組替え・全流量の折れ線・追加送流・リセット・全成分の負閉路拒否・型の上下限も含みます。350×500割当、500頂点の密グラフ、20万頂点の反復送流も実行します。

図解は5モード。費用をBigIntで計算し、potential／slack、禁止辺と最大化、奇数閉路のbase、交互路、残余逆辺と送流の単位費用を追えます。C++の固定整数型の上限を再現するものではありません。

図解モデルは2,782入力・27,955スナップショット・16拒否条件を検証。719回のBlossom縮約、双対・slack、各頂点のmate/base、流量保存・逆辺経路・費用の折れ線と、スナップショットの独立性も確認します。
