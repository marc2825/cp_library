# 実装予定リスト（3.1〜3.25）

更新日: 2026-10-20。ユーザー提供の全分野リストをS/A/B/Cの分類ごと保存し、3.1〜3.25のA/B項目を現在の実装と照合しました。[引継書](../HANDOFF.md) / [カテゴリ索引](README.md) / [先に追加した高度な20項目](advanced/README.md)。

## 今後の進め方

2026-10-18の追加候補・仕様追加・元Cの照合結果は、[未実装・追加仕様の統合一覧](remaining_work.md)へ重複を除いて整理しました。初回245作業単位（当時は未実装174、一部対応40、旧実装の整備待ち6、要仕様化25）、実装済みとして除外した27組、元C全101掲載箇所の対応表があります。2026-10-19の依頼に従い一覧の先頭から着手し、[F-01〜F-07](basic_utility/foundation.md)、続いて[F-08〜F-10](foundation_structures/README.md)を整備しました。続いて[F-11：多倍長整数](basic_utility/big_integer.md)を追加しました。続いて[F-12〜F-17の6項目](foundation_tools/README.md)を整備しました。[V-01〜V-04の開発・検証基盤](development/README.md)も整備しました。[V-05〜V-07の実行・観察ツール](development_runtime/README.md)も整備しました。現在は残218（未実装158、一部対応30、整備待ち5、要仕様化25）、追加整備済み27。[V-08〜V-10の出典・検証台帳と共通debug mode](development_records/README.md)も整備しました。次はU-01（尺取り法）です。元分類・掲載順は保持します。

**ユーザー指定の対象を優先します。A全169掲載箇所の補完は完了し、2026-10-17〜18の指定で3.1〜3.25のB全168掲載箇所も整備しました。** 3.1から3.25へカテゴリ順に進め、同じカテゴリでは下の掲載順を基本にします。依存する部品や重複項目はまとめて処理し、実装済みのものは再実装しません。S/CやPython版の次の対象は、以後の依頼に従います。

3.1〜3.6のAは47掲載箇所すべて実装済みです（今回の補完32箇所）。3.7〜3.11のAも36掲載箇所すべて実装済みです（今回の補完15箇所）。3.12〜3.15のAも32掲載箇所すべて実装済みです（補完19箇所、3.14は既存実装を保持）。3.16〜3.20のAも全29掲載箇所が実装済みです（補完9掲載箇所・8系統）。[追加APIと検証](transform_a/README.md)。3.21〜3.25のAも全25掲載箇所が実装済みです（補完10項目）。[追加APIと検証](final_a/README.md)。これで全169掲載箇所のAが実装済み（旧API3箇所を含む）です。3.1〜3.6のBは全43掲載箇所を公開API・説明・使用例・比較検証・図解付きで整備済みです。[対応範囲と検証](b_1_6/README.md)。3.7〜3.11のB全36掲載箇所も[API・比較検証・図解付き](b_7_11/README.md)で整備済みです。3.12〜3.15のB全30掲載箇所も[API・比較検証・図解付き](b_12_15/README.md)で整備済みです。3.16〜3.20のB全31掲載箇所も[API・比較検証・図解付き](b_16_20/README.md)で整備済みです。3.21〜3.25のB全28掲載箇所も[公開API・比較検証・図解付き](b_21_25/README.md)で整備しました。B全168掲載箇所が完了しました。Sは元の全体計画、Cは統合一覧での照合結果を保持します。3.1の新しい共通部品は `basic_utility/` に置きました。ダブリング・座標圧縮・Cartesian Tree・XOR基底など他カテゴリに実体がある機能は参照し、移動や複製を前提にしません。

実装前にはこの文書の状態と実ファイルを再確認し、空ファイル・雛形・WIP・内部専用部品を完成済みの公開APIと取り違えないでください。「一部対応」は注記した不足分だけを補います。コードが同じ役割を既に満たす場合は、薄い入口・説明・使用例の整備で対応できます。

1. 対象の既存実装・説明・検証を読み、必要な機能と配置先を決める。
2. C++17・`namespace cp`・0始まり／半開区間の規約に合わせ、旧APIを保持して不足分を実装する。
3. 説明・usage・API条件・計算量・問題例・出典、動く例、独立した素朴解との比較検証、操作できる図解を揃える。
4. 適切な通常／サニタイザ検証を実行し、各カテゴリのall.hpp・索引・Makefile・このリスト・引継書を更新する。オンラインACとローカル検証を区別する。

## Python版ライブラリの作成計画

2026-10-17の追加依頼により、**3.1〜3.25の全カテゴリを対象とするPython版ライブラリも作成します。** 今回は計画への追記であり、Python版の実装完了を意味しません。対象は既存・今後追加するS/A/B/Cの公開機能で、C++版との対応をカテゴリごとに追跡します。検証用Pythonスクリプトは、利用者向けのPython版実装として数えません。

- [ ] 配置先を `python/cp_library/<category>/` とし、C++版と同じカテゴリ区分を維持する。単独でコピーできるモジュールと、必要なものだけimportできる入口を用意する。
- [ ] C++版の公開APIとの対応表を作り、3.1からカテゴリ順に移植する。重複機能はPython版でも共通実装を共有し、既存C++ファイルは保持する。
- [ ] Pythonに適した関数・クラス設計、型ヒント、0始まり・半開区間、空入力・境界・例外の契約を整える。C++テンプレートの機械的な置き換えではなく、必要に応じてcallableや組み込みコンテナを使う。
- [ ] 標準ライブラリだけで動く実装を基本とし、C++拡張や外部競技ライブラリへの実行依存を設けない。対象Python版と実際に検証したCPython／PyPyの環境を明記する。
- [ ] 任意精度整数の演算コスト、再帰深さ、オブジェクト数・メモリ使用量を考慮し、各機能の計算量と実用的な入力規模を説明する。C++版と同じ速度を前提にしない。
- [ ] 各モジュールに日本語の説明・usage・API条件・計算量・問題例・出典、動くPython使用例、素朴解との比較検証を揃える。可能な機能は同じ入力でC++版とも照合する。
- [ ] Python版の検証入口とMakefileターゲット、カテゴリ索引・共通索引を追加する。既存の図解は共有し、表示モデルとPython実装に差がある場合は明記する。
- [ ] 高度な機能も対象から黙って除外せず、移植状況・性能上の制約・代替実装を対応表へ記録する。空ファイルや説明だけの項目は実装済みにしない。

着手順・一回の対象範囲は以後の依頼に従います。今回までの3.1〜3.25のB項目はC++版として整備し、Python版の実装は別の作業単位で進めます。

## 状態の読み方

- `[x] 実装あり`：該当する公開機能を確認。今回、全項目を再実行して検証したという意味ではありません。
- `[x] 旧実装あり`：既存APIで機能を確認。新APIへの移行や説明の不足と、アルゴリズムの未実装を区別します。
- `[ ] 一部対応`：関連する核はあるものの、注記した機能・公開入口・組合せの検証などを補う必要があります。
- `[ ] 未実装`：現状のコードから対応する完成機能を確認できません。再開時にもう一度検索します。
- 3.1〜3.25のA/Bを掲載箇所単位で追跡しています。Sの列挙に状態がない場合、未実装という意味ではなく未照合です。Cの元分類は保持し、実コードとの照合結果は[統合一覧](remaining_work.md)の全件対応表を参照してください。

同じ項目が複数カテゴリに載っているため、件数は**掲載箇所単位**です。現在のAは169箇所（実装あり166、旧実装あり3、一部対応0、未実装0）。チェックは実装あり／旧実装ありだけに付けています。

分類は提供リストを維持します。実体の配置は既存構成を優先し、Miller–Rabin／Pollard's Rhoは3.14、永続セグメント木は3.5、Li Chaoは3.12、集合変換は3.19にある実装を参照します。原稿にある検索結果のサイト名・「+2」等の表示断片だけを除き、項目と補足は保持しました。

## カテゴリ別の進捗

| 分野 | 主な配置先 | A掲載数 | 実装あり（旧含む） | 一部対応 | 未実装 |
|---|---|---:|---:|---:|---:|
| [3.1 基本ユーティリティ・探索](#3-1) | `basic_utility/` | 11 | 11 | 0 | 0 |
| [3.2 Union-Find・連結性管理](#3-2) | `union_find/` | 7 | 7 | 0 | 0 |
| [3.3 セグメント木・Fenwick Tree](#3-3) | `range_query/` | 7 | 7 | 0 | 0 |
| [3.4 静的クエリ・順序統計・集合管理](#3-4) | `static_query/` | 7 | 7 | 0 | 0 |
| [3.5 平衡二分探索木・永続データ構造](#3-5) | `ordered_structure/` | 7 | 7 | 0 | 0 |
| [3.6 オフラインクエリ・二次元データ構造](#3-6) | `offline_query/` | 8 | 8 | 0 | 0 |
| [3.7 グラフ：基本探索・最短路・MST](#3-7) | `graph_basic/` | 8 | 8 | 0 | 0 |
| [3.8 グラフ：連結成分・構造分解](#3-8) | `graph_connectivity/` | 8 | 8 | 0 | 0 |
| [3.9 グラフ：フロー・マッチング・最適化](#3-9) | `graph_flow/` | 6 | 6 | 0 | 0 |
| [3.10 グラフ：組合せ構造・数え上げ](#3-10) | `graph_combinatorics/` | 6 | 6 | 0 | 0 |
| [3.11 木アルゴリズム](#3-11) | `tree/` | 8 | 8 | 0 | 0 |
| [3.12 DP・最適化](#3-12) | `dp_optimization/` | 9 | 9 | 0 | 0 |
| [3.13 数論：基本整数演算](#3-13) | `number_theory_basic/` | 10 | 10 | 0 | 0 |
| [3.14 数論：乗法的関数・高度な整数問題](#3-14) | `number_theory_convolution/` | 5 | 5 | 0 | 0 |
| [3.15 組合せ論・数え上げ](#3-15) | `combinatorics/` | 8 | 8 | 0 | 0 |
| [3.16 畳み込み・高速変換](#3-16) | `convolution/` | 6 | 6 | 0 | 0 |
| [3.17 多項式・形式的冪級数（FPS）](#3-17) | `fps/` | 10 | 10 | 0 | 0 |
| [3.18 線形漸化式・母関数](#3-18) | `linear_recurrence/` | 4 | 4 | 0 | 0 |
| [3.19 集合関数・Bitmask変換](#3-19) | `set_function/` | 3 | 3 | 0 | 0 |
| [3.20 線形代数](#3-20) | `linear_algebra/` | 6 | 6 | 0 | 0 |
| [3.21 文字列アルゴリズム](#3-21) | `string_algorithm/` | 6 | 6 | 0 | 0 |
| [3.22 計算幾何](#3-22) | `computational_geometry/` | 7 | 7 | 0 | 0 |
| [3.23 組合せ最適化・マトロイド](#3-23) | `combinatorial_optimization/` | 4 | 4 | 0 | 0 |
| [3.24 ゲーム理論](#3-24) | `game_theory/` | 3 | 3 | 0 | 0 |
| [3.25 探索・ヒューリスティック](#3-25) | `heuristic_search/` | 5 | 5 | 0 | 0 |

## 3.1〜3.6 Bの進捗

| 分野 | B掲載数 | 実装あり | 一部対応 | 未実装 |
|---|---:|---:|---:|---:|
| 3.1 基本ユーティリティ・探索 | 7 | 7 | 0 | 0 |
| 3.2 Union-Find・連結性管理 | 5 | 5 | 0 | 0 |
| 3.3 セグメント木・Fenwick Tree | 7 | 7 | 0 | 0 |
| 3.4 静的クエリ・順序統計・集合管理 | 8 | 8 | 0 | 0 |
| 3.5 平衡二分探索木・永続データ構造 | 9 | 9 | 0 | 0 |
| 3.6 オフラインクエリ・二次元データ構造 | 7 | 7 | 0 | 0 |
| 合計 | 43 | 43 | 0 | 0 |

[3.1〜3.6のAPI・共有先・検証](b_1_6/README.md)。

## 3.7〜3.11 Bの進捗

| 分野 | B掲載数 | 実装あり | 一部対応 | 未実装 |
|---|---:|---:|---:|---:|
| 3.7 基本探索・最短路・MST | 6 | 6 | 0 | 0 |
| 3.8 連結成分・構造分解 | 7 | 7 | 0 | 0 |
| 3.9 フロー・マッチング | 7 | 7 | 0 | 0 |
| 3.10 組合せ構造・数え上げ | 7 | 7 | 0 | 0 |
| 3.11 木アルゴリズム | 9 | 9 | 0 | 0 |
| 合計 | 36 | 36 | 0 | 0 |

[APIの対応範囲・共有先・検証](b_7_11/README.md)。最大重みマッチングは二部版、動的全方位DPと直径は固定森の更新、簡易同型は1-WLの未確定を含みます。3.12〜3.15のBも下記のとおり整備しました。Python版は計画のみで未着手です。

## 3.12〜3.15 Bの進捗

| 分野 | B掲載数 | 実装あり | 一部対応 | 未実装 |
|---|---:|---:|---:|---:|
| 3.12 DP・最適化 | 7 | 7 | 0 | 0 |
| 3.13 基本整数演算 | 8 | 8 | 0 | 0 |
| 3.14 高度な整数問題 | 6 | 6 | 0 | 0 |
| 3.15 組合せ論 | 9 | 9 | 0 | 0 |
| 合計 | 30 | 30 | 0 | 0 |

[API・共有先・検証](b_12_15/README.md)。オンライン・オフライン変換はCDQ、Four RussiansはBoolean行列積、乗法的総和は線形篩prefix、標本点シフトは既存補間と多点評価の共有版です。3.16〜3.20のB全31掲載箇所も[API・比較検証・図解付き](b_16_20/README.md)で整備済み。3.21〜3.25のB全28掲載箇所も[公開API・比較検証・図解付き](b_21_25/README.md)で整備し、B全168掲載箇所が完了しました。Python版は計画のみで未着手です。

## 3.16〜3.20 Bの進捗

| カテゴリ | 掲載数 | 整備済み | 未実装 |
|---|---:|---:|---:|
| 3.16 | 7 | 7 | 0 |
| 3.17 | 8 | 8 | 0 |
| 3.18 | 6 | 6 | 0 |
| 3.19 | 4 | 4 | 0 |
| 3.20 | 6 | 6 | 0 |
| 合計 | 31 | 31 | 0 |

[共有先・API・計算量・検証](b_16_20/README.md)。重複を含む掲載箇所単位で整備し、3.1〜3.20のBは計140掲載箇所です。C項目とPython版の未着手計画は保持します。

## 3.21〜3.25 Bの進捗

| カテゴリ | 掲載数 | 整備済み | 未実装 |
|---|---:|---:|---:|
| 3.21 | 7 | 7 | 0 |
| 3.22 | 8 | 8 | 0 |
| 3.23 | 5 | 5 | 0 |
| 3.24 | 3 | 3 | 0 |
| 3.25 | 5 | 5 | 0 |
| 合計 | 28 | 28 | 0 |

[全公開API・適用範囲・比較検証](b_21_25/README.md)。3.1〜3.25のBは重複を含む全168掲載箇所を整備しました。既存の最遠点対・Manhattan MST等は共通利用。半平面交差は明示した矩形内、線形パリティは厳密小規模版とMonte Carlo代数版を分けます。S/CとPython版計画は保持します。

## 重複項目・取り違えやすい箇所

- Mo on Tree（3.6／3.11）、Functional Graph（3.7／3.8）、Cartesian Tree（3.1／3.11）、集合変換・畳み込み（3.16／3.19）、フロー・割当（3.9／3.23）は共通実装を使い、関係するチェックをまとめて更新します。
- Persistent Segtreeのpoint get/setは永続配列として利用できます。一方、永続Stack／QueueやRollback DSUは永続Union-Findの代用ではありません。
- 任意mod畳み込みの内部CRTは、一般の合同式を解くCRT／Garnerの公開APIとは別です。Subset Convolutionのrank別変換は、今回追加した公開Ranked Zeta APIを共有します。
- Implicit Treapと汎用作用付きLazy Reversible Treapは別APIとして整備済みです。既存Beam Searchは重複を除去する契約ではありません。
- 新二次元Segtreeはrange_query/segtree_2d.hpp。旧 `data_structure/2D_segtree.hpp` はWIPのまま保持。旧 `graph_theory/funtional_graph.hpp` は説明雛形のまま、新APIはgraph_basic/functional_graph.hppです。
- 行列式・階数・Lagrange補間・線形篩・ゲームの直和など、他カテゴリや既存APIの返り値で既に満たせるものは再実装しません。追加の高速化や対象型の拡張は別の要件として扱います。

## 全実装予定リスト

<a id="3-1"></a>

### 3.1 基本ユーティリティ・探索

主な配置先: `basic_utility/`。

- **S**：二分探索、三分探索、座標圧縮、累積和、二次元累積和、いもす法、二次元いもす法、差分配列、ランレングス圧縮、転倒数、床・天井除算、整数平方根、整数累乗、chmin/chmax

**A（優先実装）**

- [x] ダブリング — **実装あり**。 [tree/binary_lifting.hpp](../tree/binary_lifting.hpp)。 終端・循環を含む遷移の反復。
- [x] 循環列の操作 — **実装あり**。 [basic_utility/cyclic_sequence.hpp](../basic_utility/cyclic_sequence.hpp)。 [説明](basic_utility/cyclic_sequence.md)。
- [x] 区間のマージ・分割 — **実装あり**。 [basic_utility/interval_operations.hpp](../basic_utility/interval_operations.hpp)。 [説明](basic_utility/interval_operations.md)。
- [x] 商列挙 — **実装あり**。 [basic_utility/quotient_enumeration.hpp](../basic_utility/quotient_enumeration.hpp)。 [説明](basic_utility/quotient_enumeration.md)。
- [x] 倍数・約数列挙 — **実装あり**。 [basic_utility/multiple_enumeration.hpp](../basic_utility/multiple_enumeration.hpp)。 [説明](basic_utility/multiple_enumeration.md)。約数は既存のnumber_theory_basic/divisors.hppを共有。
- [x] Gray Code — **実装あり**。 [basic_utility/gray_code.hpp](../basic_utility/gray_code.hpp)。 [説明](basic_utility/gray_code.md)。
- [x] 部分集合列挙 — **実装あり**。 [set_function/submask_enumeration.hpp](../set_function/submask_enumeration.hpp)。 全体maskのsubmask列挙で部分集合を列挙できる。
- [x] 部分集合の部分集合列挙 — **実装あり**。 [basic_utility/subset_pairs.hpp](../basic_utility/subset_pairs.hpp)。 [説明](basic_utility/subset_pairs.md)。既存for_each_submaskを共有し全(S,T⊆S)を列挙。
- [x] 順列のランク・アンランク — **実装あり**。 [basic_utility/permutation_rank.hpp](../basic_utility/permutation_rank.hpp)。 [説明](basic_utility/permutation_rank.md)。
- [x] Cartesian Tree — **旧実装あり**。 [data_structure/cartesian_tree.hpp](../data_structure/cartesian_tree.hpp)。 旧API。再実装せず必要なら移行入口・説明・検証だけを補う。
- [x] XOR基底 — **実装あり**。 [linear_algebra/xor_basis.hpp](../linear_algebra/xor_basis.hpp)。

**B**

- [x] Fractional Cascading — **実装あり**。 [basic_utility/fractional_cascading.hpp](../basic_utility/fractional_cascading.hpp) / [説明](basic_utility/fractional_cascading.md)。
- [x] 整数 k 乗根 — **実装あり**。 [basic_utility/integer_kth_root.hpp](../basic_utility/integer_kth_root.hpp) / [説明](basic_utility/integer_kth_root.md)。
- [x] Stern–Brocot Tree — **実装あり**。 [basic_utility/stern_brocot_tree.hpp](../basic_utility/stern_brocot_tree.hpp) / [説明](basic_utility/stern_brocot_tree.md)。
- [x] 分数二分探索 — **実装あり**。 [basic_utility/fraction_binary_search.hpp](../basic_utility/fraction_binary_search.hpp) / [説明](basic_utility/fraction_binary_search.md)。
- [x] 平方分割テンプレート — **実装あり**。 [basic_utility/sqrt_decomposition.hpp](../basic_utility/sqrt_decomposition.hpp) / [説明](basic_utility/sqrt_decomposition.md)。
- [x] 非単調関数の探索 — **実装あり**。 [basic_utility/nonmonotone_search.hpp](../basic_utility/nonmonotone_search.hpp) / [説明](basic_utility/nonmonotone_search.md)。 有限整数区間の分枝限定。正しい区間下界が必要。
- [x] 連続区間集合の管理 — **実装あり**。 [basic_utility/range_set.hpp](../basic_utility/range_set.hpp) / [説明](basic_utility/range_set.md)。 3.6の既存RangeSetを共有。

- **C**：部分永続の探索補助、Retroactiveな操作管理、特殊な整数列挙

補足：日常的に書き直す処理を最優先で共通化します。

<a id="3-2"></a>

### 3.2 Union-Find・連結性管理

主な配置先: `union_find/`。

- **S**：通常のUnion-Find、重み付きUnion-Find（Potentialized DSU）、連結成分サイズ・個数管理

**A（優先実装）**

- [x] Undo/Rollback Union-Find — **実装あり**。 [union_find/rollback_disjoint_set_union.hpp](../union_find/rollback_disjoint_set_union.hpp)。
- [x] 部分永続Union-Find — **実装あり**。 [union_find/partially_persistent_disjoint_set_union.hpp](../union_find/partially_persistent_disjoint_set_union.hpp)。 [説明](union_find/partially_persistent_disjoint_set_union.md)。
- [x] 完全永続Union-Find — **実装あり**。 [union_find/persistent_disjoint_set_union.hpp](../union_find/persistent_disjoint_set_union.hpp)。 [説明](union_find/persistent_disjoint_set_union.md)。
- [x] 連結成分のモノイド集約 — **実装あり**。 [union_find/aggregate_disjoint_set_union.hpp](../union_find/aggregate_disjoint_set_union.hpp)。 [説明](union_find/aggregate_disjoint_set_union.md)。
- [x] 二部グラフ判定付きUnion-Find — **実装あり**。 [union_find/bipartite_disjoint_set_union.hpp](../union_find/bipartite_disjoint_set_union.hpp)。 [説明](union_find/bipartite_disjoint_set_union.md)。
- [x] 区間Union-Find — **実装あり**。 [union_find/interval_disjoint_set_union.hpp](../union_find/interval_disjoint_set_union.hpp)。 [説明](union_find/interval_disjoint_set_union.md)。
- [x] マージ履歴木（Merge Tree） — **実装あり**。 [union_find/merge_tree.hpp](../union_find/merge_tree.hpp)。 [説明](union_find/merge_tree.md)。

**B**

- [x] Offline Dynamic Connectivity — **実装あり**。 [union_find/offline_dynamic_connectivity.hpp](../union_find/offline_dynamic_connectivity.hpp) / [説明](union_find/offline_dynamic_connectivity.md)。 3.6の既存実装を共有。
- [x] Parallel Union-Find — **実装あり**。 [union_find/parallel_disjoint_set_union.hpp](../union_find/parallel_disjoint_set_union.hpp) / [説明](union_find/parallel_disjoint_set_union.md)。 区間同士をoffsetごとに併合。スレッド並列ではない。
- [x] 動的Union-Find — **実装あり**。 [union_find/dynamic_disjoint_set_union.hpp](../union_find/dynamic_disjoint_set_union.hpp) / [説明](union_find/dynamic_disjoint_set_union.md)。 未知ラベルの頂点追加。オンライン辺削除は対象外。
- [x] 加算などの操作付きDSU — **実装あり**。 [union_find/additive_disjoint_set_union.hpp](../union_find/additive_disjoint_set_union.hpp) / [説明](union_find/additive_disjoint_set_union.md)。
- [x] Incremental Bridge Connectivity — **実装あり**。 [union_find/incremental_bridge_connectivity.hpp](../union_find/incremental_bridge_connectivity.hpp) / [説明](union_find/incremental_bridge_connectivity.md)。

- **C**：非可換群上のPotentialized DSU、一般的なFully Dynamic Connectivity

補足：特に Rollback DSU + 時間軸セグ木 は、削除を含む連結性クエリのためにセットで用意したい構成です。

<a id="3-3"></a>

### 3.3 セグメント木・Fenwick Tree

主な配置先: `range_query/`。

- **S**：Fenwick Tree、Segtree、Lazy Segtree、Dual Segtree、Fenwick Treeの lower_bound、区間加算・区間和BIT

**A（優先実装）**

- [x] Dynamic Segtree — **実装あり**。 [range_query/dynamic_segtree.hpp](../range_query/dynamic_segtree.hpp)。 [説明](range_query/dynamic_segtree.md)。
- [x] Dynamic Lazy Segtree — **実装あり**。 [range_query/dynamic_lazy_segtree.hpp](../range_query/dynamic_lazy_segtree.hpp)。 [説明](range_query/dynamic_lazy_segtree.md)。
- [x] Persistent Segtree — **実装あり**。 [ordered_structure/persistent_segment_tree.hpp](../ordered_structure/persistent_segment_tree.hpp)。 区間k番目は ordered_structure/persistent_order_statistics.hpp。
- [x] Segment Tree Beats — **実装あり**。 [range_query/segment_tree_beats.hpp](../range_query/segment_tree_beats.hpp)。 chmin/chmax/addとmin/max/sum。原稿推奨のassignはchmin→chmaxで合成可能、専用range_assign APIはない。
- [x] 二次元Fenwick Tree — **実装あり**。 [range_query/fenwick_tree_2d.hpp](../range_query/fenwick_tree_2d.hpp)。 [説明](range_query/fenwick_tree_2d.md)。
- [x] 二次元Segtree — **実装あり**。 [range_query/segtree_2d.hpp](../range_query/segtree_2d.hpp)。 [説明](range_query/segtree_2d.md)。旧WIPは保持し、新しい完成APIを追加。
- [x] Range Add/Assign/Affine のLazy Segtree用モノイド — **実装あり**。 [range_query/range_actions.hpp](../range_query/range_actions.hpp)。 [説明](range_query/range_actions.md)。

**B**

- [x] Sparse Dynamic Segtree — **実装あり**。 [range_query/sparse_dynamic_segtree.hpp](../range_query/sparse_dynamic_segtree.hpp) / [説明](range_query/sparse_dynamic_segtree.md)。 既存DynamicSegtreeの疎な節点確保を共有。
- [x] Rollback Segtree — **実装あり**。 [range_query/rollback_segtree.hpp](../range_query/rollback_segtree.hpp) / [説明](range_query/rollback_segtree.md)。
- [x] Persistent Lazy Segtree — **実装あり**。 [range_query/persistent_lazy_segtree.hpp](../range_query/persistent_lazy_segtree.hpp) / [説明](range_query/persistent_lazy_segtree.md)。
- [x] Sortable Segtree — **実装あり**。 [range_query/sortable_segtree.hpp](../range_query/sortable_segtree.hpp) / [説明](range_query/sortable_segtree.md)。 一意な整数keyと非可換payloadの区間sort/集約。
- [x] Range Tree — **実装あり**。 [range_query/range_tree.hpp](../range_query/range_tree.hpp) / [説明](range_query/range_tree.md)。
- [x] Segtree with Fractional Cascading — **実装あり**。 [range_query/segtree_fractional_cascading.hpp](../range_query/segtree_fractional_cascading.hpp) / [説明](range_query/segtree_fractional_cascading.md)。
- [x] 2D Dual Segtree — **実装あり**。 [range_query/dual_segtree_2d.hpp](../range_query/dual_segtree_2d.hpp) / [説明](range_query/dual_segtree_2d.md)。 可換な作用モノイド。

- **C**：Kinetic Segment Tree、RBST-based Segment Tree、複雑な範囲変換をサポートするBeats

補足：最も実用性の高い分野の一つです。

Segment Tree Beats は range chmin/chmax/add/assign と range min/max/sum を扱うものを一つ用意すると、多くの問題に対応できます。

参考：ei1333、maspypy、sotanishy。

<a id="3-4"></a>

### 3.4 静的クエリ・順序統計・集合管理

主な配置先: `static_query/`。

- **S**：Sparse Table、Monotonic Stack、Monotonic Queue、Sliding Window Minimum、SWAG（Sliding Window Aggregation）

**A（優先実装）**

- [x] Disjoint Sparse Table — **実装あり**。 [static_query/disjoint_sparse_table.hpp](../static_query/disjoint_sparse_table.hpp)。 [説明](static_query/disjoint_sparse_table.md)。
- [x] Wavelet Matrix — **実装あり**。 [static_query/wavelet_matrix.hpp](../static_query/wavelet_matrix.hpp)。
- [x] Binary Trie — **実装あり**。 [static_query/binary_trie.hpp](../static_query/binary_trie.hpp)。 [説明](static_query/binary_trie.md)。
- [x] Li Chao Tree — **実装あり**。 [dp_optimization/li_chao_tree.hpp](../dp_optimization/li_chao_tree.hpp)。
- [x] Convex Hull Trick — **旧実装あり**。 [data_structure/ConvexHullTrick.hpp](../data_structure/ConvexHullTrick.hpp)。 旧API。再実装せず必要なら移行入口・説明・検証だけを補う。
- [x] Fast Set / Predecessor Set — **実装あり**。 [static_query/fast_set.hpp](../static_query/fast_set.hpp)。 [説明](static_query/fast_set.md)。
- [x] Ordered Set — **実装あり**。 [ordered_structure/ordered_set.hpp](../ordered_structure/ordered_set.hpp)。 [説明](ordered_structure/ordered_set.md)。3.4のstatic_query/ordered_set.hppは共通入口。

**B**

- [x] Wavelet Matrixの区間和拡張 — **実装あり**。 [static_query/weighted_wavelet_matrix.hpp](../static_query/weighted_wavelet_matrix.hpp) / [説明](static_query/weighted_wavelet_matrix.md)。
- [x] Wavelet Tree — **実装あり**。 [static_query/wavelet_tree.hpp](../static_query/wavelet_tree.hpp) / [説明](static_query/wavelet_tree.md)。
- [x] Persistent Binary Trie — **実装あり**。 [static_query/persistent_binary_trie.hpp](../static_query/persistent_binary_trie.hpp) / [説明](static_query/persistent_binary_trie.md)。
- [x] Priority Sum（上位・下位k個の和） — **実装あり**。 [static_query/priority_sum.hpp](../static_query/priority_sum.hpp) / [説明](static_query/priority_sum.md)。
- [x] Dynamic Bitset — **実装あり**。 [static_query/dynamic_bitset.hpp](../static_query/dynamic_bitset.hpp) / [説明](static_query/dynamic_bitset.md)。
- [x] Succinct Bit Vector — **実装あり**。 [static_query/succinct_bit_vector.hpp](../static_query/succinct_bit_vector.hpp) / [説明](static_query/succinct_bit_vector.md)。 word+prefix形式。rank O(1)、select O(log N)。
- [x] Linear RMQ — **実装あり**。 [static_query/linear_rmq.hpp](../static_query/linear_rmq.hpp) / [説明](static_query/linear_rmq.md)。
- [x] Sqrt Tree — **実装あり**。 [static_query/sqrt_tree.hpp](../static_query/sqrt_tree.hpp) / [説明](static_query/sqrt_tree.md)。

- **C**：Static Range Mode、Static Range LIS、静的転倒数クエリ、動的Wavelet Matrix、特殊なRank/Select構造

補足：Wavelet Matrixは、rank、kth_smallest、range_freq、predecessor/successor を一つのAPIにまとめるのがよいです。

<a id="3-5"></a>

### 3.5 平衡二分探索木・永続データ構造

主な配置先: `ordered_structure/`。

- **S**：std::set/map と multiset を使った典型操作、座標圧縮とBITによるOrder Statistics

**A（優先実装）**

- [x] Treap — **実装あり**。 [ordered_structure/treap.hpp](../ordered_structure/treap.hpp)。 [説明](ordered_structure/treap.md)。
- [x] RBST — **実装あり**。 [ordered_structure/rbst.hpp](../ordered_structure/rbst.hpp)。 [説明](ordered_structure/rbst.md)。
- [x] Implicit Treap — **実装あり**。 [ordered_structure/implicit_treap.hpp](../ordered_structure/implicit_treap.hpp)。
- [x] Lazy Reversible Treap — **実装あり**。 [ordered_structure/lazy_reversible_treap.hpp](../ordered_structure/lazy_reversible_treap.hpp)。 [説明](ordered_structure/lazy_reversible_treap.md)。
- [x] 永続配列 — **実装あり**。 [ordered_structure/persistent_segment_tree.hpp](../ordered_structure/persistent_segment_tree.hpp)。 get/setと版の分岐で永続配列として利用可能。専用ラッパを重複実装する必要はない。
- [x] 永続Stack — **実装あり**。 [ordered_structure/persistent_stack.hpp](../ordered_structure/persistent_stack.hpp)。 [説明](ordered_structure/persistent_stack.md)。
- [x] 永続Queue — **実装あり**。 [ordered_structure/persistent_queue.hpp](../ordered_structure/persistent_queue.hpp)。 [説明](ordered_structure/persistent_queue.md)。

**B**

- [x] Splay Tree — **実装あり**。 [ordered_structure/splay_tree.hpp](../ordered_structure/splay_tree.hpp) / [説明](ordered_structure/splay_tree.md)。
- [x] Red-Black Tree — **実装あり**。 [ordered_structure/red_black_tree.hpp](../ordered_structure/red_black_tree.hpp) / [説明](ordered_structure/red_black_tree.md)。
- [x] Weight-Balanced Tree — **実装あり**。 [ordered_structure/weight_balanced_tree.hpp](../ordered_structure/weight_balanced_tree.hpp) / [説明](ordered_structure/weight_balanced_tree.md)。
- [x] Lazy Balanced BST — **実装あり**。 [ordered_structure/lazy_balanced_bst.hpp](../ordered_structure/lazy_balanced_bst.hpp) / [説明](ordered_structure/lazy_balanced_bst.md)。 既存LazyReversibleTreapを共有。
- [x] 永続平衡BST — **実装あり**。 [ordered_structure/persistent_balanced_bst.hpp](../ordered_structure/persistent_balanced_bst.hpp) / [説明](ordered_structure/persistent_balanced_bst.md)。
- [x] Skew Heap — **実装あり**。 [ordered_structure/skew_heap.hpp](../ordered_structure/skew_heap.hpp) / [説明](ordered_structure/skew_heap.md)。
- [x] Leftist Heap — **実装あり**。 [ordered_structure/leftist_heap.hpp](../ordered_structure/leftist_heap.hpp) / [説明](ordered_structure/leftist_heap.md)。
- [x] Radix Heap — **実装あり**。 [ordered_structure/radix_heap.hpp](../ordered_structure/radix_heap.hpp) / [説明](ordered_structure/radix_heap.md)。 既存iwiwi実装を共有し、符号なし32/64bitに限定。
- [x] Erasable Priority Queue — **実装あり**。 [ordered_structure/erasable_priority_queue.hpp](../ordered_structure/erasable_priority_queue.hpp) / [説明](ordered_structure/erasable_priority_queue.md)。

- **C**：Finger Tree、Partially Retroactive Priority Queue、完全永続Lazy Balanced BST

補足：TreapやRBSTは「値の集合」と「暗黙キーによる列操作」を分けて実装すると扱いやすいです。

<a id="3-6"></a>

### 3.6 オフラインクエリ・二次元データ構造

主な配置先: `offline_query/`。

- **S**：Mo's Algorithm、Parallel Binary Search（並列二分探索）、Sweep Line、CDQ Divide and Conquerの基本形

**A（優先実装）**

- [x] Mo with Updates（時空間Mo） — **実装あり**。 [offline_query/mo_with_updates.hpp](../offline_query/mo_with_updates.hpp)。
- [x] Mo on Tree — **実装あり**。 [offline_query/mo_on_tree.hpp](../offline_query/mo_on_tree.hpp)。 [説明](offline_query/mo_on_tree.md)。
- [x] Rollback Mo — **実装あり**。 [offline_query/rollback_mo.hpp](../offline_query/rollback_mo.hpp)。 [説明](offline_query/rollback_mo.md)。
- [x] 二次元BIT（座圧版） — **実装あり**。 [offline_query/compressed_fenwick_tree_2d.hpp](../offline_query/compressed_fenwick_tree_2d.hpp)。 [説明](offline_query/compressed_fenwick_tree_2d.md)。
- [x] Static Rectangle Sum — **実装あり**。 [offline_query/sweep_line.hpp](../offline_query/sweep_line.hpp)。
- [x] Dynamic Point Add Rectangle Sum — **実装あり**。 [offline_query/cdq_rectangle_sum.hpp](../offline_query/cdq_rectangle_sum.hpp)。 登録操作列に対するオフライン処理。オンライン2D構造は別項目。
- [x] Range Set — **実装あり**。 [offline_query/range_set.hpp](../offline_query/range_set.hpp)。 [説明](offline_query/range_set.md)。
- [x] Offline RMQ — **実装あり**。 [offline_query/offline_rmq.hpp](../offline_query/offline_rmq.hpp)。 [説明](offline_query/offline_rmq.md)。

**B**

- [x] 長方形加算・長方形和 — **実装あり**。 [offline_query/rectangle_add_rectangle_sum.hpp](../offline_query/rectangle_add_rectangle_sum.hpp) / [説明](offline_query/rectangle_add_rectangle_sum.md)。
- [x] 長方形加算・点取得 — **実装あり**。 [offline_query/rectangle_add_point_get.hpp](../offline_query/rectangle_add_point_get.hpp) / [説明](offline_query/rectangle_add_point_get.md)。
- [x] Kd-Tree — **実装あり**。 [offline_query/kd_tree.hpp](../offline_query/kd_tree.hpp) / [説明](offline_query/kd_tree.md)。
- [x] Range Tree — **実装あり**。 [offline_query/range_tree.hpp](../offline_query/range_tree.hpp) / [説明](offline_query/range_tree.md)。 3.3のRange Treeを共有。
- [x] Hilbert Order Mo — **実装あり**。 [offline_query/hilbert_order_mo.hpp](../offline_query/hilbert_order_mo.hpp) / [説明](offline_query/hilbert_order_mo.md)。
- [x] Static Range Count Distinct — **実装あり**。 [offline_query/static_range_count_distinct.hpp](../offline_query/static_range_count_distinct.hpp) / [説明](offline_query/static_range_count_distinct.md)。
- [x] Dynamic Rectangle Affine Sum — **実装あり**。 [offline_query/dynamic_rectangle_affine_sum.hpp](../offline_query/dynamic_rectangle_affine_sum.hpp) / [説明](offline_query/dynamic_rectangle_affine_sum.md)。 オンライン点挿入・点代入・矩形affine/和。

- **C**：三次元以上の直交範囲クエリ、特殊な動的最近傍探索、汎用多次元Fenwick Tree

補足：これらはmaspypy、ei1333のライブラリで特に豊富です。

<a id="3-7"></a>

### 3.7 グラフ：基本探索・最短路・MST

主な配置先: `graph_basic/`。

- **S**：BFS、DFS、0-1 BFS、Dijkstra、Bellman–Ford、Warshall–Floyd、トポロジカルソート、閉路検出、Kruskal、Prim、経路復元

**A（優先実装）**

- [x] Radix Heap Dijkstra — **実装あり**。 [graph_basic/radix_heap_dijkstra.hpp](../graph_basic/radix_heap_dijkstra.hpp)。 [説明](graph_basic/radix_heap_dijkstra.md)。
- [x] Dial's Algorithm — **実装あり**。 [graph_basic/dial_shortest_path.hpp](../graph_basic/dial_shortest_path.hpp)。 [説明](graph_basic/dial_shortest_path.md)。
- [x] DAG最短路 — **実装あり**。 [graph_basic/dag_shortest_path.hpp](../graph_basic/dag_shortest_path.hpp)。 [説明](graph_basic/dag_shortest_path.md)。
- [x] Johnson's Algorithm — **実装あり**。 [graph_basic/johnson.hpp](../graph_basic/johnson.hpp)。 [説明](graph_basic/johnson.md)。
- [x] K-th Shortest Path — **実装あり**。 [graph_basic/k_shortest_paths.hpp](../graph_basic/k_shortest_paths.hpp)。 [説明](graph_basic/k_shortest_paths.md)。Yen法、頂点重複なし。
- [x] K-th Shortest Walk — **実装あり**。 [graph_basic/k_shortest_walks.hpp](../graph_basic/k_shortest_walks.hpp)。 [説明](graph_basic/k_shortest_walks.md)。Eppstein法、繰り返しを許し遅延復元。
- [x] 補グラフBFS — **実装あり**。 [graph_basic/complement_bfs.hpp](../graph_basic/complement_bfs.hpp)。 [説明](graph_basic/complement_bfs.md)。
- [x] Functional Graph — **実装あり**。 [graph_basic/functional_graph.hpp](../graph_basic/functional_graph.hpp)。 [説明](graph_basic/functional_graph.md)。

**B**

- [x] Directed MST（Chu–Liu/Edmonds） — **実装あり**。 [graph_basic/directed_mst.hpp](../graph_basic/directed_mst.hpp) / [説明](graph_basic/directed_mst.md)。
- [x] Manhattan MST — **実装あり**。 [graph_basic/manhattan_mst.hpp](../graph_basic/manhattan_mst.hpp) / [説明](graph_basic/manhattan_mst.md)。
- [x] 最短有向閉路 — **実装あり**。 [graph_basic/shortest_directed_cycle.hpp](../graph_basic/shortest_directed_cycle.hpp) / [説明](graph_basic/shortest_directed_cycle.md)。
- [x] 最短無向閉路 — **実装あり**。 [graph_basic/shortest_undirected_cycle.hpp](../graph_basic/shortest_undirected_cycle.hpp) / [説明](graph_basic/shortest_undirected_cycle.md)。
- [x] 最小全域木の感度解析 — **実装あり**。 [graph_basic/mst_sensitivity.hpp](../graph_basic/mst_sensitivity.hpp) / [説明](graph_basic/mst_sensitivity.md)。
- [x] 全域木の第二最小コスト — **実装あり**。 [graph_basic/second_best_mst.hpp](../graph_basic/second_best_mst.hpp) / [説明](graph_basic/second_best_mst.md)。

- **C**：高速な特殊重み最短路、グラフ制約付きShortest Nonzero Path、Minimum Diameter Spanning Tree

補足：最短路は経路復元対応版を最初から作成するのがおすすめです。

<a id="3-8"></a>

### 3.8 グラフ：連結成分・構造分解

主な配置先: `graph_connectivity/`。

- **S**：SCC（強連結成分）、Lowlink、橋・関節点、二部グラフ判定

**A（優先実装）**

- [x] 二重辺連結成分 — **実装あり**。 [graph/two_edge_connected_components.hpp](../graph/two_edge_connected_components.hpp)。
- [x] 二重頂点連結成分 — **実装あり**。 [graph/biconnected_components.hpp](../graph/biconnected_components.hpp)。
- [x] Block-Cut Tree — **実装あり**。 [graph_connectivity/block_cut_tree.hpp](../graph_connectivity/block_cut_tree.hpp)。 [説明](graph_connectivity/block_cut_tree.md)。既存graph/biconnected_components.hppのblock_cut_forestを共有。
- [x] Bridge Tree — **実装あり**。 [graph/two_edge_connected_components.hpp](../graph/two_edge_connected_components.hpp)。 返り値forestが橋の縮約木。
- [x] 2-SAT — **実装あり**。 [graph_connectivity/two_sat.hpp](../graph_connectivity/two_sat.hpp)。 [説明](graph_connectivity/two_sat.md)。
- [x] Eulerian Trail — **実装あり**。 [graph_connectivity/eulerian_trail.hpp](../graph_connectivity/eulerian_trail.hpp)。 [説明](graph_connectivity/eulerian_trail.md)。
- [x] サイクル復元 — **実装あり**。 [graph_basic/cycle.hpp](../graph_basic/cycle.hpp)。 頂点列と辺ID列を復元。
- [x] Functional Graph分解 — **実装あり**。 [graph_connectivity/functional_graph.hpp](../graph_connectivity/functional_graph.hpp)。 [説明](graph_connectivity/functional_graph.md)。3.7と共通実装。

**B**

- [x] Dominator Tree — **実装あり**。 [graph_connectivity/dominator_tree.hpp](../graph_connectivity/dominator_tree.hpp) / [説明](graph_connectivity/dominator_tree.md)。
- [x] 三重辺連結成分 — **実装あり**。 [graph_connectivity/three_edge_connected_components.hpp](../graph_connectivity/three_edge_connected_components.hpp) / [説明](graph_connectivity/three_edge_connected_components.md)。
- [x] Cactus Graph分解 — **実装あり**。 [graph_connectivity/cactus_decomposition.hpp](../graph_connectivity/cactus_decomposition.hpp) / [説明](graph_connectivity/cactus_decomposition.md)。
- [x] Chordal Graph判定 — **実装あり**。 [graph_connectivity/chordal_graph.hpp](../graph_connectivity/chordal_graph.hpp) / [説明](graph_connectivity/chordal_graph.md)。
- [x] Tree Decomposition（幅2） — **実装あり**。 [graph_connectivity/tree_decomposition_width_2.hpp](../graph_connectivity/tree_decomposition_width_2.hpp) / [説明](graph_connectivity/tree_decomposition_width_2.md)。
- [x] Incremental SCC — **実装あり**。 [graph_connectivity/incremental_scc.hpp](../graph_connectivity/incremental_scc.hpp) / [説明](graph_connectivity/incremental_scc.md)。
- [x] DAG Reachability — **実装あり**。 [graph_connectivity/dag_reachability.hpp](../graph_connectivity/dag_reachability.hpp) / [説明](graph_connectivity/dag_reachability.md)。

- **C**：SPQR Tree、一般グラフの木幅分解、特殊なグラフの標準形

<a id="3-9"></a>

### 3.9 グラフ：フロー・マッチング・最適化

主な配置先: `graph_flow/`。

- **S**：Dinic最大流、最小カット復元、二部最大マッチング

**A（優先実装）**

- [x] 最小費用流（Primal-Dual） — **実装あり**。 [combinatorial_optimization/min_cost_flow.hpp](../combinatorial_optimization/min_cost_flow.hpp)。
- [x] Hopcroft–Karp — **実装あり**。 [graph_flow/bipartite_matching.hpp](../graph_flow/bipartite_matching.hpp)。
- [x] Hungarian Algorithm（割当問題） — **実装あり**。 [combinatorial_optimization/hungarian.hpp](../combinatorial_optimization/hungarian.hpp)。
- [x] 下限制約付きフロー — **実装あり**。 [graph_flow/lower_bound_flow.hpp](../graph_flow/lower_bound_flow.hpp)。 [説明](graph_flow/lower_bound_flow.md)。
- [x] 一般グラフの最大マッチング（Blossom） — **実装あり**。 [combinatorial_optimization/general_matching.hpp](../combinatorial_optimization/general_matching.hpp)。
- [x] Project Selection Problem（燃やす埋める） — **実装あり**。 [graph_flow/project_selection.hpp](../graph_flow/project_selection.hpp)。 [説明](graph_flow/project_selection.md)。

**B**

- [x] Min-Cost b-Flow — **実装あり**。 [graph_flow/min_cost_b_flow.hpp](../graph_flow/min_cost_b_flow.hpp) / [説明](graph_flow/min_cost_b_flow.md)。
- [x] 一般グラフの最小カット（Stoer–Wagner） — **実装あり**。 [graph_flow/stoer_wagner.hpp](../graph_flow/stoer_wagner.hpp) / [説明](graph_flow/stoer_wagner.md)。
- [x] Push-Relabel — **実装あり**。 [graph_flow/push_relabel.hpp](../graph_flow/push_relabel.hpp) / [説明](graph_flow/push_relabel.md)。
- [x] 最小費用循環流 — **実装あり**。 [graph_flow/min_cost_circulation.hpp](../graph_flow/min_cost_circulation.hpp) / [説明](graph_flow/min_cost_circulation.md)。
- [x] DM分解 — **実装あり**。 [graph_flow/dulmage_mendelsohn.hpp](../graph_flow/dulmage_mendelsohn.hpp) / [説明](graph_flow/dulmage_mendelsohn.md)。
- [x] 二部グラフ辺彩色 — **実装あり**。 [graph_flow/bipartite_edge_coloring.hpp](../graph_flow/bipartite_edge_coloring.hpp) / [説明](graph_flow/bipartite_edge_coloring.md)。
- [x] 最大重み二部マッチング — **実装あり**。 [graph_flow/maximum_weight_bipartite_matching.hpp](../graph_flow/maximum_weight_bipartite_matching.hpp) / [説明](graph_flow/maximum_weight_bipartite_matching.md)。 二部版。一般グラフの重み付きBlossomはC。

- **C**：一般グラフの最大重みマッチング（Blossomの重み付き版）、Rank-Maximal Matching、特殊な動的フロー

<a id="3-10"></a>

### 3.10 グラフ：組合せ構造・数え上げ

主な配置先: `graph_combinatorics/`。

**A（優先実装）**

- [x] 三角形列挙 — **実装あり**。 [graph_combinatorics/triangle_enumeration.hpp](../graph_combinatorics/triangle_enumeration.hpp)。
- [x] クリーク列挙 — **実装あり**。 [graph_combinatorics/clique_enumeration.hpp](../graph_combinatorics/clique_enumeration.hpp)。
- [x] 最大クリーク — **実装あり**。 [graph_combinatorics/maximum_clique.hpp](../graph_combinatorics/maximum_clique.hpp)。
- [x] 最大独立集合 — **実装あり**。 [graph_combinatorics/maximum_independent_set.hpp](../graph_combinatorics/maximum_independent_set.hpp)。
- [x] 彩色数 — **実装あり**。 [graph_combinatorics/chromatic_number.hpp](../graph_combinatorics/chromatic_number.hpp)。
- [x] 全域木の個数（行列木定理） — **実装あり**。 [graph_combinatorics/matrix_tree.hpp](../graph_combinatorics/matrix_tree.hpp)。

**B**

- [x] 彩色多項式 — **実装あり**。 [graph_combinatorics/chromatic_polynomial.hpp](../graph_combinatorics/chromatic_polynomial.hpp) / [説明](graph_combinatorics/chromatic_polynomial.md)。
- [x] C4の個数 — **実装あり**。 [graph_combinatorics/count_four_cycles.hpp](../graph_combinatorics/count_four_cycles.hpp) / [説明](graph_combinatorics/count_four_cycles.md)。
- [x] 極大クリーク列挙 — **実装あり**。 [graph_combinatorics/maximal_clique_enumeration.hpp](../graph_combinatorics/maximal_clique_enumeration.hpp) / [説明](graph_combinatorics/maximal_clique_enumeration.md)。 既存実体を共有。
- [x] Steiner Tree DP — **実装あり**。 [graph_combinatorics/steiner_tree.hpp](../graph_combinatorics/steiner_tree.hpp) / [説明](graph_combinatorics/steiner_tree.md)。
- [x] DAGの最小Path Cover — **実装あり**。 [graph_combinatorics/dag_minimum_path_cover.hpp](../graph_combinatorics/dag_minimum_path_cover.hpp) / [説明](graph_combinatorics/dag_minimum_path_cover.md)。
- [x] 辺素・頂点素パス — **実装あり**。 [graph_combinatorics/disjoint_paths.hpp](../graph_combinatorics/disjoint_paths.hpp) / [説明](graph_combinatorics/disjoint_paths.md)。
- [x] グラフ同型の簡易判定 — **実装あり**。 [graph_combinatorics/graph_isomorphism_filter.hpp](../graph_combinatorics/graph_isomorphism_filter.hpp) / [説明](graph_combinatorics/graph_isomorphism_filter.md)。

- **C**：ラベル付き連結グラフ数、強連結有向グラフ数、Euler閉路数（BEST定理）、一般的なグラフ同型、特殊なグラフ分解

補足：ei1333、hitonanode、maspypyには、一般マッチングや有向MST、彩色数などの実装があります。Library Checkerにもこれらの検証問題が用意されています。

<a id="3-11"></a>

### 3.11 木アルゴリズム

主な配置先: `tree/`。

- **S**：LCA、木の直径、Euler Tour、部分木サイズ・深さ、全方位木DP（Rerooting）、Binary Lifting、Tree DPの基本テンプレート

**A（優先実装）**

- [x] Heavy-Light Decomposition — **実装あり**。 [tree/heavy_light_decomposition.hpp](../tree/heavy_light_decomposition.hpp)。
- [x] Virtual Tree（Auxiliary Tree） — **実装あり**。 [tree/virtual_tree.hpp](../tree/virtual_tree.hpp)。
- [x] Centroid Decomposition — **実装あり**。 [tree/centroid_decomposition.hpp](../tree/centroid_decomposition.hpp)。
- [x] DSU on Tree — **実装あり**。 [tree/dsu_on_tree.hpp](../tree/dsu_on_tree.hpp)。
- [x] 木のハッシュ — **実装あり**。 [tree/tree_hash.hpp](../tree/tree_hash.hpp)。 [説明](tree/tree_hash.md)。二重mod、衝突あり得る。
- [x] Cartesian Tree — **旧実装あり**。 [data_structure/cartesian_tree.hpp](../data_structure/cartesian_tree.hpp)。 3.1と同じ実装。重複実装しない。
- [x] 木上のPath Query — **実装あり**。 [tree/heavy_light_decomposition.hpp](../tree/heavy_light_decomposition.hpp)。 方向付き区間をSegtree等へ渡す既存APIを使用。examples/tree_hld.cppも参照。
- [x] 木上のMo — **実装あり**。 [offline_query/mo_on_tree.hpp](../offline_query/mo_on_tree.hpp)。 [説明](offline_query/mo_on_tree.md)。3.6と共通実装。

**B**

- [x] Static Top Tree — **実装あり**。 [tree/static_top_tree.hpp](../tree/static_top_tree.hpp) / [説明](tree/static_top_tree.md)。
- [x] Link-Cut Tree（B項目） — **実装あり**。 [tree/link_cut_tree.hpp](../tree/link_cut_tree.hpp) / [説明](tree/link_cut_tree_b.md)。 既存実体を共有。
- [x] Euler Tour Tree — **実装あり**。 [tree/euler_tour_tree.hpp](../tree/euler_tour_tree.hpp) / [説明](tree/euler_tour_tree.md)。
- [x] 動的木DP（Rerooting対応） — **実装あり**。 [tree/dynamic_rerooting.hpp](../tree/dynamic_rerooting.hpp) / [説明](tree/dynamic_rerooting.md)。 固定topologyの更新。
- [x] 木の同型判定 — **実装あり**。 [tree/tree_isomorphism.hpp](../tree/tree_isomorphism.hpp) / [説明](tree/tree_isomorphism.md)。
- [x] 木の距離分布 — **実装あり**。 [tree/tree_distance_distribution.hpp](../tree/tree_distance_distribution.hpp) / [説明](tree/tree_distance_distribution.md)。
- [x] Contour Query — **実装あり**。 [tree/contour_query.hpp](../tree/contour_query.hpp) / [説明](tree/contour_query.md)。
- [x] 動的木直径 — **実装あり**。 [tree/dynamic_tree_diameter.hpp](../tree/dynamic_tree_diameter.hpp) / [説明](tree/dynamic_tree_diameter.md)。 固定topologyの更新。
- [x] 木上のマージ過程 — **実装あり**。 [tree/merge_tree.hpp](../tree/merge_tree.hpp) / [説明](tree/merge_tree.md)。 既存実体を共有。

- **C**：Dynamic Top Tree、Lazy Top Tree、動的部分木集約、Fully Dynamic Tree DP

補足：HLD + セグ木 と LCA + Virtual Tree は、組み合わせとして検証しておくと実戦で使いやすくなります。

<a id="3-12"></a>

### 3.12 DP・最適化

主な配置先: `dp_optimization/`。

- **S**：LIS、LCS、編集距離、0-1/完全/個数制限ナップサック、Bitset Knapsack、区間DP、桁DP、部分集合DP、Largest Rectangle

**A（優先実装）**

- [x] Divide and Conquer DP Optimization — **実装あり**。 [dp_optimization/divide_conquer_dp.hpp](../dp_optimization/divide_conquer_dp.hpp)。 DP段更新・到達不能値・半開区間の解復元。既存Monotone Minimaを共有。
- [x] Knuth Optimization — **実装あり**。 [dp_optimization/knuth_optimization.hpp](../dp_optimization/knuth_optimization.hpp)。 区間DP結果型と復元を共有。
- [x] Monotone Minima — **実装あり**。 [dp_optimization/monotone_minima.hpp](../dp_optimization/monotone_minima.hpp)。
- [x] SMAWK — **実装あり**。 [dp_optimization/smawk.hpp](../dp_optimization/smawk.hpp)。
- [x] Convex Hull Trick DP — **実装あり**。 [data_structure/ConvexHullTrick.hpp](../data_structure/ConvexHullTrick.hpp)。 [DPへの組込み・使用例・比較検証](dp_optimization/convex_hull_trick_dp.md)。旧CHTを共有。
- [x] Li Chao Tree DP — **実装あり**。 [dp_optimization/li_chao_tree.hpp](../dp_optimization/li_chao_tree.hpp)。 [DPへの組込み・使用例・比較検証](dp_optimization/li_chao_dp.md)。既存Li Chaoを共有。
- [x] Slope Trick — **実装あり**。 [dp_optimization/slope_trick.hpp](../dp_optimization/slope_trick.hpp)。 単位hinge、絶対値・prefix/suffix最小・区間shift・merge。
- [x] Aliens Trick（WQS Binary Search） — **実装あり**。 [dp_optimization/aliens_trick.hpp](../dp_optimization/aliens_trick.hpp)。 離散凸・整数罰則・同値なら多い個数のoracle。
- [x] SOS DP — **実装あり**。 [set_function/sos_dp.hpp](../set_function/sos_dp.hpp)。

**B**

- [x] Min-Plus Convolution（凸・凹） — **実装あり**。 [dp_optimization/min_plus_convolution.hpp](../dp_optimization/min_plus_convolution.hpp) / [説明](dp_optimization/min_plus_convolution_b.md)。 既存凸版を共有し凹×任意も追加。
- [x] Monge Shortest Path — **実装あり**。 [dp_optimization/monge_shortest_path.hpp](../dp_optimization/monge_shortest_path.hpp) / [説明](dp_optimization/monge_shortest_path.md)。
- [x] Mongeの辺数制約付き最短路 — **実装あり**。 [dp_optimization/monge_shortest_path_k_edges.hpp](../dp_optimization/monge_shortest_path_k_edges.hpp) / [説明](dp_optimization/monge_shortest_path_k_edges.md)。
- [x] LARSCH — **実装あり**。 [dp_optimization/larsch.hpp](../dp_optimization/larsch.hpp) / [説明](dp_optimization/larsch.md)。
- [x] オンライン・オフライン変換（CDQ） — **実装あり**。 [dp_optimization/online_offline_dp.hpp](../dp_optimization/online_offline_dp.hpp) / [説明](dp_optimization/online_offline_dp.md)。 CDQと畳み込みDP。
- [x] 分枝限定ナップサック — **実装あり**。 [dp_optimization/branch_bound_knapsack.hpp](../dp_optimization/branch_bound_knapsack.hpp) / [説明](dp_optimization/branch_bound_knapsack.md)。
- [x] Four Russians法（Boolean行列積） — **実装あり**。 [dp_optimization/four_russians.hpp](../dp_optimization/four_russians.hpp) / [説明](dp_optimization/four_russians.md)。 Boolean行列積のM4RM。

- **C**：高度なMonge行列積、複雑なSlope Trick、動的な最適化DP、特殊な高速ナップサック

補足：SMAWK、Monotone Minima、Min-Plus Convolutionは独立した便利な部品として整備する価値があります。

<a id="3-13"></a>

### 3.13 数論：基本整数演算

主な配置先: `number_theory_basic/`。

- **S**：GCD/LCM、拡張Euclid、mod累乗、mod逆元、素数篩、SPF（最小素因数）、素因数分解、約数列挙

**A（優先実装）**

- [x] Linear Sieve — **実装あり**。 [number_theory_basic/smallest_prime_factor.hpp](../number_theory_basic/smallest_prime_factor.hpp)。 SmallestPrimeFactorは線形篩を実装済み。
- [x] Euler φ — **実装あり**。 [number_theory_basic/euler_phi.hpp](../number_theory_basic/euler_phi.hpp)。 SPFによる線形篩と64bit単一値。
- [x] Möbius μ — **実装あり**。 [number_theory_basic/mobius.hpp](../number_theory_basic/mobius.hpp)。 SPFによる線形篩と64bit単一値。
- [x] CRT — **実装あり**。 [number_theory_basic/crt.hpp](../number_theory_basic/crt.hpp)。 一般化CRTを共有。非互いに素・矛盾・lcm超過に対応。
- [x] 中国剰余定理の一般化 — **実装あり**。 [number_theory_basic/crt.hpp](../number_theory_basic/crt.hpp)。 CRTと同じ公開APIを共有。
- [x] Garner — **実装あり**。 [number_theory_basic/garner.hpp](../number_theory_basic/garner.hpp)。 任意長の互いに素な法、任意の正の出力法。積全体は構築しない。
- [x] Floor Sum — **実装あり**。 [number_theory_basic/floor_sum.hpp](../number_theory_basic/floor_sum.hpp)。 負係数・32bitのn/m、long longの中間値超過を検査。
- [x] Miller–Rabin — **実装あり**。 [number_theory_convolution/miller_rabin.hpp](../number_theory_convolution/miller_rabin.hpp)。
- [x] Pollard's Rho — **実装あり**。 [number_theory_convolution/pollard_rho.hpp](../number_theory_convolution/pollard_rho.hpp)。
- [x] Primitive Root — **実装あり**。 [number_theory_basic/primitive_root.hpp](../number_theory_basic/primitive_root.hpp)。 素数法の最小原始根。既存64bit因数分解を共有。

**B**

- [x] Discrete Log（BSGS） — **実装あり**。 [number_theory_basic/discrete_log.hpp](../number_theory_basic/discrete_log.hpp) / [説明](number_theory_basic/discrete_log.md)。
- [x] mod平方根（Tonelli–Shanks） — **実装あり**。 [number_theory_basic/mod_sqrt.hpp](../number_theory_basic/mod_sqrt.hpp) / [説明](number_theory_basic/mod_sqrt.md)。
- [x] 整数k乗根（数論入口） — **実装あり**。 [number_theory_basic/integer_kth_root.hpp](../number_theory_basic/integer_kth_root.hpp) / [説明](number_theory_basic/integer_kth_root.md)。 3.1の実体を共有。
- [x] modテトレーション — **実装あり**。 [number_theory_basic/mod_tetration.hpp](../number_theory_basic/mod_tetration.hpp) / [説明](number_theory_basic/mod_tetration.md)。
- [x] 商列挙（数論入口） — **実装あり**。 [number_theory_basic/quotient_enumeration.hpp](../number_theory_basic/quotient_enumeration.hpp) / [説明](number_theory_basic/quotient_enumeration.md)。 3.1の実体を共有。
- [x] 線形合同方程式 — **実装あり**。 [number_theory_basic/linear_congruence.hpp](../number_theory_basic/linear_congruence.hpp) / [説明](number_theory_basic/linear_congruence.md)。
- [x] Lucasの定理 — **実装あり**。 [number_theory_basic/lucas.hpp](../number_theory_basic/lucas.hpp) / [説明](number_theory_basic/lucas.md)。
- [x] 素数カウント（共通入口） — **実装あり**。 [number_theory_basic/prime_count.hpp](../number_theory_basic/prime_count.hpp) / [説明](number_theory_basic/prime_count.md)。 3.14のLucy DPを共有。

- **C**：mod k乗根、64bit高性能modint、Gaussian Integer、連分数・有理数近似、Pell方程式、二平方数和の表示

64bit高性能modintは2026-10-19に[F-04](basic_utility/montgomery_modint.md)として整備しました（奇数法1以上2^63未満のMontgomery）。ほかの残作業は[統合一覧](remaining_work.md)を参照してください。

<a id="3-14"></a>

### 3.14 数論：乗法的関数・高度な整数問題

主な配置先: `number_theory_convolution/`。

**A（優先実装）**

- [x] 約数ゼータ変換 — **実装あり**。 [number_theory_convolution/divisor_zeta_transform.hpp](../number_theory_convolution/divisor_zeta_transform.hpp)。
- [x] 約数メビウス変換 — **実装あり**。 [number_theory_convolution/divisor_mobius_transform.hpp](../number_theory_convolution/divisor_mobius_transform.hpp)。
- [x] 倍数ゼータ変換 — **実装あり**。 [number_theory_convolution/multiple_zeta_transform.hpp](../number_theory_convolution/multiple_zeta_transform.hpp)。
- [x] Dirichlet Convolution — **実装あり**。 [number_theory_convolution/dirichlet_convolution.hpp](../number_theory_convolution/dirichlet_convolution.hpp)。
- [x] GCD/LCM Convolution — **実装あり**。 [number_theory_convolution/all.hpp](../number_theory_convolution/all.hpp)。 gcd_convolution.hpp・lcm_convolution.hpp。

**B**

- [x] 約数個数和 — **実装あり**。 [number_theory_convolution/divisor_count_sum.hpp](../number_theory_convolution/divisor_count_sum.hpp) / [説明](number_theory_convolution/divisor_count_sum.md)。
- [x] Euler φの総和 — **実装あり**。 [number_theory_convolution/summatory_totient.hpp](../number_theory_convolution/summatory_totient.hpp) / [説明](number_theory_convolution/summatory_totient.md)。
- [x] Mertens Function — **実装あり**。 [number_theory_convolution/mertens.hpp](../number_theory_convolution/mertens.hpp) / [説明](number_theory_convolution/mertens.md)。
- [x] 素数個数計算（Lucy DP） — **実装あり**。 [number_theory_convolution/prime_counting.hpp](../number_theory_convolution/prime_counting.hpp) / [説明](number_theory_convolution/prime_counting.md)。
- [x] 乗法的関数の総和 — **実装あり**。 [number_theory_convolution/multiplicative_function_sum.hpp](../number_theory_convolution/multiplicative_function_sum.hpp) / [説明](number_theory_convolution/multiplicative_function_sum.md)。 全prefixの線形篩版。
- [x] Square-Free Counting — **実装あり**。 [number_theory_convolution/square_free_counting.hpp](../number_theory_convolution/square_free_counting.hpp) / [説明](number_theory_convolution/square_free_counting.md)。

- **C**：高速素数和、一般化されたMin_25 Sieve、Dirichlet逆元、任意乗法的関数の大規模総和、特殊なmod和

<a id="3-15"></a>

### 3.15 組合せ論・数え上げ

主な配置先: `combinatorics/`。

- **S**：階乗・逆階乗、二項係数、順列・組合せ、Catalan数、重複組合せ、包除原理

**A（優先実装）**

- [x] Stirling数（第1種・第2種） — **実装あり**。 [combinatorics/stirling.hpp](../combinatorics/stirling.hpp)。 第1種の符号あり/なし、第2種。
- [x] Bell数 — **実装あり**。 [combinatorics/bell.hpp](../combinatorics/bell.hpp)。 O(N)領域のBell三角形。
- [x] 分割数 — **実装あり**。 [combinatorics/partition_numbers.hpp](../combinatorics/partition_numbers.hpp)。 五角数定理による整数分割。
- [x] Montmort数 — **実装あり**。 [combinatorics/derangements.hpp](../combinatorics/derangements.hpp)。 完全順列、合成数法も可。
- [x] Burnsideの補題 — **実装あり**。 [combinatorics/burnside.hpp](../combinatorics/burnside.hpp)。 固定点平均のmod版と正確な整数版。
- [x] Pólya Enumeration — **実装あり**。 [combinatorics/polya.hpp](../combinatorics/polya.hpp)。 全置換群のcycle index、色数・重みの代入。
- [x] Lagrange補間 — **実装あり**。 [fps/interpolation.hpp](../fps/interpolation.hpp)。 相異なる点の多項式補間は対応済み。等間隔点から1点を評価する特化版を追加するなら拡張として扱う。
- [x] 二項係数の前計算 — **実装あり**。 [combinatorics/binomial.hpp](../combinatorics/binomial.hpp)。 FactorialTableとPascalBinomialを実装済み。

**B**

- [x] Eulerian数 — **実装あり**。 [combinatorics/eulerian_numbers.hpp](../combinatorics/eulerian_numbers.hpp) / [説明](combinatorics/eulerian_numbers.md)。
- [x] Bernoulli数 — **実装あり**。 [combinatorics/bernoulli_numbers.hpp](../combinatorics/bernoulli_numbers.hpp) / [説明](combinatorics/bernoulli_numbers.md)。
- [x] Narayana数 — **実装あり**。 [combinatorics/narayana.hpp](../combinatorics/narayana.hpp) / [説明](combinatorics/narayana.md)。
- [x] Lagrange Inversion — **実装あり**。 [combinatorics/lagrange_inversion.hpp](../combinatorics/lagrange_inversion.hpp) / [説明](combinatorics/lagrange_inversion.md)。
- [x] Stirling Transform — **実装あり**。 [combinatorics/stirling_transform.hpp](../combinatorics/stirling_transform.hpp) / [説明](combinatorics/stirling_transform.md)。
- [x] 二項変換 — **実装あり**。 [combinatorics/binomial_transform.hpp](../combinatorics/binomial_transform.hpp) / [説明](combinatorics/binomial_transform.md)。
- [x] 階乗べきとの基底変換 — **実装あり**。 [combinatorics/factorial_basis.hpp](../combinatorics/factorial_basis.hpp) / [説明](combinatorics/factorial_basis.md)。
- [x] 標本点シフト — **実装あり**。 [combinatorics/shift_sample_points.hpp](../combinatorics/shift_sample_points.hpp) / [説明](combinatorics/shift_sample_points.md)。 既存補間・多点評価を共有。
- [x] 二項係数のPrefix Sum — **実装あり**。 [combinatorics/binomial_prefix_sum.hpp](../combinatorics/binomial_prefix_sum.hpp) / [説明](combinatorics/binomial_prefix_sum.md)。

- **C**：RSK対応、Young Tableaux、Hook-Length Formula、P-recursive数列、高度な組合せ恒等式用の変換

補足：数論・組合せ論・母関数の高度な部分は、Nyaan、maspypy、hitonanodeが特に参考になります。

<a id="3-16"></a>

### 3.16 畳み込み・高速変換

主な配置先: `convolution/`。

- **S**：NTT、通常の畳み込み、FFT（実数・複素数）

**A（優先実装）**

- [x] 任意mod畳み込み（CRT/Garner） — **実装あり**。 [convolution/convolution_mod.hpp](../convolution/convolution_mod.hpp)。
- [x] Karatsuba — **実装あり**。 [convolution/karatsuba.hpp](../convolution/karatsuba.hpp)。 一般の環上の3再帰積、整数の中間超過検査。
- [x] FWHT — **実装あり**。 [set_function/fwht.hpp](../set_function/fwht.hpp)。 [3.16共通入口](../convolution/fwht.hpp)。 自然順、逆は各段の2除算。
- [x] AND/OR/XOR Convolution — **実装あり**。 [set_function/bitwise_convolution.hpp](../set_function/bitwise_convolution.hpp)。 [3.16共通入口](../convolution/bitwise_convolution.hpp)。 3.19と共有。
- [x] Subset/Superset Zeta・Möbius Transform — **実装あり**。 [set_function/all.hpp](../set_function/all.hpp)。 subset/superset両方向のzeta/mobiusを実装済み。
- [x] Subset Convolution — **実装あり**。 [set_function/subset_convolution.hpp](../set_function/subset_convolution.hpp)。

**B**

- [x] Relaxed/Online Convolution — **実装あり**。 [convolution/relaxed_convolution.hpp](../convolution/relaxed_convolution.hpp)。 [API・条件・検証](convolution/relaxed_convolution.md)。
- [x] GCD/LCM Convolution — **実装あり**。 [convolution/gcd_lcm_convolution.hpp](../convolution/gcd_lcm_convolution.hpp)。 [API・条件・検証](convolution/gcd_lcm_convolution.md)。
- [x] Min-Plus Convolution — **実装あり**。 [convolution/min_plus_convolution.hpp](../convolution/min_plus_convolution.hpp)。 [API・条件・検証](convolution/min_plus_convolution.md)。
- [x] 多次元畳み込み — **実装あり**。 [convolution/multidimensional_convolution.hpp](../convolution/multidimensional_convolution.hpp)。 [API・条件・検証](convolution/multidimensional_convolution.md)。
- [x] 巨大配列の畳み込み — **実装あり**。 [convolution/convolution_large.hpp](../convolution/convolution_large.hpp)。 [API・条件・検証](convolution/convolution_large.md)。
- [x] Middle Product — **実装あり**。 [convolution/middle_product.hpp](../convolution/middle_product.hpp)。 [API・条件・検証](convolution/middle_product.md)。
- [x] Chirp-Z Transform — **実装あり**。 [convolution/chirp_z.hpp](../convolution/chirp_z.hpp)。 [API・条件・検証](convolution/chirp_z.md)。

- **C**：Rader FFT、Bluestein FFT、高性能64bit NTT、AVX2 NTT、多変数巡回畳み込み、Multiplicative Convolution

補足：重要度が高く、ライブラリ化の効果も大きい分野です。

<a id="3-17"></a>

### 3.17 多項式・形式的冪級数（FPS）

主な配置先: `fps/`。

- **S**：多項式加減乗、微分、積分、係数切り詰め

**A（優先実装）**

- [x] FPS逆数 inv — **実装あり**。 [fps/inv.hpp](../fps/inv.hpp)。
- [x] log — **実装あり**。 [fps/log.hpp](../fps/log.hpp)。
- [x] exp — **実装あり**。 [fps/exp.hpp](../fps/exp.hpp)。
- [x] pow — **実装あり**。 [fps/pow.hpp](../fps/pow.hpp)。
- [x] sqrt — **実装あり**。 [fps/sqrt.hpp](../fps/sqrt.hpp)。
- [x] 多項式除算・剰余 — **実装あり**。 [fps/division.hpp](../fps/division.hpp)。
- [x] 多点評価 — **実装あり**。 [fps/multipoint_evaluation.hpp](../fps/multipoint_evaluation.hpp)。
- [x] 補間 — **実装あり**。 [fps/interpolation.hpp](../fps/interpolation.hpp)。
- [x] Taylor Shift — **実装あり**。 [fps/taylor_shift.hpp](../fps/taylor_shift.hpp)。
- [x] Bostan–Mori — **実装あり**。 [fps/bostan_mori.hpp](../fps/bostan_mori.hpp)。

**B**

- [x] FPS合成 — **実装あり**。 [fps/composition.hpp](../fps/composition.hpp)。 [API・条件・検証](fps/composition.md)。
- [x] 合成逆関数 — **実装あり**。 [fps/compositional_inverse.hpp](../fps/compositional_inverse.hpp)。 [API・条件・検証](fps/compositional_inverse.md)。
- [x] Sparse FPS — **実装あり**。 [fps/sparse_fps.hpp](../fps/sparse_fps.hpp)。 [API・条件・検証](fps/sparse_fps.md)。
- [x] Online FPS — **実装あり**。 [fps/online_fps.hpp](../fps/online_fps.hpp)。 [API・条件・検証](fps/online_fps.md)。
- [x] Polynomial GCD — **実装あり**。 [fps/polynomial_gcd.hpp](../fps/polynomial_gcd.hpp)。 [API・条件・検証](fps/polynomial_gcd.md)。
- [x] Polynomial Mod Pow — **実装あり**。 [fps/polynomial_mod_pow.hpp](../fps/polynomial_mod_pow.hpp)。 [API・条件・検証](fps/polynomial_mod_pow.md)。
- [x] Subproduct Tree — **実装あり**。 [fps/subproduct_tree.hpp](../fps/subproduct_tree.hpp)。 [API・条件・検証](fps/subproduct_tree.md)。
- [x] Partial Fraction Decomposition — **実装あり**。 [fps/partial_fraction.hpp](../fps/partial_fraction.hpp)。 [API・条件・検証](fps/partial_fraction.md)。

- **C**：Power Projection、多項式の根の高速計算、高速FPS合成、特殊なFPS変換、二変数FPS、多変数FPS、FPSの微分方程式、多項式行列の行列式・Prefix Product、Composed Sum/Product、高速中間積

<a id="3-18"></a>

### 3.18 線形漸化式・母関数

主な配置先: `linear_recurrence/`。

**A（優先実装）**

- [x] 行列累乗による漸化式 — **実装あり**。 [linear_recurrence/matrix_recurrence.hpp](../linear_recurrence/matrix_recurrence.hpp)。
- [x] Kitamasa — **実装あり**。 [linear_recurrence/kitamasa.hpp](../linear_recurrence/kitamasa.hpp)。
- [x] Berlekamp–Massey — **実装あり**。 [linear_recurrence/berlekamp_massey.hpp](../linear_recurrence/berlekamp_massey.hpp)。
- [x] Bostan–Mori — **実装あり**。 [linear_recurrence/bostan_mori.hpp](../linear_recurrence/bostan_mori.hpp)。

**B**

- [x] 線形漸化式の推定と第N項 — **実装あり**。 [linear_recurrence/infer_nth.hpp](../linear_recurrence/infer_nth.hpp)。 [API・条件・検証](linear_recurrence/infer_nth.md)。
- [x] P-recursive数列 — **実装あり**。 [linear_recurrence/p_recursive.hpp](../linear_recurrence/p_recursive.hpp)。 [API・条件・検証](linear_recurrence/p_recursive.md)。
- [x] 指数型母関数（EGF）の演算 — **実装あり**。 [linear_recurrence/egf.hpp](../linear_recurrence/egf.hpp)。 [API・条件・検証](linear_recurrence/egf.md)。
- [x] 母関数の係数抽出 — **実装あり**。 [linear_recurrence/coefficient_extraction.hpp](../linear_recurrence/coefficient_extraction.hpp)。 [API・条件・検証](linear_recurrence/coefficient_extraction.md)。
- [x] 有理型母関数 — **実装あり**。 [linear_recurrence/rational_generating_function.hpp](../linear_recurrence/rational_generating_function.hpp)。 [API・条件・検証](linear_recurrence/rational_generating_function.md)。
- [x] 等比数列の多項式重み付き和 — **実装あり**。 [linear_recurrence/polynomial_geometric_sum.hpp](../linear_recurrence/polynomial_geometric_sum.hpp)。 [API・条件・検証](linear_recurrence/polynomial_geometric_sum.md)。

- **C**：Holonomic Sequence、Reeds–Sloane、特殊な母関数の合成、線形漸化式の大量項計算

補足：最近扱っていたようなEGF・スターリング変換・オンライン畳み込み・一次分数変換を使う問題まで視野に入れるなら、ここは重点的に整備するとよい分野です。

参考：Nyaan FPS、maspypy Polynomial。

<a id="3-19"></a>

### 3.19 集合関数・Bitmask変換

主な配置先: `set_function/`。

- **S**：Submask/Supermask列挙、SOS DP、集合ゼータ・メビウス変換

**A（優先実装）**

- [x] XOR/AND/OR Convolution — **実装あり**。 [set_function/bitwise_convolution.hpp](../set_function/bitwise_convolution.hpp)。 3.16と共通実装。
- [x] Subset Convolution — **実装あり**。 [set_function/subset_convolution.hpp](../set_function/subset_convolution.hpp)。
- [x] Ranked Zeta Transform — **実装あり**。 [set_function/ranked_zeta_transform.hpp](../set_function/ranked_zeta_transform.hpp)。 rank別zetaと層ごとのmobiusを公開、Subset Convolutionでも共有。

**B**

- [x] Set Power Series（SPS）の exp/log/inv — **実装あり**。 [set_function/set_power_series.hpp](../set_function/set_power_series.hpp)。 [API・条件・検証](set_function/set_power_series.md)。
- [x] SPSの合成 — **実装あり**。 [set_function/sps_composition.hpp](../set_function/sps_composition.hpp)。 [API・条件・検証](set_function/sps_composition.md)。
- [x] 集合分割DP — **実装あり**。 [set_function/set_partition.hpp](../set_function/set_partition.hpp)。 [API・条件・検証](set_function/set_partition.md)。
- [x] 高速な部分集合畳み込み — **実装あり**。 [set_function/fast_subset_convolution.hpp](../set_function/fast_subset_convolution.hpp)。 [API・条件・検証](set_function/fast_subset_convolution.md)。

- **C**：SPS Power Projection、Online Set Function Convolution、特殊な集合関数の変換

補足：Library CheckerにはSubset ConvolutionやSet Power Seriesの検証問題があります。

<a id="3-20"></a>

### 3.20 線形代数

主な配置先: `linear_algebra/`。

- **S**：Matrix、行列積、行列累乗、連立一次方程式（Gauss–Jordan）、XOR基底

**A（優先実装）**

- [x] 行列式 — **実装あり**。 [graph_combinatorics/determinant_mod.hpp](../graph_combinatorics/determinant_mod.hpp)。 素数modの行列式は実装済み。汎用体版・整数版を追加するなら拡張として扱う。
- [x] 逆行列 — **実装あり**。 [linear_algebra/inverse.hpp](../linear_algebra/inverse.hpp)。 体型・実数・素数mod、特異ならnullopt。
- [x] 階数 — **実装あり**。 [linear_algebra/gauss_jordan.hpp](../linear_algebra/gauss_jordan.hpp)。 返り値rankを利用。
- [x] GF(2)上のBitset Gauss — **実装あり**。 [linear_algebra/gf2_gauss.hpp](../linear_algebra/gf2_gauss.hpp)。 bitsetの消去、rank・特解・零空間。
- [x] 線形空間の交差 — **実装あり**。 [linear_algebra/space_intersection.hpp](../linear_algebra/space_intersection.hpp)。 体型・素数mod・XorBasisの交差基底。
- [x] 特性多項式 — **実装あり**。 [linear_algebra/characteristic_polynomial.hpp](../linear_algebra/characteristic_polynomial.hpp)。 Hessenberg相似変形、昇冪順det(xI−A)、小標数可。

**B**

- [x] Sparse Matrix — **実装あり**。 [linear_algebra/sparse_matrix.hpp](../linear_algebra/sparse_matrix.hpp)。 [API・条件・検証](linear_algebra/sparse_matrix.md)。
- [x] Black-Box Linear Algebra — **実装あり**。 [linear_algebra/black_box.hpp](../linear_algebra/black_box.hpp)。 [API・条件・検証](linear_algebra/black_box.md)。
- [x] Wiedemann Algorithm — **実装あり**。 [linear_algebra/wiedemann.hpp](../linear_algebra/wiedemann.hpp)。 [API・条件・検証](linear_algebra/wiedemann.md)。
- [x] 最小多項式 — **実装あり**。 [linear_algebra/minimal_polynomial.hpp](../linear_algebra/minimal_polynomial.hpp)。 [API・条件・検証](linear_algebra/minimal_polynomial.md)。
- [x] Matrix Tree Theorem — **実装あり**。 [linear_algebra/matrix_tree.hpp](../linear_algebra/matrix_tree.hpp)。 [API・条件・検証](linear_algebra/matrix_tree.md)。
- [x] Low-Rank Update — **実装あり**。 [linear_algebra/low_rank_update.hpp](../linear_algebra/low_rank_update.hpp)。 [API・条件・検証](linear_algebra/low_rank_update.md)。

- **C**：Frobenius Normal Form、行列のHafnian、多項式行列の行列式、高速な特性多項式計算

<a id="3-21"></a>

### 3.21 文字列アルゴリズム

主な配置先: `string_algorithm/`。

- **S**：Z Algorithm、KMP / Prefix Function、Rolling Hash、Trie、Suffix Array、LCP Array、Manacher

**A（優先実装）**

- [x] Aho–Corasick — **実装あり**。 [string_algorithm/aho_corasick.hpp](../string_algorithm/aho_corasick.hpp)。
- [x] Suffix Automaton — **実装あり**。 [string_algorithm/suffix_automaton.hpp](../string_algorithm/suffix_automaton.hpp)。 任意byte、オンライン追加、出現数・初出・異なる部分文字列数。
- [x] Palindromic Tree（Eertree） — **実装あり**。 [string_algorithm/eertree.hpp](../string_algorithm/eertree.hpp)。
- [x] Lyndon Factorization — **実装あり**。 [string_algorithm/lyndon_factorization.hpp](../string_algorithm/lyndon_factorization.hpp)。 Duval、unsigned byte順、半開区間。
- [x] Longest Common Substring — **実装あり**。 [string_algorithm/longest_common_substring.hpp](../string_algorithm/longest_common_substring.hpp)。 SAMを共有し、両方の連続区間を復元。
- [x] 2D Rolling Hash — **実装あり**。 [string_algorithm/rolling_hash_2d.hpp](../string_algorithm/rolling_hash_2d.hpp)。 二つの素数法、矩形取得・比較、衝突あり。

**B**

- [x] Runs Enumeration — **実装あり**。 [string_algorithm/runs_enumeration.hpp](../string_algorithm/runs_enumeration.hpp)。 [API・条件・検証](string_algorithm/runs_enumeration.md)。
- [x] Suffix Tree — **実装あり**。 [string_algorithm/suffix_tree.hpp](../string_algorithm/suffix_tree.hpp)。 [API・条件・検証](string_algorithm/suffix_tree.md)。
- [x] Wildcard Pattern Matching — **実装あり**。 [string_algorithm/wildcard_pattern_matching.hpp](../string_algorithm/wildcard_pattern_matching.hpp)。 [API・条件・検証](string_algorithm/wildcard_pattern_matching.md)。
- [x] Dynamic Rolling Hash — **実装あり**。 [string_algorithm/dynamic_rolling_hash.hpp](../string_algorithm/dynamic_rolling_hash.hpp)。 [API・条件・検証](string_algorithm/dynamic_rolling_hash.md)。
- [x] 文字列の最小巡回シフト — **実装あり**。 [string_algorithm/minimum_rotation.hpp](../string_algorithm/minimum_rotation.hpp)。 [API・条件・検証](string_algorithm/minimum_rotation.md)。
- [x] 複数文字列の比較 — **実装あり**。 [string_algorithm/multi_string_compare.hpp](../string_algorithm/multi_string_compare.hpp)。 [API・条件・検証](string_algorithm/multi_string_compare.md)。
- [x] 回文分割DP — **実装あり**。 [string_algorithm/palindrome_partition.hpp](../string_algorithm/palindrome_partition.hpp)。 [API・条件・検証](string_algorithm/palindrome_partition.md)。

- **C**：Double-Ended Palindromic Tree、Online Z Algorithm、Generalized Suffix Automaton、高度な文字列周期問題

補足：文字列は基礎アルゴリズムの割に実装ミスの影響が大きいため、特にverifyを重視したい分野です。

<a id="3-22"></a>

### 3.22 計算幾何

主な配置先: `computational_geometry/`。

- **S**：Point/Vector、内積・外積、CCW、線分交差判定、直線交点、射影、距離、Convex Hull

**A（優先実装）**

- [x] 点の偏角ソート — **実装あり**。 [computational_geometry/angular_sort.hpp](../computational_geometry/angular_sort.hpp)。
- [x] Polygon Area — **実装あり**。 [computational_geometry/polygon.hpp](../computational_geometry/polygon.hpp)。
- [x] Point in Polygon — **実装あり**。 [computational_geometry/polygon.hpp](../computational_geometry/polygon.hpp)。
- [x] Convex Polygon Contains — **実装あり**。 [computational_geometry/polygon.hpp](../computational_geometry/polygon.hpp)。
- [x] Convex Polygon Cut — **実装あり**。 [computational_geometry/convex_cut.hpp](../computational_geometry/convex_cut.hpp)。
- [x] Rotating Calipers — **実装あり**。 [computational_geometry/rotating_calipers.hpp](../computational_geometry/rotating_calipers.hpp)。
- [x] 最近点対 — **実装あり**。 [computational_geometry/closest_pair.hpp](../computational_geometry/closest_pair.hpp)。

**B**

- [x] Minimum Enclosing Circle — **実装あり**。 [computational_geometry/minimum_enclosing_circle.hpp](../computational_geometry/minimum_enclosing_circle.hpp)。 [API・条件・検証](computational_geometry/minimum_enclosing_circle.md)。
- [x] Circle-Line/Circle-Circle Intersection — **実装あり**。 [computational_geometry/circle_intersection.hpp](../computational_geometry/circle_intersection.hpp)。 [API・条件・検証](computational_geometry/circle_intersection.md)。
- [x] Convex Layers — **実装あり**。 [computational_geometry/convex_layers.hpp](../computational_geometry/convex_layers.hpp)。 [API・条件・検証](computational_geometry/convex_layers.md)。
- [x] Minkowski Sum — **実装あり**。 [computational_geometry/minkowski_sum.hpp](../computational_geometry/minkowski_sum.hpp)。 [API・条件・検証](computational_geometry/minkowski_sum.md)。
- [x] Farthest Pair — **実装あり**。 [computational_geometry/farthest_pair.hpp](../computational_geometry/farthest_pair.hpp)。 [API・条件・検証](computational_geometry/farthest_pair.md)。
- [x] Manhattan MST — **実装あり**。 [computational_geometry/manhattan_mst.hpp](../computational_geometry/manhattan_mst.hpp)。 [API・条件・検証](computational_geometry/manhattan_mst.md)。
- [x] 格子点数（Pick） — **実装あり**。 [computational_geometry/pick.hpp](../computational_geometry/pick.hpp)。 [API・条件・検証](computational_geometry/pick.md)。
- [x] 半平面交差 — **実装あり**。 [computational_geometry/half_plane_intersection.hpp](../computational_geometry/half_plane_intersection.hpp)。 [API・条件・検証](computational_geometry/half_plane_intersection.md)。

- **C**：Delaunay Triangulation、Voronoi Diagram、Dynamic Convex Hull、Polygon Triangulation、Convex Polygon Union Area、三次元幾何

補足：2D幾何は、整数座標による厳密計算と浮動小数点計算のAPIを分けて実装する方が安全です。

特にmaspypyはDelaunay Triangulation、動的凸包、Minkowski Sumなどの上級幾何も収録しています。

<a id="3-23"></a>

### 3.23 組合せ最適化・マトロイド

主な配置先: `combinatorial_optimization/`。

**A（優先実装）**

- [x] Hungarian Algorithm — **実装あり**。 [combinatorial_optimization/hungarian.hpp](../combinatorial_optimization/hungarian.hpp)。
- [x] 最大マッチング — **実装あり**。 [combinatorial_optimization/general_matching.hpp](../combinatorial_optimization/general_matching.hpp)。
- [x] 最小費用流 — **実装あり**。 [combinatorial_optimization/min_cost_flow.hpp](../combinatorial_optimization/min_cost_flow.hpp)。
- [x] Assignment Problem — **実装あり**。 [combinatorial_optimization/assignment_problem.hpp](../combinatorial_optimization/assignment_problem.hpp)。

**B**

- [x] Matroid Intersection — **実装あり**。 [combinatorial_optimization/matroid_intersection.hpp](../combinatorial_optimization/matroid_intersection.hpp)。 [API・条件・検証](combinatorial_optimization/matroid_intersection.md)。
- [x] Graphic Matroid — **実装あり**。 [combinatorial_optimization/graphic_matroid.hpp](../combinatorial_optimization/graphic_matroid.hpp)。 [API・条件・検証](combinatorial_optimization/graphic_matroid.md)。
- [x] Partition Matroid — **実装あり**。 [combinatorial_optimization/partition_matroid.hpp](../combinatorial_optimization/partition_matroid.hpp)。 [API・条件・検証](combinatorial_optimization/partition_matroid.md)。
- [x] Maximum Weight Independent Set — **実装あり**。 [combinatorial_optimization/maximum_weight_independent_set.hpp](../combinatorial_optimization/maximum_weight_independent_set.hpp)。 [API・条件・検証](combinatorial_optimization/maximum_weight_independent_set.md)。
- [x] Linear Matroid Parity — **実装あり**。 [combinatorial_optimization/linear_matroid_parity.hpp](../combinatorial_optimization/linear_matroid_parity.hpp)。 [API・条件・検証](combinatorial_optimization/linear_matroid_parity.md)。

- **C**：Weighted Matroid Intersection、Base Polyhedron、Submodular Optimization、特殊な整数計画法

補足：組合せ最適化は一般的な競プロライブラリでは手薄になりがちですが、hitonanodeにはMatroid Intersection、Linear Matroid Parityなどが収録されています。

<a id="3-24"></a>

### 3.24 ゲーム理論

主な配置先: `game_theory/`。

- **S**：Nim、Misère Nim、Grundy数計算、DAGゲーム

**A（優先実装）**

- [x] Nim Product — **実装あり**。 [game_theory/nim_product.hpp](../game_theory/nim_product.hpp)。 uint64全域、mex小表と二次拡大。
- [x] 周期的Grundy数列の検出 — **実装あり**。 [game_theory/periodic_grundy.hpp](../game_theory/periodic_grundy.hpp)。 観測候補と、有限引き算ゲームの証明済み周期を分離。
- [x] ゲームの直和 — **実装あり**。 [game_theory/dag_game.hpp](../game_theory/dag_game.hpp)。 独立な駒のnim_sum・勝ち手復元。grundy.hppにも基本集約あり。

**B**

- [x] Nimberの体演算 — **実装あり**。 [game_theory/nimber.hpp](../game_theory/nimber.hpp)。 [API・条件・検証](game_theory/nimber.md)。
- [x] グラフゲーム — **実装あり**。 [game_theory/graph_game.hpp](../game_theory/graph_game.hpp)。 [API・条件・検証](game_theory/graph_game.md)。
- [x] Partizan Gameの基本操作 — **実装あり**。 [game_theory/partizan_game.hpp](../game_theory/partizan_game.hpp)。 [API・条件・検証](game_theory/partizan_game.md)。

- **C**：超限的なゲーム値・高度な組合せゲーム理論

<a id="3-25"></a>

### 3.25 探索・ヒューリスティック

主な配置先: `heuristic_search/`。

- **S**：乱数生成器、Timer、Simulated Annealing、Beam Search、局所探索

**A（優先実装）**

- [x] Zobrist Hash — **実装あり**。 [heuristic_search/zobrist_hash.hpp](../heuristic_search/zobrist_hash.hpp)。 固定seedの位置×値表、XORによる差分更新。
- [x] Tabu Search — **実装あり**。 [heuristic_search/tabu_search.hpp](../heuristic_search/tabu_search.hpp)。 訪問keyの禁止期限・aspiration・悪化手。
- [x] Hill Climbing — **実装あり**。 [heuristic_search/local_search.hpp](../heuristic_search/local_search.hpp)。 first/best improvementによるHill Climbing。
- [x] Beam Searchの重複除去 — **実装あり**。 [heuristic_search/beam_search_unique.hpp](../heuristic_search/beam_search_unique.hpp)。 同じ深さのkeyごとに最良代表を残し、幅を制限。既存版は保持。
- [x] スコア差分更新 — **実装あり**。 [heuristic_search/incremental_score.hpp](../heuristic_search/incremental_score.hpp)。 try_scored_moveと差分SA、in-place apply/undo。

**B**

- [x] Multi-Start SA — **実装あり**。 [heuristic_search/multi_start_sa.hpp](../heuristic_search/multi_start_sa.hpp)。 [API・条件・検証](heuristic_search/multi_start_sa.md)。
- [x] 温度制御 — **実装あり**。 [heuristic_search/temperature_control.hpp](../heuristic_search/temperature_control.hpp)。 [API・条件・検証](heuristic_search/temperature_control.md)。
- [x] パラメータチューニング — **実装あり**。 [heuristic_search/parameter_tuning.hpp](../heuristic_search/parameter_tuning.hpp)。 [API・条件・検証](heuristic_search/parameter_tuning.md)。
- [x] 多腕バンディット — **実装あり**。 [heuristic_search/multi_armed_bandit.hpp](../heuristic_search/multi_armed_bandit.hpp)。 [API・条件・検証](heuristic_search/multi_armed_bandit.md)。
- [x] 焼きなましの近傍管理 — **実装あり**。 [heuristic_search/neighborhood_manager.hpp](../heuristic_search/neighborhood_manager.hpp)。 [API・条件・検証](heuristic_search/neighborhood_manager.md)。

- **C**：多点並列探索、特殊な確率分布の高速サンプリング

補足：これはAtCoder Heuristic Contest（AHC）やマラソン系も対象にする場合です。
