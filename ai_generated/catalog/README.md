# 機能検索カタログ

[GitHub用機能一覧](../FEATURES.md) / [機械可読JSON](features.json) / [生成API](../docs/development/generation.md) / [検証台帳](../docs/development_records/README.md)

`ai_generated/` から実行します。

```sh
python3 tools/cplib.py search DSU
python3 tools/cplib.py search 文字列 --category test_generation
python3 tools/cplib.py search graph --status implemented
python3 tools/cplib.py search random --verification partially_tested --json
python3 tools/cplib.py catalog --update
python3 tools/cplib.py catalog --check
```

英日名・部分一致・カテゴリ・公開API・キーワード・別名を検索します。DSU/Union-Find、BIT/Fenwick、
セグ木/Segment Tree、shortest-path/最短路などを同じキーワード群へ登録します。結果は実装・説明のパスを表示します。
`find` は既存のヘッダ検索です。検索で0件なら終了コード1、カタログ不整合なら2を返します。

## 登録単位・更新元

- C++は公開ヘッダ単位。all.hppはaggregateとして区別し、detail以下・例・検証プログラムは除外します。
- Pythonは `tools/` の公開トップレベル関数・クラス単位。`__all__` があれば優先します。クラスの公開メソッドはpublic_apiへ登録します。
- 既存のドキュメントfrontmatter、contestのheader_catalog、verifyのmanifestから名前・説明・API要約・制約・計算量を再利用します。
- C++を正規表現で解析して全関数を公開APIと推測しません。個別関数・メソッドの契約は各ドキュメントを参照します。public_apiが空でもヘッダ単位の機能は登録済みです。
- 新生成器は公開関数を定義し `__all__` に追加すると収集されます。日本語名・同義語の追加はfeature_catalog.pyのメタデータ表を更新してください。
- `ai_generated/` の旧data_structure・graph_theoryもexperimentalとして収集します。`depricated/` はdeprecated、`completed/` は現在.gitkeepのみです。

JSONの全パスは **リポジトリルートから** の相対パスです（`path_base=repository_root`）。
FEATURES.mdへのリンク生成時に `ai_generated/` からの相対パスへ変換し、旧版への `../depricated/` も検査します。
IDは所属と実装パス、Pythonはモジュールと公開シンボルを基準に安定化しています。移動・改名はID変更です。

## 実装と検証

statusは `implemented / experimental / deprecated / planned`、verificationは `tested / partially_tested / untested / unknown` を区別します。
自動収集だけでtestedにはしません。現在C++はunknownとして収集します。ソース中のコメント・既存説明の「検証済み」記載は実行証跡として採用しません。
従来の検証台帳は別に保持され、現在のソースとの一致、ローカル実行、提出ACの証跡を詳しく追跡します。

生成器のローカルテスト後に限り、次で証跡を保存できます。

```sh
python3 verify/run_generation.py --record-evidence
python3 tools/cplib.py catalog --check
```

[test_evidence.json](test_evidence.json) は実行が成功したコマンドとソース・テストのSHA256を持ちます。
全入力の網羅は主張せずpartially_testedとして登録します。対象ソースが変わると証跡は失効し、
再生成後の検証状態はunknownになります。失敗時は以前の証跡を削除してから停止します。
通常の `make verify-generation` はテストだけを実行し、証跡・カタログを書き換えません。

`--check` は再生成結果の一致、重複ID、登録漏れ、消えた実装・説明、READMEと索引の相対リンク、
frontmatterが指す公開ヘッダを検査します。任意Markdownの意味上の孤立や、C++の全メソッドの登録漏れを判定する機能ではありません。
カタログを手編集せず、実装・ドキュメント・manifest・収集メタデータを更新してください。
