/*
★ Name         : 構文木（二分木）を構築するライブラリ
 ■ Snipet      :
 
 ■ Arguments   :
 ■ Return      :
 ■ Complexity  :

 ■ Description : 与えられた文脈自由文法(CFG) G = (V, T, P, S)
                    - V: 変数（葉以外のノードに対応）  - P: 生成規則（辺に対応、変数を書き換える）
                    - T: 終端記号（葉に対応、V∩T=∅）   - S: 開始記号（根に対応、S∈V）
                によって生成される記号列wに対して、構文木（具象構文木と抽象構文木の中間的な情報を保持）を生成する。
                基本的には、LL(1)構文解析器 [入力記号列を左から走査、1文字先読みすることで適用する生成規則を判断、相互再帰によるDFSで実装]によってトップダウンに構文木を構築していく。
                ここで、各演算子の優先度や結合法則はあらかじめ定まっているとし、適用する生成規則が一つに定まらない文法は扱わないこととする。また、曖昧な文法に関しては、左から生成規則が適用される。
                文脈自由文法は、Chomsky標準形 [A -> BC / A -> a （ABCは変数、aは終端記号） の形の生成規則しかもたないCFG]に必ず変形できる（厳密には空語ε生成規則と単一生成規則を含まないことを要求）ため、構文木は必ず「二分木」の形となることに注意する
                構文木から数式を復元するには、通りがけ順（In-Order）、すなわち 左部分木 -> 根 -> 右部分木 の順に再帰的に出力すればよい。

 ■ Usage       : 構文木中の頂点の情報を表す構造体 Node は、以下のような構成になっている：
                    - int left_child:           左の子の頂点番号（空なら -1）
                    - int right_child:          右の子の頂点番号（空なら -1）
                    - int var:                  現在の頂点はどの変数か？(<expr>, <term> etc. に対応) -> 問題文に応じて適宜番号を振る
                    - int type:                 どの生成規則を適用して、現在の変数を（2つ以下の）子に変換したか？(<expr>::= ~ | ~ | ~ の ~に対応) -> 問題文に応じて適宜番号を振る
                    - pair<int, int> interval:   記号列のどの区間までを生成しているか？（半開区間 [l,r)）
                    - int len:                  この変数が記号列中で生成している区間の長さ -> intervalから計算可能なので、メンバ関数として取得
                    - long long val (optional): このノードに結びついている値（任意）
                    - string S (optional)     : このノードに結びついている文字列（任意）
                 実戦的には木DPなどをすることになると思うので、適宜持つべき情報をad-hocに追加していくと良さそう


 ■ Verify      :
 ■ References  : 授業スライド

 ■ TODO        : （優先度も考慮した）Chomsky標準形を与えれば、構文解析木を自動生成出来るようにしたかったが、流石に大変...
 
*/


#pragma once
#include <vector>
#include <string>
#include <utility>

struct Node {
    int left_child = -1;
    int right_child = -1;
    int var;
    int type;
    std::pair<int, int> interval;
    // long long val;
    // std::string S;

    int len() const { return interval.second - interval.first; }

};

using ParseTree = std::vector< Node >;



const int VAR_NUM = 4; // 変数の数
const std::vector<int> TERMINALS = {3}; // 終端記号に対応する番号
std::vector<std::string> production(VAR_NUM); // 生成規則
