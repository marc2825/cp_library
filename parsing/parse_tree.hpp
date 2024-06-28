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
    // int len;
    // long long val;
    // std::string S;

    int len() const { return interval.second - interval.first; }

};

using ParseTree = std::vector< Node >;
ParseTree PT; // テストケース間のclear()を忘れない



const int VAR_NUM = 4; // 変数の数
const std::vector<int> TERMINALS = {3}; // 終端記号に対応する番号
std::vector<std::string> production(VAR_NUM); // 生成規則




/// from drken
/// https://judge.u-aizu.ac.jp/onlinejudge/review.jsp?rid=8439869#1

//
// LL1 再帰降下パーサー
//   計算結果を出力するだけでなく、構文解析木を構築する
//
// verified:
//   AOJ 2613 Unordered Operators (for operator priority)
//     https://judge.u-aizu.ac.jp/onlinejudge/description.jsp?id=2613
//
//   AOJ 2401 恒等式 (for unary "not" operator)
//     https://judge.u-aizu.ac.jp/onlinejudge/description.jsp?id=2401
//
//   TTPC 2023 F - N^a (log N)^b (for unary "log" operator)
//     https://atcoder.jp/contests/ttpc2023/tasks/ttpc2023_f
//
//   AtCoder codeFlyer E - 数式とクエリ (for parse-tree)
//     https://atcoder.jp/contests/bitflyer2018-final-open/tasks/bitflyer2018_final_e
//


/*
 basic settings:
 　・vector<set<string>> OPERATORS
        - 登場する二項演算子と、その優先順位を設定する
 　・function<T(const string&, T, T)> OPERATION
        - 二項演算子に応じた二項演算の結果を出力する処理を設定する
 
 optional settings:
 　・function<T(const string&, int&)> GET_LEAF
        - 構文解析木において葉となる部分のうち、数値以外を処理する分を設定する (第二引数は文字列中の index)
 　・string UNARY_OPERATORS
        - パースする文字列に単項演算子 ("-" や "log" など) が含まれる場合、その関数を表す文字列
 　・function<T(T)> UNARY_OPERATION
        - パースする文字列が関数 OPERATOR_FUNC を含む場合、その関数の中身を設定
 */


#define COUT(x) cout << #x << " = " << (x) << " (L" << __LINE__ << ")" << endl



#include <bits/stdc++.h>
using namespace std;


// Parser (T must have constructor T(long long))
template<class T> struct Parser {
    /* basic settings */
    vector<vector<string>> OPERATORS;               // binary operator priority
    function<T(const string&, T, T)> OPERATION;     // binary operation

    /* optional settings */
    function<T(const string&, int&)> GET_LEAF;      // leaf parser
    vector<string> UNARY_OPERATORS;                 // unary operator (ex: "-", "log")
    function<T(const string&, T)> UNARY_OPERATION;  // unary operation
    const string OPERATOR_NUMBER = "num";
    
    /* results */
    string S;
    int root;                    // vals[root] is the answer
    vector<T> vals;              // value of each node
    vector<string> ops;          // operator of each node
    vector<int> left, right;     // the index of left-node, right-node
    vector<int> ids;             // the node-index of i-th value
    
    /* constructor */
    Parser() {}
    Parser(const vector<vector<string>> &operators,
           const function<T(const string&, T, T)> operation) {
        init(operators, operation);
    }
    Parser(const vector<vector<string>> &operators,
           const function<T(const string&, T, T)> operation,
           const function<T(const string&, int&)> get_leaf) {
        init(operators, operation, get_leaf);
    }
    Parser(const vector<vector<string>> &operators,
           const function<T(const string&, T, T)> operation,
           const function<T(const string&, int&)> get_leaf,
           const vector<string> &unary_operators,
           const function<T(const string &op, T)> unary_operation) {
        init(operators, operation, get_leaf, unary_operators, unary_operation);
    }
    void init(const vector<vector<string>> &operators,
              const function<T(const string&, T, T)> operation) {
        OPERATORS = operators, OPERATION = operation;
    }
    void init(const vector<vector<string>> &operators,
              const function<T(const string&, T, T)> operation,
              const function<T(const string&, int&)> get_leaf) {
        OPERATORS = operators, OPERATION = operation;
        GET_LEAF = get_leaf;
    }
    void init(const vector<vector<string>> &operators,
              const function<T(const string&, T, T)> operation,
              const function<T(const string&, int&)> get_leaf,
              const vector<string> &unary_operators,
              const function<T(const string &op, T)> unary_operation) {
        OPERATORS = operators, OPERATION = operation;
        GET_LEAF = get_leaf;
        UNARY_OPERATORS = unary_operators, UNARY_OPERATION = unary_operation;
    }
    void clear() {
        S = "";
        vals.clear(), ops.clear(), left.clear(), right.clear(), ids.clear();
    }
    
    /* node generator */
    // value
    int new_leaf(T val, int &id) {
        vals.push_back(val);
        ops.push_back(OPERATOR_NUMBER);
        left.push_back(-1);
        right.push_back(-1);
        ids.push_back(id++);
        return (int)vals.size() - 1;
    }
    // FUNC(lp)
    int new_node_with_left(T val, const string &op, int lp) {
        vals.push_back(val);
        ops.push_back(op);
        left.push_back(lp);
        right.push_back(-1);
        ids.push_back(-1);
        return (int)vals.size() - 1;
    }
    // (lp) op (rp)
    int new_node_with_left_right(const string &op, int lp, int rp) {
        vals.push_back(OPERATION(op, vals[lp], vals[rp]));
        ops.push_back(op);
        left.push_back(lp);
        right.push_back(rp);
        ids.push_back(-1);
        return (int)vals.size() - 1;
    }
    
    /* main parser */
    T parse(const string &str, bool default_parse_numeric = true) {
        clear();
        S = str;
        int p = 0, id = 0;
        root = parse(p, id, default_parse_numeric);
        return vals[root];
    }
    int parse(int &p, int &id, bool default_parse_numeric, int depth = 0) {
        assert(p >= 0 && p < (int)S.size());
        string op;
        if (depth < (int)OPERATORS.size()) {
            // recursive descent
            int lp = parse(p, id, default_parse_numeric, depth + 1);
            bool update = false;
            do {
                update = false;
                if (is_in_the_operators(p, op,
                                        OPERATORS[(int)OPERATORS.size() - depth - 1])) {
                    int rp = parse(p, id, default_parse_numeric, depth + 1);
                    lp = new_node_with_left_right(op, lp, rp);
                    update = true;
                }
            } while (p < (int)S.size() && update);
            return lp;
        }
        else if (S[p] == '(') {
            return get_bracket(p, id, default_parse_numeric);
        }
        else if (default_parse_numeric && (S[p] == '-' || isdigit(S[p]))) {
            return get_number(p, id);
        }
        else if (is_in_the_operators(p, op, UNARY_OPERATORS)) {
            return get_unary_operation(p, id, op, default_parse_numeric);
        }
        else {
            return new_leaf(GET_LEAF(S, p), id);
        }
    }
    bool is_the_operator(int &p, const string &op) {
        if (op != "" && S.substr(p, op.size()) == op) {
            p += op.size();
            return true;
        }
        return false;
    }
    bool is_in_the_operators(int &p, string &res_op, const vector<string> &ops) {
        for (const auto &op : ops) {
            if (is_the_operator(p, op)) {
                res_op = op;
                return true;
            }
        }
        return false;
    }
    int get_number(int &p, int &id) {
        long long val = 0, sign = 1;
        if (S[p] == '-') sign = -1, ++p;
        while (p < (int)S.size() && isdigit(S[p])) {
            val = val * 10 + (int)(S[p++] - '0');
        }
        return new_leaf(T(val * sign), id);
    }
    int get_bracket(int &p, int &id, bool default_parse_numeric) {
        ++p;  // skip "("
        int lp = parse(p, id, default_parse_numeric, 0);
        ++p;  // skip ")"
        return lp;
    }
    int get_unary_operation(int &p, int &id, const string &op,
                            bool default_parse_numeric) {
        int lp;
        if (S[p] == '(') lp = get_bracket(p, id, default_parse_numeric);
        else lp = parse(p, id, default_parse_numeric, (int)OPERATORS.size());
        return new_node_with_left(UNARY_OPERATION(op, vals[lp]), op, lp);
    }
    
    /* dump */
    void dump() {
        dump_rec(root);
    }
    void dump_rec(int v, string space = "") {
        cout << space << v << ": (" << ops[v] << ", " << vals[v]
             << ") -> left: " << left[v] << ", right: " << right[v] << endl;
        if (left[v] != -1) dump_rec(left[v], space + "  ");
        if (right[v] != -1) dump_rec(right[v], space + "  ");
    }
};



/*/////////////////////////////*/
// Examples
/*/////////////////////////////*/

/* AOJ 2613 */
// <p><var>+&minus;&times;</var>の3つの二項演算子と括弧を含む数式が与えられる。
// 3つの演算子の優先順位を任意に変更して、数式を最大化したときの計算結果を答えよ。
// </p>

// <p>ただし、以下の点に注意せよ。
// </p>

// <ul><li> 演算子は必ず左結合である。
// (同じ優先順位の演算子は必ず数式の左側から計算する。)
// </li><li> 異なる演算子が同じ優先順位であってもよい。
// </li><li> 一つの数式を計算している途中で優先順位を変更してはならない。
// </li></ul>


// <h2>Input</h2>

// <p>入力は以下の形式で与えられる。<br>
// 0〜9の数字と演算子'+','-','*'、括弧'(',')'で構成された数式<br>
// </p>

// <ul><li> 正確に記述すると入力は以下のBNFで示される形式になっている
// </li></ul>

// <blockquote>
// &lt;expr&gt; ::= ( &lt;expr&gt; ) | &lt;number&gt; | &lt;expr&gt; &lt;op> &lt;expr&gt; <br> &lt;op&gt; ::= + | - | *
// </blockquote>
// <p> &lt;number&gt;は非負整数を表す。</p>
void AOJ_2613() {
    string S;
    cin >> S;
    
    // 二項演算を設定
    auto operation = [&](const string &op, long long l, long long r) -> long long {
        if (op == "+") return l + r;
        else if (op == "-") return l - r;
        else return l * r;
    };

    // 演算子の優先順位をすべて試す
    long long res = -LONG_MAX;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            for (int k = 0; k < 3; ++k) {
                vector<vector<string>> operators(3);
                operators[i].push_back("+");
                operators[j].push_back("-");
                operators[k].push_back("*");
                Parser<long long> ps(operators, operation);
                res = max(res, ps.parse(S));
            }
        }
    }
    cout << res << endl;
}


/* AOJ 2401 */
// <p>
// 論理式は以下のいずれかの形式である．
// X, Yは論理式とし， 2 項演算子は必ず括弧で囲むものとする．
// </p>

// <ul>
// <li>定数: T, F</li>
// <li>変数: a, b, c, d, e, f, g, h, i, j, k</li>
// <li>論理否定: -X</li>
// <li>論理積: (X*Y)</li>
// <li>論理和: (X+Y)</li>
// <li>論理包含: (X-&gt;Y)</li>
// </ul>

// <p>
// 2 つの論理式を等号 &quot;=&quot; で結合した等式が与えられる．
// 恒等式とは，式に現れる変数がどのような値であっても成立する等式のことである．
// 与えられた等式が恒等式であるかを判定するプログラムを作りたい．
// </p>


// <!-- end ja only -->
// </div>
// <h3>Input</h3>
// <div>
// <!-- begin ja only -->
// <p>
// 入力は複数の行で構成され，各行は 1 つのデータセットである．
// データセットはT, F, a, b, c, d, e, f, g, h, i, j, k, (, ), =, -, +, *, &gt; から成る文字列であり， 空白など他の文字を含まない．
// 1 行の文字数は 1000 文字以下と仮定してよい．
// </p>

// <p>
// 1 つのデータセットは等式ひとつを含む．
// 等式の文法は次の BNF で与えられる．
// すべての等式はこの構文規則に従う．
// </p>

// <pre>
// &lt;equation&gt; ::= &lt;formula&gt; &quot;=&quot; &lt;formula&gt;
// &lt;formula&gt;  ::= &quot;T&quot; | &quot;F&quot; |
// &quot;a&quot; | &quot;b&quot; | &quot;c&quot; | &quot;d&quot; | &quot;e&quot; | &quot;f&quot; |
// &quot;g&quot; | &quot;h&quot; | &quot;i&quot; | &quot;j&quot; | &quot;k&quot; |
// &quot;-&quot; &lt;formula&gt; |
// &quot;(&quot; &lt;formula&gt; &quot;*&quot; &lt;formula&gt; &quot;)&quot; |
// &quot;(&quot; &lt;formula&gt; &quot;+&quot; &lt;formula&gt; &quot;)&quot; |
// &quot;(&quot; &lt;formula&gt; &quot;-&gt;&quot; &lt;formula&gt; &quot;)&quot;
// </pre>
void AOJ_2401() {
    // parser settings //
    vector<vector<string>> ops = {{"+", "*", "->"}};
    auto operation = [&](const string &op, bool l, bool r) -> bool {
        if (op == "+") return (l || r);
        else if (op == "*") return (l && r);
        else if (op == "->") return (!l || r);
        else return true;
    };
    auto get_leaf = [&](const string &S, int &p) -> bool {
        return (S[p++] == 'T' ? true : false);
    };
    vector<string> unary_ops = {"-"};
    auto unary_operation = [&](const string &op, bool l) -> bool {
        if (op == "-") return !l;
        else return l;
    };
    Parser<bool> ps1(ops, operation, get_leaf, unary_ops, unary_operation);
    Parser<bool> ps2(ops, operation, get_leaf, unary_ops, unary_operation);
    
    string S;
    while (cin >> S) {
        if (S == "#") break;
        for (int i = 0; i < S.size(); ++i) if (S[i] == '=') S[i] = ' ';
        
        bool res = true;
        for (int bit = 0; bit < (1<<11); ++bit) {
            string SS = S;
            for (int i = 0; i < SS.size(); ++i) {
                if (SS[i] >= 'a' && SS[i] <= 'k') {
                    int id = SS[i] - 'a';
                    if (bit & (1<<id)) SS[i] = 'T';
                    else SS[i] = 'F';
                }
            }
            istringstream si(SS);
            string s1, s2;
            si >> s1 >> s2;
            bool r1 = ps1.parse(s1, false), r2 = ps2.parse(s2, false);
            if (r1 != r2) {
                res = false;
            }
        }
        if (res) cout << "YES" << endl;
        else cout << "NO" << endl;
    }
}


/* TTPC 2023 F */
// <expr> ::= <term> | <expr> "+" <term>
// <term> ::= <factor> | <term> "*" <factor>
// <factor> ::= "N" | "N^" <number> | "log(" <expr> ")" | "log(" <expr> ")^" <number> | "(" <expr> ")"
// <number> ::= <non_zero_digit> | <non_zero_digit> <digit_string>
// <digit_string> ::= <digit> | <digit> <digit_string>
// <non_zero_digit> ::= "1" | "2" | "3" | "4" | "5" | "6" | "7" | "8" | "9"
// <digit> ::= "0" | <non_zero_digit>
// 極限</p>
// <p>\[ \lim_{N \to \infty} \frac{F(N)}{N^a(\log N)^b} \]</p>
// <p>が有限の値（ <var>0</var> を含む）に収束するような非負整数の組 <var>(a, b)</var> 全体の集合を <var>S</var> とします。 <var>S</var> の<strong>辞書順最小の組</strong>を出力してください。</p>
struct Node {
    long long n, l, val;
    bool eps;
    Node(long long n = 0, long long l = 0, long long e = 0, long long v = 0)
        : n(n), l(l), eps(e), val(v) {}
    friend ostream& operator << (ostream &s, Node node) {
        return s << "(" << node.n << ", " << node.l << ", " << node.eps
                << ", " << node.val << ")";
    }
};

void TTPC_2023_F() {
    // operators
    vector<vector<string>> operators = {{"^"}, {"*"}, {"+"}};
    
    // define binary operation
    auto operation = [&](const string &op, const Node &l, const Node &r) -> Node {
        if (op == "+") {
            vector<long long> vl({l.n, l.l, (long long)l.eps, l.val});
            vector<long long> vr({r.n, r.l, (long long)r.eps, r.val});
            return (vl > vr ? l : r);
        }
        else if (op == "*") return Node(l.n + r.n, l.l + r.l, l.eps | r.eps, 1);
        else if (op == "^") return Node(l.n * r.val, l.l * r.val, l.eps, 1);
        else return Node();
    };
    
    // define number process
    auto get_leaf = [&](const string &S, int &p) -> Node {
        if (S[p] == 'N') {
            ++p;
            return Node(1, 0, false, 1);
        } else {
            long long val = 0;
            do {
                val = val * 10 + (int)(S[p++] - '0');
            } while (S[p] >= '0' && S[p] <= '9');
            return Node(0, 0, false, val);
        }
    };
    
    // define function (log)
    vector<string> unary_operators = {"log"};
    auto unary_operation = [&](const string &op, const Node &node) -> Node {
        if (op == "log") {
            if (node.n >= 1) return Node(0, 1, false, 1);
            else return Node(0, 0, true, 0);
        } else return Node();
    };
    
    // 入力
    string S;
    cin >> S;
    Parser<Node> ps(operators, operation, get_leaf, unary_operators, unary_operation);
    auto res = ps.parse(S, false);
    //ps.dump();

    if (res.eps) ++res.l;
    cout << res.n << " " << res.l << endl;
}


/* codeFloyer E */
template<int MOD> struct Fp {
    // inner value
    long long val;
    
    // constructor
    constexpr Fp() : val(0) { }
    constexpr Fp(long long v) : val(v % MOD) {
        if (val < 0) val += MOD;
    }
    constexpr long long get() const { return val; }
    constexpr int get_mod() const { return MOD; }
    
    // arithmetic operators
    constexpr Fp operator + () const { return Fp(*this); }
    constexpr Fp operator - () const { return Fp(0) - Fp(*this); }
    constexpr Fp operator + (const Fp &r) const { return Fp(*this) += r; }
    constexpr Fp operator - (const Fp &r) const { return Fp(*this) -= r; }
    constexpr Fp operator * (const Fp &r) const { return Fp(*this) *= r; }
    constexpr Fp operator / (const Fp &r) const { return Fp(*this) /= r; }
    constexpr Fp& operator += (const Fp &r) {
        val += r.val;
        if (val >= MOD) val -= MOD;
        return *this;
    }
    constexpr Fp& operator -= (const Fp &r) {
        val -= r.val;
        if (val < 0) val += MOD;
        return *this;
    }
    constexpr Fp& operator *= (const Fp &r) {
        val = val * r.val % MOD;
        return *this;
    }
    constexpr Fp& operator /= (const Fp &r) {
        long long a = r.val, b = MOD, u = 1, v = 0;
        while (b) {
            long long t = a / b;
            a -= t * b, swap(a, b);
            u -= t * v, swap(u, v);
        }
        val = val * u % MOD;
        if (val < 0) val += MOD;
        return *this;
    }
    constexpr Fp pow(long long n) const {
        Fp res(1), mul(*this);
        while (n > 0) {
            if (n & 1) res *= mul;
            mul *= mul;
            n >>= 1;
        }
        return res;
    }
    constexpr Fp inv() const {
        Fp res(1), div(*this);
        return res / div;
    }

    // other operators
    constexpr bool operator == (const Fp &r) const {
        return this->val == r.val;
    }
    constexpr bool operator != (const Fp &r) const {
        return this->val != r.val;
    }
    constexpr Fp& operator ++ () {
        ++val;
        if (val >= MOD) val -= MOD;
        return *this;
    }
    constexpr Fp& operator -- () {
        if (val == 0) val += MOD;
        --val;
        return *this;
    }
    constexpr Fp operator ++ (int) const {
        Fp res = *this;
        ++*this;
        return res;
    }
    constexpr Fp operator -- (int) const {
        Fp res = *this;
        --*this;
        return res;
    }
    friend constexpr istream& operator >> (istream &is, Fp<MOD> &x) {
        is >> x.val;
        x.val %= MOD;
        if (x.val < 0) x.val += MOD;
        return is;
    }
    friend constexpr ostream& operator << (ostream &os, const Fp<MOD> &x) {
        return os << x.val;
    }
    friend constexpr Fp<MOD> pow(const Fp<MOD> &r, long long n) {
        return r.pow(n);
    }
    friend constexpr Fp<MOD> inv(const Fp<MOD> &r) {
        return r.inv();
    }
};
 
/* codeFlyer_E */
// <expr> ::= <term> | <expr> '+' <term> | <expr> '-' <term>
// <term> ::= <value> | <term> '*' <value>
// <value> ::= 'a' | '(' <expr> ')' 
// <h3>問題文</h3><p><code>(</code>, <code>)</code>, <code>+</code>, <code>-</code>, <code>*</code>, <code>a</code> のみからなる数式 <var>S</var> が与えられます(詳細は問題文の後半を参照してください)。<var>S</var> に含まれる <code>a</code> の個数を <var>N</var> とします。</p>
// <p><var>N</var> 個の整数 <var>a_1, ..., a_N</var> と <var>2</var> 個の整数 <var>(b_i, x_i)</var> からなる <var>Q</var> 個のクエリが与えられるので、これらのクエリを処理してください。</p>
// <p>クエリ: 与えられた数式中の左から <var>b_i</var> 番目の <code>a</code> を <var>x_i</var> に、左から <var>1, ..., b_i-1, b_i+1, ..., N</var> 番目の <code>a</code> をそれぞれ <var>a_1, ..., a_{b_i-1}, a_{b_i+1}, ..., a_N</var> に置き換えた数式の値と <var>10^9+7</var> を法として合同な <var>0</var> 以上 <var>10^9+6</var> 以下の整数を出力する。</p>
void codeFlyer_E() {
    const int MOD = 1000000007;
    using mint = Fp<MOD>;
    
    // 入力
    string S;
    int Q;
    cin >> S >> Q;
    int N = 0;
    for (int i = 0; i < S.size(); ++i) if (S[i] == 'a') ++N;
    vector<long long> given(N);
    for (int i = 0; i < N; ++i) cin >> given[i];
    
    // set Parser
    vector<vector<string>> operators = {{"*"}, {"+", "-"}};
    auto operation = [&](const string &op, mint a, mint b) -> mint {
        if (op == "+") return a + b;
        else if (op == "-") return a - b;
        else if (op == "*") return a * b;
        else return 0;
    };
    int id = 0;
    auto get_leaf = [&](const string &S, int &p) -> mint {
        if (S[p] == 'a') {
            ++p;
            return given[id++];
        } else return 0;
    };
    Parser<mint> ps(operators, operation, get_leaf);
    mint base = ps.parse(S);
    
    // DP
    vector<mint> dp(N);
    auto rec = [&](auto self, int v, mint w) -> void {
        if (ps.ops[v] == ps.OPERATOR_NUMBER) dp[ps.ids[v]] = w;
        else if (ps.ops[v] == "+") {
            self(self, ps.left[v], w);
            self(self, ps.right[v], w);
        }
        else if (ps.ops[v] == "-") {
            self(self, ps.left[v], w);
            self(self, ps.right[v], -w);
        }
        else if (ps.ops[v] == "*") {
            self(self, ps.left[v], w * ps.vals[ps.right[v]]);
            self(self, ps.right[v], w * ps.vals[ps.left[v]]);
        }
    };
    rec(rec, ps.root, mint(1));
    
    // 出力
    for (int q = 0; q < Q; ++q) {
        long long id, x;
        cin >> id >> x;
        --id;
        mint res = base + dp[id] * (x - given[id]);
        cout << res << endl;
    }
}


int main() {
    AOJ_2613();
    //AOJ_2401();
    //TTPC_2023_F();
    //codeFlyer_E();
}