/*
★ Name         : 構文解析　テンプレート（ICPC用？）
 ■ Snipet      :
 
 ■ Arguments   :
 ■ Return      :
 ■ Complexity  :

 ■ Description :
 

 ■ Usage       :方針まとめ
                1. テンプレートを書く
                2. expression, term, factor, numberのスケルトンを書く
                3. factorとnumberを埋める
                4. expressinとtermを埋める(最初に一つ下の部分式を一つだけパースした後に、あと はループで回しながら、足したり引いたりする。)
                5. 文字列の入力を行う


 ■ Verify      : https://judge.u-aizu.ac.jp/onlinejudge/review.jsp?rid=9339017#1 (基本的な四則演算)
 ■ References  : https://gist.github.com/draftcode/1357281

 ■ TODO        :
*/

using namespace std;

#pragma once
#include <iostream>
#include <string>
#include <cctype>
#include <exception>
#include <sstream>

/// ■ Main References  : https://gist.github.com/draftcode/1357281

/// 入力でcinとgetlineを混合するときは、「cinで読み込んだ後」はgetlineをする前に「cin.ignore()」を呼ぶべき -> 改行文字が残っていると、getlineが１行ズレる
//      -> getlineを使うのは、間に空白が入る時にもまとめて受け取れて便利なため（必須ではない）

/// State型は、stringのイテレータ（string::const_iterator）で、これを直接走査する（参照なので、関数内で勝手に位置が更新されて便利）
typedef string::const_iterator State;


/// ParseErrorが発生したときのエラー情報用
// TODO : 使いやすいよう設定し直す
class ParseError : public std::exception {
    public:
        // デフォルトコンストラクタ
        ParseError() : message("Parse error") {}

        // コンストラクタ：カスタムエラーメッセージを持つ
        ParseError(const std::string& msg, int line = -1, int column = -1)
            : message(msg), line(line), column(column) {
            buildFullMessage();
        }

        // コンストラクタ：行と列情報を含む
        ParseError(int line, int column)
            : message("Parse error"), line(line), column(column) {
            buildFullMessage();
        }

        // エラーメッセージを取得するメンバ関数
        const char* what() const noexcept override {
            return fullMessage.c_str();
        }

        // 行番号を取得するメンバ関数
        int getLine() const {
            return line;
        }

        // 列番号を取得するメンバ関数
        int getColumn() const {
            return column;
        }

    private:
        std::string message;
        std::string fullMessage;
        int line;
        int column;

        // エラーメッセージを構築するプライベート関数
        void buildFullMessage() {
            std::ostringstream oss;
            oss << message;
            if (line >= 0) {
                oss << " at line " << line;
            }
            if (column >= 0) {
                oss << ", column " << column;
            }
            fullMessage = oss.str();
        }
};


/// begin が 期待される文字 expectedを指していたらbeginを一つ進める（期待する文字でなければエラー発生）
// assertのようなイメージ、デバッグ用（使用例: 式の最後に=が来るのであれば、consume(begin, '=')として最後まで到達してることを確認できる）
void consume(State& begin, const char expected) {
    if (*begin == expected) {
        begin++;
    }
    else {
        cerr << "Expected char is '" << expected << "' but got '" << *begin << "'" << endl;
        cerr << "Rest string is '";
        while (*begin) {
            cerr << *begin++;
        }
        cerr << "'" << endl;
        throw ParseError();
    }
}




/// ■ 基本的な四則演算
///
/// <四則演算の式> ::= <乗算除算の式> (+ or -) <乗算除算の式> (+ or -) ...
/// <乗算除算の式> ::= <括弧か数> (* or /) <括弧か数> (* or /) ...
/// <括弧か数>     ::= '(' <四則演算の式> ')' or <数>
/// <数>           ::= ...
///
/// 基本的には下（深い）ほど、演算の優先順位が上

long long expression(State& begin);
long long term(State& begin);
long long factor(State& begin);
long long number(State& begin);

// TODO : MOD int に対応させる、浮動小数点数に対応させる

/// 四則演算（64bit整数用）
long long expression(State& begin) {
    long long ret = term(begin);

    while(true) {
        if(*begin == '+') {
            consume(begin, '+');
            ret += term(begin);
        }
        else if(*begin == '-') {
            consume(begin, '-');
            ret -= term(begin);
        }
        else break;
    }

    return ret;
}

/// 乗除
long long term(State& begin) {
    long long ret = factor(begin);

    while(true) {
        if(*begin == '*') {
            consume(begin, '*');
            ret *= factor(begin);
        }
        else if(*begin == '/') {
            consume(begin, '/');
            ret /= factor(begin);
        }
        else break;
    }

    return ret;
}

/// 括弧付きの式
long long factor(State& begin) {
    if(*begin == '(') {
        consume(begin, '(');
        long long ret = expression(begin);
        consume(begin, ')');
        return ret;
    }
    else {
        return number(begin);
    }
}


/// 数
long long number(State& begin) {
    long long ret = 0;

    while(isdigit(*begin)) {
        ret *= 10;
        ret += *begin - '0';
        begin++;
    }

    return ret;
}

