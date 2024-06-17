/*
★ Name        ：2D計算幾何ライブラリ
 ■ Snipet      :
 
 ■ Arguments   :
 ■ Return      :
 ■ Complexity  :

 ■ Description :
 

 ■ Usage       :


 ■ Verify      : https://judge.u-aizu.ac.jp/onlinejudge/review.jsp?rid=9316790#1 (projection)
                 https://judge.u-aizu.ac.jp/onlinejudge/review.jsp?rid=9316795#1 (reflection)
                 https://judge.u-aizu.ac.jp/onlinejudge/review.jsp?rid=9316820#1 (ccw)
                 https://judge.u-aizu.ac.jp/onlinejudge/review.jsp?rid=9316838#1 (parallel / orthogonal)
                 https://judge.u-aizu.ac.jp/onlinejudge/review.jsp?rid=9316843#1 (intersectPPPP)
                 https://atcoder.jp/contests/abc016/submissions/54497180 (intersectPPPP)
                 https://atcoder.jp/contests/abc266/submissions/54497293 (cross)
                 https://judge.u-aizu.ac.jp/onlinejudge/review.jsp?rid=9316850#1 (intersectSS)
                 https://judge.u-aizu.ac.jp/onlinejudge/review.jsp?rid=9337053#1 (crosspointSS)
                 https://judge.u-aizu.ac.jp/onlinejudge/review.jsp?rid=9316859#1 (distanceSS)
                 https://judge.u-aizu.ac.jp/onlinejudge/review.jsp?rid=9316864#1 (area [polygon])
                 https://judge.u-aizu.ac.jp/onlinejudge/review.jsp?rid=9317265#1 (contain [polygon])
                 https://judge.u-aizu.ac.jp/onlinejudge/review.jsp?rid=9317298#1 (convex hull)
                 https://judge.yosupo.jp/submission/214819 (convex hull)
                 https://atcoder.jp/contests/typical90/submissions/54527906 (convex hull, area)
                 https://judge.u-aizu.ac.jp/onlinejudge/review.jsp?rid=9317349#1 (intersectCC)
                 https://judge.u-aizu.ac.jp/onlinejudge/review.jsp?rid=9317390#1 (crosspointCL)
                 https://judge.u-aizu.ac.jp/onlinejudge/review.jsp?rid=9317407#1 (crosspointCC)
                 https://judge.u-aizu.ac.jp/onlinejudge/review.jsp?rid=9318254#1 (is_convex)
                 https://judge.yosupo.jp/submission/214642 (arg_sort)
                 https://atcoder.jp/contests/arc004/submissions/54511545 (furthest_pair)
                 https://atcoder.jp/contests/abc234/submissions/54511552 (furthest_pair)
                 https://atcoder.jp/contests/abc022/submissions/54534299 (furthest_pair)
                 https://judge.yosupo.jp/submission/215106 (furthest_pair long double)
                 https://judge.yosupo.jp/submission/215105 (furthest_pair long long)
                 https://judge.yosupo.jp/submission/215108 (closest_pair, long double)
                 https://judge.yosupo.jp/submission/215109 (closest_pair, long long)
                 https://atcoder.jp/contests/abc022/submissions/54534306 (closest_pair)
                 https://judge.u-aizu.ac.jp/onlinejudge/review.jsp?rid=9324068#1 (convex diameter)
                 https://atcoder.jp/contests/abc174/submissions/54511119 (contain [circle, count_mode])
                 https://atcoder.jp/contests/abc314/submissions/54511840 (unit, rot90, crosspointSS, crosspointCC, crosspointCS, distanceSP)
                 https://atcoder.jp/contests/abc314/submissions/54511939 (disttanceSP)
                 https://judge.u-aizu.ac.jp/onlinejudge/review.jsp?rid=9327543#1 (incircle)
                 https://judge.u-aizu.ac.jp/onlinejudge/review.jsp?rid=9327549#1 (circumcircle)
                 https://atcoder.jp/contests/abc296/submissions/54531599 (LowerHull, UpperHull, ccw, x_sort)
                 https://atcoder.jp/contests/agc021/submissions/54532720 (bisector, convex hull, get_angle)
                 https://atcoder.jp/contests/abc151/submissions/54622395 (MinBound_Circle)
                 https://judge.u-aizu.ac.jp/onlinejudge/review.jsp?rid=9332877#1 (tangent_CP)
                 https://judge.u-aizu.ac.jp/onlinejudge/review.jsp?rid=9333656#1 (tangent_CC)
                 https://judge.u-aizu.ac.jp/onlinejudge/review.jsp?rid=9333670#1 (convex cut)
                 https://judge.u-aizu.ac.jp/onlinejudge/review.jsp?rid=9333981#1 (intersection_area_CC)
                 * https://judge.u-aizu.ac.jp/onlinejudge/review.jsp?rid=9335893#1 (intersection_area_CP, *need improvement)
                 https://atcoder.jp/contests/past202112-open/submissions/54674977 (intersection_area_PP, crosspointSS, arg_sort_poly)






 ■ References  : https://ei1333.github.io/luzhiled/snippets/geometry/template.html
                 https://nyaannyaan.github.io/library/geometry/geometry-base.hpp
                 https://tjkendev.github.io/procon-library/
                 螺旋本（プログラミングコンテスト攻略のためのアルゴリズムとデータ構造）16章

 ■ TODO        : 完成させる
                 containの命名規則
                 Triangle Classの実装？
                 整数で完結出来るように、円のメンバ変数に半径の2乗（ノルム）も持たせる？ -> 諸々の関数での調整
                 有理数型への対応（整数幾何ライブラリの完全整備）
                 3D幾何ライブラリとの互換性チェック
                 verify list(https://judge.u-aizu.ac.jp/onlinejudge/finder.jsp?course=CGL
                            https://judge.yosupo.jp/
                            https://atcoder.jp/contests/abc151/tasks/abc151_f
                            https://atcoder.jp/contests/abc207/tasks/abc207_d
                            https://atcoder.jp/contests/abc225/tasks/abc225_e
                            https://atcoder.jp/contests/abc033/tasks/abc033_d
                            https://atcoder.jp/contests/abc314/tasks/abc314_h
                            https://atcoder.jp/contests/abc157/tasks/abc157_f
                            https://atcoder.jp/contests/abc181/tasks/abc181_f
                            https://atcoder.jp/contests/abc251/tasks/abc251_g
                            https://atcoder.jp/contests/abc286/tasks/abc286_h
                            https://atcoder.jp/contests/abc202/tasks/abc202_f
                            http://judge.u-aizu.ac.jp/onlinejudge/description.jsp?id=1033
                            https://atcoder.jp/contests/past202112-open/tasks/past202112_n
                            https://atcoder.jp/contests/geocon2013)
*/


#pragma once
#include <cmath>
#include <vector>
#include <limits>
#include <algorithm>
#include <cassert>
#include <iomanip>
#include <iostream>
#include <random>
#include <chrono>


/// 2D計算幾何ライブラリ
// 謎のバグが発生したら、比較部分での誤差（±EPSを設定）や整数幾何で出来ないか？を考えると良いかも
// AOJの文字数制限に引っかかる場合は、仕方ないので適当に不要な関数を削る
namespace Geometry {
    constexpr long double EPS = (1e-10);
    constexpr long double pi = 3.141592653589793238462643383279L;
    std::mt19937_64 mt(std::chrono::steady_clock::now().time_since_epoch().count());

    template <class T, class U> bool equals(T a, U b) { return fabsl(a - b) < EPS; }
    template <class T, class U> bool notequals(T a, U b) { return !equals(a, b); }
    template <class T> int sign(T a) { return equals(a, 0) ? 0 : a > 0 ? 1 : -1; }


    /// 2次元平面上の点(x,y)
    template <class T>
    class Point2D { // D は dimension の D (Double ではない) -> OpenGL等の著名ライブラリとの命名規則統一すべき？
        public:
            T x, y; // privateにしても良い、要検討
            bool valid; // 有効な点か？
            int idx;

            Point2D() : x(0), y(0), valid(false) {}
            Point2D(T x, T y): x(x), y(y), valid(true) {}
            template <class U> Point2D(const Point2D<U>& P_) : x((T)P_.x), y((T)P_.y), valid(P_.valid), idx(P_.idx) {}
            template <class U1, class U2> Point2D(const std::pair<U1, U2>& p) : x(p.first), y(p.second), valid(true) {}
    
            
            Point2D operator + (const Point2D& p) const {return Point2D(x + p.x, y + p.y); }
            Point2D operator - (const Point2D& p) const {return Point2D(x - p.x, y - p.y); }
            Point2D operator * (T a) const {return Point2D(a * x, a * y); }
            Point2D operator / (T a) const {return Point2D(x / a, y / a); }
            Point2D& operator+=(const Point2D& p) { return (*this) = (*this) + p; }
            Point2D& operator-=(const Point2D& p) { return (*this) = (*this) - p; }
            Point2D& operator*=(T a) { return (*this) = (*this) * a; }
            Point2D& operator/=(T a) { return (*this) = (*this) / a; }
            bool operator<(const Point2D& p) const { return notequals(x, p.x) ? x < p.x : y < p.y; }
            bool operator==(const Point2D& p) const { return equals(x, p.x) && equals(y, p.y); }
            bool operator!=(const Point2D& p) const { return !((*this) == p); }
    

            T real() const { return x; }
            T imag() const { return y; }
            T xpos() const { return x; }
            T ypos() const { return y; }
            T dot(const Point2D& p) const { return x * p.x + y * p.y; }
            T cross(const Point2D& p) const { return x * p.y - y * p.x; }
            T norm() const { return x * x + y * y; }
            long double abs() const { return std::sqrt(norm()); }
            long double arg() const { return std::atan2(y, x); }

            // Center中心、反時計回りにrad度回転
            void rotate(const Point2D& Center, const T rad) {
                x -= Center.x, y -= Center.y;
                T x_ = x * std::cos(rad) - y * std::sin(rad);
                T y_ = x * std::sin(rad) + y * std::cos(rad);
                x = x_ + Center.x, y = y_ + Center.y;
            }

            // 原点中心、反時計回りにrad度回転
            void rotate(const T rad) { 
                rotate(Point2D(0, 0), rad);
            }

            // 原点中心、反時計回りに90度回転
            void rot90() {
                T x_ = -y;
                y = x, x = x_;
            }

            // 単位ベクトルになるように変換
            void unit() {
                T len = this->abs();
                x /= len, y /= len;
            }

            // 偏角ソート用
            int argpos() const {
                if (equals(y, 0) && 0 <= x) return 0;
                if (y < 0) return -1;
                return 1;
            }

            void set_index(int i) { idx = i; }
            
            friend std::istream& operator>>(std::istream& is, Point2D& p) {
                T a, b;
                is >> a >> b;
                p = Point2D(a, b);
                return is;
            }
            friend std::ostream& operator<<(std::ostream& os, const Point2D& p) {
                return os << std::fixed << std::setprecision(15) << p.x << " " << p.y;
            }


        private :
            // constexpr long double EPS = (1e-10);
            // constexpr long double pi = 3.141592653589793238462643383279L;
            
            // bool equals(T a, T b) { return fabsl(a - b) < EPS; }
            // bool notequals(T a, T b) { return !equlas(a, b); }
            // int sign(T a) { return equals(a, 0) ? 0 : a > 0 ? 1 : -1; }
    };
    using Point = Point2D<long double>;
    using Points = std::vector<Point>;
    using PoINT = Point2D<long long>;
    using PoINTs = std::vector<PoINT>;
    template <class T> 
    using Polygon = std::vector<Point2D<T> >; // 多角形

    template <class T>
    using Vector2D = Point2D<T>; // 向き(x,y)と大きさ(abs(x,y))を持つ
    using Vector = Vector2D<long double>;


    template <class T> T real(const Point2D<T>& p) { return p.x; } // 実部
    template <class T> T imag(const Point2D<T>& p) { return p.y; } // 虚部
    template <class T> T dot(const Point2D<T>& l, const Point2D<T>& r) { return l.x * r.x + l.y * r.y; } // 内積
    template <class T> T cross(const Point2D<T>& l, const Point2D<T>& r) { return l.x * r.y - l.y * r.x; } // 外積
    template <class T> T norm(const Point2D<T>& p) { return p.x * p.x + p.y * p.y; } // 2-ノルム（ユークリッドノルム）の2乗
    template <class T> long double abs(const Point2D<T>& p) { return std::sqrt(norm(p)); } // 大きさ（絶対値）
    template <class T> long double arg(const Point2D<T>& p) { return std::atan2(p.y, p.x); } // 偏角
    Point polar_to_xy(const long double r, const long double theta) { return Point(r * std::cos(theta), r * std::sin(theta)); } // 極座標(r, theta) -> xy座標(x, y)

    template <class T>
    long double arg(const Point2D<T>& P1, const Point2D<T>& P2) {
        return std::atan2(cross(P1, P2), dot(P1, P2));
    }

    // 偏角ソート(-pi ~ pi)
    template<class T>
    void arg_sort(std::vector<Point2D<T> >& Ps) {
        std::sort(Ps.begin(), Ps.end(), [](const Point2D<T>& P1, const Point2D<T>& P2) {
            if(P1.argpos() != P2.argpos()) return P1.argpos() < P2.argpos();
            return cross(P1, P2) > 0;
        });
    }

    // x座標でソート
    template<class T>
    void x_sort(std::vector<Point2D<T> >& Ps) {
        std::sort(Ps.begin(), Ps.end(), [](const Point2D<T>& a, const Point2D<T>& b){ return equals(a.x, b.x) ? a.y < b.y : a.x < b.x; });
    }

    // y座標でソート
    template<class T>
    void y_sort(std::vector<Point2D<T> >& Ps) {
        std::sort(Ps.begin(), Ps.end(), [](const Point2D<T>& a, const Point2D<T>& b){ return equals(a.y, b.y) ? a.x < b.x : a.y < b.y; });
    }


    /// Pを、Center中心、反時計回りにrad度回転
    template <class T>
    Point2D<T> rotate(Point2D<T> P, const Point2D<T>& Center, const T rad) {
        P -= Center;
        T x_ = P.x * std::cos(rad) - P.y * std::sin(rad);
        T y_ = P.x * std::sin(rad) + P.y * std::cos(rad);
        return Point2D<T>(x_ + Center.x, y_ + Center.y);
    }

    /// 点Pを、原点中心、反時計回りにrad度回転
    template <class T>
    Point2D<T> rotate(const Point2D<T>& P, const T rad) { 
        return rotate(P, Point2D<T>(0, 0), rad);
    }

    // 原点中心、反時計回りに90度回転
    template <class T>
    Point2D<T> rot90(const Point2D<T>& P) {
        return Point2D<T>(-P.y, P.x);
    }

    /// V1の単位ベクトルを取得
    template <class T>
    Vector2D<T> unit(const Vector2D<T>& V1) {
        return V1 / V1.abs();
    }
    template <class T>
    Vector2D<T> unit(const Point2D<T>& P1, const Point2D<T>& P2) {
        Vector2D<T> V = P2 - P1;
        return unit(V);
    }

    /// 二点間の中点を取得
    template <class T>
    Point2D<T> mid(const Point2D<T>& P1, const Point2D<T>& P2) {
        return Point2D<T>((P1.x + P2.x) / 2, (P1.y + P2.y) / 2);
    }

    /// ベクトルAとベクトルBのなす角（B->Aの回転角、[-pi,pi]）
    template <class T>
    long double get_angle(const Vector2D<T>& V1, const Vector2D<T>& V2) {
        long double ret = arg(V1) - arg(V2);
        while(ret > pi + EPS) ret -= 2 * pi;
        while(ret < -pi - EPS) ret += 2 * pi;
        return ret;
    }

    /// 2点 P1, P2 を通る二次元平面上の直線 Ax + By + C = 0
    template <class T>
    class Line2D {
        public : 
            Point2D<T> P1, P2;
            T A, B, C;
            int idx;

            Line2D() = default;

            // P1, P2 -> Ax + By + C = 0 を計算
            Line2D(Point2D<T> P1, Point2D<T> P2) : P1(P1), P2(P2) // check
            {
                A = P2.y - P1.y;
                B = P1.x - P2.x;
                C = P1.y * (P2.x - P1.x) - P1.x * (P2.y - P1.y);
            }
            Line2D(T x1, T y1, T x2, T y2) : P1(Point2D<T>(x1, y1)), P2(Point2D<T>(x2, y2)) // check
            {
                A = P2.y - P1.y;
                B = P1.x - P2.x;
                C = P1.y * (P2.x - P1.x) - P1.x * (P2.y - P1.y);
            }

            // Ax + By + C = 0 -> P1, P2 を計算
            Line2D(T A, T B, T C) : A(A), B(B), C(C) // check
            {
                if(equals(A, 0)) P1 = Point2D(0, - C / B), P2 = Point2D(1, - C / B);
                else if(equals(B, 0)) P1 = Point2D(- C / A, 0), P2 = Point2D(- C / A, 1);
                else P1 = Point2D(0, - C / B), P2 = Point2D(- C / A , 0);
            }

            template <class U>
            Line2D(const Line2D<U>& L_) : Line2D<T>(Point2D<T>(L_.P1), Point2D<T>(L_.P2)) {}

            friend std::ostream &operator<<(std::ostream &os, Line2D &L) {
                return os << "(" << L.P1 << "), (" << L.P2 << ") / " << L.A << "x + " << L.B << "y + "<< L.C;
            }


            // 直線の傾きを取得
            T Grad() {
                if(equals(B, 0)) return std::numeric_limits< T >::max();
                return - A / B;
            }

            // 中点を取得
            Point2D<T> mid() {
                return Point2D<T>((P1.x + P2.x) / 2, (P1.y + P2.y) / 2);
            }

            // 垂直二等分線を取得
            Line2D<T> bisector() {
                Point2D<T> M = this->mid();
                return Line2D<T>(M, M + rot90(P2-P1));
            }

            void set_index(int i) { idx = i; }
        

        private:
            // constexpr long double EPS = (1e-10);
            // constexpr long double pi = 3.141592653589793238462643383279L;
            
            // bool equals(T a, T b) { return fabsl(a - b) < EPS; }
            // bool notequals(T a, T b) { return !equlas(a, b); }
            // int sign(T a) { return equals(a, 0) ? 0 : a > 0 ? 1 : -1; }

    };
    using Line = Line2D<long double>;
    using Lines = std::vector<Line>;

    template <class T>
    using Segment2D = Line2D<T>; // P1, P2を端点とする線分
    using Segment = Segment2D<long double>;
    using Segments = std::vector<Segment>;

    /// 直線Aと直線Bのなす角（is_abs : 絶対値取る？, under90 : 0~pi/2に正規化？）
    template <class T>
    long double get_angle(const Line2D<T>& L1, const Line2D<T>& L2, const bool is_abs = false, const bool under90 = false) {
        long double ret = get_angle(L1.P2 - L1.P1, L2.P2 - L2.P1);
        if(is_abs) ret = std::abs(ret);
        if(under90 && ret > pi/2 + EPS) ret = pi - ret;
        if(under90 && ret < -pi/2 - EPS) ret = -pi - ret;
        return ret;
    }

    /// 二点間の垂直二等分線を取得
    template <class T>
    Line2D<T> bisector(const Point2D<T>& P1, const Point2D<T>& P2) {
        Point2D<T> M = mid(P1, P2);
        return Line2D<T>(M, M + rot90(P2-P1));
    }

    /// 中心点 c と、半径 r を持つ二次元平面上の円
    // TODO: 整数用に、半径を2乗で管理？2乗用の変数を追加しても良い 
    template <class T>
    class Circle {
        public:
            Point2D<T> center;
            T radius;

            Circle() = default;

            Circle(T x, T y, T radius) : center(Point2D<T>(x, y)), radius(radius) {}

            Circle(Point2D<T> center, T radius) : center(center), radius(radius) {}

            template<class U>
            Circle(Circle<U> C_) : center(Point2D<T>(C_.center)), radius((T)C_.radius) {}

            bool operator==(const Circle& C) const { return center == C.center && equals(radius, C.radius); }
            bool operator!=(const Circle& C) const { return !((*this) == C); }

    
    };
    template <class T>
    using Circles = std::vector<Circle<T> >;
    using Circld = Circle<long double>;
    using Circlds = std::vector<Circld >;

    static const int COUNTER_CLOCKWISE = 1; // 反時計回り
    static const int CLOCKWISE = -1; // 時計回り
    static const int ONLINE_BACK = 2; // P2 -> P0 -> P1 の順で一直線 (P0 -> P1 とは逆方向にP2が存在)
    static const int ONLINE_FRONT = -2; // P0 -> P1 -> P2 の順で一直線 (P0 -> P1 と同方向にP2が存在)
    static const int ON_SEGMENT = 0; // P0 -> P2 -> P1 の順で一直線 (P0 -> P1 内にP2が存在)

    /**
     * @brief p0, p1, p2 がこの順で反時計回り（Counter Clock Wise）か？の判定
     * 
     * @return 1 : COUNTER_CLOCKWISE（反時計回り）
     *         -1 : CLOCKWISE（時計回り）
     *         2 : ONLINE_BACK（P2 -> P0 -> P1 の順で一直線 (P0 -> P1 とは逆方向にP2が存在)）
     *         -2 : ONLINE_FRONT（P0 -> P1 -> P2 の順で一直線 (P0 -> P1 と同方向にP2が存在)）
     *         0 : ON_SEGMENT（P0 -> P2 -> P1 の順で一直線 (P0 -> P1 内にP2が存在)）
     */
    template <class T>
    int ccw(const Point2D<T>& P0, const Point2D<T>& P1, const Point2D<T>& P2) {
        Vector2D<T> V1 = P1 - P0;
        Vector2D<T> V2 = P2 - P0;

        if(cross(V1, V2) > EPS) return COUNTER_CLOCKWISE; // 反時計回り
        else if(cross(V1, V2) < -EPS) return CLOCKWISE; // 時計回り
        else if(dot(V1, V2) < -EPS) return ONLINE_BACK; // P2 -> P0 -> P1 の順で一直線 (P0 -> P1 とは逆方向にP2が存在)
        else if(V1.norm() < V2.norm()) return ONLINE_FRONT; // P0 -> P1 -> P2 の順で一直線 (P0 -> P1 と同方向にP2が存在)
        else return ON_SEGMENT; // P0 -> P2 -> P1 の順で一直線 (P0 -> P1 内にP2が存在)
    }


    /// 直交判定
    template<class T>
    bool isOrthogonal(const Vector2D<T>& a, const Vector2D<T>& b) {
        return equals(dot(a, b), 0.0);
    }
    template<class T>
    bool isOrthogonal(const Point2D<T>& a1, const Point2D<T>& a2, const Point2D<T>& b1, const Point2D<T>& b2) {
        return isOrthogonal(a1 - a2, b1 - b2);
    }
    template<class T>
    bool isOrthogonal(const Line2D<T>& l1, const Line2D<T>& l2) {
        return equals(dot(l1.P2 - l1.P1, l2.P2 - l2.P1), 0.0);
    }


    /// 平行判定
    template<class T>
    bool isParallel(const Vector2D<T>& a, const Vector2D<T>& b) {
        return equals(cross(a, b), 0.0);
    }
    template<class T>
    bool isParallel(const Point2D<T>& a1, const Point2D<T>& a2, const Point2D<T>& b1, const Point2D<T>& b2) {
        return isParallel(a1 - a2, b1 - b2);
    }
    template<class T>
    bool isParallel(const Line2D<T>& l1, const Line2D<T>& l2) {
        return equals(cross(l1.P2 - l1.P1, l2.P2 - l2.P1), 0.0);
    }


    /// 射影
    // 直線l に対して 点p から垂線をおろした交点を求める
    // 内積とcosの関係を用いて計算
    template<class T>
    Point2D<T> projection(const Line2D<T>& l, const Point2D<T>& p) {
        Vector2D<T> base = l.P2 - l.P1;
        long double t = dot(p - l.P1, base) / norm(base);
        return l.P1 + base * t;
    }


    /// 反射
    // 直線l を対称軸として、点p と線対称にある点を求める
    template<class T>
    Point2D<T> reflection(const Line2D<T>& l, const Point2D<T>& p) {
        return p + (projection(l, p) - p) * 2.0;
    }


    /// 交点の計算
    // 交差することが前提（交差してない場合は valid=false）
    // http://www.deqnotes.net/acmicpc/2d_geometry/lines
    template<class T>
    Point2D<T> crosspointLL(const Line2D<T> &L1, const Line2D<T> &L2) {
        if(isParallel(L1, L2)) return Point2D<T>();
        
        Vector2D<T> base = L2.P2 - L2.P1;
        long double d1 = cross(base, L2.P1 - L1.P1);
        long double d2 = cross(base, L1.P2 - L1.P1);

        if(equals(std::abs(d1), 0.0) && equals(std::abs(d2), 0.0)) return L1.P1;
        return L1.P1 + (L1.P2 - L1.P1) * d1 / d2;
    }
    template<class T>
    Point2D<T> crosspointLS(const Line2D<T> &L, const Segment2D<T> &S) {
        if(isParallel(L, S)) return Point2D<T>();

        Point2D<T> ret = crosspointLL(L, S);
        if(ccw(S.P1, S.P2, ret) == 0) return ret; 
        else return Point2D<T>();
    }
    template<class T>
    Point2D<T> crosspointSS(const Segment2D<T> &S1, const Segment2D<T> &S2) {
        if(isParallel(S1, S2)) return Point2D<T>();
        
        Point2D<T> ret = crosspointLL(S1, S2);
        if(ccw(S1.P1, S1.P2, ret) == 0 && ccw(S2.P1, S2.P2, ret) == 0) return ret;
        else return Point2D<T>();
    }


    /// 交差判定
    template<class T>
    bool intersectLP(const Line2D<T> &L, const Point2D<T> &P) {
        return abs(ccw(L.P1, L.P2, P)) != 1;
    }
    template<class T>
    bool intersectSP(const Segment2D<T> &S, const Point2D<T> &P) {
        return ccw(S.P1, S.P2, P) == 0;
    }
    template<class T>
    bool intersectLL(const Line2D<T> &L1, const Line2D<T> &L2) {
        return !isParallel(L1, L2);
    }
    template<class T>
    bool intersectLS(const Line2D<T> &L, const Segment2D<T> &S) {
        if(isParallel(L, S)) return false;

        return ccw(S.P1, S.P2, crosspoint(S, L)) == 0;
    }
    template<class T>
    bool intersectPPPP(const Point2D<T> &P1, const Point2D<T> &P2, const Point2D<T> &P3, const Point2D<T> &P4) { // 別の線分の端点が線分を堺に両側に分かれる（時計/反時計回り）ことを利用
        return (ccw(P1, P2, P3) * ccw(P1, P2, P4) <= 0 &&
                ccw(P3, P4, P1) * ccw(P3, P4, P2) <= 0);
    }
    template<class T>
    bool intersectSS(const Segment2D<T> &S1, const Segment2D<T> &S2) {
        return intersectPPPP(S1.P1, S1.P2, S2.P1, S2.P2);
    }


    /// 距離
    // 二点間の距離
    template<class T>
    long double distancePP(const Point2D<T> &P1, const Point2D<T> &P2) {
        return abs(P1 - P2);
    }

    // 直線と点の距離
    // 外積の絶対値が平行四辺形の面積になることを利用（距離が「高さ」に相当する）
    template<class T>
    long double distanceLP(const Line2D<T> &L, const Point2D<T> &P) {
        return std::abs(cross(L.P2 - L.P1, P - L.P1)) / abs(L.P2 - L.P1);
    }

    // 線分と点の距離
    // 角度に注目して端点で最小となるかを判定
    template<class T>
    long double distanceSP(const Segment2D<T> &S, const Point2D<T> &P) {
        if(dot(S.P2 - S.P1, P - S.P1) < 0.0) return abs(P - S.P1);
        if(dot(S.P1 - S.P2, P - S.P2) < 0.0) return abs(P - S.P2);
        return std::abs(cross(S.P2 - S.P1, P - S.P1)) / abs(S.P2 - S.P1);
    }

    // 線分と線分の距離
    // 端点の組が候補（三角不等式よりわかる）
    template<class T>
    long double distanceSS(const Segment2D<T> &S1, const Segment2D<T> &S2) {
        if(intersectSS(S1, S2)) return 0.0;
        return std::min({distanceSP(S1, S2.P1), distanceSP(S1, S2.P2), distanceSP(S2, S1.P1), distanceSP(S2, S1.P2)});
    }


    /// 円関連の交差判定など

    // 円と点の位置関係（内包関係）
    // 0: 外部, 1:内部, -1:周上
    // circumference : 周上も内部扱いする、 count_mode : 内部なら+1扱い
    template<class T>
    int contain(const Circle<T> &C, const Point2D<T> &P, const bool circumference = true, const bool count_mode = false) {
        long double d = distancePP(C.center, P);
        if(equals(d, C.radius)) return count_mode ? (circumference ? 1 : 0) : -1;
        else if(d < C.radius) return 1;
        else return 0;
    }

    // 円と直線の交差判定
    // 交点の数を返している
    template<class T>
    int intersectCL(const Circle<T> &C, const Line2D<T> &L) {
        long double d = distanceLP(L, C.center);

        if(equals(d, C.radius)) return 1;
        else if(d > C.radius) return 0;
        return 2;
    }

    // 円と線分の交差判定
    // 交点の数を返している（円が線分を内包している場合も交差していない扱い）
    template<class T>
    int intersectCS(const Circle<T> &C, const Segment2D<T> &S) {
        long double mn = distanceSP(S, C.center);
        if(equals(mn, C.radius)) return 1;
        else if(mn > C.radius) return 0;

        long double d1 = distancePP(S.P1, C.center);
        long double d2 = distancePP(S.P2, C.center);

        if(std::max(d1, d2) < C.radius) return 0;
        else if(std::min(d1, d2) > C.radius) return 2;
        else return 1;
    }

    // 二円の位置関係
    // 共通接線の数で識別（0: 完全に内包, 1: 内接, 2: 交差, 3: 外接, 4: 完全に離れている）
    template<class T>
    int intersectCC(Circle<T> C1, Circle<T> C2) {
        assert(C1!=C2);

        if(C1.radius < C2.radius) std::swap(C1, C2);
        
        // C1.radius >= C2.radius
        long double d = distancePP(C1.center, C2.center);
        if(equals(d, C1.radius + C2.radius)) return 3;
        else if(equals(d + C2.radius, C1.radius)) return 1;
        else if(C1.radius + C2.radius < d) return 4;
        else if(d + C2.radius < C1.radius) return 0;
        else return 2;
    }

    // 円と直線の交点（0~2つ）
    // x座標が小さいものから格納される、1つの場合は同じ頂点が2つ入る、0つの場合はvalid=false
    template<class T>
    std::pair<Point2D<T>, Point2D<T> > crosspointCL(const Circle<T> &C, const Line2D<T> &L) {
        Point2D<T> mid = projection(L, C.center);
        long double d = distanceLP(L, C.center);
    
        if(equals(d, C.radius)) return {mid, mid}; // 交点1つ
        else if(d > C.radius) return {Point2D<T>(), Point2D<T>()}; // 交点0つ

        Vector2D<T> e = unit(L.P1, L.P2);
        if(e.x < -EPS) e.x *= -1.0, e.y *= -1.0;
        else if(equals(e.x, 0) && e.y < -EPS) e.y *= -1.0; 
        long double base = std::sqrt(C.radius * C.radius - norm(mid - C.center));
        return {mid - e * base, mid + e * base};
    }

    // 円と線分の交点（0~2つ）
    // x座標が小さいものから格納される、1つの場合は同じ頂点が2つ入る、0つの場合はvalid=false
    template<class T>
    std::pair<Point2D<T>, Point2D<T> > crosspointCS(const Circle<T> &C, const Segment2D<T> &S) {
        int cnt = intersectCS(C, S);
        if(cnt == 0) return {Point2D<T>(), Point2D<T>()};

        std::pair<Point2D<T>, Point2D<T> > ret = crosspointCL(C, Line2D<T>(S));
        if(cnt == 2) return ret;
        else {
            if(ccw(S.P1, S.P2, ret.first) == ON_SEGMENT) ret.second = ret.first;
            else ret.first = ret.second;
            return ret;
        }
    }

    // 円と円の交点（0~2つ）
    // x座標が小さいものから格納される、1つの場合は同じ頂点が2つ入る、0つの場合はvalid=false
    // 余弦定理から求めている
    template<class T>
    std::pair<Point2D<T>, Point2D<T> > crosspointCC(const Circle<T> &C1, const Circle<T> &C2) {
        int is_int = intersectCC(C1, C2);
        if(is_int == 0 || is_int == 4) return {Point2D<T>(), Point2D<T>()};

        Vector2D<T> V = C2.center - C1.center;
        long double d = abs(V);
        long double a = std::acos((C1.radius * C1.radius + d * d - C2.radius * C2.radius) / (2 * C1.radius * d));
        long double t = arg(V);

        std::pair<Point2D<T>, Point2D<T> > ret = std::make_pair(C1.center + polar_to_xy(C1.radius, t + a), C1.center + polar_to_xy(C1.radius, t - a));
        if(ret.first.x > ret.second.x) std::swap(ret.first, ret.second);
        else if(equals(ret.first.x, ret.second.x) && ret.first.y > ret.second.y) std::swap(ret.first, ret.second);
        return ret;
    }

    /// 重心計算
    template<class T>
    Point2D<T> Centroid(const std::vector<Point2D<T> >& G) {
        int N = (int)G.size();
        assert(N>0);

        Point2D<T> ret(0, 0);
        for(auto const& P : G) {
            ret.x += P.x;
            ret.y += P.y;
        }
        ret /= N;
        
        return ret;
    }

    /// 凸多角形の頂点を反時計回りで訪問するように整列
    // 重心を基準とした偏角ソートを用いる
    // 引数 make_unique で重複除去するかを指定
    template<class T>
    void arg_sort_poly(Polygon<T>& P, const bool make_unique = false) {
        auto G = Centroid(P);

        std::sort(P.begin(), P.end(), [&](const Point2D<T>& P1, const Point2D<T>& P2) {
            auto P1_ = P1 - G, P2_ = P2 - G;
            if(P1_.argpos() != P2_.argpos()) return P1_.argpos() < P2_.argpos();
            return cross(P1_, P2_) > 0;
        });

        if(make_unique) P.erase(std::unique(P.begin(), P.end()), P.end()); 
    }


    // 多角形の面積
    // 頂点集合の順序は、隣り合った点を反時計回りに訪問することを要求（do_sort引数でソートする）
    template<class T>
    long double area(Polygon<T> &P, const bool do_sort = false) {
        if(do_sort) arg_sort_poly(P);

        long double S = 0;
        for(int i=0; i<(int)P.size(); i++) {
            S += cross(P[i], P[(i+1) % (int)P.size()]);
        }
        return S / 2;
    }


    /// 多角形と点の位置関係（内包関係）
    // 0: 外部, 1:内部, -1:周上
    // circumference : 周上も内部扱いする、 count_mode : 内部なら+1扱い
    // Polygonは凸でなくてもよい、頂点集合の順序は、隣り合った点を反時計回りに訪問することを要求（do_sort引数でソートする）
    template<class T>
    int contain(Polygon<T> &Poly, const Point2D<T> &P, const bool circumference = true, const bool count_mode = false, const bool do_sort = false) {
        if(do_sort) arg_sort_poly(Poly);
        
        int N = Poly.size();
        bool in = false;
        for(int i=0; i<N; i++) {
            Point2D<T> A = Poly[i] - P;
            Point2D<T> B = Poly[(i+1)%N] - P;
            if(std::abs(cross(A,B)) < EPS && dot(A,B) < EPS) return count_mode ? (circumference ? 1 : 0) : -1;
            if(A.y > B.y) std::swap(A, B);
            if(A.y < EPS && EPS < B.y && cross(A,B) > EPS) in = !in;
        }
        return in;
    }


    /// 下側凸包（x昇順、反時計回り）を生成
    // Andrew's Algorithm (x must be sorted)
    // boundary : 周上の点も列挙する場合 true
    template<class T>
    std::vector<Point2D<T> > LowerHull(std::vector<Point2D<T> > ps, bool boundary = false) {
        x_sort(ps);
        ps.erase(std::unique(ps.begin(), ps.end()), ps.end());        
        int N = (int)ps.size();
        if (N <= 2) return ps;

        std::vector<Point2D<T> > convex(N);
        int k = 0;
        long double th = boundary ? -EPS : +EPS;
        for (int i = 0; i < N; convex[k++] = ps[i++]) {
            while (k >= 2 && cross(convex[k - 1] - convex[k - 2], ps[i] - convex[k - 1]) < th) --k;
        }
        convex.resize(k);
        return convex;
    }

    /// 上側凸包（x昇順、時計回り）を生成
    // Andrew's Algorithm (x must be sorted)
    // boundary : 周上の点も列挙する場合 true
    template<class T>
    std::vector<Point2D<T> > UpperHull(std::vector<Point2D<T> > ps, bool boundary = false) {
        x_sort(ps);
        ps.erase(std::unique(ps.begin(), ps.end()), ps.end());        
        int N = (int)ps.size();
        if (N <= 2) return ps;

        std::vector<Point2D<T> > convex(N);
        int k = 0;
        long double th = boundary ? +EPS : -EPS;
        for (int i = 0; i < N; convex[k++] = ps[i++]) {
            while (k >= 2 && cross(convex[k - 1] - convex[k - 2], ps[i] - convex[k - 1]) > th) --k;
        }
        convex.resize(k);
        return convex;
    }

    /// 凸包（反時計回り）を生成
    // Andrew's Algorithm (x must be sorted)
    // boundary : 周上の点も列挙する場合 true    
    template<class T>
    std::vector<Point2D<T> > ConvexHull(std::vector<Point2D<T> > ps, bool boundary = false) {
        x_sort(ps);
        ps.erase(std::unique(ps.begin(), ps.end()), ps.end());        
        int N = (int)ps.size();
        if (N <= 2) return ps;

        std::vector<Point2D<T> > convex(2*N);
        int k = 0;
        long double th = boundary ? -EPS : +EPS;
        for (int i = 0; i < N; convex[k++] = ps[i++]) {
            while (k >= 2 && cross(convex[k - 1] - convex[k - 2], ps[i] - convex[k - 1]) < th) --k;
        }
        for (int i = N - 2, t = k + 1; i >= 0; convex[k++] = ps[i--]) {
            while (k >= t && cross(convex[k - 1] - convex[k - 2], ps[i] - convex[k - 1]) < th) --k;
        }
        convex.resize(k - 1);
        return convex;
    }


    /// 凸性判定
    // 頂点集合の順序は、隣り合った点を反時計回りに訪問することを要求（do_sort引数でソートする）
    template<class T>
    bool is_convex(Polygon<T>& P, const bool do_sort = false) {
        if(do_sort) arg_sort_poly(P);

        int N = (int) P.size();
        for(int i = 0; i < N; i++) {
            if(ccw(P[(i + N - 1) % N], P[i], P[(i + 1) % N]) == CLOCKWISE) return false;
        }
        return true;
    }


    /// 凸多角形の直径（最遠点対間距離）
    // キャリパー法（平行線を凸多角形に沿って一回転してるイメージ、平行線に垂直な方向ベクトルに関して射影した距離に注目している）
    // 頂点集合の順序は、隣り合った点を反時計回りに訪問することを要求（do_sort引数でソートする）
    // 返り値：（最遠点対間距離の二乗、最遠点対の頂点のペア）indexはidxから読み出せる
    template<class T>
    std::pair<T, std::pair<Point2D<T>, Point2D<T> > > convex_diameter(Polygon<T>& P, const bool do_sort = false) {
        if(do_sort) arg_sort_poly(P);
        
        const int N = (int)P.size();
        if(N == 1) return std::make_pair((T)0, std::make_pair(P[0], P[0]));
        else if(N == 2) return std::make_pair(norm(P[1] - P[0]), std::make_pair(P[0], P[1]));

        int is = 0, js = 0;
        for (int i = 1; i < N; i++) {
            if (P[i].y > P[is].y) is = i;
            if (P[i].y < P[js].y) js = i;
        }
        T maxdis = norm(P[is] - P[js]);

        int maxi = is, maxj = js;
        int i = is, j = js;
        do {
            if (cross(P[(i + 1) % N] - P[i], P[(j + 1) % N] - P[j]) < 0) i = (i + 1) % N;
            else j = (j + 1) % N;
            
            T dist = norm(P[i] - P[j]);
            if (dist > maxdis) {
                maxdis = dist;
                maxi = i;
                maxj = j;
            }
        } while (i != js || j != is); // 半周にしたら yosupo judge のバグ取れたが、コーナーケースを要検証

        if(maxi > maxj) std::swap(maxi, maxj);
        return std::make_pair(maxdis, std::make_pair(P[maxi], P[maxj]));
    }

    /// 最遠点対間距離
    // 凸包 + キャリパー法のセット
    // 返り値：（最遠点対間距離の二乗、最遠点対の頂点番号のペア）indexはidxから読み出せる
    template<class T>
    std::pair<T, std::pair<Point2D<T>, Point2D<T> > > furthest_pair(const std::vector<Point2D<T> >& Ps) {
        assert((int)Ps.size() >= 2);
        auto convex = ConvexHull(Ps);
        if((int)convex.size() == 1) return std::make_pair((T)0, std::make_pair(Ps[0], Ps[1]));
        return convex_diameter(convex);
    }

    /// 最近点対間距離
    // 分割統治を用いる
    // 返り値：（最近点対間距離の二乗、最近点対のペア）indexはidxから読み出せる
    template<class T>
    std::pair<T, std::pair<Point2D<T>, Point2D<T> > > closest_pair(std::vector<Point2D<T> > Ps) {
        assert((int)(Ps.size()) >= 2);
        x_sort(Ps);

        auto cmp_y = [&](const Point2D<T>& a, const Point2D<T>& b) -> bool {
            return a.y < b.y;
        };
        
        auto rec = [&](auto& self, int left, int right) { // [left, right)
            if(right - left <= 1) return make_pair(std::numeric_limits< T >::max(), make_pair(Point2D<T>(), Point2D<T>()));

            int mid = (left + right) >> 1;
            T xline = Ps[mid].x;
            auto ret = std::min(self(self, left, mid), self(self, mid, right)); // 分割領域内での最小距離
            T dist = ret.first;
            std::inplace_merge(Ps.begin() + left, Ps.begin() + mid, Ps.begin() + right, cmp_y); // y昇順にソートを保つ

            // 分割領域間での最小距離（distで枝刈り）
            std::vector<Point2D<T> > B;
            for(int i = left; i < right; i++) {
                if(std::norm(Ps[i].x - xline) >= dist) continue; // 境界線からx座標がdist離れてる時は無視で良い
                
                for(int j = (int)B.size() - 1; j >= 0; j--) { // y昇順に見ていることに注目して、既に採用したものをy座標がdistを超えない範囲で見る（後ろ向きで走査すれば良い）
                    Vector2D<T> D = Ps[i] - B[j];
                    if(D.y * D.y >= dist) break;

                    long double curd = D.norm();
                    if(curd < dist) {
                        dist = curd;
                        ret.first = curd;
                        ret.second.first = Ps[i];
                        ret.second.second = B[j];
                    }
                }

                B.emplace_back(Ps[i]);
            }
        
            return ret;
        };

        return rec(rec, 0, (int)(Ps.size()));
    }    


    /// 三点が三角形を成すか？
    template<class T>
    bool is_Triangle(const Point2D<T>& A, const Point2D<T>& B, const Point2D<T>& C) {
        return !isParallel(B, A, C, A);
    }

    /// 内接円
    template<class T>
    Circle<T> incircle(const Point2D<T>& A, const Point2D<T>& B, const Point2D<T>& C) {
        assert(is_Triangle(A, B, C));

        // 内心は「角の二等分線の交点」であることを利用
        long double a = get_angle(C-A, B-A), b = get_angle(A-B, C-B);
        Line2D<T> S1(A, A + rotate(B-A, a/2));
        Line2D<T> S2(B, B + rotate(C-B, b/2));

        Point2D<T> Cent = crosspointLL(S1, S2);
        return Circle<T>(Cent, distanceSP(Segment2D<T>(A, B), Cent));
    }

    /// 3点に対する外接円
    template<class T>
    Circle<T> circumcircle(const Point2D<T>& A, const Point2D<T>& B, const Point2D<T>& C) {
        assert(is_Triangle(A, B, C));

        // 外心は「各辺の垂直二等分線の交点」であることを利用
        Point2D<T> Cent = crosspointLL(bisector(A, B), bisector(A, C));
        return Circle<T>(Cent, distancePP(Cent, A));
    }

    /// 2点を周上に含む半径最小の円
    template<class T>
    Circle<T> circumcircle_2P(const Point2D<T>& A, const Point2D<T>& B) {
        Point2D<T> C = mid(A, B);
        T R = distancePP(A,C);
        return Circle<T>(C,R);
    }

    /// 垂心 (need verify)
    template<class T>
    Point2D<T> orthocenter(const Point2D<T>& A, const Point2D<T>& B, const Point2D<T>& C) {
        assert(is_Triangle(A, B, C));
        
        return Point2D<T>(crossPointLL(Line2D<T>(A, A + rot90(C-B)), crosspointLL(Line2D<T>(B, B + rot90(C-A)))));
    }

    /// 最小包含円
    // https://tubo28.me/compprog/algorithm/minball/
    // 期待計算量 O(N)
    template<class T>
    Circle<T> MinBound_Circle(std::vector<Point2D<T> > Ps) {
        int N = (int)Ps.size();
        assert(N >= 1);
        if(N == 1) {
            return Circle<T>(Ps[0], 0);
        }

        std::shuffle(Ps.begin(), Ps.end(), mt);
        Circle<T> ret = circumcircle_2P(Ps[0], Ps[1]);
        for(int i = 2; i < N; i++) {
            if(contain(ret, Ps[i]) == 0) {
                ret = circumcircle_2P(Ps[0], Ps[i]);
                for(int j = 1; j < i; j++) {
                    if(contain(ret, Ps[j]) == 0) {
                        ret = circumcircle_2P(Ps[i], Ps[j]);
                        for(int k = 0; k < j; k++) {
                            if(contain(ret, Ps[k]) == 0) {
                                ret = circumcircle(Ps[i], Ps[j], Ps[k]);
                            }
                        }
                    }
                }
            }
        }

        return ret;
    }

    /// 点から円への接線（2つ）
    // 点が円の内部にないことを要求（周上の場合は一つの接線が両方に格納される）
    // 返り値：円との接点を返す（x昇順）
    template<class T>
    std::pair<Point2D<T>, Point2D<T> > tangent_CP(const Circle<T>& C, const Point2D<T>& P) {
        int inonout = contain(C, P);
        assert(inonout != 1);

        if(inonout == -1) { // 周上
            return std::make_pair(P, P);
        }
        else { // 外部
            return crosspointCC(C, Circle<T>(P, std::sqrt(norm(P - C.center) - C.radius * C.radius)));
        }
    }

    /// 二円の共通接線（0~4つ）
    // https://tjkendev.github.io/procon-library/python/geometry/circle_common_tangent_point.html
    // 返り値：円C1との接点を返す（x昇順）
    template<class T>
    std::vector<Point2D<T> > tangent_CC(const Circle<T>& C1, const Circle<T>& C2) {
        std::vector<Point2D<T> > ret;

        T dx = C2.center.x - C1.center.x;
        T dy = C2.center.y - C1.center.y;
        T dc = norm(C1.center - C2.center);

        // 共通外接線（完全内包以外なら存在）
        T rm = (C1.radius - C2.radius) * (C1.radius - C2.radius);
        if(rm <= dc + EPS) {
            T cv = C1.radius - C2.radius;
            if(equals(dc, rm)) { // 円が内接する場合は、1つ
                Vector2D<T> V(C1.radius*cv*dx/dc, C1.radius*cv*dy/dc);
                ret.emplace_back(C1.center + V);
            }
            else { // それ以外は2つ
                T sv = std::sqrt(dc - cv*cv);
                ret.emplace_back(Point2D<T>(C1.center.x + C1.radius*(cv*dx - sv*dy)/dc, C1.center.y + C1.radius*(sv*dx + cv*dy)/dc));
                ret.emplace_back(Point2D<T>(C1.center.x + C1.radius*(cv*dx + sv*dy)/dc, C1.center.y + C1.radius*(-sv*dx + cv*dy)/dc));
            }
        }

        // 共通内接線（外接or完全に離れてるなら存在）
        T rp = (C1.radius + C2.radius) * (C1.radius + C2.radius);
        if(rp <= dc + EPS) {
            T cv = C1.radius + C2.radius;
            if(equals(dc, rp)) { // 円が外接する場合は、1つ
                Vector2D<T> V(C1.radius*cv*dx/dc, C1.radius*cv*dy/dc);
                ret.emplace_back(C1.center + V);
            }
            else {
                T sv = std::sqrt(dc - cv*cv);
                ret.emplace_back(Point2D<T>(C1.center.x + C1.radius*(cv*dx - sv*dy)/dc, C1.center.y + C1.radius*(sv*dx + cv*dy)/dc));
                ret.emplace_back(Point2D<T>(C1.center.x + C1.radius*(cv*dx + sv*dy)/dc, C1.center.y + C1.radius*(-sv*dx + cv*dy)/dc));
            }
        }

        x_sort(ret);
        return ret;
    }

    /// convex cut（凸多角形を直線で切断）
    // 直線(L1.P1 -> L1.P2)で切断した時に左側に出来る凸多角形を返す
    // 頂点集合の順序は、隣り合った点を反時計回りに訪問することを要求（do_sort引数でソートする）
    template<class T>
    Polygon<T> convex_cut(Polygon<T>& P, const Line2D<T>& L, const bool do_sort = false) {
        if(do_sort) arg_sort_poly(P);
        
        int N = P.size();
        Polygon<T> ret;
        for(int i=0; i<N; i++) {
            Point2D<T> now = P[i];
            Point2D<T> nxt = P[(i+1)%N];

            T cf = cross(L.P1 - now, L.P2 - now);
            T cs = cross(L.P1 - nxt, L.P2 - nxt);
            if(sign(cf) >= 0) ret.emplace_back(now);
            if(sign(cf) * sign(cs) < 0) ret.emplace_back(crosspointLL(Line2D<T>(now, nxt), L));
        }
        return ret;
    }

    /// 円と多角形の共通部分の面積
    // https://kyopro.hateblo.jp/entry/2019/08/01/192232
    // 頂点集合の順序は、隣り合った点を反時計回りに訪問することを要求（do_sort引数でソートする）
    // 境界線上にあるときに協会判定で誤差落ちしがちなのでなんとかしたい -> 円に半径の2乗の情報も持たせる？
    template<class T>
    long double intersection_area_CP(const Circle<T>& C, Polygon<T>& P, const bool do_sort = false) {
        if(do_sort) arg_sort_poly(P);
        
        long double area = 0.0;
        const int N = P.size();
        for(int i=0; i<N; i++) {
            const Point2D<T> P1 = P[i] - C.center;
            const Point2D<T> P2 = P[(i+1)%N] - C.center;

            if(std::abs(ccw(C.center, P1, P2)) != 1) continue;

            if((P1.norm() <= C.radius * C.radius) && (P2.norm() <= C.radius * C.radius)) {
                area += 0.5 * cross(P1, P2);
            }
            else if(P1.norm() < C.radius * C.radius) {
                auto ps = crosspointCS(Circle<long double>(C), Segment2D<long double>(P1, P2));
                if(ccw(Point2D<long double>(P2), ps.first, ps.second) == -2) std::swap(ps.first, ps.second);
                area += 0.5 * cross(Point2D<long double>(P1), ps.first);
                area += 0.5 * C.radius * C.radius * arg(ps.first, Point2D<long double>(P2));        
            }
            else if(P2.norm() < C.radius * C.radius) {
                auto ps = crosspointCS(Circle<long double>(C), Segment2D<long double>(P1, P2));
                if(ccw(Point2D<long double>(P2), ps.first, ps.second) == -2) std::swap(ps.first, ps.second);
                area += 0.5 * C.radius * C.radius * arg(Point2D<long double>(P1), ps.first);        
                area += 0.5 * cross(ps.first, Point2D<long double>(P2));
            }
            else {
                auto ps = crosspointCS(Circle<long double>(C), Segment2D<long double>(P1, P2));
                if(ccw(Point2D<long double>(P2), ps.first, ps.second) == -2) std::swap(ps.first, ps.second);
                if(!ps.first.valid) area += 0.5 * C.radius * C.radius * arg(P1, P2);
                else {
                    area += 0.5 * C.radius * C.radius * arg(Point2D<long double>(P1), ps.first);
                    area += 0.5 * cross(ps.first, ps.second);
                    area += 0.5 * C.radius * C.radius* arg(ps.second, Point2D<long double>(P2));                    
                }
            }
        }

        return area;
    }   

    /// 円と円の共通部分の面積
    // 余弦定理＆扇形2つ-三角形2つで求めている
    template<class T>
    long double intersection_area_CC(const Circle<T>& C1, const Circle<T>& C2) {
        if(C1 == C2) return pi * C1.radius * C1.radius;

        int state = intersectCC(C1, C2);
        if(state >= 3) return 0.0;
        else if(state <= 1) return pi * min(C1.radius, C2.radius) * min(C1.radius, C2.radius);

        long double dd = norm(C1.center - C2.center);
        long double p1 = C1.radius * C1.radius - C2.radius * C2.radius + dd;
        long double p2 = C2.radius * C2.radius - C1.radius * C1.radius + dd;

        long double S1 = C1.radius * C1.radius * std::atan2(std::sqrt(4*dd*C1.radius*C1.radius - p1*p1), p1);
        long double S2 = C2.radius * C2.radius * std::atan2(std::sqrt(4*dd*C2.radius*C2.radius - p2*p2), p2);
        long double S0 = std::sqrt(4*dd*C1.radius*C1.radius - p1*p1) / 2;

        return S1 + S2 - S0;
    }

    /// 多角形同士の共通部分の面積 (O(NM))
    // 頂点集合の順序は、隣り合った点を反時計回りに訪問することを要求（do_sort引数でソートする）
    // 凸多角形の頂点をO(NM)で高々O(N+M)列挙し、O((N+M)log(N+M))でソートしてから面積を求める
    // TODO : 凸包を求める際の過程を利用して、O(N+M)でも出来るらしい（https://ikatakos.com/pot/programming_algorithm/geometry/convex_intersection）
    template<class T>
    long double intersection_area_PP(Polygon<T>& P1, Polygon<T>& P2, const bool do_sort = false) {
        if(do_sort) {
            arg_sort_poly(P1);
            arg_sort_poly(P2);
        }

        Polygon<T> ret;
        int N = P1.size();
        int M = P2.size();

        // Sの辺とTの辺の交点（端点含む）
        for(int i=0; i<N; i++) {
            Segment2D<T> S1(P1[i], P1[(i+1)%N]);
            for(int j=0; j<M; j++) {
                Segment2D<T> S2(P2[j], P2[(j+1)%M]);

                Point2D<T> cp = crosspointSS(S1, S2);
                if(cp.valid) ret.emplace_back(cp);
                else { // 辺が重なってる場合のみ例外
                    if(intersectSP(S1, P2[j])) {
                        if(intersectSP(S1, P2[(j+1)%M])) {
                            ret.emplace_back(P2[j]);
                            ret.emplace_back(P2[(j+1)%M]);
                        }
                        else {
                            ret.emplace_back(P2[j]);
                            if(intersectSP(S2, P1[i])) ret.emplace_back(P1[i]);
                            else ret.emplace_back(P1[(i+1)%N]);
                        }
                    }
                    else if(intersectSP(S1, P2[(j+1)%M])) {
                        ret.emplace_back(P2[(j+1)%M]);
                        if(intersectSP(S2, P1[i])) ret.emplace_back(P1[i]);
                        else ret.emplace_back(P1[(i+1)%N]);                        
                    }
                }

            }
        }

        // Sの頂点であり、Tの内部（周上除く）に含まれるもの
        for(int i=0; i<N; i++) {
            if(contain(P2, P1[i], false)) ret.emplace_back(P1[i]);
        }

        // Tの頂点であり、Sの内部（周上除く）に含まれるもの
        for(int j=0; j<M; j++) {
            if(contain(P1, P2[j], false)) ret.emplace_back(P2[j]);
        }

        if((int)ret.size() <= 2) return 0.0;

        arg_sort_poly(ret, true);

        return area(ret);
    }   
    

    /// 傍心

    /// 線分アレンジメント
    /// 任意の2線分の交点を頂点としたグラフを構築する

    /// （重み付き＋）ボロノイ図（＋ドロネー三角形分割）

    /// kD Tree

    /// ユークリッド最小全域木

    /// 動的凸包（+ convex layers）

    /// https://judge.yosupo.jp/problem/count_points_in_triangle のやつ？

    /// （三角形クラス）


}
