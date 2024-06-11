/*
★ Name        ：2D計算幾何テンプレート
 ■ Snipet      :
 
 ■ Arguments   :
 ■ Return      :
 ■ Complexity  :

 ■ Description :
 

 ■ Usage       :


 ■ Verify      :
 ■ References  : https://ei1333.github.io/luzhiled/snippets/geometry/template.html
                 https://nyaannyaan.github.io/library/geometry/geometry-base.hpp
                 螺旋本（プログラミングコンテスト攻略のためのアルゴリズムとデータ構造）16章

 ■ TODO        : 完成させる
*/


#pragma once
#include <cmath>
#include <vector>
#include <limits>


namespace Geometry {
    constexpr long double EPS = (1e-10);
    constexpr long double pi = 3.141592653589793238462643383279L;

    template <class T> bool equals(T a, T b) { return fabsl(a - b) < EPS; }
    template <class T> bool notequals(T a, T b) { return !equlas(a, b); }
    template <class T> int sign(T a) { return equals(a, 0) ? 0 : a > 0 ? 1 : -1; }


    /// 2次元平面上の点(x,y)
    template <class T>
    class Point2D { // D は dimension の D (Double ではない) -> OpenGL等の著名ライブラリとの命名規則統一すべき？
        public:
            T x, y; // privateにしても良い、要検討

            Point2D() : x(0), y(0) {}
            Point2D(T x = 0, T y = 0): x(x), y(y) {}
            template <class U1, class U2> Point2D(const std::pair<U1, U2>& p) : x(p.first), y(p.second) {}
    
            
            Point2D operator + (const Point2D& p) {return Point2D(x + p.x, y + p.y); }
            Point2D operator - (const Point2D& p) {return Point2D(x - p.x, y - p.y); }
            Point2D operator * (T a) {return Point2D(a * x, a * y); }
            Point2D operator / (T a) {return Point2D(x / a, y / a); }
            Point2D& operator+=(const Point2D& p) { return (*this) = (*this) + p; }
            Point2D& operator-=(const Point2D& p) { return (*this) = (*this) - p; }
            Point2D& operator*=(T a) { return (*this) = (*this) * a; }
            Point2D& operator/=(T a) { return (*this) = (*this) / a; }
            Point2D& operator=(const Point2D& p) {return (*this) = p; } // check
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
            T abs() const { return std::sqrtl(norm()); }
            T arg() const { return std::atan2l(y, x); }
    
            void rotate(T rad) { // 反時計回りにrad度回転
                T x_ = x * std::cosl(rad) - y * std::sinl(rad);
                T y_ = x * std::sinl(rad) + y * std::cosl(rad);
                x = x_, y = y_;
            }
            friend Point2D rotate(const Point2D& p, T rad) { // 反時計回りにrad度回転
                return {p.x * std::cosl(rad) - p.y * std::sinl(rad), p.x * std::sinl(rad) + p.y * std::cosl(rad)};
            }
            
            friend istream& operator>>(istream& is, Point2D& p) {
                T a, b;
                is >> a >> b;
                p = Point2D(a, b);
                return is;
            }
            friend ostream& operator<<(ostream& os, const Point2D& p) {
                return os << fixed << setprecision(15) << p.x << " " << p.y;
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
    using Polygon = std::vector<Point>; // 多角形

    template <class T> 
    using Vector2D = Point2D<T>; // 向き(x,y)と大きさ(abs(x,y))を持つ


    template <class T> T real(const Point2D<T>& p) { return p.x; }
    template <class T> T imag(const Point2D<T>& p) { return p.y; }
    template <class T> T dot(const Point2D<T>& l, const Point2D<T>& r) { return l.x * r.x + l.y * r.y; }
    template <class T> T cross(const Point2D<T>& l, const Point2D<T>& r) { return l.x * r.y - l.y * r.x; }
    template <class T> T norm(const Point2D<T>& p) { return p.x * p.x + p.y * p.y; }
    template <class T> T abs(const Point2D<T>& p) { return std::sqrtl(norm(p)); }
    template <class T> T arg(const Point2D<T>& p) { return std::atan2l(p.y, p.x); }



    /// 2点 P1, P2 を通る二次元平面上の直線 Ax + By + C = 0
    template <class T>
    class Line2D {
        public : 
            Point2D<T> P1, P2;
            T A, B, C;

            Line2D() = default;

            // P1, P2 -> Ax + By + C = 0 を計算
            Line2D(Point2D<T> P1, Point2D<T> P2) : P1(P1), P2(P2) // check
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

            friend ostream &operator<<(ostream &os, Line2D &L) {
                return os << "(" << L.P1 << "), (" << L.P2 << ") / " << A << "x + " << B << "y + "<< C;
            }


            // 直線の傾きを取得
            T Grad() {
                if(equals(B, 0)) return numeric_limits< T >::max()
                return - A / B;
            }
        

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



    /// 中心点 c と、半径 r を持つ二次元平面上の円
    template <class T>
    class Circle {
        public:
            Point2D<T> c;
            T r;

            Circle() = default;

            Circle(Point2D<T> c, T r) : c(c), r(r) {}
    };


    /// p0, p1, p2 がこの順で反時計回り（Counter Clock Wise）か？の判定
    static const int COUNTER_CLOCKWISE = 1; // 反時計回り
    static const int CLOCKWISE = -1; // 時計回り
    static const int ONLINE_BACK = 2; // P2 -> P0 -> P1 の順で一直線 (P0 -> P1 とは逆方向にP2が存在)
    static const int ONLINE_FRONT = -2; // P0 -> P1 -> P2 の順で一直線 (P0 -> P1 と同方向にP2が存在)
    static const int ON_SEGMENT = 0; // P0 -> P2 -> P1 の順で一直線 (P0 -> P1 内にP2が存在)
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
        return l.P1 + t * base;
    }


    /// 反射
    // 直線l を対称軸として、点p と線対称にある点を求める
    template<class T>
    Point2D<T> reflection(const Line2D<T>& l, const Point2D<T>& p) {
        return p + (projection(l, p) - p) * 2.0;
    }


    /// 交差判定
    template<class T>
    bool intersect(const Line2D<T> &L, const Point2D<T> &P) {
        return abs(ccw(L.P1, L.P2, P)) != 1;
    }
    template<class T>
    bool intersect(const Segment2D<T> &S, const Point2D<T> &P) {
        return ccw(S.P1, S.P2, P) == 0;
    }
    template<class T>
    bool intersect(const Line2D<T> &L1, const Line2D<T> &L2) {
        return abs(cross(L1.P2 - L1.P1, L2.P2 - L1.P1)) > EPS;
    }
    template<class T>
    bool intersect(const Segment2D<T> &S, const Line2D<T> &L) {

    }
    template<class T>
    bool intersect(const Line2D<T> &L, const Segment2D<T> &S) {
        
    }


    /// 距離
    // 二点間の距離
    template<class T>
    long double distance(const Point2D<T> &P1, const Point2D<T> &P2) {
        return abs(P1 - P2);
    }

    // 直線と点の距離
    // 外積の絶対値が平行四辺形の面積になることを利用（距離が「高さ」に相当する）
    template<class T>
    long double distance(const Line2D<T> &L, const Point2D<T> &P) {
        return abs(cross(L.P2 - L.P1, P - L.P1)) / abs(L.P2 - L.P1);
    }

    // 線分と点の距離
    // 角度に注目して端点で最小となるかを判定
    template<class T>
    long double distance(const Segment2D<T> &S, const Point2D<T> &P) {
        if(dot(S.P2 - S.P1, P - S.P1) < 0.0) return abs(P - S.P1);
        if(dot(S.P1 - S.P2, P - S.P2) < 0.0) return abs(P - S.P2);
        return abs(cross(S.P2 - S.P1, P - S.P1)) / abs(S.P2 - S.P1);
    }

    // 線分と線分の距離
    // 端点の組が候補（三角不等式よりわかる）
    template<class T>
    long double distance(const Segment2D<T> &S1, const Segment2D<T> &S2) {
        if(intersect(S1, S2)) return 0.0;
        return min({distance(S1.P1, S2.P1), distance(S1.P1, S2.P2), distance(S1.P2, S2.P1), distance(S1.P2, S2.P2)});
    }

}
