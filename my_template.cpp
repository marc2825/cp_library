/// 生成AI不使用 / Not using generative AI

// #pragma GCC target ("avx")
// #pragma GCC optimize("Ofast")
#pragma GCC optimize("O3")
#pragma GCC optimize("unroll-loops")

#ifdef MARC_LOCAL
#define _GLIBCXX_DEBUG
#endif

#ifdef ATCODER
#include <atcoder/all>
using namespace atcoder;
#endif

#include <bits/stdc++.h>
using namespace std;

//#define int long long
using ll = long long; using vll = vector<ll>; using vvll = vector<vll>; using vvvll = vector<vvll>; using vvvvll = vector<vvvll>;
using vi = vector<int>; using vvi = vector<vi>; using vvvi = vector<vvi>; using vvvvi = vector<vvvi>;
using ld = long double; using vld = vector<ld>; using vvld = vector<vld>; using vd = vector<double>;
using vc = vector<char>; using vvc = vector<vc>; using vs = vector<string>;
using vb = vector<bool>; using vvb = vector<vb>; using vvvb = vector<vvb>;
using pii = pair<int, int>; using pcc = pair<char, char>; using pll = pair<ll, ll>; using pli = pair<ll, int>; using pdd = pair<double, double>; using pldld = pair<ld,ld>;
using vpii = vector<pii>; using vvpii = vector<vpii>; using vpll = vector<pll>; using vvpll = vector<vpll>; using vpldld = vector<pldld>;
using ui = unsigned int; using ull = unsigned long long;
using i128 = __int128; using f128 = __float128;
template<class T> using pqg = priority_queue<T, vector<T>, greater<T>>;
#define vec(type, name, ...) vector<type> name(__VA_ARGS__)
#define vv(type, name, h, ...) vector<vector<type> > name(h, vector<type>(__VA_ARGS__))
#define vvv(type, name, h, w, ...) vector<vector<vector<type> > > name(h, vector<vector<type> >(w, vector<type>(__VA_ARGS__)))
#define vvvv(type, name, a, b, c, ...) vector<vector<vector<vector<type> > > > name(a, vector<vector<vector<type> > >(b, vector<vector<type> >(c, vector<type>(__VA_ARGS__))))

#define overload4(a,b,c,d,name,...) name
#define rep1(n) for(ll i = 0; i < (ll)n; i++)
#define rep2(i,n) for(ll i = 0; i < (ll)n; i++)
#define rep3(i,a,b) for (ll i = a; i < (ll)b; i++)
#define rep4(i,a,b,c) for (ll i = a; i < (ll)b; i += (ll)c)
#define rep(...) overload4(__VA_ARGS__,rep4,rep3,rep2,rep1)(__VA_ARGS__)
#define repback1(n) for (ll i = (ll)n-1; i >= 0; i--)
#define repback2(i,n) for (ll i = (ll)n-1; i >= 0; i--)
#define repback3(i,a,b) for (ll i = (ll)b-1; i >= (ll)a; i--)
#define repback4(i,a,b,c) for (ll i = (ll)b-1; i >= (ll)a; i -= (ll)c)
#define repback(...) overload4(__VA_ARGS__,repback4,repback3,repback2,repback1)(__VA_ARGS__)
#define all(x) (x).begin(), (x).end()
#define include(y, x, H, W) (0 <= (y) && (y) < (H) && 0 <= (x) && (x) < (W))
#define inrange(x, down, up) ((down) <= (x) && (x) <= (up))
#define pb push_back
#define eb emplace_back
#define fir first
#define sec second
#define TIMER_START TIME_START = clock()
#ifdef MARC_LOCAL
// https://trap.jp/post/1224/
#define debug1(x) cout << "debug: " << (#x) << ": " << (x) << endl
#define debug2(x, y) cout << "debug: " << (#x) << ": " << (x) << ", " << (#y) << ": " << (y) << endl
#define debug3(x, y, z) cout << "debug: " << (#x) << ": " << (x) << ", " << (#y) << ": " << (y) << ", " << (#z) << ": " << (z) << endl
#define debug4(x, y, z, w) cout << "debug: " << (#x) << ": " << (x) << ", " << (#y) << ": " << (y) << ", " << (#z) << ": " << (z) << ", " << (#w) << ": " << (w) << endl
#define debug5(x, y, z, w, v) cout << "debug: " << (#x) << ": " << (x) << ", " << (#y) << ": " << (y) << ", " << (#z) << ": " << (z) << ", " << (#w) << ": " << (w) << ", " << (#v) << ": " << (v) << endl
#define debug6(x, y, z, w, v, u) cout << "debug: " << (#x) << ": " << (x) << ", " << (#y) << ": " << (y) << ", " << (#z) << ": " << (z) << ", " << (#w) << ": " << (w) << ", " << (#v) << ": " << (v) << ", " << (#u) << ": " << (u) << endl
#define overload6(a, b, c, d, e, f, g,...) g
#define debug(...) overload6(__VA_ARGS__, debug6, debug5, debug4, debug3, debug2, debug1)(__VA_ARGS__)
#define debuga cerr << "a" << endl
#define debugnl cout << endl
#define Case(i) cout << "Case #" << (i) << ": "
#define TIMECHECK cerr << 1000.0 * static_cast<double>(clock() - TIME_START) / CLOCKS_PER_SEC << "ms" << endl
#define LOCAL 1
#else
#define debug1(x) void(0)
#define debug2(x, y) void(0)
#define debug3(x, y, z) void(0)
#define debug4(x, y, z, w) void(0)
#define debug5(x, y, z, w, v) void(0)
#define debug6(x, y, z, w, v, u) void(0)
#define debug(...) void(0)
#define debuga void(0)
#define debugnl void(0)
#define Case(i) void(0)
#define TIMECHECK void(0)
#define LOCAL 0
#endif

//mt19937_64 rng(0);
mt19937_64 rng(chrono::steady_clock::now().time_since_epoch().count());
clock_t TIME_START;
const long double pi = 3.141592653589793238462643383279L;
const long long INFL = 1000000000000000000ll;
const long long INFLMAX = numeric_limits< long long >::max(); // 9223372036854775807
const int INF = 1000000000;
const int INFMAX = numeric_limits< int >::max(); // 2147483647
const long double INFD = numeric_limits<ld>::infinity();
const long double EPS = 1e-10;
const int mod1 = 1000000007;
const int mod2 = 998244353;
const vi dx1 = {1,0,-1,0};
const vi dy1 = {0,1,0,-1};
const vi dx2 = {0, 1, 1, 1, 0, -1, -1, -1};
const vi dy2 = {1, 1, 0, -1, -1, -1, 0, 1};
constexpr char nl = '\n';
constexpr char bl = ' ';

std::ostream &operator<<(std::ostream &dest, __int128_t value) { std::ostream::sentry s(dest); if (s) {__uint128_t tmp = value < 0 ? -value : value; char buffer[128]; char *d = std::end(buffer); do {--d;*d = "0123456789"[tmp % 10];tmp /= 10;} while (tmp != 0);if (value < 0) {--d;*d = '-';}int len = std::end(buffer) - d;if (dest.rdbuf()->sputn(d, len) != len) {dest.setstate(std::ios_base::badbit);}}return dest; }
__int128 parse(string &s) { __int128 ret = 0; for (int i = 0; i < (int)s.length(); i++) if ('0' <= s[i] && s[i] <= '9') ret = 10 * ret + s[i] - '0'; return ret; } // for __int128 (https://kenkoooo.hatenablog.com/entry/2016/11/30/163533)
template<class T> ostream& operator << (ostream& os, vector<T>& vec) { os << "["; for (int i = 0; i<(int)vec.size(); i++) { os << vec[i] << (i + 1 == (int)vec.size() ? "" : ", "); } os << "]"; return os; } /// vector 出力
template<class T, class U> ostream& operator << (ostream& os, pair<T, U>& pair_var) { os << "(" << pair_var.first << ", " << pair_var.second << ")"; return os; } // pair 出力
template<class T, class U> ostream& operator << (ostream& os, map<T, U>& map_var) { os << "{"; for (auto itr = map_var.begin(); itr != map_var.end(); itr++) { os << "(" << itr->first << ", " << itr->second << ")"; itr++; if(itr != map_var.end()) os << ", "; itr--; } os << "}"; return os; } // map出力
template<class T> ostream& operator << (ostream& os, set<T>& set_var) { os << "{"; for (auto itr = set_var.begin(); itr != set_var.end(); itr++) { os << *itr; ++itr; if(itr != set_var.end()) os << ", "; itr--; } os << "}"; return os; } /// set 出力

bool equals(long double a, long double b) { return fabsl(a - b) < EPS; }
template<class T> int sign(T a) { return equals(a, 0) ? 0 : a > 0 ? 1 : -1; }
long double radian_to_degree(long double r) { return (r * 180.0 / pi); }
long double degree_to_radian(long double d) { return (d * pi / 180.0); }

int popcnt(unsigned long long a){ return __builtin_popcountll(a); } // ll は 64bit対応！
int MSB1(unsigned long long x) { return (x == 0 ? -1 : 63 - __builtin_clzll(x)); } // MSB(1), 0-indexed ( (0, 1, 2, 3, 4) -> (-1, 0, 1, 1, 2) )
int LSB1(unsigned long long x) { return (x == 0 ? -1 : __builtin_ctzll(x)); } // LSB(1), 0-indexed ( (0, 1, 2, 3, 4) -> (-1, 0, 1, 0, 2) )
long long maskbit(int n) { return (1LL << n) - 1; }
bool bit_is1(long long x, int i) { return ((x>>i) & 1); }
string charrep(int n, char c) { return std::string(n, c); }
template<class T> T square(T x) { return (x) * (x); }
template<class T>void UNIQUE(T& A) {sort(all(A)); A.erase(unique(all(A)), A.end());}
template<class T>bool chmax(T& a, const T& b) { if (a < b) { a = b; return 1; } return 0; }
template<class T>bool chmin(T& a, const T& b) { if (b < a) { a = b; return 1; } return 0; }
template<class T, class U> bool chmin(T& a, const U& b){ return chmin(a, (T)b); }
template<class T, class U> bool chmax(T& a, const U& b){ return chmax(a, (T)b); }
void YESNO(bool b) {if(b){cout<<"YES"<<'\n';} else{cout<<"NO"<<'\n';}} void YES() {YESNO(true);} void NO() {YESNO(false);}
void yesno(bool b) {if(b){cout<<"yes"<<'\n';} else{cout<<"no"<<'\n';}} void yes() {yesno(true);} void no() {yesno(false);}
void YesNo(bool b) {if(b){cout<<"Yes"<<'\n';} else{cout<<"No"<<'\n';}} void Yes() {YesNo(true);} void No() {YesNo(false);}
void POSIMPOS(bool b) {if(b){cout<<"POSSIBLE"<<'\n';} else{cout<<"IMPOSSIBLE"<<'\n';}}
void PosImpos(bool b) {if(b){cout<<"Possible"<<'\n';} else{cout<<"Impossible"<<'\n';}}
void posimpos(bool b) {if(b){cout<<"possible"<<'\n';} else{cout<<"impossible"<<'\n';}}
void FIRSEC(bool b) {if(b){cout<<"FIRST"<<'\n';} else{cout<<"SECOND"<<'\n';}}
void firsec(bool b) {if(b){cout<<"first"<<'\n';} else{cout<<"second"<<'\n';}}
void FirSec(bool b) {if(b){cout<<"First"<<'\n';} else{cout<<"Second"<<'\n';}}
void AliBob(bool b) {if(b){cout<<"Alice"<<'\n';} else{cout<<"Bob"<<'\n';}}
void TakAok(bool b) {if(b){cout<<"Takahashi"<<'\n';} else{cout<<"Aoki"<<'\n';}}
int GetTime() {return 1000.0*static_cast<double>(clock() - TIME_START) / CLOCKS_PER_SEC;}
ll myRand(ll B) {return (unsigned long long)rng() % B;}
template<class T> void print(const T& x, const char endch = '\n') { cout << x << endch; }

template<class T> T ceil_div(T x, T y) { assert(y); return (x > 0 ? (x + y - 1) / y : x / y); }
template<class T> T floor_div(T x, T y) { assert(y); return (x > 0 ? x / y : (x - y + 1) / y); }
template<class T> pair<T, T> divmod(T x, T y) { T q = floor_div(x, y); return {q, x - q * y}; } /// (q, r) s.t. x = q*y + r 
ll GCD(ll a, ll b) { if(a < b) swap(a, b); if(b == 0) return a; if(a%b == 0) return b; else return GCD(b, a%b); }
ll LCM(ll a, ll b) { assert(GCD(a,b) != 0); return a / GCD(a, b) * b; }
ll MOD(ll &x, const ll P) { ll ret = x%P; if(ret < 0) ret += P; return x = ret; } /// x % P を非負整数に直す
ll mpow(ll x, ll n, const ll mod) { x %= mod; ll ret = 1; while(n > 0) { if(n & 1) ret = ret * x % mod; x = x * x % mod; n >>= 1; } return ret; } /// x^n % mod を計算
ll lpow(ll x, ll n) { ll ret = 1; while(n > 0){ if(n & 1) ret = ret * x; x = x * x; n >>= 1; } return ret; } /// x^nを計算
string toBinary(ll n) { if(n == 0) return "0"; assert(n > 0); string ret; while (n != 0){ ret += ( (n & 1) == 1 ? '1' : '0' ); n >>= 1; } reverse(ret.begin(), ret.end()); return ret; } /// 10進数(long long) -> 2進数(string)への変換
ll toDecimal(string S) { ll ret = 0; for(int i = 0; i < (int)S.size(); i++){ ret *= 2LL; if(S[i] == '1') ret += 1; } return ret; } /// 2進数(string)　→　10進数(long long)への変換
int ceil_pow2(ll n) { int x = 0; while ((1ll << x) < n) x++; return x;} /// return minimum non-negative `x` s.t. `n <= 2**x`
int floor_pow2(ll n) { int x = 0; while ((1ll << (x+1)) <= n) x++; return x;} /// return maximum non-negative `x` s.t. `n >= 2**x`

//typedef string::const_iterator State;


// ############################
// #                          #
// #    C O D E  S T A R T    #
// #                          #
// ############################


void solve() {
    ll N; cin >> N;
    vll A(N); rep(i,N) cin >> A[i];
    
    
}



signed main() {
    cin.tie(0); ios_base::sync_with_stdio(false);
    TIMER_START;
    //cout << fixed << setprecision(15);
    

    int tt = 1;
    //cin >> tt;
    while(tt--){
        solve();
    }

    TIMECHECK;
    return 0;
}