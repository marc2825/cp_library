#include "test_common.hpp"
#include "../../string_algorithm/all.hpp"
#include <random>
#include <set>
#include <string>
#include <tuple>
std::vector<cp::StringRun> brute_runs(const std::string&s){std::set<std::tuple<int,int,int>>runs;const int n=int(s.size());for(int p=1;p<=n/2;++p)for(int l=0;l+2*p<=n;++l){int r=l+p;while(r<n&&s[r]==s[r-p])++r;if(r-l<2*p||(l&&s[l-1]==s[l-1+p]))continue;bool minimum=true;for(int q=1;q<p;++q){bool valid=true;for(int i=l+q;i<r;++i)if(s[i]!=s[i-q])valid=false;if(valid)minimum=false;}if(minimum)runs.emplace(p,l,r);}std::vector<cp::StringRun>out;for(auto[p,l,r]:runs)out.push_back({p,l,r});return out;}
bool palindrome(const std::string&s,int l,int r){for(--r;l<r;++l,--r)if(s[l]!=s[r])return false;return true;}
int main(){
 for(int n=0;n<=12;++n)for(int code=0;code<(1<<n);++code){std::string s(n,'a');for(int i=0;i<n;++i)if(code&(1<<i))s[i]='b';CHECK(cp::runs_enumeration(s)==brute_runs(s));auto partition=cp::palindrome_partition(s);std::vector<int>minimum(n+1,n);minimum[0]=0;for(int r=1;r<=n;++r)for(int l=0;l<r;++l)if(palindrome(s,l,r))minimum[r]=std::min(minimum[r],minimum[l]+1);CHECK(partition.prefix_minimum==minimum);if(partition.intervals.size()!=std::size_t(minimum[n])){std::cerr<<"partition case "<<s<<" expected "<<minimum[n]<<" got "<<partition.intervals.size()<<"\n";for(auto[l,r]:partition.intervals)std::cerr<<l<<","<<r<<" ";std::cerr<<"\n";}CHECK(partition.intervals.size()==std::size_t(minimum[n]));int end=0;for(auto[l,r]:partition.intervals){CHECK(l==end&&palindrome(s,l,r));end=r;}CHECK(end==n);
  int rotation=0;for(int i=1;i<n;++i)if(s.substr(i)+s.substr(0,i)<s.substr(rotation)+s.substr(0,rotation))rotation=i;CHECK(cp::minimum_rotation(s)==rotation);
  if(n<=7){cp::SuffixTree tree(s);CHECK(tree.nodes().size()<=2*s.size()+1);for(int l=0;l<=n;++l)for(int r=l;r<=n;++r){auto pattern=s.substr(l,r-l);auto expected=cp::kmp_search(s,pattern);CHECK(tree.contains(pattern)&&tree.occurrences(pattern)==expected);}CHECK(!tree.contains("c"));for(std::size_t v=1;v<tree.nodes().size();++v){const auto&node=tree.nodes()[v];CHECK(node.begin>=0&&node.end<=n&&node.begin<node.end);CHECK(node.end-node.begin==node.depth-tree.nodes()[node.parent].depth);}}
 }
 std::mt19937 random(3212026);
 for(int trial=0;trial<250;++trial){std::string s(1+random()%150,' '),p(random()%40,' ');for(auto&c:s)c=char(random()%5?random()%256:'?');for(auto&c:p)c=char(random()%4?random()%256:'?');std::vector<int>expected;if(p.size()<=s.size())for(std::size_t i=0;i<=s.size()-p.size();++i){bool match=true;for(std::size_t j=0;j<p.size();++j)if(s[i+j]!='?'&&p[j]!='?'&&s[i+j]!=p[j])match=false;if(match)expected.push_back(int(i));}CHECK(cp::wildcard_pattern_matching(s,p)==expected);
  cp::SuffixTree tree(s);for(int q=0;q<20;++q){std::size_t l=random()%(s.size()+1),r=random()%(s.size()+1);if(l>r)std::swap(l,r);auto pattern=s.substr(l,r-l);CHECK(tree.occurrences(pattern)==cp::kmp_search(s,pattern));}
 }
 cp::DynamicRollingHash hash({},257);std::string text;
 for(int step=0;step<7000;++step){int op=random()%5;if(text.empty()||op==0){int i=random()%(text.size()+1);unsigned char c=random()%256;hash.insert(i,c);text.insert(text.begin()+i,char(c));}else if(op==1){int i=random()%text.size();hash.erase(i);text.erase(text.begin()+i);}else if(op==2){int i=random()%text.size();unsigned char c=random()%256;hash.set(i,c);text[i]=char(c);}else{int l=random()%(text.size()+1),r=random()%(text.size()+1);if(l>r)std::swap(l,r);hash.reverse(l,r);std::reverse(text.begin()+l,text.begin()+r);}CHECK(hash.str()==text);cp::RollingHash static_hash(text,257);for(int q=0;q<4;++q){int l=random()%(text.size()+1),r=random()%(text.size()+1);if(l>r)std::swap(l,r);CHECK(hash.get(l,r)==static_hash.get(l,r));}cp::DynamicRollingHash copy(text,257);CHECK(hash.lcp(copy,0,hash.size(),0,copy.size())==hash.size());CHECK(hash.equal(copy,0,hash.size(),0,copy.size()));}
 std::vector<std::string> strings{"",std::string("a\0b",3),"ababa",std::string("\xff\0",2),"banana"};cp::MultiStringCompare compare(strings);
 for(int a=0;a<int(strings.size());++a)for(int b=0;b<int(strings.size());++b)for(int l=0;l<=int(strings[a].size());++l)for(int r=l;r<=int(strings[a].size());++r)for(int x=0;x<=int(strings[b].size());++x)for(int y=x;y<=int(strings[b].size());++y){int common=0;while(common<std::min(r-l,y-x)&&strings[a][l+common]==strings[b][x+common])++common;CHECK(compare.lcp(a,l,r,b,x,y)==common);const auto one=strings[a].substr(l,r-l),two=strings[b].substr(x,y-x);const auto less=[](const auto&u,const auto&v){return std::lexicographical_compare(u.begin(),u.end(),v.begin(),v.end(),[](char i,char j){return static_cast<unsigned char>(i)<static_cast<unsigned char>(j);});};CHECK(compare.compare(a,l,r,b,x,y)==(less(two,one)?1:less(one,two)?-1:0));}
 CHECK(cp::runs_enumeration(std::string(30000,'a'))==std::vector<cp::StringRun>{{1,0,30000}});
 const auto partition=cp::palindrome_partition(std::string(100000,'a'));CHECK(partition.prefix_minimum.back()==1&&partition.intervals.size()==1);
 hash.clear();CHECK(hash.size()==0&&hash.get(0,0)==cp::RollingHash::Hash({0,0}));
 rejects<std::out_of_range>([&]{hash.erase(0);});rejects<std::out_of_range>([&]{compare.compare(-1,0,0,0,0,0);});
 std::cout<<"B strings: all binary runs/palindrome partitions/rotations, compressed suffixes, 7000 dynamic edits, arbitrary bytes and exact multi-string comparisons passed\n";
}
