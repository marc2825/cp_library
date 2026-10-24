#pragma once
#include <cassert>
#include <memory>
#include <utility>
#include <vector>

namespace cp {
// 計測付き永続2–3 Finger Tree。非可換monoid、両端・split/concat・区間積。
// strict評価なので両端操作も最悪O(log n)。コピーはO(1)。
template<class S,auto op,auto e> class FingerTree {
    struct Item;
    using ItemPtr=std::shared_ptr<const Item>;
    using Digit=std::vector<ItemPtr>;
    struct Item { S measure; int count; Digit children; };
    struct Tree;
    using Ptr=std::shared_ptr<const Tree>;
    struct Tree { S measure; int count; ItemPtr single; Digit prefix,suffix; Ptr middle; };
    Ptr root_;
    static int length(const Ptr& p) { return p?p->count:0; }
    static S product(const Ptr& p) { return p?p->measure:e(); }
    static int length(const Digit& d) { int n=0; for(const auto& p:d) n+=p->count; return n; }
    static S product(const Digit& d) { S value=e(); for(const auto& p:d) value=op(value,p->measure); return value; }
    static ItemPtr leaf(const S& x) { return std::make_shared<Item>(Item{x,1,{}}); }
    static ItemPtr node(Digit children) {
        assert(children.size()==2 || children.size()==3);
        auto result=std::make_shared<Item>(Item{product(children),length(children),{}});
        result->children=std::move(children); return result;
    }
    static Ptr single(ItemPtr p) { return std::make_shared<Tree>(Tree{p->measure,p->count,p,{},{},{}}); }
    static Ptr deep(Digit prefix,Ptr middle,Digit suffix) {
        assert(!prefix.empty() && prefix.size()<=4 && !suffix.empty() && suffix.size()<=4);
        auto result=std::make_shared<Tree>(Tree{op(op(product(prefix),product(middle)),product(suffix)),
            length(prefix)+length(middle)+length(suffix),{},std::move(prefix),std::move(suffix),std::move(middle)});
        return result;
    }
    static Ptr prepend(ItemPtr x,const Ptr& p) {
        if(!p) return single(x);
        if(p->single) return deep({x},{},{p->single});
        auto d=p->prefix;
        if(d.size()<4) { d.insert(d.begin(),x); return deep(std::move(d),p->middle,p->suffix); }
        return deep({x,d[0]},prepend(node({d[1],d[2],d[3]}),p->middle),p->suffix);
    }
    static Ptr append(const Ptr& p,ItemPtr x) {
        if(!p) return single(x);
        if(p->single) return deep({p->single},{},{x});
        auto d=p->suffix;
        if(d.size()<4) { d.push_back(x); return deep(p->prefix,p->middle,std::move(d)); }
        return deep(p->prefix,append(p->middle,node({d[0],d[1],d[2]})),{d[3],x});
    }
    static Ptr from_digit(const Digit& d) { Ptr p; for(const auto& x:d) p=append(p,x); return p; }
    static Ptr deep_left(Digit prefix,const Ptr& middle,const Digit& suffix) {
        if(!prefix.empty()) return deep(std::move(prefix),middle,suffix);
        if(!middle) return from_digit(suffix);
        auto [x,rest]=remove_front(middle); return deep(x->children,rest,suffix);
    }
    static Ptr deep_right(const Digit& prefix,const Ptr& middle,Digit suffix) {
        if(!suffix.empty()) return deep(prefix,middle,std::move(suffix));
        if(!middle) return from_digit(prefix);
        auto [rest,x]=remove_back(middle); return deep(prefix,rest,x->children);
    }
    static std::pair<ItemPtr,Ptr> remove_front(const Ptr& p) {
        assert(p);
        if(p->single) return {p->single,{}};
        Digit d=p->prefix; auto x=d.front(); d.erase(d.begin());
        return {x,deep_left(std::move(d),p->middle,p->suffix)};
    }
    static std::pair<Ptr,ItemPtr> remove_back(const Ptr& p) {
        assert(p);
        if(p->single) return {{},p->single};
        Digit d=p->suffix; auto x=d.back(); d.pop_back();
        return {deep_right(p->prefix,p->middle,std::move(d)),x};
    }
    static Ptr join(const Ptr& a,const Digit& between,const Ptr& b) {
        if(!a) { Ptr result=b; for(auto it=between.rbegin();it!=between.rend();++it) result=prepend(*it,result); return result; }
        if(!b) { Ptr result=a; for(const auto& x:between) result=append(result,x); return result; }
        if(a->single) return prepend(a->single,join({},between,b));
        if(b->single) return append(join(a,between,{}),b->single);
        Digit combined=a->suffix,packed;
        combined.insert(combined.end(),between.begin(),between.end());
        combined.insert(combined.end(),b->prefix.begin(),b->prefix.end());
        for(std::size_t i=0;i<combined.size();) {
            std::size_t n=combined.size()-i,group=(n==2 || n==4)?2:3;
            packed.push_back(node(Digit(combined.begin()+i,combined.begin()+i+group))); i+=group;
        }
        return deep(a->prefix,join(a->middle,packed,b->middle),b->suffix);
    }
    struct DigitSplit {Digit left;ItemPtr pivot;Digit right;};
    static DigitSplit split_digit(const Digit& d,int p) {
        for(std::size_t i=0;i<d.size();++i) {
            if(p<d[i]->count) return {Digit(d.begin(),d.begin()+i),d[i],Digit(d.begin()+i+1,d.end())};
            p-=d[i]->count;
        }
        assert(false); return {};
    }
    struct TreeSplit {Ptr left;ItemPtr pivot;Ptr right;};
    static TreeSplit cut(const Ptr& t,int p) {
        assert(t && 0<=p && p<length(t));
        if(t->single) return {{},t->single,{}};
        int first=length(t->prefix),middle=length(t->middle);
        if(p<first) {
            auto d=split_digit(t->prefix,p);
            return {from_digit(d.left),d.pivot,deep_left(d.right,t->middle,t->suffix)};
        }
        if(p<first+middle) {
            auto m=cut(t->middle,p-first);
            auto d=split_digit(m.pivot->children,p-first-length(m.left));
            return {deep_right(t->prefix,m.left,d.left),d.pivot,deep_left(d.right,m.right,t->suffix)};
        }
        auto d=split_digit(t->suffix,p-first-middle);
        return {deep_right(t->prefix,t->middle,d.left),d.pivot,from_digit(d.right)};
    }
    static S item_at(const ItemPtr& p,int index) {
        if(p->children.empty()) return p->measure;
        for(const auto& q:p->children) { if(index<q->count) return item_at(q,index); index-=q->count; }
        assert(false); return e();
    }
    static S at(const Ptr& p,int index) {
        if(p->single) return item_at(p->single,index);
        for(const auto& q:p->prefix) { if(index<q->count) return item_at(q,index); index-=q->count; }
        if(index<length(p->middle)) return at(p->middle,index);
        index-=length(p->middle);
        for(const auto& q:p->suffix) { if(index<q->count) return item_at(q,index); index-=q->count; }
        assert(false); return e();
    }
    static void collect_item(const ItemPtr& p,std::vector<S>& out) {
        if(p->children.empty()) out.push_back(p->measure);
        else for(const auto& q:p->children) collect_item(q,out);
    }
    static void collect(const Ptr& p,std::vector<S>& out) {
        if(!p) return;
        if(p->single) { collect_item(p->single,out); return; }
        for(const auto& q:p->prefix) collect_item(q,out);
        collect(p->middle,out);
        for(const auto& q:p->suffix) collect_item(q,out);
    }
public:
    FingerTree()=default;
    explicit FingerTree(const std::vector<S>& values) { for(const auto& x:values) push_back(x); }
    int size() const { return length(root_); }
    bool empty() const { return !root_; }
    void clear() { root_.reset(); }
    S all_prod() const { return product(root_); }
    void push_front(const S& x) { root_=prepend(leaf(x),root_); }
    void push_back(const S& x) { root_=append(root_,leaf(x)); }
    S front() const { assert(root_); return root_->single?root_->single->measure:root_->prefix.front()->measure; }
    S back() const { assert(root_); return root_->single?root_->single->measure:root_->suffix.back()->measure; }
    S pop_front() { auto [x,p]=remove_front(root_); root_=p; return x->measure; }
    S pop_back() { auto [p,x]=remove_back(root_); root_=p; return x->measure; }
    S get(int p) const { assert(0<=p && p<size()); return at(root_,p); }
    std::pair<FingerTree,FingerTree> split(int p) const {
        assert(0<=p && p<=size());
        if(p==size()) return {*this,{}};
        auto s=cut(root_,p); FingerTree a,b; a.root_=s.left; b.root_=prepend(s.pivot,s.right); return {a,b};
    }
    FingerTree concat(const FingerTree& other) const { FingerTree result; result.root_=join(root_,{},other.root_); return result; }
    S prod(int l,int r) const {
        assert(0<=l && l<=r && r<=size()); if(l==r) return e();
        return split(l).second.split(r-l).first.all_prod();
    }
    std::vector<S> to_vector() const { std::vector<S> result; result.reserve(size()); collect(root_,result); return result; }
};
} // namespace cp
