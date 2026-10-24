#pragma once

#include <memory>
#include <stdexcept>
#include <utility>
#include <vector>

// Adapted from ei1333/library (Unlicense), commit
// 088b4c8b49f85194099305d998e7113dc3d5dd2e.
// Source: https://github.com/ei1333/library/blob/088b4c8b49f85194099305d998e7113dc3d5dd2e/structure/dynamic-tree/lazy-top-tree.hpp
// License: verification/licenses/Unlicense.txt. Changes: owned/recycled nodes,
// top-down splay push, optional lazy policy, checked index facade.
namespace cp { namespace detail {
struct TopTreeNoLazy {
  static TopTreeNoLazy id() { return {}; }
  void propagate(const TopTreeNoLazy&) {}
};
template<class Spec, bool Enabled> struct TopTreeLazyTag { using type = TopTreeNoLazy; };
template<class Spec> struct TopTreeLazyTag<Spec, true> { using type = typename Spec::Lazy; };

template <typename TreeDPInfo, bool LazyEnabled>
struct TopTreeRakeSplay {
  using Lazy = typename TopTreeLazyTag<TreeDPInfo, LazyEnabled>::type;
  using Point = typename TreeDPInfo::Point;
  struct Node {
    Node *l, *r, *p;
    Point key, sum;
    Lazy lazy, lbuf;

    explicit Node(const Point& key)
        : l(nullptr), r(nullptr), p(nullptr), key(key), sum(key),
          lazy(Lazy::id()), lbuf(Lazy::id()) {}
  };

  TopTreeRakeSplay() = default;

  using NP = Node*;

  void rotr(NP t) const {
    NP x = t->p, y = x->p;
    push(x), push(t);
    if ((x->l = t->r)) t->r->p = x;
    t->r = x, x->p = t;
    update(x), update(t);
    if ((t->p = y)) {
      if (y->l == x) y->l = t;
      if (y->r == x) y->r = t;
    }
  }

  void rotl(NP t) const {
    NP x = t->p, y = x->p;
    push(x), push(t);
    if ((x->r = t->l)) t->l->p = x;
    t->l = x, x->p = t;
    update(x), update(t);
    if ((t->p = y)) {
      if (y->l == x) y->l = t;
      if (y->r == x) y->r = t;
    }
  }

  void update(NP t) const {
    t->sum = t->key;
    if (t->l) t->sum = TreeDPInfo::rake(t->sum, t->l->sum);
    if (t->r) t->sum = TreeDPInfo::rake(t->sum, t->r->sum);
  }

  NP get_right(NP t) const {
    while (t->r) t = t->r;
    return t;
  }

  std::vector<std::unique_ptr<Node>> storage;
  std::vector<NP> spare;
  NP alloc(const Point& v) {
    if (spare.empty()) {
      storage.push_back(std::make_unique<Node>(v));
      return storage.back().get();
    }
    NP t = spare.back(); spare.pop_back();
    *t = Node(v);
    return t;
  }

  void propagate(NP t, const Lazy& lazy) const {
    if constexpr (LazyEnabled) {
      t->key.propagate(lazy);
      t->sum.propagate(lazy);
    }
    t->lbuf.propagate(lazy);
    t->lazy.propagate(lazy);
  }

  void push(NP t) const {
    if (t->l) propagate(t->l, t->lazy);
    if (t->r) propagate(t->r, t->lazy);
    t->lazy = Lazy::id();
  }

  void splay(NP t) const {
    std::vector<NP> ancestors;
    for (NP v = t; v; v = v->p) ancestors.push_back(v);
    for (auto it = ancestors.rbegin(); it != ancestors.rend(); ++it) push(*it);
    while (t->p) {
      NP q = t->p;
      if (!q->p) {
        if (q->l == t)
          rotr(t);
        else
          rotl(t);
      } else {
        NP r = q->p;
        if (r->l == q) {
          if (q->l == t)
            rotr(q), rotr(t);
          else
            rotl(t), rotr(t);
        } else {
          if (q->r == t)
            rotl(q), rotl(t);
          else
            rotr(t), rotl(t);
        }
      }
    }
  }

  NP insert(NP t, const Point& v) {
    if (not t) {
      t = alloc(v);
      return t;
    } else {
      NP cur = get_right(t), z = alloc(v);
      splay(cur);
      z->p = cur;
      cur->r = z;
      update(cur);
      splay(z);
      return z;
    }
  }

  NP erase(NP t) {
    splay(t);
    NP x = t->l, y = t->r;
    spare.push_back(t);
    if (not x) {
      t = y;
      if (t) t->p = nullptr;
    } else if (not y) {
      t = x;
      t->p = nullptr;
    } else {
      x->p = nullptr;
      t = get_right(x);
      splay(t);
      t->r = y;
      y->p = t;
      update(t);
    }
    return t;
  }
};

template <typename TreeDPInfo, bool LazyEnabled>
struct TopTreeCore {
  using Lazy = typename TopTreeLazyTag<TreeDPInfo, LazyEnabled>::type;
  using Path = typename TreeDPInfo::Path;
  using Info = typename TreeDPInfo::Info;

 private:
  struct Node {
    Node *l, *r, *p;
    Info info;
    Path sum, mus;
    typename TopTreeRakeSplay<TreeDPInfo, LazyEnabled>::Node *light, *belong;
    bool rev;
    Lazy hlazy, llazy;

    bool is_root() const { return not p or (p->l != this and p->r != this); }

    explicit Node(const Info& info)
        : l(nullptr), r(nullptr), p(nullptr), info(info),
          sum(TreeDPInfo::vertex(info)), mus(sum), light(nullptr), belong(nullptr),
          rev(false), hlazy(Lazy::id()), llazy(Lazy::id()) {}
  };

 public:
  using NP = Node*;
  TopTreeRakeSplay<TreeDPInfo, LazyEnabled> splay_tree;

 private:
  void toggle(NP t) {
    std::swap(t->l, t->r);
    std::swap(t->sum, t->mus);
    t->rev ^= true;
  }

  void rotr(NP t) {
    NP x = t->p, y = x->p;
    push(x), push(t);
    if ((x->l = t->r)) t->r->p = x;
    t->r = x, x->p = t;
    update(x), update(t);
    if ((t->p = y)) {
      if (y->l == x) y->l = t;
      if (y->r == x) y->r = t;
    }
  }

  void rotl(NP t) {
    NP x = t->p, y = x->p;
    push(x), push(t);
    if ((x->r = t->l)) t->l->p = x;
    t->l = x, x->p = t;
    update(x), update(t);
    if ((t->p = y)) {
      if (y->l == x) y->l = t;
      if (y->r == x) y->r = t;
    }
  }

  void propagate_heavy(NP t, const Lazy& hlazy) {
    if constexpr (LazyEnabled) {
    t->hlazy.propagate(hlazy);
    t->info.propagate(hlazy);
    t->sum.propagate(hlazy);
    t->mus.propagate(hlazy);
    }
  }

  void propagate_light(NP t, const Lazy& llazy) {
    if constexpr (LazyEnabled) {
    t->llazy.propagate(llazy);
    t->sum.propagate_light(llazy);
    t->mus.propagate_light(llazy);
    }
  }

  void propagate_all(NP t, const Lazy& lazy) {
    propagate_heavy(t, lazy);
    propagate_light(t, lazy);
  }

 public:
  TopTreeCore() : splay_tree{} {}

  void push(NP t) {
    if (t->rev) {
      if (t->l) toggle(t->l);
      if (t->r) toggle(t->r);
      t->rev = false;
    }
    {
      if (t->l) {
        propagate_heavy(t->l, t->hlazy);
        propagate_light(t->l, t->llazy);
      }
      if (t->r) {
        propagate_heavy(t->r, t->hlazy);
        propagate_light(t->r, t->llazy);
      }
      if (t->light) {
        splay_tree.propagate(t->light, t->llazy);
      }
      t->hlazy = Lazy::id();
      t->llazy = Lazy::id();
    }
  }

  void push_rev(NP t) {
    if (t->rev) {
      if (t->l) toggle(t->l);
      if (t->r) toggle(t->r);
      t->rev = false;
    }
  }

  void update(NP t) {
    Path key = t->light ? TreeDPInfo::add_vertex(t->light->sum, t->info)
                        : TreeDPInfo::vertex(t->info);
    t->sum = key;
    t->mus = key;
    if (t->l) {
      t->sum = TreeDPInfo::compress(t->l->sum, t->sum);
      t->mus = TreeDPInfo::compress(t->mus, t->l->mus);
    }
    if (t->r) {
      t->sum = TreeDPInfo::compress(t->sum, t->r->sum);
      t->mus = TreeDPInfo::compress(t->r->mus, t->mus);
    }
  }

  void splay(NP t) {
    std::vector<NP> ancestors{t};
    for (NP v = t; !v->is_root();) { v = v->p; ancestors.push_back(v); }
    for (auto it = ancestors.rbegin(); it != ancestors.rend(); ++it) push(*it);
    NP rot = ancestors.back();
    t->belong = rot->belong;
    if (t != rot) rot->belong = nullptr;
    while (not t->is_root()) {
      NP q = t->p;
      if (q->is_root()) {
        push_rev(q), push_rev(t);
        if (q->l == t)
          rotr(t);
        else
          rotl(t);
      } else {
        NP r = q->p;
        push_rev(r), push_rev(q), push_rev(t);
        if (r->l == q) {
          if (q->l == t)
            rotr(q), rotr(t);
          else
            rotl(t), rotr(t);
        } else {
          if (q->r == t)
            rotl(q), rotl(t);
          else
            rotr(t), rotl(t);
        }
      }
    }
  }

  NP expose(NP t) {
    NP rp = nullptr;
    for (NP cur = t; cur; cur = cur->p) {
      splay(cur);
      if (cur->r) {
        cur->light =
            splay_tree.insert(cur->light, TreeDPInfo::add_edge(cur->r->sum));
        cur->r->belong = cur->light;
      }
      cur->r = rp;
      if (cur->r) {
        splay_tree.splay(cur->r->belong);
        propagate_all(cur->r, cur->r->belong->lbuf);
        push(cur->r);
        cur->light = splay_tree.erase(cur->r->belong);
        cur->r->belong = nullptr;
      }
      update(cur);
      rp = cur;
    }
    splay(t);
    return rp;
  }

  void link(NP child, NP parent) {
    if (is_connected(child, parent)) {
      throw std::runtime_error(
          "child and parent must be different connected components");
    }
    if (child->l) {
      throw std::runtime_error("child must be root");
    }
    child->p = parent;
    parent->r = child;
    update(parent);
  }

  void cut(NP child) {
    expose(child);
    NP parent = child->l;
    if (not parent) {
      throw std::runtime_error("child must not be root");
    }
    child->l = nullptr;
    parent->p = nullptr;
    update(child);
  }

  void evert(NP t) {
    expose(t);
    toggle(t);
    push(t);
  }

  std::vector<std::unique_ptr<Node>> storage;
  NP alloc(const Info& v) {
    storage.push_back(std::make_unique<Node>(v));
    NP t = storage.back().get();
    update(t);
    return t;
  }

  bool is_connected(NP u, NP v) {
    expose(u), expose(v);
    return u == v or u->p;
  }

  std::vector<NP> build(const std::vector<Info>& vs) {
    std::vector<NP> nodes(vs.size());
    for (int i = 0; i < (int)vs.size(); i++) {
      nodes[i] = alloc(vs[i]);
    }
    return nodes;
  }

  NP lca(NP u, NP v) {
    if (not is_connected(u, v)) return nullptr;
    expose(u);
    return expose(v);
  }

  void set_key(NP t, const Info& v) {
    expose(t);
    t->info = std::move(v);
    update(t);
  }

  void set_propagate_path(NP t, const Lazy& lazy) {
    expose(t);
    propagate_heavy(t, lazy);
    push(t);
    update(t);
  }

  void set_propagate_path(NP u, NP v, const Lazy& lazy) {
    evert(u);
    set_propagate_path(v, lazy);
  }

  void set_propagate_all(NP t, const Lazy& lazy) {
    expose(t);
    propagate_all(t, lazy);
    push(t);
    update(t);
  }

  void set_propagate_subtree(NP t, const Lazy& lazy) {
    expose(t);
    NP l = t->l;
    t->l = nullptr;
    propagate_all(t, lazy);
    push(t);
    t->l = l;
    update(t);
  }

  void set_propagate_subtree(NP r, NP u, const Lazy& lazy) {
    evert(r);
    set_propagate_subtree(u, lazy);
  }

  const Path& query(NP u) {
    evert(u);
    return u->sum;
  }

  const Path& query_path(NP u) {
    expose(u);
    return u->sum;
  }

  const Path& query_path(NP u, NP v) {
    evert(u);
    return query_path(v);
  }

  Path query_subtree(NP u) {
    expose(u);
    NP l = u->l;
    u->l = nullptr;
    update(u);
    auto ret = u->sum;
    u->l = l;
    update(u);
    return ret;
  }

  Path query_subtree(NP r, NP u) {
    evert(r);
    return query_subtree(u);
  }
};

} } // namespace cp::detail
