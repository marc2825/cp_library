#pragma once
#include "../convolution/fast_middle_product.hpp"
#include "inv.hpp"
namespace cp {
namespace projection_detail {
// Rectangular bivariate polynomials, x fastest. Packing uses a zero gap to prevent x-carries.
template <int Mod, int Root> struct Engine {
    using F = fps_detail::Field<Mod>;
    using P = std::vector<int>;
    struct Rectangle {
        std::size_t x, y;
        P a;
        Rectangle(std::size_t nx, std::size_t ny) : x(nx), y(ny), a(nx * ny) {}
    };
    struct Step {
        Rectangle negative;
        std::size_t px, py, oy;
        int parity;
    };
    std::vector<Step> steps;
    P inverse;
    std::size_t n, count, final_rows;
    static P pack(const Rectangle &a, std::size_t stride) {
        P p((a.y - 1) * stride + a.x);
        for (std::size_t j = 0; j < a.y; ++j)
            std::copy_n(a.a.begin() + j * a.x, a.x, p.begin() + j * stride);
        return p;
    }
    static Rectangle descend(const Rectangle &a, const Rectangle &b, std::size_t oy, int parity) {
        const auto stride = 2 * a.x - 1;
        auto c = fps_detail::multiply<Mod, Root>(pack(a, stride), pack(b, stride));
        Rectangle out((a.x + 1 - parity) / 2, oy);
        for (std::size_t j = 0; j < oy; ++j)
            for (std::size_t i = 0; i < out.x; ++i)
                out.a[j * out.x + i] = c[j * stride + 2 * i + parity];
        return out;
    }
    Engine(const P &g, std::size_t length, std::size_t output_count)
        : n(length), count(output_count), final_rows(1) {
        if (!n || !count)
            return;
        fps_detail::check_precision<Mod, Root>(std::max(n, count));
        Rectangle q(n, std::min<std::size_t>(2, count));
        q.a[0] = 1;
        if (q.y == 2)
            for (std::size_t i = 0; i < std::min(n, g.size()); ++i)
                q.a[n + i] = F::sub(0, F::norm(g[i]));
        std::size_t target = n - 1, py = 1;
        while (target) {
            Rectangle neg = q;
            for (std::size_t j = 0; j < q.y; ++j)
                for (std::size_t i = 1; i < q.x; i += 2)
                    neg.a[j * q.x + i] = F::sub(0, neg.a[j * q.x + i]);
            auto oy = std::min(count, py + q.y - 1);
            steps.push_back({neg, q.x, py, oy, static_cast<int>(target & 1)});
            // Numerator x extent is floor(target/2)+1; denominator is truncated to the same extent.
            auto next = descend(q, neg, std::min(count, 2 * q.y - 1), 0);
            target /= 2;
            Rectangle reduced(target + 1, next.y);
            for (std::size_t j = 0; j < next.y; ++j)
                std::copy_n(next.a.begin() + j * next.x, reduced.x, reduced.a.begin() + j * reduced.x);
            q = std::move(reduced);
            py = oy;
        }
        final_rows = py;
        P den(q.y);
        for (std::size_t j = 0; j < q.y; ++j)
            den[j] = q.a[j];
        inverse = fps_inv<Mod, Root>(den, count);
    }
    P project(const P &weights) const {
        if (!count)
            return {};
        if (!n)
            return P(count);
        Rectangle p(n, 1);
        for (std::size_t i = 0; i < n; ++i)
            p.a[n - 1 - i] = F::norm(weights[i]);
        for (const auto &s : steps)
            p = descend(p, s.negative, s.oy, s.parity);
        return fps_detail::product_prefix<Mod, Root>(p.a, inverse, count);
    }
    P compose(const P &outer) const {
        if (!n)
            return {};
        if (!count)
            return P(n);
        P w(count + inverse.size() - 1);
        for (std::size_t i = 0; i < std::min(count, outer.size()); ++i)
            w[i] = F::norm(outer[i]);
        auto last = fast_middle_product<Mod, Root>(w, inverse);
        last.resize(final_rows);
        Rectangle adj(1, final_rows);
        adj.a = std::move(last);
        for (std::size_t k = steps.size(); k-- > 0;) {
            const auto &s = steps[k];
            auto stride = 2 * s.px - 1;
            auto kernel = pack(s.negative, stride);
            auto input_len = (s.py - 1) * stride + s.px;
            P lifted(input_len + kernel.size() - 1);
            for (std::size_t j = 0; j < adj.y; ++j)
                for (std::size_t i = 0; i < adj.x; ++i)
                    lifted[j * stride + 2 * i + s.parity] = adj.a[j * adj.x + i];
            auto grad = fast_middle_product<Mod, Root>(lifted, kernel);
            Rectangle previous(s.px, s.py);
            for (std::size_t j = 0; j < s.py; ++j)
                std::copy_n(grad.begin() + j * stride, s.px, previous.a.begin() + j * s.px);
            adj = std::move(previous);
        }
        std::reverse(adj.a.begin(), adj.a.end());
        return adj.a;
    }
};
} // namespace projection_detail
// output[k] = sum_i weights[i] * [x^i] g(x)^k, k<count. O(M(n) log n + M(count)).
template <int Mod = 998244353, int Root = 3>
std::vector<int> power_projection(const std::vector<int> &weights, const std::vector<int> &g,
                                  std::size_t count) {
    return projection_detail::Engine<Mod, Root>(g, weights.size(), count).project(weights);
}
} // namespace cp
