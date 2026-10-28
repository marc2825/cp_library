#include "test_common.hpp"
using namespace htest;
int main() {
    int cases;
    std::cin >> cases;
    while (cases--) {
        char type;
        std::cin >> type;
        if (type == 'P') {
            e::Point p[4];
            for (auto &q : p)
                std::cin >> q.x >> q.y;
            std::cout << robust::orientation(p[0], p[1], p[2]) << ' '
                      << robust::incircle(p[0], p[1], p[2], p[3]) << '\n';
        } else if (type == 'F') {
            f::Point p[4];
            for (auto &q : p) {
                std::string x, y;
                std::cin >> x >> y;
                q = {std::stold(x), std::stold(y)};
            }
            std::cout << robust::orientation(p[0], p[1], p[2]) << ' '
                      << robust::incircle(p[0], p[1], p[2], p[3]) << '\n';
        } else if (type == 'R') {
            int n;
            std::cin >> n;
            std::vector<e::Rectangle> rs(n);
            for (auto &r : rs)
                std::cin >> r.x1 >> r.y1 >> r.x2 >> r.y2;
            auto m = e::rectangle_union_measure(rs);
            std::cout << m.area << ' ' << m.perimeter << '\n';
        } else if (type == 'L' || type == 'C') {
            int n;
            std::cin >> n;
            std::vector<r::Point> points(n);
            for (auto &p : points)
                std::cin >> p.x >> p.y;
            if (type == 'L')
                std::cout << r::count_lattice_points(points) << '\n';
            else {
                auto c = r::polygon_centroid(points);
                if (c)
                    std::cout << c->x << ' ' << c->y << '\n';
                else
                    std::cout << "none\n";
            }
        } else if (type == 'A') {
            int n;
            std::cin >> n;
            std::vector<r::Segment> segments(n);
            for (auto &s : segments)
                std::cin >> s.a.x >> s.a.y >> s.b.x >> s.b.y;
            auto a = r::segment_arrangement(segments);
            std::cout << a.vertices.size() << ' ' << a.edges.size();
            for (const auto &p : a.vertices)
                std::cout << ' ' << p.x << ' ' << p.y;
            std::cout << '\n';
        } else
            throw std::invalid_argument("unknown query type");
    }
}
