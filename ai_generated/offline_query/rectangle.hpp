#pragma once

// Coordinates need a strict total order via <. Rectangles are [left,right)x[down,up).
namespace cp {

template <class Coord = long long, class Weight = long long> struct WeightedPoint {
    Coord x, y;
    Weight weight;
};

template <class Coord = long long> struct Rectangle {
    Coord left, down, right, up;
    bool empty() const { return !(left < right) || !(down < up); }
};

}  // namespace cp
