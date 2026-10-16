#pragma once
namespace cp { namespace geometry {
enum class Ccw { clockwise=-1, on_segment=0, counter_clockwise=1, online_back=2, online_front=-2 };
enum class Location { outside=0, boundary=1, inside=2 };
enum class LineRelation { unique, parallel, coincident };
} } // namespace cp::geometry
