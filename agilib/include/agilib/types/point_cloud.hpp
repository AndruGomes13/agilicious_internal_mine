#pragma once

#include <vector>
#include <memory>
#include "agilib/types/point.hpp"
#include "agilib/math/types.hpp"


namespace agi {

    struct PointCloud {
        Scalar t;  // Timestamp of the point cloud
        std::vector<Point> points;  // Vector of points representing the point cloud
        size_t size() const { return points.size(); }
    };

}  // namespace agi
