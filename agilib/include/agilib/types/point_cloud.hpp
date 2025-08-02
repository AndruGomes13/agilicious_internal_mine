#pragma once

#include <vector>
#include <memory>
#include "agilib/types/pose.hpp"
#include "agilib/math/types.hpp"


namespace agi {

    struct PointCloud {
        Scalar t;  // Timestamp of the point cloud
        std::vector<Pose> poses;  // Vector of poses representing the point cloud
        size_t size() const { return poses.size(); }
    };

}  // namespace agi
