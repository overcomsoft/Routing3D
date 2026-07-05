#pragma once
#include "geometry.hpp"
#include <vector>
#include <string>
#include <algorithm>
#include <cmath>

namespace routing3d {

// Point3D is alias for Vec3 to remain consistent with routing3d's architecture
using Point3D = Vec3;

struct DesignFeaturePoints {
    std::vector<double> frequent_z_levels;       
    std::vector<AABB> frequent_bend_zones;       
};

struct RoutingConfig {
    int max_vertical_bends = 5;                  // Default max vertical bends
    double safety_margin = 50.0;
    double tray_width = 600.0;
    double tray_height = 300.0;
    double pipe_pitch = 100.0;                   // Pitch between multiple pipes
    int pipe_count = 3;                          // Number of pipes in the tray
};

// Snapshot structure for debug timeline and visualization steps
struct DeformationStep {
    int step_index;
    std::string step_description;
    std::vector<Point3D> rubber_band_wps;        // Yellow Lines
    std::vector<Point3D> collision_points;       // Red Nodes
};

// Helper function to check line segment vs AABB intersection
inline bool SegmentAABBCollide(const Point3D& p1, const Point3D& p2, const AABB& box) {
    double tmin = 0.0;
    double tmax = 1.0;
    
    // X Axis
    double dx = p2.x - p1.x;
    if (std::abs(dx) < 1e-9) {
        if (p1.x < box.lo.x || p1.x > box.hi.x) return false;
    } else {
        double ood = 1.0 / dx;
        double t1 = (box.lo.x - p1.x) * ood;
        double t2 = (box.hi.x - p1.x) * ood;
        if (t1 > t2) std::swap(t1, t2);
        tmin = std::max(tmin, t1);
        tmax = std::min(tmax, t2);
        if (tmin > tmax) return false;
    }
    
    // Y Axis
    double dy = p2.y - p1.y;
    if (std::abs(dy) < 1e-9) {
        if (p1.y < box.lo.y || p1.y > box.hi.y) return false;
    } else {
        double ood = 1.0 / dy;
        double t1 = (box.lo.y - p1.y) * ood;
        double t2 = (box.hi.y - p1.y) * ood;
        if (t1 > t2) std::swap(t1, t2);
        tmin = std::max(tmin, t1);
        tmax = std::min(tmax, t2);
        if (tmin > tmax) return false;
    }
    
    // Z Axis
    double dz = p2.z - p1.z;
    if (std::abs(dz) < 1e-9) {
        if (p1.z < box.lo.z || p1.z > box.hi.z) return false;
    } else {
        double ood = 1.0 / dz;
        double t1 = (box.lo.z - p1.z) * ood;
        double t2 = (box.hi.z - p1.z) * ood;
        if (t1 > t2) std::swap(t1, t2);
        tmin = std::max(tmin, t1);
        tmax = std::min(tmax, t2);
        if (tmin > tmax) return false;
    }
    
    return true;
}

// Helper to expand AABB by tray size + safety margin
inline AABB ExpandAABB(const AABB& box, double tray_w, double tray_h, double margin) {
    double pad_h = (tray_w / 2.0) + margin;
    double pad_v = (tray_h / 2.0) + margin;
    return AABB(
        Vec3{box.lo.x - pad_h, box.lo.y - pad_h, box.lo.z - pad_v},
        Vec3{box.hi.x + pad_h, box.hi.y + pad_h, box.hi.z + pad_v}
    );
}

class DataDrivenRubberBandEngine {
private:
    RoutingConfig config;
    DesignFeaturePoints features;
    std::vector<AABB> raw_obstacles;
    std::vector<AABB> expanded_obstacles;
    std::vector<DeformationStep> debug_timeline;

public:
    void Initialize(const RoutingConfig& cfg, const DesignFeaturePoints& f_pts);
    void IngestObstacles(const std::vector<AABB>& obstacles);
    std::vector<std::vector<Point3D>> ExecuteGroupRouting(Point3D start, Point3D end);

    // Timeline accessors for Visual Debugger
    size_t GetStepCount() const { return debug_timeline.size(); }
    const DeformationStep& GetStep(size_t index) const { return debug_timeline[index]; }
};

} // namespace routing3d
