#include "routing3d/rubber_band_engine.hpp"
#include <cstdio>
#include <string>
#include <vector>

using namespace routing3d;

static int g_failures = 0;
static void check(bool cond, const std::string& msg) {
    std::printf("  [%s] %s\n", cond ? "PASS" : "FAIL", msg.c_str());
    if (!cond) ++g_failures;
}

static void test_rubber_band_engine() {
    std::printf("=== test_rubber_band_engine (Case 1: Tunneling) ===\n");
    
    DataDrivenRubberBandEngine engine;
    
    RoutingConfig cfg;
    cfg.max_vertical_bends = 5;
    cfg.safety_margin = 50.0;
    cfg.tray_width = 600.0;
    cfg.tray_height = 300.0;
    cfg.pipe_pitch = 100.0;
    cfg.pipe_count = 3;

    DesignFeaturePoints f_pts;
    f_pts.frequent_z_levels = {1000.0, 3000.0, 5000.0};
    
    // Add a frequent bend zone (tunnel option)
    f_pts.frequent_bend_zones.push_back(AABB(
        Vec3{14500, 14500, 2850},
        Vec3{15500, 15500, 3150}
    ));

    engine.Initialize(cfg, f_pts);

    std::vector<AABB> obstacles;
    obstacles.push_back(AABB(
        Vec3{14000, 13000, 2000},
        Vec3{16000, 17000, 4000}
    ));
    engine.IngestObstacles(obstacles);

    Point3D start{5000, 15000, 1000};
    Point3D end{25000, 15000, 5000};

    auto paths = engine.ExecuteGroupRouting(start, end);

    check(engine.GetStepCount() == 4, "Timeline step count must be exactly 4");
    check(paths.size() == 3, "Pipes count must be 3");
    
    const auto& centerline = paths[1];
    check(centerline.size() == 5, "Centerline should have 5 waypoints");

    // Case 2: Vertical Over/Underpass (Policy 2)
    std::printf("=== test_rubber_band_engine (Case 2: Vertical Underpass) ===\n");
    f_pts.frequent_bend_zones.clear(); // Remove tunnel
    engine.Initialize(cfg, f_pts);
    engine.IngestObstacles(obstacles);
    
    auto paths_under = engine.ExecuteGroupRouting(start, end);
    const auto& centerline_under = paths_under[1];
    bool underpassed = false;
    for (const auto& wp : centerline_under) {
        if (wp.z == 1000.0 && wp.x > 5000.0 && wp.x < 25000.0) {
            underpassed = true;
        }
    }
    check(underpassed, "Path successfully performed vertical underpass");

    // Case 3: Horizontal Bypass (Policy 3)
    std::printf("=== test_rubber_band_engine (Case 3: Horizontal Bypass) ===\n");
    cfg.max_vertical_bends = 2; // Prevent vertical detours (S->z_way and z_way->D take 2 bends already)
    engine.Initialize(cfg, f_pts);
    engine.IngestObstacles(obstacles);
    
    auto paths_bypass = engine.ExecuteGroupRouting(start, end);
    const auto& centerline_bypass = paths_bypass[1];
    
    std::printf("Bypass Waypoints:\n");
    for (const auto& wp : centerline_bypass) {
        std::printf("    (%.1f, %.1f, %.1f)\n", wp.x, wp.y, wp.z);
    }
    
    bool detoured = false;
    for (const auto& wp : centerline_bypass) {
        if (std::abs(wp.y - 17350.0) < 1.0 || std::abs(wp.y - 12650.0) < 1.0) {
            detoured = true;
        }
    }
    check(detoured, "Path successfully detoured around the expanded obstacle Y boundaries");
}

int main() {
    test_rubber_band_engine();
    std::printf("\n%s (failures=%d)\n", g_failures == 0 ? "ALL PASS" : "FAILED", g_failures);
    return g_failures == 0 ? 0 : 1;
}
