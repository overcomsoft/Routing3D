#include "routing3d/rubber_band_engine.hpp"
#include <iostream>
#include <cmath>
#include <algorithm>

namespace routing3d {

void DataDrivenRubberBandEngine::Initialize(const RoutingConfig& cfg, const DesignFeaturePoints& f_pts) {
    config = cfg;
    features = f_pts;
    raw_obstacles.clear();
    expanded_obstacles.clear();
    debug_timeline.clear();
}

void DataDrivenRubberBandEngine::IngestObstacles(const std::vector<AABB>& obstacles) {
    raw_obstacles = obstacles;
    expanded_obstacles.clear();
    for (const auto& obs : obstacles) {
        expanded_obstacles.push_back(ExpandAABB(obs, config.tray_width, config.tray_height, config.safety_margin));
    }
}

std::vector<std::vector<Point3D>> DataDrivenRubberBandEngine::ExecuteGroupRouting(Point3D start, Point3D end) {
    debug_timeline.clear();
    
    // Determine vertical direction tendency
    double z_diff = end.z - start.z;
    enum class VertTrend { FLAT, UP, DOWN } trend = VertTrend::FLAT;
    if (std::abs(z_diff) >= 1.0) {
        trend = (z_diff > 0) ? VertTrend::UP : VertTrend::DOWN;
    }

    // ---------------------------------------------------------
    // Step 1: Initial Tension (초기 인장)
    // ---------------------------------------------------------
    DeformationStep step1;
    step1.step_index = 1;
    step1.step_description = "Step 1 (초기 인장): 출발지와 목적지 간의 직선 연결선 형성 (장애물 무시)";
    step1.rubber_band_wps = {start, end};
    debug_timeline.push_back(step1);

    // ---------------------------------------------------------
    // Step 2: Z-Layer Snap (특징점 스냅)
    // ---------------------------------------------------------
    double z_way = start.z;
    if (!features.frequent_z_levels.empty()) {
        if (trend == VertTrend::FLAT) {
            // Find closest Z level to start.z
            double min_d = 1e18;
            double best_z = start.z;
            for (double hz : features.frequent_z_levels) {
                double d = std::abs(hz - start.z);
                if (d < min_d) {
                    min_d = d;
                    best_z = hz;
                }
            }
            // Only snap if it is within 2000mm from start Z
            if (min_d < 2000.0) {
                z_way = best_z;
            } else {
                z_way = start.z;
            }
        } else {
            // UP or DOWN: Find Z level between start.z and end.z closest to midpoint
            double mid = (start.z + end.z) / 2.0;
            double min_d = 1e18;
            bool found_between = false;
            double best_z = start.z;
            
            double lower = std::min(start.z, end.z);
            double upper = std::max(start.z, end.z);
            
            for (double hz : features.frequent_z_levels) {
                if (hz >= lower && hz <= upper) {
                    double d = std::abs(hz - mid);
                    if (d < min_d) {
                        min_d = d;
                        best_z = hz;
                        found_between = true;
                    }
                }
            }
            if (found_between) {
                z_way = best_z;
            } else {
                // Check if any Z level is slightly outside the range (within 1000mm)
                min_d = 1e18;
                bool found_near = false;
                for (double hz : features.frequent_z_levels) {
                    if (hz >= lower - 1000.0 && hz <= upper + 1000.0) {
                        double d = std::abs(hz - mid);
                        if (d < min_d) {
                            min_d = d;
                            best_z = hz;
                            found_near = true;
                        }
                    }
                }
                if (found_near) {
                    z_way = best_z;
                } else {
                    // Fallback to start.z to prevent massive detours
                    z_way = start.z;
                }
            }
        }
    }

    Point3D p1 = start;
    Point3D p2{start.x, start.y, z_way};
    Point3D p3{end.x, end.y, z_way};
    Point3D p4 = end;

    DeformationStep step2;
    step2.step_index = 2;
    step2.step_description = "Step 2 (특징점 스냅): 주입된 다빈도 Z 레이어(Z=" + std::to_string(static_cast<int>(z_way)) + "mm)로 수직 분할 세그먼트 생성";
    step2.rubber_band_wps = {p1, p2, p3, p4};
    debug_timeline.push_back(step2);

    // ---------------------------------------------------------
    // Step 3: Obstacle Interference (장애물 간섭)
    // ---------------------------------------------------------
    std::vector<Point3D> col_pts;
    std::vector<size_t> colliding_indices;
    
    // Check collisions for the horizontal segment p2 -> p3
    for (size_t i = 0; i < expanded_obstacles.size(); ++i) {
        if (SegmentAABBCollide(p2, p3, expanded_obstacles[i])) {
            colliding_indices.push_back(i);
            // Calculate a rough collision point (intersection midpoint or closest approach)
            // For simplicity, find the intersection along the segment
            double tx_min = std::min(p2.x, p3.x);
            double tx_max = std::max(p2.x, p3.x);
            double ty_min = std::min(p2.y, p3.y);
            double ty_max = std::max(p2.y, p3.y);
            
            double cx = std::max(tx_min, std::min(tx_max, (expanded_obstacles[i].lo.x + expanded_obstacles[i].hi.x) / 2.0));
            double cy = std::max(ty_min, std::min(ty_max, (expanded_obstacles[i].lo.y + expanded_obstacles[i].hi.y) / 2.0));
            
            col_pts.push_back(Point3D{cx, cy, z_way});
        }
    }

    DeformationStep step3;
    step3.step_index = 3;
    step3.step_description = "Step 3 (장애물 간섭): 수평 이동 구간 내 간섭을 일으키는 거대 장애물(" + std::to_string(col_pts.size()) + "개) 및 충돌 모서리 검출";
    step3.rubber_band_wps = step2.rubber_band_wps;
    step3.collision_points = col_pts;
    debug_timeline.push_back(step3);

    // ---------------------------------------------------------
    // Step 4: Final Deformation (최종 변형 완료)
    // ---------------------------------------------------------
    // We deform the path by routing around the colliding obstacles on the segment p2 -> p3.
    std::vector<Point3D> centerline_path = {p1, p2};
    
    // Sort colliding obstacles by proximity to p2 (to handle them in order)
    std::sort(colliding_indices.begin(), colliding_indices.end(), [&](size_t idx_a, size_t idx_b) {
        const auto& box_a = expanded_obstacles[idx_a];
        const auto& box_b = expanded_obstacles[idx_b];
        double dist_a = std::pow(box_a.lo.x - p2.x, 2) + std::pow(box_a.lo.y - p2.y, 2);
        double dist_b = std::pow(box_b.lo.x - p2.x, 2) + std::pow(box_b.lo.y - p2.y, 2);
        return dist_a < dist_b;
    });

    Point3D curr = p2;
    int vertical_bends_used = 2; // S -> p2 (1), p3 -> D (1)

    for (size_t idx : colliding_indices) {
        const auto& raw_obs = raw_obstacles[idx];
        const auto& exp_obs = expanded_obstacles[idx];

        // Verify if segment from 'curr' to 'p3' still collides with this obstacle
        if (!SegmentAABBCollide(curr, p3, exp_obs)) {
            continue;
        }

        bool bypassed = false;

        // Policy 1: Tunneling (우선순위 1)
        for (const auto& zone : features.frequent_bend_zones) {
            // Check if the zone is close to the raw obstacle and can serve as a tunnel
            // We verify if zone center lies near the obstacle boundary
            double zone_cx = (zone.lo.x + zone.hi.x) / 2.0;
            double zone_cy = (zone.lo.y + zone.hi.y) / 2.0;
            double zone_cz = (zone.lo.z + zone.hi.z) / 2.0;

            if (std::abs(zone_cz - z_way) < 500.0) { // Same height level roughly
                double dx = zone_cx - ((raw_obs.lo.x + raw_obs.hi.x) / 2.0);
                double dy = zone_cy - ((raw_obs.lo.y + raw_obs.hi.y) / 2.0);
                double dist = std::sqrt(dx*dx + dy*dy);
                if (dist < 4000.0) { // Within tunnel distance range
                    // Verify if this tunnel path segment is collision free
                    Point3D tunnel_wp{zone_cx, zone_cy, z_way};
                    
                    // Check if segment from 'curr' to tunnel_wp and tunnel_wp to p3 is free of other obstacles
                    bool collision = false;
                    for (size_t k = 0; k < expanded_obstacles.size(); ++k) {
                        if (k == idx) continue;
                        if (SegmentAABBCollide(curr, tunnel_wp, expanded_obstacles[k]) ||
                            SegmentAABBCollide(tunnel_wp, p3, expanded_obstacles[k])) {
                            collision = true;
                            break;
                        }
                    }
                    if (!collision) {
                        // Make orthogonal detour through tunnel_wp
                        if (std::abs(curr.x - tunnel_wp.x) > 1e-3 && std::abs(curr.y - tunnel_wp.y) > 1e-3) {
                            // Insert corner to maintain orthogonal turns
                            centerline_path.push_back(Point3D{tunnel_wp.x, curr.y, z_way});
                        }
                        centerline_path.push_back(tunnel_wp);
                        curr = tunnel_wp;
                        bypassed = true;
                        break;
                    }
                }
            }
        }

        if (bypassed) continue;

        // Policy 2: Vertical Over/Underpass (우선순위 2)
        if (vertical_bends_used + 2 <= config.max_vertical_bends && !features.frequent_z_levels.empty()) {
            double best_alt_z = -1.0;
            double min_detour_dist = 1e18;
            bool found_alt_z = false;

            for (double alt_z : features.frequent_z_levels) {
                // alt_z should be completely above or below the raw obstacle
                if (alt_z > raw_obs.hi.z + (config.tray_height / 2.0) + config.safety_margin ||
                    alt_z < raw_obs.lo.z - (config.tray_height / 2.0) - config.safety_margin) {
                    
                    // Verify if routing at alt_z is free of collisions
                    Point3D up_p1{curr.x, curr.y, alt_z};
                    Point3D up_p2{p3.x, p3.y, alt_z};
                    
                    bool collision = false;
                    for (size_t k = 0; k < expanded_obstacles.size(); ++k) {
                        if (SegmentAABBCollide(curr, up_p1, expanded_obstacles[k]) ||
                            SegmentAABBCollide(up_p1, up_p2, expanded_obstacles[k]) ||
                            SegmentAABBCollide(up_p2, p3, expanded_obstacles[k])) {
                            collision = true;
                            break;
                        }
                    }

                    if (!collision) {
                        double detour_dist = 2.0 * std::abs(alt_z - z_way);
                        if (detour_dist < min_detour_dist) {
                            min_detour_dist = detour_dist;
                            best_alt_z = alt_z;
                            found_alt_z = true;
                        }
                    }
                }
            }

            // We prefer vertical overpass if the vertical detour distance is shorter than horizontal bypass distance
            double horizontal_est = std::min(std::abs(exp_obs.hi.y - curr.y), std::abs(curr.y - exp_obs.lo.y)) * 2.0;
            if (found_alt_z && min_detour_dist < horizontal_est) {
                // Perform vertical overpass
                Point3D up_p1{curr.x, curr.y, best_alt_z};
                // Find horizontal transition point past the obstacle
                // If moving along X
                double next_x = curr.x;
                if (std::abs(p3.x - curr.x) > std::abs(p3.y - curr.y)) {
                    next_x = (p3.x > curr.x) ? exp_obs.hi.x : exp_obs.lo.x;
                } else {
                    next_x = p3.x;
                }
                double next_y = curr.y;
                if (std::abs(p3.y - curr.y) > std::abs(p3.x - curr.x)) {
                    next_y = (p3.y > curr.y) ? exp_obs.hi.y : exp_obs.lo.y;
                } else {
                    next_y = p3.y;
                }

                Point3D up_p2{next_x, next_y, best_alt_z};
                Point3D down_p{next_x, next_y, z_way};

                centerline_path.push_back(up_p1);
                centerline_path.push_back(up_p2);
                centerline_path.push_back(down_p);
                
                curr = down_p;
                vertical_bends_used += 2;
                bypassed = true;
            }
        }

        if (bypassed) continue;

        // Policy 3: Min Margin Horizontal Bypass (우선순위 3)
        // Determine segment orientation
        bool is_x_axis = std::abs(p3.x - curr.x) > std::abs(p3.y - curr.y);
        
        if (is_x_axis) {
            // Detour along Y axis
            double y1 = exp_obs.hi.y;
            double y2 = exp_obs.lo.y;
            
            // Choose detour that minimizes deviation and is collision free
            double detour_y = y1;
            double d1 = std::abs(y1 - curr.y);
            double d2 = std::abs(y2 - curr.y);
            
            // Check if detour route y1 is collision free (except with the current obstacle)
            bool y1_col = false;
            Point3D turn1_a{exp_obs.lo.x, curr.y, z_way};
            if (p3.x < curr.x) turn1_a.x = exp_obs.hi.x;
            Point3D turn1_b{turn1_a.x, y1, z_way};
            Point3D turn1_c{(p3.x > curr.x) ? exp_obs.hi.x : exp_obs.lo.x, y1, z_way};
            Point3D turn1_d{turn1_c.x, curr.y, z_way};

            for (size_t k = 0; k < expanded_obstacles.size(); ++k) {
                if (k == idx) continue;
                if (SegmentAABBCollide(curr, turn1_a, expanded_obstacles[k]) ||
                    SegmentAABBCollide(turn1_a, turn1_b, expanded_obstacles[k]) ||
                    SegmentAABBCollide(turn1_b, turn1_c, expanded_obstacles[k]) ||
                    SegmentAABBCollide(turn1_c, turn1_d, expanded_obstacles[k])) {
                    y1_col = true;
                    break;
                }
            }

            bool y2_col = false;
            Point3D turn2_a{turn1_a.x, curr.y, z_way};
            Point3D turn2_b{turn2_a.x, y2, z_way};
            Point3D turn2_c{turn1_c.x, y2, z_way};
            Point3D turn2_d{turn2_c.x, curr.y, z_way};

            for (size_t k = 0; k < expanded_obstacles.size(); ++k) {
                if (k == idx) continue;
                if (SegmentAABBCollide(curr, turn2_a, expanded_obstacles[k]) ||
                    SegmentAABBCollide(turn2_a, turn2_b, expanded_obstacles[k]) ||
                    SegmentAABBCollide(turn2_b, turn2_c, expanded_obstacles[k]) ||
                    SegmentAABBCollide(turn2_c, turn2_d, expanded_obstacles[k])) {
                    y2_col = true;
                    break;
                }
            }

            if (y1_col && !y2_col) {
                detour_y = y2;
            } else if (!y1_col && y2_col) {
                detour_y = y1;
            } else {
                detour_y = (d1 < d2) ? y1 : y2;
            }

            double turn_x_start = (p3.x > curr.x) ? exp_obs.lo.x : exp_obs.hi.x;
            double turn_x_end = (p3.x > curr.x) ? exp_obs.hi.x : exp_obs.lo.x;

            centerline_path.push_back(Point3D{turn_x_start, curr.y, z_way});
            centerline_path.push_back(Point3D{turn_x_start, detour_y, z_way});
            centerline_path.push_back(Point3D{turn_x_end, detour_y, z_way});
            centerline_path.push_back(Point3D{turn_x_end, curr.y, z_way});
            
            curr = Point3D{turn_x_end, curr.y, z_way};
        } else {
            // Detour along X axis
            double x1 = exp_obs.hi.x;
            double x2 = exp_obs.lo.x;
            
            double detour_x = x1;
            double d1 = std::abs(x1 - curr.x);
            double d2 = std::abs(x2 - curr.x);

            bool x1_col = false;
            Point3D turn1_a{curr.x, (p3.y > curr.y) ? exp_obs.lo.y : exp_obs.hi.y, z_way};
            Point3D turn1_b{x1, turn1_a.y, z_way};
            Point3D turn1_c{x1, (p3.y > curr.y) ? exp_obs.hi.y : exp_obs.lo.y, z_way};
            Point3D turn1_d{curr.x, turn1_c.y, z_way};

            for (size_t k = 0; k < expanded_obstacles.size(); ++k) {
                if (k == idx) continue;
                if (SegmentAABBCollide(curr, turn1_a, expanded_obstacles[k]) ||
                    SegmentAABBCollide(turn1_a, turn1_b, expanded_obstacles[k]) ||
                    SegmentAABBCollide(turn1_b, turn1_c, expanded_obstacles[k]) ||
                    SegmentAABBCollide(turn1_c, turn1_d, expanded_obstacles[k])) {
                    x1_col = true;
                    break;
                }
            }

            bool x2_col = false;
            Point3D turn2_a{curr.x, turn1_a.y, z_way};
            Point3D turn2_b{x2, turn2_a.y, z_way};
            Point3D turn2_c{x2, turn1_c.y, z_way};
            Point3D turn2_d{curr.x, turn2_c.y, z_way};

            for (size_t k = 0; k < expanded_obstacles.size(); ++k) {
                if (k == idx) continue;
                if (SegmentAABBCollide(curr, turn2_a, expanded_obstacles[k]) ||
                    SegmentAABBCollide(turn2_a, turn2_b, expanded_obstacles[k]) ||
                    SegmentAABBCollide(turn2_b, turn2_c, expanded_obstacles[k]) ||
                    SegmentAABBCollide(turn2_c, turn2_d, expanded_obstacles[k])) {
                    x2_col = true;
                    break;
                }
            }

            if (x1_col && !x2_col) {
                detour_x = x2;
            } else if (!x1_col && x2_col) {
                detour_x = x1;
            } else {
                detour_x = (d1 < d2) ? x1 : x2;
            }

            double turn_y_start = (p3.y > curr.y) ? exp_obs.lo.y : exp_obs.hi.y;
            double turn_y_end = (p3.y > curr.y) ? exp_obs.hi.y : exp_obs.lo.y;

            centerline_path.push_back(Point3D{curr.x, turn_y_start, z_way});
            centerline_path.push_back(Point3D{detour_x, turn_y_start, z_way});
            centerline_path.push_back(Point3D{detour_x, turn_y_end, z_way});
            centerline_path.push_back(Point3D{curr.x, turn_y_end, z_way});
            
            curr = Point3D{curr.x, turn_y_end, z_way};
        }
    }

    // Connect to end coordinates
    if (std::abs(curr.x - p3.x) > 1e-3 || std::abs(curr.y - p3.y) > 1e-3) {
        centerline_path.push_back(p3);
    }
    centerline_path.push_back(p4);

    // Save final deformation step
    DeformationStep step4;
    step4.step_index = 4;
    step4.step_description = "Step 4 (최종 변형 완료): 3대 장애물 회피 전략 적용으로 직각 우회 웨이포인트 분할 및 90도 직교화 완료";
    step4.rubber_band_wps = centerline_path;
    debug_timeline.push_back(step4);

    // ---------------------------------------------------------
    // Individual Pipe Coordinate Generation (Post-Processing)
    // ---------------------------------------------------------
    std::vector<std::vector<Point3D>> multi_pipe_paths(config.pipe_count);
    
    for (int i = 0; i < config.pipe_count; ++i) {
        double offset_mag = (i - (config.pipe_count - 1) / 2.0) * config.pipe_pitch;
        
        // For each segment in the centerline, offset it horizontally
        for (size_t j = 0; j < centerline_path.size(); ++j) {
            Point3D pt = centerline_path[j];
            Point3D dir{0, 0, 0};
            
            // Calculate direction vector of the segment
            if (j == 0) {
                if (centerline_path.size() > 1) {
                    dir.x = centerline_path[1].x - centerline_path[0].x;
                    dir.y = centerline_path[1].y - centerline_path[0].y;
                    dir.z = centerline_path[1].z - centerline_path[0].z;
                }
            } else {
                dir.x = centerline_path[j].x - centerline_path[j-1].x;
                dir.y = centerline_path[j].y - centerline_path[j-1].y;
                dir.z = centerline_path[j].z - centerline_path[j-1].z;
            }

            double len = std::sqrt(dir.x*dir.x + dir.y*dir.y);
            if (len > 1e-3) {
                // Normal vector in XY plane: (-dir.y, dir.x)
                double nx = -dir.y / len;
                double ny = dir.x / len;
                
                pt.x += nx * offset_mag;
                pt.y += ny * offset_mag;
            }
            multi_pipe_paths[i].push_back(pt);
        }
    }

    return multi_pipe_paths;
}

} // namespace routing3d
