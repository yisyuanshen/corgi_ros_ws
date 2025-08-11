#include <deque>
#include <vector>
#include <algorithm>
#include <cmath>
#include <Eigen/Dense>

#include "plane_segmentation.hpp"

struct TrackedPlane {
    std::deque<double> recent_distances;
    int no_update_count = 0;

    void add_distance(double d, size_t max_size = 5) {
        recent_distances.push_back(d);
        if (recent_distances.size() > max_size)
            recent_distances.pop_front();
        no_update_count = 0;  // reset on update
    }

    void increment_no_update() {
        no_update_count++;
    }

    double average() const {
        double sum = 0.0;
        for (double d : recent_distances) sum += d;
        return sum / recent_distances.size();
    }

    bool is_close(double d, double threshold = 0.03) const {
        return std::abs(average() - d) <= threshold;
    }

    bool is_stable(size_t required_count = 5) const {
        return recent_distances.size() >= required_count;
    }
};

struct TrackedNormal {
    std::deque<Eigen::Vector3d> recent_normals;

    void add_normal(const Eigen::Vector3d& n, size_t max_size = 5) {
        recent_normals.push_back(n);
        if (recent_normals.size() > max_size)
            recent_normals.pop_front();
    }

    Eigen::Vector3d average() const {
        Eigen::Vector3d sum = Eigen::Vector3d::Zero();
        for (const auto& n : recent_normals) sum += n;
        return sum.normalized();
    }
};

struct PlaneTracker {
    std::vector<TrackedPlane> horizontal_planes;
    std::vector<TrackedPlane> vertical_planes;
    TrackedNormal h_normal;
    TrackedNormal v_normal;

    void update_planes(const std::vector<double>& new_distances, std::vector<TrackedPlane>& tracked_planes, bool accept_if_larger) {
        std::vector<bool> updated_flags(tracked_planes.size(), false);

        for (double d : new_distances) {
            bool matched = false;
            for (size_t i = 0; i < tracked_planes.size(); ++i) {
                if (tracked_planes[i].is_close(d)) {
                    tracked_planes[i].add_distance(d);
                    updated_flags[i] = true;
                    matched = true;
                    break;
                }
            }

            if (!matched) {
                // 僅當距離比現有極值「更大/更小」才新增
                bool allow_insert = false;
                if (tracked_planes.empty()) {
                    allow_insert = true;  // 第一個平面直接接受
                } else {
                    double reference = tracked_planes.back().average();
                    allow_insert = accept_if_larger? (d > reference) : (d < reference);
                }

                if (allow_insert) {
                    TrackedPlane new_plane;
                    new_plane.add_distance(d);
                    tracked_planes.push_back(new_plane);
                    updated_flags.push_back(true); // 新增的也算有更新
                }
            }
        }

        // 增加 no_update_count，並移除不穩定 + 沒更新超過 5 次的
        for (size_t i = 0; i < tracked_planes.size(); /* no ++ */) {
            if (!updated_flags[i]) {
                tracked_planes[i].increment_no_update();
            }

            if (!tracked_planes[i].is_stable() && tracked_planes[i].no_update_count >= 5) {
                tracked_planes.erase(tracked_planes.begin() + i);
                updated_flags.erase(updated_flags.begin() + i);
            } else {
                ++i;
            }
        }

    }

    void update(const PlaneDistances& new_distances) {
        update_planes(new_distances.horizontal, horizontal_planes, true);
        update_planes(new_distances.vertical, vertical_planes, true);
        if (!new_distances.vertical.empty()) {
            v_normal.add_normal(new_distances.v_normal);
        }
        if (!new_distances.horizontal.empty()) {
            h_normal.add_normal(new_distances.h_normal);
        }
    }

    std::vector<double> get_horizontal_averages() const {
        std::vector<double> result;
        for (const auto& p : horizontal_planes)
            if (p.is_stable())
                result.push_back(p.average());
        return result;
    }

    std::vector<double> get_vertical_averages() const {
        std::vector<double> result;
        for (const auto& p : vertical_planes)
            if (p.is_stable())
                result.push_back(p.average());
        return result;
    }
    
    Eigen::Vector3d get_vertical_normal() const {
        return v_normal.average();
    }

    Eigen::Vector3d get_horizontal_normal() const {
        return h_normal.average();
    }
};
