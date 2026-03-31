#ifndef PPCA_SRC_HPP
#define PPCA_SRC_HPP
#include "math.h"

// Forward declaration to match OJ environment includes
class Monitor;

class Controller {

public:
    Controller(const Vec &_pos_tar, double _v_max, double _r, int _id, Monitor *_monitor) {
        pos_tar = _pos_tar;
        v_max = _v_max;
        r = _r;
        id = _id;
        monitor = _monitor;
        step_count = 0;
        robot_num_cache = -1;
    }

    void set_pos_cur(const Vec &_pos_cur) {
        pos_cur = _pos_cur;
    }

    void set_v_cur(const Vec &_v_cur) {
        v_cur = _v_cur;
    }

private:
    int id;
    Vec pos_tar;
    Vec pos_cur;
    Vec v_cur;
    double v_max, r;
    Monitor *monitor;

    int step_count;          // increases every call to get_v_next
    int robot_num_cache;     // cache of total robots

    // Compute a desired velocity towards target with speed cap
    Vec desired_towards_target() const {
        Vec to_tar = pos_tar - pos_cur;
        double dist = to_tar.norm();
        if (dist <= EPSILON) return Vec(0.0, 0.0);
        double max_step_speed = v_max;
        double need_speed = dist / TIME_INTERVAL;
        double speed = need_speed < max_step_speed ? need_speed : max_step_speed;
        Vec dir = to_tar / dist; // normalized
        return dir * speed;
    }

    // Check if candidate velocity v is safe against robot j (assuming j stays stationary this step)
    bool safe_against_robot_j(const Vec &v_candidate, int j_id) const {
        if (j_id == id) return true;
        Vec pos_j = monitor->get_pos_cur(j_id);
        double r_j = monitor->get_r(j_id);
        Vec delta_pos = pos_cur - pos_j;
        Vec delta_v = v_candidate; // other assumed 0 when not moving

        // If relative speed is near zero, distance must be safe now
        double dv_norm = delta_v.norm();
        if (dv_norm < 1e-12) {
            double min_dis_sqr = delta_pos.norm_sqr();
            double delta_r = r + r_j;
            return min_dis_sqr > delta_r * delta_r - EPSILON;
        }

        double project = delta_pos.dot(delta_v);
        if (project >= 0) {
            // moving away or tangential in terms of closest approach
            return true;
        }
        project /= -dv_norm; // distance along velocity to closest approach within this step
        double delta_r = r + r_j;
        double min_dis_sqr;
        if (project < dv_norm * TIME_INTERVAL) {
            min_dis_sqr = delta_pos.norm_sqr() - project * project;
        } else {
            Vec end_delta = delta_pos + delta_v * TIME_INTERVAL;
            min_dis_sqr = end_delta.norm_sqr();
        }
        return min_dis_sqr > delta_r * delta_r - EPSILON;
    }

    // Check if candidate velocity is safe against all other robots (assuming others are stationary this step)
    bool safe_against_all(const Vec &v_candidate) const {
        int n = robot_num_cache > 0 ? robot_num_cache : monitor->get_robot_number();
        for (int j = 0; j < n; ++j) {
            if (j == id) continue;
            if (!safe_against_robot_j(v_candidate, j)) return false;
        }
        return true;
    }

    // Find a safe velocity by scaling down towards target
    Vec find_safe_velocity() const {
        Vec v_des = desired_towards_target();
        // Quick accept
        if (safe_against_all(v_des)) return v_des;
        // Binary search best scaling factor in [0,1]
        double lo = 0.0, hi = 1.0;
        Vec best(0.0, 0.0);
        for (int it = 0; it < 20; ++it) {
            double mid = (lo + hi) * 0.5;
            Vec v_mid = v_des * mid;
            if (safe_against_all(v_mid)) {
                best = v_mid;
                lo = mid;
            } else {
                hi = mid;
            }
        }
        return best;
    }

public:

    Vec get_v_next() {
        // Cache robot number once available
        if (robot_num_cache <= 0) robot_num_cache = monitor->get_robot_number();

        // Simple safe scheduler: only one robot moves per round, others stop
        int current_round = step_count;
        int mover_id = robot_num_cache > 0 ? (current_round % robot_num_cache) : 0;

        Vec v_next(0.0, 0.0);

        if ((id == mover_id)) {
            // Move towards target while ensuring no collision with stationary others
            v_next = find_safe_velocity();
        } else {
            v_next = Vec(0.0, 0.0);
        }

        // Ensure speed cap strictly respected (with a tiny margin)
        double sp2 = v_next.norm_sqr();
        double vmax2 = v_max * v_max;
        if (sp2 > vmax2) {
            double scale = v_max / std::sqrt(sp2);
            v_next = v_next * scale;
        }

        // Advance local step counter for next round
        ++step_count;

        return v_next;
    }
};


#endif //PPCA_SRC_HPP

