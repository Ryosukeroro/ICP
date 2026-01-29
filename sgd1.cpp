#include <iostream>
#include <cmath>
#include <ctime>
#include <limits>
#include <string>
#include <array>
#include <vector>
#include <fstream>
#include <sstream>
#include <chrono>
#include <iomanip>
#include <random>
#include <algorithm>
#include "point.h"
#include "constants.h"
// #include "sgd.h" // ヘッダーファイルがない場合はこの行は不要（今回は直接実装しているため）

// ==========================================
// 1. 構造体定義 (エラー2の修正: error_thetaに統一)
// ==========================================
struct ExperimentResult {
    double true_dx, true_dy, true_theta;
    double est_dx, est_dy, est_theta;
    double error_trans;
    double error_theta; // ★ここを error_theta に統一しました
    int iterations;
    double time_ms;
    bool converged;
};

// プロトタイプ宣言
ExperimentResult icp_sgd(FILE* pipe, const std::vector<Point>& original_source, const std::vector<Point>& target, double true_dx, double true_dy, double true_theta, bool visual_mode);

// ==========================================
// 2. ヘルパー関数群
// ==========================================
std::vector<Point> read_scan_points(const std::string& file_path) {
    std::ifstream file(file_path);
    std::vector<Point> points;
    if (!file.is_open()) return points;
    std::string line_str;
    while (std::getline(file, line_str)) {
        std::istringstream iss(line_str);
        float x, y;
        if (!(iss >> x >> y)) continue;
        points.push_back({x, y});
    }
    return points;
}

std::array<std::array<float, 3>, 3> make_transformation_matrix(float tx, float ty, float theta) {
    return {{
        {std::cos(theta), -std::sin(theta), tx},
        {std::sin(theta), std::cos(theta), ty},
        {0.0f, 0.0f, 1.0f}
    }};
}

std::vector<Point> transformpoints(const std::vector<Point>& points, float dx, float dy, double theta) {
    std::vector<Point> moved_points;
    double c = std::cos(theta), s = std::sin(theta);
    for (const auto& point : points) {
        moved_points.push_back({
            static_cast<float>(point.x * c - point.y * s + dx),
            static_cast<float>(point.x * s + point.y * c + dy)
        });
    }
    return moved_points;
}

void plot(FILE* gnuplot_pipe, const std::vector<Point>& target, const std::vector<Point>& Source, int iteration) {
    if (gnuplot_pipe == nullptr) return;
    fprintf(gnuplot_pipe, "set size ratio 1\n");
    fprintf(gnuplot_pipe, "set autoscale\n"); 
    fprintf(gnuplot_pipe, "set title 'Iteration %d'\n", iteration);
    fprintf(gnuplot_pipe, "plot '-' with points pointtype 7 pointsize 0.5 lc rgb 'blue' title 'Target','-' with points pointtype 7 pointsize 0.5 lc rgb 'red' title 'Source'\n");
    for (const auto& p : target) fprintf(gnuplot_pipe, "%f %f\n", p.x, p.y);
    fprintf(gnuplot_pipe, "e\n");
    for (const auto& p : Source) fprintf(gnuplot_pipe, "%f %f\n", p.x, p.y);
    fprintf(gnuplot_pipe, "e\n");
    fflush(gnuplot_pipe);
}

std::vector<Point> transform_points_all(const std::vector<Point>& points, double dx, double dy, double theta) {
    std::vector<Point> moved_points;
    moved_points.reserve(points.size());
    double c = std::cos(theta);
    double s = std::sin(theta);
    for (const auto& p : points) {
        moved_points.push_back({
            static_cast<float>(p.x * c - p.y * s + dx),
            static_cast<float>(p.x * s + p.y * c + dy)
        });
    }
    return moved_points;
}

void shuffle_data(std::vector<Point>& points) {
    std::random_device rd;
    std::mt19937 g(rd());
    std::shuffle(points.begin(), points.end(), g);
}

std::vector<Point> createBatch(std::vector<Point>& source, size_t& m_current_offset, size_t m_batch_size) {
    std::vector<Point> batch;
    if (source.empty()) return batch;
    
    for(size_t i=0; i<m_batch_size; ++i) {
        if(m_current_offset >= source.size()) {
            shuffle_data(source);
            m_current_offset = 0;
        }
        batch.push_back(source[m_current_offset++]);
    }
    return batch;
}

double dist_sq(const Point& a, const Point& b) {
    return (a.x - b.x)*(a.x - b.x) + (a.y - b.y)*(a.y - b.y);
}

int findClosestPoint(const Point& p, const std::vector<Point>& target) {
    int idx = -1;
    double min_d = std::numeric_limits<double>::max();
    for (size_t i = 0; i < target.size(); ++i) {
        double d = dist_sq(p, target[i]);
        if (d < min_d) { min_d = d; idx = i; }
    }
    return idx;
}

// 数値微分用ヘルパー
Point get_transformed_point(const Point& p, double tx, double ty, double theta) {
    double c = std::cos(theta), s = std::sin(theta);
    return { static_cast<float>(p.x * c - p.y * s + tx), static_cast<float>(p.x * s + p.y * c + ty) };
}

double calc_loss(const Point& target_p, const Point& source_orig, double tx, double ty, double th) {
    Point p_trans = get_transformed_point(source_orig, tx, ty, th);
    double dx = target_p.x - p_trans.x;
    double dy = target_p.y - p_trans.y;
    return dx * dx + dy * dy;
}

void compute_gradients(const Point& target_p, const Point& source_orig, const Point& source_curr, 
    double est_x, double est_y, double est_th, double& grad_tx, double& grad_ty, double& grad_th) {
    double loss_curr = calc_loss(target_p, source_orig, est_x, est_y, est_th);
    grad_tx = (calc_loss(target_p, source_orig, est_x + delta, est_y, est_th) - loss_curr) / delta;
    grad_ty = (calc_loss(target_p, source_orig, est_x, est_y + delta, est_th) - loss_curr) / delta;
    grad_th = (calc_loss(target_p, source_orig, est_x, est_y, est_th + delta) - loss_curr) / delta;
}

float get_absolute_max(const std::vector<Point>& source, const std::vector<Point>& target){
    float max_val = 0.0f;
    for(const auto& p : source) max_val = std::max(max_val, std::max(std::abs(p.x), std::abs(p.y)));
    for(const auto& p : target) max_val = std::max(max_val, std::max(std::abs(p.x), std::abs(p.y)));
    return max_val;
}

std::vector<Point> normalise_clouds(const std::vector<Point>& cloud, float max_absolute) {
    std::vector<Point> normalised_cloud = cloud;
    if (max_absolute == 0.0f) return normalised_cloud;
    for(auto& p : normalised_cloud) {
        p.x /= max_absolute;
        p.y /= max_absolute;
    }
    return normalised_cloud;
}

void rescale_transformation_matrix(double &x, double &y, float max_absolute) {
    x *= max_absolute;
    y *= max_absolute;
}

// ==========================================
// 3. ICPメイン関数
// ==========================================
ExperimentResult icp_sgd(FILE* pipe, const std::vector<Point>& original_source, const std::vector<Point>& target, double true_dx, double true_dy, double true_theta, bool visual_mode){
    
    auto icp_start = std::chrono::high_resolution_