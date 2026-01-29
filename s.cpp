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
#include <iomanip> // output formatting
#include <random>
#include <unistd.h>
#include <algorithm>
#include "point.h"
#include "constants.h"
#include "sgd.h"

std::vector<Point> read_scan_points(const std::string& file_path) {
    std::ifstream file(file_path);
    std::vector<Point> points;
    if (!file.is_open()) {
        std::cerr << "File could not be opened." << std::endl;
        return points;
    }
    std::string line_str;
    while (std::getline(file, line_str)) {
        std::istringstream iss(line_str);
        float x, y;
        if (!(iss >> x >> y)) {
            std::cerr << "Failed to parse line: " << line_str << std::endl;
            continue;
        }
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
    auto transformation_matrix = make_transformation_matrix(dx, dy, theta);
    for (const auto& point : points) {
        float new_x = transformation_matrix[0][0] * point.x + transformation_matrix[0][1] * point.y + transformation_matrix[0][2];
        float new_y = transformation_matrix[1][0] * point.x + transformation_matrix[1][1] * point.y + transformation_matrix[1][2];
        moved_points.push_back({new_x, new_y});
    }
    return moved_points;
}

void plot(FILE* gnuplot_pipe, const std::vector<Point>& target, const std::vector<Point>& Source, int iteration) {
    fprintf(gnuplot_pipe, "set size ratio 1\n");
    fprintf(gnuplot_pipe, "set xrange [-20:20]\n");
    fprintf(gnuplot_pipe, "set yrange [-20:20]\n");
    fprintf(gnuplot_pipe, "set title 'Iteration %d'\n", iteration);
    fprintf(gnuplot_pipe, "plot '-' with points pointtype 7 pointsize 1 lc rgb 'blue' title 'Target points','-' with points pointtype 7 pointsize 1 lc rgb 'red' title 'Source points'\n");
    for (const auto& point : target) {
        fprintf(gnuplot_pipe, "%f %f\n", point.x, point.y);
    }
    fprintf(gnuplot_pipe, "e\n");
    for (const auto& point : Source) {
        fprintf(gnuplot_pipe, "%f %f\n", point.x, point.y);
    }
    fprintf(gnuplot_pipe, "e\n");
    fflush(gnuplot_pipe);
}

// 1点だけを変換する関数 
Point get_transformed_point(const Point& p, double tx, double ty, double theta) {
    double c = std::cos(theta);
    double s = std::sin(theta);
    // 回転 → 並進
    return { p.x * c - p.y * s + tx, 
             p.x * s + p.y * c + ty };
}

// データ全体を変換する関数 (正解データ生成や結果確認用)
std::vector<Point> transform_points_all(const std::vector<Point>& points, double dx, double dy, double theta) {
    std::vector<Point> moved_points;
    moved_points.reserve(points.size());
    double c = std::cos(theta);
    double s = std::sin(theta);
    for (const auto& p : points) {
        double nx = p.x * c - p.y * s + dx;
        double ny = p.x * s + p.y * c + dy;
        moved_points.push_back({nx, ny});
    }
    return moved_points;
}

void shuffle_data(std::vector<Point>& points) {
    std::random_device rd;
    std::mt19937 g(rd());
    std::shuffle(points.begin(), points.end(), g);
}

// バッチを作成する関数
std::vector<Point> createBatch(std::vector<Point>& source, size_t& m_current_offset, size_t m_batch_size) {
    std::vector<Point> batch;
    auto target_offset = m_current_offset + m_batch_size;

    while (target_offset >= source.size()) {
        while (m_current_offset < source.size()) {
            batch.push_back(source[m_current_offset++]);
        }
        shuffle_data(source);
        m_current_offset = 0;
        target_offset = target_offset - source.size();
    }
    while (m_current_offset < target_offset) {
        batch.push_back(source[m_current_offset++]);
    }
    return batch;
}

double dist_sq(const Point& a, const Point& b) {
    return (a.x - b.x)*(a.x - b.x) + (a.y - b.y)*(a.y - b.y);
}

// 最近傍点探索
int findClosestPoint(const Point& p, const std::vector<Point>& target) {
    int idx = -1;
    double min_d = std::numeric_limits<double>::max();
    for (size_t i = 0; i < target.size(); ++i) {
        double d = dist_sq(p, target[i]);
        if (d < min_d) { min_d = d; idx = i; }
    }
    return idx;
}

Point get_centroid(const std::vector<Point>& points) {
    double sum_x = 0.0;
    double sum_y = 0.0;
    for (const auto& p : points) {
        sum_x += p.x;
        sum_y += p.y;
    }
    if (points.empty()) return {0,0};
    return {sum_x / points.size(), sum_y / points.size()};
}

// ---------------------------------------------------------
// 数値微分ロジック
// ---------------------------------------------------------

// 1点のペアに対するLoss (距離の2乗)
// ※ source_orig は初期位置の点。これを tx, ty, th で動かして loss を測る
double calc_loss(const Point& target_p, const Point& source_orig, double tx, double ty, double th) {
    Point p_trans = get_transformed_point(source_orig, tx, ty, th);
    double dx = target_p.x - p_trans.x;
    double dy = target_p.y - p_trans.y;
    return dx * dx + dy * dy;
}



// 数値微分を一括で行う関数
void compute_gradients(
    const Point& target_p, const Point& source_orig, const Point& source_curr, 
    double est_x, double est_y, double est_th,
    double& grad_tx, double& grad_ty, double& grad_th
) {
    // 1. 現在のLoss
    double loss_curr = calc_loss(target_p, source_orig, est_x, est_y, est_th);

    // 2. Tx をずらして勾配計算 (Loss(x+d) - Loss(x)) / d
    double loss_tx = calc_loss(target_p, source_orig, est_x + delta, est_y, est_th);
    grad_tx = (loss_tx - loss_curr) / delta;

    // 3. Ty をずらして勾配計算
    double loss_ty = calc_loss(target_p, source_orig, est_x, est_y + delta, est_th);
    grad_ty = (loss_ty - loss_curr) / delta;

    // 4. Theta をずらして勾配計算
    double loss_th = calc_loss(target_p, source_orig, est_x, est_y, est_th + delta);
    grad_th = (loss_th - loss_curr) / delta;
}

float get_absolute_max(const std::vector<Point>& source, const std::vector<Point>& target){
    float min_x = std::numeric_limits<float>::max();
    float max_x = std::numeric_limits<float>::lowest();
    float min_y = std::numeric_limits<float>::max();
    float max_y = std::numeric_limits<float>::lowest();

    auto update_bounds = [&](const std::vector<Point>& points) {
        for (const auto& p : points) {
            if (p.x < min_x) min_x = p.x;
            if (p.x > max_x) max_x = p.x;
            if (p.y < min_y) min_y = p.y;
            if (p.y > max_y) max_y = p.y;
        }
    };

    update_bounds(source);
    update_bounds(target);

    float abs_max = 0.0f;
    abs_max = std::max(abs_max, std::abs(min_x));
    abs_max = std::max(abs_max, std::abs(max_x));
    abs_max = std::max(abs_max, std::abs(min_y));
    abs_max = std::max(abs_max, std::abs(max_y));

    return abs_max;


}
// sgd.cpp に追加

// 点群を正規化して返す関数
// input: 元の点群, 最大絶対値(スケーリング係数)
// output: 正規化された新しい点群
std::vector<Point> normalise_clouds(const std::vector<Point>& cloud, float max_absolute)
{
    // 元の点群をコピーして、新しいベクターを作る
    std::vector<Point> normalised_cloud = cloud;

    // ゼロ除算を防ぐための安全策（念のため）
    if (max_absolute == 0.0f) return normalised_cloud;

    // 全ての点を割り算して縮小
    for(auto& p : normalised_cloud)
    {
        p.x = p.x / max_absolute;
        p.y = p.y / max_absolute;
        // p.z = p.z / max_absolute; // Zがある場合はコメントアウト解除
    }

    return normalised_cloud;
}

// sgd.cpp に追加

// 正規化された変換行列を、現実のスケールに戻す関数
void rescale_transformation_matrix(double &x, double &y, float max_absolute)
{
   
    x = x * max_absolute;
    y = y * max_absolute;
    // 回転(theta)はスケールの影響を受けないので何もしない


    // ※重要: 回転成分 (matrix.a, matrix.b, matrix.c, matrix.d) は
    // 角度に関するものなので、スケールの影響を受けないため触らない！
}


// === SGD ICP 実装 (修正版) ===
void icp_sgd(FILE* pipe, const std::vector<Point>& original_source, const std::vector<Point>& target, double true_dx, double true_dy, double true_theta){
    
    auto icp_start = std::chrono::high_resolution_clock::now();
    std::chrono::nanoseconds total_plot_time(0);

    float max_scale = get_absolute_max(original_source, target);
    std::cout << "Max Scale Factor: " << max_scale << std::endl;

    // 正規化された点群を作成 (値は -1.0 〜 1.0 になる)
    std::vector<Point> norm_source = normalise_clouds(original_source, max_scale);
    std::vector<Point> norm_target = normalise_clouds(target, max_scale);

    // シャッフル用バッファ (中身の座標は初期値のまま不変、順番だけ変える)
    std::vector<Point> shuffle_source = norm_source;
    
    // バッチ処理用のオフセット
    size_t m_current_offset = 0;
    shuffle_data(shuffle_source);

    // ★ パラメータ推定値 (これを更新していく)
    double est_x = 0.0;
    double est_y = 0.0;
    double est_th = 0.0;

    int iter = 0;
    
    std::cout << "Starting SGD-ICP (Parameter Update Mode)..." << std::endl;
    std::cout << "Total Data: " << original_source.size() << ", Batch: " << m_batch_size << std::endl;

    for(iter = 1; iter <= MAX_ITERATION; ++iter){
        
        // 1. バッチ取得 (オリジナル座標の点が入ってくる)
        std::vector<Point> batch = createBatch(shuffle_source, m_current_offset, m_batch_size);

        std::vector<Point> current_batch = transformpoints(batch, est_x, est_y, est_th);

        double sum_grad_Tx = 0;
        double sum_grad_Ty = 0;
        double sum_grad_Theta = 0;

        // 2. 勾配計算
        for(size_t i = 0; i < batch.size(); ++i){
            
            const auto& p_orig = batch[i]; // オリジナル点 (勾配計算用)
            // (A) 対応点探索のために、現在の推定値で一時的に変換する
            const auto& p_curr = current_batch[i]; // 変換済み点 (対応点探索用)

            
            // (B) ターゲットの中から最近傍を探す
            int index = findClosestPoint(p_curr, norm_target);
            Point closest_target = norm_target[index]; 

            // (C) 数値微分 (オリジナルの点 source_p を使うのが重要！)
            double g_tx, g_ty, g_th;
            compute_gradients(closest_target, p_orig, p_curr, est_x, est_y, est_th, g_tx, g_ty, g_th);

            sum_grad_Tx += g_tx;
            sum_grad_Ty += g_ty;
            sum_grad_Theta += g_th;
        }

        // 3. パラメータ更新 (平均勾配 * 学習率)
        double dx = - (sum_grad_Tx / batch.size()) * learning_rate_xy;
        double dy = - (sum_grad_Ty / batch.size()) * learning_rate_xy;
        double dtheta = - (sum_grad_Theta / batch.size()) * learning_rate_th;

        est_x += dx;
        est_y += dy;
        est_th += dtheta;

        // 4. 描画処理
        if (iter % 100 == 0) {
            auto plot_start = std::chrono::high_resolution_clock::now();
            
            // 描画のために、現在の推定パラメータで全点を変換した一時データを作る
            // (original_sourceは不変なので、こうしないと描画が動かない)
            std::vector<Point> plot_source = transform_points_all(norm_source, est_x, est_y, est_th);
            plot(pipe, norm_target, plot_source, iter);
            
            auto plot_end = std::chrono::high_resolution_clock::now();
            total_plot_time += (plot_end - plot_start);
        }

        // 5. 収束判定
        double update_mag = dx*dx + dy*dy + dtheta*dtheta;
        if(update_mag < 1.0e-12) { // 少し厳しめに
            std::cout << "Converged (Update too small) at iter: " << iter << std::endl;
            break;
        }
    }

    // // 最終結果の描画
    // auto plot_start = std::chrono::high_resolution_clock::now();
    // std::vector<Point> final_transformed = transform_points_all(original_source, est_x, est_y, est_th);
    // plot(pipe, target, final_transformed, iter);
    // auto plot_end = std::chrono::high_resolution_clock::now();
    // total_plot_time += (plot_end - plot_start);

   // ... (SGDループ終了) ...

    // 時間計測終了
    auto icp_end = std::chrono::high_resolution_clock::now();
    auto total_duration = icp_end - icp_start;

    // --- 2. 後処理：リスケール (Rescaling) ---
    // 正規化空間の移動量(est_x, est_y)を現実世界の距離に戻す
    double final_est_x = est_x;
    double final_est_y = est_y;
    
    // ここで変換！正規化係数(max_scale)を掛けて元のスケールに戻す
    rescale_transformation_matrix(final_est_x, final_est_y, max_scale); 

    // === 結果評価パート ===
    
    // 【修正点1】推定したパラメータ(final_est_x, final_est_y)を使って、元の点群を変換
    // ※ 以前のコードでは、リスケール前の est_x, est_y を使っていたため、ここがズレていました
    std::vector<Point> final_transformed = transform_points_all(original_source, final_est_x, final_est_y, est_th);
    
    // RMSEの計算
    double sum_sq_error = 0.0;
    for(const auto& p : final_transformed) {
        int index = findClosestPoint(p, target);
        sum_sq_error += dist_sq(p, target[index]);
    }
    double rmse = std::sqrt(sum_sq_error / final_transformed.size());

    // 最終結果の描画
    auto plot_start_final = std::chrono::high_resolution_clock::now();
    plot(pipe, target, final_transformed, iter); // target(現実) と合わせる
    auto plot_end_final = std::chrono::high_resolution_clock::now();
    total_plot_time += (plot_end_final - plot_start_final);
    
    auto pure_compute_time = std::chrono::duration_cast<std::chrono::milliseconds>(total_duration - total_plot_time);

    // 【修正点2】真値との誤差計算 (Error Calculation)
    double error_x = true_dx - final_est_x;
    double error_y = true_dy - final_est_y;
    double error_dist = std::sqrt(error_x * error_x + error_y * error_y); // 並進誤差の距離

    double error_th_rad = true_theta - est_th;
    // 角度の正規化 (-PI ~ PI)
    while (error_th_rad > M_PI) error_th_rad -= 2.0 * M_PI;
    while (error_th_rad < -M_PI) error_th_rad += 2.0 * M_PI;
    double error_th_deg = error_th_rad * 180.0 / M_PI;

    std::cout << "\n========================================" << std::endl;
    std::cout << "      SGD-ICP ACCURACY REPORT           " << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "Iterations     : " << iter - 1 << " / " << MAX_ITERATION << std::endl; // iterはループ抜け時に+1されているため
    std::cout << "Pure Comp Time : " << pure_compute_time.count() << " ms" << std::endl;
    std::cout << "RMSE           : " << rmse << " m" << std::endl;
    std::cout << "----------------------------------------" << std::endl;
    std::cout << "Ground Truth   : dx=" << true_dx << ", dy=" << true_dy << ", th=" << true_theta * 180.0/M_PI << " deg" << std::endl;
    std::cout << "Estimated      : dx=" << final_est_x << ", dy=" << final_est_y << ", th=" << est_th * 180.0/M_PI << " deg" << std::endl;
    std::cout << "----------------------------------------" << std::endl;
    std::cout << "Error (dx)     : " << error_x << " m" << std::endl;
    std::cout << "Error (dy)     : " << error_y << " m" << std::endl;
    std::cout << "Error (Trans)  : " << error_dist << " m (Euclidean Dist)" << std::endl;
    std::cout << "Error (Theta)  : " << error_th_deg << " deg" << std::endl;
    std::cout << "========================================\n" << std::endl;
}

int main() {

    std::vector<Point> target_ = read_scan_points("scan_1.txt");
    std::vector<Point> source = target_;
    
    double true_dx = -0.1, true_dy = 0, true_theta = 0; 
    std::vector<Point> target = transformpoints(target_, true_dx, true_dy, true_theta);
    
    // 3. 実行 (popenでパイプを開く)
    FILE* pipe = popen("gnuplot -persistent", "w");
    
    // std::ofstream gnuplot_script; // ← これは不要なので削除してOKです

    if (pipe == NULL) {
        std::cerr << "Error: Could not open pipe to gnuplot." << std::endl;
        return -1;
    }

    // 初期状態のプロット
    std::cout << "Plotting Initial State..." << std::endl;
    
    // ★修正箇所: pipeを渡し、引数の数を合わせる
    plot(pipe, target, source, 0);

    // 一時停止
    std::cout << "Press [Enter] key to start..." << std::endl;
    std::cin.get(); 
    
    icp_sgd(pipe, source, target, true_dx, true_dy, true_theta);
    
    if(pipe) pclose(pipe);
    return 0;
}