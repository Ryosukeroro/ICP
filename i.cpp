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
#include "point.h"
#include "constants.h"

// 実験結果を保存する構造体
struct ExperimentResult {
    double true_dx, true_dy, true_theta; // 正解値
    double est_dx, est_dy, est_theta;    // 推定値
    double error_trans, error_theta;     // 誤差
    int iterations;                      // かかった反復回数
    double time_ms;                      // 計算時間
    bool converged;                      // 収束したか
};

std::vector<Point> read_scan_points(const std::string& file_path){
    std::ifstream file(file_path);
    std::vector<Point> points;
    if (!file.is_open()) {
        std::cerr << "File could not be opened." << std::endl;
        return points;
    }
    std::string line_str;
    while(std::getline(file, line_str)){
        std::istringstream iss(line_str);
        float x,y;
        if(!(iss >> x >> y)){
            continue;
        }
        points.push_back({x,y});
    }
    return points;
}

std::array<std::array<float, 3>, 3> make_transformation_matrix(float tx, float ty, float theta){
    return {{
        {std::cos(theta), -std::sin(theta), tx},
        {std::sin(theta),  std::cos(theta), ty},
        {0.0f,             0.0f,            1.0f}
    }};
}

std::vector<Point> transformpoints(const std::vector<Point>& points, float dx, float dy, float theta){
    std::vector<Point> moved_points;
    auto transformation_matrix = make_transformation_matrix(dx, dy, theta);
    for(const auto& point : points){
        float new_x = transformation_matrix[0][0] * point.x + transformation_matrix[0][1] * point.y + transformation_matrix[0][2];
        float new_y = transformation_matrix[1][0] * point.x + transformation_matrix[1][1] * point.y + transformation_matrix[1][2];
        moved_points.push_back({new_x, new_y});
    }
    return moved_points;
}

// 点群の重心(中心座標)を計算する関数
Point get_centroid(const std::vector<Point>& points) {
    double sum_x = 0.0;
    double sum_y = 0.0;
    for (const auto& p : points) {
        sum_x += p.x;
        sum_y += p.y;
    }
    if (points.empty()) return {0.0, 0.0};
    return {sum_x / points.size(), sum_y / points.size()};
}

void plot (std::ofstream& gnuplot_script, const std::vector<Point>& target, const std::vector<Point>& Source, bool block, int iteration){
    gnuplot_script.open("plot_commands.gp");
    gnuplot_script << "set size ratio 1\n";
    gnuplot_script << "set xrange [-20:20]\n";
    gnuplot_script << "set yrange [-20:20]\n";
    gnuplot_script << "set title 'Iteration " << iteration << "'\n";
    gnuplot_script << "plot '-' with points pointtype 7 pointsize 1 lc rgb 'blue' title 'Target', '-' with points pointtype 7 pointsize 1 lc rgb 'red' title 'Source'\n";
    for(const auto& point : target){
        gnuplot_script << point.x << " " << point.y << "\n";
    }
    gnuplot_script << "e\n";
    for(const auto& point : Source){
        gnuplot_script << point.x << " " << point.y << "\n";
    }
    gnuplot_script << "e\n";
    gnuplot_script.flush();
    gnuplot_script.close();
    system("gnuplot -p plot_commands.gp");
}

float distance_sq(const Point& points, const Point& point){
    return (points.x - point.x) * (points.x - point.x) + (points.y - point.y) * (points.y - point.y);
}

int findClosestPoint(const Point& point, const std::vector<Point>& target){
    int Index = -1;
    float minDist = std::numeric_limits<float>::max();
    for(size_t i = 0; i < target.size(); ++i){
        float dist = distance_sq(target[i], point);
        if(dist < minDist){
            minDist = dist;
            Index = i;
        }
    }
    return Index;
}

// 数値微分関数群
float diffx(Point Target, Point Source){
    double fx_delta = (Target.x - (Source.x + delta)) * (Target.x - (Source.x + delta)) + (Target.y - Source.y) * (Target.y - Source.y);
    double fx = (Target.x - Source.x) * (Target.x - Source.x) + (Target.y - Source.y) * (Target.y - Source.y);
    return (fx_delta - fx) / delta;
}

float diffy(Point Target, Point Source){
    double fx_delta = (Target.x - Source.x) * (Target.x - Source.x) + (Target.y - (Source.y + delta)) * (Target.y - (Source.y + delta));
    double fx = (Target.x - Source.x) * (Target.x - Source.x) + (Target.y - Source.y) * (Target.y - Source.y);
    return (fx_delta - fx) / delta;
}

float difftheta(Point Target, Point Source){
    // 回転(原点周り)をdeltaだけずらした時の座標
    double cos_d = cos(delta);
    double sin_d = sin(delta);
    double x_rot = Source.x * cos_d - Source.y * sin_d;
    double y_rot = Source.x * sin_d + Source.y * cos_d;

    // ずらした後の誤差 - 現在の誤差
    double fx_delta = (Target.x - x_rot) * (Target.x - x_rot) + (Target.y - y_rot) * (Target.y - y_rot);
    double fx = (Target.x - Source.x) * (Target.x - Source.x) + (Target.y - Source.y) *(Target.y - Source.y);
    
    return (fx_delta - fx) / delta;
}

/// ==========================================
// 1. ヘルパー関数群 (SGDの時と同じもの)
// ==========================================
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

std::vector<Point> normalise_clouds(const std::vector<Point>& cloud, float max_absolute) {
    std::vector<Point> normalised_cloud = cloud;
    if (max_absolute == 0.0f) return normalised_cloud;
    for(auto& p : normalised_cloud) {
        p.x = p.x / max_absolute;
        p.y = p.y / max_absolute;
    }
    return normalised_cloud;
}

void rescale_transformation_matrix(double &x, double &y, float max_absolute) {
    x = x * max_absolute;
    y = y * max_absolute;
}

// データ全体を変換する関数（結果描画用）
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

// ==========================================
// 2. 修正版 ICP関数 (正規化ロジック追加)
// ==========================================
ExperimentResult icp_scan_matching(std::ofstream& gnuplot_script, const std::vector<Point>& original_source, const std::vector<Point>& target, double true_dx, double true_dy, double true_theta, bool visual_mode){
    
    auto icp_start = std::chrono::high_resolution_clock::now();
    std::chrono::milliseconds total_plot_time(0);

    // ★ 1. 正規化の準備
    float max_scale = get_absolute_max(original_source, target);
    if(visual_mode) std::cout << "Max Scale Factor: " << max_scale << std::endl;

    // ★ 2. データを正規化 (-1.0 ~ 1.0 に変換)
    // これをしないと、座標の値がデカすぎて勾配計算が暴走します
    std::vector<Point> norm_target = normalise_clouds(target, max_scale);
    std::vector<Point> norm_source = normalise_clouds(original_source, max_scale);

    // 計算用の変数は正規化された空間で動かす
    std::vector<Point> transformed_source = norm_source;
    
    double previous_error_sum = std::numeric_limits<double>::max();
    double est_x = 0.0;
    double est_y = 0.0;
    double est_th = 0.0;
    
    int iter = 0;
    bool is_converged = false;

    for(iter = 1; iter <= MAX_ITERATION; ++iter){
        double error_sum = 0;
        double grad_Tx = 0;
        double grad_Ty = 0;
        double grad_Theta = 0;
        
        // 全点探索 (Standard ICP)
        for(const auto& current_source : transformed_source ){
            int index = findClosestPoint(current_source, norm_target); // 正規化ターゲットに対して探索
            Point closest_target = norm_target[index];

            double e_x = closest_target.x - current_source.x;
            double e_y = closest_target.y - current_source.y;
            error_sum += e_x * e_x + e_y * e_y;

            grad_Tx += diffx(closest_target, current_source);
            grad_Ty += diffy(closest_target, current_source);
            grad_Theta += difftheta(closest_target, current_source);
        }

        int num_points = transformed_source.size();

        // 勾配降下
        double dx = - (grad_Tx / num_points) * learning_rate_xy;
        double dy = - (grad_Ty / num_points) * learning_rate_xy;
        double dtheta = - (grad_Theta / num_points) * learning_rate_th;

        // 座標更新
        double cos_th = cos(dtheta);
        double sin_th = sin(dtheta);

        // 推定パラメータの更新 (累積)
        double new_est_x = est_x * cos_th - est_y * sin_th;
        double new_est_y = est_x * sin_th + est_y * cos_th;
        est_x = new_est_x + dx;
        est_y = new_est_y + dy;
        est_th += dtheta;

        // 点群の更新 (変形)
        for(auto& p: transformed_source){
            double x_new = p.x * cos_th - p.y * sin_th;
            double y_new = p.x * sin_th + p.y * cos_th;
            p.x = x_new + dx;
            p.y = y_new + dy;
        }

        // 描画 (Visual Modeのみ)
        if (visual_mode) {
            auto plot_start = std::chrono::high_resolution_clock::now();
            // 描画中は正規化された座標で見ることになります（形は同じ）
            plot(gnuplot_script, norm_target, transformed_source, true, iter);
            auto plot_end = std::chrono::high_resolution_clock::now();
            total_plot_time += std::chrono::duration_cast<std::chrono::milliseconds>(plot_end - plot_start);
        }

        // 収束判定
        double update_sq_norm = (dx * dx) + (dy * dy) + (dtheta *dtheta);
        if (update_sq_norm < 1e-5) { // 正規化されているので閾値は小さくてOK
            if(visual_mode) std::cout << "Converged. Update norm is tiny." << std::endl;
            is_converged = true;
            break;
        }
        
        if(std::abs(previous_error_sum - error_sum) < 1e-5){
            if(visual_mode) std::cout << "Converged by error diff." << std::endl;
            is_converged = true;
            break;
        }
        previous_error_sum = error_sum;
    }

    auto icp_end = std::chrono::high_resolution_clock::now();
    auto total_duration = icp_end - icp_start - total_plot_time;
    double time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(total_duration).count();

    // ★ 3. 後処理：結果をリアルスケールに戻す
    double final_est_x = est_x;
    double final_est_y = est_y;
    rescale_transformation_matrix(final_est_x, final_est_y, max_scale);

    // ここからは評価用計算（リアルスケールで行う）
    
    // 推定されたパラメータで元の点群を変換
    std::vector<Point> final_transformed = transform_points_all(original_source, final_est_x, final_est_y, est_th);

    // RMSE計算
    double sum_sq_error = 0.0;
    for(const auto& p : final_transformed) {
        int index = findClosestPoint(p, target);
        Point closest = target[index];
        double dist_sq = (p.x - closest.x)*(p.x - closest.x) + (p.y - closest.y)*(p.y - closest.y);
        sum_sq_error += dist_sq;
    }
    double mse = sum_sq_error / final_transformed.size();
    double rmse = std::sqrt(mse);

    // 推定誤差計算
double trans_err_x = true_dx - final_est_x; 
    double trans_err_y = true_dy - final_est_y;
    double trans_err_total = std::sqrt(trans_err_x*trans_err_x + trans_err_y*trans_err_y);

    double err_th_rad = true_theta - est_th; // 回転も同様
    double err_th_deg = err_th_rad * 180.0 / M_PI;

    // レポート出力
    if (visual_mode) {
        double true_theta_deg = true_theta * 180.0 / M_PI;
        // 最終的な合わせ結果をプロット（リアルスケール）
        plot(gnuplot_script, target, final_transformed, true, iter);

        std::cout << "\n========================================" << std::endl;
        std::cout << "         ICP (Normalized) REPORT        " << std::endl;
        std::cout << "========================================" << std::endl;
        std::cout << "Experiment Conditions (True Shift):" << std::endl;
        std::cout << "  X Axis         : " << true_dx << " m" << std::endl;
        std::cout << "  Y Axis         : " << true_dy << " m" << std::endl;
        std::cout << "  Theta          : " << true_theta_deg << " deg" << std::endl;

        std::cout << "\nConvergence Info:" << std::endl;
        std::cout << "  Iterations     : " << iter << std::endl;
        std::cout << "  Compute Time   : " << time_ms << " ms" << std::endl;
        
        std::cout << "\nAccuracy Metrics (Residual Error):" << std::endl;
        std::cout << "  Rotation Error : " << err_th_deg << " deg" << std::endl;
        std::cout << "  RMSE (Points)  : " << rmse << " m" << std::endl; 
        std::cout << "  Trans Error 2D : " << trans_err_total << " m" << std::endl;
        std::cout << "========================================\n" << std::endl;
    }
    
    return {true_dx, true_dy, true_theta, final_est_x, final_est_y, est_th, trans_err_total, err_th_deg, iter, time_ms, is_converged};
}
// ベンチマーク実行関数
void run_benchmark(const std::vector<Point>& source_data) {
    std::cout << "Starting Benchmark Mode (Scan X: -1.0m ~ +1.0m, Theta: 0 deg)..." << std::endl;
    
    // CSVファイルを開く
    std::ofstream csv("benchmark_result_icp.csv");
    // ヘッダー書き込み
    csv << "True_DX,True_DY,True_Theta_Deg,Est_DX,Est_DY,Est_Theta_Deg,Error_Trans,Error_Theta,Iter,Time_ms,Converged" << std::endl;

    // ★ 実験条件: X方向のズレ (-1.0m 〜 +1.0m, 0.1m刻み)
    std::vector<double> test_dxs;
    for(int i = -10; i <= 10; ++i) {
        test_dxs.push_back(i * 0.1); 
    }

    // 角度は 0度 に固定
    double true_theta = 0.0; 
    double true_dy = 0.0; // Yは0固定（必要に応じて変更）

    // ダミーのストリーム（ベンチマーク時は描画しないため）
    std::ofstream dummy_script;

    int total_tests = test_dxs.size();
    int current_test = 0;

    // ループ実行
    for (double true_dx : test_dxs) {
        current_test++;

        // ターゲット（正解データ）を生成
        std::vector<Point> target = transformpoints(source_data, true_dx, true_dy, true_theta);

        // 進捗表示
        std::cout << "\rRunning test " << current_test << "/" << total_tests 
                  << " [dx=" << std::fixed << std::setprecision(1) << true_dx << "m]... " << std::flush;
        
        // ICP実行 (visual_mode = false)
        ExperimentResult res = icp_scan_matching(dummy_script, source_data, target, true_dx, true_dy, true_theta, false);

        // CSVに書き込み
        csv << res.true_dx << "," << res.true_dy << "," << res.true_theta * 180.0/M_PI << ","
            << res.est_dx << "," << res.est_dy << "," << res.est_theta * 180.0/M_PI << ","
            << res.error_trans << "," << res.error_theta << ","
            << res.iterations << "," << res.time_ms << "," << res.converged << std::endl;
    }
    std::cout << "\nBenchmark finished! Saved to 'benchmark_result_icp.csv'" << std::endl;
    csv.close();
}
// ==========================================
// Y軸方向のベンチマーク関数 (新規追加)
// ==========================================
void run_benchmark_y(const std::vector<Point>& source_data) {
    std::cout << "Starting Benchmark Mode (Scan Y: -1.0m ~ +1.0m, Theta: 0 deg)..." << std::endl;
    
    // CSVファイルを開く (ファイル名を変更)
    std::ofstream csv("benchmark_result_icp_y.csv");
    // ヘッダー書き込み
    csv << "True_DX,True_DY,True_Theta_Deg,Est_DX,Est_DY,Est_Theta_Deg,Error_Trans,Error_Theta,Iter,Time_ms,Converged" << std::endl;

    // ★ 実験条件: Y方向のズレ (-1.0m 〜 +1.0m, 0.1m刻み)
    std::vector<double> test_dys;
    for(int i = -10; i <= 10; ++i) {
        test_dys.push_back(i * 0.1); 
    }

    // Xと角度は 0 に固定
    double true_dx = 0.0; 
    double true_theta = 0.0;

    // ダミーのストリーム
    std::ofstream dummy_script;

    int total_tests = test_dys.size();
    int current_test = 0;

    // ループ実行
    for (double true_dy : test_dys) {
        current_test++;

        // ターゲット（正解データ）を生成 (X=0, Y=変化, Theta=0)
        std::vector<Point> target = transformpoints(source_data, true_dx, true_dy, true_theta);

        // 進捗表示
        std::cout << "\rRunning test " << current_test << "/" << total_tests 
                  << " [dy=" << std::fixed << std::setprecision(1) << true_dy << "m]... " << std::flush;
        
        // ICP実行 (visual_mode = false)
        ExperimentResult res = icp_scan_matching(dummy_script, source_data, target, true_dx, true_dy, true_theta, false);

        // CSVに書き込み
        csv << res.true_dx << "," << res.true_dy << "," << res.true_theta * 180.0/M_PI << ","
            << res.est_dx << "," << res.est_dy << "," << res.est_theta * 180.0/M_PI << ","
            << res.error_trans << "," << res.error_theta << ","
            << res.iterations << "," << res.time_ms << "," << res.converged << std::endl;
    }
    std::cout << "\nBenchmark finished! Saved to 'benchmark_result_icp_y.csv'" << std::endl;
    csv.close();
}
// ==========================================
// 回転(Theta)方向のベンチマーク関数 (新規追加)
// ==========================================
void run_benchmark_theta(const std::vector<Point>& source_data) {
    std::cout << "Starting Benchmark Mode (Scan Theta: -90 deg ~ +90 deg, Trans: 0 m)..." << std::endl;
    
    // CSVファイルを開く
    std::ofstream csv("benchmark_result_icp_theta.csv");
    // ヘッダー書き込み
    csv << "True_DX,True_DY,True_Theta_Deg,Est_DX,Est_DY,Est_Theta_Deg,Error_Trans,Error_Theta,Iter,Time_ms,Converged" << std::endl;

    // ★ 実験条件: 回転方向のズレ (-90度 〜 +90度, 2度刻み)
    // ※ ICPは初期角度ズレが大きいと局所解に陥りやすいため、どこまで耐えられるかを確認します
    std::vector<double> test_thetas_deg;
    for(int i = -50; i <= 50; i += 5) {
        test_thetas_deg.push_back((double)i);
    }

    // XとYは 0 に固定
    double true_dx = 0.0; 
    double true_dy = 0.0;

    // ダミーのストリーム
    std::ofstream dummy_script;

    int total_tests = test_thetas_deg.size();
    int current_test = 0;

    // ループ実行
    for (double deg : test_thetas_deg) {
        current_test++;
        
        // 度数法 -> ラジアン変換
        double true_theta = deg * M_PI / 180.0;

        // ターゲット（正解データ）を生成 (X=0, Y=0, Theta=変化)
        std::vector<Point> target = transformpoints(source_data, true_dx, true_dy, true_theta);

        // 進捗表示
        std::cout << "\rRunning test " << current_test << "/" << total_tests 
                  << " [theta=" << std::fixed << std::setprecision(1) << deg << " deg]... " << std::flush;
        
        // ICP実行 (visual_mode = false)
        ExperimentResult res = icp_scan_matching(dummy_script, source_data, target, true_dx, true_dy, true_theta, false);

        // CSVに書き込み
        csv << res.true_dx << "," << res.true_dy << "," << res.true_theta * 180.0/M_PI << ","
            << res.est_dx << "," << res.est_dy << "," << res.est_theta * 180.0/M_PI << ","
            << res.error_trans << "," << res.error_theta << ","
            << res.iterations << "," << res.time_ms << "," << res.converged << std::endl;
    }
    std::cout << "\nBenchmark finished! Saved to 'benchmark_result_icp_theta.csv'" << std::endl;
    csv.close();
}
// ==========================================
// Main 関数の修正
// ==========================================
// ==========================================
// Main 関数の修正
// ==========================================
int main(void){
    std::vector<Point> target = read_scan_points("scan_1.txt");
    std::vector<Point> source = target;

    // メニュー表示の更新
    std::cout << "Select Mode:\n";
    std::cout << " 1: Single Run (with Plot)\n";
    std::cout << " 2: Benchmark X-Axis (CSV output)\n";
    std::cout << " 3: Benchmark Y-Axis (CSV output)\n";
    std::cout << " 4: Benchmark Rotation (CSV output)\n"; // ★ここを追加
    std::cout << "> ";
    
    int mode;
    std::cin >> mode;
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n'); // バッファクリア

    if (mode == 2) {
        // ベンチマークモード (X軸)
        run_benchmark(source);
    } 
    else if (mode == 3) {
        // ベンチマークモード (Y軸)
        run_benchmark_y(source);
    }
    else if (mode == 4) {
        // ベンチマークモード (回転) ★ここを追加
        run_benchmark_theta(source);
    }
    else {
        // 通常モード
        // 初期ズレ量 (動作確認用に回転を入れてみる)
        float true_dx = 0;
        float true_dy = 0;
        float true_theta = -40.0 * M_PI / 180.0; // 例: 30度ずらす

        std::vector<Point> moved_target = transformpoints(target, true_dx, true_dy, true_theta);
        std::vector<Point> Target = moved_target;

        std::ofstream gnuplot_script;

        // 初期状態のプロット
        std::cout << "Plotting Initial State..." << std::endl;
        plot(gnuplot_script, Target, source, true, 0);

        // 一時停止
        std::cout << "Press [Enter] key to start..." << std::endl;
        std::cin.get(); 

        // 計算開始 (visual_mode = true)
        icp_scan_matching(gnuplot_script, source, Target, true_dx, true_dy, true_theta, true);
    }

    return 0;
}