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

// 実験結果を保存する構造体
struct ExperimentResult {
    double true_dx, true_dy, true_theta; // 正解値
    double est_dx, est_dy, est_theta;    // 推定値
    double error_trans, error_theta_deg; // 誤差
    int iterations;                      // かかった反復回数
    double time_ms;                      // 計算時間
    bool converged;                      // 収束したか
};

// ICP関数の戻り値を void から ExperimentResult に変更
// visual_mode フラグを追加 (trueなら描画、falseなら高速計算のみ)
ExperimentResult icp_sgd(FILE* pipe, const std::vector<Point>& original_source, const std::vector<Point>& target, double true_dx, double true_dy, double true_theta, bool visual_mode);

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

// 範囲固定を外して、正規化データでも元データでも自動で拡大縮小されるようにしたplot関数
void plot(FILE* gnuplot_pipe, const std::vector<Point>& target, const std::vector<Point>& Source, int iteration) {
    if (gnuplot_pipe == nullptr) return;

    fprintf(gnuplot_pipe, "set size ratio 1\n");
    // 自動縮尺にするため、xrange, yrange の指定を削除しました
    fprintf(gnuplot_pipe, "set autoscale\n"); 
    fprintf(gnuplot_pipe, "set title 'Iteration %d'\n", iteration);
    fprintf(gnuplot_pipe, "plot '-' with points pointtype 7 pointsize 0.5 lc rgb 'blue' title 'Target','-' with points pointtype 7 pointsize 0.5 lc rgb 'red' title 'Source'\n");
    
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
ExperimentResult icp_sgd(FILE* pipe, const std::vector<Point>& original_source, const std::vector<Point>& target, double true_dx, double true_dy, double true_theta, bool visual_mode){
    
    auto icp_start = std::chrono::high_resolution_clock::now();
    std::chrono::nanoseconds total_plot_time(0);

    float max_scale = get_absolute_max(original_source, target);
    if(visual_mode) std::cout << "Max Scale Factor: " << max_scale << std::endl;

    // 正規化
    std::vector<Point> norm_source = normalise_clouds(original_source, max_scale);
    std::vector<Point> norm_target = normalise_clouds(target, max_scale);
    std::vector<Point> shuffle_source = norm_source;
    
    size_t m_current_offset = 0;
    shuffle_data(shuffle_source);

    double est_x = 0.0, est_y = 0.0, est_th = 0.0;
    int iter = 0;
    bool is_converged = false;

    // SGD Loop
    for(iter = 1; iter <= MAX_ITERATION; ++iter){
        std::vector<Point> batch = createBatch(shuffle_source, m_current_offset, m_batch_size);
        std::vector<Point> current_batch = transformpoints(batch, est_x, est_y, est_th);

        double sum_grad_Tx = 0, sum_grad_Ty = 0, sum_grad_Theta = 0;

        for(size_t i = 0; i < batch.size(); ++i){
            const auto& p_orig = batch[i];
            const auto& p_curr = current_batch[i];
            int index = findClosestPoint(p_curr, norm_target);
            Point closest_target = norm_target[index]; 

            double g_tx, g_ty, g_th;
            compute_gradients(closest_target, p_orig, p_curr, est_x, est_y, est_th, g_tx, g_ty, g_th);
            sum_grad_Tx += g_tx;
            sum_grad_Ty += g_ty;
            sum_grad_Theta += g_th;
        }

        double dx = - (sum_grad_Tx / batch.size()) * learning_rate_xy;
        double dy = - (sum_grad_Ty / batch.size()) * learning_rate_xy;
        double dtheta = - (sum_grad_Theta / batch.size()) * learning_rate_th;

        est_x += dx;
        est_y += dy;
        est_th += dtheta;

        // ★修正点: 毎回描画する (visual_mode時)
        // ※ループ中は「正規化されたデータ」でプロットします
        if (visual_mode && pipe != nullptr) {
            auto plot_start = std::chrono::high_resolution_clock::now();
            
            // 現在の正規化パラメータで変換した点群を作成
            std::vector<Point> plot_source = transform_points_all(norm_source, est_x, est_y, est_th);
            
            // Gnuplotへ送信 (autoscaleなので -1~1 の範囲で綺麗に映ります)
            plot(pipe, norm_target, plot_source, iter);

            // 速すぎて見えない場合は、ここに usleep(10000); // 10ms待機 などを入れてください
            
            auto plot_end = std::chrono::high_resolution_clock::now();
            total_plot_time += (plot_end - plot_start);
        }

        // 収束判定
        if(dx*dx + dy*dy + dtheta*dtheta < 1.0e-6) {
            is_converged = true;
            break;
        }
    }

    auto icp_end = std::chrono::high_resolution_clock::now();
    auto total_duration = icp_end - icp_start - total_plot_time;
    double time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(total_duration).count();

    // --- 後処理：リスケール ---
    double final_est_x = est_x;
    double final_est_y = est_y;
    rescale_transformation_matrix(final_est_x, final_est_y, max_scale); 

    // --- 誤差計算 ---
    double error_x = true_dx - final_est_x;
    double error_y = true_dy - final_est_y;
    double error_dist = std::sqrt(error_x * error_x + error_y * error_y);

    double error_th_rad = true_theta - est_th;
    while (error_th_rad > M_PI) error_th_rad -= 2.0 * M_PI;
    while (error_th_rad < -M_PI) error_th_rad += 2.0 * M_PI;
    double error_th_deg = error_th_rad * 180.0 / M_PI;

    // ★修正点: 最後だけ「元のスケール」に戻して描画する
    if (visual_mode && pipe != nullptr) {
        // 推定されたパラメータ(final_est_x, final_est_y, est_th)で、
        // オリジナルの点群(original_source)を変換
        std::vector<Point> final_transformed = transform_points_all(original_source, final_est_x, final_est_y, est_th);
        
        // 元のターゲット(target) と合わせて描画
        // plot関数が autoscale なので、自動的に数十メートルの範囲に拡大されます
        plot(pipe, target, final_transformed, iter); 
        
        std::cout << "Final plot displayed in original scale." << std::endl;
    }

    return {true_dx, true_dy, true_theta, final_est_x, final_est_y, est_th, error_dist, std::abs(error_th_deg), iter, time_ms, is_converged};
}

// ベンチマーク実行関数 (修正版: 複数回試行対応)
void run_benchmark(const std::vector<Point>& source_data, bool is_sgd) {
    std::string filename = is_sgd ? "sgdicp_x_multi.csv" : "icp_x_multi.csv";
    std::cout << "Starting Benchmark: " << (is_sgd ? "SGD-ICP" : "Standard ICP") 
              << " -> " << filename << std::endl;
    
    std::ofstream csv(filename);
    // ヘッダー (Trial列を追加してもいいですが、解析側で勝手に集計してくれるのでこのままでもOK)
    csv << "True_DX,True_DY,True_Theta_Deg,Est_DX,Est_DY,Est_Theta_Deg,Error_Trans,Error_Theta,Iter,Time_ms,Converged" << std::endl;

    // ★ 試行回数の設定
    // SGDならばらつきを見るために20回、ICPなら確定的なので1回
    int num_trials = is_sgd ? 20 : 1; 

    // 実験条件: X方向のズレ (-1.0m 〜 +1.0m, 0.1m刻み)
    std::vector<double> test_dxs;
    for(int i = -10; i <= 10; ++i) {
        test_dxs.push_back(i * 0.1); 
    }
    double true_theta = 0.0;
    double true_dy = 0.0;
    std::ofstream dummy_script; // 描画しない

    int total_steps = test_dxs.size() * num_trials;
    int current_step = 0;

    for (double true_dx : test_dxs) {
        for(int t = 0; t < num_trials; ++t) { // ★ ここにループを追加！
            current_step++;
            
            // 進捗表示
            std::cout << "\rProgress: " << current_step << "/" << total_steps 
                      << " [dx=" << std::fixed << std::setprecision(1) << true_dx 
                      << ", trial=" << t+1 << "]   " << std::flush;

            // ターゲット生成
            std::vector<Point> target = transformpoints(source_data, true_dx, true_dy, true_theta);

            // ICP実行 (visual_mode = false)
            // std::ofstream dummy_script; // 不要なので削除またはコメントアウト
    // ... (中略)
            // ICP実行 (visual_mode = false)
            // 第一引数を nullptr に変更
            ExperimentResult res = icp_sgd(nullptr, source_data, target, true_dx, true_dy, true_theta, false);
            // CSV書き込み
            csv << res.true_dx << "," << res.true_dy << "," << res.true_theta * 180.0/M_PI << ","
                << res.est_dx << "," << res.est_dy << "," << res.est_theta * 180.0/M_PI << ","
                // ここを error_theta_deg に変更
                << res.error_trans << "," << res.error_theta_deg << ","
                << res.iterations << "," << res.time_ms << "," << res.converged << std::endl;
        }
    }
    std::cout << "\nBenchmark finished! Saved to " << filename << std::endl;
    csv.close();
}

// main関数内での呼び出しイメージ
// run_benchmark(source, true);  // SGD-ICP (20回試行)
// run_benchmark(source, false); // Standard ICP (1回試行)

int main() {
    std::vector<Point> source = read_scan_points("scan_1.txt");
    if(source.empty()) return -1;

    std::cout << "Select Mode:\n 1: Single Run (with Plot)\n 2: Benchmark (CSV output, No Plot)\n> ";
    int mode;
    std::cin >> mode;
    // バッファクリア
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

    if (mode == 2) {
        // まとめて実験モード
        run_benchmark(source, true);
    } 
    else {
        // 今まで通りの単発実行モード
        double true_dx = 0, true_dy = 0, true_theta = 10*M_PI/180;
        // 必要ならここでユーザーに入力させることも可能
        // std::cout << "Enter true_dx: "; std::cin >> true_dx; ...

        std::vector<Point> target = transformpoints(source, true_dx, true_dy, true_theta);
        
        FILE* pipe = popen("gnuplot -persistent", "w");
        if (!pipe) { std::cerr << "Gnuplot error" << std::endl; return -1; }

        std::cout << "Press [Enter] to start..." << std::endl;
        std::cin.get();

        // visual_mode = true
        ExperimentResult res = icp_sgd(pipe, source, target, true_dx, true_dy, true_theta, true);
        
        // 結果表示
        std::cout << "Result: Error Trans=" << res.error_trans << "m, Error Th=" << res.error_theta_deg << "deg" << std::endl;

        pclose(pipe);
    }

    return 0;
}