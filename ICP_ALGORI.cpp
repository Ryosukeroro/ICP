// // //std::string linesToSave_dx[MAX_ItERATION];
// // /*点群数*/
// // const int numPoints1 = 723;
// // const int numPoints2 = 737;

// // float random_x;
// // float random_y;
















// std::vector<Point> read_scan_points(const std::string& file_path){
//     std::ifstream file(file_path);
//     std::vector<Point> points;
//     if (!file.is_open()) {
//         std::cerr << "File could not be opened." << std::endl;
//         throw std::runtime_error("Error File could not be opened at path: " + file_path);
//         return points;
//     }
//     std::string line_str;
//     while(std::getline(file, line_str)){
//         std::istringstream iss(line_str);
//         double x,y;
//         if(!(iss >> x >> y)){
//             std::cerr << "Failed to parse line: " << line_str << std::endl;
//             continue;
//         }
//         points.push_back({x,y});
//     }
//     return points;
// }

// Point calculate_average(const std::vector<Point>& points){
//     float sum_x = 0.0f;
//     float sum_y = 0.0f;
//     for(const auto& point : points){
//         sum_x += point.x;
//         sum_y += point.y;
//     }
//     float avg_x = sum_x / points.size();
//     float avg_y = sum_y / points.size();
//     return {avg_x, avg_y};
// }

// std::array<std::array<float, 3>, 3> make_transformation_matrix(float tx, float ty, float theta){
//     return {{
//         {std::cos(theta), -std::sin(theta), tx},
//         {std::sin(theta),  std::cos(theta), ty},
//         {0.0f,             0.0f,            1.0f}
//     }};
// }

// std::vector<Point> transformpoints(const std::vector<Point>& points, float dx, float dy, float theta){
//     std::vector<Point> moved_points;
//     auto transformation_matrix = make_transformation_matrix(dx, dy, theta);
//     for(const auto& point : points){
//         float new_x = transformation_matrix[0][0] * point.x + transformation_matrix[0][1] * point.y + transformation_matrix[0][2];
//         float new_y = transformation_matrix[1][0] * point.x + transformation_matrix[1][1] * point.y + transformation_matrix[1][2];
//         moved_points.push_back({new_x, new_y});
//     }
//     return moved_points;
// }

// void plot (std::ofstream& gnuplot_script, const std::vector<Point>& target, const std::vector<Point>& Source, bool block,int iteration){
//     gnuplot_script.open("plot_commands.gp");
//     gnuplot_script << "set size ratio 1\n";
//     gnuplot_script << "set xrange [-20:20]\n";
//     gnuplot_script << "set xrange [-20:20]\n";
//     gnuplot_script << "set yrange [-20:20]\n";
//     gnuplot_script << "set title 'Iteration " << iteration << "'\n"; // 現在のループ回数を表示
//     gnuplot_script << "plot '-' with points pointtype 7 pointsize 1 lc rgb 'blue' title 'Target points', '-' with points pointtype 7 pointsize 1 lc rgb 'red' title 'Source points'\n";
//     for(const auto& point : target){
//         gnuplot_script << point.x << " " << point.y << "\n";
//     }
//     gnuplot_script << "e\n";
//     for(const auto& point : Source){
//         gnuplot_script << point.x << " " << point.y << "\n";
//     }
//     gnuplot_script << "e\n";
//      //gnuplot_script << "pause -1\n";  // プロットを更新するために一時停止
//     gnuplot_script.flush(); // スクリプトをフラッシュして即時反映させる
//     gnuplot_script << "set size ratio 1\n";
//      gnuplot_script.close();
//     // gnuplot_script.close();
//      std::string gnuplot_command = "gnuplot -p plot_commands.gp";
//     // /*if (block) {
//     //     std::cout << "Press Enter to continue...";
//     // std::cin.ignore(); // 
//      system(gnuplot_command.c_str());
// }

// double distance_sq(const Point& points, const Point& point){
//     return (points.x - point.x) * (points.x - point.x) + (points.y - point.y) * (points.y - point.y);

// }

// int findClosestPoint(const Point& point, const std::vector<Point>& target){
//     int Index = -1;
//     float minDist = std::numeric_limits<float>::max();
//     for(size_t i = 0; i < target.size(); ++i){
//         float dist = distance_sq(target[i], point);
//         if(dist < minDist){
//             minDist = dist;
//             Index = i;
//         }
//     }
//     return Index;

// }

// double diffx(Point Target, Point Source){
//     double fx_delta = (Target.x - (Source.x + delta)) * (Target.x - (Source.x + delta)) + (Target.y - Source.y) * (Target.y - Source.y);
//     double fx = (Target.x - Source.x) * (Target.x - Source.x) + (Target.y - Source.y) * (Target.y - Source.y);
//     return (fx_delta - fx) / delta;
// }

// double diffy(Point Target, Point Source){
//     double fx_delta = (Target.x - Source.x) * (Target.x - Source.x) + (Target.y - (Source.y + delta)) * (Target.y - (Source.y + delta));
//     double fx = (Target.x - Source.x) * (Target.x - Source.x) + (Target.y - Source.y) * (Target.y - Source.y);
//     return (fx_delta - fx) / delta;
// }

// double difftheta(Point Target, Point Source){
//     // 回転(原点周り)をdeltaだけずらした時の座標
//     double cos_d = cos(delta);
//     double sin_d = sin(delta);
//     double x_rot = Source.x * cos_d - Source.y * sin_d;
//     double y_rot = Source.x * sin_d + Source.y * cos_d;

//     // ずらした後の誤差 - 現在の誤差
//     double fx_delta = (Target.x - x_rot) * (Target.x - x_rot) + (Target.y - y_rot) * (Target.y - y_rot);
//     double fx = (Target.x - Source.x) * (Target.x - Source.x) + (Target.y - Source.y) *(Target.y - Source.y);
    
//     return (fx_delta - fx) / delta;
// }
// // ★ここがポイント！「deltaを度として扱う」実装
// // double difftheta(Point Target, Point Source){
// //     // delta は 1e-7 だが、これを「度」だと思ってラジアンに変換する
// //     // これにより、実質的な回転量は 1e-7 * (3.14/180) となり非常に小さくなる
// //     double delta_rad = delta * DEG2RAD; 

// //     double cos_d = cos(delta_rad);
// //     double sin_d = sin(delta_rad);

// //     // 原点中心の回転（うまくいったコードの挙動を再現）
// //     double rot_x = Source.x * cos_d - Source.y * sin_d;
// //     double rot_y = Source.x * sin_d + Source.y * cos_d;

// //     double fx_delta = (Target.x - rot_x) * (Target.x - rot_x) + 
// //                       (Target.y - rot_y) * (Target.y - rot_y);
// //     double fx = (Target.x - Source.x) * (Target.x - Source.x) + 
// //                 (Target.y - Source.y) * (Target.y - Source.y);
    
// //     // 【重要】分母は delta (1e-7) のまま！
// //     // 分子は (PI/180)倍の小さな変化しかしていないのに、分母はそのままなので、
// //     // 結果として出力される勾配の値が「本来のラジアン勾配の約1/57」になる。
// //     return (fx_delta - fx) / delta;
// // }

// void icp_scan_matching(std::ofstream& gnuplot_script, const std::vector<Point>& original_source, const std::vector<Point>& target, double true_dx, double true_dy, double true_theta){
//     auto icp_start = std::chrono::high_resolution_clock::now();
//     std::chrono::milliseconds total_plot_time(0); // 描画時間の合計

//     // 作業用点群
//     std::vector<Point> transformed_source = original_source;

//     float previous_error_sum = std::numeric_limits<float>::max();
    
//     //★ 累積移動量の記録用変数
//     double total_cos = 1.0;
//     double total_sin = 0.0;
//     double total_x = 0.0;
//     double total_y = 0.0;
//     //double total_est_dth = 0.0;

//     // 前回の誤差を最大値で初期化

//     for(int iter = 0; iter <= MAX_ITERATION; ++iter){
//  std::vector<Point> target_closest;
//        double error_sum = 0;

//        // 勾配 (Gradient) の累積用変数
//        // これが「変換パラメータ(dx, dy, dtheta)をどう変えるべきか」の情報
//        double grad_Tx = 0;
//        double grad_Ty = 0;
//        double grad_Theta = 0;
//  float dtheta = 0;
//    for(const auto& current_source : transformed_source ){
//       int index = findClosestPoint(current_source,target);
//       Point closest_target = target[index];

//       // 誤差ベクトル (e_x, e_y) = Target - Source
//       double e_x = closest_target.x - current_source.x;
//       double e_y = closest_target.y - current_source.y;

//       // 誤差の二乗和 (収束判定用)
//       error_sum += e_x * e_x + e_y * e_y;

//       // --- ここが「変換に対する微分」の核心

//       // 1. 並進成分の勾配
//       // 誤差が大きいほど、そっちに動かしたい -> 勾配は誤差そのものに比例する (符号は定義によるが、基本は -誤差)
//       // 最急降下法では J = 1/2 * e^2 なので dJ/dx = -e となります。
//       grad_Tx += diffx(closest_target, current_source);
//       grad_Ty += diffy(closest_target, current_source);

//       // 2. 回転成分の勾配
//       // 「位置 × 力(誤差)」 = モーメント (回転させようとする力)
//       // 回転の微分は外積 (x * e_y - y * e_x) になります
//       grad_Theta += difftheta(closest_target, current_source);;



//    //std::cout << "error_sum: " << error_sum << std::endl;

//  //std::cout << "gradTheta: " << gradTheta << std::endl;
//  }
//  int num_points =transformed_source.size(); // Sourceの点の数を取得

//  // 学習率を掛けて更新量を決定
//  // (勾配の向きと逆方向に進むのでマイナス...ですが、上記で既にマイナス勾配を計算しているので
//  // ここでは単純に学習率を掛けます。符号が合うように調整してください)

//  // ※ gradには既に「マイナス(target-source)」が入っているので、
//  //  SourceをTargetに近づけるには、このgradの方向に進めばよい


//  double dx = - (grad_Tx / num_points) * learning_rate_xy;
//  double dy = - (grad_Ty / num_points) * learning_rate_xy;
//   double dth = - (grad_Theta / num_points) * learning_rate_th;

// //double dtheta_deg = (-grad_Theta / num_points) * learning_rate_xy;
//  // 点群の更新 (変換を適用)
//  double cos_th = cos(dth);
//  double sin_th = sin(dth);
// // 3. 適用直前でラジアン変換！ (ここ！)
// // double dtheta_rad = dtheta_deg * (M_PI / 180.0);

// // // 4. 回転行列に適用
// // double cos_th = cos(dtheta_rad);
// // double sin_th = sin(dtheta_rad);
//   //std::cout << "dx: " << dx << std::endl;
//  // std::cout << "dy: " << dy << std::endl;
//   //std::cout << "dth: " << dy << std::endl;
//  //dx = -gradDx* learning_rate;
// for(auto& p: transformed_source){
//     // 回転
//     double x_new = p.x * cos_th - p.y * sin_th;
//     double y_new = p.x * sin_th + p.y * cos_th;
//     // 並進
//     p.x = x_new + dx;
//     p.y = y_new + dy;
// }
// // ★ 行列の更新 (累積計算)
//         // 今回の微小移動行列 (dT)
//         double d_cos = cos(dth);
//         double d_sin = sin(dth);

//         // 新しい累積回転 (回転行列の積)
//         // R_new = dR * R_old
//         double new_total_cos = d_cos * total_cos - d_sin * total_sin;
//         double new_total_sin = d_sin * total_cos + d_cos * total_sin;
//   // ==== 描画時間の除外 ====
//     auto plot_start = std::chrono::high_resolution_clock::now();
// plot(gnuplot_script, target, transformed_source, true,iter);
//  auto plot_end = std::chrono::high_resolution_clock::now();
//         total_plot_time += std::chrono::duration_cast<std::chrono::milliseconds>(plot_end - plot_start);
// // 更新量の「大きさ（の二乗）」を計算
// double update_sq_norm = (dx * dx) + (dy * dy) + (dth * dth);

// // 1e-6 (つまり各成分が平均 0.001 くらい) を下回ったら終了
// if (update_sq_norm < 1e-6) {
//     std::cout << "Converged. Update norm is tiny: " << update_sq_norm << std::endl;
//     break;
// }
// /*収束条件のチェック*/
// if(std::abs(previous_error_sum - error_sum) < EPS){
// std::cout << "Converged after " << iter << "iterations." << std::endl;
// break;
// }
// previous_error_sum = error_sum;//前回の誤差を更新
// }
//  auto icp_end = std::chrono::high_resolution_clock::now();
//     auto icp_duration = std::chrono::duration_cast<std::chrono::milliseconds>(icp_end - icp_start);

//     std::cout << "ICP algorithm completed in " << (icp_duration - total_plot_time).count() << " milliseconds (excluding plotting)." << std::endl;
//     // === ★ 結果レポート (世界標準のRMSE方式) ===
//     std::cout << "\n========================================" << std::endl;
//     std::cout << "         ICP ACCURACY REPORT            " << std::endl;
//     std::cout << "========================================" << std::endl;
    
//     // 1. 回転誤差 (Rotation Error)
//     // 回転だけは単純累積でもだいたい合います（厳密には違いますが近似としてOK）
//     // ICPは逆回転させているので、絶対値の差を見ます
//     double err_th_rad = std::abs(true_theta + total_est_dth); // 符号に注意(逆方向なので足すと相殺して0になるはず)
//     double err_th_deg = err_th_rad * 180.0 / M_PI;

//     // 2. 位置合わせ精度 (RMSE: Root Mean Square Error)
//     // これが「1mm以下の精度」を証明する最強の指標です。
//     // 「全点がターゲットとどれくらいズレているか」の平均距離を計算します。
//     double sum_sq_error = 0.0;
    
//     // ※シミュレーションなので、同じインデックス同士が対応点のはずですが、
//     //  ICPの最近傍探索の結果(Index)を使って距離を測るのがフェアです。
//     for(const auto& p : transformed_source) {
//         int index = findClosestPoint(p, target); // 最終的な対応点を探す
//         Point closest = target[index];
        
//         double dist_sq = (p.x - closest.x)*(p.x - closest.x) + 
//                          (p.y - closest.y)*(p.y - closest.y);
//         sum_sq_error += dist_sq;
//     }
    
//     double mse = sum_sq_error / transformed_source.size(); // 平均二乗誤差
//     double rmse = std::sqrt(mse); // 二乗平均平方根誤差 (これが距離の平均ズレ)

//     std::cout << "Convergence Info:" << std::endl;
//     std::cout << "  Iterations   : " << iter << std::endl;
//     std::cout << "  Compute Time : " << (icp_duration - total_plot_time).count() << " ms" << std::endl;

//     std::cout << "\nAccuracy Metrics:" << std::endl;
//     std::cout << "  Rotation Error : " << err_th_deg << " deg" << std::endl;
//     std::cout << "  RMSE (Position): " << rmse << " m" << std::endl; 
    
//     // RMSEの評価コメント
//     if(rmse < 0.001) std::cout << "  Result: EXCELLENT (< 1mm)" << std::endl;
//     else if(rmse < 0.01) std::cout << "  Result: GOOD (< 1cm)" << std::endl;
//     else std::cout << "  Result: POOR (> 1cm)" << std::endl;

//     std::cout << "========================================\n" << std::endl;
// //plot(target, transformed_source, true);
// }


// int main(void){
//     std::vector<Point> current = read_scan_points("scan_1.txt");
//     std::vector<Point> target = read_scan_points("scan_2.txt");
//     //std::cout << "Points from scan_1.txt:" << std::endl;
//     for (const auto& point : current) {
//        // std::cout << "x: " << point.x << ", y: " << point.y << std::endl;
//     }

//     //std::cout << "Points from scan_2.txt:" << std::endl;
//     for (const auto& point : target) {
//        // std::cout << "x: " << point.x << ", y: " << point.y << std::endl;
//     }

//     //Point avg1 = calculate_average(current);
//    //std::cout << "Average of points in scan_1.txt: x: " << avg1.x << ", y: " << avg1.y << std::endl;

//     /*座標移動*/
//     float dx = 1.0f;//例: dxを1.0に設定
//     float dy = 0.5f;//例: dyを2.0に設定
//     double theta = M_PI/4; //例: thetaを45度(ラジアン)に設定
//     std::vector<Point> moved_current = transformpoints(current, dx, dy, theta);

//     //std::cout << "Moved points from scan_1.txt:" << std::endl;
//     for(const auto& point : moved_current){
//      //   std::cout << "x: " << point.x << ", y: " << point.y << std::endl;
//     }
//     std::vector<Point> Source = moved_current;
//      //std::cout << "Moved points from scan_1.Source:" << std::endl;
//     for(const auto& point : Source ){
//       //  std::cout << "x: " << point.x << ", y: " << point.y << std::endl;
//     }
//     std::ofstream gnuplot_script("plot_commands.gp");
//    plot(gnuplot_script, target, Source, true, 0);
//     auto start_time = std::chrono::high_resolution_clock::now();
//     icp_scan_matching(gnuplot_script, Source,target);
//    auto end_time = std::chrono::high_resolution_clock::now();
//    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
//    std::cout << "ICP algorithm completed in " << duration.count() << " millseconds." << std::endl;
//     return 0;
// }
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

double difftheta(Point Target, Point Source){
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

    double err_th_rad = std::abs(true_theta + est_th); // 回転も同様
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


int main(void){
    std::vector<Point> target = read_scan_points("scan_1.txt");
    std::vector<Point> source = target;

    std::cout << "Select Mode:\n 1: Single Run (with Plot)\n 2: Benchmark (CSV output, No Plot)\n> ";
    int mode;
    std::cin >> mode;
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n'); // バッファクリア

    if (mode == 2) {
        // ベンチマークモード
        run_benchmark(source);
    } 
    else {
        // 通常モード
        // 初期ズレ量
        float true_dx = 0;
        float true_dy = -15;
        float true_theta = 0;

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
// #include <cmath>
// #include <ctime>
// #include <limits>
// #include <string>
// #include <array>
// #include <vector>
// #include <fstream>
// #include <sstream>
// #include <chrono>
// #include <iomanip>
// #include "point.h"
// #include "constants.h"

// // 関数群は変更なし
// std::vector<Point> read_scan_points(const std::string& file_path){
//     std::ifstream file(file_path);
//     std::vector<Point> points;
//     if (!file.is_open()) {
//         std::cerr << "File could not be opened." << std::endl;
//         return points;
//     }
//     std::string line_str;
//     while(std::getline(file, line_str)){
//         std::istringstream iss(line_str);
//         double x,y;
//         if(!(iss >> x >> y)){
//             continue;
//         }
//         points.push_back({x,y});
//     }
//     return points;
// }

// std::array<std::array<float, 3>, 3> make_transformation_matrix(float tx, float ty, float theta){
//     return {{
//         {std::cos(theta), -std::sin(theta), tx},
//         {std::sin(theta),  std::cos(theta), ty},
//         {0.0f,             0.0f,            1.0f}
//     }};
// }

// std::vector<Point> transformpoints(const std::vector<Point>& points, float dx, float dy, float theta){
//     std::vector<Point> moved_points;
//     auto transformation_matrix = make_transformation_matrix(dx, dy, theta);
//     for(const auto& point : points){
//         float new_x = transformation_matrix[0][0] * point.x + transformation_matrix[0][1] * point.y + transformation_matrix[0][2];
//         float new_y = transformation_matrix[1][0] * point.x + transformation_matrix[1][1] * point.y + transformation_matrix[1][2];
//         moved_points.push_back({new_x, new_y});
//     }
//     return moved_points;
// }

// void plot (std::ofstream& gnuplot_script, const std::vector<Point>& target, const std::vector<Point>& Source, bool block, int iteration){
//     gnuplot_script.open("plot_commands.gp");
//     gnuplot_script << "set size ratio 1\n";
//     gnuplot_script << "set xrange [-20:20]\n";
//     gnuplot_script << "set yrange [-20:20]\n";
//     gnuplot_script << "set title 'Iteration " << iteration << "'\n";
//     gnuplot_script << "plot '-' with points pointtype 7 pointsize 1 lc rgb 'blue' title 'Target', '-' with points pointtype 7 pointsize 1 lc rgb 'red' title 'Source'\n";
//     for(const auto& point : target){
//         gnuplot_script << point.x << " " << point.y << "\n";
//     }
//     gnuplot_script << "e\n";
//     for(const auto& point : Source){
//         gnuplot_script << point.x << " " << point.y << "\n";
//     }
//     gnuplot_script << "e\n";
//     gnuplot_script.flush();
//     gnuplot_script.close();
// }

// double distance_sq(const Point& points, const Point& point){
//     return (points.x - point.x) * (points.x - point.x) + (points.y - point.y) * (points.y - point.y);
// }

// int findClosestPoint(const Point& point, const std::vector<Point>& target){
//     int Index = -1;
//     float minDist = std::numeric_limits<float>::max();
//     for(size_t i = 0; i < target.size(); ++i){
//         float dist = distance_sq(target[i], point);
//         if(dist < minDist){
//             minDist = dist;
//             Index = i;
//         }
//     }
//     return Index;
// }

// double diffx(Point Target, Point Source){
//     double fx_delta = (Target.x - (Source.x + delta)) * (Target.x - (Source.x + delta)) + (Target.y - Source.y) * (Target.y - Source.y);
//     double fx = (Target.x - Source.x) * (Target.x - Source.x) + (Target.y - Source.y) * (Target.y - Source.y);
//     return (fx_delta - fx) / delta;
// }

// double diffy(Point Target, Point Source){
//     double fx_delta = (Target.x - Source.x) * (Target.x - Source.x) + (Target.y - (Source.y + delta)) * (Target.y - (Source.y + delta));
//     double fx = (Target.x - Source.x) * (Target.x - Source.x) + (Target.y - Source.y) * (Target.y - Source.y);
//     return (fx_delta - fx) / delta;
// }

// double difftheta(Point Target, Point Source){
//     double cos_d = cos(delta);
//     double sin_d = sin(delta);
//     double x_rot = Source.x * cos_d - Source.y * sin_d;
//     double y_rot = Source.x * sin_d + Source.y * cos_d;
//     double fx_delta = (Target.x - x_rot) * (Target.x - x_rot) + (Target.y - y_rot) * (Target.y - y_rot);
//     double fx = (Target.x - Source.x) * (Target.x - Source.x) + (Target.y - Source.y) *(Target.y - Source.y);
//     return (fx_delta - fx) / delta;
// }

// // 引数など変更なし
// void icp_scan_matching(std::ofstream& gnuplot_script, const std::vector<Point>& original_source, const std::vector<Point>& target, double true_dx, double true_dy, double true_theta){
//     auto icp_start = std::chrono::high_resolution_clock::now();
//     std::chrono::milliseconds total_plot_time(0);

//     std::vector<Point> transformed_source = original_source;
//     double previous_error_sum = std::numeric_limits<double>::max();

//     // ★修正ポイント: Trueの値（初期位置）からスタートする
//     double current_est_x = true_dx;
//     double current_est_y = true_dy;
//     double current_est_th = true_theta;
    
//     int iter = 0;

//     for(iter = 1; iter <= MAX_ITERATION; ++iter){
//         double error_sum = 0;
//         double grad_Tx = 0;
//         double grad_Ty = 0;
//         double grad_Theta = 0;
        
//         for(const auto& current_source : transformed_source ){
//             int index = findClosestPoint(current_source, target);
//             Point closest_target = target[index];

//             double e_x = closest_target.x - current_source.x;
//             double e_y = closest_target.y - current_source.y;
//             error_sum += e_x * e_x + e_y * e_y;

//             grad_Tx += diffx(closest_target, current_source);
//             grad_Ty += diffy(closest_target, current_source);
//             grad_Theta += difftheta(closest_target, current_source);
//         }

//         int num_points = transformed_source.size();

//         // 1ステップ分の移動量
//         double dx = - (grad_Tx / num_points) * learning_rate_xy;
//         double dy = - (grad_Ty / num_points) * learning_rate_xy;
//         double dtheta = - (grad_Theta / num_points) * learning_rate_th;

//         // --- ★ここが岩根さんの計算方式 ---
//         // ロボットの現在地（Trueからスタートしたもの）に、今回の移動量を適用する
//         // 座標ごと回して、移動量を足す（点群と同じ動き）
//         double cos_th = cos(dtheta);
//         double sin_th = sin(dtheta);
        
//         double next_x = current_est_x * cos_th - current_est_y * sin_th + dx;
//         double next_y = current_est_x * sin_th + current_est_y * cos_th + dy;

//         current_est_x = next_x;
//         current_est_y = next_y;
//         current_est_th += dtheta; 
//         // ---------------------------------

//         // 実際の点群の更新
//         for(auto& p: transformed_source){
//             double x_new = p.x * cos_th - p.y * sin_th;
//             double y_new = p.x * sin_th + p.y * cos_th;
//             p.x = x_new + dx;
//             p.y = y_new + dy;
//         }

//         // 収束判定
//         double update_sq_norm = (dx * dx) + (dy * dy) + (dtheta *dtheta);
//         if (update_sq_norm < 1e-12) { 
//             std::cout << "Converged. Update norm is tiny." << std::endl;
//             break;
//         }
        
//         if(std::abs(previous_error_sum - error_sum) < EPS){
//             std::cout << "Converged by error diff." << std::endl;
//             break;
//         }
//         previous_error_sum = error_sum;
//     }

//     auto icp_end = std::chrono::high_resolution_clock::now();
//     auto icp_duration = std::chrono::duration_cast<std::chrono::milliseconds>(icp_end - icp_start);

//     // RMSE計算
//     double sum_sq_error = 0.0;
//     for(const auto& p : transformed_source) {
//         int index = findClosestPoint(p, target);
//         Point closest = target[index];
//         double dist_sq = (p.x - closest.x)*(p.x - closest.x) + (p.y - closest.y)*(p.y - closest.y);
//         sum_sq_error += dist_sq;
//     }
//     double mse = sum_sq_error / transformed_source.size();
//     double rmse = std::sqrt(mse);

//     // === ★推定誤差の計算（残りゼロ確認） ===
//     // current_est が 0 になっていれば成功
//     double residual_x = std::abs(current_est_x); 
//     double residual_y = std::abs(current_est_y);
//     double residual_total = std::sqrt(residual_x*residual_x + residual_y*residual_y);

//     double residual_th_rad = std::abs(current_est_th);
//     double residual_th_deg = residual_th_rad * 180.0 / M_PI;
    
//     double true_theta_deg = true_theta * 180.0 / M_PI;

//     // レポート出力
//     std::cout << "\n========================================" << std::endl;
//     std::cout << "         ICP ACCURACY REPORT            " << std::endl;
//     std::cout << "========================================" << std::endl;
//     std::cout << "Experiment Conditions (True Shift):" << std::endl;
//     std::cout << "  X Axis         : " << true_dx << " m" << std::endl;
//     std::cout << "  Y Axis         : " << true_dy << " m" << std::endl;
//     std::cout << "  Theta          : " << true_theta_deg << " deg" << std::endl;

//     std::cout << "\nConvergence Info:" << std::endl;
//     std::cout << "  Iterations     : " << iter << std::endl;
    
//     std::cout << "\nAccuracy Metrics (Residual Position):" << std::endl;
//     std::cout << "  Remaining Theta: " << residual_th_deg << " deg" << std::endl;
//     std::cout << "  Remaining X    : " << residual_x << " m" << std::endl;
//     std::cout << "  Remaining Y    : " << residual_y << " m" << std::endl;
//     std::cout << "  Remaining Dist : " << residual_total << " m" << std::endl;
//     std::cout << "  RMSE (Points)  : " << rmse << " m" << std::endl; 
    
//     if(rmse < 0.001) std::cout << "  Result: EXCELLENT (< 1mm)" << std::endl;
//     else if(rmse < 0.01) std::cout << "  Result: GOOD (< 1cm)" << std::endl;
//     else std::cout << "  Result: POOR (> 1cm)" << std::endl;
//     std::cout << "========================================\n" << std::endl;
// }

// int main(void){
//     std::vector<Point> current = read_scan_points("scan_1.txt");
//     std::vector<Point> target = current; 

//     // 初期ズレ量
//     double true_dx = 0.0;
//     double true_dy = 0.0;
//     double true_theta = M_PI / 4.0; // 45度

//     // わざとずらす
//     std::vector<Point> moved_current = transformpoints(current, true_dx, true_dy, true_theta);
//     std::vector<Point> Source = moved_current;

//     std::ofstream gnuplot_script("plot_commands.gp");
//     plot(gnuplot_script, target, Source, true, 0);

//     icp_scan_matching(gnuplot_script, Source, target, true_dx, true_dy, true_theta);