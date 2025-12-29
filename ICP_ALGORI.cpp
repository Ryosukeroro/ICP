// #include <cmath>
// #include <ctime>
// #include <limits>
// #include <string>
// #include <array>
// #include <vector> // std::vectorを使用するために必要
// #include <fstream>//ファイル操作用のライブラリ
// #include <sstream>
// #include <chrono>
// #include "point.h"
// #include "constants.h"
// #include <iomanip> // きれいに表示するために必要

// // /*初期化フラグ*/
// // bool initialized = false; // 初期化フラグ：リソースが初期化されたかどうかを示す
// // float preError = 0.0f; // 前回のエラー値。初期値はゼロ
// // float dError = std::numeric_limits<float>::max();
// // float errorHistory[MAX_iteration];
// // //float dyHistory[MAX_iTERATION];
// // //std::string linesToSave[MAX_iTERATION]; // CSVファイルに保存するデータ
// // //std::string linesToSave_dy[MAX_ItERATION];
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
        double x,y;
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

double distance_sq(const Point& points, const Point& point){
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
double diffx(Point Target, Point Source){
    double fx_delta = (Target.x - (Source.x + delta)) * (Target.x - (Source.x + delta)) + (Target.y - Source.y) * (Target.y - Source.y);
    double fx = (Target.x - Source.x) * (Target.x - Source.x) + (Target.y - Source.y) * (Target.y - Source.y);
    return (fx_delta - fx) / delta;
}

double diffy(Point Target, Point Source){
    double fx_delta = (Target.x - Source.x) * (Target.x - Source.x) + (Target.y - (Source.y + delta)) * (Target.y - (Source.y + delta));
    double fx = (Target.x - Source.x) * (Target.x - Source.x) + (Target.y - Source.y) * (Target.y - Source.y);
    return (fx_delta - fx) / delta;
}

// // deltaを度として扱い、内部でラジアン変換する方式（うまくいったやつ）
// double difftheta(Point Target, Point Source){
//     double delta_rad = delta * (M_PI / 180.0); 
//     double cos_d = cos(delta_rad);
//     double sin_d = sin(delta_rad);

//     double rot_x = Source.x * cos_d - Source.y * sin_d;
//     double rot_y = Source.x * sin_d + Source.y * cos_d;

//     double fx_delta = (Target.x - rot_x) * (Target.x - rot_x) + (Target.y - rot_y) * (Target.y - rot_y);
//     double fx = (Target.x - Source.x) * (Target.x - Source.x) + (Target.y - Source.y) * (Target.y - Source.y);
    
//     return (fx_delta - fx) / delta;
// }

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

// 引数を追加しています (true_dx, true_dy, true_theta)
void icp_scan_matching(std::ofstream& gnuplot_script, const std::vector<Point>& original_source, const std::vector<Point>& target, double true_dx, double true_dy, double true_theta){
    auto icp_start = std::chrono::high_resolution_clock::now();
    std::chrono::milliseconds total_plot_time(0);

    std::vector<Point> transformed_source = original_source;
    double previous_error_sum = std::numeric_limits<double>::max();

    // ★岩根さん方式の累積変数を初期化 (0,0)からスタート
    // double est_x = 0.0;
    // double est_y = 0.0;
    // double est_th = 0.0;
    // ★修正ポイント: Trueの値（初期位置）からスタートする
    double current_est_x = true_dx;
    double current_est_y = true_dy;
    double current_est_th = true_theta;

    // ★修正1: 累積変数をここで宣言！
    double total_est_dth = 0.0;
    
    // ★修正2: iterをここで宣言！（ループの外でも使えるように）
    int iter = 0;

    for(iter = 1; iter <= MAX_ITERATION; ++iter){
        double error_sum = 0;
        double grad_Tx = 0;
        double grad_Ty = 0;
        double grad_Theta = 0;
        
        for(const auto& current_source : transformed_source ){
            int index = findClosestPoint(current_source, target);
            Point closest_target = target[index];

            double e_x = closest_target.x - current_source.x;
            double e_y = closest_target.y - current_source.y;
            error_sum += e_x * e_x + e_y * e_y;

            grad_Tx += diffx(closest_target, current_source);
            grad_Ty += diffy(closest_target, current_source);
            grad_Theta += difftheta(closest_target, current_source);
        }

        int num_points = transformed_source.size();

        double dx = - (grad_Tx / num_points) * learning_rate_xy;
        double dy = - (grad_Ty / num_points) * learning_rate_xy;
        double dtheta = - (grad_Theta / num_points) * learning_rate_th; // 度として扱う

        // ★累積加算 (回転だけ)
        // 並進(dx, dy)は回転しながら動いているので単純和は意味がないため、計算しなくてOK
        //double dtheta_rad = dtheta_deg * (M_PI / 180.0);
        total_est_dth += dtheta;

        // 座標更新
        double cos_th = cos(dtheta);
        double sin_th = sin(dtheta);

        // 1. まず現在の推定値を回転させる (Old * Rot)
        double new_est_x = est_x * cos_th - est_y * sin_th;
        double new_est_y = est_x * sin_th + est_y * cos_th;

        // 2. 並進成分を足す (+ dx)
        est_x = new_est_x + dx;
        est_y = new_est_y + dy;
        est_th += dtheta; // 回転角は単純和でOK
        // ---------------------------------

        for(auto& p: transformed_source){
            double x_new = p.x * cos_th - p.y * sin_th;
            double y_new = p.x * sin_th + p.y * cos_th;
            p.x = x_new + dx;
            p.y = y_new + dy;
        }

        // 描画
        auto plot_start = std::chrono::high_resolution_clock::now();
       // plot(gnuplot_script, target, transformed_source, true, iter);
        auto plot_end = std::chrono::high_resolution_clock::now();
        total_plot_time += std::chrono::duration_cast<std::chrono::milliseconds>(plot_end - plot_start);

        // 収束判定
        double update_sq_norm = (dx * dx) + (dy * dy) + (dtheta *dtheta); // dthetaは度で評価してもOK
        if (update_sq_norm < 1e-9) {
            std::cout << "Converged. Update norm is tiny." << std::endl;
            break;
        }
        
        // 誤差変動チェック
        if(std::abs(previous_error_sum - error_sum) < EPS){
            std::cout << "Converged by error diff." << std::endl;
            break;
        }
        previous_error_sum = error_sum;
    }

    auto icp_end = std::chrono::high_resolution_clock::now();
    auto icp_duration = std::chrono::duration_cast<std::chrono::milliseconds>(icp_end - icp_start);

    // // === RMSE (位置合わせ精度) の計算 ===
    // // 世界標準の評価指標
    // double sum_sq_error = 0.0;
    // for(const auto& p : transformed_source) {
    //     int index = findClosestPoint(p, target);
    //     Point closest = target[index];
    //     double dist_sq = (p.x - closest.x)*(p.x - closest.x) + (p.y - closest.y)*(p.y - closest.y);
    //     sum_sq_error += dist_sq;
    // }
    // double mse = sum_sq_error / transformed_source.size();
    // double rmse = std::sqrt(mse);

    // // 回転誤差の計算
    // double err_th_rad = std::abs(true_theta + total_est_dth);
    // double err_th_deg = err_th_rad * 180.0 / M_PI;

    // // // レポート出力
    // // std::cout << "\n========================================" << std::endl;
    // // std::cout << "         ICP ACCURACY REPORT            " << std::endl;
    // // std::cout << "========================================" << std::endl;
    // // std::cout << "Convergence Info:" << std::endl;
    // // std::cout << "  Iterations   : " << iter << std::endl;
    // // std::cout << "  Compute Time : " << (icp_duration - total_plot_time).count() << " ms" << std::endl;
    // // std::cout << "\nAccuracy Metrics:" << std::endl;
    // // std::cout << "  Rotation Error : " << err_th_deg << " deg" << std::endl;
    // // std::cout << "  RMSE (Position): " << rmse << " m" << std::endl; 
    
    // // if(rmse < 0.001) std::cout << "  Result: EXCELLENT (< 1mm)" << std::endl;
    // // else if(rmse < 0.01) std::cout << "  Result: GOOD (< 1cm)" << std::endl;
    // // else std::cout << "  Result: POOR (> 1cm)" << std::endl;
    // // std::cout << "========================================\n" << std::endl;
    // double true_theta_deg = true_theta * 180.0 / M_PI; // 表示用に度数法に変換

    // std::cout << "\n========================================" << std::endl;
    // std::cout << "         ICP ACCURACY REPORT            " << std::endl;
    // std::cout << "========================================" << std::endl;
    
    // // ★ここに追加しました
    // std::cout << "Experiment Conditions (True Shift):" << std::endl;
    // std::cout << "  X Axis         : " << true_dx << " m" << std::endl;
    // std::cout << "  Y Axis         : " << true_dy << " m" << std::endl;
    // std::cout << "  Theta          : " << true_theta_deg << " deg" << std::endl;

    // std::cout << "\nConvergence Info:" << std::endl;
    // std::cout << "  Iterations   : " << iter << std::endl;
    // std::cout << "  Compute Time : " << (icp_duration - total_plot_time).count() << " ms" << std::endl;
    
    // std::cout << "\nAccuracy Metrics:" << std::endl;
    // std::cout << "  Rotation Error : " << err_th_deg << " deg" << std::endl;
    // std::cout << "  RMSE (Position): " << rmse << " m" << std::endl; 
    
    // if(rmse < 0.001) std::cout << "  Result: EXCELLENT (< 1mm)" << std::endl;
    // else if(rmse < 0.01) std::cout << "  Result: GOOD (< 1cm)" << std::endl;
    // else std::cout << "  Result: POOR (> 1cm)" << std::endl;
    // std::cout << "========================================\n" << std::endl;
    // === RMSE (位置合わせ精度) の計算 ===
    double sum_sq_error = 0.0;
    for(const auto& p : transformed_source) {
        int index = findClosestPoint(p, target);
        Point closest = target[index];
        double dist_sq = (p.x - closest.x)*(p.x - closest.x) + (p.y - closest.y)*(p.y - closest.y);
        sum_sq_error += dist_sq;
    }
    double mse = sum_sq_error / transformed_source.size();
    double rmse = std::sqrt(mse);

    // === ★推定誤差の計算（岩根さん方式の結果を使用） ===
    // ICPの移動量(est)は、初期ズレ(true)を打ち消す方向に動くので
    // 正解なら「true + est = 0」になるはず
    double trans_err_x = std::abs(true_dx + est_x); 
    double trans_err_y = std::abs(true_dy + est_y);
    double trans_err_total = std::sqrt(trans_err_x*trans_err_x + trans_err_y*trans_err_y);

    // 回転誤差の計算
    double err_th_rad = std::abs(true_theta + total_est_dth);
    double err_th_deg = err_th_rad * 180.0 / M_PI;

    // ★追加: 並進誤差の計算（重心のズレを見る）
    Point target_centroid = get_centroid(target);
    Point final_source_centroid = get_centroid(transformed_source);
    
    double err_x = std::abs(target_centroid.x - final_source_centroid.x);
    double err_y = std::abs(target_centroid.y - final_source_centroid.y);
    double err_trans_total = std::sqrt(err_x*err_x + err_y*err_y);


    // レポート出力
    double true_theta_deg = true_theta * 180.0 / M_PI;

    std::cout << "\n========================================" << std::endl;
    std::cout << "         ICP ACCURACY REPORT            " << std::endl;
    std::cout << "========================================" << std::endl;
    
    std::cout << "Experiment Conditions (True Shift):" << std::endl;
    std::cout << "  X Axis         : " << true_dx << " m" << std::endl;
    std::cout << "  Y Axis         : " << true_dy << " m" << std::endl;
    std::cout << "  Theta          : " << true_theta_deg << " deg" << std::endl;

    std::cout << "\nConvergence Info:" << std::endl;
    std::cout << "  Iterations     : " << iter << std::endl;
    std::cout << "  Compute Time   : " << (icp_duration - total_plot_time).count() << " ms" << std::endl;
    
    std::cout << "\nAccuracy Metrics (Residual Error):" << std::endl;
    std::cout << "  Rotation Error : " << err_th_deg << " deg" << std::endl;
    // ★ここに追加！
    std::cout << "  Trans Error X  : " << err_x << " m" << std::endl;
    std::cout << "  Trans Error Y  : " << err_y << " m" << std::endl;
    std::cout << "  Trans Error 2D : " << err_trans_total << " m" << std::endl;
    std::cout << "  RMSE (Points)  : " << rmse << " m" << std::endl; 
    std::cout << "  Trans Error X  : " << trans_err_x << " m" << std::endl;
    std::cout << "  Trans Error Y  : " << trans_err_y << " m" << std::endl;
    std::cout << "  Trans Error 2D : " << trans_err_total << " m" << std::endl;
    
    if(rmse < 0.001) std::cout << "  Result: EXCELLENT (< 1mm)" << std::endl;
    else if(rmse < 0.01) std::cout << "  Result: GOOD (< 1cm)" << std::endl;
    else std::cout << "  Result: POOR (> 1cm)" << std::endl;
    std::cout << "========================================\n" << std::endl;
}

int main(void){
    std::vector<Point> current = read_scan_points("scan_1.txt");
    //std::vector<Point> target = read_scan_points("scan_2.txt");
std::vector<Point> target = current;
    // 初期ズレ量 (正解データ)
    double true_dx = 0.0;
    double true_dy = 0.0;
    double true_theta = M_PI / 4.0;

    // わざとずらす
    std::vector<Point> moved_current = transformpoints(current, true_dx, true_dy, true_theta);
    std::vector<Point> Source = moved_current;

    std::ofstream gnuplot_script("plot_commands.gp");
    plot(gnuplot_script, target, Source, true, 0);

    // ★修正3: ここで引数を渡す！
    icp_scan_matching(gnuplot_script, Source, target, true_dx, true_dy, true_theta);

    return 0;
}

// #include <iostream>
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

//     return 0;
// }