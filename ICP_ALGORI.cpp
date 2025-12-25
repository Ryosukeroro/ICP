#include <iostream>
#include <cmath>
#include <ctime>
#include <limits>
#include <string>
#include <array>
#include <vector> // std::vectorを使用するために必要
#include <fstream>//ファイル操作用のライブラリ
#include <sstream>
#include <chrono>
#include "point.h"
#include "constants.h"

// /*初期化フラグ*/
// bool initialized = false; // 初期化フラグ：リソースが初期化されたかどうかを示す
// float preError = 0.0f; // 前回のエラー値。初期値はゼロ
// float dError = std::numeric_limits<float>::max();
// float errorHistory[MAX_iteration];
// //float dyHistory[MAX_iTERATION];
// //std::string linesToSave[MAX_iTERATION]; // CSVファイルに保存するデータ
// //std::string linesToSave_dy[MAX_ItERATION];
// //std::string linesToSave_dx[MAX_ItERATION];
// /*点群数*/
// const int numPoints1 = 723;
// const int numPoints2 = 737;

// float random_x;
// float random_y;
// float angle = 0.0f;

// std::array<float, 3> motion = {0.0f, 0.0f, 0.0f};
// std::vector<float> transformedX(numPoints2);
















std::vector<Point> read_scan_points(const std::string& file_path){
    std::ifstream file(file_path);
    std::vector<Point> points;
    if (!file.is_open()) {
        std::cerr << "File could not be opened." << std::endl;
        throw std::runtime_error("Error File could not be opened at path: " + file_path);
        return points;
    }
    std::string line_str;
    while(std::getline(file, line_str)){
        std::istringstream iss(line_str);
        double x,y;
        if(!(iss >> x >> y)){
            std::cerr << "Failed to parse line: " << line_str << std::endl;
            continue;
        }
        points.push_back({x,y});
    }
    return points;
}

Point calculate_average(const std::vector<Point>& points){
    float sum_x = 0.0f;
    float sum_y = 0.0f;
    for(const auto& point : points){
        sum_x += point.x;
        sum_y += point.y;
    }
    float avg_x = sum_x / points.size();
    float avg_y = sum_y / points.size();
    return {avg_x, avg_y};
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

void plot (std::ofstream& gnuplot_script, const std::vector<Point>& target, const std::vector<Point>& Source, bool block,int iteration){
    gnuplot_script.open("plot_commands.gp");
    gnuplot_script << "set size ratio 1\n";
    gnuplot_script << "set xrange [-20:20]\n";
    gnuplot_script << "set xrange [-20:20]\n";
    gnuplot_script << "set yrange [-20:20]\n";
    gnuplot_script << "set title 'Iteration " << iteration << "'\n"; // 現在のループ回数を表示
    gnuplot_script << "plot '-' with points pointtype 7 pointsize 1 lc rgb 'blue' title 'Target points', '-' with points pointtype 7 pointsize 1 lc rgb 'red' title 'Source points'\n";
    for(const auto& point : target){
        gnuplot_script << point.x << " " << point.y << "\n";
    }
    gnuplot_script << "e\n";
    for(const auto& point : Source){
        gnuplot_script << point.x << " " << point.y << "\n";
    }
    gnuplot_script << "e\n";
     //gnuplot_script << "pause -1\n";  // プロットを更新するために一時停止
    gnuplot_script.flush(); // スクリプトをフラッシュして即時反映させる
    gnuplot_script << "set size ratio 1\n";
     gnuplot_script.close();
    // gnuplot_script.close();
     std::string gnuplot_command = "gnuplot -p plot_commands.gp";
    // /*if (block) {
    //     std::cout << "Press Enter to continue...";
    // std::cin.ignore(); // 
     system(gnuplot_command.c_str());
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

float diffx(Point Target, Point SOurce){
    float fx_delta = (Target.x - (SOurce.x + delta)) * (Target.x - (SOurce.x + delta)) + (Target.y - SOurce.y) * (Target.y - SOurce.y);
    float fx = (Target.x - SOurce.x) * (Target.x - SOurce.x) + (Target.y - SOurce.y) * (Target.y - SOurce.y);
    return (fx_delta - fx) / delta;
}

float diffy(Point Target, Point SOurce){
    float fx_delta = (Target.x - SOurce.x) * (Target.x - SOurce.x) + (Target.y - (SOurce.y + delta)) * (Target.y - (SOurce.y + delta));
    float fx = (Target.x - SOurce.x) * (Target.x - SOurce.x) + (Target.y - SOurce.y) * (Target.y - SOurce.y);
    return (fx_delta - fx) / delta;
}

float difftheta(Point Target, Point SOurce){
    float fx_delta = (Target.x - ((SOurce.x)* cos(delta))-(SOurce.y)* sin(delta))* (Target.x - ((SOurce.x)* cos(delta)-(SOurce.y)* sin(delta))) + (Target.y - ((SOurce.x) * (sin(delta)) + (SOurce.y) * cos(delta))) * (Target.y - ((SOurce.x) * (sin(delta)) + (SOurce.y) * cos(delta)));
    float fx = (Target.x - SOurce.x) * (Target.x - SOurce.x) + (Target.y - SOurce.y) * (Target.y - SOurce.y);
    return (fx_delta - fx) / delta;
}

void icp_scan_matching(std::ofstream& gnuplot_script, const std::vector<Point>& original_source, const std::vector<Point>& target){
    auto icp_start = std::chrono::high_resolution_clock::now();
    std::chrono::milliseconds total_plot_time(0); // 描画時間の合計

    // 作業用点群
    std::vector<Point> transformed_source = original_source;

    float previous_error_sum = std::numeric_limits<float>::max(); // 前回の誤差を最大値で初期化

    for(int iter = 0; iter <= MAX_ITERATION; ++iter){
 std::vector<Point> target_closest;
       double error_sum = 0;

       // 勾配 (Gradient) の累積用変数
       // これが「変換パラメータ(dx, dy, dtheta)をどう変えるべきか」の情報
       double grad_Tx = 0;
       double grad_Ty = 0;
       double grad_Theta = 0;
 float dtheta = 0;
   for(const auto& current_source : transformed_source ){
      int index = findClosestPoint(current_source,target);
      Point closest_target = target[index];

      // 誤差ベクトル (e_x, e_y) = Target - Source
      double e_x = closest_target.x - current_source.x;
      double e_y = closest_target.y - current_source.y;

      // 誤差の二乗和 (収束判定用)
      error_sum += e_x * e_x + e_y * e_y;

      // --- ここが「変換に対する微分」の核心

      // 1. 並進成分の勾配
      // 誤差が大きいほど、そっちに動かしたい -> 勾配は誤差そのものに比例する (符号は定義によるが、基本は -誤差)
      // 最急降下法では J = 1/2 * e^2 なので dJ/dx = -e となります。
      grad_Tx += -e_x;
      grad_Ty += -e_y;

      // 2. 回転成分の勾配
      // 「位置 × 力(誤差)」 = モーメント (回転させようとする力)
      // 回転の微分は外積 (x * e_y - y * e_x) になります
      grad_Theta += -(current_source.x * e_y - current_source.y * e_x);



  Point SOurce = {Source.x, Source.y};
   //std::cout << "error_sum: " << error_sum << std::endl;

 //std::cout << "gradTheta: " << gradTheta << std::endl;
 }
 int num_points =transformed_source.size(); // Sourceの点の数を取得

 // 学習率を掛けて更新量を決定
 // (勾配の向きと逆方向に進むのでマイナス...ですが、上記で既にマイナス勾配を計算しているので
 // ここでは単純に学習率を掛けます。符号が合うように調整してください)

 // ※ gradには既に「マイナス(target-source)」が入っているので、
 //  SourceをTargetに近づけるには、このgradの方向に進めばよい


 double dx = - (grad_Tx / num_points) * learning_rate;
 double dy = - (grad_Ty / num_points) * learning_rate;
 double dth = - (grad_Theta / num_points) * learning_rate;

 // 点群の更新 (変換を適用)
 double cos_th = cos(dth);
 double sin_th = sin(dth);
  //std::cout << "dx: " << dx << std::endl;
 // std::cout << "dy: " << dy << std::endl;
  //std::cout << "dth: " << dy << std::endl;
 //dx = -gradDx* learning_rate;
for(auto& p: transformed_source){
    // 回転
    double x_new = p.x * cos_th - p.y * sin_th;
    double y_new = p.x * sin_th + p.y * cos_th;
    // 並進
    p.x = x_new + dx;
    p.y = y_new + dy;
}
  // ==== 描画時間の除外 ====
    auto plot_start = std::chrono::high_resolution_clock::now();
plot(gnuplot_script, target, transformed_source, true,iter);
 auto plot_end = std::chrono::high_resolution_clock::now();
        total_plot_time += std::chrono::duration_cast<std::chrono::milliseconds>(plot_end - plot_start);

/*収束条件のチェック*/
if(std::abs(previous_error_sum - error_sum) < EPS){
std::cout << "Converged after " << iter << "iterations." << std::endl;
break;
}
previous_error_sum = error_sum;//前回の誤差を更新
}
 auto icp_end = std::chrono::high_resolution_clock::now();
    auto icp_duration = std::chrono::duration_cast<std::chrono::milliseconds>(icp_end - icp_start);

    std::cout << "ICP algorithm completed in " << (icp_duration - total_plot_time).count() << " milliseconds (excluding plotting)." << std::endl;
//plot(target, transformed_source, true);
}


int main(void){
    std::vector<Point> current = read_scan_points("scan_1.txt");
    std::vector<Point> target = read_scan_points("scan_2.txt");
    //std::cout << "Points from scan_1.txt:" << std::endl;
    for (const auto& point : current) {
       // std::cout << "x: " << point.x << ", y: " << point.y << std::endl;
    }

    //std::cout << "Points from scan_2.txt:" << std::endl;
    for (const auto& point : target) {
       // std::cout << "x: " << point.x << ", y: " << point.y << std::endl;
    }

    //Point avg1 = calculate_average(current);
   //std::cout << "Average of points in scan_1.txt: x: " << avg1.x << ", y: " << avg1.y << std::endl;

    /*座標移動*/
    float dx = 1.0f;//例: dxを1.0に設定
    float dy = 0.5f;//例: dyを2.0に設定
    double theta = M_PI/4; //例: thetaを45度(ラジアン)に設定
    std::vector<Point> moved_current = transformpoints(current, dx, dy, theta);

    //std::cout << "Moved points from scan_1.txt:" << std::endl;
    for(const auto& point : moved_current){
     //   std::cout << "x: " << point.x << ", y: " << point.y << std::endl;
    }
    std::vector<Point> Source = moved_current;
     //std::cout << "Moved points from scan_1.Source:" << std::endl;
    for(const auto& point : Source ){
      //  std::cout << "x: " << point.x << ", y: " << point.y << std::endl;
    }
    std::ofstream gnuplot_script("plot_commands.gp");
   plot(gnuplot_script, target, Source, true, 0);
    auto start_time = std::chrono::high_resolution_clock::now();
    icp_scan_matching(gnuplot_script, Source,target);
   auto end_time = std::chrono::high_resolution_clock::now();
   auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
   std::cout << "ICP algorithm completed in " << duration.count() << " millseconds." << std::endl;
    return 0;
}
