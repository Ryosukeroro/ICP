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
/*構造体*/
struct Point{
    float x,y;
};
/*定義*/
#define MAX_iteration 30
#define max_dist 3

/*収束判定の閾値*/
#define EPS  0.001

/*微小変位*/
#define delta 1.0e-7


/*学習率*/
#define learning_rate 1


/*count値*/
int count = 0;

// /*error値*/
// float distanceSquared = 0.0f; // 距離の二乗
// float Error = std::numeric_limits<float>::max();

// /*1回めのError値*/
// float initialError;

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

// /*移動量*/
// std::array<float, 3> motion = {0.0f, 0.0f, 0.0f};
// std::vector<float> transformedX(numPoints2);
// std::vector<float> transformedY(numPoints2);
// float diffX;

















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
    // }*/
     system(gnuplot_command.c_str());
}

float distance(const Point& points, const Point& point){
    return sqrt((points.x - point.x) * (points.x - point.x) + (points.y - point.y) * (points.y - point.y));

}

int findClosestPoint(const Point& point, const std::vector<Point>& target){
    int Index = -1;
    float minDist = std::numeric_limits<float>::max();
    for(size_t i = 0; i < target.size(); ++i){
        float dist = distance(target[i], point);
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
    float fx_delta = (Target.x - ((SOurce.x)* cos(delta*M_PI/180)-(SOurce.y)* sin(delta*M_PI/180)))* (Target.x - ((SOurce.x)* cos(delta*M_PI/180)-(SOurce.y)* sin(delta*M_PI/180))) + (Target.y - ((SOurce.x) * (sin(delta*M_PI/180)) + (SOurce.y) * cos(delta*M_PI/180))) * (Target.y - ((SOurce.x) * (sin(delta*M_PI/180)) + (SOurce.y) * cos(delta*M_PI/180)));
    float fx = (Target.x - SOurce.x) * (Target.x - SOurce.x) + (Target.y - SOurce.y) * (Target.y - SOurce.y);
    return (fx_delta - fx) / delta;
}

void icp_scan_matching(std::ofstream& gnuplot_script, const std::vector<Point>& Source, const std::vector<Point>& target){
std::vector<Point> transformed_source = Source;
float previous_error_sum = std::numeric_limits<float>::max(); // 前回の誤差を最大値で初期化
for(int iter = 0; iter <= MAX_iteration; ++iter){
 std::vector<Point> target_closest;
 float error_sum = 0;
 double gradDx = 0;
 double gradDy = 0;
 double gradTheta = 0;
 float dx = 0;
 float dy = 0;
 float dth = 0;
 float dtheta = 0;
 for(const auto& Source : transformed_source ){
  int index = findClosestPoint(Source,target);
  target_closest.push_back(target[index]);
  Point error = {target[index].x - Source.x, target[index].y - Source.y};
  Point Target = {target[index].x, target[index].y};
  Point SOurce = {Source.x, Source.y};
  error_sum += error.x * error.x + error.y * error.y;
   //std::cout << "error_sum: " << error_sum << std::endl;
  gradDx += diffx(Target, SOurce);
  gradDy += diffy(Target, SOurce);
  gradTheta += difftheta(Target, SOurce);

 //std::cout << "gradTheta: " << gradTheta << std::endl;
 }
  int num_points =Source.size(); // Sourceの点の数を取得

dx = (-gradDx / num_points) * learning_rate;
dy = (-gradDy / num_points) * learning_rate;
dth = (-gradTheta / num_points) * learning_rate;
  //std::cout << "dx: " << dx << std::endl;
 // std::cout << "dy: " << dy << std::endl;
  //std::cout << "dth: " << dy << std::endl;
 //dx = -gradDx* learning_rate;
for(auto& Source : transformed_source){
    //Source.x += dx;
    float x_new = Source.x * cos(dth) - Source.y * sin(dth);
    float y_new = Source.x * sin(dth) + Source.y * cos(dth);
    Source.x = x_new + dx;
    Source.y = y_new + dy;
}
plot(gnuplot_script, target, transformed_source, true,iter);

/*収束条件のチェック*/
if(std::abs(previous_error_sum - error_sum) < EPS){
std::cout << "Converged after " << iter << "iterations." << std::endl;
break;
}
previous_error_sum = error_sum;//前回の誤差を更新
}

//plot(target, transformed_source, true);
}


int main(void){
    auto start_time = std::chrono::high_resolution_clock::now();
    std::vector<Point> current = read_scan_points("点群ファイル/scan_1.txt");
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
    // gnuplot_script << "set size ratio 1\n";
    // gnuplot_script << "set xrange [-20:20]\n";
    // gnuplot_script << "set xrange [-20:20]\n";
    // gnuplot_script << "set yrange [-20:20]\n";
    icp_scan_matching(gnuplot_script, Source,target);
    // gnuplot_script.close();
    // std::string gnuplot_command = "gnuplot -p plot_commands.gp";
    // /*if (block) {
    //     std::cout << "Press Enter to continue...";
   auto end_time = std::chrono::high_resolution_clock::now();
   auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
   std::cout << "ICP algorithm completed in " << duration.count() << " millseconds." << std::endl;
    return 0;
}
