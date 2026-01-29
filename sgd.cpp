#include "sgd.h"
#include <random>
#include <algorithm>

void shuffle_data(std::vector<Point>& points) {
    std::random_device rd;
    std::mt19937 g(rd());
    std::shuffle(points.begin(), points.end(), g);
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
void rescale_transformation_matrix(mat3x3 &matrix, float max_absolute)
{
    // 平行移動成分 (x, y) だけを元のサイズに戻す
    matrix.x = matrix.x * max_absolute;
    matrix.y = matrix.y * max_absolute;

    // ※重要: 回転成分 (matrix.a, matrix.b, matrix.c, matrix.d) は
    // 角度に関するものなので、スケールの影響を受けないため触らない！
}