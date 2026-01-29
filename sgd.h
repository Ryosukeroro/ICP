#ifndef SGD_H
#define SGD_H

#include <vector>
#include "point.h"

constexpr size_t m_batch_size = 40;
// データのシャッフル
void shuffle_data(std::vector<Point>& points);

float get_absolute_max(const std::vector<Point>& source, const std::vector<Point>& target);
std::vector<Point> normalise_clouds(const std::vector<Point>& cloud, float max_absolute);
void rescale_transformation_matrix(double &x, double &y, float max_absolute);


#endif // SGD_H