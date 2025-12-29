#ifndef CONSTANTS_H
#define CONSTANTS_H

#include <cmath> // M_PIを使うため

constexpr int MAX_ITERATION = 1000; // 最大反復回数 (ICPなどで使用)
constexpr double learning_rate_xy = 0.05;
constexpr double learning_rate_th = 0.007;
constexpr double learning_rate = 0.01;
constexpr double EPS = 1e-7;

// 例: 0.3m (30cm) 以内の点を使って法線を計算する
constexpr double NORMAL_CALC_RADIUS = 0.3;

// 対応点とみなす最大距離の閾値 (の2乗)
// 例: 0.5m以内を有効とする -> 0.5 * 0.5 = 0.25
constexpr double MAX_CORRESPONDENCE_DIST_SQ = 0.16;

constexpr double RAD2DEG = 180.0 / M_PI;
constexpr double DEG2RAD = M_PI / 180.0;

/*微小変位*/
constexpr double delta = 1.0e-7;

// 関数形式で使いたい場合
inline constexpr double rad2deg(double rad) { return rad * RAD2DEG; }
inline constexpr double deg2rad(double deg) { return deg * DEG2RAD; }

#endif // CONSTANTS_H

