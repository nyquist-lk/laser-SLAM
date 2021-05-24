#include <ros/ros.h>
#include <iostream>
#include <ceres/ceres.h>
#include <chrono>

using namespace std;
using namespace ceres;

struct SolverData
{
    double x;
    double y;
    double th;
};

struct LINE_FITTING_COST
{
    LINE_FITTING_COST(double x, double y):
        _x(x), _y(y) {}

    // 残差计算
    template<typename T>
    bool operator() (const T* const line_params, // 模型参数，k b
                     T* residual) const // 残差
    {
        residual[0] = T(_y) - line_params[0] * _x + line_params[1];
        return true;
    }
    
    const double _x, _y; // ｘ, y 数据
};

struct CIRCLE_FITTING_COST
{
    CIRCLE_FITTING_COST(double x, double y) : _x(x), _y(y) {}
    // 残差计算
    template <typename T>
    bool operator() (
        const T* const circle_params, // 模型参数，x0 y0 r^2
        T* residual) const // 残差
    {
        residual[0] = circle_params[2] - 
                    (T(_x) - circle_params[0]) * (T(_x) - circle_params[0]) -
                    (T(_y) - circle_params[1]) * (T(_y) - circle_params[1]);
                
        return true;
    }

    const double _x, _y;
};

class GSSlover
{
public:
    GSSlover() {}
    ~GSSlover() {}

    bool GetOptimzeParams(std::vector<SolverData>& datas, 
                                        std::string type,
                                        std::vector<double>& params);

private:
    double m_line_params[2];
    double m_circle_params[3];
};