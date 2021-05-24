#include "solver.h"


bool GSSlover::GetOptimzeParams(std::vector<SolverData>& datas, 
                                            std::string type, 
                                            std::vector<double>& params)
{
    if (type == "line")
    {
        m_line_params[0] = 0.01;
        m_line_params[1] = 0.01;
    }
    else if (type == "circle")
    {
        m_circle_params[0] = 0.0;
        m_circle_params[1] = 0.0;
        m_circle_params[2] = 1.0;
    }
    else
    {
        ROS_INFO("type is not line or circle");
        return false;
    }

    ceres::Problem problem;
    for (int i = 0; i < datas.size(); i++)
    {
        if (type == "line")
        {
            problem.AddResidualBlock( //　向问题中添加误差项
                new ceres::AutoDiffCostFunction<LINE_FITTING_COST, 1, 2>(
                    new LINE_FITTING_COST(datas[i].x, datas[i].y)
                ),
                nullptr,  // 核函数，这里不使用，为空
                m_line_params
            );
        }

        if (type == "circle")
        {
            problem.AddResidualBlock( //　向问题中添加误差项
                new ceres::AutoDiffCostFunction<CIRCLE_FITTING_COST, 1, 3>(
                    new CIRCLE_FITTING_COST(datas[i].x, datas[i].y)
                ),
                nullptr,  // 核函数，这里不使用，为空
                m_circle_params
            );
        }
    }

    // 配置求解器
    ceres::Solver::Options options; //这里有很多配置项可以填
    options.linear_solver_type = ceres::DENSE_QR; //　增量方程如何求解
    options.minimizer_progress_to_stdout = true; // 输出到cout

    ceres::Solver::Summary summary; //优化信息
    ceres::Solve(options, &problem, &summary); // 开始优化

    cout << summary.BriefReport() << endl;

    if (type == "line")
    {
        params.push_back(m_line_params[0]);
        params.push_back(m_line_params[1]);
    }
    else if (type == "circle")
    {
        params.push_back(m_circle_params[0]);
        params.push_back(m_circle_params[1]);
        params.push_back(m_circle_params[2]);
    }

    return true;
}