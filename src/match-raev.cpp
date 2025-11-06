#include <vector>
#include <cmath>
#include <limits>
#include <optional>
#include <algorithm>
#include <iostream>
#include <chrono>
#include <string>

#ifndef CVTA_DEG_RAD
#define CVTA_DEG_RAD(x) ((x)/57.29577951308232)
#endif

#ifndef CVTA_RAD_DEG
#define CVTA_RAD_DEG(x) ((x)*57.29577951308232)
#endif

struct Target
{
    double range_m;      // 距离
    double az_deg;       // 方位 [0,360)
    double el_deg;       // 俯仰 [-90,90] or [ -180,180 ) 视你的定义
    double vr;           // 径向速度（有符号）
};

// 环形角差，返回最小差（度）
static inline double ang_diff_deg(double a, double b)
{
    return std::remainder(a - b, 360.0); // (-180,180]
}

static inline double clip(double x, double lo, double hi) {
    return std::max(lo, std::min(x, hi));
}

struct MatchResult
{
    int index;        // 目标索引
    double mu;        // 隶属度
    double d2;        // 归一化距离平方
};

struct MatchParams {
    double sigma_r_m     = 500.0;  // 距离σ
    double az_sigma_min  = 0.05;   // 角度σ下限（deg）
    double az_sigma_max  = 1.0;    // 角度σ上限（deg）
    double v_gate_tight  = 100.0;  // 速度粗门（abs Δv < 100 m/s）
};


std::optional<MatchResult>
match_best(const std::vector<Target>& targets,
           const Target& meas,
           const MatchParams& p = {})
{
    // 先进行速度初筛
    struct Cand { int idx; double dv_abs; };
    std::vector<Cand> Cands;
    Cands.reserve(targets.size());
    for (int i = 0; i < (int)targets.size(); ++i)
    {
        double dv = std::abs(targets[i].vr - meas.vr);
        if( dv < p.v_gate_tight)
            Cands.push_back({i,dv});
    }

    if(Cands.empty())
    {
        for (int i = 0; i < (int)targets.size(); ++i)
            Cands.push_back({i, std::abs(targets[i].vr - meas.vr)});
    }

    int best_idx = -1;
    double best_d2 = std::numeric_limits<double>::infinity();

    auto sigma_ang_deg = [&](double Rm)
    {
        double R = std::max(Rm, 1.0);
        double s = CVTA_RAD_DEG(p.sigma_r_m / R); // σ_ang = σ_r/R (rad) -> deg
        return clip(s, p.az_sigma_min, p.az_sigma_max);
    };

    // 根据RAE细筛
    for (const auto& c : Cands)
    {
        const auto& t = targets[c.idx];

        double R = std::max(t.range_m, 1.0);
        double sig_ang = sigma_ang_deg(R);

        double dr  = (meas.range_m - t.range_m) / p.sigma_r_m;
        double daz = ang_diff_deg(meas.az_deg, t.az_deg) / sig_ang;
        double del = (meas.el_deg - t.el_deg) / sig_ang;

        double d2 = dr*dr + daz*daz + del*del;
        if (d2 < best_d2)
        {
            best_d2 = d2;
            best_idx = c.idx;
        }
    }

    if (best_idx == -1) return std::nullopt;

    double mu = std::exp(-0.5 * best_d2);
    return MatchResult{ best_idx, mu, best_d2 };
}

// 测试辅助函数
void printTarget(const Target& t, const std::string& name) {
    std::cout << name << ": range=" << t.range_m << "m,azi=" << t.az_deg
              << ",ele=" << t.el_deg << ", vel=" << t.vr << "m/s" << std::endl;
}

void printMatchResult(const std::optional<MatchResult>& result, const std::string& testName)
{
    if (result->index != -1) {
        std::cout << testName << " - match success: index=" << result->index
                  << ", membership=" << result->mu << ", distance square=" << result->d2 << std::endl;
    } else {
        std::cout << testName << " - no match result" << std::endl;
    }
}

int main()
{

    std::cout << "=== target match algorithm test case ===" << std::endl << std::endl;
#if 1
    // 测试用例1: 单个目标匹配
    std::cout << "[test case 1: single target match]" << std::endl;
    {
        std::vector<Target> targets = {
            {1000.0, 45.0, 10.0, 50.0}  // 距离1000m, 方位45°, 俯仰10°, 速度50m/s
        };

        Target measurement = {1050.0, 46.0, 11.0, 55.0};  // 测量值

        printTarget(targets[0], "target 1");
        printTarget(measurement, "measurement");

        auto result = match_best(targets, measurement);
        printMatchResult(result, "test 1");
    }
    std::cout << std::endl;
#endif
#if 1
    // 测试用例2: 3个目标匹配
    std::cout << "[test case 2: 3 target match]" << std::endl;
    {
        std::vector<Target> targets = {
            {800.0, 30.0, 5.0, 40.0},   // 目标1
            {1200.0, 60.0, 15.0, 60.0}, // 目标2
            {1500.0, 90.0, 20.0, 80.0}  // 目标3
        };

        Target measurement = {1250.0, 62.0, 16.0, 65.0};  // 应该匹配目标2

        for (int i = 0; i < targets.size(); i++) {
                printTarget(targets[i], "target " + std::to_string(i+1));
        }
        printTarget(measurement, "measurement");

        auto result = match_best(targets, measurement);
        printMatchResult(result, "test 2");
    }
    std::cout << std::endl;
#endif
#if 1
    // 测试用例3: 10个目标匹配（最大数量）
    std::cout << "[test case 3: 10 target match]" << std::endl;
    {
        std::vector<Target> targets = {
            {50000.0, 0.0, 0.0, 200.0},    // target 1
            {60000.0, 36.0, 5.0, 230.0},   // target 2
            {70000.0, 72.0, 10.0, 400.0},  // target 3
            {80000.0, 108.0, 15.0, 300.0}, // target 4
            {90000.0, 144.0, 20.0, 201.0}, // target 5
            {100000.0, 180.0, 25.0, 500.0},// target 6
            {210000.0, 216.0, 30.0, 800.0},// target 7
            {320000.0, 252.0, 35.0, 200.0},// target 8
            {330000.0, 288.0, 40.0, 300.0},// target 9
            {440000.0, 324.0, 45.0, 1000.0} // target 10
        };

        Target measurement = {95000.0, 150.0, 22.0, 300.0};  // 应该匹配target 5

        std::cout << "target number: " << targets.size() << std::endl;
        printTarget(measurement, "measurement");

        auto result = match_best(targets, measurement);
        printMatchResult(result, "test 3");
    }
    std::cout << std::endl;
#endif
#if 1
    // 测试用例4: 速度门限测试
    std::cout << "[test case 4: speed gate test]" << std::endl;
    {
        std::vector<Target> targets = {
            {1000.0, 45.0, 10.0, 50.0},  // 目标1: 速度50m/s
            {1000.0, 45.0, 10.0, 200.0}  // 目标2: 速度200m/s (超出默认门限100m/s)
        };

        Target measurement = {1050.0, 46.0, 11.0, 55.0};  // 速度55m/s，应该匹配目标1

        printTarget(targets[0], "target 1");
        printTarget(targets[1], "target 2");
        printTarget(measurement, "measurement");

        auto result = match_best(targets, measurement);
        printMatchResult(result, "test 4");
    }
    std::cout << std::endl;
#endif
#if 1
    // 测试用例5: 角度边界测试（0°和360°边界）
    std::cout << "[test case 5: angle boundary test]" << std::endl;
    {
        std::vector<Target> targets = {
            {1000.0, 1.1, 10.0, 50.0},   // 目标1: 5°
            {1000.0, 355.0, 10.0, 50.0}  // 目标2: 355°
        };

        Target measurement = {1050.0, 358.0, 11.0, 55.0};  // 358°，应该匹配目标2

        printTarget(targets[0], "target 1");
        printTarget(targets[1], "target 2");
        printTarget(measurement, "measurement");

        auto result = match_best(targets, measurement);
        printMatchResult(result, "test 5");
    }
    std::cout << std::endl;
#endif
#if 1
    // 测试用例6: 距离变化测试
    std::cout << "[test case 6: distance change test]" << std::endl;
    {
        std::vector<Target> targets = {
            {100.0, 45.0, 10.0, 50.0},   // 目标1: 近距离
            {10000.0, 45.0, 10.0, 50.0}  // 目标2: 远距离
        };

        Target measurement = {150.0, 46.0, 11.0, 55.0};  // 应该匹配目标1

        printTarget(targets[0], "target 1");
        printTarget(targets[1], "target 2");
        printTarget(measurement, "measurement");

        auto result = match_best(targets, measurement);
        printMatchResult(result, "test 6");
    }
    std::cout << std::endl;
#endif
#if 1
    // 测试用例7: 无匹配测试（空目标列表）
    std::cout << "[test case 7: no match test]" << std::endl;
    {
        std::vector<Target> targets = {};  // 空目标列表

        Target measurement = {1000.0, 45.0, 10.0, 50.0};

        printTarget(measurement, "measurement");
        std::cout << "target number: " << targets.size() << std::endl;

        auto result = match_best(targets, measurement);
        printMatchResult(result, "test 7");
    }
    std::cout << std::endl;
#endif
#if 1
    // 测试用例8: 性能测试（10个目标，多次匹配）
    std::cout << "[test case 8: performance test]" << std::endl;
    {
        std::vector<Target> targets = {
            {500.0, 0.0, 0.0, 30.0}, {600.0, 36.0, 5.0, 35.0}, {700.0, 72.0, 10.0, 40.0},
            {800.0, 108.0, 15.0, 45.0}, {900.0, 144.0, 20.0, 50.0}, {1000.0, 180.0, 25.0, 55.0},
            {1100.0, 216.0, 30.0, 60.0}, {1200.0, 252.0, 35.0, 65.0}, {1300.0, 288.0, 40.0, 70.0},
            {1400.0, 324.0, 45.0, 75.0}
        };

        std::vector<Target> measurements = {
            {550.0, 5.0, 2.0, 32.0}, {650.0, 40.0, 7.0, 37.0}, {750.0, 75.0, 12.0, 42.0},
            {850.0, 110.0, 17.0, 47.0}, {950.0, 145.0, 22.0, 52.0}
        };

        auto start = std::chrono::high_resolution_clock::now();

        for (int i = 0; i < 1000; i++) {  // 执行1000次匹配
            for (const auto& meas : measurements) {
                auto result = match_best(targets, meas);
            }
        }

        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);

        std::cout << "execute 1000 times match (10 targets × 5 measurements) time: "
                    << duration.count() << " microseconds" << std::endl;
            std::cout << "average each match time: " << duration.count() / 5000.0 << " microseconds" << std::endl;
    }
    std::cout << std::endl;
#endif
    std::cout << "=== all test cases executed ===" << std::endl;

    return 0;
}


