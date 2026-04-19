#include "lemlib/chassis/chassis.hpp"
#include <vector>
#include <cmath>

class PathGenerator {
public:
    /**
     * @brief 核心算法：生成带 S 曲线的样条路径点
     * @param waypoints 关键点 {x, y, theta} (theta 仅在起点和终点生效)
     * @param maxVel 巡航速度 (1-127)
     * @param sampleDist 采样步长 (建议 2.0 吋)
     */
    static std::vector<lemlib::Pose> generate(std::vector<lemlib::Pose> waypoints, float maxVel, float sampleDist = 3.0) {
        std::vector<lemlib::Pose> fullPath;
        if (waypoints.size() < 2) return fullPath;

        // 1. 遍历每一段路径 (Segment)
        for (size_t i = 0; i < waypoints.size() - 1; i++) {
            lemlib::Pose p0 = waypoints[i];
            lemlib::Pose p1 = waypoints[i+1];

            // 计算切线强度 (Mag)
            float dist = p0.distance(p1);
            float mag = dist * 0.6f;

            // 角度转弧度 (假定 theta 存储的是度数)
            float a0 = p0.theta * M_PI / 180.0f;
            float a1 = p1.theta * M_PI / 180.0f;

            // 如果是中间点，切线方向取两点连线
            if (i > 0) a0 = std::atan2(p1.y - waypoints[i-1].y, p1.x - waypoints[i-1].x);

            // 2. 样条插值采样
            int steps = std::max(5, (int)(dist / sampleDist));
            for (int j = 0; j < steps; j++) {
                float t = (float)j / steps;
                fullPath.push_back(hermite(p0, a0, mag, p1, a1, mag, t));
            }
        }
        fullPath.push_back(waypoints.back());

        // 3. 注入 S 曲线速度规划
        injectVelocity(fullPath, maxVel);
        
        return fullPath;
    }

private:
    // 三次埃尔米特数学公式
    static lemlib::Pose hermite(lemlib::Pose p0, float a0, float m0, lemlib::Pose p1, float a1, float m1, float t) {
        float h00 = 2*std::pow(t,3) - 3*std::pow(t,2) + 1;
        float h10 = std::pow(t,3) - 2*std::pow(t,2) + t;
        float h01 = -2*std::pow(t,3) + 3*std::pow(t,2);
        float h11 = std::pow(t,3) - std::pow(t,2);

        float x = h00*p0.x + h10*m0*std::cos(a0) + h01*p1.x + h11*m1*std::cos(a1);
        float y = h00*p0.y + h10*m0*std::sin(a0) + h01*p1.y + h11*m1*std::sin(a1);
        return lemlib::Pose(x, y, 0); // theta 留给速度
    }

    // S 曲线速度注入器
    static void injectVelocity(std::vector<lemlib::Pose>& path, float maxVel) {
            size_t n = path.size();
            
            // --- 物理调优参数 ---
            float startVel = 80.0f;  // 提高起步速度，直接冲破静摩擦 (原 20)
            float endVel = 15.0f;    // 终点不要直接切 0，留一点爬行速度给 PID 纠偏
            float accelExit = 0.3f;  // 加速段延长到 30% (原 20%)
            float decelStart = 0.6f; // 减速段提前到 60% 处就开始 (原 80%)

            for (size_t i = 0; i < n; i++) {
                float progress = (float)i / (n - 1);

                if (progress < accelExit) { 
                    // 快速起步 S 曲线
                    float t = progress / accelExit;
                    float factor = 0.5f * (1.0f - std::cos(M_PI * t));
                    path[i].theta = startVel + (maxVel - startVel) * factor;
                } 
                else if (progress > decelStart) { 
                    // 提早且平缓的减速
                    float t = (progress - decelStart) / (1.0f - decelStart);
                    float factor = 0.5f * (1.0f + std::cos(M_PI * t));
                    // 减速到 endVel 而不是 0，防止最后一段完全没力气
                    path[i].theta = endVel + (maxVel - endVel) * factor;
                } 
                else {
                    path[i].theta = maxVel;
                }
            }
            
            // 只有最后一个点强制为 0，触发 LemLib 的停止逻辑
            path.back().theta = 0; 
        }
};