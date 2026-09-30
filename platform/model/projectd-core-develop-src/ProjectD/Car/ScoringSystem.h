#pragma once

#include "Core/Core.h"
#include <string>
#include <vector>
#include <unordered_map>
#pragma pack(push, 8)          // <== 新增，强制该类型用 8 对齐
namespace D
{

// 仅前置声明，避免在头文件强依赖 Car 定义；实现见 .cpp
struct Car;

struct ScoringVar
{
    std::string name;
    float       value;
};

struct ScoringConfig
{
    ScoringConfig();
    ~ScoringConfig();

    static ScoringConfig* get();

    // 初始化默认参数（.cpp 内包含最优控制所需的权重/阈值）
    void initDefaults();
    void removeVars();

    // 变量管理
    void  addVar(const std::string& name, float value);
    void  setVar(const std::string& name, float value);
    float getVar(const std::string& name) const;

    std::vector<ScoringVar*>                          vvars;
    std::unordered_map<std::string, ScoringVar*>      mvars;

};

struct ScoringSystem
{
    ScoringSystem();
    ~ScoringSystem();

    // 绑定车辆与配置
    void init(struct Car* car);

    // 每帧调用：先 computeDriftScore()（仅遥测），再 computeAgentReward()
    void step(float dt);

    // 清零累计奖励（等价于清零累计代价的相反数）
    void reset();

    // === 评分/代价计算 ===
    // 在 .cpp 中采用“代价聚合”：
    //   stepReward = -stepCost; totalReward 累加 stepReward
    void computeAgentReward(float dt);

    // === 漂移统计（保留为遥测用；不再影响奖励/代价）===
    void computeDriftScore(float dt);
    void resetDrift();
    void validateDrift();
    bool checkExtremeDrift(float triggerSlipLevel = 0.8f) const;
    float scoreDriftPenalty(float dt);

    // 便捷访问配置
    inline float getVar(const std::string& name) const { return config->getVar(name); }

    // === 依赖对象 ===
    struct Car*      car     = nullptr;
    ScoringConfig*   config  = nullptr;

    // === 漂移统计缓存（仅用于遥测/可视化）===
    bool  drifting             = false;
    bool  driftExtreme         = false;
    bool  driftInvalid         = false;
    float currentDriftAngle    = 0.0f;
    float currentSpeedMultiplier = 0.0f;
    float lastDriftDirection   = 0.0f;
    float driftStraightTimer   = 0.0f;
    float instantDriftDelta    = 0.0f;
    float instantDrift         = 0.0f;
    float driftPoints          = 0.0f;
    float e_y                  = 0.0f;
    float e_psi                = 0.0f;
    int   driftNow             = 0;
    float ratio                = 0.0f;   
    float tang                 = 0.0f;   
    float driftComboCounter    = 0;
    float g_lastFramePenalty   = 0;

    float m_prevSteer = 0.0f;
    float m_prevLong  = 0.0f;   // longCmd = gas - brake
    float m_prevDSteer = 0.0f;  // 上一帧 dSteer
    float m_prevDLong  = 0.0f;  // 上一帧 dLong
    int   m_prevSteerSign = 0;  // 用于 flip 检测
    float discarandbound =0.0f;

    // --- crossing 统计状态 ---
    float crossPrevEySign   = 0.0f;  // 上一帧 e_y（去抖后）的符号：-1/0/+1；0 表示尚未“武装”
    float crossCooldownLeft = 0.0f;  // 冷却倒计时（秒）

    int   m_crossCount = 0;        // 本episode累计有效穿越次数
    float m_crossCooldown = 0.0f;  // 冷却计时器（秒）
    int   m_prevSide = 0;          // 上一次“有效侧”：-1=左, +1=右, 0=未知

    // === 对外累计量（接口保持 reward 语义；内部等价最小化 cost）===
    float trackW            = 0.0f;  
    float C_track           = 0.0f;  
    float C_speed           = 0.0f;  
    float C_smooth          = 0.0f;  

    float cost_y            = 0.0f;  
    float cost_psi          = 0.0f;  
    float cost_cross        = 0.0f;   
    float cost_bound        = 0.0f;  
    float cost_drift        = 0.0f;  

    float stepReward           = 0.0f;   
    float totalReward          = 0.0f;   
    float prevEpisodeReward    = 0.0f;  

    // === 轨迹点缓存（延续原逻辑）===
    int   oldPointId           = 0;
    int   oldSplinePointId     = 0;
};

} // namespace D
#pragma pack(pop)