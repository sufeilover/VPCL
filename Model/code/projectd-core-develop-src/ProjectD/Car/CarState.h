#pragma once

#include "Car/CarControls.h"
#include "Core/Math.h"
#include <array>
#include <vector>
#include <algorithm>
#include <cmath>
#include <cstdint>

namespace D {

#pragma pack(push, 8)

struct CarState
{
    // ===== 基本标识 & 时间戳 =====
    int32_t carId = 0;
    int32_t simId = 0;
    float   timestamp = 0;

    // ===== 控制量 =====
    CarControls controls;

    // ===== 事件/状态标记 =====
    int32_t collisionFlag   = 0;
    int32_t outOfTrackFlag  = 0;
    int32_t trackPointId    = 0;
    float   lastTrackPointTimestamp = 0;
    float   trackLocation   = 0;
    float   kappa           = 0;
    int     driftNow        = 0;

    vec3f trackLeft;
    vec3f trackRight;
    
    // ===== 与赛道相对的角度（弧度） =====
    float bodyVsTrack     = 0;   // rad
    float velocityVsTrack = 0;   // rad

    // ===== 传动/转速/速度等 =====
    double finalRatio     = 0;
    double diffPowerRamp  = 0;
    double diffCoastRamp  = 0;
    double outShaftLvelocity = 0;
    double outShaftRvelocity = 0;
    double drivevelocity  = 0;
    double rootVelocity   = 0;
    double enginevelocity = 0;

    float engineRPM = 0;
    float speedMS   = 0;
    int32_t gear    = 0;      // 0=R,1=N,2=H1,3=H2,4=H3,5=H4,6=H5,7=H6
    int32_t gearGrinding = 0;

    float travelstrut1;
	float travelstrut2;
	float travelaxle1;	
    float travelaxle2;	

    // ===== 姿态/位置/速度（世界/车体系） =====
    mat44f bodyMatrix;
    mat44f fuelTankyMatrix;
    mat44f hub1Matrix;
    mat44f hub2Matrix;
    mat44f strutBodyM1;
    mat44f strutBodyM2;
    mat44f axleMatrix;
    vec3f  bodyPos;                  // (x,y,z)
    vec3f  bodyEuler;
    vec3f  accG;
    vec3f  velocity;
    vec3f  localVelocity;            // (vx,vy,vz) in car frame
    vec3f  angularVelocity;
    vec3f  localAngularVelocity;

    std::array<mat44f, 4> hubMatrix;
    std::array<vec3f, 4>  tyreContacts;
    std::array<vec3f, 4>  tyrecontactNormal;
    std::array<vec3f, 4>  tyreroadHeading;
    std::array<vec3f, 4>  tyreroadRight;
    std::array<float, 4>  tyreLoad    {0,0,0,0};
    std::array<float, 4>  fDepth    {0,0,0,0};
    std::array<float, 4>  tyreAngularSpeed {0,0,0,0};
    std::array<float, 4>  tyreSlipRatio {0,0,0,0};
    std::array<float, 4>  tyreNdSlip   {0,0,0,0};
    std::array<float, 4>  slipAngleRAD {0,0,0,0};
    std::array<float, 4>  fy          {0,0,0,0};
    std::array<vec3f, 4>  tyreRayOrigin{};  // 轮胎 RayCaster 默认起点（世界坐标）
    std::array<vec3f, 4>  tyreRayDir{};     // 轮胎 RayCaster 默认方向（单位向量，世界坐标）
    std::array<float, 4>  tyreRayLength{};  // 轮胎 RayCaster 默认长度（米）

    // ===== 探针/前瞻 =====
    enum { MaxProbes = 10 };
    std::array<float, MaxProbes> probes{};

    enum { MaxLookAhead = 5 };
    std::array<float, MaxLookAhead> lookAhead{};

    // ===== 对外奖励（注意新版语义：reward = -cost），当前帧 =====
    float trackW           = 0.0f;  
    float Ctrack           = 0.0f;  
    float Cspeed           = 0.0f;  
    float Csmooth          = 0.0f;  
    float costy           = 0.0f;  
    float costpsi           = 0.0f;  
    float costcross          = 0.0f;   
    float costbound           = 0.0f;  
    float costdrift          = 0.0f;  
    float stepReward  = 0;
    float totalReward = 0;

    // ------------------------------------------------------------------
    // 实时直出派生指标（当前帧，标准单位）
    // ------------------------------------------------------------------
    float driftComboCounter  = 0.0f;  // rad：侧滑角 β
    float lateralUtil        = 0.0f;  // Σ|Fy|/Σ|Fz|
    float ePsi               = 0.0f;  // rad：航向误差（已wrap到[-π,π]）
    float eY                 = 0.0f;  // m：横向误差（相对中心线）
    float glastFramePenalty  = 0.0f;
    float vAlongTrack        = 0.0f;  // m/s：沿赛道切线的速度
    float borderMarginRatio = 1.0f;  // 0..1

    // ===== Basicline（环形缓冲）样本：供活跃段/方向优化使用 =====
    struct BasiclineSample
    {
        float timestamp = 0.f;

        // 位置与相对角（序列）
        vec3f bodyPos{0,0,0};
        float bodyVsTrack     = 0.f;  // rad
        float velocityVsTrack = 0.f;  // rad

        // 安全/状态（序列）
        int32_t collisionFlag  = 0;
        int32_t outOfTrackFlag = 0;

        // 轮胎力/载荷（序列）
        std::array<float, 4> fy{0,0,0,0};
        std::array<float, 4> tyreLoad{0,0,0,0};

        // --- 派生指标（序列）---
        float kappa       = 0.0f;  // 1/m：局部曲率
        int32_t bestPoint = 0;  // 1/m：局部曲率
        float driftComboCounter     = 0.0f;  // rad：侧滑角
        float lateralUtil = 0.0f;  // Σ|Fy|/Σ|Fz|

        // --- 新增：把原来仅“当前帧”的派生指标也串进序列 ---
        float ePsi              = 0.0f;  // rad
        float eY                = 0.0f;  // m
        float glastFramePenalty  = 0.0f;
        float borderMarginRatio = 1.0f;  // 0..1
        float vAlongTrack       = 0.0f;  // m/s

        // --- 奖励（序列版本）---
        float stepReward  = 0.0f;
        float totalReward = 0.0f;  // 若希望“样本时刻的累计值”，可以写入；否则保持 0
    };

    // 固定容量（保持 300；可按需调大）
    static constexpr int BasiclineCap = 300;

    // 容器：固定 200 的环形缓冲
    std::array<BasiclineSample, BasiclineCap> basicline{};
    int basiclineHead  = 0; // 下一次写入的位置
    int basiclineCount = 0; // 已写入元素数（<=200）

    inline void basiclineClear()
    {
        basiclineHead  = 0;
        basiclineCount = 0;
    }

    inline void basiclinePush(const BasiclineSample& s)
    {
        basicline[basiclineHead] = s;
        basiclineHead = (basiclineHead + 1) % BasiclineCap;
        if (basiclineCount < BasiclineCap) basiclineCount++;
    }

    inline int basiclineSize() const { return basiclineCount; }

    // 逻辑序号（0=最旧 … size-1=最新）转物理下标
    inline int basiclineLogicalToPhysical(int logical) const
    {
        const int start = (basiclineHead - basiclineCount + BasiclineCap) % BasiclineCap;
        return (start + logical) % BasiclineCap;
    }

    // 导出最近 N 帧（从旧到新）
    inline void basiclineExportLastN(std::vector<BasiclineSample>& out, int N) const
    {
        out.clear();
        const int n = std::min(N, basiclineCount);
        out.reserve(n);
        const int start = (basiclineHead - n + BasiclineCap) % BasiclineCap;
        for (int i = 0; i < n; ++i)
            out.push_back(basicline[(start + i) % BasiclineCap]);
    }
};
} // namespace D
#pragma pack(pop)
