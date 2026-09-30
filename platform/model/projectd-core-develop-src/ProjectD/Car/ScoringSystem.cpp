//=================================================================================================
// ScoringSystem.cpp  —— 代价函数版（用于最优控制）
// （已改：漂移检测与惩罚改为 slipAngle + NdSlip≥0.5，0/0.2/0.3 rad 三档 + 三档惩罚）
//
// 变更要点：
// - 不再用 slipRatio 判漂移；仅当 NdSlip ≥ SkidNdGate(默认0.5) 时，按 slipAngle(rad) 分档：
//     > SlipAngleMinorRad(默认0.0)   → 轻微(1)
//     > SlipAngleModerateRad(默认0.2)→ 中等(2)
//     > SlipAngleSevereRad(默认0.3)  → 严重(3)
// - 三档惩罚：wDriftMinor / wDriftModerate / wDriftSevere（逐轮计数求和；再乘速度倍率与 dt）。
// - 极限漂移：至少两条达到“严重”档视为极限（driftExtreme=true），再乘 DriftExtremeMul。
// - 其余代价/奖励框架保持原语义。
//=================================================================================================

#include "Car/ScoringSystem.h"
#include "Car/Car.h"
#include "Car/Tyre.h"
#include "Car/Drivetrain.h"
#include "Car/Engine.h"
#include "Sim/Track.h"
#include <cfloat>
#include <cmath>

#define DECL_VAR(name) \
	static const std::string Name_##name (#name)

//===================== 旧有参数名（保留） =====================
DECL_VAR(SmoothSteerSpeed);
DECL_VAR(MinBonusSpeed);
DECL_VAR(MaxBonusSpeed);
DECL_VAR(StallRpm);
DECL_VAR(DirectionThreshold);
DECL_VAR(OutOfTrackThreshold);
DECL_VAR(ApproachDistance);
DECL_VAR(CriticalDistance);

DECL_VAR(TravelBonus);
DECL_VAR(TravelSplineBonus);
DECL_VAR(DriftBonus);
DECL_VAR(SpeedBonus);
DECL_VAR(ThrottleBonus);
DECL_VAR(EngineRpmBonus);
DECL_VAR(DirectionBonus);

DECL_VAR(DirectionPenalty);
DECL_VAR(ObstApproachPenalty);
DECL_VAR(CollisionPenalty);
DECL_VAR(OffTrackPenalty);
DECL_VAR(GearGrindPenalty);
DECL_VAR(StallPenalty);

//===================== 新增：用于“代价函数”的权重与阈值 =====================
// [NEW] 轨迹跟踪
DECL_VAR(wPosErr);          // 横向（到中心线）的平方代价权重
DECL_VAR(wYawErr);          // 航向角误差平方代价权重
// [NEW] β（侧滑角）监管（仍保留）
DECL_VAR(wBetaBand);
DECL_VAR(wBetaBar);
DECL_VAR(betaLoDeg);
DECL_VAR(betaHiDeg);
DECL_VAR(betaSafeDeg);

// [NEW] 控制平滑
DECL_VAR(wDeltaSteer);
DECL_VAR(wJerkSteer);
DECL_VAR(wDeltaLong);
// [NEW] 进度（沿赛道前进）——负代价（=正向奖励）
DECL_VAR(wProgress);
DECL_VAR(wSteerCenter);
DECL_VAR(wSteerFlip);

namespace D {

ScoringConfig::ScoringConfig() { initDefaults(); }
ScoringConfig::~ScoringConfig() { removeVars(); }

ScoringConfig* ScoringConfig::get()
{
	static ScoringConfig inst;
	return &inst;
}

void ScoringConfig::initDefaults()
{
	//===================== 原默认参数（保留） =====================
	addVar(Name_SmoothSteerSpeed, 10.0f);
	addVar(Name_MinBonusSpeed, 0.1f);
	addVar(Name_MaxBonusSpeed, 50.0f);
	addVar(Name_StallRpm, 200.0f);
	addVar(Name_DirectionThreshold, 0.75f);
	addVar(Name_OutOfTrackThreshold, 0.44f);
	addVar(Name_ApproachDistance, 2.0f);
	addVar(Name_CriticalDistance, 1.2f);

	// 旧奖励/惩罚（默认仍为0，不影响代价）
	addVar(Name_TravelBonus, 0.0f);        // once per track point
	addVar(Name_TravelSplineBonus, 0.0f);  // once per spline point
	addVar(Name_DriftBonus, 1.0f);
	addVar(Name_SpeedBonus, 5.0f);
	addVar(Name_ThrottleBonus, 5.0f);
	addVar(Name_EngineRpmBonus, 5.0f);
	addVar(Name_DirectionBonus, 0.0f);

	addVar(Name_DirectionPenalty, 0.0f);
	addVar(Name_ObstApproachPenalty, 2.0f);   // [建议] 安全项>0才会生效
	addVar(Name_CollisionPenalty, 1.0f);      // [建议]
	addVar(Name_OffTrackPenalty, 1.0f);      // [建议]
	addVar(Name_GearGrindPenalty, 0.5f);      // [建议]
	addVar(Name_StallPenalty, 0.5f);          // [建议]

	//===================== 新增：代价函数相关权重与阈值 =====================
	// 轨迹跟踪
	addVar(Name_wPosErr,        2.0f);
	addVar(Name_wYawErr,        0.5f);

	// β监管（角度单位：度）
	addVar(Name_wBetaBand,      0.2f);
	addVar(Name_wBetaBar,       25.0f);
	addVar(Name_betaLoDeg,      4.5f);
	addVar(Name_betaHiDeg,      5.5f);
	addVar(Name_betaSafeDeg,    6.0f);

	// 控制平滑
	addVar(Name_wDeltaSteer,    200.0f);
	addVar(Name_wJerkSteer,     0.08f);
	addVar(Name_wDeltaLong,     15.0f);

	addVar(Name_wSteerCenter, 10.0f);
	addVar(Name_wSteerFlip, 3.0f);   // 可从 3~10f 试	
	// 进度（沿赛道方向的前进作为负代价——鼓励）
	addVar(Name_wProgress,  0.1f);

	// =====================（旧）漂移阈值（保留但不再用 slipRatio 判别）=====================
	addVar("SkidEntryNd",        0.50f);  // 旧：进入“轻微漂移”的 ndSlip 下限
	addVar("SkidSevereNd",       0.80f);  // 旧：严重漂移的 ndSlip 下限
	addVar("SlipRatioSevere",    0.90f);  // 旧：严重 slipRatio 阈值（不再用于判漂，仅保留兼容）
	addVar("GripModMin",         0.90f);
	addVar("SpeedMinKmh",        1.00f);
	addVar("WheelAngSpeedLow",   8.00f);

	// ===================== 新：基于 slipAngle 的漂移参数 =====================
	addVar("SkidNdGate",           0.50f); // 仅当 NdSlip≥此值才进入角度分档
	addVar("SlipAngleMinorRad",    0.1f);  // >0.0 → 轻微
	addVar("SlipAngleModerateRad", 0.2f);  // >0.2 → 中等
	addVar("SlipAngleSevereRad",   0.3f);  // >0.3 → 严重

	addVar("wDriftMinor",          5.0f);  // 三档惩罚权重（逐轮计数）
	addVar("wDriftModerate",       50.0f);
	addVar("wDriftSevere",         500.0f);

	addVar("DriftExtremeMul",      2.0f);  // 极限漂移放大倍数
	addVar("DriftStopSeconds",     1.0f);  // 退出计时阈值

	addVar("CrossPenalty",       1.0f);  // 单次穿越扣分（一次性），可按总体代价量级微调
	addVar("CrossDeadbandM",      0.30f); // e_y 去抖死区（米）：|e_y| < deadband 视为 0，防止零点抖动
	addVar("CrossMinSpeedKmh",   25.0f);  // 仅当车速 > 此门槛才认定为有效穿越（低速微调不扣）
	addVar("CrossYawGateRad",     0.05f); // 航向误差门槛（弧度≈2.9°），几乎平行赛道时不扣
	addVar("CrossCooldownSec",    0.70f); // 冷却时间（秒）：触发一次后在冷却期内不重复扣
}

void ScoringConfig::removeVars()
{
	std::vector<ScoringVar*> tmp;
	std::swap(tmp, vvars);
	mvars.clear();

	for (auto* var : tmp)
		delete var;
}

void ScoringConfig::addVar(const std::string& name, float value)
{
	auto* var = new ScoringVar{name, value};
	vvars.push_back(var);
	mvars[name] = var;
}

void ScoringConfig::setVar(const std::string& name, float value)
{
	mvars[name]->value = value;
}

float ScoringConfig::getVar(const std::string& name) const
{
	auto iter = mvars.find(name);
	if (iter != mvars.end())
		return iter->second->value;
	return 0;
}

//=================================================================================================

ScoringSystem::ScoringSystem() {}
ScoringSystem::~ScoringSystem() {}

void ScoringSystem::init(struct Car* _car)
{
	car = _car;
	config = ScoringConfig::get();
}

void ScoringSystem::step(float dt)
{
	// 仍然以“代价”为主；漂移部分只做惩罚
	crossCooldownLeft = std::max(0.0f, crossCooldownLeft - dt);
	scoreDriftPenalty(dt);
	computeAgentReward(dt);
}

void ScoringSystem::reset()
{
	prevEpisodeReward = totalReward;
	totalReward = 0;
	stepReward = 0;
	C_track = 0;
	C_speed = 0;
	C_smooth = 0;
	oldPointId = 0;
	oldSplinePointId = 0;
	crossPrevEySign = 0.0f;
	crossCooldownLeft = 0.0f;

	m_crossCount = 0;        // 本episode累计有效穿越次数
	m_crossCooldown = 0.0f;  // 冷却计时器（秒）
	m_prevSide = 0;          // 上一次“有效侧”：-1=左, +1=右, 0=未知
	m_prevSteer = 0.0f;
	m_prevLong  = 0.0f;
	m_prevDSteer = 0.0f;
	m_prevDLong  = 0.0f;
	m_prevSteerSign = 0;
}

// 小工具函数
static inline float sqr(float x) { return x * x; }
static inline float rad2deg(float r) { return r * (180.0f / 3.14159265358979323846f); }
static inline float clamp01(float x) { return x < 0 ? 0 : (x > 1 ? 1 : x); }
static thread_local float g_lastFramePenalty = 0.0f;
static inline float clampf(float x, float lo, float hi) {
    return (x < lo) ? lo : (x > hi ? hi : x);
}
static inline float sat01(float x) { return clampf(x, 0.0f, 1.0f); }

static inline float linscale01(float x, float a, float b) {
    return sat01((x - a) / std::fmax(1e-6f, (b - a)));
}

static inline float deg2rad(float deg) {
    return deg * 0.01745329251994329577f; // pi/180
}

static inline float sat_quad01(float x) {
    x = std::fabs(x);
    return sat01(x * x);
}

// ---- smooth 0..1 mapping: x<=soft -> 0, x>=hard -> 1, smoothstep in between ----
static inline float soft01_abs(float x, float soft, float hard) {
    x = std::fabs(x);
    if (x <= soft) return 0.0f;
    if (x >= hard) return 1.0f;
    float t = (x - soft) / std::fmax(1e-6f, (hard - soft));
    t = sat01(t);
    // smoothstep
    return t * t * (3.0f - 2.0f * t);
}

// ---- safe acos: argument must be in [0,1] or [-1,1] ----
static inline float acos_safe(float x) {
    x = clampf(x, -1.0f, 1.0f);
    return std::acos(x);
}

static inline float cost_drift_soft(int driftNow) {
    // driftNow: 0..8
    if (driftNow <= 1) return 0.0f;
    if (driftNow <= 2) return 0.25f;    
	if (driftNow <= 3) return 0.5f;    
	if (driftNow <= 4) return 0.75f;   
    return 1.0f;                      
}

static inline float smoothstep01(float t) {
    t = sat01(t);
    return t * t * (3.0f - 2.0f * t);  // 先缓后急，且单调
}

// “不同光滑度”：用幂指数调形状（p>1 更“慢起快收”，p<1 更“快起慢收”）
static inline float smooth_pow(float t, float p) {
    float s = smoothstep01(t);
    return std::pow(s, p);
}

// 四段随 |e_y| 平滑切换全局权重：track / speed / smooth  权重1
static inline void global_weights_by_ey_4zone(
    float abs_ey_m, float halfW_m,
    float& W_track, float& W_speed, float& W_smooth)
{
    const float z0 = 0.5f;
    const float z1 = 1.4f;
    const float z2 = 4.2f;
    float z3 = std::fmax(halfW_m, z2 + 0.1f);

    auto lerp = [](float a, float b, float t){ return a + (b - a) * t; };

    // 四个“状态”的原型（每组和=1）
    // 0~0.5：极近，允许更追求速度（贴线时不必过度降速），smooth 很小
    const float T0 = 0.55f, S0 = 0.45f, M0 = 0.00f;

    // 0.5~1.4：近，略偏 track（你原来 0.6/0.4）
    const float T1 = 0.575f, S1 = 0.425f, M1 = 0.00f;

    // 1.4~4.2：中，进一步偏 track（开始“回线”优先）
    const float T2 = 0.65f, S2 = 0.3f, M2 = 0.05f;

    // 4.2~z3：远，强烈偏 track，并抬一点 smooth 防止猛打方向/猛刹回线
    const float T3 = 0.72f, S3 = 0.18f, M3 = 0.1f;

    // 三段平滑过渡：V->N, N->M, M->F
    float t01 = (abs_ey_m - 0.0f) / std::fmax(1e-6f, (z0 - 0.0f));
    float t12 = (abs_ey_m - z0)   / std::fmax(1e-6f, (z1 - z0));
    float t23 = (abs_ey_m - z2)   / std::fmax(1e-6f, (z3 - z2));

    t01 = smooth_pow(t01, 1.8f);
    t12 = smooth_pow(t12, 1.6f);
    t23 = smooth_pow(t23, 1.2f);

    // 串联 lerp：(((0->1)->2)->3)
    float T01 = lerp(T0, T1, t01);
    float S01 = lerp(S0, S1, t01);
    float M01 = lerp(M0, M1, t01);

    float T012 = lerp(T01, T2, t12);
    float S012 = lerp(S01, S2, t12);
    float M012 = lerp(M01, M2, t12);

    W_track  = lerp(T012, T3, t23);
    W_speed  = lerp(S012, S3, t23);
    W_smooth = lerp(M012, M3, t23);

    // 防御：归一化，保证和=1
    float sum = std::fmax(1e-6f, W_track + W_speed + W_smooth);
    W_track  /= sum;
    W_speed  /= sum;
    W_smooth /= sum;
}

// 四段 cost_y（单位：米；单调、连续、到边界饱和为 1）
static inline float cost_y_4zone_linear(float abs_ey_m, float halfW_m) {
    const float z0 = 0.5f;   // 新增：极近区上界
    const float z1 = 1.4f;
    const float z2 = 4.2f;
    float z3 = std::fmax(halfW_m, z2 + 0.1f);

    // 权重总和=1（把原来的 w1=0.45 拆成两段）
    const float w0 = 0.35f; // 0   -> 0.5
    const float w1 = 0.3f; // 0.5 -> 1.4
    const float w2 = 0.25f;  // 1.4 -> 4.2
    const float w3 = 0.10f;  // 4.2 -> z3

    // linear normalized t in each zone, clamped to [0,1]
    float t0 = (abs_ey_m - 0.0f) / std::fmax(1e-6f, (z0 - 0.0f));
    float t1 = (abs_ey_m - z0)   / std::fmax(1e-6f, (z1 - z0));
    float t2 = (abs_ey_m - z1)   / std::fmax(1e-6f, (z2 - z1));
    float t3 = (abs_ey_m - z2)   / std::fmax(1e-6f, (z3 - z2));

    // chained saturations: each term contributes only in/after its zone, and saturates at 1
    float cost = w0 * sat01(t0)
               + w1 * sat01(t1)
               + w2 * sat01(t2)
               + w3 * sat01(t3);

    return sat01(cost);
}

// 随 |e_y| 平滑切换权重（近->中->远）
// 随 |e_y| 平滑切换权重（极近->近->中->远）  权重1
static inline void weights_by_ey_4zone(
    float abs_ey_m, float halfW_m,
    float& a_y, float& a_psi, float& a_cross, float& a_bound, float& a_drift)
{
    // 四段边界
    const float z0 = 0.5f;   // 新增：极近区上界
    const float z1 = 1.4f;   // 近区上界
    const float z2 = 4.2f;   // 中区上界
    float z3 = std::fmax(halfW_m, z2 + 0.1f); // 远区上界（防御：确保 z3>z2）

    auto saturate = [](float x){ return (x < 0.f) ? 0.f : (x > 1.f ? 1.f : x); };
    auto lerp = [](float a, float b, float t){ return a + (b - a) * t; };

    // 四个“状态”的权重原型（每组和=1，便于调参）
    // V: 0~0.5 极近（更强调“贴线/对齐”，cross/psi 更敏感，bound 很轻）
    const float Vy=0.55f, Vpsi=0.25f, Vcross=0.1f, Vbound=0.05f, Vdrift=0.05f;

    // N: 0.5~1.4 近（你当前的 Near，可保留/微调）
    const float Ny=0.50f, Npsi=0.20f, Ncross=0.1f, Nbound=0.1f, Ndrift=0.1f;

    // M: 1.4~4.2 中（你当前的 Mid）
    const float My=0.45f, Mpsi=0.18f, Mcross=0.10f, Mbound=0.12f, Mdrift=0.15f;

    // F: 4.2~ 远（你当前的 Far）
    const float Fy=0.38f, Fpsi=0.08f, Fcross=0.02f, Fbound=0.27f, Fdrift=0.25f;

    // 三段平滑过渡参数（先做归一化到[0,1]，再 smooth_pow）
    float tVN = (abs_ey_m - 0.0f) / std::fmax(1e-6f, (z0 - 0.0f)); // 极近->近
    float tNM = (abs_ey_m - z0)   / std::fmax(1e-6f, (z1 - z0));   // 近->中（注意这里从 z0 开始）
    float tMF = (abs_ey_m - z2)   / std::fmax(1e-6f, (z3 - z2));   // 中->远

    tVN = smooth_pow(saturate(tVN), 1.8f); // 极近->近：建议更“慢起”，避免 0 附近抖动
    tNM = smooth_pow(saturate(tNM), 1.6f); // 近->中：你原来那套
    tMF = smooth_pow(saturate(tMF), 1.2f); // 中->远：稍快把 bound 拉起来

    // 串联三段 lerp：(((V->N)->M)->F)
    float ay0     = lerp(Vy,     Ny,     tVN);
    float apsi0   = lerp(Vpsi,   Npsi,   tVN);
    float across0 = lerp(Vcross, Ncross, tVN);
    float abound0 = lerp(Vbound, Nbound, tVN);
    float adrift0 = lerp(Vdrift, Ndrift, tVN);

    float ay1     = lerp(ay0,     My,     tNM);
    float apsi1   = lerp(apsi0,   Mpsi,   tNM);
    float across1 = lerp(across0, Mcross, tNM);
    float abound1 = lerp(abound0, Mbound, tNM);
    float adrift1 = lerp(adrift0, Mdrift, tNM);

    a_y     = lerp(ay1,     Fy,     tMF);
    a_psi   = lerp(apsi1,   Fpsi,   tMF);
    a_cross = lerp(across1, Fcross, tMF);
    a_bound = lerp(abound1, Fbound, tMF);
    a_drift = lerp(adrift1, Fdrift, tMF);

    // 防御：归一化一次，避免浮点误差导致和!=1
    float sum = std::fmax(1e-6f, a_y + a_psi + a_cross + a_bound + a_drift);
    a_y/=sum; a_psi/=sum; a_cross/=sum; a_bound/=sum; a_drift/=sum;
}

// ─────────────────────────────────────────────────────────
// 轮级：按 slipAngle + NdSlip Gate 进行分档
// 返回：0=无；1=轻微；2=中等；3=严重
// ─────────────────────────────────────────────────────────
static inline int tyreSlipAngleLevel(
	const Tyre* tyre, float speedKmh,
	float gripMin, float speedMinKmh, float wheelAngLow,
	float ndGate, float aMinorRad, float aModerRad, float aSevereRad)
{
	if (!tyre || !tyre->surfaceDef) return 0;
	if (tyre->surfaceDef->gripMod < gripMin) return 0;

	// 低速且轮角速也低：过滤
	if (speedKmh <= speedMinKmh && fabsf(tyre->status.angularVelocity) <= wheelAngLow)
		return 0;

	// NdSlip总开关
	const float nd = fabsf(tyre->status.ndSlip);
	if (nd < ndGate) return 0;

	const float a = fabsf(tyre->status.slipAngleRAD); // 弧度
	if (a > aSevereRad)   return 3;
	if (a > aModerRad)    return 2;
	if (a > aMinorRad)    return 1;
	return 0;
}

// 0..8（共9级）整车侧滑等级：层层递进版
// 依赖你已统计好的：cntLvl1/2/3、rearLvl2/3、maxAngleAbs（弧度）
// 以及严重阈值 aSevereRad（弧度）
static inline int vehicleSlipGrade9_layered(
    int   cntLvl1, int   cntLvl2, int   cntLvl3,
    int   rearLvl2, int  rearLvl3,
    float speedKmh,
    float maxAngleAbs,
    float aSevereRad)
{
    const int cntAny = cntLvl1 + cntLvl2 + cntLvl3;

    // 基本门槛：低速或不足两轮达档 => 0（无）
    if (speedKmh <= 5.0f || cntAny < 2)
        return 0;

    // 连续强度分：轻微=1，中等=2，严重=3
    int base = 1 * cntLvl1 + 2 * cntLvl2 + 3 * cntLvl3;

    // 后轴加权：中等 +1 / 严重 +2，合计上限 4
    int rear_bonus = rearLvl2 * 1 + rearLvl3 * 2;
    if (rear_bonus > 4) rear_bonus = 4;

    // 极端角度加分（两级）：>1.25× severe 阈 +1；>1.75× 再 +1（合计上限 2）
    int angle_bonus = 0;
    if (maxAngleAbs > 1.25f * aSevereRad) angle_bonus += 1;
    if (maxAngleAbs > 1.75f * aSevereRad) angle_bonus += 1;

    int total = base + rear_bonus + angle_bonus; // 典型范围约 2..20

    // —— 分段映射到 0..8（层层递进，可按需微调阈值）——
    int grade;
    if      (total <= 2)        grade = 1; // 刚过门槛（最弱可判定）
    else if (total == 3)        grade = 2;
    else if (total == 4)        grade = 3;
    else if (total <= 6)        grade = 4;
    else if (total <= 8)        grade = 5;
    else if (total <= 10)       grade = 6;
    else if (total <= 12)       grade = 7;
    else                        grade = 8;

    // —— 下限保障：出现严重就不低于相应档位（保证常识）——
    if (cntLvl3 >= 1 && grade < 4) grade = 4; // 单轮严重，至少4级
    if (cntLvl3 >= 2 && grade < 6) grade = 6; // 双轮严重，至少6级
    if (cntLvl3 >= 3 || (cntLvl3 >= 2 && maxAngleAbs > 1.5f * aSevereRad))
        grade = 8; // 三轮严重或“双严重+角度显著超阈”直接拉满

    // 低速限幅：<20km/h 不超过2级，避免低速抖动被过判
    if (speedKmh < 20.0f && grade > 2)
        grade = 1;

    return grade; // 0..8
}

static inline float cost_from_probeHits(
    const std::vector<float>& probeHits,
    float criticalDistance,
    float approachDistance,
    float speedKmh,             // 可用于风险随速度增强
    float speedKmhSoft = 30.0f,  // 低于此速度不增强
    float speedKmhHard = 120.0f, // 高于此速度增强到最大
    bool  useQuadratic = true    // 二次增强：更“怕贴边”
){
    if (probeHits.empty()) return 0.0f;
    if (!(approachDistance > criticalDistance && criticalDistance > 0.0f)) return 0.0f;

    float closest = FLT_MAX;
    for (float d : probeHits) {
        if (d > 0.0f && d < closest) closest = d;
    }
    if (closest == FLT_MAX) return 0.0f;

    // 未进入警戒距离 -> 0
    if (closest >= approachDistance) return 0.0f;

    // 映射：closest=approach -> 0；closest=critical -> 1；critical以内饱和为1
    float t = (closest - criticalDistance) / std::fmax(1e-6f, (approachDistance - criticalDistance));
    t = sat01(t);                 // t: 0 at critical, 1 at approach
    float x = 1.0f - t;           // x: 1 near critical, 0 near approach
    float cost = useQuadratic ? (x * x) : x;   // 0..1

    // 速度增强（仍保持 0..1）：高速更怕贴边
    float sv = sat01((speedKmh - speedKmhSoft) / std::fmax(1e-6f, (speedKmhHard - speedKmhSoft)));
    float gain = 0.5f + 0.5f * sv;            // 0.5..1.0
    cost = sat01(cost * gain);

    return cost; // 0..1
}

static inline float dist_point_segment3D(const vec3f& p, const vec3f& a, const vec3f& b)
{
    vec3f ab = b - a;
    float ab2 = ab * ab; // dot(ab,ab)  (你代码里 vec3f 的 * 已经用作 dot)
    if (ab2 < 1e-12f) return (p - a).len();

    float t = ((p - a) * ab) / ab2;
    t = clampf(t, 0.0f, 1.0f);
    vec3f proj = a + ab * t;
    return (p - proj).len();
}

static inline float cost_from_edge_distance(
    float closestEdgeDistM,
    float criticalDistanceM,   // 例如 1.0m：<=这个就直接 1
    float approachDistanceM,   // 例如 3.0m：>=这个就 0
    float speedKmh,
    float speedKmhSoft = 30.0f,
    float speedKmhHard = 120.0f,
    bool  useQuadratic = true
){
    if (!(approachDistanceM > criticalDistanceM && criticalDistanceM > 0.0f)) return 0.0f;

    // 未进入警戒距离 -> 0
    if (closestEdgeDistM >= approachDistanceM) return 0.0f;

    // 映射：dist=approach -> 0；dist=critical -> 1；critical以内饱和为1
    float t = (closestEdgeDistM - criticalDistanceM) / std::fmax(1e-6f, (approachDistanceM - criticalDistanceM));
    t = sat01(t);                 // 0 at critical, 1 at approach
    float x = 1.0f - t;           // 1 near critical, 0 near approach
    float cost = useQuadratic ? (x * x) : x;   // 0..1

    // 速度增强（高速更怕贴边）
    float sv = sat01((speedKmh - speedKmhSoft) / std::fmax(1e-6f, (speedKmhHard - speedKmhSoft)));
    float gain = 0.5f + 0.5f * sv;            // 0.5..1.0
    return sat01(cost * gain);
}

// 漂移惩罚评分（正数 = 惩罚；返回值已乘 dt）
float ScoringSystem::scoreDriftPenalty(float dt)
{
	validateDrift();
	g_lastFramePenalty = 0.0f;
	const float fSpeedKmh   = car->speed.kmh();
	const float gripMin     = getVar("GripModMin");
	const float speedMinKmh = getVar("SpeedMinKmh");
	const float wheelAngLow = getVar("WheelAngSpeedLow");

	// 新：角度 + NdGate 阈值
	const float ndGate      = getVar("SkidNdGate");
	const float aMinorRad   = getVar("SlipAngleMinorRad");
	const float aModerRad   = getVar("SlipAngleModerateRad");
	const float aSevereRad  = getVar("SlipAngleSevereRad");

	// 三档权重与极限系数/退出时间
	const float wMinor      = getVar("wDriftMinor");
	const float wModerate   = getVar("wDriftModerate");
	const float wSevere     = getVar("wDriftSevere");
	const float extremeMul  = (getVar("DriftExtremeMul") > 0.0f ? getVar("DriftExtremeMul") : 2.0f);
	const float stopSec     = (getVar("DriftStopSeconds") > 0.0f ? getVar("DriftStopSeconds") : 1.0f);

	// 速度倍率（沿用原思路）
	float fSpeedMult = (fSpeedKmh - 20.0f) * 0.015384615f;   // (kmh-20)/65
	fSpeedMult = tclamp(fSpeedMult, 0.0f, 2.0f);
	currentSpeedMultiplier = fSpeedMult;

    // === 统计四轮分档 + 记录后轴档位计数 ===
	int cntLvl1 = 0, cntLvl2 = 0, cntLvl3 = 0;
	int rearLvl2 = 0, rearLvl3 = 0;
	float maxAngleAbs = 0.0f;

	for (int i = 0; i < 4; ++i) {
		const Tyre* t = car->tyres[i].get();
		const int lvl = tyreSlipAngleLevel(
			t, fSpeedKmh, gripMin, speedMinKmh, wheelAngLow,
			ndGate, aMinorRad, aModerRad, aSevereRad
		);
		if      (lvl == 1) ++cntLvl1;
		else if (lvl == 2) { ++cntLvl2; if (i >= 2) ++rearLvl2; }
		else if (lvl == 3) { ++cntLvl3; if (i >= 2) ++rearLvl3; }

		if (t) maxAngleAbs = std::max(maxAngleAbs, fabsf(t->status.slipAngleRAD));
	}

    // === 用 0~4 级整车分级直接赋值 driftNow ===
	this->driftNow = vehicleSlipGrade9_layered(
		cntLvl1, cntLvl2, cntLvl3,
		rearLvl2, rearLvl3,
		fSpeedKmh,
		maxAngleAbs,
		aSevereRad
	);

	// 漂移状态与极限
	const int  cntAny        = cntLvl1 + cntLvl2 + cntLvl3;
	const bool driftableNow  = (cntAny >= 2);
	
	driftExtreme             = (cntLvl3 >= 2);
	currentDriftAngle        = maxAngleAbs; // 弧度（便于UI转换角度显示）

	float framePenalty = 0.0f;
	
	if (driftableNow)
	{	
		auto v = car->body->getLocalVelocity();

		if (!drifting) {
			lastDriftDirection = (v.x >= 0.0f) ? 1.0f : -1.0f;
			driftComboCounter  = 1;
			driftInvalid       = false;
			instantDrift       = 0.0f;
			drifting           = true;
		}

		// 三档逐轮计费（互不包含）
		float perFrame = wMinor * cntLvl1 + wModerate * cntLvl2 + wSevere * cntLvl3;
		perFrame *= fSpeedMult;
		if (driftExtreme) perFrame *= extremeMul;

		framePenalty = perFrame * dt;

		// 连击/换向（保持原有语义：速度足够且达到重档时记一次）
		if (fabsf(v.x) > 4.0f) {
			const float fDir = (v.x >= 0.0f) ? 1.0f : -1.0f;
			if (lastDriftDirection != fDir && cntLvl3 >= 2) {
				instantDrift      += 50.0f; // 遥测
				driftComboCounter += 1;
				lastDriftDirection = fDir;
			}
		}

		driftStraightTimer = 0.0f;
	}
	else if (drifting)
	{
		// 原语义：>20km/h 且直行累计 > stopSec 才退出
		if (fSpeedKmh > 20.0f) driftStraightTimer += dt;
		else                   driftStraightTimer  = 0.0f;

		if (driftInvalid) {
			resetDrift();
			return framePenalty;
		}

		if (driftStraightTimer > stopSec) {
			driftComboCounter = 0;
			driftPoints      += instantDrift; // 遥测
			drifting          = false;
			instantDrift      = 0.0f;
		}
	}

	if (driftInvalid) resetDrift();
	g_lastFramePenalty = framePenalty;
    return framePenalty;
}

void ScoringSystem::computeAgentReward(float dt)
{
	//===================== reward=====================
	float reward = 0.0f; // [NEW] 聚合所有要最小化的项

	//===================== 控制器参数（仍保留） =====================
	car->smoothSteerSpeed = getVar(Name_SmoothSteerSpeed);

	//===================== 状态辅助量 =====================
	const float curRpm = car->getEngineRpm();
	const float maxRpm = (float)car->drivetrain->engineModel->getLimiterRPM();

	// //===================== 进度（沿赛道推进） —— 负代价 =====================
	// {
	// 	const float w = getVar(Name_wProgress);
	// 	if (w > 0.0f) {
	// 		const float v_along = tmax(0.0f, car->velocityVsTrack) * car->speed.ms();
	// 		reward += w * v_along; // 作为负代价（鼓励向前）
	// 	}
	// }

	//===================== 轨迹跟踪（横向 & 航向） =====================

	const int trackPointId = car->nearestTrackPointId;
	if (trackPointId >= 0 && trackPointId < (int)car->track->fatPoints.size())
	{
		// —— 带符号横向误差：右正、左负 ——
		// 依赖类型：vec3f，且支持：加减、与标量相乘、点积（*），以及 get_norm()/len()

		const auto& pt  = car->track->fatPoints[trackPointId];
		const vec3f pos = car->body->getPosition(0.0f);

		// 车到参考点向量
		const vec3f d = pos - pt.best;

		// 赛道切线（单位向量）
		vec3f fwd = pt.forwardDir.get_norm();

		// 先按你原来的方式求“横向距离的绝对值”
		const float tang    = d * fwd;         // dot(d, fwd)
		const vec3f d_perp  = d - fwd * tang;  // 去掉切向分量
		const float e_y_abs = d_perp.len();    // m（无符号）

		// 选“向上”方向：优先赛道法线；若没有就用世界 up
		vec3f up = vec3f(0, 1, 0);
		// 如果 fatPoint 有上向/法线，请取消下面一行注释（按你的字段名改动）
		// up = pt.upDir.get_norm();  // 或 pt.normalDir.get_norm();

		// 赛道右向（右手系）：right = up × fwd
		// —— 用分量写法，避免依赖外部 cross()
		vec3f right(
			fwd.y * up.z - fwd.z * up.y,
			fwd.z * up.x - fwd.x * up.z,
			fwd.x * up.y - fwd.y * up.x
		);
		// …下面保持不变：
		float rightLen2 = right.x*right.x + right.y*right.y + right.z*right.z;
		if (rightLen2 > 1e-12f) {
			float invLen = 1.0f / std::sqrt(rightLen2);
			right.x *= invLen; right.y *= invLen; right.z *= invLen;
		} else {
			right = vec3f(1,0,0);
		}
		const float side = d * right;                 // >0 右侧，<0 左侧
		const float e_y  = (side >= 0.0f) ? (+e_y_abs) : (-e_y_abs);  // 右正左负

		// -------------------- Normalized cost_y: lateral error (0..1) --------------------
		// 推荐：用赛道宽度自适应归一化（换赛道不崩）
		// halfW = 0.5*trackWidth, ey_ratio = |e_y|/halfW

		const float EySoftRatio = 0.02f;  // 2% 半赛道宽以内 -> 0 惩罚（例如 halfW=5m => 0.1m）
		const float EyHardRatio = 0.25f;  // 25% 半赛道宽以上 -> 1 惩罚（halfW=5m => 1.25m）
		
		auto& fatPoints = car->track->fatPoints;
		int N = (int)fatPoints.size();
		if (N < 2) {
			// 赛道点不足，无法形成线段，直接返回或跳过本段惩罚
			// cost_bound = 0; 或者 return;
		} else {
			int id0 = trackPointId;
			if (id0 < 0) id0 = 0;
			if (id0 >= N) id0 = N - 1;
			

			int id1 = (id0 > 0) ? (id0 - 1) : (N - 1);  // ✅ id0==0 时回绕到最后一个点

			const auto& fat0 = fatPoints[id0];
			const auto& fat1 = fatPoints[id1];
			vec3f dir0 = fat0.forwardDir;
			vec3f dir1 = fat1.forwardDir;

			float dL = dist_point_segment3D(pos, fat0.left,  fat1.left);
			float dR = dist_point_segment3D(pos, fat0.right, fat1.right);
			trackW =(fat0.left-fat0.right).len();
		}
		
		this->trackW = trackW;
		float halfW  = 0.5f * trackW;
		halfW = std::fmax(halfW, 1.0f);   // 防御：避免极端情况下除零/过小（1m 作为下限）

		float abs_ey_m = std::fabs(e_y);       // 直接用“米”做三段
		float cost_y   = cost_y_4zone_linear(abs_ey_m, halfW);

		// -------------------- Normalized cost_psi: heading error (0..1) --------------------
		// 先从 bodyVsTrack 得到航向误差 e_psi（弧度），再做软阈值归一化。
		// e_psi = acos(|bodyVsTrack|) -> [0, pi/2]，越大越偏航

		const float PsiSoftRad = deg2rad(2.0f);   // 2° 内 -> 0 惩罚
		const float PsiHardRad = deg2rad(20.0f);  // 20° 以上 -> 1 惩罚

		float bvt_abs = std::fabs(car->bodyVsTrack);  // 0..1（理想情况下）
		bvt_abs = clampf(bvt_abs, 0.0f, 1.0f);   // 防御数值误差
		float e_psi  = acos_safe(bvt_abs);       // rad
		float cost_psi = soft01_abs(e_psi, PsiSoftRad, PsiHardRad); // 0..1

			// ================== cost_cross (0..1) ==================
		// 可调参数（建议先用这些默认值）
		const float CrossCooldownSec   = 0.10f;          // 两次穿越之间最小间隔
		const float CrossMinSpeedKmh   = 20.0f;          // 低速穿越不算（避免起步抖动）
		const float CrossYawGateRad    = deg2rad(8.0f);  // 航向较对准时才认为是“蛇形穿越”
		const int   CrossRefCount      = 3;              // >=2 次就认为“反复穿越”趋向 1.0

		// 侧判定死区（避免 e_y 在 0 附近抖动导致误判）
		// 推荐与赛道半宽相关；如果你暂时拿不到 trackWidth，可用固定值 0.05~0.15m
		halfW = std::fmax(halfW, 1.0f);                 // 防御：下限 1m
		float EyDeadband = 0.02f * halfW;               // 2%半宽作为死区（halfW=5m -> 0.1m）

		// 速度、航向
		float speedKmh = car->speed.kmh();
		// 冷却更新
		m_crossCooldown = std::fmax(0.0f, m_crossCooldown - dt);

		// 当前侧（带死区）：-1 左，+1 右，0 未定义
		int currentside = 0;
		if (e_y >  EyDeadband) currentside = +1;
		if (e_y < -EyDeadband) currentside = -1;

		// 如果在死区内，保持上一侧（避免在 0 附近反复翻转）
		if (currentside == 0) currentside = m_prevSide;

		// 判断“是否换侧”
		bool sideChanged = (m_prevSide != 0) && (currentside != 0) && (currentside != m_prevSide);

		// gate：只在“对准（e_psi小）+ 有一定速度 + 冷却结束”时认定为有效穿越
		bool passGate = (e_psi < CrossYawGateRad) && (speedKmh > CrossMinSpeedKmh);

		// 穿越事件（本帧 0/1）
		int crossEvent = 0;
		if (sideChanged && passGate && (m_crossCooldown <= 0.0f)) {
			crossEvent = 1;
			m_crossCount += 1;
			m_crossCooldown = CrossCooldownSec;
		}

		// 更新 prevSide（只要 side 有定义）
		if (currentside != 0) m_prevSide = currentside;

		// 归一化 cost_cross：累计次数 -> [0,1]
		// 0 次 => 0；达到 CrossRefCount 次 => 1；超过也保持 1
		float cost_cross = sat01(float(m_crossCount) / float(std::max(1, CrossRefCount)));

		// （可选）如果你更希望“当帧惩罚”为 0/1，而不是累计：
		// float cost_cross = float(crossEvent);  // 0 or 1

		//边界惩罚
		float criticalDistance = getVar(Name_CriticalDistance);
		float approachDistance = getVar(Name_ApproachDistance);

		// 0..1：越贴边越大
		float cost_bound = 0.0f;

		if (N < 2) {
			// 赛道点不足，无法形成线段，直接返回或跳过本段惩罚
			// cost_bound = 0; 或者 return;
		} else {
			int id0 = trackPointId;
			if (id0 < 0) id0 = 0;
			if (id0 >= N) id0 = N - 1;

			int id1 = (id0 > 0) ? (id0 - 1) : (N - 1);  // ✅ id0==0 时回绕到最后一个点

			const auto& fat0 = fatPoints[id0];
			const auto& fat1 = fatPoints[id1];

			float dL = dist_point_segment3D(pos, fat0.left,  fat1.left);
			float dR = dist_point_segment3D(pos, fat0.right, fat1.right);
			float closest = std::fmin(dL, dR);

			// === out-of-track heuristic flag ===
			constexpr float kSumLRThreshold   = 16.0f;
			constexpr float kClosestThreshold = 1.5f;
			
			int p_cost_bound = 1;
			discarandbound =(car->body->getPosition(0) - pt.center).len();
			if (discarandbound > trackW * getVar(Name_OutOfTrackThreshold)) // EPIC FAIL!!!
			{
				car->outOfTrackFlag = true;
				p_cost_bound = 2;
			}

			cost_bound = cost_from_edge_distance(closest, criticalDistance, approachDistance, speedKmh)*p_cost_bound;
		}


		//侧滑惩罚
		int driftNow = this->driftNow;  // 或从 state 中取
		bool driftHardFail = (driftNow >= 3);  // >=3 直接抛弃（交给优化器剔除）
		float cost_drift = driftHardFail ? 1.0f : cost_drift_soft(driftNow); // 0..1（便于调试/统计）

		// ---- Track组内权重（可调，和为1不是必须，后面会归一化）----
		float a_y, a_psi, a_cross, a_bound, a_drift;
		weights_by_ey_4zone(std::fabs(e_y), halfW, a_y, a_psi, a_cross, a_bound, a_drift);
			
		// float C_track =
		// 	(a_y*cost_y + a_psi*cost_psi + a_cross*cost_cross + a_bound*cost_bound + a_drift*cost_drift) /
		// 	std::fmax(1e-6f, (a_y+a_psi+a_cross+a_bound+a_drift));  // 0..1   细优化
		float C_track =
			(1*cost_y + 0*cost_psi + 0*cost_cross + 0*cost_bound + 0*cost_drift) /
			std::fmax(1e-6f, (1));  // 0..1  粗优化

		this->e_y   = e_y;         // 若你想保留原始量，也可另存 norm
		this->e_psi = e_psi;
		this->cost_y = cost_y;
		this->cost_psi = cost_psi;
		this->cost_cross = cost_cross;
		this->cost_bound = cost_bound;
		this->cost_drift = cost_drift;
		this->trackW = trackW;
		this->C_track = C_track;
		this->discarandbound =discarandbound;
	}
	

	// ================== Speed group: C_speed (0..1) ==================
	// 目标：车在可行域内尽量有“前向进度”，但仍然是第二优先级（权重较 track 小）

	// ---- 可调参数（按你的车/赛道再调）----
	const float VSoft = 15.0f;     // m/s：低于此速度，bonus 接近 0（约 54 km/h）
	const float VHard = 60.0f;    // m/s：达到此速度，bonus 饱和到 1（约 216 km/h）

	// 可选：倒车惩罚（0..1），倒车越快惩罚越大
	const float VBackSoft = 1.0f; // m/s：轻微后退不太管
	const float VBackHard = 6.0f; // m/s：明显后退认为很糟

	// ---- 1) 计算沿赛道方向速度 v_along_ms ----
	// 方案 A：如果你有 vAlongTrack（很多模拟会直接给 m/s）
	// float v_along_ms = carState.vAlongTrack;

	// 方案 B：如果你有 speedMS 和 velocityVsTrack（cosine in [-1,1]）
	// v_along_ms = speedMS * velocityVsTrack

	// 方案 C（弱备选）：没有 velocityVsTrack 时，可用 bodyVsTrack 近似（不推荐长期用）
	// v_along_ms = speedMS * bodyVsTrack

	float speedMS = car->speed.ms()/* 你已有的 speedMS 变量 */;
	float v_along_ms = 0.0f;

	// ---- 请选择你工程里实际可用的字段：----
	#if 1
	// B：推荐（速度大小 * 速度方向与赛道切向夹角 cos）
	float velocityVsTrack = car->velocityVsTrack/* 你工程里的 velocityVsTrack，范围通常 [-1,1] */;
	v_along_ms = speedMS * velocityVsTrack;
	#else
	// A：直接使用 vAlongTrack（m/s）
	// v_along_ms = /* vAlongTrack */;
	#endif

	// ---- 2) speed bonus 归一化到 0..1 ----
	// 只奖励“前向进度”，倒车不加分
	float v_fwd = std::fmax(0.0f, v_along_ms);
	float bonus_speed = linscale01(v_fwd, VSoft, VHard);   // 0..1

	// ---- 3) 可选：倒车惩罚（也归一化到 0..1） ----
	float back_speed = std::fmax(0.0f, -v_along_ms);       // 倒车速度（正数）
	float cost_back = linscale01(back_speed, VBackSoft, VBackHard); // 0..1

	// ---- 4) Speed 组 cost：仍保持 0..1 ----
	// 解释：
	// - 前向越快：bonus_speed 越大 => C_speed 越小
	// - 倒车越快：cost_back 越大 => C_speed 越大
	//
	// 你可以把倒车惩罚权重设得小一点（通常就 0.2~0.4），避免它压过 track 组
	const float b_fwd  = 0.8f;  // 前向速度项
	const float b_back = 0.2f;  // 倒车惩罚项

	float C_speed = (b_fwd * (1.0f - bonus_speed) + b_back * cost_back) /
					std::fmax(1e-6f, (b_fwd + b_back));   // 0..1

	this->C_speed = C_speed;				
	// ================== Smooth group: C_smooth (0..1) ==================
	// 输入：steer, gas, brake, speedKmh, e_y(米), e_psi(rad), dt
	// 依赖：m_prevSteer, m_prevLong, m_prevDSteer, m_prevDLong, m_prevSteerSign

	// ---- 参考尺度（针对 dt=0.003 的推荐起点）----
	// 解释：这些是“每一帧允许变化的参考幅度”；超过则 cost 接近 1
	const float DSteerRef  = 0.03f;  // 每帧方向变化参考（0.03/step，333Hz 下较严格）
	const float DDSteerRef = 0.02f;  // 二阶差分参考（抑制高频抖动/拐点抖）
	const float DLongRef   = 0.05f;  // 每帧纵向合成命令变化参考（gas-brake）

	// flip 判定参数
	const float FlipSpeedKmh = 30.0f;
	const float FlipSteerThr = 0.03f;

	// center 判定（你现在文件里的条件思路）
	const float AlignEyAbs    = 0.30f;              // m
	const float AlignPsiAbs   = deg2rad(3.0f);      // rad
	const float CenterSteerRef = 0.15f;             // 对准时 steer 超过 0.15 认为“明显乱动”
	const float CenterSteerRef2 = CenterSteerRef * CenterSteerRef;

	// ---- 当前纵向合成指令 ----
	float longCmd = clampf(car->controls.gas, 0.0f, 1.0f) - clampf(car->controls.brake, 0.0f, 1.0f); // [-1,1]

	// ---- 一阶差分 ----
	float dSteer = car->controls.steer - m_prevSteer;   // [-2,2]
	float dLong  = longCmd - m_prevLong;  // [-2,2]

	// ---- 二阶差分（推荐用 dd，不除 dt，数值稳定，避免 dt 很小导致爆炸）----
	float ddSteer = dSteer - m_prevDSteer;
	float ddLong  = dLong  - m_prevDLong;

	// ---- 归一化子项（0..1）----
	float c_dsteer  = sat_quad01(dSteer / DSteerRef);
	float c_ddsteer = sat_quad01(ddSteer / DDSteerRef);
	float c_dlong   = sat_quad01(dLong  / DLongRef);

	// （可选）你也可以对 ddLong 增加一项（如果你觉得纵向也有高频抖动问题）
	// const float DDLongRef = 0.03f;
	// float c_ddlong = sat_quad01(ddLong / DDLongRef);

	// ---- flip 子项：高速反打事件（0/1）----
	int signNow = 0;
	if (car->controls.steer >  FlipSteerThr) signNow = +1;
	if (car->controls.steer < -FlipSteerThr) signNow = -1;

	bool flipTriggered = false;
	if (car->speed.kmh() > FlipSpeedKmh) {
		if (m_prevSteerSign != 0 && signNow != 0 && signNow != m_prevSteerSign) {
			flipTriggered = true;
		}
	}
	float c_flip = flipTriggered ? 1.0f : 0.0f;

	// ---- center 子项：对准时抑制方向盘微抖（0..1）----
	bool aligned = (std::fabs(e_y) < AlignEyAbs) && (std::fabs(e_psi) < AlignPsiAbs);
	float c_center = 0.0f;
	if (aligned) {
		float s2 = car->controls.steer * car->controls.steer; // 0..1
		c_center = sat01(s2 / std::fmax(1e-6f, CenterSteerRef2));
	}

	// ---- Smooth 组内加权合成（仍保持 0..1）----
	// 建议比例：一阶变化 + 二阶差分 + 纵向变化为主，flip/center 为辅
	const float b_dsteer  = 0.30f;
	const float b_ddsteer = 0.25f;
	const float b_dlong   = 0.25f;
	const float b_flip    = 0.10f;
	const float b_center  = 0.10f;

	float C_smooth =
		(b_dsteer * c_dsteer +
		b_ddsteer* c_ddsteer +
		b_dlong  * c_dlong +
		b_flip   * c_flip +
		b_center * c_center) /
		std::fmax(1e-6f, (b_dsteer + b_ddsteer + b_dlong + b_flip + b_center)); // 0..1

	this->C_smooth = C_smooth;	

	// ---- 更新历史状态（放在 computeAgentReward 末尾或本段末尾均可）----
	m_prevSteer  = car->controls.steer;
	m_prevLong   = longCmd;
	m_prevDSteer = dSteer;
	m_prevDLong  = dLong;
	if (signNow != 0) m_prevSteerSign = signNow;

	float W_track = 0.6f;
	float W_speed = 0.4f;
	float W_smooth = 0.0f;
	float halfW  = 0.5f * trackW;
	halfW = std::fmax(halfW, 1.0f);
	global_weights_by_ey_4zone(std::fabs(e_y), halfW, W_track, W_speed, W_smooth);

	// float C_total = (W_track*C_track + W_speed*C_speed + W_smooth*C_smooth) /
	// 				std::fmax(1e-6f, (W_track + W_speed + W_smooth));  // 0..1 细优化
	float C_total = (0.5*C_track + 0.5*C_speed) /
					std::fmax(1e-6f, (0.5 + 0.5));  // 0..1 粗优化

	//===================== 汇总输出 =====================
	float stepReward = -C_total;   // [-1,0]，越接近0越好
	totalReward += stepReward;     // 或者乘 dt 做积分：totalReward += stepReward*dt;
}

// ===================== 漂移状态管理 =====================

void ScoringSystem::resetDrift()
{
	currentDriftAngle = 0.0;
	currentSpeedMultiplier = 0.0;
	driftExtreme = false;
	drifting = false;
	instantDrift = 0.0;
	driftComboCounter = 0;
}

inline bool isDirty(Tyre* tyre)
{
	auto* surf = tyre->surfaceDef;
	return (surf && surf->dirtAdditiveK > 0.001f);
}

void ScoringSystem::validateDrift()
{
	bool bInvalid = true;

	int nDirtyTyres = 0;
	for (int i = 0; i < 4; ++i)
		nDirtyTyres += (int)isDirty(car->tyres[i].get());

	if (nDirtyTyres <= 2)
	{
		if (car->speed.kmh() >= 20.0f)
		{
			bool bDamage = false;
			for (int i = 0; i < 5; ++i)
			{
				if (fabsf(car->damageZoneLevel[i] - car->oldDamageZoneLevel[i]) > 0.001f)
				{
					bDamage = true;
					break;
				}
			}

			if (!bDamage && car->drivetrain->currentGear)
			{
				bInvalid = false;
			}
		}
	}

	if (bInvalid)
		driftInvalid = true;
}

// ===================== 极限漂移（角度版） =====================

inline bool isExtremeDriftByAngle(const Tyre* tyre, float aSevereRad)
{
	if (!tyre || !tyre->surfaceDef) return false;
	return ( fabsf(tyre->status.angularVelocity) > 4.0f
	      && fabsf(tyre->status.slipAngleRAD)      > aSevereRad
	      && tyre->status.load                  > 10.0f
	      && tyre->surfaceDef->gripMod          >= 0.9f );
}

// 兼容旧接口：参数不再使用
bool ScoringSystem::checkExtremeDrift(float /*triggerSlipLevel_not_used*/) const
{
	const float aSevereRad = ScoringConfig::get()->getVar("SlipAngleSevereRad");
	int n = 0;
	for (int i = 0; i < 4; ++i) {
		const Tyre* t = car->tyres[i].get();
		if (isExtremeDriftByAngle(t, aSevereRad)) ++n;
	}
	return (n >= 2); // 至少两条轮胎处于“严重”角度
}

} // namespace D
