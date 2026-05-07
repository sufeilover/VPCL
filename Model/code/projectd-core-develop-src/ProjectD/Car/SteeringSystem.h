#pragma once

#include "Car/CarCommon.h"
#include "Car/DynamicController.h"
#pragma pack(push, 8)          // <== 新增，强制该类型用 8 对齐
namespace D {

struct SteeringSystem : public NonCopyable
{
	SteeringSystem();
	~SteeringSystem();
	void init(Car* car);
	void step(float dt);

	// config
	DynamicController ctrl4ws;
	float linearRatio = 0.003f;
	bool has4ws = false;

	// runtime
	Car* car = nullptr;
};

}
#pragma pack(pop)              // <== 新增，恢复之前的对齐
