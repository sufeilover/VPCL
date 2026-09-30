#include "Car/CarImpl.h"
#include "Sim/Simulator.h"
#include "Sim/Track.h"
//setVelocity
//setAngularVelocity
//setPosition
//setRotation
#include <fstream>
#include <string>
#include <vector>

namespace D {

// —— 静态 kappa 表 & 固定 CSV 路径（无兜底解析：两列数字，无表头）——
static std::vector<float> s_kappaTable;
static const char* kSplineCsvPath =
R"(C:\Users\itserv\OneDrive - Tshwane University of Technology\AC-simulator\code\FAMILIAR-TEST\projectd-core-develop\projectd-core-develop\content\tracks\ks_silverstone1967\spline.csv)";

static void LoadKappaTableOnce()
{
    if (!s_kappaTable.empty()) return; // 仅首次装载
    std::ifstream fin(kSplineCsvPath);
    std::string line;
    while (std::getline(fin, line))
    {
        // 仅支持 “id,kappa” 两列纯数字；不做任何健壮性处理
        std::size_t comma = line.find(',');
        int   id    = std::stoi(line.substr(0, comma));
        float kappa = std::stof(line.substr(comma + 1));

        if ((size_t)id >= s_kappaTable.size())
            s_kappaTable.resize((size_t)id + 1, 0.0f);

        s_kappaTable[(size_t)id] = kappa;
    }
}


Car::Car(Track* _track)
{
	TRACE_CTOR(Car);

	track = _track;
	sim = _track->sim;

	bounds.length = 4.0f;
	bounds.width = 2.0f;
	bounds.lengthFront = 2.0f;
	bounds.lengthRear = 2.0f;

	lastBodyMassUpdateTime = -100000000.0;
}

Car::~Car()
{
	TRACE_DTOR(Car);
}

//=============================================================================
// INIT
//=============================================================================

bool Car::init(const std::wstring& modelName)
{
	log_printf(L"Car: init: carId=%d modelName=\"%s\"", physicsGUID, modelName.c_str());

	//physicsGUID = (int)sim->cars.size(); // now managed by Simulator::addCar

	auto& pCore = sim->physics;
	body = pCore->createRigidBody();
	fuelTankBody = pCore->createRigidBody();

	unixName = modelName;
	//carDataPath = L"C:\\steam\\steamapps\\common\\assettocorsa\\content\\cars\\" + unixName + L"\\data\\";
	carDataPath = sim->basePath + L"content/cars/" + unixName + L"/data/";
	log_printf(L"carData: \"%s\"", carDataPath.c_str());

	initCarData();
	initProbes();
	initLookAhead();

	fuelTankBody->setMassBox(1.0f, 0.5f, 0.5f, 0.5f); // TODO: check
	fuelTankBody->setPosition(fuelTankPos);
	fuelTankJoint = sim->physics->createFixedJoint(fuelTankBody, body);

	//log_printf(L"car body=%p", body.get());
	//log_printf(L"car fuelTankBody=%p", fuelTankBody.get());

	// Suspensions

	antirollBars.emplace_back(std::make_unique<AntirollBar>());
	antirollBars.emplace_back(std::make_unique<AntirollBar>());

	auto ini(std::make_unique<INIReader>(carDataPath + L"suspensions.ini"));

	auto strRearType = ini->getString(L"REAR", L"TYPE");
	if (strRearType == L"AXLE")
	{
		rigidAxle = pCore->createRigidBody();
		axleTorqueReaction = ini->getFloat(L"AXLE", L"TORQUE_REACTION");
		//log_printf(L"car rigidAxle=%p", rigidAxle.get());
	}

	for (int index = 0; index < 4; ++index)
	{
		std::wstring strSuspType;
		SuspensionType eSuspType;
		std::unique_ptr<SuspensionBase> pSusp;

		if (index >= 2)
			strSuspType = ini->getString(L"REAR", L"TYPE");
		else
			strSuspType = ini->getString(L"FRONT", L"TYPE");

		//log_printf(L"create suspension id=%d type=%s", index, strSuspType.c_str());

		if (strSuspType == L"STRUT")
		{
			eSuspType = SuspensionType::Strut;
			auto pImpl = new SuspensionStrut(); pSusp.reset(pImpl);
			pImpl->init(pCore, body, index, carDataPath);
		}
		else if (strSuspType == L"DWB")
		{
			eSuspType = SuspensionType::DoubleWishbone;
			auto pImpl = new SuspensionDW(); pSusp.reset(pImpl);
			pImpl->init(pCore, body, antirollBars[0].get(), antirollBars[1].get(), index, carDataPath);
		}
		else if (strSuspType == L"ML")
		{
			eSuspType = SuspensionType::Multilink;
			auto pImpl = new SuspensionML(); pSusp.reset(pImpl);
			pImpl->init(pCore, body, index, carDataPath);
		}
		else if (strSuspType == L"AXLE" && index >= 2)
		{
			eSuspType = SuspensionType::Axle;
			auto eSide = (index == 2) ? RigidAxleSide::Left : RigidAxleSide::Right;
			auto pImpl = new SuspensionAxle(); pSusp.reset(pImpl);
			pImpl->init(pCore, body, rigidAxle, index, eSide, carDataPath);
		}
		else
		{
			SHOULD_NOT_REACH_FATAL;
		}

		if (index >= 2)
			suspensionTypeR = eSuspType;
		else
			suspensionTypeF = eSuspType;

		ISuspension* pSuspInterface = pSusp.get();
		suspensions.emplace_back(pSuspInterface);
		suspensionsImpl.emplace_back(std::move(pSusp));

		auto pTyre = std::make_unique<Tyre>();
		pTyre->init(this, pSuspInterface, sim->track.get(), index, carDataPath);
		tyres.emplace_back(std::move(pTyre));
	}

	for (const auto& compound : tyres[0]->compoundDefs)
	{
		tyreCompounds.emplace_back(compound->name);
	}

	#if 1
	for (size_t i = 0; i < 4; i += 2)
	{
		if (suspensions[i]->getType() == SuspensionType::DoubleWishbone && 
			suspensions[i + 1]->getType() == SuspensionType::DoubleWishbone)
		{
			auto susA = (SuspensionDW*)suspensions[i];
			auto susB = (SuspensionDW*)suspensions[i + 1];
			bool isFront = (i == 0);

			auto pSpring = std::make_unique<HeaveSpring>();
			pSpring->init(body.get(), susA, susB, isFront, carDataPath);
			heaveSprings.emplace_back(std::move(pSpring));
		}
	}
	#endif

	ridePickupPoint[0].z = suspensions[0]->getBasePosition().z;
	ridePickupPoint[1].z = suspensions[2]->getBasePosition().z;

	antirollBars[0]->init(body.get(), suspensions[0], suspensions[1]);
	antirollBars[1]->init(body.get(), suspensions[2], suspensions[3]);
	antirollBars[0]->k = ini->getFloat(L"ARB", L"FRONT");
	antirollBars[1]->k = ini->getFloat(L"ARB", L"REAR");
	auto strArb0 = carDataPath + L"ctrl_arb_front.ini";
	auto strArb1 = carDataPath + L"ctrl_arb_rear.ini";
	if (osFileExists(strArb0))
	{
		antirollBars[0]->ctrl.init(this, strArb0);
	}
	if (osFileExists(strArb1))
	{
		antirollBars[1]->ctrl.init(this, strArb1);
	}

	// Components

	colliderManager.reset(new CarColliderManager());
	colliderManager->init(this);
	loadColliderBlob();

	slipStream.reset(new SlipStream());
	aeroMap.reset(new AeroMap());
	auto vFrontPos = suspensions[0]->getBasePosition();
	auto vRearPos = suspensions[2]->getBasePosition();
	aeroMap->init(this, vFrontPos, vRearPos);

	water.reset(new ThermalObject());
	water->tmass = 20.0f;
	water->coolSpeedK = 0.002f;

	brakeSystem.reset(new BrakeSystem());
	brakeSystem->init(this);

	steeringSystem.reset(new SteeringSystem());
	steeringSystem->init(this);
	steeringSystem->linearRatio = steerLinearRatio;

	drivetrain.reset(new Drivetrain());
	drivetrain->init(this);
	controls.isShifterSupported = drivetrain->isShifterSupported;

	gearChanger.reset(new GearChanger());
	gearChanger->init(this);

	autoClutch.reset(new AutoClutch());
	autoClutch->init(this);

	autoBlip.reset(new AutoBlip());
	autoBlip->init(this);

	autoShift.reset(new AutoShifter());
	autoShift->init(this);

	scoring.reset(new ScoringSystem());
	scoring->init(this);

	setup.reset(new SetupManager());
	setup->init(this);

	state.reset(new CarState());

	LoadKappaTableOnce(); // 一次性读CSV并构建静态表

	updateBodyMass();

	auto pThis = this;
	sim->evOnStepCompleted.add(this, [pThis](double dt) {
		pThis->postStep((float)dt);
	});

	return true;
}
//=============================================================================

void Car::initCarData()
{
	auto ini(std::make_unique<INIReader>(carDataPath + L"car.ini"));
	GUARD_FATAL(ini->ready);

	screenName = ini->getString(L"INFO", L"SCREEN_NAME");
	mass = ini->getFloat(L"BASIC", L"TOTALMASS");

	if (ini->hasSection(L"EXPLICIT_INERTIA"))
	{
		explicitInertia = ini->getFloat3(L"EXPLICIT_INERTIA", L"INERTIA");
		body->setMassExplicitInertia(mass, explicitInertia.x, explicitInertia.y, explicitInertia.z);
	}
	else
	{
		bodyInertia = ini->getFloat3(L"BASIC", L"INERTIA");
		body->setMassBox(mass, bodyInertia.x, bodyInertia.y, bodyInertia.z);
	}

	if (ini->hasSection(L"FUEL_EXT"))
	{
		fuelKG = ini->getFloat(L"FUEL_EXT", L"KG_PER_LITER");
	}

	ffMult = ini->getFloat(L"CONTROLS", L"FFMULT") * 0.001f;
	steerLock = ini->getFloat(L"CONTROLS", L"STEER_LOCK");
	steerRatio = ini->getFloat(L"CONTROLS", L"STEER_RATIO");

	steerLinearRatio = ini->getFloat(L"CONTROLS", L"LINEAR_STEER_ROD_RATIO");
	if (steerLinearRatio == 0.0f)
		steerLinearRatio = 0.003f;

	steerAssist = ini->getFloat(L"CONTROLS", L"STEER_ASSIST");
	if (steerAssist == 0.0f)
		steerAssist = 1.0f;

	fuelConsumptionK = ini->getFloat(L"FUEL", L"CONSUMPTION");
	fuel = ini->getFloat(L"FUEL", L"FUEL");
	maxFuel = ini->getFloat(L"FUEL", L"MAX_FUEL");

	if (maxFuel == 0.0f)
		maxFuel = 30.0f;
	if (fuel == 0.0f)
		fuel = 30.0f;
	requestedFuel = (float)fuel;

	float fPickup = ini->getFloat(L"RIDE", L"PICKUP_FRONT_HEIGHT");
	ridePickupPoint[0] = vec3f(0, fPickup, 0);

	fPickup = ini->getFloat(L"RIDE", L"PICKUP_REAR_HEIGHT");
	ridePickupPoint[1] = vec3f(0, fPickup, 0);

	fuelTankPos = ini->getFloat3(L"FUELTANK", L"POSITION");

	graphicsOffset = ini->getFloat3(L"BASIC", L"GRAPHICS_OFFSET");
	graphicsPitchRotation = ini->getFloat(L"BASIC", L"GRAPHICS_PITCH_ROTATION") * (M_PI / 180.0f);
}

void Car::initProbes()
{
	auto ini(std::make_unique<INIReader>(sim->basePath + L"cfg/sim.ini"));
	if (!ini->ready)
		return;

	for (int id = 1; id <= CarState::MaxProbes; ++id)
	{
		auto section = strwf(L"CAR_PROBE_%d", id);
		if (!ini->hasSection(section))
			break;

		float yaw = ini->getFloat(section, L"YAW");
		float length = ini->getFloat(section, L"LENGTH");

		probes.push_back(ray3f(vec3f(0, 0, 0), vec3f(0, 0, 1).rotateAxisAngle(vec3f(0, 1, 0), yaw * M_DEG2RAD), length));
	}
}

void Car::initLookAhead()
{
	auto ini(std::make_unique<INIReader>(sim->basePath + L"cfg/sim.ini"));
	if (ini->ready)
	{
		ini->tryGetInt(L"CAR_LOOK_AHEAD", L"COUNT", lookAheadCount);
		ini->tryGetFloat(L"CAR_LOOK_AHEAD", L"STEP", lookAheadStep);
	}

	lookAhead.resize(lookAheadCount);
}

//=============================================================================

void Car::loadColliderBlob()
{
	#pragma pack(push, 1)
	struct BlobCollider
	{
		uint32_t magic = 0;
		uint32_t numVertices = 0;
		uint32_t numIndices = 0;
	};
	#pragma pack(pop)

	FileHandle file;
	auto strPath = carDataPath + L"collider.bin";
	GUARD_FATAL(file.open(strPath.c_str(), L"rb"));

	BlobCollider raw;
	GUARD_FATAL(fread(&raw, sizeof(raw), 1, file.fd) == 1);
	GUARD_FATAL(raw.numVertices && raw.numIndices);

	auto mesh = sim->physics->createTriMesh();
	mesh->resize(raw.numVertices, raw.numIndices);
	GUARD_FATAL(fread(mesh->getVB(), raw.numVertices * sizeof(TriMeshVertex), 1, file.fd) == 1);
	GUARD_FATAL(fread(mesh->getIB(), raw.numIndices * sizeof(TriMeshIndex), 1, file.fd) == 1);

	auto gm = getGraphicsOffsetMatrix();
	initColliderMesh(mesh, gm);
}

//=============================================================================

void Car::initColliderMesh(ITriMeshPtr mesh, const mat44f& bodyMatrix)
{
	vec3f vMin(FLT_MAX, FLT_MAX, FLT_MAX);
	vec3f vMax(vMin * -1.0f);

	auto* pVertices = mesh->getVB();
	auto nVertexCount = mesh->getVertexCount();

	for (auto i = 0; i < nVertexCount; ++i)
	{
		auto& v = pVertices[i];

		vMin.x = tmin(vMin.x, v.x);
		vMin.y = tmin(vMin.y, v.y);
		vMin.z = tmin(vMin.z, v.z);

		vMax.x = tmax(vMax.x, v.x);
		vMax.y = tmax(vMax.y, v.y);
		vMax.z = tmax(vMax.z, v.z);
	}

	vec3f vPos(&bodyMatrix.M41);
	bounds.min = vPos + vMin;
	bounds.max = vPos + vMax;
	bounds.length = fabsf(bounds.max.z - bounds.min.z);
	bounds.width = fabsf(bounds.max.x - bounds.min.x);
	bounds.lengthFront = fabsf(bounds.max.z);
	bounds.lengthRear = fabsf(bounds.min.z);

	body->addMeshCollider(mesh, bodyMatrix, physicsGUID + 1, C_CATEGORY_CAR, C_MASK_CAR_MESH);
	collider = mesh;
}

//=============================================================================
// STEP
//=============================================================================

void Car::reset()
{
	framesToSleep = 50; //（加载后“休眠帧”，常用于稳定/冻结几帧），但是这个参数越小车辆稳定所占用的帧数越多，多大车辆稳定所占用的帧数越少，目前看50不需要动
	water->t = 60; //（冷却液温度直接设为 60℃），虽然我们是热启动，但是其实还是从半0初始状态开始的，可以从模型中记录这个参数并使用，或者自行利用读取的ac进行计算更新
	fuel = requestedFuel; //（燃油加到“请求量”），ac暴露

	collisionFlag = false; //我们的模型不可考虑碰撞，保持这个数，ac暴露
	oldCollisionFlag = false; //我们的模型不可考虑碰撞，保持这个数，ac暴露
	outOfTrackFlag = false; //我们的模型在出赛道的时候停止预测，ac暴露

	lastTrackPointTimestamp = sim ? (float)sim->physicsTime : 0;
	nearestTrackPointId = 0;//没暴露,热启动不需要管这个，这是用于车辆移动的
	oldTrackPointId = 0;//没暴露,热启动不需要管这个，这是用于车辆移动的
	splinePointId = 0;//没暴露,热启动不需要管这个，这是用于车辆移动的

	trackLocation = 0;//没暴露,热启动不需要管这个，这是用于车辆移动的
	oldTrackLocation = 0;//没暴露,热启动不需要管这个，这是用于车辆移动的

	for (int i = 0; i < 5; ++i) 
	{
		damageZoneLevel[i] = 0;
		oldDamageZoneLevel[i] = 0; //目前这个参数一直是0，保持默认就好
	}

	scoring->reset(); //我们不需要这个
}

//=============================================================================

void Car::stepPreCacheValues(float dt)
{
	speed.value = body->getVelocity().len();
}

//=============================================================================

void Car::step(float dt)
{
	collisionFlag = false;
	outOfTrackFlag = false;

	if (!physicsGUID)
	{
		vec3f vBodyVelocity = body->getVelocity();
		float fVelSq = vBodyVelocity.sqlen();
		float fERP;

		if (fVelSq >= 1.0f)
		{
			fERP = 0.3f;
			for (auto* pSusp : suspensions)
			{
				auto* pImpl = (SuspensionBase*)pSusp;
				pSusp->setERPCFM(fERP, pImpl->baseCFM);
			}
		}
		else
		{
			fERP = 0.9f;
			for (auto& pSusp : suspensions)
			{
				pSusp->setERPCFM(fERP, 0.0000001f);
			}
		}

		fuelTankJoint->setERPCFM(fERP, -1.0f);
	}

	pollControls(dt);

	controls.steer = tclamp(controls.steer, -1.0f, 1.0f);
	controls.clutch = tclamp(controls.clutch, 0.0f, 1.0f);
	controls.brake = tclamp(controls.brake, 0.0f, 1.0f);
	controls.handBrake = tclamp(controls.handBrake, 0.0f, 1.0f);
	controls.gas = tclamp(controls.gas, 0.0f, 1.0f);

	smoothSteerTarget = controls.steer;

	if (smoothSteer)
	{
		float diff = smoothSteerTarget - smoothSteerValue;
		smoothSteerValue += diff * smoothSteerSpeed * dt;
		controls.steer = smoothSteerValue;
	}
	else
	{
		smoothSteerValue = smoothSteerTarget;
	}

	updateAirPressure();

	float fRpmAbs = fabsf(drivetrain->getEngineRPM());
	float fTurboBoost = tmax(0.0f, drivetrain->engineModel->status.turboBoost);
	double fNewFuel = fuel - (fRpmAbs * dt * drivetrain->engineModel->gasUsage) * (fTurboBoost + 1.0) * fuelConsumptionK * 0.001 * sim->fuelConsumptionRate;
	fuel = fNewFuel;

	if (fNewFuel > 0.0f)
	{
		drivetrain->engineModel->fuelPressure = 1.0f;
	}
	else
	{
		fuel = 0;
		drivetrain->engineModel->fuelPressure = 0;
	}

	updateBodyMass();

	float fSteerAngleSig = (steerLock * controls.steer) / steerRatio;
	if (!isfinite(fSteerAngleSig))
	{
		SHOULD_NOT_REACH_WARN;
		fSteerAngleSig = 0;
	}

	finalSteerAngleSignal = fSteerAngleSig;

	bool bAllTyresLoaded = true;
	for (int i = 0; i < 4; ++i)
	{
		if (tyres[i]->status.load <= 0.0f)
		{
			bAllTyresLoaded = false;
			break;
		}
	}

	autoClutch->step(dt);

	float fSpeed = speed.ms();
	vec3f fAngVel = body->getAngularVelocity();
	float fAngVelSq = fAngVel.sqlen();

	if (fSpeed >= 0.5f || fAngVelSq >= 1.0f)
	{
		sleepingFrames = 0;
	}
	else
	{
		if (bAllTyresLoaded 
			&& (controls.gas <= 0.01f 
				|| controls.clutch <= 0.01f 
				|| drivetrain->currentGear == 1))
		{
			sleepingFrames++;
		}
		else
		{
			sleepingFrames = 0;
		}

		if (sleepingFrames > framesToSleep)
		{
			body->stop();
			fuelTankBody->stop();
		}
	}

	vec3f vBodyVel = body->getVelocity();
	vec3f vAccel = (vBodyVel - lastVelocity) * (1.0f / dt) * 0.10197838f;
	lastVelocity = vBodyVel;
	accG = body->worldToLocalNormal(vAccel);

	stepThermalObjects(dt);
	stepComponents(dt);

	//updateColliderStatus(dt); // TODO
	//if (!physicsGUID) stepJumpStart(dt); // TODO
}

//=============================================================================

void Car::updateAirPressure()
{
	float fAirDensity = sim->getAirDensity();

	if (slipStreamEffectGain > 0.0f)
	{
		vec3f vPos = body->getPosition(0.0f);
		float fMinSlip = 1.0f;

		//for (SlipStream* pSS : sim->slipStreams)
		for (auto* otherCar : sim->cars)
		{
			//if (pSS != slipStream.get())
			if (otherCar != this)
			{
				auto* pSS = otherCar->slipStream.get();

				float fSlip = tclamp((1.0f - (pSS->getSlipEffect(vPos) * slipStreamEffectGain)), 0.0f, 1.0f);

				if (fMinSlip > fSlip)
					fMinSlip = fSlip;
			}
		}

		fAirDensity = ((fAirDensity - (fMinSlip * fAirDensity)) * (0.75f / slipStreamEffectGain)) + (fMinSlip * fAirDensity);
	}

	aeroMap->airDensity = fAirDensity;
}

//=============================================================================

void Car::updateBodyMass()
{
	if (sim->physicsTime - lastBodyMassUpdateTime > 1000.0)
	{
		if (bodyInertia.x != 0.0f || bodyInertia.y != 0.0f || bodyInertia.z != 0.0f)
		{
			float fBodyMass = calcBodyMass();
			body->setMassBox(fBodyMass, bodyInertia.x, bodyInertia.y, bodyInertia.z);
		}
		else
		{
			body->setMassExplicitInertia(mass, explicitInertia.x, explicitInertia.y, explicitInertia.z);
			log_printf(L"setMassExplicitInertia");
		}

		float fFuelMass = tmax(0.1f, (fuelKG * (float)fuel));
		fuelTankBody->setMassBox(fFuelMass, 0.5f, 0.5f, 0.5f); // TODO: check

		lastBodyMassUpdateTime = sim->physicsTime;
	}
}

//=============================================================================

float Car::calcBodyMass()
{
	float fSuspMass = 0;
	for (int i = 0; i < 4; ++i)
		fSuspMass += suspensions[i]->getMass();

	return (mass - fSuspMass) + ballastKG;
}

//=============================================================================

void Car::stepThermalObjects(float dt)
{
	float fRpm = drivetrain->getEngineRPM();
	if (fRpm > (drivetrain->engineModel->data.minimum * 0.8f))
	{
		float fLimiter = (float)drivetrain->engineModel->getLimiterRPM();
		water->addHeadSource((((fRpm / fLimiter) * 20.0f) * controls.gas) + 85.0f);
	}
	
	water->step(dt, sim->ambientTemperature, speed);
}

//=============================================================================

void Car::stepComponents(float dt)
{
	brakeSystem->step(dt);
	//edl.step(dt);

	for (auto& iter : suspensions)
	{
		iter->step(dt);
	}

	for (auto& iter : tyres)
	{
		iter->step(dt);
	}
	sendFF(dt);

	for (auto& iter : heaveSprings)
	{
		if (iter->k != 0.0f)
			iter->step(dt);
	}

	//drs.step(dt);
	aeroMap->step(dt);
	//kers.step(dt);
	//ers.step(dt);
	steeringSystem->step(dt);
	autoBlip->step(dt);
	autoShift->step(dt);
	gearChanger->step(dt);
	drivetrain->step(dt);

	for (auto& iter : antirollBars)
	{
		iter->step(dt);
	}

	//abs.step(dt);
	//tractionControl.step(dt);
	//speedLimiter.step(dt);
	//colliderManager.step(dt);
	//stabilityControl.step(dt);
	//fuelLapEvaluator.step(dt);
}

//=============================================================================

void Car::postStep(float dt)
{
	OnStepCompleteEvent e;
	e.car = this;
	e.physicsTime = sim->physicsTime;
	evOnStepComplete.fire(e);

	vec3f vBodyVel = body->getVelocity();
	vec3f vBodyPos = body->getPosition(0);
	slipStream->setPosition(vBodyPos, vBodyVel);

	updateTrackLocator(dt);
	updateLookAhead();

	scoring->step(dt);

	updateCarState();

	if (senseiEnabled)
		updateSensei();

	for (int i = 0; i < 5; ++i)
		oldDamageZoneLevel[i] = damageZoneLevel[i];

	oldCollisionFlag = collisionFlag;

	if (audioRenderer)
		audioRenderer->update(dt);
}

//=============================================================================

void Car::updateTrackLocator(float dt)
{
	const auto numRays = probes.size();
	if (numRays > 0)
	{
		if (probeHits.size() != numRays)
			probeHits.resize(numRays);

		for (size_t rayId = 0; rayId < numRays; ++rayId)
		{
			const auto& r = probes[rayId];
			const auto rayStart = body->localToWorld(r.pos);
			const auto rayEnd = body->localToWorld(r.pos + r.dir * r.length);
			probeHits[rayId] = track->rayCastTrackBounds(rayStart, (rayEnd - rayStart).get_norm(), r.length);
		}
	}

	const auto bodyPos = body->getPosition(0);
	const int bestPoint = (int)track->getPointIdAtLocation(bodyPos);
	const float kappa = s_kappaTable[(size_t)bestPoint];
	state->kappa = kappa;

	if (nearestTrackPointId != bestPoint)
	{
		oldTrackPointId = nearestTrackPointId;
		nearestTrackPointId = bestPoint;
		lastTrackPointTimestamp = (float)sim->physicsTime;
	}

	oldTrackLocation = trackLocation;
	trackLocation = 0;

	const int numPoints = (int)track->fatPoints.size();
	if (bestPoint >= 0 && bestPoint < numPoints)
	{
		Spline3dPointInfo info;
		//trackLocation = tclamp((float)bestPoint / (float)numTrackPoints, 0.0f, 1.0f); // TODO: compute accurate location
		if (track->getDistanceAlongSplineAtLocation(bodyPos, bestPoint, info))
		{
			splinePointId = info.id;
			trackLocation = tclamp(info.dist / track->computedTrackLength, 0.0f, 1.0f);
			worldSplinePosition = info.pos;
		}

		const auto bodyR = body->getWorldMatrix(0).getRotator();
		const auto bodyFrontDir = (vec3f(0, 0, 1) * bodyR).get_norm();
		const auto bodyVelDir = body->getVelocity().get_norm();

		const auto& pt = track->fatPoints[bestPoint];
		bodyVsTrack = bodyFrontDir * pt.forwardDir;

		if (speed.kmh() > 3.0f)
			velocityVsTrack = bodyVelDir * pt.forwardDir;
		else
			velocityVsTrack = 0.0f;
	}
}

//=============================================================================

void Car::updateLookAhead()
{
	if (lookAhead.size() != (size_t)lookAheadCount)
		lookAhead.resize((size_t)lookAheadCount);

	const vec3f up(0, 1, 0);
	const vec3f curTrackDir = track->getTrackDirectionAtDistance(trackLocation);

	const auto bodyR = body->getWorldMatrix(0).getRotator();
	const auto bodyFrontDir = (vec3f(0, 0, 1) * bodyR).get_norm();

	// is car driving in the right direction?
	const float driveDir = signf(bodyFrontDir * curTrackDir);

	for (int i = 0; i < lookAheadCount; ++i)
	{
		const float distanceNorm = trackLocation + ((lookAheadStep * (float)(i + 1)) / track->computedTrackLength) * driveDir;
		const vec3f dir = track->getTrackDirectionAtDistance(distanceNorm);

		const float angle = atan2f(dir.cross(curTrackDir) * up, curTrackDir * dir);
		lookAhead[i] = angle;
		//lookAhead[i] = linscalef(angle, -M_PI, M_PI, -1.0f, 1.0f);
	}
}

//=============================================================================
void Car::updateCarState()
{
    // see Car::getPhysicsState, SharedMemoryWriter::updatePhysics

    // ───────────── 基础与标识 ─────────────
    state->carId      = (int32_t)physicsGUID;
    state->simId      = (int32_t)sim->simulatorId;
    state->timestamp  = (float)sim->physicsTime;

    // 控制量（供速度优化器候选边界判断）
    state->controls   = controls;

    // 事件/轨迹相关
    state->collisionFlag            = collisionFlag;
    state->outOfTrackFlag           = outOfTrackFlag;
    state->trackPointId             = nearestTrackPointId;
    state->lastTrackPointTimestamp  = lastTrackPointTimestamp;
    state->trackLocation            = trackLocation;
		
		// 赛道左右边缘（基于最近的 fatPoint）
	{
		const int n = (int)track->fatPoints.size();
		int id = (int)nearestTrackPointId;

		if (n > 0) {
			if (id < 0) id = 0;
			if (id >= n) id = n - 1;

			const auto& pt = track->fatPoints[id];
			state->trackLeft  = pt.left;
			state->trackRight = pt.right;

			// 可选
			// state->trackCenter = 0.5f * (pt.left + pt.right);
			// state->trackWidth  = (pt.left - pt.right).len();
		} else {
			state->trackLeft  = vec3f(0,0,0);
			state->trackRight = vec3f(0,0,0);
			// state->trackCenter = vec3f(0,0,0);
			// state->trackWidth  = 0.0f;
		}
	}

    // 与赛道的相对角（弧度）
    state->bodyVsTrack     = bodyVsTrack;       // 夹角的余弦值
    state->velocityVsTrack = velocityVsTrack;   // 夹角的余弦值

    // 传动/转速/速度
	state->driftNow           = scoring ->driftNow;
    state->finalRatio         = drivetrain->finalRatio;
    state->diffPowerRamp      = drivetrain->diffPowerRamp;
    state->diffCoastRamp      = drivetrain->diffCoastRamp;   // ← 补齐
    state->outShaftLvelocity  = drivetrain->outShaftL.velocity;
    state->outShaftRvelocity  = drivetrain->outShaftR.velocity;
    state->drivevelocity      = drivetrain->drive.velocity;
    state->enginevelocity     = drivetrain->engine.velocity;
    state->rootVelocity       = drivetrain->rootVelocity;

    state->engineRPM          = getEngineRpm();
    state->speedMS            = speed.ms();
    state->gear               = drivetrain->currentGear;
    state->gearGrinding       = drivetrain->isGearGrinding ? 1 : 0;

    // 先在外层声明，保证后面可见
    mat44f hubM1;  bool haveHub1 = false;
    mat44f hubM2;  bool haveHub2 = false;
	mat44f strutBodyM1;  
    mat44f strutBodyM2;  
    mat44f axM;    bool haveAx   = false;

	float travelstrut1 = 0.0f;
	float travelstrut2 = 0.0f;
	float travelaxle1 = 0.0f;
	float travelaxle2 = 0.0f;

    // 前轴两个支柱（i=0,1），读取各自 hub 的世界矩阵
    for (size_t i = 0; i < 2; ++i)
    {
        if (auto* susp = dynamic_cast<SuspensionStrut*>(suspensions[i])) {
            const mat44f hubm = susp->hub->getWorldMatrix(0);
			const mat44f strutBodym = susp->strutBody->getWorldMatrix(0);
			const float travelstrut = susp->status.travel;
            if (i == 0) { hubM1 = hubm; haveHub1 = true; strutBodyM1 =strutBodym;  travelstrut1 = travelstrut;}
            else        { hubM2 = hubm; haveHub2 = true; strutBodyM2 =strutBodym; travelstrut2 = travelstrut;}
        }
    }

    // 后桥（i=2,3）——如果你的车只有一个整体 axle，就取其中任意一个（或按你需要挑选）
    for (size_t i = 2; i < 4; ++i)
    {
        if (auto* susp = dynamic_cast<SuspensionAxle*>(suspensions[i])) {
            axM = susp->axle->getWorldMatrix(0);
			const float travelaxle = susp->status.travel;
			if (i == 2) {travelaxle1 = travelaxle;}
            else        {travelaxle2 = travelaxle;}
            haveAx = true;
            // 如果两个 i 都能取到且你只想保留第一个，break;
            // break;
        }
    }
	state->travelstrut1 = travelstrut1;
	state->travelstrut2 = travelstrut2;
	state->travelaxle1 = travelaxle1;
	state->travelaxle2 = travelaxle2;
	// 姿态/位置/速度
    auto bodyM = body->getWorldMatrix(0);
	auto fuelTankBodyM = fuelTankBody->getWorldMatrix(0);
    state->bodyMatrix = (bodyM);
	state->fuelTankyMatrix = (fuelTankBodyM);
	state->hub1Matrix = (hubM1);
	state->hub2Matrix = (hubM2);
	state->strutBodyM1 = (strutBodyM1);
	state->strutBodyM2 = (strutBodyM2);
	state->axleMatrix = (axM);
    state->bodyPos    = {bodyM.M41, bodyM.M42, bodyM.M43};
    state->bodyEuler  = (bodyM.getEulerAngles());
    state->accG       = accG;
    state->velocity         = body->getVelocity();         // world
    state->localVelocity    = body->getLocalVelocity();    // local
    state->angularVelocity      = body->getAngularVelocity();
    state->localAngularVelocity = body->getLocalAngularVelocity();

    // 轮胎/悬挂 & 轮胎力/载荷
    for (int i = 0; i < 4; ++i)
    {
        auto* pTyre = tyres[i].get();
        state->hubMatrix[i]        = (suspensions[i]->getHubWorldMatrix());
        state->tyreContacts[i]     = pTyre->contactPoint;
        state->tyrecontactNormal[i]= pTyre->contactNormal;
        state->tyreroadHeading[i]  = pTyre->roadHeading;
        state->tyreroadRight[i]    = pTyre->roadRight;
        state->tyreLoad[i]         = pTyre->status.load;
		state->fDepth[i]           = pTyre->status.depth;
        state->tyreAngularSpeed[i] = pTyre->status.angularVelocity;
        state->tyreSlipRatio[i]    = pTyre->status.slipRatio;
        state->tyreNdSlip[i]       = pTyre->status.ndSlip;
        state->slipAngleRAD[i]     = pTyre->status.slipAngleRAD;
        state->fy[i]               = pTyre->status.Fy;
		// —— 写出四个轮胎的 RayCaster 默认参数（起点/方向/长度）——
    }
	for (int wi = 0; wi < 4; ++wi)
	{
		auto* ty = tyres[wi].get();
		// 默认起点：轮胎世界位置（轮心/轮胎所用的世界位姿）
		state->tyreRayOrigin[wi] = ty->worldPosition;

		// 默认方向：世界向下 (0,-1,0)
		// 如果你要用轮胎姿态来“略偏”的向下，可替换为：
		// const mat44f& R = ty->worldRotation;
		// vec3f downLocal(0, -1, 0);
		// vec3f downWorld = (downLocal * R.getRotator()).get_norm();
		// state->tyreRayDir[wi] = downWorld;
		state->tyreRayDir[wi] = vec3f(0, -1, 0);

		// 默认长度：与 Tyre::init() 创建时一致（3.0f）
		state->tyreRayLength[wi] = 3.0f;
	}

    // 探针/前瞻
    for (size_t i = 0; i < probes.size(); ++i)
        state->probes[i] = probeHits[i];
    for (size_t i = 0; i < lookAhead.size(); ++i)
        state->lookAhead[i] = lookAhead[i];

    // 奖励（注意：你的系统 reward = -cost）

	state->trackW  = scoring->trackW;
	state->Ctrack  = scoring->C_track;
	state->Cspeed  = scoring->C_speed;
	state->Csmooth = scoring->C_smooth;

	state->costy  = scoring->cost_y;
	state->costpsi  = scoring->cost_psi;
	state->costcross = scoring->cost_cross;
	state->costbound  = scoring->cost_bound;
	state->costdrift = scoring->cost_drift;

    state->stepReward  = scoring->stepReward;
    state->totalReward = scoring->totalReward;

    // ─────────────────────────────────────────────────────────
    // 实时派生量（CarState 顶层）—— 全部使用你直出或简单代数，不做兜底
    // ─────────────────────────────────────────────────────────

    // 1) 侧滑角 β（弧度）：引擎直接给
    state->driftComboCounter = scoring->driftComboCounter;   // 若已有缓存字段，可直接赋值

    // 2) μ 占用：Σ|Fy| / Σ|Fz|
    {
        float sumAbsFy = 0.0f, sumAbsFz = 0.0f;
        for (int w = 0; w < 4; ++w) {
            sumAbsFy += std::fabs(state->fy[w]);
            sumAbsFz += std::fabs(state->tyreLoad[w]);
        }
        state->lateralUtil = (sumAbsFz > 1e-6f) ? (sumAbsFy / sumAbsFz) : 0.0f;
    }

    // 3) 航向误差 ePsi（弧度，wrap 到 [-π,π]）
    state->ePsi = scoring->e_psi;

    // 4) 横向误差 eY（米）—— 若 Track 提供 API，替换 TODO
    state->eY = scoring->e_y;
	state->glastFramePenalty = scoring->g_lastFramePenalty;

    // 5) 边界裕度（0..1）—— 若 Track 提供 API，替换 TODO
    state->borderMarginRatio = scoring->ratio;
	
    // 6) 沿赛道方向速度（非负）
    state->vAlongTrack = std::max(0.0f, state->speedMS * std::cos(state->velocityVsTrack));

    // ─────────────────────────────────────────────────────────
    // Basicline 环形缓冲（序列）：写入完整样本
    // ─────────────────────────────────────────────────────────
    {
        CarState::BasiclineSample sample;
        sample.timestamp        = state->timestamp;
        sample.bodyPos          = state->bodyPos;

        // 与赛道相对角（弧度）
        sample.bodyVsTrack      = state->bodyVsTrack;
        sample.velocityVsTrack  = state->velocityVsTrack;

        // 碰撞/越界
        sample.collisionFlag    = state->collisionFlag;
        sample.outOfTrackFlag   = state->outOfTrackFlag;

        // 轮胎力/载荷
        for (int i = 0; i < 4; ++i) {
            sample.fy[i]        = state->fy[i];
            sample.tyreLoad[i]  = state->tyreLoad[i];
        }

        // 曲率（1/m）—— 如有 Track API，填真实值
        sample.kappa = state->kappa;
		sample.bestPoint = state->trackPointId;

        // 侧滑角/μ 占用（序列）
        sample.driftComboCounter     = state->driftComboCounter;
        sample.lateralUtil = state->lateralUtil;

        // —— 你新增到 BasiclineSample 的字段（序列化）——
        sample.ePsi              = state->ePsi;
        sample.eY                = state->eY;
        sample.borderMarginRatio = state->borderMarginRatio;
        sample.vAlongTrack       = state->vAlongTrack;

        // 奖励的序列版本（按需选择：此处直接带入当前值）
        sample.stepReward  = state->stepReward;
        sample.totalReward = state->totalReward; // 若希望“该时刻累计”，保持一致即可

        // 推入环形缓冲
        state->basiclinePush(sample);
    }

    // （可选）图形坐标
    #if 0
    auto bodyGM = getGraphicsOffsetMatrix();
    state->graphicsMatrix = (bodyGM);
    state->graphicsPos = {bodyGM.M41, bodyGM.M42, bodyGM.M43};
    state->graphicsEuler = (bodyGM.getEulerAngles());
    #endif
}


//=============================================================================

void Car::updateSensei()
{
	const int numTrackPoints = (int)track->fatPoints.size();
	if (!numTrackPoints)
		return;

	const int numSenseiPoints = track->interpolatedSpline->node_count();

	if ((int)senseiPoints.size() != numSenseiPoints)
		senseiPoints.resize(numSenseiPoints);

	if (splinePointId >= 0 && splinePointId < numSenseiPoints)
	{
		CarSenseiData data;

		data.controls = controls;
		data.bodyMatrix = state->bodyMatrix;
		data.worldSplinePosition = worldSplinePosition;
		data.velocity = state->velocity;
		data.localVelocity = state->localVelocity;
		data.angularVelocity = state->angularVelocity;
		data.localAngularVelocity = state->localAngularVelocity;

		data.trackPointId = nearestTrackPointId;
		data.trackLocation = trackLocation;
		data.bodyVsTrack = bodyVsTrack;
		data.velocityVsTrack = velocityVsTrack;

		data.gear = state->gear;
		data.engineRPM = state->engineRPM;
		data.speedMS = state->speedMS;

		senseiPoints[splinePointId] = data;
	}

	if (senseiLapStarted && nearestTrackPointId == numTrackPoints - 1 && oldTrackPointId == numTrackPoints - 2)
	{
		senseiLapStarted = false;
		track->senseiPoints.resize(senseiPoints.size());
		memcpy(track->senseiPoints.data(), senseiPoints.data(), senseiPoints.size() * sizeof(senseiPoints[0]));
	}

	if (nearestTrackPointId == 0)
	{
		senseiLapStarted = true;
	}
}

//=============================================================================
// COLLISION
//=============================================================================

void Car::onCollisionCallback(
	void* userData0, void* shape0, 
	void* userData1, void* shape1, 
	const vec3f& normal, const vec3f& pos, float depth)
{
	if (!(body.get() == userData0 || body.get() == userData1))
		return;

	IRigidBody* pOtherBody;
	ICollisionObject* pOtherShape;

	if (body.get() == userData0)
	{
		pOtherBody = (IRigidBody*)userData1;
		pOtherShape = (ICollisionObject*)shape1;
	}
	else
	{
		pOtherBody = (IRigidBody*)userData0;
		pOtherShape = (ICollisionObject*)shape0;
	}

	unsigned long ulOtherGroup = pOtherShape ? pOtherShape->getGroup() : 0;
	unsigned long ulGroup0 = 0;
	unsigned long ulGroup1 = 0;
	bool bFlag0 = false;
	bool bFlag1 = false;

	if (shape0)
	{
		ulGroup0 = ((ICollisionObject*)shape0)->getGroup();
		bFlag0 = (ulGroup0 == 1 || ulGroup0 == 16);
	}

	if (shape1)
	{
		ulGroup1 = ((ICollisionObject*)shape1)->getGroup();
		bFlag1 = (ulGroup1 == 1 || ulGroup1 == 16);
	}

	lastCollisionTime = sim->physicsTime;
	collisionFlag = true;

	vec3f vPosLocal = body->worldToLocal(pos);
	vec3f vVelocity = body->getPointVelocity(pos);

	vec3f vOtherVelocity(0, 0, 0);
	if (pOtherBody)
	{
		vOtherVelocity = pOtherBody->getPointVelocity(pos);
	}

	vec3f vDeltaVelocity = vVelocity - vOtherVelocity;
	float fRelativeSpeed = -((vDeltaVelocity * normal) * 3.6f);
	float fDamage = fRelativeSpeed * sim->mechanicalDamageRate;

	if (userData0 && userData1 && !bFlag0 && !bFlag1)
		lastCollisionWithCarTime = sim->physicsTime;

	float* pDmg = damageZoneLevel;

	if (fRelativeSpeed > 0.0f && !bFlag0 && !bFlag1)
	{
		if (fRelativeSpeed * sim->mechanicalDamageRate > 150.0f)
			drivetrain->engineModel->blowUp();

		vec3f vNorm = vPosLocal.get_norm();
		int iZoneId;

		if (fabsf(vNorm.z) <= 0.70700002f)
		{
			iZoneId = (vPosLocal.x >= 0.0f) ? 2 : 3;
		}
		else
		{
			iZoneId = (vPosLocal.z <= 0.0f) ? 1 : 0;
		}
		
		pDmg[iZoneId] = tmax(pDmg[iZoneId], fDamage);
		pDmg[4] = tmax(pDmg[4], fDamage);
	}

	#if 0
	if (pDmg[0] > 0.0f && pDmg[2] > 0.0f)
		suspensions[0]->setDamage((pDmg[0] + pDmg[2]) * 0.5f);

	if (pDmg[0] > 0.0f && pDmg[3] > 0.0f)
		suspensions[1]->setDamage((pDmg[0] + pDmg[3]) * 0.5f);

	if (pDmg[1] > 0.0f && pDmg[2] > 0.0f)
		suspensions[2]->setDamage((pDmg[1] + pDmg[2]) * 0.5f);

	if (pDmg[1] > 0.0f && pDmg[3] > 0.0f)
		suspensions[3]->setDamage((pDmg[1] + pDmg[3]) * 0.5f);
	#endif

	#if 0
	ACPhysicsEvent pe;
	pe.type = eACEventType::acEvent_OnCollision;
	pe.param1 = (float)physicsGUID;
	pe.param2 = depth;
	pe.param3 = -1;
	pe.param4 = fRelativeSpeed;
	pe.vParam1 = pos;
	pe.vParam2 = normal;
	pe.voidParam0 = nullptr;
	pe.voidParam1 = nullptr;
	pe.ulParam0 = ulOtherGroup; // TODO: ulGroup1 OR ulOtherGroup ?
	sim->eventQueue.push(pe); // processed by Sim::stepPhysicsEvent
	#endif

	if (fRelativeSpeed > 0.0f && !bFlag0 && !bFlag1)
	{
		//log_printf(L"collision %u %u %.3f", (UINT)ulGroup0, (UINT)ulGroup1, fRelativeSpeed);

		OnCollisionEvent ce;
		ce.body = pOtherBody;
		ce.relativeSpeed = fRelativeSpeed;
		ce.worldPos = pos;
		ce.relPos = vPosLocal;
		ce.colliderGroup = ulOtherGroup; // TODO: ulGroup1 OR ulOtherGroup ?
		evOnCollisionEvent.fire(ce);
	}
}

//=============================================================================
// CONTROLS
//=============================================================================

void Car::pollControls(float dt)
{
	if (lockControls)
	{
		controls.clutch = 0;
		controls.brake = 1;
		controls.gas = 0;
		return;
	}

	if (externalControls) // controls managed by external system
		return;

	if (!controlsProvider)
		return;

	CarControlsInput input = {steerLock, speed.value};
	controlsProvider->acquireControls(&input, &controls, dt);

	float fSpeedN = tclamp(speed.value, 0.0f, 1.0f);

	vibrationPhase += speed.value * dt;
	slipVibrationPhase += dt;

	float fVibSum = 0;
	float fMaxGain = 0;
	float fMaxSlip = 0;
	int iSurfCount = 0;

	for (int i = 0; i < 4; ++i)
	{
		auto* pSurf = tyres[i]->surfaceDef;
		if (pSurf)
		{
			fMaxGain = tmax(fMaxGain, pSurf->vibrationGain);
			if (pSurf->vibrationLength != 0.0f && pSurf->vibrationGain != 0.0f)
			{
				fVibSum += pSurf->vibrationLength;
				iSurfCount++;
			}
		}
		float fSlip = tyres[i]->status.ndSlip * 0.75f;
		fMaxSlip = tmax(fMaxSlip, fSlip);
	}

	if (!iSurfCount)
	{
		//SHOULD_NOT_REACH_WARN;
		return;
	}

	if (fMaxSlip <= 1.0f)
		fMaxSlip *= fMaxSlip;

	VibrationDef vd;
	float fVibAvg = fVibSum / (float)iSurfCount;

	if (fVibAvg != 0.0f && fMaxGain != 0.0f)
	{
		float fLoad0 = tclamp(tyres[0]->status.load, 0.0f, 1.0f);
		float fLoad1 = tclamp(tyres[1]->status.load, 0.0f, 1.0f);
		vd.curbs = (((sawToothWave(vibrationPhase, fVibAvg) * fLoad0) * fLoad1) * fMaxGain) * fSpeedN;
	}

	float fPhase30 = sinf(vibrationPhase * 30.0f);
	vd.gforce = tclamp(fabsf(accG.y), 0.0f, 1.0f) * fPhase30;

	float fPhase120 = sinf(vibrationPhase * 120.0f);
	vd.slips = tclamp(fMaxSlip * 0.4f, 0.0f, 1.0f) * fPhase120;

	float v36 = ((float)drivetrain->engine.velocity * 0.15915507f) * 60.0f;
	float v37 = v36 / drivetrain->engineModel->getLimiterRPM();
	vd.engine = tclamp(v37, 0.0f, 1.0f);

	#if 0
	if (abs.isPresent && abs->isInAction())
		vd.abs = ksSquareWave(pksPhysics->physicsTime, 100.0f);
	else
		vd.abs = 0;
	#endif
	
	// TODO: WTF?
	vd.curbs *= fSpeedN;
	vd.slips *= fSpeedN;
	vd.abs *= fSpeedN;
	lastVibr = vd;

	controlsProvider->setVibrations(&vd);
}

void Car::sendFF(float dt)
{
	if (controlsProvider)
	{
		lastFF = getSteerFF(dt);

		#if 1
		float fLspSpeed = sim->mzLowSpeedReduction.speedKMH;
		float fLspMin = sim->mzLowSpeedReduction.minValue;

		if (fLspSpeed != 0.0f)
		{
			float fReduct = tclamp((speed.value * 3.6f) / fLspSpeed, 0.0f, 1.0f);
			lastFF *= (((1.0f - fLspMin) * fReduct) + fLspMin);
		}
		#endif

		// TODO: mess here!!!
		float fVar4 = tclamp((1.0f - (speed.value * 3.6f * 0.1f)), 0.0f, 1.0f);
		float dmin = sim->ffDamperMinValue;
		float d = ((1.0f - dmin) * fVar4 + dmin) * sim->ffDamperGain;
		lastDamp = d;

		controlsProvider->sendFF(lastFF, d, userFFGain);
	}
	else
	{
		mzCurrent = 0;
	}
}

float Car::getSteerFF(float dt)
{
	float fTorq0 = suspensions[0]->getSteerTorque();
	float fTorq1 = suspensions[1]->getSteerTorque();
	float fTorqFront = fTorq0 + fTorq1;

	float fSteerPos = controls.steer * steerLock;
	float fSteerDelta = (fSteerPos - lastSteerPosition) * 333.33334f;
	float fMz = ((mzCurrent - fTorqFront) * controlsProvider->getFeedbackFilter()) + fTorqFront;
	mzCurrent = fMz;

	float fAv0 = fabsf(tyres[0]->status.angularVelocity);
	float fAv1 = fabsf(tyres[1]->status.angularVelocity);
	float fAvFront = fAv0 + fAv1;

	float fGyroFF = ((fSteerDelta / fabsf(steerRatio))
		* (fAvFront * tyres[0]->data.angularInertia))
		* sim->ffGyroWheelGain;

	lastSteerPosition = fSteerPos;
	lastPureMZFF = fMz * 1.4f;

	float fSignal = (-(fMz + fGyroFF)) * 1.4f;

	float fBlisterF = (float)tmax(tyres[0]->status.blister, tyres[1]->status.blister);
	float fBlisterR = (float)tmax(tyres[2]->status.blister, tyres[3]->status.blister);
	float fBlister = tmax(fBlisterF, fBlisterR) * 0.003f;

	float fFlatSpot = (float)tmax(tyres[0]->status.flatSpot, tyres[1]->status.flatSpot);
	if (fFlatSpot < fBlister)
		fFlatSpot = fBlister;

	flatSpotPhase += fAvFront * dt;

	if (fAvFront > 7.0f)
	{
		float fWave = sawToothWave(flatSpotPhase, 12.56636f) * fFlatSpot;
		float fLoad = tyres[0]->status.load + tyres[1]->status.load;
		fSignal += ((fWave * sim->ffFlatSpotGain * fLoad * 0.5f) + 1.0f);
	}

	float fSteerFF;
	if (steerAssist == 1.0f)
	{
		fSteerFF = fSignal * ffMult;
	}
	else
	{
		float fAssist = powf(fabsf(fSignal * ffMult), steerAssist);
		fSteerFF = fAssist * signf(fSignal);
	}

	float fResult = -fSteerFF;
	lastGyroFF = fGyroFF * 1.4f;

	#if 0
	if (controlsProvider->useFakeUndersteerFF)
	{
		// TODO
	}
	#endif

	return fResult;
}

//=============================================================================
// UTILS
//=============================================================================

static inline double rpm_to_radps(double rpm) {
    return rpm * (2.0 * M_PI) / 60.0;
}
//新的定位函数
// ========= 工具函数（与项目向量/矩阵类型保持一致） =========
static inline vec3f vnormalize(const vec3f& v) {
    float L = sqrtf(v.x*v.x + v.y*v.y + v.z*v.z);
    return (L > 1e-8f) ? vec3f(v.x/L, v.y/L, v.z/L) : vec3f(0,0,0);
}
static inline float vdot(const vec3f& a, const vec3f& b) {
    return a.x*b.x + a.y*b.y + a.z*b.z;
}
static inline vec3f vcross(const vec3f& a, const vec3f& b) {
    return vec3f(
        a.y*b.z - a.z*b.y,
        a.z*b.x - a.x*b.z,
        a.x*b.y - a.y*b.x
    );
}
// 罗德里格旋转：把向量 v 围绕单位轴 n 旋转 angle（弧度）
static inline vec3f rotateAroundAxis(const vec3f& v, const vec3f& n_unit, float angle) {
    float c = cosf(angle), s = sinf(angle);
    return v * c + vcross(n_unit, v) * s + n_unit * (vdot(n_unit, v) * (1.0f - c));
}

// ========= 新的 forcePosition（绝对放置） =========
// 直接把车身放到 (x,y,z)。不再做射线/基准高度/偏置；默认清零速度并重挂悬挂。
void Car::applyWorldMatricesFast(
    const mat44f& bodyMatrix,
    const mat44f& fuelTankMatrix,
    const mat44f& hubFLMatrix,
    const mat44f& hubFRMatrix,
	const mat44f& strutBodyFLMatrix,
    const mat44f& strutBodyFRMatrix,
    const mat44f& axleOrRearMatrix,
    bool zeroVel /*= false*/,   // 如需清零速度
    bool reattach /*= true*/    // 是否重挂悬挂关节
)
{
    auto setPose = [](IRigidBody* rb, const mat44f& m){
        if (!rb) return;
        rb->setRotation(m);
        rb->setPosition(vec3f(m.M41, m.M42, m.M43));
    };

    if (zeroVel) { 
		body->stop(); 
		fuelTankBody->stop();
		if (rigidAxle)
			rigidAxle->stop();}

    // 1) 车身与油箱
    setPose(body.get(),        bodyMatrix);
    setPose(fuelTankBody.get(), fuelTankMatrix);

    // 2) 前悬（假设 0:FL, 1:FR 为 Strut；你的项目中这两位常为 Strut）
    if (auto* suspFL = dynamic_cast<SuspensionStrut*>(suspensions[0])) {
		suspFL->hub->stop();
		suspFL->strutBody->stop();
        setPose(suspFL->hub.get(), hubFLMatrix);
        setPose(suspFL->strutBody.get(),strutBodyFLMatrix); // 若需要杆件同向（可选）
    }
    if (auto* suspFR = dynamic_cast<SuspensionStrut*>(suspensions[1])) {
		suspFR->hub->stop();
		suspFR->strutBody->stop();
        setPose(suspFR->hub.get(),  hubFRMatrix);
        setPose(suspFR->strutBody.get(),strutBodyFRMatrix); // 若需要杆件同向（可选）
    }
    // 3) 后桥（若为刚性桥 AXLE）
    if (auto* suspRearL = dynamic_cast<SuspensionAxle*>(suspensions[2])) {
		suspRearL->axle->stop();
        setPose(suspRearL->axle.get(), axleOrRearMatrix);
    }
    // 若后端不是 AXLE（如 DWB/ML），你可以把两个后 hub 的矩阵传进来，分别 setPose()

    if (reattach) {
        for (auto& susp : suspensions) { susp->stop();}
    }

	reset();

	drivetrain->reset();
	brakeSystem->reset();

	for (int i = 0; i < 4; ++i)
		tyres[i]->reset();

}

void Car::newforcePosition(const vec3f& pos, bool zeroVel /*=true*/, bool reattach /*=true*/,const vec3f& hubflpos,const vec3f& hubfrpos,const vec3f& axlepos)
{
    if (zeroVel) { body->stop(); fuelTankBody->stop(); }
	reset();
	body->stop(); // 线/角速度清零
    body->setPosition(pos);
    fuelTankBody->setPosition(body->localToWorld(fuelTankPos));

    if (reattach) {
        for (auto& susp : suspensions) { susp->stop();}
    }

	drivetrain->reset();
	brakeSystem->reset();

	for (int i = 0; i < 4; ++i)
		tyres[i]->reset();
		
	for (size_t i = 0; i < 2; i++)
	{
		SuspensionStrut* susp = dynamic_cast<SuspensionStrut*>(suspensions[i]);
		if (susp) {
			if (i == 0)
			{
				susp->hub->setPosition(hubflpos);
			}
			else
			{
				susp->hub->setPosition(hubfrpos);
			}
		}
	}

	for (size_t i = 2; i < 4; i++)
	{  
		SuspensionAxle* susp = dynamic_cast<SuspensionAxle*>(suspensions[i]);
		if (susp) {
			susp->axle->setPosition(axlepos);
		}
	}


    if (zeroVel) { body->stop(); fuelTankBody->stop(); }
}

// 构造旋转矩阵：Yaw(heading), Pitch, Roll
mat44f makeRotationYPR(float yaw, float pitch, float roll) {
    float cy = cosf(yaw),  sy = sinf(yaw);
    float cp = cosf(pitch),sp = sinf(pitch);
    float cr = cosf(roll), sr = sinf(roll);

    mat44f m{};
    // yaw->pitch->roll (Y->X->Z)
    m.M11 = cy*cr + sy*sp*sr;  m.M12 = sr*cp;   m.M13 = -sy*cr + cy*sp*sr;
    m.M21 = -cy*sr + sy*sp*cr; m.M22 = cr*cp;   m.M23 = sr*sy + cy*sp*cr;
    m.M31 = sy*cp;             m.M32 = -sp;     m.M33 = cy*cp;
    // 填写齐次部分
    m.M14 = m.M24 = m.M34 = m.M41 = m.M42 = m.M43 = 0.0f;
    m.M44 = 1.0f;
    return m;
}

void Car::newforceRotation(float heading, float roll, float pitch,
                        bool zeroVel, bool reattach) {
    mat44f R = makeRotationYPR(heading, pitch, roll);
    if (zeroVel) { body->stop(); fuelTankBody->stop(); }

    body->setRotation(R);
    fuelTankBody->setRotation(R);

    if (reattach) {
        for (auto& susp : suspensions) {susp->attach();}
    }

	for (size_t i = 0; i < 2; i++)
	{
		SuspensionStrut* susp = dynamic_cast<SuspensionStrut*>(suspensions[i]);
		if (susp) {
			susp->strutBody->setRotation(R);
		}
	}

	for (size_t i = 2; i < 4; i++)
	{  
		SuspensionAxle* susp = dynamic_cast<SuspensionAxle*>(suspensions[i]);
		if (susp) {
			susp->axle->setRotation(R);
		}
	}

    if (zeroVel) { 
		body->stop(); 
		fuelTankBody->stop();
	} 
}

// 工具函数：计算向量长度
inline float vlength(const vec3f& v) {
    return std::sqrt(v.x*v.x + v.y*v.y + v.z*v.z);
}
// 热启动
bool Car::hotstart(bool hotstarttag,
    double newrootVelocity, double newengineRPM, int newcurrentGear,
    double newoutShaftLvelocity, double newoutShaftRvelocity, float newbraketemp,
    const std::array<float,3>& newVelocity,
    const std::array<float,4>& newslipAngleRAD, const std::array<float,4>& newslipRatio,
    const std::array<float,4>& newangularVelocity, const std::array<float,4>& newangularVelocityold, 
	const std::array<float,4>& newMz,
    const std::array<float,4>& newdirtyLevel, const std::array<float,4>& newcoretemp,
    const std::array<float,4>& newpatchtemp)
{
    if (hotstarttag) return false;
    if (!drivetrain)  return false;

    // rpm -> rad/s
    auto rpm_to_rad = [](double rpm)->double { return rpm * (2.0 * M_PI / 60.0); };

    // —— 1) 传动系 / 根速度 —— //
    drivetrain->rootVelocity        = static_cast<float>(newrootVelocity);        // m/s
    drivetrain->engine.velocity     = static_cast<float>(rpm_to_rad(newengineRPM)); // rad/s
    drivetrain->outShaftL.velocity  = static_cast<float>(newoutShaftLvelocity);   // rad/s
    drivetrain->outShaftR.velocity  = static_cast<float>(newoutShaftRvelocity);   // rad/s
    drivetrain->drive.velocity      = 0.5f * (drivetrain->outShaftL.velocity + drivetrain->outShaftR.velocity);  //注意这里
	drivetrain->currentGear         = newcurrentGear;
    // —— 2) 轮胎状态 + 胎温（含 oldAngularVelocity）—— //
    const size_t n = std::min<size_t>(4, tyres.size());
    for (size_t i = 0; i < n; ++i) {
        Tyre* tyre = tyres[i] ? tyres[i].get() : nullptr;
        if (!tyre) continue;

        tyre->status.slipAngleRAD    = newslipAngleRAD[i];
        tyre->status.slipRatio       = newslipRatio[i];
        tyre->status.angularVelocity = newangularVelocity[i];
        tyre->oldAngularVelocity     = newangularVelocity[i];  // 关键：减少第一步能量损失
        tyre->status.Mz              = newMz[i];
        tyre->status.dirtyLevel      = newdirtyLevel[i];
		
        if (tyre->thermalModel && tyre->thermalModel->isActive) {
            const float coreC  = newcoretemp[i];   // FIX: 用实值，而不是 std::isfinite 的返回
            const float patchC = newpatchtemp[i];  // FIX: 同上
            // 可选：若需健壮性，可先判断 isfinite 再调用
            tyre->thermalModel->sethotTemperature(coreC, patchC);
        }
    }

    // —— 3) 刹车温度 —— //
    if (brakeSystem) {
        // 若有分轮接口可改为 per-wheel；此处统一设置
        brakeSystem->hotset(newbraketemp);
    }

    // —— 4) 线速度：把 std::array<float,3> 显式转为 vec3f，并判空刚体 —— //
    const vec3f v_world { newVelocity[0], newVelocity[1], newVelocity[2]};

    if (body)         body->setVelocity(v_world);
    if (fuelTankBody) fuelTankBody->setVelocity(v_world);

	for (size_t i = 0; i < 2; i++)
	{
		SuspensionStrut* susp = dynamic_cast<SuspensionStrut*>(suspensions[i]);
		if (susp) {
			susp->hub->setVelocity(v_world);
			susp->strutBody->setVelocity(v_world);
		}
	}

	for (size_t i = 2; i < 4; i++)
	{  
		SuspensionAxle* susp = dynamic_cast<SuspensionAxle*>(suspensions[i]);
		if (susp) {
			susp->axle->setVelocity(v_world);
		}
	}

    // —— 5) 取消“稳定睡眠”，避免吞初速 —— //
    //framesToSleep  = 0;
    //sleepingFrames = 0;
	
    // —— 6) 刷新派生量/缓存 —— //
	scoring->reset();
    updateCarState();
    // updateSensei();

    return true;
}

// 设定车辆速度 (单位：km/h)
void Car::setForwardSpeed(float speedKmh) {
    // 1. 换算成 m/s
    float speedMs = speedKmh / 3.6f;

    // 2. 当前车头方向 (假设局部 +Z 为前向)
    vec3f forward = body->localToWorldNormal({0, 0, 1});
    // 投影到水平面，避免有 pitch/roll 时带垂直速度
    forward.y = 0.0f;
    if (vlength(forward) > 1e-6f) {
        forward = forward / vlength(forward); // 归一化
    } else {
        forward = {0,0,1}; // fallback
    }

    // 3. 生成速度向量
    vec3f v0 = forward * speedMs;

    // 4. 应用到刚体
    body->setVelocity(v0);
    fuelTankBody->setVelocity(v0);
}

//新的定位函数

void Car::forcePosition(const vec3f& pos, float offsetY)
{
	vec3f bodyPos = pos;
	auto hit = sim->physics->rayCast(pos + vec3f(0, 10, 0), vec3f(0, -1, 0), 1000);
	if (hit.hasContact)
	{
		bodyPos.y = hit.pos.y;
	}
	// 从 (pos.x, pos.y+10, pos.z) 向下射线 1000m，若命中则把 bodyPos.y 设为地面 y。
	// 注意：一旦命中，传入的 pos.y 基本被忽略（y 由地形决定）

	bodyPos.y += (getBaseCarHeight() + offsetY + 0.01f);
	// 最终车身 y：
	// finalY = groundY + getBaseCarHeight() + offsetY + 0.01
	// getBaseCarHeight()：车体原点到“标准放置高度”的基准（通常使底盘不穿地）。
	// offsetY：你可控的额外抬升（正数抬高、负数压低）。  0.01f：微小防穿插余量。

	reset();

	body->stop(); // 线/角速度清零
	body->setPosition(bodyPos);   // 瞬移车身

	fuelTankBody->setPosition(body->localToWorld(fuelTankPos));
	//先全车 reset，再将主刚体与副刚体（油箱）放到新位置，并把速度清零避免瞬移后的“飞车”

	for (auto& susp : suspensions)
	{
		susp->stop();
		susp->attach();
	}

	drivetrain->reset();
	brakeSystem->reset();

	for (int i = 0; i < 4; ++i)
		tyres[i]->reset();

	drivetrain->setCurrentGear(1, true);

	body->stop();
	fuelTankBody->stop();
	// 悬挂重新“挂接”到当前车身姿态；
	// 传动系、制动系统、轮胎全部复位；
	// 强制 1 挡：如果你想空挡，需在调用后手动改回。
}

void Car::forceRotation(const vec3f& heading) // **作用：**把车的朝向设为 heading 指向（世界坐标），并保持世界竖直向量为 (0,1,0)，不考虑车身横滚。
{
	const vec3f ihed = heading * -1.0f;  // 右向量（right）≈ normalize(cross(forward, up))，但强制 y=0

	float vM13 = ihed.x;
	float vM11 = -ihed.z;
	float vM12 = 0;
	float v6 = sqrtf((vM12 * vM12) + (vM11 * vM11) + (vM13 * vM13));
	float s = 1.0f / v6;

	mat44f m;  // right（单位化）
	m.M11 = vM11 * s;
	m.M12 = vM12 * s;
	m.M13 = vM13 * s;

	m.M21 = 0;  // up = 世界Y
	m.M22 = 1;
	m.M23 = 0;

	m.M31 = -ihed.x; // forward = heading
	m.M32 = -ihed.y;
	m.M33 = -ihed.z;

	// 第一行 (M11, M12, M13) ← 右方向向量 r
	// 第二行 (M21, M22, M23) ← 上方向向量 up
	// 第三行 (M31, M32, M33) ← 前方向向量 f
	// r = 车体右边朝向（右手边）
	// up = 车体顶部朝向
	// f = 车体前方朝向

	// 要点与坑：
	// 没有单位化 forward：heading 最好传入单位向量（或至少不要是零向量/接近竖直），否则旋转矩阵会“非正交”。
	// 固定 up=(0,1,0)：也就是说不会给车添加横滚/俯仰，只是水平方向对齐。
	// 若 heading 几乎指向竖直方向，right 的长度会接近 0 ⇒ 归一化爆炸（1/0）。务必传入水平向的 heading（y≈0）。

	body->setRotation(m);
	fuelTankBody->setRotation(m);

	for (auto& susp : suspensions)
	{
		susp->attach();
	}

	body->stop();
	fuelTankBody->stop();
}

void Car::teleport(const mat44f& m)
{
	forceRotation(D::vec3f(&m.M31));
	forcePosition(D::vec3f(&m.M41));
}

void Car::teleportToPits(int pitId)
{
	const auto& pits = track->pits;
	if (pitId >= 0 && pitId < (int)pits.size())
	{
		teleport(pits[pitId]);
	}
}

void Car::teleportToSpline(float distanceNorm)
{
	const auto& points = track->fatPoints;
	const size_t n = points.size();
	if (n)
	{
		//int pointId = (int)(tclamp(distanceNorm, 0.0f, 1.0f) * (n - 1));
		const size_t pointId = track->getPointIdAtDistance(distanceNorm);
		if (pointId < n)
		{
			auto& pt = points[pointId];
			forceRotation(pt.forwardDir);
			forcePosition(D::vec3f(&pt.center.x));
		}
	}
}

void Car::teleportByMode(TeleportMode mode)
{
	switch (mode)
	{
		case TeleportMode::Start:
			teleportToSpline(0.0f);
			break;

		case TeleportMode::Nearest:
			teleportToSpline(trackLocation);
			break;

		case TeleportMode::Random:
			teleportToSpline(randR(0.0f, 1.0f));
			break;
	}
}

float Car::getBaseCarHeight() const
{
	float t0 = fabsf(suspensions[0]->getBasePosition().y - tyres[0]->data.rimRadius);
	float t2 = fabsf(suspensions[2]->getBasePosition().y - tyres[2]->data.rimRadius);
	return tmax(t0, t2);
}

vec3f Car::getGroundWindVector() const
{
	plane4f ground(
		tyres[0]->unmodifiedContactPoint,
		tyres[1]->unmodifiedContactPoint,
		tyres[2]->unmodifiedContactPoint);

	auto wind = sim->wind.vector;
	float dot = (wind * ground.normal);

	return (wind - (ground.normal * dot)) * 0.44f;
}

float Car::getPointGroundHeight(const vec3f& pt) const
{
	plane4f pl1(
		tyres[0]->contactPoint,
		tyres[1]->contactPoint,
		tyres[2]->contactPoint);

	plane4f pl2(
		tyres[0]->contactPoint,
		tyres[1]->contactPoint,
		tyres[3]->contactPoint);

	vec3f axis(0, -1, 0);
	float dot1 = pl1.normal * axis;
	float dot2 = pl2.normal * axis;

	float y1 = 0, y2 = 0;

	if (dot1 != 0.0f)
		y1 = pt.y + ((pt * pl1.normal + pl1.d) / dot1);

	if (dot2 != 0.0f)
		y2 = pt.y + ((pt * pl2.normal + pl2.d) / dot2);

	return ((pt.y - y2) + (pt.y - y1)) * 0.5f;
}

mat44f Car::getGraphicsOffsetMatrix() const
{
	auto m = body->getWorldMatrix(0);
	auto off = graphicsOffset * m;

	auto gm = m;
	gm.M41 = off.x;
	gm.M42 = off.y;
	gm.M43 = off.z;

	if (graphicsPitchRotation != 0.0f)
	{
		auto rotator = mat44f::createFromAxisAngle(vec3f(1, 0, 0), graphicsPitchRotation);
		gm = mat44f::mult(rotator, gm);
	}

	return gm;
}

bool Car::isSleeping() const
{
	return sleepingFrames > framesToSleep;
}

float Car::getEngineRpm() const
{
	return ((float)drivetrain->engine.velocity * 0.15915507f * 60.0f);
}

inline float getLoad(const Car* car, int tyreId)
{
	auto* pTyre = car->tyres[tyreId].get();
	float fDX = pTyre->getDX(pTyre->status.load);
	float fD = pTyre->getCorrectedD(fDX, nullptr);
	return fD * pTyre->status.load;
}

float Car::getOptimalBrake() const
{
	float fLoad0 = getLoad(this, 0);
	float fLoad1 = getLoad(this, 1);
	float fLoad2 = getLoad(this, 2);
	float fLoad3 = getLoad(this, 3);

	float fLoadFront = (fLoad0 + fLoad1) * 0.5f * tyres[0]->status.loadedRadius;
	float fLoadRear = (fLoad2 + fLoad3) * 0.5f * tyres[2]->status.loadedRadius;

	float fBrakeFront = fLoadFront / (brakeSystem->brakePower * brakeSystem->frontBias);
	float fBrakeRear = fLoadRear / (brakeSystem->brakePower * (1.0f - brakeSystem->frontBias));

	return tmin(fBrakeFront, fBrakeRear);
}

float Car::getDrivingTyresSlip() const
{
	if (drivetrain->tractionType == TractionType::FWD)
	{
		return tmax(tyres[0]->status.ndSlip, tyres[1]->status.ndSlip);
	}
	else
	{
		return tmax(tyres[2]->status.ndSlip, tyres[3]->status.ndSlip);
	}
}

float Car::getBetaRad() const // TODO: check
{
	auto vel = body->getLocalVelocity();

	float vlong = vel.z;
    float vlat  = vel.x;
	float speed = sqrt(vlong*vlong + vlat*vlat);
	const float EPS = 0.1f;

	if (speed < EPS) {
        return 0.0f; // 或者返回上一次有效beta，或标记为无效
    }
	
	float fLen = vel.len();
	if (fLen != 0.0f)
		vel.x /= fLen;

	if (vel.x <= -1.0f || vel.x >= 1.0f)
		return 1.5707964f;

	return asinf(vel.x);
}

}