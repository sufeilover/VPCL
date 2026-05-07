---@ext:basic

local BTN_F0 = const(ui.ButtonFlags.PressedOnClick)
local BTN_FA = const(bit.bor(ui.ButtonFlags.Active, ui.ButtonFlags.PressedOnClick))
local BTN_FN = const(function (active) return active and BTN_FA or BTN_F0 end)

local car = ac.getCar(0) or error()
local sim = ac.getSim()


local weightShift = not sim.isVRConnected and ac.DriverWeightShift(0) or nil

local anyExtraHint = false

local PHYS_DT = 2.9
local COVER_FRAMES = 10

local lastTs = nil
local overrideAcc = 0.0
local coverSteps = 0

local lastTs2 = nil
local override2Acc = 0.0
local override2Index = 1

-- freeze baseline for mode2 window
local baseGas2 = 0.0
local baseBrake2 = 0.0
local baseSteer2 = 0.0
local baseCaptured2 = false

-- 你的 StructItem layout（保持你已经跑通的那套）
local layout = {
  seq   = ac.StructItem.uint32(),
  steer = ac.StructItem.float(),
  gas   = ac.StructItem.float(),
  brake = ac.StructItem.float()
}

local layout2 = {
  seq   = ac.StructItem.uint32(),
  steer = ac.StructItem.float(),
  gas   = ac.StructItem.float(),
  brake = ac.StructItem.float(),
}

local shm = nil
local shmErr = nil
local connectAcc = 0.0

local lastSeq = -1
local lastSteer, lastGas, lastBrake = 0.0, 0.0, 0.0
local lastNode0 = { seq = -1, steer = 0.0, gas = 0.0, brake = 0.0 }
local overrideactive = false
local connected = false
local timeSinceUpdate = 0.0

-- ===== UnboundExt2 (batch) =====
-- Layout of UnboundExt2 (as written by Python batch writer):
-- Header: magic(4s='UB2B'), seq(u32), group(u32), reserved(u32)
-- Payload: group frames (steer, gas, brake) as float32; we map first FIXED_GROUP frames.
-- UnboundExt2 batch is 16 + 64*12 = 784 bytes on writer side
-- ===== UnboundExt2 SIMPLE (seq + 20 frames) =====
local FIXED_GROUP2 = 10

-- ===== UnboundExt2 (u0/u1 nodes only) =====
-- New layout written by Python (28 bytes):
--   seq(u32), s0,f0 g0,f0 b0,f0, s1,f1 g1,f1 b1,f1  (all float32 little-endian)
-- We expand u0->u1 in Lua into FIXED_GROUP2 frames, with slope limiting and gas/brake mutual exclusivity.

local MAX_STEP = 0.01

-- 冻结的基线（进入 override 时采样一次）
local baseGas = 0.0
local baseBrake = 0.0
local baseSteer = 0.0
local baseCaptured = false

local function beginOverride10Freeze(car)
  overrideAcc = 0.0
  coverSteps = 0

  -- 只采样一次当前输入作为基线（冻结）
  baseGas = car.gas or 0.0
  baseBrake = car.brake or 0.0
  baseSteer = (car.steer or 0.0) / 450.0
  baseCaptured = true
end

local function beginOverride10()
  overrideAcc = 0.0
  coverSteps = 0
end

local function beginOverride2(car)
  override2Acc = 0.0
  override2Index = 1

  -- freeze baseline once
  baseGas2 = car.gas or 0.0
  baseBrake2 = car.brake or 0.0
  baseSteer2 = (car.steer or 0.0) / 450.0
  baseCaptured2 = true
end

local function _mutual_exclusive_gb(g, b, thr)
  thr = thr or 0.05
  g = g or 0.0
  b = b or 0.0
  if g > thr and b > thr then
    if g >= b then
      b = 0.0
    else
      g = 0.0
    end
  end
  return g, b
end

local function _clip(x, lo, hi)
  if x < lo then return lo end
  if x > hi then return hi end
  return x
end

local function _sign(x)
  return x >= 0 and 1 or -1
end

-- Expand between u0=(s0,g0,b0) and u1=(s1,g1,b1) into `group` frames (u0->u1, not including u1).
local function expand_u0u1_segment(s0, g0, b0, s1, g1, b1, group)
  group = tonumber(group) or 1
  if group < 1 then group = 1 end

  -- clip nodes
  s0 = _clip(tonumber(s0) or 0.0, -1.0, 1.0)
  s1 = _clip(tonumber(s1) or 0.0, -1.0, 1.0)
  g0 = _clip(tonumber(g0) or 0.0,  0.0, 1.0)
  g1 = _clip(tonumber(g1) or 0.0,  0.0, 1.0)
  b0 = _clip(tonumber(b0) or 0.0,  0.0, 1.0)
  b1 = _clip(tonumber(b1) or 0.0,  0.0, 1.0)

  local frames = {}

  -- initial linear interp (t=i/group => [0,1), avoid duplicating u1)
  for i = 0, group - 1 do
    local t = i / group
    local s = (1.0 - t) * s0 + t * s1
    local g = (1.0 - t) * g0 + t * g1
    local b = (1.0 - t) * b0 + t * b1
    frames[#frames + 1] = { steer = s, gas = g, brake = b }
  end

  -- slope limit + exclusivity (like Python)
  for i = 2, #frames do
    local prev = frames[i - 1]
    local cur = frames[i]

    -- steer slope
    local d = cur.steer - prev.steer
    if math.abs(d) > MAX_STEP then
      cur.steer = prev.steer + MAX_STEP * _sign(d)
    end

    -- gas slope
    local dg = cur.gas - prev.gas
    if math.abs(dg) > MAX_STEP then
      cur.gas = prev.gas + MAX_STEP * _sign(dg)
    end

    -- brake slope
    local db = cur.brake - prev.brake
    if math.abs(db) > MAX_STEP then
      cur.brake = prev.brake + MAX_STEP * _sign(db)
    end

    cur.gas, cur.brake = _mutual_exclusive_gb(cur.gas, cur.brake, 0.05)

    -- final clip
    cur.steer = _clip(cur.steer, -1.0, 1.0)
    cur.gas   = _clip(cur.gas,    0.0, 1.0)
    cur.brake = _clip(cur.brake,  0.0, 1.0)
  end

  -- also clip first element
  if #frames >= 1 then
    frames[1].steer = _clip(frames[1].steer, -1.0, 1.0)
    frames[1].gas   = _clip(frames[1].gas,    0.0, 1.0)
    frames[1].brake = _clip(frames[1].brake,  0.0, 1.0)
    frames[1].gas, frames[1].brake = _mutual_exclusive_gb(frames[1].gas, frames[1].brake, 0.05)
  end

  return frames
end

local shm2 = nil
local shm2Err = nil
local connectAcc2 = 0.0
local pp1 = 1
local lastSeq2 = -1
local lastFrames2 = {}
local lastGroup2 = 0
local overrideactive2 = false
local connected2 = false
local timeSinceUpdate2 = 0.0

-- ===== Unbound2 (3rd node, for expansion only) =====
local shm3 = nil
local shm3Err = nil
local connectAcc3 = 0.0
local lastSeq3 = -1
local lastNode2 = nil
local connected3 = false
local timeSinceUpdate3 = 0.0

-- mode2 playback state (optional): step through stored frames when overrideactive2 is true
local override2Index = 1
local override2Acc = 0.0
local OVERRIDE2_STEP_DT = 0.003  -- 333 Hz step (match your sim dt)

local function tryConnectSHM2()
  if type(ac.readMemoryMappedFile) ~= 'function' then
    shm2Err = 'ac.readMemoryMappedFile not available'
    return false
  end
  local ok, ret = pcall(ac.readMemoryMappedFile, 'Unbound1', layout, true)
  if not (ok and ret) then
    ok, ret = pcall(ac.readMemoryMappedFile, 'Unbound123', layout, true)  -- fallback
  end
  if ok and ret then
    shm2 = ret
    shm2Err = nil
    return true
  else
    shm2 = nil
    shm2Err = tostring(ret)
    return false
  end
end


local function tryConnectSHM3()
  if type(ac.readMemoryMappedFile) ~= 'function' then
    shm3Err = 'ac.readMemoryMappedFile not available'
    return false
  end
  -- try new name first
  local ok, ret = pcall(ac.readMemoryMappedFile, 'Unbound2', layout, true)
  if ok and ret then
    shm3 = ret
    shm3Err = nil
    return true
  else
    shm3 = nil
    shm3Err = tostring(ret)
    return false
  end
end

local function shmTick3(dt, car)
  if not shm3 then
    connectAcc3 = connectAcc3 + dt
    if connectAcc3 >= 0.5 then
      connectAcc3 = 0.0
      tryConnectSHM3()
    end
    connected3 = false
    return
  end

  local seq = shm3.seq or 0
  if seq ~= lastSeq3 then
    lastSeq3 = seq
    lastNode2 = {
      seq = seq,
      steer = shm3.steer or 0.0,
      gas   = shm3.gas   or 0.0,
      brake = shm3.brake or 0.0
    }
    connected3 = true
    timeSinceUpdate3 = 0.0
  else
    timeSinceUpdate3 = timeSinceUpdate3 + dt
    if timeSinceUpdate3 > 0.2 then
      connected3 = false
    end
  end
end

local function shmTick2(dt, car)
  if not shm2 then
    connectAcc2 = connectAcc2 + dt
    if connectAcc2 >= 0.5 then
      connectAcc2 = 0.0
      tryConnectSHM2()
    end
    connected2 = false
    return
  end
  local seq = shm2.seq or 0
  
  if seq ~= lastSeq2 then
    lastSeq2 = seq

    -- node u0 comes from UnboundExt (shmTick ran earlier)
    local s0 = (lastNode0 and lastNode0.steer) or 0.0
    local g0 = (lastNode0 and lastNode0.gas) or 0.0
    local b0 = (lastNode0 and lastNode0.brake) or 0.0

    -- node u1 comes from UnboundExt2 (same 16B layout: seq/steer/gas/brake)
    local s1 = shm2.steer or 0.0
    local g1 = shm2.gas   or 0.0
    local b1 = shm2.brake or 0.0

    -- optional: require seq alignment; if not aligned, keep previous frames
    -- if lastNode0 and lastNode0.seq ~= seq then return end
    -- expand u0->u1 (+ u2 assist) into frames:
    --  - segment0: u0->u1 (u0 included, u1 excluded)
    --  - segment1: u1->u2 (u1 included, u2 excluded)  <-- u1 is in sequence, u2 is NOT
    if connected3 and lastNode2 then
      local s2 = lastNode2.steer or 0.0
      local g2 = lastNode2.gas   or 0.0
      local b2 = lastNode2.brake or 0.0

      local seg01 = expand_u0u1_segment(s0, g0, b0, s1, g1, b1, FIXED_GROUP2)
      local seg12 = expand_u0u1_segment(s1, g1, b1, s2, g2, b2, FIXED_GROUP2)

      lastFrames2 = seg01
      for i = 1, #seg12 do
        lastFrames2[#lastFrames2 + 1] = seg12[i]
      end
    else
      -- fallback: old behavior (u0 included, u1 excluded)
      lastFrames2 = expand_u0u1_segment(s0, g0, b0, s1, g1, b1, FIXED_GROUP2)
    end
    lastGroup2 = #lastFrames2


    connected2 = true
    timeSinceUpdate2 = 0.0
    overrideactive2 = true
    beginOverride2(car)

    -- reset playback cursor
    override2Index = 1
    override2Acc = 0.0
  else
    timeSinceUpdate2 = timeSinceUpdate2 + dt
    if timeSinceUpdate2 > 0.2 then
      connected2 = false
    end
  end
end


local function clamp(x, lo, hi)
  if x < lo then return lo end
  if x > hi then return hi end
  return x
end

local function tryConnectSHM()
  if type(ac.readMemoryMappedFile) ~= 'function' then
    shmErr = 'ac.readMemoryMappedFile not available in this context'
    return false
  end
  local ok, ret = pcall(ac.readMemoryMappedFile, 'UnboundExt', layout, true)
  if ok and ret then
    shm = ret
    shmErr = nil
    return true
  else
    shm = nil
    shmErr = tostring(ret)
    return false
  end
end

-- 在你的每帧函数里调用这个 tick
local function shmTick(dt, car)
  -- 还没连上：每 0.5s 重试一次连接（避免每帧报错刷屏）
  if not shm then
    connectAcc = connectAcc + dt
    if connectAcc >= 0.5 then
      connectAcc = 0.0
      tryConnectSHM()
    end
    connected = false
    return
  end

  -- 已连上：读数据
  local seq = shm.seq or 0
  if seq ~= lastSeq then
    lastSeq = seq
    lastSteer = shm.steer or 0.0
    lastGas   = shm.gas   or 0.0
    lastBrake = shm.brake or 0.0
    lastNode0 = { seq = seq, steer = lastSteer, gas = lastGas, brake = lastBrake }
    connected = true
    timeSinceUpdate = 0.0
    overrideactive=true
    beginOverride10()
    beginOverride10Freeze(car)
  else
    timeSinceUpdate = timeSinceUpdate + dt
    -- 超时断开（例如 0.2s 没更新，认为 Python 停了或卡了）
    if timeSinceUpdate > 0.2 then
      connected = false
    end
  end
end

--设置固定模式驾驶行为的参数
local fixedPathActive = false      -- 是否启用固定路径模式
local pathStage = 0                -- 当前状态阶段
local initialDistance = 0          -- 记录进入固定路径时的里程数据（以车圈内距离为依据）
local currentSteerAngle = 0        -- 当前的方向盘角度，单位以度表示
local deltaAngle = 0.01               -- 每帧变化的角度步长（单位：度）

-- 辅助函数：判断是否达到目标行驶距离
--local function reachedDistance(targetDistance)
--  return (car.lapDistance - initialDistance) >= targetDistance
--end

local extraHints = table.range(6, function (index)
  local r = ac.getExtraSwitchName(index - 1) or false
  if r then anyExtraHint = true end
  return r
end)

if not ac.getCarGearLabel then  --获取挡位
  ac.getCarGearLabel = function (index)
    local gear = index == 0 and car.gear or ac.getCar(index).gear
    return gear < 0 and 'R' or gear == 0 and 'N' or tostring(gear)
  end
end

local controlsConfig = ac.INIConfig.controlsConfig()
local controlsBindings = {}

local function bindingInfoGen(section)
  local pieces = section:split(';')
  if #pieces > 1 then
    local r = {}
    for _, v in ipairs(pieces) do
      local p = string.split(v, ':', 2, true)
      local i = bindingInfoGen(p[2])
      if string.regfind(i, '^(?:Not |Keyboard:|Gamepad:)') then i = i:sub(1, 1):lower()..i:sub(2) end
      r[#r + 1] = p[1]..': '..i:replace('\n', '\n\t')
    end
    return table.concat(r, '\n')
  end

  local entries = {}
  local baseSection = section
  section = string.reggsub(section, '\\W+', '')

  if baseSection:endsWith('$') then
    return 'Keyboard: '..baseSection:sub(1, #baseSection - 1)
  end
  
  if section:startsWith('_') or sim.inputMode == ac.UserInputMode.Keyboard or controlsConfig:get('ADVANCED', 'COMBINE_WITH_KEYBOARD_CONTROL', true) then
    local k = controlsConfig:get(section, 'KEY', -1)
    if k > 0 then
      local modifiers = table.map(controlsConfig:get(section, 'KEY_MODIFICATOR', nil) or {}, function (v)
        if v == '' then return nil end
        if tonumber(v) == 16 then return 'Shift' end
        if tonumber(v) == 17 then return 'Ctrl' end
        if tonumber(v) == 18 then return 'Alt' end
        return '<'..v..'>'
      end)
      if #modifiers == 0 and baseSection:endsWith('!') then
        table.insert(modifiers, 'Ctrl')
      end

      local m
      for n, v in pairs(ac.KeyIndex) do
        if v == k then
          m = n
          break
        end
      end 

      table.insert(modifiers, m or string.char(k))
      entries[#entries + 1] = 'Keyboard: '..table.concat(modifiers, '+')
    end
  end

  if sim.inputMode == ac.UserInputMode.Gamepad then
    local x = controlsConfig:get(section, 'XBOXBUTTON', '')
    if x ~= '' and (tonumber(x) or 1) > 0 then
      entries[#entries + 1] = 'Gamepad: '..x
    end
  end

  local j = controlsConfig:get(section, 'JOY', -1)
  if j >= 0 then
    local n = controlsConfig:get('CONTROLLERS', 'CON'..j, 'Unknown device')
    local d = controlsConfig:get(section, 'BUTTON', -1)
    if d >= 0 and (tonumber(x) or 1) > 0 then
      -- if #n > 28 then n = n:sub(1, 27)..'…' end
      local m = controlsConfig:get(section, 'BUTTON_MODIFICATOR', -1)
      if m >= 0 then
        -- TODO: JOY_MODIFICATOR
        entries[#entries + 1] = n..': buttons #'..(m + 1)..'+'..(d + 1)
      else
        entries[#entries + 1] = n..': button #'..(d + 1)
      end
    else
      local p = controlsConfig:get(section, '__CM_POV', -1)
      if p >= 0 then
        local dir = {[0] = '←', [1] = '↑', [2] = '→', [3] = '↓'}
        entries[#entries + 1] = n..': D-pad #'..(p + 1)..(dir[controlsConfig:get(section, '__CM_POV_DIR', -1)] or '')
      end
    end
  end

  if #entries == 0 then
    return 'Not bound to anything'
  else
    return table.concat(entries, '\n')
  end
end

local function bindingInfo(section)
  return table.getOrCreate(controlsBindings, section, bindingInfoGen, section)
end

local function bindingInfoTooltip(section, prefix)
  if ui.itemHovered() then
    ui.tooltip(function ()
      if prefix then
        ui.pushFont(ui.Font.Main)
        ui.textWrapped(prefix, 500)
        ui.popFont()
        ui.offsetCursorY(4)
      end
      ui.pushFont(ui.Font.Small)
      ui.textWrapped(bindingInfo(section), 500)
      ui.popFont()
    end)
  end
end

local function isExtraPressed(i)
  if i == 1 then return car.extraA end
  if i == 2 then return car.extraB end
  if i == 3 then return car.extraC end
  if i == 4 then return car.extraD end
  if i == 5 then return car.extraE end
  if i == 6 then return car.extraF end
  return false
end

local steerApplied = 0
local steerLocked = 0
local shiftApplied = 0
local shiftLocked = 0
local controls = ac.overrideCarControls()  --这个函数调用返回一个对象，该对象提供多个字段（例如 steer、clutch、brake、gas、handbrake、gearUp、gearDown 等），
                                           --用于手动覆盖游戏内车辆的默认控制输入。只要在 UI 循环中更新这些字段，模拟器就能依据这些值来改变车辆的状态和行为。
-- local controls = {}

local function releaseHeld()
  if weightShift and shiftApplied ~= 0 then
    weightShift.input = 0
  end
end

ac.onRelease(releaseHeld)

local colTurningLights = rgbm(0.5, 1, 0.5, 1)
local colHazards = rgbm(1, 0.5, 0.5, 1)

local function blockCarInstruments()
  local w2 = (ui.availableSpaceX() - 4) / 2
  local w3 = (ui.availableSpaceX() - 8) / 3
  local w6 = (ui.availableSpaceX() - 4 * 5) / 6

  -- Turning lights & hazards
  if car.hasTurningLights then
    if car.turningLightsActivePhase and car.turningLeftLights then ui.pushStyleColor(ui.StyleColor.Text, colTurningLights) end
    if ui.iconButton(ui.Icons.TurnSignalLeft, vec2(w3, 0), 4, true, BTN_FN(car.turningLeftOnly)) then ac.setTurningLights(car.turningLeftOnly and ac.TurningLights.None or ac.TurningLights.Left) end
    if car.turningLightsActivePhase and car.turningLeftLights then ui.popStyleColor() end
    bindingInfoTooltip('__EXT_TURNSIGNAL_LEFT', 'Left turning lights')
    ui.sameLine(0, 4)
    if car.turningLightsActivePhase and car.hazardLights then ui.pushStyleColor(ui.StyleColor.Text, colHazards) end
    if ui.iconButton(ui.Icons.Hazard, vec2(w3, 0), 4, true, BTN_FN(car.hazardLights)) then ac.setTurningLights(car.hazardLights and ac.TurningLights.None or ac.TurningLights.Hazards) end
    if car.turningLightsActivePhase and car.hazardLights then ui.popStyleColor() end
    bindingInfoTooltip('__EXT_HAZARDS', 'Hazards')
    ui.sameLine(0, 4)
    if car.turningLightsActivePhase and car.turningRightLights then ui.pushStyleColor(ui.StyleColor.Text, colTurningLights) end
    if ui.iconButton(ui.Icons.TurnSignalRight, vec2(w3, 0), 4, true, BTN_FN(car.turningRightOnly)) then ac.setTurningLights(car.turningRightOnly and ac.TurningLights.None or ac.TurningLights.Right) end
    if car.turningLightsActivePhase and car.turningRightLights then ui.popStyleColor() end
    bindingInfoTooltip('__EXT_TURNSIGNAL_RIGHT', 'Right turning lights')
  end

  -- Other functions
  local lightsW2 = car.hasHornAudioEvent or car.hasAnalogTelltale or car.hasFlashingLights
  if car.headlightsAreHeadlights and car.hasLowBeams then
    ui.setNextItemWidth(lightsW2 and w2 or -0.1)
    local value = ui.slider('##lights', not car.headlightsActive and 0 or car.lowBeams and 1 or 2, 0, 2, 
      not car.headlightsActive and 'No lights' or car.lowBeams and 'Low beams' or 'High beams')
    if ui.itemEdited() then
      value = math.round(value)
      ac.setHeadlights(value ~= 0)
      ac.setHighBeams(value == 2)
    end
    if lightsW2 then ui.sameLine(0, 4) end
    bindingInfoTooltip('Lights: ACTION_HEADLIGHTS; Low/high beams: __EXT_LOW_BEAM', 'Headlights can help in subpar lighting conditions')
  else
    ui.setNextItemWidth(lightsW2 and w2 or -0.1)
    if ui.checkbox('Lights', car.headlightsActive) then ac.setHeadlights(not car.headlightsActive) end
    bindingInfoTooltip('ACTION_HEADLIGHTS', car.headlightsAreHeadlights and 'Lights on this car act are used for a different role'
      or 'Headlights can help in subpar lighting conditions')
    if lightsW2 then ui.sameLine(0, 4) end
  end

  if car.hasFlashingLights then
    if ui.button('Flash', btnAutofill, BTN_FN(car.flashingLightsActive)) then controls.headlightsFlash = true end
    bindingInfoTooltip('ACTION_HEADLIGHTS_FLASH', 'Flash headlights')
  end
  if car.hasHornAudioEvent then
    local w = ui.getCursorX() < 40 and car.hasAnalogTelltale and w2 or -0.1
    if car.sirenHorn then
      ui.setNextItemWidth(w)
      if ui.checkbox('Siren', car.hornActive) then
        controls.horn = true
        setTimeout(function ()
          controls.horn = false
        end, 0.1)
      end
      bindingInfoTooltip('ACTION_HORN', 'Toggle siren (replaces horn on this car)')
    else
      ui.setNextItemIcon(ui.Icons.Speaker)
      ui.button('Horn', vec2(w, 0))
      controls.horn = ui.itemActive()
      bindingInfoTooltip('ACTION_HORN', 'Helps to alert other drivers')
    end
    if w > 0 then ui.sameLine(0, 4) end
  end

  if car.hasAnalogTelltale then
    local resetTelltaleNarrow = ui.availableSpaceX() < 100
    if resetTelltaleNarrow then
      ui.pushFont(ui.Font.Small)
      ui.pushStyleVar(ui.StyleVar.FramePadding, vec2(0, 5))
    end
    if ui.button('Reset telltale', btnAutofill, BTN_F0) then
      ac.simulateCustomHotkeyPress('__EXT_TELLTALE_RESET')
    end
    bindingInfoTooltip('__EXT_TELLTALE_RESET', 'Reset maximum RPM mark')
    if resetTelltaleNarrow then
      ui.popFont()
      ui.popStyleVar()
    end
  end

  if car.wiperModes > 1 then
    ui.setNextItemWidth(-0.1)
    local value = ui.slider('##wiper', car.wiperSelectedMode, 0, car.wiperModes - 1, car.wiperSelectedMode == 0 and 'Wipers: off' or 'Wipers: %d/%d' % {car.wiperMode, car.wiperModes - 1})
    if ui.itemEdited() then
      ac.setWiperMode(math.round(value))
    end
    bindingInfoTooltip('Next: __EXT_WIPERS_MORE; Previous: __EXT_WIPERS_LESS; Stop: __EXT_WIPERS_OFF', 'Current wipers mode')
  end

  ui.pushStyleVar(ui.StyleVar.FramePadding, vec2(0, 4))
  for i = 1, 6 do
    local t = extraHints[i]
    local a = ac.isExtraSwitchAvailable(i - 1, false)
    if not a then ui.pushDisabled() end
    local p = ac.accessExtraSwitchParams(i - 1)
    if anyExtraHint and not t then ui.pushStyleColor(ui.StyleColor.Text, rgbm.colors.gray) end
    if ui.button(string.char(('A'):byte(1) + i - 1), vec2(w6, 0), BTN_FN(isExtraPressed(i))) and not p.holdMode then
      ac.simulateCustomHotkeyPress('__EXT_LIGHT_'..string.char(('A'):byte(1) + i - 1))
    end
    if p and p.holdMode and ui.itemActive() then
      ac.simulateCustomHotkeyPress('__EXT_LIGHT_'..string.char(('A'):byte(1) + i - 1))
    end
    if anyExtraHint and not t then ui.popStyleColor() end
    if ui.itemHovered() then
      bindingInfoTooltip('__EXT_LIGHT_%c' % (('A'):byte(1) + i - 1), 'Extra switch %c%s' % {('A'):byte(1) + i - 1, t and '\nRole: '..t or ''})
    end
    if not a then ui.popDisabled() end
    if i < 6 then ui.sameLine(0, 4) end
  end  
  ui.popStyleVar()
end

local steerIcon, steerAngle = ui.ExtraCanvas(32), math.huge
local pedalLM, pedalLX, pedalNeutrals = vec2(), vec2(), {}
local btnAutofill = vec2(-0.1, 0)

ac.onSessionStart(function (sessionIndex, restarted)
  pedalNeutrals = {}
end)

local function pedalButton(title, size, color)
  ui.button('##'..title or title, size)
  local w = 0
  local rm = ui.itemRectMin()
  local rx = ui.itemRectMax()
  local rm2 = pedalLM:set(rm):sub(20)
  local rx2 = pedalLX:set(rx):add(20)
  if ui.itemActive() then
    w = math.lerpInvSat(ui.mouseLocalPos().y, rx.y, rm.y)
    if ui.mouseClicked(ui.MouseButton.Right) then
      pedalNeutrals[title] = w
    end
  else
    if ui.itemClicked(ui.MouseButton.Right) then
      if pedalNeutrals[title] and pedalNeutrals[title] > 0 then
        pedalNeutrals[title] = 0
      else
        w = math.lerpInvSat(ui.mouseLocalPos().y, rx.y, rm.y)
        pedalNeutrals[title] = w
      end
    end
    w = pedalNeutrals[title] or 0
    if ui.itemHovered() then
      ui.setTooltip('Click right mouse button while steering to set the new neutral value. Click the slider with right mouse button to reset the neutral value.')
    end
  end
  if w > 0 then
    rm.y = math.lerp(rx.y, rm.y, w)
    ui.drawRectFilled(rm, rx, color)
  end
  ui.beginRotation()
  ui.pushFont(ui.Font.Small)
  ui.drawTextClipped(title, rm2, rx2, rgbm.colors.white, 0.5)
  if w > 0 then
    ui.pushClipRect(rm, rx, false)
    ui.drawTextClipped(title, rm2, rx2, rgbm.colors.black, 0.5)
    ui.popClipRect()
  end
  ui.endRotation(180)
  ui.popFont()
  return w
end

local function blockCarDrive()  --
  local w4 = (ui.availableSpaceX() - 12) / 4

  if math.abs(steerAngle - car.steer) > 5 then
    steerAngle = car.steer
    steerIcon:clear(rgbm.colors.transparent):update(function (dt)
      ui.beginRotation()
      ui.setShadingOffset(1, 1, 1, 0)
      ui.image(ui.Icons.SteeringWheel, ui.windowSize())
      ui.resetShadingOffset()
      ui.endRotation(90 - car.steer)
    end)
  end


  -- -- 注释部分，为了取消干扰
  -- ui.setNextItemWidth(-0.1)
  -- ui.setNextItemIcon(steerIcon)
  -- local value = ui.slider('##steer', car.steer, -car.steerLock, car.steerLock, 'Steer: %.0f°')  --将当前的方向盘参数赋值到UI，就是说UI和当前的方向盘角度之间是有交互的，而踏板时没有交互的
  -- if not ui.itemEdited() and not ui.itemActive() then
  --   if ui.itemClicked(ui.MouseButton.Right) then
  --     steerLocked = 0
  --   end
  --   value = steerLocked
  --   if ui.itemHovered() then
  --     ui.setTooltip('Hold Shift for more precise steering. Click right mouse button while steering to set the new neutral value. Click the slider with right mouse button to reset the neutral value.')
  --   end
  -- elseif ui.mouseClicked(ui.MouseButton.Right) then
  --   steerLocked = value
  -- end

  --  steerApplied = math.applyLag(steerApplied, value / car.steerLock, 0.8, ui.deltaTime()) --通过 math.applyLag 函数对返回的 value 进行平滑处理，以避免突变产生不自然的控制效果
  -- if steerApplied == 0 then   --将UI的方向盘角度传到控制方向盘的函数内
  --   controls.steer = math.huge  -- 在一些游戏控制 API 中，使用极端值（如无穷大）往往被作为一个特殊信号，通知引擎忽略该项覆盖，采用车辆当前的状态
  -- elseif math.abs(steerApplied) < 0.001 then 
  --   steerApplied = 0
  --   controls.steer = 0
  -- else
  --   controls.steer = steerApplied  --这部分注释掉之后会导致如果进入一次固定模式则方向盘会失效，但是不注释掉会导致方向盘赋值两次，后续进行优化
  -- end

  -- local ps = vec2(w4, 68)      --读取离合，刹车，油门，手刹的设置并赋值给control
  -- controls.clutch = 1 - pedalButton('Clutch', ps, rgbm.colors.cyan)
  -- ui.sameLine(0, 4)
  -- controls.brake = pedalButton('Brakes', ps, rgbm.colors.red)
  -- ui.sameLine(0, 4)
  -- controls.gas = pedalButton('Throttle', ps, rgbm.colors.lime)
  -- ui.sameLine(0, 4)
  -- controls.handbrake = pedalButton('Handbrake', ps, rgbm.colors.yellow)

  ui.pushFont(ui.Font.Title)
  local c = ui.getCursor()
  ui.textAligned(ac.getCarGearLabel(0), 0.5, vec2(56, 44))
  -- ui.pathLineTo(vec2(ui.itemRectMin().x, ui.itemRectMax().y))
  ui.pathArcTo(c + vec2(28, 23), 16, -2.4 - math.pi / 2, 2.4 - math.pi / 2, 20)
  ui.pathStroke(rgbm.colors.gray, false, 1)
  ui.pathArcTo(c + vec2(28, 23), 16, -2.4 - math.pi / 2, math.lerp(-2.4, 2.4, math.saturateN(car.rpm / math.max(1.2 * car.rpmLimiter, 4e3))) - math.pi / 2, 20)
  ui.pathStroke(car.rpm > car.rpmLimiter and rgbm.colors.red or rgbm.colors.white, false, 1)
  ui.popFont()
  ui.sameLine(80, 0)
  ui.beginGroup(-0.1)

  if w4 < 60 then  --设置升降挡位
    if ui.iconButton(ui.Icons.Down, vec2(ui.availableSpaceX() / 2 - 2, 0), 6, true, BTN_F0) then controls.gearDown = true end
    bindingInfoTooltip('GEARDN', 'Previous gear')
    ui.sameLine(0, 4)
    if ui.iconButton(ui.Icons.Up, btnAutofill, 6, true, BTN_F0) then controls.gearUp = true end
    bindingInfoTooltip('GEARUP', 'Next gear')
  else
    if ui.button('Previous gear', vec2(ui.availableSpaceX() / 2 - 2, 0), BTN_F0) then controls.gearDown = true end
    bindingInfoTooltip('GEARDN', 'Previous gear')
    ui.sameLine(0, 4)
    if ui.button('Next gear', btnAutofill, BTN_F0) then controls.gearUp = true end
    bindingInfoTooltip('GEARUP', 'Next gear')
  end
  if ui.button('Neutral gear', btnAutofill, BTN_F0) then ac.switchToNeutralGear() end
  bindingInfoTooltip('__EXT_GEAR_NEUTRAL', 'Quickly reset to the neutral gear')
  ui.endGroup()
end

local targetTC2, targetFuelMap
local shiftIcon, shiftAngle = ui.ExtraCanvas(32), math.huge

local function blockCarTweaks()
  local w2 = (ui.availableSpaceX() - 4) / 2

  -- Car controls
  if car.absModes > 0 then
    ui.setNextItemWidth(car.tractionControlModes > 0 and w2 or -0.1)
    if car.absModes > 1 then
      local value = ui.slider('##abs', car.absMode, 0, car.absModes, 'ABS: %%.0f/%d' % car.absModes)
      if ui.itemEdited() then ac.setABS(math.round(value)) end
    elseif ui.checkbox('ABS', car.absMode == 1) then
      ac.setABS(1 - car.absMode)
    end
    if car.tractionControlModes > 0 then ui.sameLine(0, 4) end
    bindingInfoTooltip('Cycle: ABS!; Next: ABSUP; Previous: ABSDN', 'Change current ABS mode')
  end

  if car.tractionControlModes > 0 then
    ui.setNextItemWidth(car.absModes == 0 and car.tractionControl2Modes > 0 and w2 or -0.1)
    if car.tractionControlModes > 1 then
      local value = ui.slider('##tc', car.tractionControlMode, 0, car.tractionControlModes, 'TC: %%.0f/%d' % car.tractionControlModes)
      if ui.itemEdited() then ac.setTC(math.round(value)) end
    elseif ui.checkbox('TC', car.tractionControlMode == 1) then
      ac.setTC(1 - car.tractionControlMode)
    end
    if car.absModes == 0 and car.tractionControl2Modes > 0 then ui.sameLine(0, 4) end
    bindingInfoTooltip('Cycle: TRACTION_CONTROL!; Next: TCUP; Previous: TCDN', 'Change current traction control mode')
  end

  if car.tractionControl2Modes > 0 then
    ui.setNextItemWidth(-0.1)
    local tc2Value = targetTC2 or car.tractionControl2
    if car.tractionControl2Modes > 1 then
      local value = ui.slider('##tc2', tc2Value, 0, car.tractionControl2Modes, 'TC2: %%.0f/%d' % car.tractionControl2Modes)
      if ui.itemEdited() then targetTC2 = math.round(value) end
    elseif ui.checkbox('TC2', tc2Value == 1) then
      ac.setTC(1 - tc2Value)
    end
    bindingInfoTooltip('Next: __EXT_TC2_UP; Previous: __EXT_TC2_DOWN', 'Change secondary traction control mode')
    if targetTC2 and bit.band(sim.frame, 1) == 1 then
      if targetTC2 > car.tractionControl2 then
        ac.simulateCustomHotkeyPress('__EXT_TC2_UP', 1)
      elseif targetTC2 < car.tractionControl2 then
        ac.simulateCustomHotkeyPress('__EXT_TC2_DOWN', 1)
      else
        targetTC2 = nil
      end
    end
  end

  if car.fuelMaps > 0 then
    ui.setNextItemWidth(-0.1)
    local fuelMapValue = targetFuelMap or car.fuelMap
    local value = ui.slider('##fuelmap', fuelMapValue, 0, car.fuelMaps, 'Fuel map: %%.0f/%d' % car.fuelMaps)
    bindingInfoTooltip('__EXT_ENGINEMAP_UP', 'Change current fuel map (aka engine map)')
    if ui.itemEdited() then targetFuelMap = math.round(value) end
    if targetFuelMap and bit.band(sim.frame, 1) == 1 then
      if targetFuelMap ~= car.fuelMap then
        ac.simulateCustomHotkeyPress('__EXT_ENGINEMAP_UP', 1)
      else
        targetFuelMap = nil
      end
    end
  end

  if car.adjustableTurbo then
    ui.setNextItemWidth(-0.1)
    local value = ui.slider('##turbo', car.turboWastegates[0] * 10, 0, 10, 'Turbo: %.0f/10')
    if ui.itemEdited() then ac.setTurboWastegate(value / 10) end
    bindingInfoTooltip('Increase: TURBOUP; Reduce: TURBODN', 'Change turbo wastegate altering its efficiency')
  end

  if car.brakesCockpitBias then
    ui.setNextItemWidth(-0.1)    
    local value = ui.slider('##brakes', car.brakeBias * 100, car.brakesBiasLimitDown * 100, car.brakesBiasLimitUp * 100, 'Brakes: %.0f%%')
    if ui.itemEdited() then ac.setBrakeBias(value / 100) end
    bindingInfoTooltip('Move forward: BALANCEUP; Move back: BALANCEDN', 'Change brake bias')
  end

  if car.hasEngineBrakeSettings then
    ui.setNextItemWidth(-0.1)    
    local value = ui.slider('##ebrake', car.currentEngineBrakeSetting + 1, 1, car.engineBrakeSettingsCount, 'Engine brake: %%.0f/%d' % car.engineBrakeSettingsCount)
    if ui.itemEdited() then ac.setEngineBrakeSetting(value - 1) end
    bindingInfoTooltip('Increase: ENGINE_BRAKE_UP; Reduce: ENGINE_BRAKE_DN', 'Change engine braking intensity')
  end

  if car.hasCockpitSwitchForUserSpeedLimiter then
    ui.setNextItemWidth(-0.1)
    if ui.checkbox('Speed limiter', car.userSpeedLimiterEnabled) then ac.simulateCustomHotkeyPress('__EXT_SPEED_LIMITER') end
    bindingInfoTooltip('__EXT_SPEED_LIMITER', 'Click to toggle car’s speed limiter')
  end

  if not sim.isPitsSpeedLimiterForced then
    ui.setNextItemWidth(-0.1)
    if ui.checkbox('Pits limiter', car.manualPitsSpeedLimiterEnabled) then ac.simulateCustomHotkeyPress('__EXT_PIT_LIMITER') end
    bindingInfoTooltip('__EXT_PIT_LIMITER', 'Click to toggle pits limiter (in this session, limiter is not forced to activate)')
  end

  if car.drsPresent then
    ui.setNextItemWidth(car.kersPresent and w2 or -0.1)
    if not car.drsAvailable then ui.pushDisabled() end
    if ui.checkbox('DRS', car.drsActive) then controls.drs = true end
    bindingInfoTooltip('DRS', 'Click to toggle DRS')
    if not car.drsAvailable then ui.popDisabled() end
    if car.kersPresent then ui.sameLine(0, 4) end
  end

  if car.kersPresent then
    ui.button('KERS', btnAutofill)
    bindingInfoTooltip('KERS', 'Hold to activate KERS')
    local n, x = ui.itemRectMin(), ui.itemRectMax():sub(2)
    n.x, n.y = n.x + 2, x.y
    ui.drawLine(n, x, rgbm.colors.black)
    x.x = math.lerp(n.x, x.x, car.kersCharge)
    ui.drawLine(n, x, rgbm.colors.cyan)
    controls.kers = ui.itemActive() and ac.CarControlsInput.Flag.Enable or ac.CarControlsInput.Flag.Skip
  end

  if car.hasCockpitMGUHMode then
    if ui.checkbox('MGU-H charging', car.mguhChargingBatteries) then
      ac.setMGUHCharging(not car.mguhChargingBatteries)
    end
    bindingInfoTooltip('MGUH_MODE', 'Change between battery and motor modes')
    
    ui.pushFont(ui.Font.Small)
    ui.pushStyleVar(ui.StyleVar.FramePadding, vec2(0, 5))
    ui.setNextItemWidth(-0.1)    
    local value = ui.slider('##md', car.mgukDelivery + 1, 1, car.mgukDeliveryCount, 
      'MGU-K delivery: %s' % ac.getMGUKDeliveryName(0, car.mgukDelivery))
    if ui.itemEdited() then ac.setMGUKDelivery(value - 1) end
    bindingInfoTooltip('Next: MGUK_DELIVERY_UP; Previous: MGUK_DELIVERY_DN', 'Change MGU-K delivery mode')

    ui.setNextItemWidth(-0.1)    
    local value = ui.slider('##mr', car.mgukRecovery, 0, 10, 'MGU-K recovery: %.0f/10')
    if ui.itemEdited() then ac.setMGUKRecovery(value) end
    bindingInfoTooltip('Next: MGUK_RECOVERY_UP; Previous: MGUK_RECOVERY_DN', 'Change MGU-K recovery intensity')
    ui.popFont()
    ui.popStyleVar()
  end

  if weightShift then
    local current = -weightShift.input
    if math.abs(shiftAngle - current) > 0.01 then
      shiftAngle = current
      shiftIcon:clear(rgbm.colors.transparent):update(function (dt)
        ui.beginRotation()
        ui.setShadingOffset(1, 1, 1, 0)
        ui.image(ui.Icons.Driver, ui.windowSize())
        ui.resetShadingOffset()
        ui.endRotation(90 + shiftAngle * 60)
      end)
    end

    ui.setNextItemWidth(-0.1)
    ui.setNextItemIcon(shiftIcon)
    if math.abs(current) < 0.001 then current = 0 end
    local value = ui.slider('##weight', current * 1e3, -weightShift.range * 1e3, weightShift.range * 1e3, 'Shift: %.0f mm') / 1e3
    if not ui.itemEdited() and not ui.itemActive() then
      if ui.itemClicked(ui.MouseButton.Right) then
        shiftLocked = 0
      end
      value = shiftLocked
    elseif ui.mouseClicked(ui.MouseButton.Right) then
      shiftLocked = value
    end
    bindingInfoTooltip('Left: __EXT_DRIVER_SHIFT_LEFT; Right: __EXT_DRIVER_SHIFT_RIGHT', 'Shift driver weight left or right')
    shiftApplied = math.applyLag(shiftApplied, -value, 0.8, ui.deltaTime())
    if math.abs(shiftApplied) > 0.001 then
      weightShift.input = shiftApplied
    end
  end
end

local lastCarCamera = ac.CameraMode.Cockpit

local function selectNextDriveableCamera()
  if sim.cameraMode == ac.CameraMode.Cockpit then
    ac.setCurrentCamera(ac.CameraMode.Drivable)
    ac.setCurrentDrivableCamera(ac.DrivableCamera.Chase)
  elseif sim.cameraMode ~= ac.CameraMode.Drivable or sim.driveableCameraMode == ac.DrivableCamera.Dash then
    ac.setCurrentCamera(ac.CameraMode.Cockpit)
  else
    ac.setCurrentDrivableCamera((sim.driveableCameraMode + 1) % 5)
  end
end

local function blockView()
  ui.pushStyleVar(ui.StyleVar.FramePadding, vec2(0, 4))
  local w2 = (ui.availableSpaceX() - 4) / 2
  local w3 = (ui.availableSpaceX() - 8) / 3

  ui.setNextItemIcon(ui.Icons.VideoCamera)
  if ui.button('Camera', vec2(-22 * 3 - 4 * 3, 0), BTN_F0) then
    if controls:active() then
      controls.changeCamera = true
    else
      selectNextDriveableCamera()
    end
  end
  bindingInfoTooltip('F1$', 'Switch to the next driving camera')

  if sim.cameraMode == ac.CameraMode.Cockpit or sim.cameraMode == ac.CameraMode.Drivable then
    lastCarCamera = sim.cameraMode
  else
    ui.pushDisabled()
  end

  -- ui.popDisabled()

  if not controls:active() then ui.pushDisabled() end
  ui.sameLine(0, 4)
  ui.iconButton(ui.Icons.ArrowLeft, 22, 4, true)
  bindingInfoTooltip('GLANCELEFT', 'Hold to glance left')
  controls.lookLeft = ui.itemActive()
  ui.sameLine(0, 4)
  ui.iconButton(ui.Icons.ArrowDown, 22, 4, true)
  bindingInfoTooltip('GLANCEBACK', 'Hold to glance back')
  controls.lookBack = ui.itemActive()
  ui.sameLine(0, 4)
  ui.iconButton(ui.Icons.ArrowRight, 22, 4, true)
  bindingInfoTooltip('GLANCERIGHT', 'Hold to glance right')
  controls.lookRight = ui.itemActive()
  if not controls:active() then ui.popDisabled() end

  if sim.cameraMode ~= ac.CameraMode.Cockpit and sim.cameraMode ~= ac.CameraMode.Drivable then
    ui.popDisabled()
  end

  if ui.button('Free', vec2(w3, 0), BTN_FN(sim.cameraMode == ac.CameraMode.Free)) then 
    if sim.cameraMode == ac.CameraMode.Free then
      ac.setCurrentCamera(lastCarCamera) 
    else
      ac.setCurrentCamera(ac.CameraMode.Free) 
    end
  end
  bindingInfoTooltip('F7 (if enabled in AC system settings)$',
    'Enable free camera (use right mouse button to look around, arrows to move the camera, hold Control and Shift to alter camera speed)')
  ui.sameLine(0, 4)
  if ui.button(sim.orbitOnboardCamera and 'Orbit' or 'Onboard', vec2(w3, 0), BTN_FN(sim.cameraMode == ac.CameraMode.OnBoardFree)) then
    if sim.cameraMode == ac.CameraMode.OnBoardFree then
      ac.setOrbitOnboardCamera(not sim.orbitOnboardCamera)
    else
      ac.setCurrentCamera(ac.CameraMode.OnBoardFree)
    end
  end
  bindingInfoTooltip('F5$', 'Fixed camera moving relative to the car, either in orbit or free mode')
  ui.sameLine(0, 4)
  if ui.button('Car', vec2(w3, 0), BTN_FN(sim.cameraMode == ac.CameraMode.Car)) then 
    if sim.cameraMode == ac.CameraMode.Car then
      ac.setCurrentCarCamera((sim.carCameraIndex + 1) % 6)
    else
      ac.setCurrentCamera(ac.CameraMode.Car) 
    end
  end
  bindingInfoTooltip('F6$', 'A few custom preconfigured cameras positioned relative to the car')
  if sim.isVRConnected then
    ui.button('Reset VR', btnAutofill)
    if ui.itemActive() then ac.resetVRPose() end
    bindingInfoTooltip('Ctrl+Space$', 'Reset VR orientation')
  end
  ui.popStyleVar()
end

local function blockCars()
  local w3 = (ui.availableSpaceX() - 8) / 3
  ui.pushFont(ui.Font.Small)
  ui.pushStyleVar(ui.StyleVar.FramePadding, vec2(0, 4))
  if ui.button('Previous', vec2(w3, 0), BTN_F0) then
    ac.trySimKeyPressCommand('Previous Car')
  end
  bindingInfoTooltip('PREVIOUS_CAR!', 'Focus on the previous car in the list')
  ui.sameLine(0, 4)
  if ui.button('Own', vec2(w3, 0), BTN_F0) then
    ac.trySimKeyPressCommand('Player Car')
  end
  bindingInfoTooltip('PLAYER_CAR!', 'Focus on your car')
  ui.sameLine(0, 4)
  if ui.button('Next', vec2(w3, 0), BTN_F0) then
    ac.trySimKeyPressCommand('Next Car')
  end
  bindingInfoTooltip('NEXT_CAR!', 'Focus on the next car in the list')
  ui.popFont()
  ui.popStyleVar()
  ui.childWindow('cars', vec2(-0.1, 80), function ()
    for i = 0, sim.carsCount - 1 do
      ui.pushID(i)
      ui.pushStyleColor(ui.StyleColor.Text, ac.DriverTags(ac.getDriverName(i)).color)
      if ui.selectable(string.format(' %d. %s', i + 1, ac.getDriverName(i)), sim.focusedCar == i) then
        ac.focusCar(i)
      end
      ui.popID()
      ui.popStyleColor()
    end
  end)
end

local function blockReplay()
  if ui.button('Previous lap', btnAutofill, BTN_F0) then
    ac.trySimKeyPressCommand('Previous Lap')
  end
  bindingInfoTooltip('PREVIOUS_LAP!', 'Rewind to the previous lap')
  if ui.button('Next lap', btnAutofill, BTN_F0) then
    ac.trySimKeyPressCommand('Next Lap')
  end
  bindingInfoTooltip('NEXT_LAP!', 'Rewind to the next lap')
end

local autopilot = 0

local function blockMiscellaneous()
  local w2 = (ui.availableSpaceX() - 4) / 2
  if ui.button('Save clip', vec2(w2, 0), BTN_F0) then
    ac.simulateCustomHotkeyPress('__EXT_SAVE_CLIP')
  end
  bindingInfoTooltip('__EXT_SAVE_CLIP', 'Save last 30 seconds in a separate replay clip')
  ui.sameLine(0, 4)

  ui.setNextItemWidth(-0.1)
  local value = ui.slider('##ffb', car.ffbMultiplier * 100, 0, 200, 'FFB: %.0f%%') / 100
  if ui.itemEdited() then
    ac.setFFBMultiplier(value)
    ac.setMessage(string.format('User level for %s: %.0f%%', ac.getCarName(0, true), value * 100), 'Force Feedback')
  end
  bindingInfoTooltip('Increase: __EXT_FFB_INCREASE; Reduce: __EXT_FFB_DECREASE', 'Alter FFB intensity')

  ui.pushFont(ui.Font.Small)
  ui.setNextItemWidth(w2)
  if ui.checkbox('Damage', sim.damageDisplayerShown) then ac.trySimKeyPressCommand('Hide Damage') end
  bindingInfoTooltip('HIDE_DAMAGE!', 'Damage displayer showing state of the car')

  local boardParams = ac.accessOverlayLeaderboardParams()
  if boardParams then
    ui.sameLine(0, 4)
    ui.setNextItemWidth(-0.1)
    local boardOldValue = boardParams.displayMode + (boardParams.verticalLayout and 3 or 0)
    local boardValue = ui.slider('##leaderboard', boardOldValue, 0, 6, 'Board: %.0f/6')
    if boardValue ~= boardOldValue then
      boardParams.verticalLayout = boardValue > 3
      boardParams.displayMode = boardValue > 3 and boardValue - 3 or boardValue
      ac.setMessage('Overlay Leaderboard', ({'Off', 'Showing difference from first', 'Showing difference from previous', 'Alternate mode'})
        [boardParams.displayMode + 1])
    end
    bindingInfoTooltip('F9$', 'Current overlay leaderboard mode')
  end

  ui.setNextItemWidth(w2)
  if ui.checkbox('Names', sim.driverNamesShown) then ac.trySimKeyPressCommand('Driver Names') end
  bindingInfoTooltip('DRIVER_NAMES!', 'Show or hide driver names')
  ui.sameLine(0, 4)
  ui.setNextItemWidth(-0.1)
  if ui.checkbox('Ideal line', sim.idealLineShown) then ac.trySimKeyPressCommand('Ideal Line') end
  bindingInfoTooltip('IDEAL_LINE!', 'Show ideal trajectory')

  ui.setNextItemWidth(w2)
  if ui.checkbox('Auto shift', car.autoShift) then ac.trySimKeyPressCommand('Auto Shifter') end
  bindingInfoTooltip('AUTO_SHIFTER!', 'Shift gears automatically (won’t be as efficient as manual shifting)')
  if not sim.isOnlineRace then
    ui.sameLine(0, 4)
    ui.setNextItemWidth(-0.1)
    if ui.checkbox('AI', car.isAIControlled and autopilot == 0) then
      if autopilot ~= 0 then
        autopilot = 0
        physics.setAITopSpeed(0, math.huge)
      end
      ac.trySimKeyPressCommand('Activate AI') 
    end
    bindingInfoTooltip('ACTIVATE_AI!', 'Use AI to drive the car (deactivating stops any inputs, restart the session to get the control back)')
  end

  ui.popFont()
end

function script.windowMain(dt) --UI主函数
  physics.setExtraAIGrip(0, 1)
  local f = false
  local s = ui.getCursorY()
  local notAvailable = not car.isUserControlled and autopilot == 0 or sim.isReplayActive
  local frameCounter = 0
  if notAvailable then
    ui.pushDisabled()
  end

  if not ui.windowFocused() then
    releaseHeld()
  end
  
  if ui.windowHeight() > 320 then
    ui.header('Car')
  end

  blockCarInstruments()

  if ui.windowHeight() > 240 then
    blockCarDrive()
  else
    ui.setExtraContentMark(true)
  end

  blockCarTweaks()

  if notAvailable then
    ui.popDisabled()
    ui.drawRectFilled(vec2(ui.getCursorX() + 4, s + 20), vec2(ui.getCursorX() + ui.availableSpaceX() - 4, ui.getCursorY() - 20), 
      rgbm(0.1, 0.1, 0.1, 0.8), 4)
    ui.drawTextClipped(sim.isReplayActive and 'Not available in replay' or 'Can’t control the car', vec2(ui.getCursorX() + 8, s + 8), vec2(ui.getCursorX() + ui.availableSpaceX() - 8, ui.getCursorY() - 8), rgbm.colors.white, 0.5)

    if not sim.isReplayActive and not sim.isOnlineRace then
      local c = ui.getCursor()
      ui.setCursor(vec2(c.x + ui.availableSpaceX() / 2 - 60, (c.y + s) / 2 + 20))
      ui.pushFont(ui.Font.Small)
      ui.setNextItemIcon(ui.Icons.Restart)
      if ui.button('Restart session', vec2(120, 0)) then
        ac.tryToRestartSession()
      end
      bindingInfoTooltip('__CM_RESET_SESSION', 'Restart session restoring car controls')
      ui.popFont()
      ui.setCursor(c)
    end
  end

  if ui.windowHeight() > 320 then
    ui.offsetCursorY(12)
    ui.header('Camera')
    blockView()
  
    if ui.availableSpaceY() > 20 then
      if sim.isReplayActive then
        ui.offsetCursorY(12)
        ui.header('Replay')
        blockReplay()
      else
        ui.offsetCursorY(12)
        ui.header('Race')
        blockMiscellaneous()
      end
  
      if sim.carsCount > 1 then
        if ui.availableSpaceY() > 20 then
          f = true
          ui.offsetCursorY(12)
          ui.header('Cars')
          blockCars()
        end
      else
        f = true
      end
    end
  end

  if ac.isCarResetAllowed() then
    ui.offsetCursorY(12)
    ui.header('Autopilot')
    ui.setNextItemWidth(-0.1  )
    autopilot = ui.slider('##ai', autopilot, 0, 300, 'Limit: %.0f km/h')
    if ui.itemEdited() then
      physics.setAITopSpeed(0, autopilot == 0 and math.huge or autopilot)
      physics.setCarAutopilot(autopilot > 0, false)
    elseif ui.itemActive() and autopilot == 0 then
      physics.forceUserBrakesFor(0.1, 1)
    elseif not car.isAIControlled then
      autopilot = 0
      physics.setAITopSpeed(0, math.huge)
    end
  end

  if not f then
    ui.setExtraContentMark(true)
  end

  if not ui.windowResizing() then
    local h = ui.getCursorY() + 16
    ac.setWindowSizeConstraints('main', vec2(200, h), vec2(200, h))
  else
    ac.setWindowSizeConstraints('main', vec2(200, 80), vec2(200, 2000))
  end

    -- 重要：readMemoryMappedFile 读出来的内容“不要写回去”，文档也警告写会崩溃
 -- 读 shm

  shmTick(dt, car)
  shmTick3(dt, car)
  shmTick2(dt, car)

  -- -- 基于 timestamp 的真实 dt
  -- local ts = car.timestamp
  -- if lastTs == nil then lastTs = ts end
  -- local dtPhysTime = ts - lastTs
  -- if dtPhysTime < 0 then dtPhysTime = 0 end
  -- lastTs = ts

  -- if overrideactive then
  --   -- 如果由于某种原因没采到基线，就补采一次（保险）
  --   if not baseCaptured then
  --     baseGas = car.gas or 0.0
  --     baseBrake = car.brake or 0.0
  --     baseSteer = (car.steer or 0.0) / 450.0
  --     baseCaptured = true
  --   end

  --   overrideAcc = overrideAcc + dtPhysTime

  --   while overrideAcc >= PHYS_DT do
  --     overrideAcc = overrideAcc - PHYS_DT
  --     coverSteps = coverSteps + 1

  --     -- 用冻结基线进行混合（10 帧内不变）
  --     controls.gas   = 0.5 * (lastGas   + baseGas)
  --     controls.brake = 0.5 * (lastBrake + baseBrake)
  --     controls.steer = 0.5 * (lastSteer + baseSteer)

  --     if coverSteps >= COVER_FRAMES then
  --       controls.gas = math.huge
  --       controls.brake = math.huge
  --       controls.steer = math.huge
  --       overrideactive = false

  --       -- 清理
  --       overrideAcc = 0.0
  --       coverSteps = 0
  --       baseCaptured = false
  --       break
  --     end
  --   end
  -- else
  --   -- 非接管状态，清零（避免下次继承）
  --   overrideAcc = 0.0
  --   coverSteps = 0
  --   baseCaptured = false
  -- end

-- ===== overrideactive2: play stored UnboundExt2 frames (group=20) =====

  -- physics-time delta from car.timestamp
  local ts2 = car.timestamp
  if lastTs2 == nil then lastTs2 = ts2 end
  local dtPhys2 = ts2 - lastTs2
  if dtPhys2 < 0 then dtPhys2 = 0 end
  lastTs2 = ts2
  
  if overrideactive2 and #lastFrames2 >= 1 then
    -- safety: if baseline not captured (should be captured in beginOverride2), capture now
    -- if not baseCaptured2 then
    baseGas2 = car.gas or 0.0
    baseBrake2 = car.brake or 0.0
    baseSteer2 = (car.steer or 0.0) / 450.0
    baseCaptured2 = true
    -- end

    override2Acc = override2Acc + dtPhys2

    while override2Acc >= PHYS_DT do
      override2Acc = override2Acc - PHYS_DT

      local n = #lastFrames2
      local idx = override2Index
      if idx > n then
        -- 播放结束：释放控制
        controls.gas = math.huge
        controls.brake = math.huge
        controls.steer = math.huge

        overrideactive2 = false
        override2Index = 1
        override2Acc = 0.0
        baseCaptured2 = false
        break
      end

      local frm = lastFrames2[idx]
      if frm then
        controls.gas   = 0.5 * (frm.gas   + baseGas2)
        controls.brake = 0.5 * (frm.brake + baseBrake2)
        controls.steer = 0.5 * (frm.steer + baseSteer2)
      end

      override2Index = override2Index + 1
    end
  else
    -- not active: reset accumulators to avoid carrying across windows
    override2Acc = 0.0
    override2Index = 1
    baseCaptured2 = false
  end
  -- debug UI (unchanged)

  ui.text(string.format("gas=%.3f, gas1=%.3f", lastGas, baseGas2))
  ui.text(string.format("steer=%.3f, steer1=%.3f", lastSteer, baseSteer2))
  ui.text(string.format("brake=%.3f, brak1e=%.3f", lastBrake, baseBrake2))
  ui.text(string.format("Seq=%.3f", lastSeq))

    -- 处理固定路线模式的触发条件：当车灯开启时初始化固定模式，车灯关闭时中断固定模式
--   if car.headlightsActive then
--       if not fixedPathActive then
--           -- 车灯刚从关闭变为开启时初始化固定路线参数
--           fixedPathActive = true
--           pathStage = 0
--           currentSteerAngle = 0             -- 起始状态下方向盘角度为 0
--       end
--   else
--       if fixedPathActive then
--           fixedPathActive = false          -- 车灯关闭时立即中断固定路线模式
--       end
--   end
--   frameCounter = frameCounter + 1
--   if fixedPathActive then
--     if pathStage == 0 then
--         -- 阶段 0：加速阶段
--         -- 当车速小于 km/h 时，保持全油门且方向盘直行
--         if car.speedKmh <45 then
--             controls.gas = 0.6    -- 全油门
--             controls.brake = 0 
--             controls.steer = 0
--         else
--             -- 一旦车速达到 km/h，进入转向阶段
--             pathStage = 1
--         end
--     elseif pathStage == 1 then
--         -- 阶段 1：转向阶段——向右逐帧增加方向盘角度，油门与刹车都保持 0
--         controls.gas = 0
--         controls.brake = 0
-- --[[     if frameCounter % 5 == 0 then
--           currentSteerAngle = currentSteerAngle - deltaAngle
--           if currentSteerAngle <= -0.5 then
--               currentSteerAngle = -0.5  -- 限制最大向左转角
--               pathStage = 2
--           end
--         end
--         controls.steer = currentSteerAngle]]
       
--           currentSteerAngle = currentSteerAngle - deltaAngle
--           if currentSteerAngle <= -0.5 then
--               currentSteerAngle = -0.5 -- 限制最大向左转角
--               pathStage = 2
--           end
        
--         controls.steer = currentSteerAngle        
--     elseif pathStage == 2 then
--         -- 阶段 2：转向恢复阶段——逐帧减少方向盘角度，直至恢复为 0
--         controls.gas = 0
--         controls.brake = 0
--         currentSteerAngle = currentSteerAngle + deltaAngle
--         if currentSteerAngle >= 0 then
--             currentSteerAngle = 0
--             pathStage = 3  -- 转向过程结束后，重新进入加速阶段
--         end
--         controls.steer = currentSteerAngle

--     elseif pathStage == 3 then
--         -- 阶段 3：半刹车阶段
--         -- 关闭油门，设置半刹车（例如0.5），保持直行，直到车速降为0
--         controls.gas = 0
--         controls.brake = 0
--         controls.steer = 0
--         if car.speedKmh < 25 then  -- 当车速非常低时（接近0）
--             controls.gas = 0
--             controls.brake = 0
--             controls.steer = 0
--             pathStage = 0   -- 重新进入加速阶段
--         end
--     end
--   else
--     -- 非固定模式下，执行正常的用户控制逻辑（例如方向盘、踏板等其他 UI 控件刷新）
--     -- 此处可保留原有代码
--   end
end