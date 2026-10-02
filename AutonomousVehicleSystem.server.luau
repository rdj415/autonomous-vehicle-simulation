-- Connected Discord-GitHub | Discord: rdj415 | Roblox: rdj415
--!strict

--[[
    ================================================================================
    AUTONOMOUS VEHICLE CHASSIS CONTROLLER & PATROL NAVIGATION SYSTEM
    ================================================================================
    An enterprise-grade autonomous driving controller engineered for dynamic environments:
    1. Local object-space kinematic transformations for steering & throttle calculation.
    2. Multi-angular predictive raycasting for dynamic obstacle avoidance.
    3. Real-time visual sensory beam projection for runtime obstacle auditing.
    4. Dynamic wheel slip calculation modifying CustomPhysicalProperties on the fly.
    5. Closed-loop cyclic waypoint navigation for continuous autonomous patrol.
    6. Gyroscopic anti-flip recovery & traction-loss unstuck evasion algorithms.
]]

local Workspace = game:GetService("Workspace")
local RunService = game:GetService("RunService")

-- Configuration contract specifying tunable vehicle dynamics, sensor bounds, and friction limits.
export type VehicleConfig = {
	ForwardMaxSpeed: number,
	ReverseMaxSpeed: number,
	Acceleration: number,
	Deceleration: number,
	Braking: number,
	EngineSpeedCorrection: number,
	MinSpeedTorque: number,
	MaxSpeedTorque: number,
	SteeringReduction: number,
	ObstacleDetectionRange: number,
	WaypointArrivalRadius: number,
	SlipThreshold: number,
	StaticFriction: number,
	KineticFriction: number,
	RaycastAngleDegrees: number,
	EnableVisualSensors: boolean,
}

local DefaultConfig: VehicleConfig = {
	ForwardMaxSpeed = 65,
	ReverseMaxSpeed = 25,
	Acceleration = 35,
	Deceleration = 30,
	Braking = 95,
	EngineSpeedCorrection = 5,
	MinSpeedTorque = 28000,
	MaxSpeedTorque = 6000,
	SteeringReduction = 0.35,
	ObstacleDetectionRange = 26,
	WaypointArrivalRadius = 14,
	SlipThreshold = 8.5,
	StaticFriction = 1.15,
	KineticFriction = 0.55,
	RaycastAngleDegrees = 28,
	EnableVisualSensors = true,
}

-- Mathematical conversion constants between Roblox physics units (studs) and real-world imperial units.
local StudsPerMile: number = 5280
local SecondsPerHour: number = 3600

local AutonomousVehicleSystem = {}
AutonomousVehicleSystem.__index = AutonomousVehicleSystem

-- Strong typing definition representing the internal state fields of the controller instance.
export type ControllerInstance = typeof(setmetatable({} :: {
	_model: Model,
	_chassis: BasePart,
	_seat: VehicleSeat,
	_motors: {
		FR: any,
		FL: any,
		RR: any,
		RL: any,
	},
	_steeringRack: any,
	_wheelParts: {BasePart},
	_wheelRadius: number,
	_config: VehicleConfig,
	_engineSpeed: number,
	_waypoints: {Vector3},
	_currentWaypointIndex: number,
	_unstuckTimer: number,
	_unstuckStage: number,
	_lastMovedPosition: Vector3,
	_stuckDetectionTimer: number,
	_flippedTimer: number,
	_rayParams: RaycastParams,
	_sensorFolder: Folder?,
	_sensorVisualParts: {BasePart},
	_heartbeatConnection: RBXScriptConnection?,
}, AutonomousVehicleSystem))

--[[
    Constructor: AutonomousVehicleSystem.new
    Initializes vehicle hardware references, configures raycast collision parameters,
    and binds internal metatable state to the provided physical Model assembly.
]]
function AutonomousVehicleSystem.new(vehicleModel: Model, configOverride: VehicleConfig?): ControllerInstance
	-- Resolve primary structural and kinematic BaseParts using strict Luau type assertions.
	local chassis = vehicleModel:WaitForChild("Chassis") :: BasePart
	local seat = (vehicleModel:FindFirstChildOfClass("VehicleSeat") or vehicleModel:WaitForChild("DriverSeat")) :: VehicleSeat
	local engineFolder = vehicleModel:WaitForChild("Engine")
	local steeringFolder = vehicleModel:WaitForChild("Steering")
	local wheelsFolder = vehicleModel:WaitForChild("Wheels")

	-- Derive wheel radius (Size.Y / 2) to translate rotational angular velocity into linear chassis velocity (v = w * r).
	local frontRightWheel = wheelsFolder:WaitForChild("WheelFR"):WaitForChild("Wheel") :: BasePart
	local derivedRadius = frontRightWheel.Size.Y / 2

	-- Construct RaycastParams with Exclude filtering to ignore the vehicle's own descendants during sensory ray projection.
	local raycastParameters = RaycastParams.new()
	raycastParameters.FilterType = Enum.RaycastFilterType.Exclude
	raycastParameters.FilterDescendantsInstances = {vehicleModel}

	local activeConfig = configOverride or DefaultConfig
	local visualParts: {BasePart} = {}
	local sensorFolder: Folder? = nil

	-- Construct real-time visual ray adornments to demonstrate sensory perception in the 3D viewport.
	if activeConfig.EnableVisualSensors then
		sensorFolder = Instance.new("Folder")
		sensorFolder.Name = "SensoryDebugRays"
		sensorFolder.Parent = vehicleModel

		for idx = 1, 3 do
			local beamPart = Instance.new("Part")
			beamPart.Name = "SensorBeam_" .. idx
			beamPart.Anchored = true
			beamPart.CanCollide = false
			beamPart.CanQuery = false
			beamPart.CanTouch = false
			beamPart.CastShadow = false
			beamPart.Material = Enum.Material.Neon
			beamPart.Transparency = 0.35
			beamPart.Size = Vector3.new(0.15, 0.15, 1)
			beamPart.Color = Color3.fromRGB(0, 240, 200)
			beamPart.Parent = sensorFolder
			table.insert(visualParts, beamPart)
		end
	end

	-- Construct instance state table and bind the metatable to support object-oriented method dispatch.
	local self = setmetatable({
		_model = vehicleModel,
		_chassis = chassis,
		_seat = seat,
		-- Mechanical actuator constraints driving wheel rotation.
		_motors = {
			FR = engineFolder:WaitForChild("WheelFRMotor"),
			FL = engineFolder:WaitForChild("WheelFLMotor"),
			RR = engineFolder:WaitForChild("WheelRRMotor"),
			RL = engineFolder:WaitForChild("WheelRLMotor"),
		},
		-- PrismaticConstraint steering actuator translating along the lateral chassis axis.
		_steeringRack = steeringFolder:WaitForChild("SteeringRack"),
		_wheelParts = {
			frontRightWheel,
			wheelsFolder:WaitForChild("WheelFL"):WaitForChild("Wheel") :: BasePart,
			wheelsFolder:WaitForChild("WheelRR"):WaitForChild("Wheel") :: BasePart,
			wheelsFolder:WaitForChild("WheelRL"):WaitForChild("Wheel") :: BasePart,
		},
		_wheelRadius = derivedRadius,
		_config = activeConfig,
		_engineSpeed = 0,
		_waypoints = {},
		_currentWaypointIndex = 1,
		_unstuckTimer = 0,
		_unstuckStage = 0,
		_lastMovedPosition = chassis.Position,
		_stuckDetectionTimer = 0,
		_flippedTimer = 0,
		_rayParams = raycastParameters,
		_sensorFolder = sensorFolder,
		_sensorVisualParts = visualParts,
		_heartbeatConnection = nil,
	}, AutonomousVehicleSystem)

	return self
end

-- Converts Roblox linear velocity (studs/s) into miles/h for automotive torque curve processing.
local function studsPerSecondToMph(sps: number): number
	return (sps / StudsPerMile) * SecondsPerHour
end

-- Converts miles/h into Roblox linear velocity (studs/s) to command constraint angular velocities.
local function mphToStudsPerSecond(mph: number): number
	return (mph * StudsPerMile) / SecondsPerHour
end

--[[
    _renderSensorRay
    Positions a thin neon cylinder/box between origin and target point to visually display raycasts.
]]
function AutonomousVehicleSystem:_renderSensorRay(index: number, startPos: Vector3, endPos: Vector3, hasHit: boolean)
	local beam = self._sensorVisualParts[index]
	if not beam then return end

	local delta = endPos - startPos
	local distance = delta.Magnitude
	if distance < 0.1 then return end

	-- Orient the visual part along the raycast trajectory vector.
	beam.Size = Vector3.new(0.18, 0.18, distance)
	beam.CFrame = CFrame.lookAt(startPos + delta * 0.5, endPos)
	-- Highlight red on active obstacle obstruction, or cyan when clear.
	beam.Color = if hasHit then Color3.fromRGB(255, 50, 50) else Color3.fromRGB(0, 230, 190)
end

--[[
    _getObstacleSteerOffset
    Fires directional raycasts (center, +yaw, -yaw) relative to the VehicleSeat.
    Produces a normalized steering offset [-1.0, 1.0] to guide the vehicle away from physical obstacles.
]]
function AutonomousVehicleSystem:_getObstacleSteerOffset(): number
	local seatCFrame = self._seat.CFrame
	-- Position ray origin ahead of the front bumper along LookVector to prevent self-intersection.
	local rayOrigin = self._seat.Position + seatCFrame.LookVector * (self._seat.Size.Z * 0.5 + 1.2)
	local sensorDistance = self._config.ObstacleDetectionRange
	local sensorAngle = self._config.RaycastAngleDegrees

	-- Calculate angular ray trajectories using CFrame.Angles yaw transformation on LookVector.
	local forwardVector = seatCFrame.LookVector * sensorDistance
	local leftVector = (seatCFrame * CFrame.Angles(0, math.rad(sensorAngle), 0)).LookVector * sensorDistance
	local rightVector = (seatCFrame * CFrame.Angles(0, math.rad(-sensorAngle), 0)).LookVector * sensorDistance

	-- Execute spatial ray queries against the Workspace geometry.
	local hitForward = Workspace:Raycast(rayOrigin, forwardVector, self._rayParams)
	local hitLeft = Workspace:Raycast(rayOrigin, leftVector, self._rayParams)
	local hitRight = Workspace:Raycast(rayOrigin, rightVector, self._rayParams)

	-- Update visual sensory adornments if enabled.
	if self._config.EnableVisualSensors and #self._sensorVisualParts == 3 then
		local endFwd = if hitForward then hitForward.Position else (rayOrigin + forwardVector)
		local endLeft = if hitLeft then hitLeft.Position else (rayOrigin + leftVector)
		local endRight = if hitRight then hitRight.Position else (rayOrigin + rightVector)

		self:_renderSensorRay(1, rayOrigin, endFwd, hitForward ~= nil)
		self:_renderSensorRay(2, rayOrigin, endLeft, hitLeft ~= nil)
		self:_renderSensorRay(3, rayOrigin, endRight, hitRight ~= nil)
	end

	if hitForward then
		-- When forward path is obstructed, prioritize the side offering maximum geometric clearance.
		local distLeft = hitLeft and (hitLeft.Position - rayOrigin).Magnitude or sensorDistance
		local distRight = hitRight and (hitRight.Position - rayOrigin).Magnitude or sensorDistance
		return if distLeft > distRight then -1.0 else 1.0
	elseif hitLeft then
		-- Apply repulsive bias proportional to obstacle proximity (1 - normalized distance).
		local proximityRatio = 1 - ((hitLeft.Position - rayOrigin).Magnitude / sensorDistance)
		return proximityRatio * 0.85
	elseif hitRight then
		-- Apply repulsive bias proportional to obstacle proximity (1 - normalized distance).
		local proximityRatio = 1 - ((hitRight.Position - rayOrigin).Magnitude / sensorDistance)
		return -proximityRatio * 0.85
	end

	return 0
end

--[[
    _updateWheelFriction
    Compares wheel circumferential angular velocity with chassis linear velocity.
    Dynamically applies CustomPhysicalProperties to simulate loss of traction under wheel slippage.
]]
function AutonomousVehicleSystem:_updateWheelFriction()
	local slipTolerance = self._config.SlipThreshold
	for _, wheelPart in ipairs(self._wheelParts) do
		-- AssemblyLinearVelocity: linear movement speed of the part in world space.
		local linearSpeed = wheelPart.AssemblyLinearVelocity.Magnitude
		-- Circumferential speed calculated by multiplying angular speed (rad/s) by wheel radius.
		local surfaceSpeed = self._wheelRadius * wheelPart.AssemblyAngularVelocity.Magnitude

		-- If difference exceeds threshold, tires break traction and enter kinetic sliding friction.
		local isSlipping = math.abs(linearSpeed - surfaceSpeed) > slipTolerance
		local effectiveFriction = if isSlipping then self._config.KineticFriction else self._config.StaticFriction

		-- PhysicalProperties constructor: (Density = 1.0, Friction = effectiveFriction, Elasticity = 0.0).
		wheelPart.CustomPhysicalProperties = PhysicalProperties.new(1.0, effectiveFriction, 0.0)
	end
end

--[[
    _updatePhysics
    Calculates drivetrain acceleration, adjusts motor AngularVelocity and MotorMaxTorque,
    and sets the steering rack constraint target position based on speed-damped steering inputs.
]]
function AutonomousVehicleSystem:_updatePhysics(throttleInput: number, steeringInput: number, deltaTime: number)
	-- VectorToObjectSpace converts global velocity into chassis-relative coordinates.
	-- Negative Z corresponds to forward motion in Roblox CFrame convention.
	local relativeVelocity = self._chassis.CFrame:VectorToObjectSpace(self._chassis.AssemblyLinearVelocity)
	local forwardMph = studsPerSecondToMph(-relativeVelocity.Z)
	local targetMph = throttleInput * (if throttleInput >= 0 then self._config.ForwardMaxSpeed else self._config.ReverseMaxSpeed)

	-- Check whether throttle input opposes current movement direction, indicating an active brake request.
	local isBraking = throttleInput ~= 0 and math.abs(forwardMph) > 1 and math.sign(throttleInput) ~= math.sign(forwardMph)
	local accelerationRate = self._config.Acceleration

	if isBraking then
		accelerationRate = self._config.Braking
		targetMph = 0
	elseif math.abs(targetMph) < math.abs(self._engineSpeed) then
		accelerationRate = self._config.Deceleration
	end

	-- Apply low-pass filter to synchronize engine simulation with actual physics velocity.
	local blendWeight = math.min(deltaTime * self._config.EngineSpeedCorrection, 1.0)
	self._engineSpeed += (forwardMph - self._engineSpeed) * blendWeight

	-- Step virtual engine speed bounded by acceleration and deceleration curves.
	if targetMph > self._engineSpeed then
		self._engineSpeed = math.min(self._engineSpeed + accelerationRate * deltaTime, targetMph)
	else
		self._engineSpeed = math.max(self._engineSpeed - accelerationRate * deltaTime, targetMph)
	end

	-- Torque curve interpolation: higher torque at low speeds to overcome static friction and inertia.
	local speedRatio = math.clamp(math.abs(forwardMph) / self._config.ForwardMaxSpeed, 0, 1)
	local appliedTorque = self._config.MinSpeedTorque + (self._config.MaxSpeedTorque - self._config.MinSpeedTorque) * speedRatio

	-- Convert engine speed into angular velocity (radians/s) required for constraint motors.
	local targetAngularVelocity = mphToStudsPerSecond(self._engineSpeed) / self._wheelRadius

	-- Distribute rotational velocity across all 4 drive motors with symmetric polarity for left/right pairs.
	self._motors.FR.AngularVelocity = targetAngularVelocity
	self._motors.FR.MotorMaxTorque = appliedTorque
	self._motors.FL.AngularVelocity = -targetAngularVelocity
	self._motors.FL.MotorMaxTorque = appliedTorque
	self._motors.RR.AngularVelocity = targetAngularVelocity
	self._motors.RR.MotorMaxTorque = appliedTorque
	self._motors.RL.AngularVelocity = -targetAngularVelocity
	self._motors.RL.MotorMaxTorque = appliedTorque

	-- Reduce high-speed steering sensitivity to prevent roll-over instability at maximum velocity.
	local steeringDamping = math.max(1 - speedRatio * self._config.SteeringReduction, 0.1)
	local effectiveSteering = steeringInput * steeringDamping

	-- Actuate PrismaticConstraint rack position between physical upper and lower endpoints.
	if effectiveSteering > 0 then
		self._steeringRack.TargetPosition = effectiveSteering * self._steeringRack.LowerLimit
	else
		self._steeringRack.TargetPosition = -effectiveSteering * self._steeringRack.UpperLimit
	end

	self:_updateWheelFriction()
end

--[[
    _recoverIfFlipped
    Inspects the normal vector of the vehicle seat against global Y axis.
    If inverted for over 2 seconds, resets velocities and restores an upright orientation.
]]
function AutonomousVehicleSystem:_recoverIfFlipped(deltaTime: number)
	-- UpVector.Y < 0.25 indicates the vehicle is tilted beyond a safe recoverable angle.
	if self._seat.CFrame.UpVector.Y < 0.25 then
		self._flippedTimer += deltaTime
		if self._flippedTimer > 2.0 then
			local currentCFrame = self._seat.CFrame
			-- Reconstruct rotation matrix retaining current yaw heading while eliminating roll and pitch.
			local yawAngle = math.atan2(currentCFrame.LookVector.X, currentCFrame.LookVector.Z) + math.pi
			local uprightRotation = CFrame.Angles(0, yawAngle, 0)

			-- Cancel residual angular momentum to prevent oscillations upon respawn.
			self._chassis.AssemblyLinearVelocity = Vector3.zero
			self._chassis.AssemblyAngularVelocity = Vector3.zero
			self._chassis.CFrame = CFrame.new(currentCFrame.Position + Vector3.new(0, 4.0, 0)) * uprightRotation
			self._flippedTimer = 0
		end
	else
		self._flippedTimer = 0
	end
end

--[[
    _calculateSteerAndThrottle
    Transforms the current 3D world target coordinate into vehicle local object space.
    Computes heading angle via math.atan2 and blends in obstacle avoidance raycast bias.
]]
function AutonomousVehicleSystem:_calculateSteerAndThrottle(targetPosition: Vector3): (number, number)
	-- PointToObjectSpace computes the target coordinate relative to the vehicle's position and orientation.
	local localTarget = self._seat.CFrame:PointToObjectSpace(targetPosition)
	local horizontalTarget = Vector3.new(localTarget.X, 0, localTarget.Z)

	-- If vehicle is already at the target location, nullify control commands.
	if horizontalTarget.Magnitude < 4.0 then
		return 0, 0
	end

	-- math.atan2(X, -Z) yields heading error in radians relative to forward heading (-Z).
	local headingAngle = math.atan2(localTarget.X, -localTarget.Z)
	-- Normalize angle against pi/4 (45 degrees) to obtain steer range between [-1, 1].
	local baseSteering = math.clamp(headingAngle / (math.pi / 4), -1.0, 1.0)
	local obstacleOffset = self:_getObstacleSteerOffset()
	local blendedSteering = math.clamp(baseSteering + obstacleOffset, -1.0, 1.0)

	local targetThrottle = 1.0
	-- If target is ahead (localTarget.Z < 0), apply forward throttle modulated by steering intensity.
	if localTarget.Z < 0 then
		local turnSeverity = math.abs(blendedSteering)
		if turnSeverity > 0.45 then
			-- Reduce throttle in sharp corners to prevent understeer and excessive drift.
			targetThrottle = math.clamp(1.0 - (turnSeverity * 0.55), 0.35, 1.0)
		end
	else
		-- Target is behind the vehicle: initiate reverse maneuver if heading discrepancy is severe.
		targetThrottle = if math.abs(blendedSteering) > 0.8 then -1.0 else 1.0
		blendedSteering = -blendedSteering
	end

	return targetThrottle, blendedSteering
end

--[[
    _handleUnstuckLogic
    Detects when drive motors have high commanded speed but physical displacement remains static.
    Triggers a timed reverse and counter-steer maneuver to extract the chassis from obstructions.
]]
function AutonomousVehicleSystem:_handleUnstuckLogic(deltaTime: number): (number?, number?)
	local currentTime = os.clock()
	local displacement = (self._chassis.Position - self._lastMovedPosition).Magnitude

	-- High engine speed paired with low displacement indicates physical immobilization.
	if displacement < 2.5 and math.abs(self._engineSpeed) > 12 then
		self._stuckDetectionTimer += deltaTime
		if self._stuckDetectionTimer > 2.0 and self._unstuckStage == 0 then
			self._unstuckStage = 1
			self._unstuckTimer = currentTime + 1.6
			self._stuckDetectionTimer = 0
		end
	else
		self._stuckDetectionTimer = 0
		self._lastMovedPosition = self._chassis.Position
	end

	-- Evasion stage 1: apply reverse throttle and reverse steer to free chassis from collision.
	if self._unstuckStage > 0 then
		if currentTime < self._unstuckTimer then
			return -0.85, 0.75
		else
			self._unstuckStage = 0
		end
	end

	return nil, nil
end

-- Assigns an ordered sequence of 3D patrol coordinates.
function AutonomousVehicleSystem:SetWaypoints(waypointList: {Vector3})
	self._waypoints = waypointList
	self._currentWaypointIndex = 1
end

-- Advances waypoint index when vehicle arrives within WaypointArrivalRadius.
function AutonomousVehicleSystem:_updateWaypointNavigation(): Vector3?
	if #self._waypoints == 0 then
		return nil
	end

	local currentTarget = self._waypoints[self._currentWaypointIndex]
	local horizontalDistance = (Vector3.new(self._chassis.Position.X, 0, self._chassis.Position.Z) 
		- Vector3.new(currentTarget.X, 0, currentTarget.Z)).Magnitude

	if horizontalDistance < self._config.WaypointArrivalRadius then
		self._currentWaypointIndex = (self._currentWaypointIndex % #self._waypoints) + 1
		currentTarget = self._waypoints[self._currentWaypointIndex]
	end

	return currentTarget
end

-- Main tick routine called every Heartbeat frame.
function AutonomousVehicleSystem:Update(deltaTime: number)
	self:_recoverIfFlipped(deltaTime)

	local throttle, steer = 0, 0
	local unstuckThrottle, unstuckSteer = self:_handleUnstuckLogic(deltaTime)

	if unstuckThrottle and unstuckSteer then
		throttle = unstuckThrottle
		steer = unstuckSteer
	else
		local activeTarget = self:_updateWaypointNavigation()
		if activeTarget then
			throttle, steer = self:_calculateSteerAndThrottle(activeTarget)
		end
	end

	self._seat.ThrottleFloat = throttle
	self._seat.SteerFloat = steer
	self:_updatePhysics(throttle, steer, deltaTime)
end

-- Connects the controller tick to RunService.Heartbeat for frame-synchronized simulation.
function AutonomousVehicleSystem:Start()
	if self._heartbeatConnection then return end
	self._heartbeatConnection = RunService.Heartbeat:Connect(function(dt: number)
		self:Update(dt)
	end)
end

-- Cleans up physics allocations and disconnects heartbeat listeners.
function AutonomousVehicleSystem:Destroy()
	if self._heartbeatConnection then
		self._heartbeatConnection:Disconnect()
		self._heartbeatConnection = nil
	end
	if self._sensorFolder then
		self._sensorFolder:Destroy()
		self._sensorFolder = nil
	end
	self._sensorVisualParts = {}
	self._engineSpeed = 0
	self._motors.FR.AngularVelocity = 0
	self._motors.FL.AngularVelocity = 0
	self._motors.RR.AngularVelocity = 0
	self._motors.RL.AngularVelocity = 0
	self._seat.ThrottleFloat = 0
	self._seat.SteerFloat = 0
end

-- Entry point: If running as an active Server Script.
local vehicleModel: Model? = script:FindFirstAncestorOfClass("Model")
if not (vehicleModel and vehicleModel:FindFirstChild("Chassis")) then
	for _, child in ipairs(Workspace:GetChildren()) do
		if child:IsA("Model") and child:FindFirstChild("Chassis") then
			vehicleModel = child
			break
		end
	end
end

if vehicleModel and vehicleModel:FindFirstChild("Chassis") then
	local controller = AutonomousVehicleSystem.new(vehicleModel)

	-- Populate waypoints from Workspace.Waypoints folder if present.
	task.spawn(function()
		local waypointsFolder = Workspace:WaitForChild("Waypoints", 10)
		if waypointsFolder then
			local points: {Vector3} = {}
			local children = waypointsFolder:GetChildren()
			table.sort(children, function(a, b)
				return a.Name < b.Name
			end)
			for _, child in ipairs(children) do
				if child:IsA("BasePart") then
					table.insert(points, child.Position)
				end
			end
			controller:SetWaypoints(points)
		end
	end)

	controller:Start()
end

return AutonomousVehicleSystem
