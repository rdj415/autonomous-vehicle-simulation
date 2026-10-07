-- Connected Discord-GitHub | Discord: rd_rb | Roblox: DiscoUnicorn478
--!strict
--[[
    ================================================================================
    AUTONOMOUS VEHICLE CHASSIS CONTROLLER & PATROL NAVIGATION SYSTEM
    ================================================================================
    Architectural Overview & Engineering Principles:
    
    1. Kinematic Reference Frame Transformations:
       Autonomous steering and navigation rely on local body-fixed coordinate
       transformations (ISO 8855 standard). By mapping world-space waypoint targets
       into the vehicle seat's local object space (where -Z represents forward heading,
       +X represents lateral right, and +Y represents upward normal), we decouple the
       vehicle's global heading from the local steering error calculation.
       
    2. Proportional Steering Controller with Speed Damping:
       The angular yaw discrepancy between the chassis heading vector and the waypoint
       direction is extracted using the arctangent function. To prevent high-speed
       oversteer and dynamic rollover instability, steering gain is actively damped as
       linear velocity approaches maximum forward speed.
       
    3. Multi-Raycast Predictive Obstacle Avoidance:
       Mimics an array of forward-facing time-of-flight / LIDAR sensors. Projecting
       tri-directional rays (boresight, left offset, and right offset) ahead of the
       front bumper calculates real-time obstacle proximity gradients, applying a
       repulsive lateral bias to the baseline pathfinding trajectory.
       
    4. Longitudinal Powertrain Dynamics & Slip-Adaptive Friction:
       Simulates an electric drivetrain torque-speed curve (high stall torque tapering
       at terminal velocity) and calculates wheel longitudinal slip. When the differential
       between wheel rim circumferential velocity (v = omega * r) and ground velocity
       exceeds the slip limit, physical properties dynamically transition from static to
       kinetic friction, accurately modeling traction loss.
       
    5. Gyroscopic Stability & Anti-Entrapment State Machines:
       Monitors vehicle attitude (UpVector tilt) and translational displacement over time.
       Automatically triggers upright gyroscopic restitution when inverted and initiates
       a timed counter-steering reverse maneuver if the vehicle is physically immobilized.
    ================================================================================
]]

-- System Services
local Workspace = game:GetService("Workspace")
local RunService = game:GetService("RunService")

--------------------------------------------------------------------------------
-- NAMED CONSTANTS & TUNABLE PARAMETERS (ZERO MAGIC NUMBERS)
--------------------------------------------------------------------------------

-- Kinematic Plane & Unit Conversion Constants
local STUDS_PER_MILE: number = 5280.0
local SECONDS_PER_HOUR: number = 3600.0
local HORIZONTAL_PLANE_MASK: Vector3 = Vector3.new(1.0, 0.0, 1.0)
local RADIUS_FROM_DIAMETER_RATIO: number = 0.5

-- Steering Transfer Function Constants
-- In Roblox coordinates (-Z forward, +X right), atan2(X, -Z) yields error angle theta.
-- A 45-degree (pi / 4 radians) heading error maps to 100% normalized steering deflection.
local MAX_STEER_REFERENCE_ANGLE_RAD: number = math.pi / 4.0
local STEER_NORMALIZATION_FACTOR: number = 1.0 / MAX_STEER_REFERENCE_ANGLE_RAD
local MIN_NORMALIZED_STEER: number = -1.0
local MAX_NORMALIZED_STEER: number = 1.0
local MIN_STEERING_DAMPING_FLOOR: number = 0.1

-- Throttle & Cornering Dynamics Constants
local TARGET_ARRIVAL_DEADZONE_STUDS: number = 4.0
local SHARP_TURN_STEER_THRESHOLD: number = 0.45
local CORNERING_THROTTLE_REDUCTION_FACTOR: number = 0.55
local MIN_CORNERING_THROTTLE_FLOOR: number = 0.35
local REVERSE_MANEUVER_STEER_THRESHOLD: number = 0.8
local FORWARD_HEMISPHERE_Z_BOUNDARY: number = 0.0
local FULL_FORWARD_THROTTLE: number = 1.0
local FULL_REVERSE_THROTTLE: number = -1.0
local NEUTRAL_THROTTLE: number = 0.0
local BRAKE_DETECTION_SPEED_THRESHOLD_MPH: number = 1.0

-- Sensory Perception & Obstacle Avoidance Constants
local SENSOR_BEAM_COUNT: number = 3
local SENSOR_BEAM_INITIAL_THICKNESS: number = 0.15
local SENSOR_BEAM_ACTIVE_THICKNESS: number = 0.18
local SENSOR_BEAM_INITIAL_LENGTH: number = 1.0
local SENSOR_INDEX_FORWARD: number = 1
local SENSOR_INDEX_LEFT: number = 2
local SENSOR_INDEX_RIGHT: number = 3
local SENSOR_BEAM_TRANSPARENCY: number = 0.35
local MIN_RAY_RENDER_DISTANCE: number = 0.1
local BUMPER_CLEARANCE_OFFSET_STUDS: number = 1.2
local OBSTACLE_LATERAL_BIAS_WEIGHT: number = 0.85
local OBSTACLE_HARD_AVOIDANCE_DEFLECTION: number = 1.0
local BEAM_COLOR_CLEAR: Color3 = Color3.fromRGB(0, 230, 190)
local BEAM_COLOR_BLOCKED: Color3 = Color3.fromRGB(255, 50, 50)

-- Wheel Physical Properties Constants
local WHEEL_DENSITY: number = 1.0
local WHEEL_ELASTICITY: number = 0.0

-- Inverted Attitude / Anti-Flip Recovery Constants
-- UpVector.Y represents the cosine of the inclination angle against world vertical (0, 1, 0).
-- When UpVector.Y < 0.25, vehicle inclination exceeds ~75.5 degrees (imminent or active rollover).
local FLIPPED_INCLINATION_COSINE_THRESHOLD: number = 0.25
local FLIPPED_PERSISTENCE_TIME_SECONDS: number = 2.0
local RESPAWN_GROUND_CLEARANCE_STUDS: Vector3 = Vector3.new(0.0, 4.0, 0.0)

-- Deadlock / Unstuck Evasion State Machine Constants
local STUCK_DISPLACEMENT_THRESHOLD_STUDS: number = 2.5
local STUCK_MIN_ENGINE_SPEED_MPH: number = 12.0
local STUCK_DETECTION_WINDOW_SECONDS: number = 2.0
local UNSTUCK_MANEUVER_DURATION_SECONDS: number = 1.6
local UNSTUCK_STAGE_IDLE: number = 0
local UNSTUCK_STAGE_ACTIVE: number = 1
local UNSTUCK_ESCAPE_THROTTLE: number = -0.85
local UNSTUCK_ESCAPE_STEER: number = 0.75

-- Infrastructure & Resource Discovery Timeout Constants
local CHASSIS_DISCOVERY_TIMEOUT_SECONDS: number = 5.0
local WAYPOINTS_FOLDER_TIMEOUT_SECONDS: number = 10.0
local ACTIVE_VEHICLES_FOLDER_TIMEOUT_SECONDS: number = 5.0

--------------------------------------------------------------------------------
-- CONFIGURATION CONTRACT & DEFAULT SPECIFICATION
--------------------------------------------------------------------------------

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

--------------------------------------------------------------------------------
-- CONTROLLER CLASS & INSTANCE TYPING
--------------------------------------------------------------------------------

export type MotorConstraint = HingeConstraint | CylindricalConstraint

export type ControllerInstance = typeof(setmetatable({} :: {
	_model: Model,
	_chassis: BasePart,
	_seat: VehicleSeat,
	_motors: {
		FR: MotorConstraint,
		FL: MotorConstraint,
		RR: MotorConstraint,
		RL: MotorConstraint,
	},
	_steeringRack: PrismaticConstraint,
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
	_staticPhysicalProps: PhysicalProperties,
	_kineticPhysicalProps: PhysicalProperties,
	_wheelSlippingState: { [BasePart]: boolean },
	_heartbeatConnection: RBXScriptConnection?,
}, {} :: any))

local AutonomousVehicleSystem = {}
AutonomousVehicleSystem.__index = AutonomousVehicleSystem

--------------------------------------------------------------------------------
-- UNIT CONVERSION HELPER FUNCTIONS
--------------------------------------------------------------------------------

--[[
    studsPerSecondToMph
    Translates Roblox physics simulation velocity (studs/s) into statute miles per hour.
    Formula: v_mph = (v_studs_per_sec / 5280) * 3600
]]
local function studsPerSecondToMph(sps: number): number
	return (sps / STUDS_PER_MILE) * SECONDS_PER_HOUR
end

--[[
    mphToStudsPerSecond
    Translates automotive speed in miles per hour back into Roblox linear studs per second.
    Formula: v_studs_per_sec = (v_mph * 5280) / 3600
]]
local function mphToStudsPerSecond(mph: number): number
	return (mph * STUDS_PER_MILE) / SECONDS_PER_HOUR
end

--------------------------------------------------------------------------------
-- CONSTRUCTOR & HARDWARE INITIALIZATION
--------------------------------------------------------------------------------

--[[
    AutonomousVehicleSystem.new
    Constructs an autonomous vehicle controller bound to the target Model assembly.
    - Resolves and validates mechanical assemblies (Chassis, Seat, Motors, SteeringRack).
    - Caches PhysicalProperties objects to eliminate runtime memory allocation overhead.
    - Configures raycast exclusion filters to avoid self-collisions.
    - Initializes sensory visual indicators if enabled in the vehicle configuration.
]]
function AutonomousVehicleSystem.new(vehicleModel: Model, configOverride: VehicleConfig?): ControllerInstance
	local chassis = vehicleModel:WaitForChild("Chassis") :: BasePart
	local seat = (vehicleModel:FindFirstChildOfClass("VehicleSeat") or vehicleModel:WaitForChild("DriverSeat")) :: VehicleSeat
	local engineFolder = vehicleModel:WaitForChild("Engine")
	local steeringFolder = vehicleModel:WaitForChild("Steering")
	local wheelsFolder = vehicleModel:WaitForChild("Wheels")

	-- Derive wheel radius from vertical bounding dimension: r = diameter * 0.5.
	-- Wheel radius is fundamental for converting chassis linear velocity (studs/s)
	-- into wheel motor angular velocity (radians/s) via v = omega * r.
	local frontRightWheel = wheelsFolder:WaitForChild("WheelFR"):WaitForChild("Wheel") :: BasePart
	local derivedRadius = frontRightWheel.Size.Y * RADIUS_FROM_DIAMETER_RATIO

	-- Configure RaycastParams with Exclude filtering to guarantee sensory rays ignore
	-- the vehicle's own chassis, wheels, and debug geometry during spatial queries.
	local raycastParameters = RaycastParams.new()
	raycastParameters.FilterType = Enum.RaycastFilterType.Exclude
	raycastParameters.FilterDescendantsInstances = {vehicleModel}

	local activeConfig = configOverride or DefaultConfig
	local visualParts: {BasePart} = {}
	local sensorFolder: Folder? = nil

	-- Construct 3D sensory indicator beams for real-time visualization of obstacle perception.
	if activeConfig.EnableVisualSensors then
		sensorFolder = Instance.new("Folder")
		sensorFolder.Name = "SensoryDebugRays"
		sensorFolder.Parent = vehicleModel

		for idx = 1, SENSOR_BEAM_COUNT do
			local beamPart = Instance.new("Part")
			beamPart.Name = "SensorBeam_" .. idx
			beamPart.Anchored = true
			beamPart.CanCollide = false
			beamPart.CanQuery = false
			beamPart.CanTouch = false
			beamPart.CastShadow = false
			beamPart.Material = Enum.Material.Neon
			beamPart.Transparency = SENSOR_BEAM_TRANSPARENCY
			beamPart.Size = Vector3.new(SENSOR_BEAM_INITIAL_THICKNESS, SENSOR_BEAM_INITIAL_THICKNESS, SENSOR_BEAM_INITIAL_LENGTH)
			beamPart.Color = BEAM_COLOR_CLEAR
			beamPart.Parent = sensorFolder
			table.insert(visualParts, beamPart)
		end
	end

	-- Pre-construct and cache PhysicalProperties instances.
	-- Allocating `PhysicalProperties.new` in inner per-frame loops causes excessive garbage collection
	-- overhead in Luau. Pre-allocating immutable static and kinetic profiles eliminates this.
	local staticProps = PhysicalProperties.new(WHEEL_DENSITY, activeConfig.StaticFriction, WHEEL_ELASTICITY)
	local kineticProps = PhysicalProperties.new(WHEEL_DENSITY, activeConfig.KineticFriction, WHEEL_ELASTICITY)

	local wheelPartsList: {BasePart} = {
		frontRightWheel,
		wheelsFolder:WaitForChild("WheelFL"):WaitForChild("Wheel") :: BasePart,
		wheelsFolder:WaitForChild("WheelRR"):WaitForChild("Wheel") :: BasePart,
		wheelsFolder:WaitForChild("WheelRL"):WaitForChild("Wheel") :: BasePart,
	}

	-- Track initial slip states to only update CustomPhysicalProperties on state transitions.
	local initialSlipState: { [BasePart]: boolean } = {}
	for _, part in ipairs(wheelPartsList) do
		initialSlipState[part] = false
		part.CustomPhysicalProperties = staticProps
	end

	local self = setmetatable({
		_model = vehicleModel,
		_chassis = chassis,
		_seat = seat,
		_motors = {
			FR = engineFolder:WaitForChild("WheelFRMotor") :: MotorConstraint,
			FL = engineFolder:WaitForChild("WheelFLMotor") :: MotorConstraint,
			RR = engineFolder:WaitForChild("WheelRRMotor") :: MotorConstraint,
			RL = engineFolder:WaitForChild("WheelRLMotor") :: MotorConstraint,
		},
		_steeringRack = steeringFolder:WaitForChild("SteeringRack") :: PrismaticConstraint,
		_wheelParts = wheelPartsList,
		_wheelRadius = derivedRadius,
		_config = activeConfig,
		_engineSpeed = 0.0,
		_waypoints = {},
		_currentWaypointIndex = 1,
		_unstuckTimer = 0.0,
		_unstuckStage = UNSTUCK_STAGE_IDLE,
		_lastMovedPosition = chassis.Position,
		_stuckDetectionTimer = 0.0,
		_flippedTimer = 0.0,
		_rayParams = raycastParameters,
		_sensorFolder = sensorFolder,
		_sensorVisualParts = visualParts,
		_staticPhysicalProps = staticProps,
		_kineticPhysicalProps = kineticProps,
		_wheelSlippingState = initialSlipState,
		_heartbeatConnection = nil,
	}, AutonomousVehicleSystem)

	return self
end

--------------------------------------------------------------------------------
-- SENSORY PERCEPTION & OBSTACLE AVOIDANCE
--------------------------------------------------------------------------------

--[[
    _renderSensorRay
    Renders an individual raycast beam using an oriented, anchored neon cylinder/box.
    Orienting via CFrame.lookAt from the midpoint towards the target point creates
    a zero-overhead 3D trajectory visualizer without requiring complex Beam constraints.
]]
function AutonomousVehicleSystem:_renderSensorRay(index: number, startPos: Vector3, endPos: Vector3, hasHit: boolean)
	local beam = self._sensorVisualParts[index]
	if not beam then return end

	local delta = endPos - startPos
	local distance = delta.Magnitude
	if distance < MIN_RAY_RENDER_DISTANCE then return end

	-- Position beam at vector midpoint and orient along ray direction.
	local midpoint = startPos + (delta * RADIUS_FROM_DIAMETER_RATIO)
	beam.Size = Vector3.new(SENSOR_BEAM_ACTIVE_THICKNESS, SENSOR_BEAM_ACTIVE_THICKNESS, distance)
	beam.CFrame = CFrame.lookAt(midpoint, endPos)
	beam.Color = if hasHit then BEAM_COLOR_BLOCKED else BEAM_COLOR_CLEAR
end

--[[
    _castSensoryRays
    Performs tri-directional spatial raycasts (center boresight, left spread, right spread)
    emanating from a projection point located ahead of the front bumper.
    
    Offsetting the origin forward avoids near-field self-intersection with decorative
    mesh hoods, bull-bars, or attachments, while yawing the lateral vectors creates
    a predictive radar cone ahead of the chassis.
]]
function AutonomousVehicleSystem:_castSensoryRays(): (Vector3, RaycastResult?, RaycastResult?, RaycastResult?)
	local seatCFrame = self._seat.CFrame
	-- Calculate bumper origin offset along forward heading vector (-Z in vehicle space).
	local forwardOffsetDistance = (self._seat.Size.Z * RADIUS_FROM_DIAMETER_RATIO) + BUMPER_CLEARANCE_OFFSET_STUDS
	local rayOrigin = self._seat.Position + (seatCFrame.LookVector * forwardOffsetDistance)
	local sensorDistance = self._config.ObstacleDetectionRange
	local sensorAngleRad = math.rad(self._config.RaycastAngleDegrees)

	-- Compute directional raycast vectors with angular yaw offsets.
	local forwardVector = seatCFrame.LookVector * sensorDistance
	local leftVector = (seatCFrame * CFrame.Angles(0, sensorAngleRad, 0)).LookVector * sensorDistance
	local rightVector = (seatCFrame * CFrame.Angles(0, -sensorAngleRad, 0)).LookVector * sensorDistance

	local hitForward = Workspace:Raycast(rayOrigin, forwardVector, self._rayParams)
	local hitLeft = Workspace:Raycast(rayOrigin, leftVector, self._rayParams)
	local hitRight = Workspace:Raycast(rayOrigin, rightVector, self._rayParams)

	-- Update visual beams if enabled.
	if self._config.EnableVisualSensors and #self._sensorVisualParts == SENSOR_BEAM_COUNT then
		local endFwd = if hitForward then hitForward.Position else (rayOrigin + forwardVector)
		local endLeft = if hitLeft then hitLeft.Position else (rayOrigin + leftVector)
		local endRight = if hitRight then hitRight.Position else (rayOrigin + rightVector)

		self:_renderSensorRay(SENSOR_INDEX_FORWARD, rayOrigin, endFwd, hitForward ~= nil)
		self:_renderSensorRay(SENSOR_INDEX_LEFT, rayOrigin, endLeft, hitLeft ~= nil)
		self:_renderSensorRay(SENSOR_INDEX_RIGHT, rayOrigin, endRight, hitRight ~= nil)
	end

	return rayOrigin, hitForward, hitLeft, hitRight
end

--[[
    _getObstacleSteerOffset
    Evaluates spatial clearance from sensory ray hits and computes a normalized steering offset [-1.0, 1.0].
    
    Decision Logic:
    1. Direct Forward Obstruction:
       If the path directly ahead is blocked, the controller compares clearance on both
       flanks and steers sharply toward whichever direction has greater open distance.
    2. Lateral Asymmetric Obstruction:
       If only one flank detects an approaching obstacle, a repulsive proportional steering
       bias is generated inversely proportional to obstacle distance (closer obstacle = higher repulsion).
]]
function AutonomousVehicleSystem:_getObstacleSteerOffset(): number
	local rayOrigin, hitForward, hitLeft, hitRight = self:_castSensoryRays()
	local sensorDistance = self._config.ObstacleDetectionRange

	if hitForward then
		-- Path is directly obstructed: evaluate which side provides maximum free clearance.
		local distLeft = hitLeft and (hitLeft.Position - rayOrigin).Magnitude or sensorDistance
		local distRight = hitRight and (hitRight.Position - rayOrigin).Magnitude or sensorDistance
		return if distLeft > distRight then -OBSTACLE_HARD_AVOIDANCE_DEFLECTION else OBSTACLE_HARD_AVOIDANCE_DEFLECTION
	elseif hitLeft then
		-- Left obstacle detected: compute repulsive bias pushing vehicle rightward (+steer).
		local normalizedProximity = 1.0 - ((hitLeft.Position - rayOrigin).Magnitude / sensorDistance)
		return normalizedProximity * OBSTACLE_LATERAL_BIAS_WEIGHT
	elseif hitRight then
		-- Right obstacle detected: compute repulsive bias pushing vehicle leftward (-steer).
		local normalizedProximity = 1.0 - ((hitRight.Position - rayOrigin).Magnitude / sensorDistance)
		return -normalizedProximity * OBSTACLE_LATERAL_BIAS_WEIGHT
	end

	return 0.0
end

--------------------------------------------------------------------------------
-- TIRE TRACTION & DYNAMIC SLIP MECHANICS
--------------------------------------------------------------------------------

--[[
    _updateWheelFriction
    Applies classical Coulomb tire friction mechanics by detecting longitudinal wheel slip.
    
    Longitudinal slip occurs when there is a velocity disparity between the tire surface
    rim speed (v_rim = omega * r) and the wheel hub linear velocity across the ground (v_hub).
    When |v_hub - v_rim| exceeds SlipThreshold:
    - Tires lose static grip adhesion and break into kinetic sliding.
    - CustomPhysicalProperties.Friction is reduced from StaticFriction to KineticFriction.
    
    Optimization:
    Properties are only updated during state transitions (grip -> slip or slip -> grip)
    to prevent repetitive physics userdata allocation and property mutation overhead.
]]
function AutonomousVehicleSystem:_updateWheelFriction()
	local slipTolerance = self._config.SlipThreshold

	for _, wheelPart in ipairs(self._wheelParts) do
		local linearSpeed = wheelPart.AssemblyLinearVelocity.Magnitude
		local surfaceSpeed = self._wheelRadius * wheelPart.AssemblyAngularVelocity.Magnitude

		local isSlipping = math.abs(linearSpeed - surfaceSpeed) > slipTolerance
		local previousSlipState = self._wheelSlippingState[wheelPart]

		-- Only mutate physical properties if a transition in traction state occurred.
		if isSlipping ~= previousSlipState then
			self._wheelSlippingState[wheelPart] = isSlipping
			wheelPart.CustomPhysicalProperties = if isSlipping then self._kineticPhysicalProps else self._staticPhysicalProps
		end
	end
end

--------------------------------------------------------------------------------
-- POWERTRAIN, STEERING ACTUATION & PHYSICS INTEGRATION
--------------------------------------------------------------------------------

--[[
    _updatePhysics
    Executes the vehicle's dynamic physics simulation step:
    1. Resolves longitudinal chassis speed and detects counter-throttle active braking.
    2. Applies a low-pass filter to engine speed to model rotational inertia and drivetrain momentum.
    3. Calculates torque curves: high stall torque for low-speed breakaway, tapering at high speed.
    4. Calculates speed-damped steering sensitivity to prevent roll-over instability at high velocities.
    5. Actuates HingeConstraint wheel motors and PrismaticConstraint steering rack actuators.
]]
function AutonomousVehicleSystem:_updatePhysics(throttleInput: number, steeringInput: number, deltaTime: number)
	-- Project global chassis velocity into object space to isolate longitudinal motion (-Z).
	local relativeVelocity = self._chassis.CFrame:VectorToObjectSpace(self._chassis.AssemblyLinearVelocity)
	local forwardMph = studsPerSecondToMph(-relativeVelocity.Z)
	
	-- Determine target speed dictated by throttle input polarity.
	local targetMph = throttleInput * (if throttleInput >= 0 then self._config.ForwardMaxSpeed else self._config.ReverseMaxSpeed)

	-- Active Braking Detection:
	-- Throttle input directly opposing the current direction of motion constitutes an active brake request.
	local isBraking = throttleInput ~= 0 
		and math.abs(forwardMph) > BRAKE_DETECTION_SPEED_THRESHOLD_MPH 
		and math.sign(throttleInput) ~= math.sign(forwardMph)

	local accelerationRate = self._config.Acceleration

	if isBraking then
		accelerationRate = self._config.Braking
		targetMph = 0.0
	elseif math.abs(targetMph) < math.abs(self._engineSpeed) then
		-- Target velocity is closer to zero than current engine speed: apply coasting deceleration rate.
		accelerationRate = self._config.Deceleration
	end

	-- Low-pass filter smoothing engine speed towards actual ground velocity.
	-- Prevents motor target speed from diverging excessively during burnouts or external collisions.
	local correctionWeight = math.min(deltaTime * self._config.EngineSpeedCorrection, 1.0)
	self._engineSpeed += (forwardMph - self._engineSpeed) * correctionWeight

	-- Integrate acceleration/deceleration bounded by commanded target speed.
	if targetMph > self._engineSpeed then
		self._engineSpeed = math.min(self._engineSpeed + accelerationRate * deltaTime, targetMph)
	else
		self._engineSpeed = math.max(self._engineSpeed - accelerationRate * deltaTime, targetMph)
	end

	-- Electric Powertrain Torque Curve:
	-- High torque at low speed overcomes static tire friction and chassis inertia.
	-- Torque tapers linearly towards MaxSpeedTorque as speed approaches ForwardMaxSpeed.
	local speedRatio = math.clamp(math.abs(forwardMph) / self._config.ForwardMaxSpeed, 0.0, 1.0)
	local appliedTorque = self._config.MinSpeedTorque + ((self._config.MaxSpeedTorque - self._config.MinSpeedTorque) * speedRatio)

	-- Convert virtual engine speed (mph) to linear speed (studs/s), then divide by wheel radius
	-- to determine motor angular velocity (radians/s) required for constraint actuators: omega = v / r.
	local targetAngularVelocity = mphToStudsPerSecond(self._engineSpeed) / self._wheelRadius

	-- Command drivetrain motors: opposite polarity on left vs right side due to mirrored constraint axes.
	self._motors.FR.AngularVelocity = targetAngularVelocity
	self._motors.FR.MotorMaxTorque = appliedTorque
	self._motors.FL.AngularVelocity = -targetAngularVelocity
	self._motors.FL.MotorMaxTorque = appliedTorque
	self._motors.RR.AngularVelocity = targetAngularVelocity
	self._motors.RR.MotorMaxTorque = appliedTorque
	self._motors.RL.AngularVelocity = -targetAngularVelocity
	self._motors.RL.MotorMaxTorque = appliedTorque

	-- Speed-Damped Steering Transfer:
	-- At high speeds, lateral tire slip angles generate extreme cornering forces. Unattenuated steering
	-- at top speeds causes vehicle tripping, rollovers, or violent spinning. Steering sensitivity is
	-- progressively damped as velocity increases.
	local steeringDamping = math.max(1.0 - (speedRatio * self._config.SteeringReduction), MIN_STEERING_DAMPING_FLOOR)
	local effectiveSteering = steeringInput * steeringDamping

	-- Actuate PrismaticConstraint steering rack within physical mechanical endpoints.
	if effectiveSteering > 0 then
		self._steeringRack.TargetPosition = effectiveSteering * self._steeringRack.LowerLimit
	else
		self._steeringRack.TargetPosition = -effectiveSteering * self._steeringRack.UpperLimit
	end

	self:_updateWheelFriction()
end

--------------------------------------------------------------------------------
-- GYROSCOPIC STABILITY & ANTI-FLIP RECOVERY
--------------------------------------------------------------------------------

--[[
    _recoverIfFlipped
    Monitors vehicle attitude to detect rollovers and restores an upright orientation.
    
    Mathematical Principle:
    Seat.CFrame.UpVector.Y represents the dot product between the vehicle's local up-axis
    and the global vertical unit vector (0, 1, 0).
    - UpVector.Y = 1.0 indicates a perfectly level vehicle.
    - UpVector.Y <= 0.0 indicates a completely inverted vehicle (upside down).
    - UpVector.Y < 0.25 indicates a tilt angle exceeding 75.5 degrees.
    
    If sustained for over FLIPPED_PERSISTENCE_TIME_SECONDS:
    1. Extracts current yaw heading (LookVector in horizontal plane) while discarding roll and pitch.
    2. Nullifies all linear and angular momentum to eliminate post-reset rebound oscillations.
    3. Translates the chassis upward by RESPAWN_GROUND_CLEARANCE_STUDS to clear ground geometry.
]]
function AutonomousVehicleSystem:_recoverIfFlipped(deltaTime: number)
	if self._seat.CFrame.UpVector.Y < FLIPPED_INCLINATION_COSINE_THRESHOLD then
		self._flippedTimer += deltaTime
		if self._flippedTimer > FLIPPED_PERSISTENCE_TIME_SECONDS then
			local currentCFrame = self._seat.CFrame
			
			-- Reconstruct rotation matrix around the global Y-axis, preserving yaw while resetting roll and pitch.
			local yawAngle = math.atan2(currentCFrame.LookVector.X, currentCFrame.LookVector.Z) + math.pi
			local uprightRotation = CFrame.Angles(0, yawAngle, 0)

			-- Cancel residual momentum to prevent violent collision impulses upon repositioning.
			self._chassis.AssemblyLinearVelocity = Vector3.zero
			self._chassis.AssemblyAngularVelocity = Vector3.zero
			self._chassis.CFrame = CFrame.new(currentCFrame.Position + RESPAWN_GROUND_CLEARANCE_STUDS) * uprightRotation
			self._flippedTimer = 0.0
		end
	else
		self._flippedTimer = 0.0
	end
end

--------------------------------------------------------------------------------
-- LOCAL-SPACE KINEMATIC TRAJECTORY CONTROLLER
--------------------------------------------------------------------------------

--[[
    _calculateSteerAndThrottle
    Transforms global 3D target coordinates into local vehicle object-space to compute
    proportional steering and speed-adjusted throttle commands.
    
    Kinematic Mechanics:
    1. PointToObjectSpace transforms the target coordinate into the vehicle frame of reference.
    2. Horizontal displacement is evaluated using HORIZONTAL_PLANE_MASK (ignoring vertical differences).
    3. Heading error theta = math.atan2(localTarget.X, -localTarget.Z). In Roblox's coordinate space,
       -Z is forward and +X is right, so atan2 accurately yields the signed yaw deviation in radians.
    4. Heading error is scaled by STEER_NORMALIZATION_FACTOR (where 45 deg error yields 1.0 steer).
    5. Obstacle avoidance vector offset is blended with the pathfinding steering angle.
    6. Cornering Throttle Derating: Sharp steering inputs throttle down the engine to maintain
       centripetal acceleration within safe tire friction limits and prevent drift.
]]
function AutonomousVehicleSystem:_calculateSteerAndThrottle(targetPosition: Vector3): (number, number)
	local localTarget = self._seat.CFrame:PointToObjectSpace(targetPosition)
	local horizontalTarget = localTarget * HORIZONTAL_PLANE_MASK

	-- Nullify control commands if vehicle has arrived within the target deadzone radius.
	if horizontalTarget.Magnitude < TARGET_ARRIVAL_DEADZONE_STUDS then
		return NEUTRAL_THROTTLE, 0.0
	end

	-- Compute signed angular yaw error relative to forward heading (-Z).
	local headingAngleRad = math.atan2(localTarget.X, -localTarget.Z)
	local baseSteering = math.clamp(headingAngleRad * STEER_NORMALIZATION_FACTOR, MIN_NORMALIZED_STEER, MAX_NORMALIZED_STEER)
	
	-- Blend dynamic obstacle avoidance bias.
	local obstacleOffset = self:_getObstacleSteerOffset()
	local blendedSteering = math.clamp(baseSteering + obstacleOffset, MIN_NORMALIZED_STEER, MAX_NORMALIZED_STEER)

	local targetThrottle = FULL_FORWARD_THROTTLE

	-- Evaluate if target lies ahead in the front hemisphere (-Z in local object space).
	if localTarget.Z < FORWARD_HEMISPHERE_Z_BOUNDARY then
		local turnSeverity = math.abs(blendedSteering)
		if turnSeverity > SHARP_TURN_STEER_THRESHOLD then
			-- Reduce throttle through sharp turns to stay within the tire traction circle and avoid understeer.
			local corneringThrottle = 1.0 - (turnSeverity * CORNERING_THROTTLE_REDUCTION_FACTOR)
			targetThrottle = math.clamp(corneringThrottle, MIN_CORNERING_THROTTLE_FLOOR, FULL_FORWARD_THROTTLE)
		end
	else
		-- Target is behind the vehicle: execute a three-point turn or reverse maneuver if heading error is acute.
		targetThrottle = if math.abs(blendedSteering) > REVERSE_MANEUVER_STEER_THRESHOLD 
			then FULL_REVERSE_THROTTLE 
			else FULL_FORWARD_THROTTLE
		blendedSteering = -blendedSteering
	end

	return targetThrottle, blendedSteering
end

--------------------------------------------------------------------------------
-- DEADLOCK DETECTION & UNSTUCK EVASION STATE MACHINE
--------------------------------------------------------------------------------

--[[
    _handleUnstuckLogic
    Monitors translational displacement versus motor velocity to detect chassis entrapment.
    
    Entrapment Criterion:
    When commanded engine speed is high (|engineSpeed| > 12 mph) but chassis spatial displacement
    remains under 2.5 studs over a 2.0-second integration window, the chassis is classified
    as beached, wedged against geometry, or physically obstructed.
    
    Evasion Action:
    Engages a timed reverse maneuver with counter-steering to dislodge the chassis from the obstacle.
]]
function AutonomousVehicleSystem:_handleUnstuckLogic(deltaTime: number): (number?, number?)
	local currentTime = os.clock()
	local displacement = (self._chassis.Position - self._lastMovedPosition).Magnitude

	-- High motor velocity combined with negligible physical movement indicates immobilization.
	if displacement < STUCK_DISPLACEMENT_THRESHOLD_STUDS and math.abs(self._engineSpeed) > STUCK_MIN_ENGINE_SPEED_MPH then
		self._stuckDetectionTimer += deltaTime
		if self._stuckDetectionTimer > STUCK_DETECTION_WINDOW_SECONDS and self._unstuckStage == UNSTUCK_STAGE_IDLE then
			self._unstuckStage = UNSTUCK_STAGE_ACTIVE
			self._unstuckTimer = currentTime + UNSTUCK_MANEUVER_DURATION_SECONDS
			self._stuckDetectionTimer = 0.0
		end
	else
		self._stuckDetectionTimer = 0.0
		self._lastMovedPosition = self._chassis.Position
	end

	-- Active escape maneuver: command reverse throttle and counter-steer to pivot away from collision geometry.
	if self._unstuckStage == UNSTUCK_STAGE_ACTIVE then
		if currentTime < self._unstuckTimer then
			return UNSTUCK_ESCAPE_THROTTLE, UNSTUCK_ESCAPE_STEER
		else
			self._unstuckStage = UNSTUCK_STAGE_IDLE
		end
	end

	return nil, nil
end

--------------------------------------------------------------------------------
-- WAYPOINT PATROL & PATH NAVIGATION
--------------------------------------------------------------------------------

--[[
    SetWaypoints
    Configures an ordered series of 3D world coordinates for cyclic autonomous patrol.
]]
function AutonomousVehicleSystem:SetWaypoints(waypointList: {Vector3})
	self._waypoints = waypointList
	self._currentWaypointIndex = 1
end

--[[
    GetWaypoints
    Returns the active array of patrol waypoints.
]]
function AutonomousVehicleSystem:GetWaypoints(): {Vector3}
	return self._waypoints
end

--[[
    GetCurrentWaypointIndex
    Returns the index of the waypoint currently targeted by the navigation loop.
]]
function AutonomousVehicleSystem:GetCurrentWaypointIndex(): number
	return self._currentWaypointIndex
end

--[[
    _updateWaypointNavigation
    Advances through the patrol path when the vehicle enters the WaypointArrivalRadius.
    Calculates horizontal Euclidean distance (ignoring elevation variations) between
    the chassis centroid and the current target waypoint.
]]
function AutonomousVehicleSystem:_updateWaypointNavigation(): Vector3?
	local totalWaypoints = #self._waypoints
	if totalWaypoints == 0 then
		return nil
	end

	local currentTarget = self._waypoints[self._currentWaypointIndex]
	local horizontalDelta = (self._chassis.Position - currentTarget) * HORIZONTAL_PLANE_MASK
	local horizontalDistance = horizontalDelta.Magnitude

	if horizontalDistance < self._config.WaypointArrivalRadius then
		-- Advance cyclically to the next patrol coordinate.
		self._currentWaypointIndex = (self._currentWaypointIndex % totalWaypoints) + 1
		currentTarget = self._waypoints[self._currentWaypointIndex]
	end

	return currentTarget
end

--------------------------------------------------------------------------------
-- LIFECYCLE, FRAME UPDATE & CLEANUP
--------------------------------------------------------------------------------

--[[
    Update
    Per-frame simulation tick invoked on RunService.Heartbeat:
    1. Executes gyroscopic rollover check.
    2. Evaluates stuck detection state machine (overriding normal navigation if active).
    3. Calculates pursuit steering and throttle commands towards the active patrol waypoint.
    4. Writes commands to the VehicleSeat and simulates drivetrain and tire friction physics.
]]
function AutonomousVehicleSystem:Update(deltaTime: number)
	self:_recoverIfFlipped(deltaTime)

	local throttle, steer = NEUTRAL_THROTTLE, 0.0
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

--[[
    Start
    Binds the controller's Update step to RunService.Heartbeat for frame-synchronized simulation.
]]
function AutonomousVehicleSystem:Start()
	if self._heartbeatConnection then return end
	self._heartbeatConnection = RunService.Heartbeat:Connect(function(dt: number)
		self:Update(dt)
	end)
end

--[[
    Stop
    Temporarily suspends controller execution and neutralizes active motor velocities.
]]
function AutonomousVehicleSystem:Stop()
	if self._heartbeatConnection then
		self._heartbeatConnection:Disconnect()
		self._heartbeatConnection = nil
	end
	self._engineSpeed = 0.0
	self._motors.FR.AngularVelocity = 0.0
	self._motors.FL.AngularVelocity = 0.0
	self._motors.RR.AngularVelocity = 0.0
	self._motors.RL.AngularVelocity = 0.0
	self._seat.ThrottleFloat = NEUTRAL_THROTTLE
	self._seat.SteerFloat = 0.0
end

--[[
    Destroy
    Permanently terminates the controller instance, disconnects event listeners,
    removes debug sensory adornments from the Workspace, and resets hardware actuators.
]]
function AutonomousVehicleSystem:Destroy()
	self:Stop()
	if self._sensorFolder then
		self._sensorFolder:Destroy()
		self._sensorFolder = nil
	end
	self._sensorVisualParts = {}
	self._wheelSlippingState = {}
end

--------------------------------------------------------------------------------
-- SERVER BOOTSTRAP & DYNAMIC VEHICLE BINDING
--------------------------------------------------------------------------------

--[[
    bindControllerToModel
    Instantiates an AutonomousVehicleSystem controller for a designated vehicle model.
    Asynchronously loads patrol waypoints from Workspace.Waypoints if available.
]]
local function bindControllerToModel(targetModel: Model)
	local chassis = targetModel:WaitForChild("Chassis", CHASSIS_DISCOVERY_TIMEOUT_SECONDS) :: BasePart?
	if not chassis then return end

	local controller = AutonomousVehicleSystem.new(targetModel)

	-- Populate patrol coordinates from Workspace.Waypoints in an asynchronous thread.
	task.spawn(function()
		local waypointsFolder = Workspace:WaitForChild("Waypoints", WAYPOINTS_FOLDER_TIMEOUT_SECONDS)
		if waypointsFolder then
			local points: {Vector3} = {}
			local children = waypointsFolder:GetChildren()
			table.sort(children, function(a: Instance, b: Instance)
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

-- Automatic server initialization: bind to parent model or discover active models in Workspace.
local vehicleModel: Model? = script:FindFirstAncestorOfClass("Model")
if not (vehicleModel and vehicleModel:FindFirstChild("Chassis")) then
	for _, child in ipairs(Workspace:GetChildren()) do
		if child:IsA("Model") and child:FindFirstChild("Chassis") then
			vehicleModel = child
			break
		end
	end
end

if not vehicleModel then
	local activeFolder = Workspace:FindFirstChild("ActiveVehicles") or Workspace:WaitForChild("ActiveVehicles", ACTIVE_VEHICLES_FOLDER_TIMEOUT_SECONDS)
	if activeFolder then
		for _, child in ipairs(activeFolder:GetChildren()) do
			if child:IsA("Model") and child:FindFirstChild("Chassis") then
				vehicleModel = child
				break
			end
		end
		if not vehicleModel then
			activeFolder.ChildAdded:Connect(function(child)
				if child:IsA("Model") then
					bindControllerToModel(child)
				end
			end)
		end
	end
end

if vehicleModel then
	bindControllerToModel(vehicleModel)
end

return AutonomousVehicleSystem


