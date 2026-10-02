--!strict
-- Track & Circuit Environment Setup for Autonomous Vehicle Simulation Demonstration

local Workspace = game:GetService("Workspace")
local RunService = game:GetService("RunService")

-- 1. Create Track Environment Folder
local trackFolder = Workspace:FindFirstChild("TrackEnvironment")
if not trackFolder then
	trackFolder = Instance.new("Folder")
	trackFolder.Name = "TrackEnvironment"
	trackFolder.Parent = Workspace
end

-- 2. Create Waypoints Folder
local waypointsFolder = Workspace:FindFirstChild("Waypoints")
if not waypointsFolder then
	waypointsFolder = Instance.new("Folder")
	waypointsFolder.Name = "Waypoints"
	waypointsFolder.Parent = Workspace
end

-- 10-point cyclic patrol track coordinates
local waypointCoordinates = {
	Vector3.new(0, 1, 0),
	Vector3.new(120, 1, 0),
	Vector3.new(190, 1, 60),
	Vector3.new(190, 1, 180),
	Vector3.new(120, 1, 250),
	Vector3.new(0, 1, 250),
	Vector3.new(-120, 1, 250),
	Vector3.new(-190, 1, 180),
	Vector3.new(-190, 1, 60),
	Vector3.new(-120, 1, 0),
}

for i, pos in ipairs(waypointCoordinates) do
	local wpName = string.format("WP_%02d", i)
	local existingWp = waypointsFolder:FindFirstChild(wpName)
	if not existingWp then
		local wp = Instance.new("Part")
		wp.Name = wpName
		wp.Size = Vector3.new(10, 0.5, 10)
		wp.Position = pos + Vector3.new(0, 0.25, 0)
		wp.Anchored = true
		wp.CanCollide = false
		wp.Transparency = 0.4
		wp.Material = Enum.Material.Neon
		wp.Color = Color3.fromRGB(0, 170, 255)
		wp.Parent = waypointsFolder

		local bb = Instance.new("BillboardGui")
		bb.Size = UDim2.new(0, 60, 0, 26)
		bb.AlwaysOnTop = true
		bb.StudsOffset = Vector3.new(0, 4, 0)
		bb.Parent = wp

		local tl = Instance.new("TextLabel")
		tl.Size = UDim2.new(1, 0, 1, 0)
		tl.BackgroundTransparency = 1
		tl.TextColor3 = Color3.fromRGB(255, 255, 255)
		tl.TextStrokeTransparency = 0.2
		tl.TextStrokeColor3 = Color3.fromRGB(0, 50, 100)
		tl.TextScaled = true
		tl.Text = "WP " .. i
		tl.Font = Enum.Font.GothamBold
		tl.Parent = bb
	end
end

-- 3. Build Road Segments between consecutive waypoints
local roadsFolder = trackFolder:FindFirstChild("RoadSegments")
if not roadsFolder then
	roadsFolder = Instance.new("Folder")
	roadsFolder.Name = "RoadSegments"
	roadsFolder.Parent = trackFolder

	local numPoints = #waypointCoordinates
	for i = 1, numPoints do
		local p1 = waypointCoordinates[i]
		local p2 = waypointCoordinates[(i % numPoints) + 1]
		local midPoint = (p1 + p2) * 0.5
		local dist = (p2 - p1).Magnitude
		local dir = (p2 - p1).Unit

		-- Asphalt Road Slab
		local road = Instance.new("Part")
		road.Name = string.format("RoadSegment_%02d", i)
		road.Size = Vector3.new(24, 1, dist)
		road.CFrame = CFrame.lookAt(midPoint, midPoint + dir)
		road.Anchored = true
		road.Material = Enum.Material.Asphalt
		road.Color = Color3.fromRGB(35, 35, 38)
		road.TopSurface = Enum.SurfaceType.Smooth
		road.Parent = roadsFolder

		-- Curbs Left
		local curbL = Instance.new("Part")
		curbL.Name = "CurbL"
		curbL.Size = Vector3.new(2, 1.8, dist)
		curbL.CFrame = road.CFrame * CFrame.new(-12, 0.4, 0)
		curbL.Anchored = true
		curbL.Material = Enum.Material.Concrete
		curbL.Color = Color3.fromRGB(230, 75, 75)
		curbL.Parent = roadsFolder

		-- Curbs Right
		local curbR = Instance.new("Part")
		curbR.Name = "CurbR"
		curbR.Size = Vector3.new(2, 1.8, dist)
		curbR.CFrame = road.CFrame * CFrame.new(12, 0.4, 0)
		curbR.Anchored = true
		curbR.Material = Enum.Material.Concrete
		curbR.Color = Color3.fromRGB(230, 75, 75)
		curbR.Parent = roadsFolder
	end
end

-- 4. Create Obstacles on Track
local obstaclesFolder = Workspace:FindFirstChild("Obstacles")
if not obstaclesFolder then
	obstaclesFolder = Instance.new("Folder")
	obstaclesFolder.Name = "Obstacles"
	obstaclesFolder.Parent = Workspace
end

local obstacleConfigs = {
	{pos = Vector3.new(60, 3, 0), size = Vector3.new(7, 5, 7), name = "Barrier_Straight_1"},
	{pos = Vector3.new(190, 3, 120), size = Vector3.new(9, 5, 9), name = "Barrier_Turn_1"},
	{pos = Vector3.new(0, 3, 250), size = Vector3.new(7, 5, 7), name = "Barrier_Straight_2"},
	{pos = Vector3.new(-190, 3, 120), size = Vector3.new(9, 5, 9), name = "Barrier_Turn_2"},
}

for _, obs in ipairs(obstacleConfigs) do
	if not obstaclesFolder:FindFirstChild(obs.name) then
		local part = Instance.new("Part")
		part.Name = obs.name
		part.Size = obs.size
		part.Position = obs.pos
		part.Anchored = true
		part.Material = Enum.Material.DiamondPlate
		part.Color = Color3.fromRGB(235, 55, 55)
		part.Parent = obstaclesFolder
	end
end

-- 5. Position Vehicle at Starting Waypoint (WP_01)
task.spawn(function()
	task.wait(1.5)
	local vehicle = Workspace:FindFirstChild("Vehicle") or Workspace:FindFirstChildOfClass("Model")
	if vehicle and vehicle:FindFirstChild("Chassis") then
		local startPos = waypointCoordinates[1]
		local secondPos = waypointCoordinates[2]
		local heading = CFrame.lookAt(startPos + Vector3.new(0, 4, 0), secondPos + Vector3.new(0, 4, 0))
		vehicle:PivotTo(heading)
	end
end)

print("[TrackEnvironment] Circuit, Waypoints, and Obstacles initialized successfully!")
