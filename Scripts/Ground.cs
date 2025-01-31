using Godot;
using System;
using System.IO;
using System.Net.Http;
using System.Threading.Tasks;
using Newtonsoft.Json;
using System.Collections.Generic;



public partial class Ground : Node3D
{
	[Export]
	private Vehicle vehicle;

	[Export]
	private SubViewport subViewport;
	[Export]
	private SubViewport subViewportRight;
	[Export]
	private SubViewport subViewportLeft;
	[Export]
	private SubViewport subViewportBack;

	[Export]
	private SubViewport subViewportMidLeft;

	private Camera3D frontCamera;
	private Camera3D backCamera;
	private Camera3D rightCamera;
	private Camera3D leftCamera;

	private Camera3D leftMidCamera;
	[Export]
	private Camera3D SvpFrontCamera;
	[Export]
	private Camera3D SvpRightCamera;
	[Export]
	private Camera3D SvpLeftCamera;

	[Export]
	private Camera3D SvpbackCamera;

	[Export]
	private Camera3D SvpLeftMidCamera;

	public static Ground ground;
	
	private static int photoNumber = 0;
	private Vector3 TargetVelocity = new Vector3(0, 0, 0);
	private const float DecelerationRate = 0.05f;
	private bool record = true;
	private float timeElapsed = 0f;
	private float yoloTimeElapsed = 0f;
	private bool alreadyDetectStopSign = false;
	private float giveTime = 2f;
	// private const string ServerUrl = "https://11f5-34-45-12-177.ngrok-free.app";

	private const string ServerUrl = "http://127.0.0.1:4567"; // Flask server URL

	 // Dataset path for Dave-2 model
	private string savePath = "/Use you own path/Dave2/IMG";
	private string logFilePath = "/Use you own path/Dave2/annotations.csv";

	// Dataset path for parking spaces YOLO model
	private string parkingDatsasetPath = "/Use you own path/parkingDataset";


	private bool flag = true;

	private float realStopSignHeight = 1.8f;

	private float stopTime = 0f;

	private float laneWidth = 4f;
	private bool isChangingLane = false;
	private Vector3 position;
	private Vector3 roadDirection;
	private Vector3 lateralDirection;
	private Vector3 endPosition;
	private Vector3 controlPoint;
	private Vector3 slotDirection;
	private float changeLaneTime = 0.0f;
	private float transitionTime = 2.0f;

	private List<Vector3> bezierPoints = new List<Vector3>();
	private int currentPointIndex = 0;

	private List<RayCast3D> leftLanerayCast3s = new List<RayCast3D>();
	private List<RayCast3D> rightLanerayCast3s = new List<RayCast3D>();
	private List<RayCast3D> parkLanerayCast3s = new List<RayCast3D>();
	private bool start = false;
	private bool startRightLaneChange = false;
	private bool startLeftLaneChange = false;
	private bool startParking = false, searchForSlot = false, planStraightLine = false;

	private bool autonomousMode = true;
	private float parkingSlotSearchTime = 0f;

	private float sharpness = 4;

	private bool done = true;

	private System.Collections.Generic.Dictionary<string, float> detections = new System.Collections.Generic.Dictionary<string, float>();

	private float detectionTime = 0;
	bool val = false;
	[Export] private Label detection;
	string label_string = ""; 


	[Export] private Label label;
	[Export] private Label speedLabel;

	Midpoint midpoint;
	private List<Vector3> trackPositions = new List<Vector3>()
	{
		new Vector3(677.91f, 0.829f, -111.102f),
		new Vector3(1033.413f,1.11f, -81.679f),
		new Vector3(893.984f, 0.231f, -352.851f),
		new Vector3(677.73f, 1.086f, -348.498f),
		new Vector3(395.652f, 0.607f, -854.168f),
		new Vector3(898.942f, 0.221f, -559.462f),
		new Vector3(969.187f, 0.414f, -447.453f)
		// new Vector3(1263.573f, 0.421f, -718.8f)
	};
	private List<Vector3> trackRotations = new List<Vector3>()
	{
		new Vector3(0, 8.5f, 0),
		new Vector3(0,-84.6f,0),
		new Vector3(0,177.9f,0),
		new Vector3(0, 161.4f, 0),
		new Vector3(0, -88.5f, 0),
		new Vector3(0, 178.4f, 0),
		new Vector3(0, 86.1f, 0)
		// new Vector3(0, -91.4f, 0)
	};

	private bool YoloActive = true, LCActive = false, ParkingActive = false;

	[Export]private PathFollow3D car1;
	[Export]private PathFollow3D car2;
	[Export]private PathFollow3D car3;
	[Export]private PathFollow3D car4;
	[Export]private PathFollow3D car5;
	[Export]private PathFollow3D car6;
	[Export]private PathFollow3D car7;
	
	
	////////////////////////////////////////////// Prepare tracks according to user choice //////////////////////////////////////////////////
	private void PrepareTrack()
	{
		if(Global.SelectedTrack == 1)
		{
			YoloActive = false;
			LCActive = true;
		}
		else if(Global.SelectedTrack == 2)
		{
			YoloActive = false;
			LCActive = false;
		}
				
		else if(Global.SelectedTrack == 3)
		{
			YoloActive = false;
			LCActive = true;
		}
		else if(Global.SelectedTrack == 4)
		{
			YoloActive = true;
			vehicle.MaxEngineForce = 60;
			vehicle.RotationSpeed = 20f;
			vehicle.MaxRotation = -35;
			vehicle.SetTargetSpeed(120);
		}
		else if(Global.SelectedTrack == 5)
		{
			YoloActive = true;
			vehicle.MaxEngineForce = 50;
			vehicle.RotationSpeed = 5f;
			vehicle.MaxRotation = -30;
		}
		else if(Global.SelectedTrack == 6)
		{
			// YoloActive = true;
			LCActive = true;
		}
		else if(Global.SelectedTrack == 7)
		{
			ParkingActive = true;
			vehicle.SetTargetSpeed(5);
			// YoloActive = true;
		}
		vehicle.GlobalPosition = trackPositions[Global.SelectedTrack-1];
		Vector3 newRotation = new Vector3(Mathf.DegToRad(trackRotations[Global.SelectedTrack-1].X), Mathf.DegToRad(trackRotations[Global.SelectedTrack-1].Y),Mathf.DegToRad(trackRotations[Global.SelectedTrack-1].Z));
		vehicle.GlobalRotation = newRotation;	
	}
	public override void _Ready()
	{

		ground = this;
		frontCamera = vehicle.GetChild<Camera3D>(7);
		rightCamera = vehicle.GetChild<Camera3D>(9);
		leftCamera = vehicle.GetChild<Camera3D>(10);
		leftMidCamera = vehicle.GetChild<Camera3D>(11);

		subViewport.RenderTargetUpdateMode = SubViewport.UpdateMode.Always;
		subViewportRight.RenderTargetUpdateMode = SubViewport.UpdateMode.Always;
		subViewportLeft.RenderTargetUpdateMode = SubViewport.UpdateMode.Always;
		subViewportMidLeft.RenderTargetUpdateMode = SubViewport.UpdateMode.Always;

		if (!Directory.Exists(savePath))
		{
			Directory.CreateDirectory(savePath);
		}
		if (!Directory.Exists(parkingDatsasetPath))
		{
			Directory.CreateDirectory(parkingDatsasetPath);
		}

		// Create or open the log file
		if (!File.Exists(logFilePath))
		{
			File.WriteAllText(logFilePath, "center,left,right,steering,throttle,reverse,speed\n"); // Add CSV header
		}
		if(Global.GameMode == "Training")
		{
			label.Text = "Training";
			detection.Text = "";
		}
		else
		{
			label.Text = "Autonomous";
			detection.Text = "";
		}

		PrepareTrack();
	}

	private void PlanStraightLine(float offset)
	{
		Vector3 targetPosition = vehicle.GlobalPosition + vehicle.GlobalTransform.Basis.Z.Normalized() * offset;
		GD.Print("vehicle.GlobalPosition, ", vehicle.GlobalPosition);
		GD.Print("targetPosition: ", targetPosition);
		Vector3 controlPoint = (vehicle.GlobalPosition + targetPosition)/2;
		Node3D currentRoad = DetectCurrentRoad(vehicle.GlobalPosition);
		bezierPoints = GenerateBezierPoints(vehicle.GlobalPosition, controlPoint, targetPosition, targetPosition, 5, currentRoad, parkLanerayCast3s);
		// for(int i = 3; i < bezierPoints.Count; i++)
		// {
		// 	// Create a new MeshInstance3D for the point
		// 	MeshInstance3D meshInstance = new MeshInstance3D();

		// 	// Set the mesh (e.g., a sphere)
		// 	SphereMesh sphereMesh = new SphereMesh();
		// 	sphereMesh.Radius = 0.1f; // Adjust the size of the sphere
		// 	sphereMesh.Height = 0.2f;
		// 	meshInstance.Mesh = sphereMesh;

		// 	// Position the mesh at the current Bezier point
		// 	meshInstance.GlobalTransform = new Transform3D(Basis.Identity, bezierPoints[i]);

		// 	// Add the mesh instance to the scene
		// 	AddChild(meshInstance);
		// }
		currentPointIndex = 3;

	}
	private Vector3 CalculateSlotCenter(Vector3 carPosition, Vector3 slotDirection, float slotWidth, float slotHeight, int side, string dir)
	{
		// Perpendicular direction to the slot (90-degree rotation from slotDirection)
		Vector3 perpendicularSlotDirection = new Vector3(-slotDirection.Z, slotDirection.Y, slotDirection.X);

		// Calculate slot center
		int val = side;
		if(dir == "right")
		{
			val = -side;
		}
		Vector3 slotCenter = carPosition
							- (slotDirection * (slotHeight / 2)* val) // Reverse direction (behind the car)
							+ (perpendicularSlotDirection * (slotWidth / 2) * side); // Shift right or left based on side

		return slotCenter;
}

	private void StartParking(int side, string dir)
	{
		position = vehicle.GlobalPosition;
		slotDirection = vehicle.GlobalTransform.Basis.X.Normalized();
		Vector3 slotCenter = CalculateSlotCenter(position, slotDirection, 8, 11, side, dir);
		Vector3 controlP = (position + slotCenter)/2;
		

		Vector3 perpendicularSlotDirection = new Vector3(-slotDirection.Z, slotDirection.Y, slotDirection.X); // 90-degree rotation

		float offsetMagnitude = 3f;
		controlP = controlP + (perpendicularSlotDirection * offsetMagnitude) * side;

		Node3D currentRoad = DetectCurrentRoad(position);

		// Generate the cubic bezier curve
		GD.Print("position: ", position);
		GD.Print("controlP: ", controlP);
		GD.Print("slotCenter: ", slotCenter);
		foreach(var ray in parkLanerayCast3s)
		{
			ray.QueueFree();
		}
		parkLanerayCast3s = new List<RayCast3D>();
		bezierPoints = GenerateBezierPoints(position, controlP, slotCenter, slotCenter, 40, currentRoad, parkLanerayCast3s);
		// for(int i = 10; i < bezierPoints.Count; i++)
		// {
		// 	GD.Print("in hereee");
		// 	// Create a new MeshInstance3D for the point
		// 	MeshInstance3D meshInstance = new MeshInstance3D();

		// 	// Set the mesh (e.g., a sphere)
		// 	SphereMesh sphereMesh = new SphereMesh();
		// 	sphereMesh.Radius = 0.1f; // Adjust the size of the sphere
		// 	sphereMesh.Height = 0.2f;
		// 	meshInstance.Mesh = sphereMesh;

		// 	// Position the mesh at the current Bezier point
		// 	meshInstance.GlobalTransform = new Transform3D(Basis.Identity, bezierPoints[i]);

		// 	// Add the mesh instance to the scene
		// 	AddChild(meshInstance);
		// }
		
		currentPointIndex = 1;

	}
	
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////          The following methods creates the Bezier points            /////////////////////////////////////////////
///////////////////////////////////////////       We use time variable t to get the the point at each unit of time          ///////////////////////////////////////////
///////////////////////////////////////////        then we use pure persuit algorithm in order compute the steering angle the car need to take          ///////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///

	private Vector3 AdjustPointToSurface(Vector3 point, Node3D roadSurface)
	{
		var space = roadSurface.GetWorld3D().DirectSpaceState;
		var rayOrigin = point + new Vector3(0, 10, 0);
		var rayEnd = point + new Vector3(0, -50, 0); 

		var query = new PhysicsRayQueryParameters3D
		{
			From = rayOrigin,
			To = rayEnd,
			CollideWithAreas = true,
			CollideWithBodies = true
		};

		var result = space.IntersectRay(query);

		if (result.Count > 0)
		{
			Vector3 res = (Vector3)result["position"];
			res.Y += 0.1f;
			
			return res;
		}

		return point;
	}


	public Node3D DetectCurrentRoad(Vector3 vehiclePosition)
	{
		var space = GetWorld3D().DirectSpaceState;
		var rayOrigin = vehiclePosition + new Vector3(3, 10, 0);  
		var rayEnd = vehiclePosition + new Vector3(3, -50, 0);

		var query = new PhysicsRayQueryParameters3D
		{
			From = rayOrigin,
			To = rayEnd,
			CollideWithAreas = true,
			CollideWithBodies = true
		};

		var result = space.IntersectRay(query);

		if (result.Count > 0)
		{
			Node3D detectedRoad = (Node3D)result["collider"];
			return detectedRoad;
		}


		return null;
	}


	private Vector3 CalculateBezierPoint(float t, Vector3 p0, Vector3 p1, Vector3 p2)
	{
		float u = 1 - t;
		float tt = t * t;
		float uu = u * u;

		Vector3 point = (uu * p0) + (2 * u * t * p1) + (tt * p2);
		return point;
	}


	public float CalculateSteeringAngle(Vector3 currentPosition, Vector3 nextPosition, Vector3 carForward, float wheelBase)
	{
		
		Vector3 toNextPosition = nextPosition - currentPosition;

		float lookaheadDistance = toNextPosition.Length();

		Vector3 direction = toNextPosition.Normalized();

		// Calculate the angle between the car's forward direction and the direction to the next position
		float dotProduct = carForward.Dot(direction);
		dotProduct = Mathf.Clamp(dotProduct, -1f, 1f);

		float angleToTarget = Mathf.Acos(dotProduct);

		Vector3 crossProduct = carForward.Cross(direction);

		// Determine sign
		if (crossProduct.Y < 0)
		{
			angleToTarget = -angleToTarget;
		}

		// Calculate the steering angle using the Pure Pursuit formula
		float steeringAngle = Mathf.Atan((2 * wheelBase * Mathf.Sin(angleToTarget)) / lookaheadDistance);

		// Return the steering angle in radians
		return steeringAngle;
	}

	private void StartLaneChange(int direction, List<RayCast3D> rayCast3Ds)
	{
		roadDirection = vehicle.GlobalTransform.Basis.Z.Normalized();
		position = vehicle.GlobalTransform.Origin + 3 * roadDirection;
		lateralDirection = roadDirection.Cross(Vector3.Up).Normalized();
		

		// Adjust lane width for smooth lane change
		endPosition = position + (lateralDirection + roadDirection * sharpness * direction) * laneWidth * direction;
		Vector3 endRayPosition = position + (lateralDirection + roadDirection * sharpness * direction) * 3* laneWidth * direction;


		Vector3 controlPoint = (position + endPosition)/2;
		Vector3 curveOffset = (lateralDirection * laneWidth* direction * 0.5f) + (Vector3.Up * 2f);  // Adjust for arc effect
		controlPoint += curveOffset;	

		changeLaneTime = 0.0f;
		isChangingLane = true;

		// Detect the road 
		Vector3 checkPosition = position + (lateralDirection*1.5f * direction * laneWidth);
		Node3D currentRoad = DetectCurrentRoad(checkPosition);

		// Generate the cubic bezier curve
		bezierPoints = GenerateBezierPoints(position, controlPoint, endPosition, endRayPosition, 40, currentRoad, rayCast3Ds);

		currentPointIndex = 10;

	}
	public List<Vector3> GenerateBezierPoints(Vector3 startPoint, Vector3 controlPoint, Vector3 endPoint, Vector3 endRayPosition, int numPoints, Node3D roadSurface, List<RayCast3D> rayCast3Ds)
	{
		bezierPoints.Clear();

		// Generate points along the cubic Bezier curve
		for (int i = 0; i <= numPoints; i++)
		{
			float t = (float)i / numPoints;
			Vector3 point = CalculateBezierPoint(t, startPoint, controlPoint, endPoint);
			
			// Adjust the point height based on the road surface
			point = AdjustPointToSurface(point, roadSurface);
			bezierPoints.Add(point);
		}


		List<Vector3> rayPoints = new List<Vector3>();


		for (int i = 0; i <= numPoints; i++)
		{
			float t = (float)i / numPoints;
			Vector3 point = CalculateBezierPoint(t, startPoint, controlPoint, endRayPosition);
			
			// Adjust the point height based on the road surface
			point = AdjustPointToSurface(point, roadSurface);
			rayPoints.Add(point);
		}
		if(rayCast3Ds.Count > 0)
		{		
		// Create RayCast3D nodes from each point to the next
			for (int i = 0; i < bezierPoints.Count - 1; i++)
			{
				rayCast3Ds[i].GlobalPosition = rayPoints[i];
				Vector3 direction = rayPoints[i + 1] - rayPoints[i];
				rayCast3Ds[i].TargetPosition = direction;  
				
			}
		} 
		else
		{
			for (int i = 0; i < bezierPoints.Count - 1; i++)
			{
				RayCast3D rayCast = new RayCast3D();

				AddChild(rayCast);
				rayCast.GlobalPosition = rayPoints[i];
				Vector3 direction = rayPoints[i + 1] - rayPoints[i];
				rayCast.TargetPosition = direction;  
				rayCast.Enabled = true;
				rayCast3Ds.Add(rayCast);
			}
		}      

		return bezierPoints;
	}

	private void TurnOffOrOneSensors(List<RayCast3D> rayCast3Ds, bool val)
	{
		foreach(RayCast3D ray in rayCast3Ds)
		{
			ray.Visible = val;
		}
	}

	// this function manily respoible on visiualizing the trajectory of the curve
	private void UpdateSensors(int lane)
	{
		List<RayCast3D> sensors;
		roadDirection = vehicle.GlobalTransform.Basis.Z.Normalized();
		position = vehicle.GlobalTransform.Origin + 3 * roadDirection;
		lateralDirection = roadDirection.Cross(Vector3.Up).Normalized();

		if(lane == 0) // same lane
		{
			vehicle.laneChangeRay.Visible = true;
			TurnOffOrOneSensors(leftLanerayCast3s,false);
			TurnOffOrOneSensors(rightLanerayCast3s, false);
		}
		else if(lane == 1) // right lane
		{
			sensors = rightLanerayCast3s;
			vehicle.laneChangeRay.Visible = false;
			Vector3 checkPosition = position + (lateralDirection*1f * 1 * laneWidth);
			Node3D currentRoad = DetectCurrentRoad(checkPosition);
			if(currentRoad.Name != "road_mesh_col")
			{
				GD.Print("lane: 1, ", currentRoad.Name);
				TurnOffOrOneSensors(rightLanerayCast3s, false);
				StartLaneChange(-1, leftLanerayCast3s);
				TurnOffOrOneSensors(leftLanerayCast3s, true);
				TurnOffOrOneSensors(rightLanerayCast3s, false);

			}
			else
			{
				StartLaneChange(1, rightLanerayCast3s);
				TurnOffOrOneSensors(rightLanerayCast3s, true);
				TurnOffOrOneSensors(leftLanerayCast3s,false);
			}
			

		}
		else // left lane
		{
			Vector3 checkPosition = position + (lateralDirection*1f * -1 * laneWidth);
			Node3D currentRoad = DetectCurrentRoad(checkPosition);
			sensors = leftLanerayCast3s;
			vehicle.laneChangeRay.Visible = false;
			if(currentRoad.Name != "road_mesh_col")
			{
				GD.Print("lane: -1, ", currentRoad.Name);
				TurnOffOrOneSensors(leftLanerayCast3s, false);
				StartLaneChange(1, rightLanerayCast3s);
				TurnOffOrOneSensors(rightLanerayCast3s, true);

			}
			else
			{
				StartLaneChange(-1, leftLanerayCast3s);
				TurnOffOrOneSensors(leftLanerayCast3s, true);
			}
		}

		
	}

	private bool IsVehicleCentered(Node3D road, Vehicle vehicle)
	{
		Vector3 roadDirection = road.GlobalTransform.Basis.Z.Normalized();
		roadDirection.X = Mathf.Abs(roadDirection.X);
		roadDirection.Y = Mathf.Abs(roadDirection.Y); 
		roadDirection.Z = Mathf.Abs(roadDirection.Z);  
		Vector3 vehicleDirection = vehicle.GlobalTransform.Basis.Z.Normalized();
		vehicleDirection.X = Mathf.Abs(vehicleDirection.X);
		vehicleDirection.Y = Mathf.Abs(vehicleDirection.Y); 
		vehicleDirection.Z = Mathf.Abs(vehicleDirection.Z);  

		float angleDifference = Mathf.RadToDeg(vehicleDirection.AngleTo(roadDirection));

		return angleDifference < 4f;
	}
// 	private Vector3 MapPointToWorld(Vector2 slotCenter)
// 	{
//    float normX = (slotCenter.X / 512) * 2.0f - 1.0f;
// 		float normY = 1.0f - (slotCenter.Y / 512) * 2.0f; // Invert Y-axis

// 		// Get camera properties
// 		float fov = Mathf.DegToRad(leftMidCamera.Fov);  // Convert FOV to radians
// 		float aspectRatio = (512) / (512-350);
// 		float nearPlane = leftMidCamera.Near;  // Near clip plane distance

// 		// Compute the ray direction in camera space
// 		float tanFov = Mathf.Tan(fov / 2.0f);
// 		Vector3 rayDirection = new Vector3(
// 			normX * aspectRatio * tanFov,
// 			normY * tanFov,
// 			1.0f // In Godot, the camera looks along the negative Z-axis
// 		).Normalized();

// 		// Transform the ray direction to world space
// 		Vector3 rayOrigin = leftMidCamera.GlobalTransform.Origin;
// 		Vector3 worldRayDirection = leftMidCamera.GlobalTransform.Basis * rayDirection;

// 		// Solve for intersection with the ground plane (Y = 0)
// 		float groundY = 0.0f;
// 		float t = (groundY - rayOrigin.Y) / worldRayDirection.Y;
// 		Vector3 worldPosition = rayOrigin + worldRayDirection * t;

// 		GD.Print($"Converted World Position: {worldPosition}");
// 		return worldPosition;
// 	}
	public override void _Process(double delta)
	{
		speedLabel.Text = Mathf.FloorToInt(vehicle.GetSpeed()*3.6f).ToString();
		timeElapsed += (float)delta;
		yoloTimeElapsed += (float)delta;
		detectionTime += (float)delta;
		parkingSlotSearchTime += (float)delta;
		
		SvpFrontCamera.GlobalTransform = frontCamera.GlobalTransform;

		if(Global.GameMode == "Training")
		{
			if(Input.IsActionPressed("record"))
			{
				record = !record;
			}
			if(timeElapsed > 0.1f)
			{
				timeElapsed = 0;
				if(record)
				{
					SvpRightCamera.GlobalTransform = rightCamera.GlobalTransform;
					SvpLeftCamera.GlobalTransform = leftCamera.GlobalTransform;
					CaptureAndLogData();
				}
			}
			// if(Input.IsActionPressed("record"))
			// {
			// 	SvpLeftMidCamera.GlobalTransform = leftMidCamera.GlobalTransform;
			// 	// CaptureParkingSpaceData();
			// 	// CapturetTrafficSignData();
			// 	CaptureAndLogData();
			// }
		}
		else
		{
			if(LCActive)
			{
				if(vehicle.laneChangeRay.IsColliding() && done)
				{
					if(IsVehicleCentered(DetectCurrentRoad(vehicle.GlobalRotation), vehicle))
					{
						float distance = vehicle.GlobalPosition.DistanceTo(vehicle.laneChangeRay.GetCollisionPoint());
						// handle sharpness of turns with different distaces
						if(distance >= 15)
						{
							sharpness = 4;
						}
						else
						{
							sharpness = 2;
							vehicle.SetTargetSpeed(5);
						}
						
						if(vehicle.rightRay.IsColliding() && vehicle.laneChangeRay.IsColliding() && vehicle.leftRay.IsColliding()) // braking
						{
							vehicle.BrakeForce = Mathf.Lerp(vehicle.BrakingForce, vehicle.BrakeForce, 1.0f * (float)delta);
							vehicle.SetTargetSpeed(0);
						}
						else if(vehicle.rightRay.IsColliding() && vehicle.laneChangeRay.IsColliding() && !vehicle.leftRay.IsColliding() && !vehicle.leftSide.IsColliding())
						{
							label.Text = "Lane Changing Mode";
							startLeftLaneChange = true;
							done = false;
							UpdateSensors(-1);

						}
						else if(!vehicle.rightRay.IsColliding() && vehicle.laneChangeRay.IsColliding() && vehicle.leftRay.IsColliding() && !vehicle.rightSide.IsColliding())
						{
							label.Text = "Lane Changing Mode";
							startRightLaneChange = true;
							done = false;
							UpdateSensors(1);
						}
						
						else if(!vehicle.rightRay.IsColliding() && vehicle.laneChangeRay.IsColliding()&& !vehicle.rightSide.IsColliding())
						{
							GD.Print("in here");
							label.Text = "Lane Changing Mode";
							startRightLaneChange = true;
							done = false;
							UpdateSensors(1);
						}
						else if(!vehicle.leftRay.IsColliding() && vehicle.laneChangeRay.IsColliding() && !vehicle.leftSide.IsColliding())
						{
							label.Text = "Lane Changing Mode";
							startLeftLaneChange = true;
							done = false;
							UpdateSensors(-1);
						}
					}
				}

				if (startRightLaneChange || startLeftLaneChange)
				{
					if(currentPointIndex == bezierPoints.Count-5)
					{
						startRightLaneChange = false;
						startLeftLaneChange = false;
						done = true;
						UpdateSensors(0);
						vehicle.SetTargetSpeed(20);
						label.Text = "Autonomous Mode";
					}
					else
					{
						Vector3 nextPoint = bezierPoints[currentPointIndex];

						float steeringAngle = CalculateSteeringAngle(vehicle.GlobalPosition, nextPoint, vehicle.Transform.Basis.Z,2f);
						vehicle.SetSteeringAngle(steeringAngle);

						if (vehicle.GlobalTransform.Origin.DistanceTo(nextPoint) <3f)
						{
							currentPointIndex++;				
						}
					}			
				}
			}
			if(yoloTimeElapsed > 0.1f && !startLeftLaneChange && !startRightLaneChange)
			{
				yoloTimeElapsed = 0;
				Image image = subViewport.GetViewport().GetTexture().GetImage();
				if(timeElapsed > 0.15f && autonomousMode)
				{
					timeElapsed = 0f;
					// Send the image to the Flask server for steering prediction
					Task.Run(async () =>
					{
						try
						{
							
							if(!vehicle.IsStopping)
							{
								var predictionResponse = await SendImageToServer(image);
								if (predictionResponse != null)
								{
									ApplyPrediction(predictionResponse.Steering);
								}
							}
						}
						catch (Exception ex)
						{
							GD.PrintErr($"Error sending image to server: {ex.Message}");
						}
					});
				}
				if(vehicle.forwardRay.IsColliding())
				{

					// this code in case pf use stop sign
					// if(alreadyDetectStopSign && giveTime > 0)
					// {
					// 	giveTime -= (float)delta;
					// }
					// else
					// {
					// 	alreadyDetectStopSign = false;
					// 	giveTime = 0;
					// }
					// if(stopTime == 0f)
					// {
					// 	vehicle.IsStopping = false;
					// }
					// else
					// {
					// 	stopTime -= (float)delta;
					// 	if(stopTime < 0)
					// 	{
					// 		stopTime = 0f;
					// 	}
					// }

					// Send the image to the Flask server for yolo detection
					if(!vehicle.IsStopping && !alreadyDetectStopSign && YoloActive)
					{

						Task.Run(async () =>
						{
							try
							{
								if(Global.SelectedTrack == 4)
								{
									string yoloResponse = await SendYoloRequestDirectly(image, $"{ServerUrl}/predict_yolo_1");
									label_string = ParseYoloResponse(yoloResponse);
								}
								else if(Global.SelectedTrack == 5)
								{
									string yoloResponse = await SendYoloRequestDirectly(image, $"{ServerUrl}/predict_yolo_2");
									label_string = ParseYoloResponse2(yoloResponse);
								}
								
							}
							catch (Exception ex)
							{
								GD.PrintErr($"Error sending image to server: {ex.Message}");
							}
						});
					}
				}
				// adjust detected speed
				if(label_string != "")
				{
					detection.Text = label_string;
				}
			}

			// move AI cars
			if(Global.SelectedTrack == 6)
			{
				car1.Progress += 3 * (float)delta;
				car2.Progress += 1.5f * (float)delta;
				car3.Progress += 1 * (float)delta;
				car4.Progress += 1 * (float)delta;
				car5.Progress += 1 * (float)delta;
				car6.Progress += 1 * (float)delta;
				car7.Progress += 1 * (float)delta;
				
			}
			if(ParkingActive && Input.IsActionPressed("park"))
			{
				StartParking(-1, "left");
				startParking = true;
			}

			if(ParkingActive)
			{
				string result = "";
				if(parkingSlotSearchTime > 0.5f && !searchForSlot && !startParking)
				{
					parkingSlotSearchTime = 0f;
					SvpLeftMidCamera.GlobalTransform = leftMidCamera.GlobalTransform;

					Image image = subViewportMidLeft.GetTexture().GetImage();

					Task.Run(async () =>
					{
						try
						{
							string yoloResponse = await SendYoloRequestDirectly(image, $"{ServerUrl}/predict_parkinglot_yolo");
							// GD.Print("IN HEREEEE");
							result = ParseParkingSlotYOLO(yoloResponse);
							// var detectionResponse = JsonConvert.DeserializeObject<DetectionResponse>(yoloResponse);
							// midpoint = detectionResponse.Midpoint;
							

							GD.Print(result);
							if(result.Contains( "Free-Space") && !result.Contains("Occupied"))
							{
								GD.Print("found empty slot");
								// vehicle.SetTargetSpeed(0);
								planStraightLine = true;
								autonomousMode = false;
								// label.Text = "Parking Mode";
							}
						}
						catch (Exception ex)
						{
							GD.PrintErr($"Error sending image to server: {ex.Message}");
						}
					});
					if(!planStraightLine)
					{
						detection.Text = "Occupied";
					}
				}
				if(planStraightLine)
				{
					if(!vehicle.leftBackParking.IsColliding() && !vehicle.leftFrontParking.IsColliding())
					{
						vehicle.SetTargetSpeed(0);
						label.Text = "Parking Mode";
						PlanStraightLine(6f);
						GD.Print("plan ");
						planStraightLine = false;
						searchForSlot = true;
						detection.Text = "Free-Space";
					}

					// Vector3 worldPosition = MapPointToWorld(new Vector2(midpoint.X, midpoint.Y));
					// MeshInstance3D meshInstance = new MeshInstance3D();
					// SphereMesh sphereMesh = new SphereMesh();
					// sphereMesh.Radius = 0.1f; // Adjust the size of the sphere
					// meshInstance.Mesh = sphereMesh;

					// // Position the mesh at the current Bezier point
					// meshInstance.GlobalTransform = new Transform3D(Basis.Identity, worldPosition);

					// // Add the mesh instance to the scene
					// AddChild(meshInstance);

				}
				if(searchForSlot)
				{
					GD.Print(bezierPoints.Count);
					if(currentPointIndex == bezierPoints.Count)
					{
						startParking = true;
						searchForSlot = false;
						vehicle.SetTargetSpeed(0);
						GD.Print("end for straight");
						GD.Print("start parking");
						StartParking(-1, "left");
					}
					else
					{
						Vector3 nextPoint = bezierPoints[currentPointIndex];

						float steeringAngle = CalculateSteeringAngle(vehicle.GlobalPosition, nextPoint, vehicle.Transform.Basis.Z,2f);
						// GD.Print("steeringAngle: ", steeringAngle);
						vehicle.SetSteeringAngle(steeringAngle);
						vehicle.SetTargetSpeed(3);
						// GD.Print("vehicle.GlobalPosition.DistanceTo(nextPoint): ", vehicle.GlobalPosition.DistanceTo(nextPoint));
						if (vehicle.GlobalPosition.DistanceTo(nextPoint) <1)
						{
							currentPointIndex++;
							GD.Print("incrementing");				
						}
					}
				}
				if(startParking)
				{
					if(currentPointIndex == bezierPoints.Count)
					{
						startParking = false;
						ParkingActive = false;
						vehicle.SetTargetSpeed(0);
						vehicle.SetSteeringAngle(0);
						label.Text = "Parked";
						GD.Print("parked");
					}
					else
					{
						Vector3 nextPoint = bezierPoints[currentPointIndex];

						float steeringAngle = CalculateSteeringAngle(vehicle.GlobalPosition, nextPoint, vehicle.Transform.Basis.Z,2f);
						// GD.Print("steeringAngle: ", steeringAngle);
						vehicle.SetSteeringAngle(steeringAngle);
						vehicle.SetTargetSpeed(-3);
						float val = 2.5f;
						if(currentPointIndex > 30)
						{
							val = 1;
						}
						if (vehicle.GlobalTransform.Origin.DistanceTo(nextPoint) <val)
						{
							currentPointIndex++;				
						}
					}
				}
			}
		}
	}
	private async Task<string> SendYoloRequestDirectly(Image image, string post)
	{
		try
		{
			using (var client = new System.Net.Http.HttpClient())
			{
				// Convert the image to PNG byte array
				byte[] pngData = image.SavePngToBuffer();

				//request content
				var content = new MultipartFormDataContent();
				var imageContent = new ByteArrayContent(pngData);
				imageContent.Headers.ContentType = new System.Net.Http.Headers.MediaTypeHeaderValue("image/png");
				content.Add(imageContent, "image", "frame.png");

				// Send request to Flask server
				HttpResponseMessage response = await client.PostAsync(post, content);

				if (response.IsSuccessStatusCode)
				{
					string jsonResponse = await response.Content.ReadAsStringAsync();
					return jsonResponse;
				}
				else
				{
					GD.PrintErr($"YOLO Server Error: {response.StatusCode}");
				}
			}
		}
		catch (Exception ex)
		{
			GD.PrintErr($"Error sending YOLO request: {ex.Message}");
		}
		return null;
	}

	

	private class YoloResponse
	{
		public bool stop_sign_detected { get; set; }
		public System.Collections.Generic.List<Detection> detections { get; set; }
	}
	private class Detection
	{
		public string @class { get; set; }
		public float confidence { get; set; }
		public float[] bbox { get; set; }
	}

	private async Task<PredictionResponse> SendImageToServer(Image image)
	{
		try
		{
			using (var client = new System.Net.Http.HttpClient())
			{
				
				// Convert Image to a byte array
				byte[] pngData = image.SavePngToBuffer();
				
				var content = new MultipartFormDataContent();
				var imageContent = new ByteArrayContent(pngData);
				imageContent.Headers.ContentType = new System.Net.Http.Headers.MediaTypeHeaderValue("image/png");
				content.Add(imageContent, "image", "frame.png"); 

				// Send the image to the Flask server
				HttpResponseMessage response = await client.PostAsync($"{ServerUrl}/predict_steering", content);

				// Handle the server response
				if (response.IsSuccessStatusCode)
				{
					string jsonResponse = await response.Content.ReadAsStringAsync();
					return JsonConvert.DeserializeObject<PredictionResponse>(jsonResponse);
				}
				else
				{
					GD.PrintErr($"Server Error: {response.StatusCode}");
				}
			}
		}
		catch (Exception ex)
		{
			GD.PrintErr($"Error Sending Image: {ex.Message}");
		}
		return null;
	}

	private void ApplyPrediction(float steering)
	{
		vehicle.SetSteeringAngle(steering);
	}

// Class to deserialize server prediction response
	public class PredictionResponse
	{
		public float Steering { get; set; }
		public float Throttle { get; set; }
	}

	private void CaptureAndLogData()
	{
		// Capture the image from the SubViewport
		Image frontImage = subViewport.GetTexture().GetImage();
		// Image rightImage = subViewportRight.GetTexture().GetImage();
		// Image leftImage = subViewportLeft.GetTexture().GetImage();

		string frontImageName = $"center_image_{photoNumber:D6}.png";
		string frontImagePath = $"{savePath}/{frontImageName}";
		frontImage.SavePng(frontImagePath);

		// string rightImageName = $"right_image_{photoNumber:D6}.png";
		// string rightImagePath = $"{savePath}/{rightImageName}";
		// rightImage.SavePng(rightImagePath);

		// string leftImageName = $"left_image_{photoNumber:D6}.png";
		// string leftImagePath = $"{savePath}/{leftImageName}";
		// leftImage.SavePng(leftImagePath);

		// Log annotations
		float steeringAngle = vehicle.GetSteeringAngle();
		float speed = vehicle.GetSpeed();
		float throttle = (float)vehicle.EngineForce / vehicle.MaxEngineForce;

		// string logEntry = $"{frontImageName},{leftImageName},{rightImageName},{steeringAngle:F4},{throttle},{0},{speed:F4}\n";
		string logEntry = $"{frontImageName},{""},{""},{steeringAngle:F4},{throttle},{0},{speed:F4}\n";
		File.AppendAllText(logFilePath, logEntry);

		GD.Print($"Captured: {frontImageName}, Steering Angle: {steeringAngle}, Throttle: {throttle} ,Speed: {speed}");

		photoNumber++;
	}

	private void CaptureParkingSpaceData()
	{
		Image sideImage = subViewportMidLeft.GetTexture().GetImage();
		string photoName = $"{parkingDatsasetPath}/{photoNumber}.png";
		sideImage.SavePng(photoName);
		photoNumber++;

		GD.Print(photoName);
	}
	private void CapturetTrafficSignData()
	{
		Image sideImage = subViewport.GetTexture().GetImage();
		string photoName = $"{parkingDatsasetPath}/{photoNumber}.png";
		sideImage.SavePng(photoName);
		photoNumber++;

		GD.Print(photoName);
	}
/////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////   The following 3 methods handle the Yolo response //////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////
	private string ParseYoloResponse(string response)
	{
		try
		{
			var detectionResponse = JsonConvert.DeserializeObject<DetectionResponse>(response);
			foreach (var detection in detectionResponse.Detections)
			{
				// GD.Print(detection.Class, detection.Confidence);
				if (detection.Class == "Stop")
				{
					float[] bbox = detection.Bbox.ToArray(); // [x1, y1, x2, y2]
					float bboxHeight = bbox[3] - bbox[1]; // y2 - y1
					GD.Print("bboxHeight: ", bboxHeight);

					// Get camera properties
					float fov = frontCamera.Fov;
					int screenHeight = (int)ProjectSettings.GetSetting("display/window/size/viewport_height");
					GD.Print("screenHeight: ", screenHeight);

					// Calculate focal length and distance
					float focalLength = CalculateFocalLength(fov, screenHeight);
					float distance = CalculateDistanceToStopSign(bboxHeight, focalLength);

					GD.Print($"Stop sign detected! Distance: {distance:F2} meters");

					if (distance <= 50.0f) // Example threshold for stopping
					{

						// vehicle.IsStopping = true;
						alreadyDetectStopSign = true;
						stopTime = 0.7f;
						giveTime = 2f;
						GD.Print("Vehicle stopping at stop sign...");

					}
					else
					{
						vehicle.IsStopping = false;
					}
					return ""; // Stop processing once stop sign is found
				}
				else if((detection.Class == "Speed Limit 50" || detection.Class == "Speed-Limit-50" )&& detection.Confidence >= 0.7)
				{
					GD.Print("Speed Limit 50 detected,", detection.Confidence);
					vehicle.SetTargetSpeed(50);
					if(detections.ContainsKey("Speed Limit 50"))
					{
						detections["Speed Limit 50"] = Mathf.Max(detections["Speed Limit 50"], detection.Confidence);
					}
					else
					{
						detections.Add("Speed Limit 50", detection.Confidence);
					}
					// this.detection.Text = "Speed Limit 60, " + detection.Confidence.ToString();
					return "Speed Limit 50, " + detection.Confidence.ToString();
				}
				else if((detection.Class == "Speed Limit 30"|| detection.Class == "Speed-Limit-30" ) && detection.Confidence >= 0.7)
				{
					vehicle.SetTargetSpeed(30);
					GD.Print("Speed Limit 30 detected, ", detection.Confidence);
					if(detections.ContainsKey("Speed Limit 30"))
					{
						detections["Speed Limit 30"] = Mathf.Max(detections["Speed Limit 30"], detection.Confidence);
					}
					else
					{
						detections.Add("Speed Limit 30", detection.Confidence);
					}
					// this.detection.Text = "Speed Limit 40, " + detection.Confidence.ToString();
					return "Speed Limit 30, " + detection.Confidence.ToString();
				}
				else if((detection.Class == "Speed Limit 20" || detection.Class == "Speed-Limit-20") && detection.Confidence >= 0.7)
				{
					vehicle.SetTargetSpeed(20);
					GD.Print("Speed Limit 20 detected, ", detection.Confidence);
					if(detections.ContainsKey("Speed Limit 20"))
					{
						detections["Speed Limit 20"] = Mathf.Max(detections["Speed Limit 20"], detection.Confidence);
					}
					else
					{
						detections.Add("Speed Limit 20", detection.Confidence);
					}
					// this.detection.Text = "Speed Limit 20, " + detection.Confidence.ToString();
					return "Speed Limit 20, " + detection.Confidence.ToString();
				}
				else if((detection.Class == "Speed Limit 90" || detection.Class == "Speed-Limit-90") && detection.Confidence >= 0.7)
				{
					vehicle.SetTargetSpeed(90);
					GD.Print("Speed Limit 90 detected,", detection.Confidence);
					if(detections.ContainsKey("Speed Limit 90") )
					{
						detections["Speed Limit 90"] = Mathf.Max(detections["Speed Limit 90"], detection.Confidence);
					}
					else
					{
						detections.Add("Speed Limit 90", detection.Confidence);
					}
					// this.detection.Text = "Speed Limit 120, " + detection.Confidence.ToString();
					return "Speed Limit 90, " + detection.Confidence.ToString();
				}


			}

			// vehicle.IsStopping = false; // No stop sign detected
		}
		catch (Exception ex)
		{
			GD.PrintErr($"Error parsing YOLO response: {ex.Message}");
		}
		return "";
	}
		private string ParseYoloResponse2(string response)
	{
		try
		{
			var detectionResponse = JsonConvert.DeserializeObject<DetectionResponse>(response);
			foreach (var detection in detectionResponse.Detections)
			{
				// GD.Print(detection.Class, detection.Confidence);
				if (detection.Class == "Stop")
				{
					float[] bbox = detection.Bbox.ToArray(); // [x1, y1, x2, y2]
					float bboxHeight = bbox[3] - bbox[1]; // y2 - y1
					GD.Print("bboxHeight: ", bboxHeight);

					// Get camera properties
					float fov = frontCamera.Fov;
					int screenHeight = (int)ProjectSettings.GetSetting("display/window/size/viewport_height");
					GD.Print("screenHeight: ", screenHeight);

					// Calculate focal length and distance
					float focalLength = CalculateFocalLength(fov, screenHeight);
					float distance = CalculateDistanceToStopSign(bboxHeight, focalLength);

					GD.Print($"Stop sign detected! Distance: {distance:F2} meters");

					if (distance <= 50.0f) // Example threshold for stopping
					{

						// vehicle.IsStopping = true;
						alreadyDetectStopSign = true;
						stopTime = 0.7f;
						giveTime = 2f;
						GD.Print("Vehicle stopping at stop sign...");

					}
					else
					{
						vehicle.IsStopping = false;
					}
					return ""; // Stop processing once stop sign is found
				}
				else if((detection.Class == "Speed Limit 50" || detection.Class == "Speed-Limit-50" ))
				{
					GD.Print("Speed Limit 50 detected,", detection.Confidence);
					vehicle.SetTargetSpeed(50);
					if(detections.ContainsKey("Speed Limit 50"))
					{
						detections["Speed Limit 50"] = Mathf.Max(detections["Speed Limit 50"], detection.Confidence);
					}
					else
					{
						detections.Add("Speed Limit 50", detection.Confidence);
					}
					// this.detection.Text = "Speed Limit 60, " + detection.Confidence.ToString();
					return "Speed Limit 50, " + detection.Confidence.ToString();
				}
				else if((detection.Class == "Speed Limit 30"|| detection.Class == "Speed-Limit-30" ))
				{
					vehicle.SetTargetSpeed(30);
					GD.Print("Speed Limit 30 detected, ", detection.Confidence);
					if(detections.ContainsKey("Speed Limit 30"))
					{
						detections["Speed Limit 30"] = Mathf.Max(detections["Speed Limit 30"], detection.Confidence);
					}
					else
					{
						detections.Add("Speed Limit 30", detection.Confidence);
					}
					// this.detection.Text = "Speed Limit 40, " + detection.Confidence.ToString();
					return "Speed Limit 30, " + detection.Confidence.ToString();
				}
				else if((detection.Class == "Speed Limit 20" || detection.Class == "Speed-Limit-20") )
				{
					vehicle.SetTargetSpeed(20);
					GD.Print("Speed Limit 20 detected, ", detection.Confidence);
					if(detections.ContainsKey("Speed Limit 20"))
					{
						detections["Speed Limit 20"] = Mathf.Max(detections["Speed Limit 20"], detection.Confidence);
					}
					else
					{
						detections.Add("Speed Limit 20", detection.Confidence);
					}
					// this.detection.Text = "Speed Limit 20, " + detection.Confidence.ToString();
					return "Speed Limit 20, " + detection.Confidence.ToString();
				}
				else if((detection.Class == "Speed Limit 90" || detection.Class == "Speed-Limit-90"))
				{
					vehicle.SetTargetSpeed(90);
					GD.Print("Speed Limit 90 detected,", detection.Confidence);
					if(detections.ContainsKey("Speed Limit 90") )
					{
						detections["Speed Limit 90"] = Mathf.Max(detections["Speed Limit 90"], detection.Confidence);
					}
					else
					{
						detections.Add("Speed Limit 90", detection.Confidence);
					}
					// this.detection.Text = "Speed Limit 120, " + detection.Confidence.ToString();
					return "Speed Limit 90, " + detection.Confidence.ToString();
				}


			}

			// vehicle.IsStopping = false; // No stop sign detected
		}
		catch (Exception ex)
		{
			GD.PrintErr($"Error parsing YOLO response: {ex.Message}");
		}
		return "";
	}
	private string ParseParkingSlotYOLO(string response)
	{
		try
		{
			var detectionResponse = JsonConvert.DeserializeObject<DetectionResponse>(response);
			foreach (var detection in detectionResponse.Detections)
			{
				if(detection.Class == "Free-Space")
				{
					return "Free-Space";
				}
				else if(detection.Class == "Occupied")
				{
					return "Occupied";
				}
			}
		}
		catch (Exception ex)
		{
			GD.PrintErr($"Error parsing YOLO response: {ex.Message}");
		}
		return "";
	}

	private float CalculateFocalLength(float fov, int screenHeight)
	{
		// Calculate focal length based on camera's FOV and screen height
		return screenHeight / (2 * Mathf.Tan(Mathf.DegToRad(fov / 2)));
	}

	private float CalculateDistanceToStopSign(float bboxHeightPixels, float focalLength, float realHeight = 1.5f)
	{
		// Avoid division by zero
		if (bboxHeightPixels <= 0)
			return float.PositiveInfinity;

		// Pin-hole camera model formula
		return (focalLength * realHeight) / bboxHeightPixels;
	}

	public void _on_button_pressed()
	{
		LoadScene("res://Scenes/second_menu.tscn");
	}
	private void LoadScene(string scenePath)
	{
		PackedScene scene = GD.Load<PackedScene>(scenePath);
		Node instance = scene.Instantiate();
		GetTree().Root.AddChild(instance);

		QueueFree(); 
	}
}
