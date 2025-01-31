using Godot;
using System;


public partial class Vehicle : VehicleBody3D
{
	[Export] public float MaxEngineForce =30;
	[Export] private float MaxSteeringAngle = 1f; // In radians
	[Export] public float BrakeForce = 70f;

	private float SteeringAngle = 0f;
	public float BrakingForce = 0f;
	private int maxSpeed = 30;

	private float SteeringSpeed = 0.45f; // Rate of steering angle change
	private float CurrentSteeringAngle = 0f;

	public bool IsStopping = false;

	[Export]
	public Camera3D camera1;
	[Export] // Main Camera
	private Camera3D camera2;
	
	[Export]
	private Camera3D rightCamera;
	[Export] // Main Camera
	private Camera3D leftCamera;

	[Export]
	private Camera3D leftMidCamera;
	[Export]
	public RayCast3D laneChangeRay;

	[Export]
	public RayCast3D forwardRay;

	[Export]
	public RayCast3D leftRay;

	[Export]
	public RayCast3D rightRay;

	[Export]
	public RayCast3D leftSide;

	[Export]
	public RayCast3D leftBackParking;

	[Export]
	public RayCast3D rightBackParking;
	[Export]
	public RayCast3D leftFrontParking;

	[Export]
	public RayCast3D rightFrontParking;

	[Export]
	public RayCast3D rightSide;
	 public float RotationSpeed = 10f;
	public float MaxRotation = -30.0f;

	int photoNumber = 0;
	private float targetSpeed = 30; 

	private VehicleWheel3D[] VehicleWheels;

	public static Vehicle Instance;

	private float _timeElapsed = 0f;

	public override void _Ready()
	{

		Instance = this;
		camera1.Current = true;
		camera2.Current = false;
		leftRay.Visible = false;
		leftSide.Visible = false;
		rightSide.Visible = false;
		rightRay.Visible = false;
		laneChangeRay.Visible = false;
		forwardRay.Visible = false;
		

		// Initialize the wheel nodes
		VehicleWheels = new VehicleWheel3D[4];
		try
		{
			// Directly access the wheels relative to VehicleBody3D
			VehicleWheels[0] = GetNode<VehicleWheel3D>("FrontLeftWheel");
			VehicleWheels[1] = GetNode<VehicleWheel3D>("FrontRightWheel");
			VehicleWheels[2] = GetNode<VehicleWheel3D>("RearLeftWheel");
			VehicleWheels[3] = GetNode<VehicleWheel3D>("RearRightWheel");

			GD.Print("Successfully initialized all wheels.");
		}
		catch (Exception e)
		{
			GD.PrintErr($"Failed to initialize wheels: {e.Message}");
		}

	}
	public override void _PhysicsProcess(double delta)
	{

		_timeElapsed += (float)delta *RotationSpeed;
		float angle = MaxRotation * Mathf.Abs(Mathf.Sin(_timeElapsed));
		Vector3 rotation = forwardRay.RotationDegrees;
		rotation.Y = angle;
		forwardRay.RotationDegrees = rotation;

		angle = 7 * Mathf.Sin(_timeElapsed* 5);
		rotation = laneChangeRay.RotationDegrees;
		rotation.Y = angle;
		laneChangeRay.RotationDegrees = rotation;


		angle = 40 * Mathf.Sin(_timeElapsed* 5);
		rotation = leftSide.RotationDegrees;
		rotation.Y = angle;
		leftSide.RotationDegrees = rotation;

		angle = 40 * Mathf.Sin(_timeElapsed* 5);
		rotation = rightSide.RotationDegrees;
		rotation.Y = angle;
		rightSide.RotationDegrees = rotation;

		if(Global.GameMode == "Training")
		{
			SetTargetSpeed(10);
			if (Input.IsActionPressed("ui_up") || Input.IsActionPressed("forward"))
			{
				EngineForce = Mathf.Lerp(EngineForce, MaxEngineForce, 1.0f * (float)delta);
			}
			else if (Input.IsActionPressed("ui_down")|| Input.IsActionPressed("backward"))
			{
				EngineForce = Mathf.Lerp(EngineForce, -MaxEngineForce, 1.0f * (float)delta);
			}
			else
			{
				GD.Print("in here");
				EngineForce = Mathf.Lerp(EngineForce, 0, 1.0f * (float)delta);
				if(EngineForce < 0.01)
				{
					EngineForce = 0;
				}
			}
			
			// Handle steering
			if (Input.IsActionPressed("ui_right") || Input.IsActionPressed("right"))
			{
				CurrentSteeringAngle = Mathf.Lerp(CurrentSteeringAngle, -MaxSteeringAngle, SteeringSpeed * (float)delta);
			}
			else if (Input.IsActionPressed("ui_left") || Input.IsActionPressed("left"))
			{
				CurrentSteeringAngle = Mathf.Lerp(CurrentSteeringAngle, MaxSteeringAngle, SteeringSpeed * (float)delta);
			}
			else
			{
				CurrentSteeringAngle = Mathf.Lerp(CurrentSteeringAngle, 0f, SteeringSpeed * 3*(float)delta);
				if (Mathf.Abs(CurrentSteeringAngle) < 0.08f)
				{
					CurrentSteeringAngle = 0f;
				}
			}

			// Handle braking
			if (Input.IsActionPressed("brake"))
			{
				GD.Print("in braking");
				BrakingForce = Mathf.Lerp(BrakingForce, BrakeForce, 1.0f * (float)delta);
				// EngineForce = Mathf.Lerp(EngineForce, 0, 2.0f * (float)delta);
				if(EngineForce < 0.01)
				{
					EngineForce = 0;
				}
			}
			else
			{
				BrakingForce = 0f;
			}
		}
		else
		{
			if(targetSpeed > 0)
			{
				float currentSpeed = this.LinearVelocity.Length() * 3.6f;
				if (currentSpeed < targetSpeed)
				{
					EngineForce = Mathf.Lerp(EngineForce, MaxEngineForce, 1.0f * (float)delta);
				}
				else if(currentSpeed > targetSpeed)
				{
					EngineForce = 0;
					BrakeForce = Mathf.Lerp(BrakeForce, BrakingForce, 1.5f * (float)delta);
				}
				else
				{
					EngineForce = 0;
					BrakeForce = 0;
				}
			}
			else if(targetSpeed < 0)
			{
				float currentSpeed = this.LinearVelocity.Length() * 3.6f;
				if (currentSpeed < -targetSpeed)
				{
					EngineForce = Mathf.Lerp(EngineForce, -MaxEngineForce, 1.0f * (float)delta);
				}
				else if(currentSpeed > -targetSpeed)
				{
					EngineForce = 0;
					BrakeForce = Mathf.Lerp(BrakeForce, BrakingForce, 1.5f * (float)delta);
				}
				else
				{
					EngineForce = 0;
					BrakeForce = 0;
				}
			}
			else
			{
				float currentSpeed = this.LinearVelocity.Length() * 3.6f;
				if(currentSpeed >=10)
				{
					if (currentSpeed < targetSpeed)
					{
						EngineForce = Mathf.Lerp(EngineForce, MaxEngineForce, 1.0f * (float)delta);
					}
					else if(currentSpeed > targetSpeed)
					{
						EngineForce = 0;
						BrakeForce = Mathf.Lerp(BrakeForce, BrakingForce, 10f * (float)delta);
					}
				}
				else
				{
					EngineForce = 0;
					BrakeForce = BrakingForce;
					this.LinearVelocity = Vector3.Zero;
				}
			}
		}
		// Apply engine force and braking to rear wheels
		ApplyWheelForces(EngineForce, BrakingForce);

		// Apply steering angle to front wheels
		ApplyWheelSteering(CurrentSteeringAngle);
		
	}
	private void ApplyWheelForces(float engineForce, float brakeForce)
	{
		VehicleWheels[2].EngineForce = engineForce;
		VehicleWheels[2].Brake = brakeForce;

		VehicleWheels[3].EngineForce = engineForce;
		VehicleWheels[3].Brake = brakeForce;

	}

	private void ApplyWheelSteering(float steeringAngle)
	{
		VehicleWheels[0].Steering = steeringAngle;

		VehicleWheels[1].Steering = steeringAngle;

	}
	public override void _Input(InputEvent inputEvent)
	{
		if (inputEvent.IsActionPressed("switch_camera"))
		{
			GD.Print("in camera changing");
			if (camera1.Current)
			{
				camera1.Current = false;
				camera2.Current = true;
			}
			else
			{
				camera2.Current = false;
				camera1.Current = true;
			}
			GD.Print($"Camera1 Current: {camera1.Current}, Camera2 Current: {camera2.Current}");

		}
	}
	public float GetSteeringAngle()
	{
		return CurrentSteeringAngle;
	}
	public void SetSteeringAngle(float steeringAngle)
	{
		CurrentSteeringAngle = steeringAngle;
	}
	public void SetSpeed(float speed)
	{
		EngineForce = speed;
	}
	public float GetSpeed()
	{
		return LinearVelocity.Length();
	}
	public void SetTargetSpeed(float targetSpeed)
	{
		this.targetSpeed = targetSpeed;
	}
	public float GetTargetSpeed()
	{
		return targetSpeed;
	}
	

}
