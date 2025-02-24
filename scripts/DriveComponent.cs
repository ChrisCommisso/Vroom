using Godot;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.InteropServices;
/// <summary>
/// drive docs:
/// expl: should be able to cache wheel positions and keep them upon loading. car will always attempt to push wheels to og position. works by raycasting down from above og wheel pos looking only for the ground, when it finds it it will use the wheels orientation+the last ray used+the closest ray to true down, this gets details on grip/height/suspension/and current speed. each wheel will have an output force on the car, originating from a suspension connection point specified in the wheel
/// TODO: implement rigidbody, spinning wheels, moving, suspension, rotational inertia
/// only ONE RIGIDBODY
/// </summary>
public partial class DriveComponent : Node
{	
	public Vector3 lookTo;
	public Vector3 lastGroundNormal;
	public bool OnGround{get{
		var groundQuery = PhysicsRayQueryParameters3D.Create(car.GlobalPosition,car.GlobalPosition-car.GlobalBasis.Y.Normalized()*10f);
    	var groundResult = DriveComponent.globalState.IntersectRay(groundQuery);
		return groundResult.Count>0;
	}}
	bool engaged => gas != 0;
	public float WheelRpm;
	public float engine_ratio;
	public float slowdown_factor=.97f;
	public float acc;
	public float mass;
	public float brake;
	public float gas;
	public RigidBody3D car;
	public static PhysicsDirectSpaceState3D globalState;
	// Called when the node enters the scene tree for the first time.
	public Dictionary<Wheel,Vector3> IntendedWheelPositions;
	public override void _Ready()
	{
		Engine.TimeScale = 1f;
		car = (RigidBody3D)GetParent();
		IntendedWheelPositions = new();
		var children = car.GetChildren();
		foreach(Wheel wheel in children.Where(obj => obj is Wheel))
		{
			IntendedWheelPositions.Add(wheel,wheel.Position);
		}

		car.AngularDamp = .1f;
		car.LinearDamp = .1f;
		//car.Inertia = Vector3.One*.03f;
	}
    public override void _EnterTree()
    {
        base._EnterTree();
		_Ready();
    }
    public void ApplySuspensionForce(Wheel wheel,double delta){
		var f = wheel.SuspensionForce(IntendedWheelPositions[wheel],this);
		car.ApplyForce(((float)delta)*f,wheel.GlobalPosition-car.GlobalPosition);
	}
	
	public void ApplyForwardForce(Wheel w,double delta){
		//rotation speed is a compound of old rotation speed and gas capped by engine ratio @ 9000k each engine will have a different gas injection pattern(not as realistic as it could be kill me)
		WheelRpm = Mathf.Clamp((WheelRpm+(gas))*slowdown_factor,-9000/engine_ratio,9000/engine_ratio);
		//moving the wheel is the first nessicary step to get the speed to apply, missing framerate should theoretically be accounted for since apply force uses delta t and so do all needed functions(looking @ rotating wheels!!!)
		w.Rotate(w.Basis.Z.Normalized(),(float)(WheelRpm*60*Mathf.Pi*delta));
		var force = w.ForwardForce(this,WheelRpm,delta);
		lookTo+=force;
		if(engaged){
			car.ApplyForce(force,w.GlobalPosition-car.GlobalPosition);           
			//car.ApplyTorque(force);
			}
			//car.ApplyTorque(force.Length()*car.GlobalBasis.X);
			//car.ApplyForce(force,w.LastDownPoint);
		

	}
	public void integrateForces(){}
    public override void _PhysicsProcess(double delta)
    {
        base._PhysicsProcess(delta);
		if(Input.IsKeyPressed(Key.Space)){
			gas = 1;
		}
		else{
			gas = 0;
		}
		float turn = 0;
		if(Input.IsKeyPressed(Key.A))
		{
			turn+=1;
		}
		if(Input.IsKeyPressed(Key.D))
		{
			turn-=1;
		}
		IntendedWheelPositions.Keys.ToArray()[1].RotationDegrees = new Vector3(IntendedWheelPositions.Keys.ToArray()[1].RotationDegrees.X,90+turn*30,IntendedWheelPositions.Keys.ToArray()[1].RotationDegrees.Z);
		IntendedWheelPositions.Keys.ToArray()[3].RotationDegrees = new Vector3(IntendedWheelPositions.Keys.ToArray()[1].RotationDegrees.X,90+turn*30,IntendedWheelPositions.Keys.ToArray()[1].RotationDegrees.Z);
		//IntendedWheelPositions.Keys.ToArray()[0].RotationDegrees = new Vector3(IntendedWheelPositions.Keys.ToArray()[1].RotationDegrees.X,90-turn*40,IntendedWheelPositions.Keys.ToArray()[1].RotationDegrees.Z);
		//IntendedWheelPositions.Keys.ToArray()[2].RotationDegrees = new Vector3(IntendedWheelPositions.Keys.ToArray()[1].RotationDegrees.X,90-turn*40 ,IntendedWheelPositions.Keys.ToArray()[1].RotationDegrees.Z);

		car.ConstantForce = Vector3.Zero;
		lookTo = Vector3.Zero;
		foreach(Wheel wheel in IntendedWheelPositions.Keys)
		{
			globalState = PhysicsServer3D.SpaceGetDirectState(wheel.GetWorld3D().Space);
			ApplySuspensionForce(wheel,delta);
			ApplyForwardForce(wheel,delta);
		}
		if(gas == 0 && OnGround)
			{
				
				var intended = car.LinearVelocity.Dot(car.GlobalBasis.Y)*car.GlobalBasis.Y;
				var offset = (intended-car.LinearVelocity).Clamp(-Vector3.One*.05f,Vector3.One*.05f);
				//                         car.LinearVelocity+=offset;
				
			}
		if(OnGround){
			car.AngularVelocity += (float)delta*car.Basis.Y.Normalized()*(car.Basis.X.Normalized().Dot(lookTo.Normalized())*car.LinearVelocity.Length()*.01f)*(2*MathF.PI);                       
		}	
		else{
			//                                                  car.AngularVelocity *= (1f-(.03f*(float)delta*60));

		}
			
		}
		//if(brake!=0&&OnGround)
		//	{
		//		var intended = car.LinearVelocity.Dot(car.GlobalBasis.Y)*car.GlobalBasis.Y;
		//		
		//		var offset = intended-car.LinearVelocity;
		//		if(offset.Length()<.09){
		//			car.LinearVelocity=intended;
		//		}
		//		else{
		//			offset = offset.Normalized()*(1-brake);
		//			car.LinearVelocity+=offset;
		//		}
		//	}
			
		
    }

