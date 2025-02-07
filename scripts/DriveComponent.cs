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
	public Vector3 lastGroundNormal;
	public bool OnGround{get{
		var groundQuery = PhysicsRayQueryParameters3D.Create(car.GlobalPosition,car.GlobalPosition-car.GlobalBasis.Y.Normalized()*10f);
    	var groundResult = DriveComponent.globalState.IntersectRay(groundQuery);
		return groundResult.Count>0;
	}}
	public float engine_ratio;
	public float slowdown_factor;
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
		car = (RigidBody3D)GetParent();
		IntendedWheelPositions = new();
		var children = car.GetChildren();
		foreach(Wheel wheel in children.Where(obj => obj is Wheel))
		{
			IntendedWheelPositions.Add(wheel,wheel.Position);
		}
		car.AngularDamp = .5f;
	}
    public override void _EnterTree()
    {
        base._EnterTree();
		_Ready();
    }
    public void ApplySuspensionForce(Wheel wheel){
		var f = wheel.SuspensionForce(IntendedWheelPositions[wheel],this);
		car.ApplyForce(f,wheel.Position);
	}
	public void ApplyForwardForce(Wheel w,double delta){
		//rotation speed is a compound of old rotation speed and gas capped by engine ratio @ 9000k each engine will have a different gas injection pattern(not as realistic as it could be kill me)
		w.rotationSpeed = Mathf.Clamp((gas*2*Mathf.Pi*acc+w.rotationSpeed)*slowdown_factor,-9000/engine_ratio,9000/engine_ratio);
		//moving the wheel is the first nessicary step to get the speed to apply, missing framerate should theoretically be accounted for since apply force uses delta t and so do all needed functions(looking @ rotating wheels!!!)
		w.Rotate(w.GlobalBasis.Column2.Normalized(),(float)(w.rotationSpeed*delta));
		var force = w.ForwardForce(this);
		//car.ApplyImpulse(force,w.LastGripPoint);
		//change rotation speed based on how car body responds to forces
		w.rotationSpeed+=(car.LinearVelocity-force).Dot(car.GlobalBasis.Column2);
	}
    public override void _PhysicsProcess(double delta)
    {
        base._PhysicsProcess(delta);
		car.ConstantForce = Vector3.Zero;
		foreach(Wheel wheel in IntendedWheelPositions.Keys)
		{
			
			globalState = PhysicsServer3D.SpaceGetDirectState(wheel.GetWorld3D().Space);
			ApplySuspensionForce(wheel);
			//ApplyForwardForce(wheel,delta);
			//car.GlobalPosition = Vector3.Up*30;
			//car.Rotate(Vector3.Up,.02f); 
			
			if(gas==0&&OnGround)
			{
				var intended = car.LinearVelocity.Dot(car.GlobalBasis.Y)*car.GlobalBasis.Y;
				var offset = intended-car.LinearVelocity;
				if(offset.Length()<.03){
					car.LinearVelocity=intended;
				}
				else{
					offset = offset.Normalized()*.03f;
					car.LinearVelocity+=offset;
				}
			}
		}
		
    }
}
