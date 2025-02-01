using Godot;
using System.Collections.Generic;
using System.Linq;
/// <summary>
/// drive docs:
/// expl: should be able to cache wheel positions and keep them upon loading. car will always attempt to push wheels to og position. works by raycasting down from above og wheel pos looking only for the ground, when it finds it it will use the wheels orientation+the last ray used+the closest ray to true down, this gets details on grip/height/suspension/and current speed. each wheel will have an output force on the car, originating from a suspension connection point specified in the wheel
/// TODO: implement rigidbody, spinning wheels, moving, suspension
/// only ONE RIGIDBODY
/// </summary>
public partial class DriveComponent : Node
{	
	public float mass {get; set;}
	public float brake {get; set;}
	public float gas {get; set;}
	public RigidBody3D car;
	public static PhysicsDirectSpaceState3D globalState;
	// Called when the node enters the scene tree for the first time.
	public Dictionary<Node,Vector3> IntendedWheelPositions;
	public override void _EnterTree()
	{
		car = (RigidBody3D)GetParent();
		IntendedWheelPositions = new();
		var children = GetChildren();
		foreach(Node3D wheel in GetChildren().Where(obj => obj.Name.ToString().Contains("Wheel")))
		{
			IntendedWheelPositions.Add(wheel,wheel.GlobalPosition);
		}
	}
	public void ApplySuspensionForce(Wheel w){
		var force = w.SuspensionForce(IntendedWheelPositions[w],this);
		var wheelLen = (((Node3D)w.GetChild(0)).GlobalPosition-w.GlobalPosition).Length();
		var offset = wheelLen*Vector3.Down*car.Basis;
		var RayOffset = offset+IntendedWheelPositions[w]-w.GlobalPosition;
		var groundQuery = PhysicsRayQueryParameters3D.Create(w.GlobalPosition,w.GlobalPosition+RayOffset);
    	var groundResult = DriveComponent.globalState.IntersectRay(groundQuery);
		if(groundResult.Count>0)
		{
			w.GlobalPosition=(Vector3)groundResult["position"]-offset;
		}
		else
		{
			w.GlobalPosition=IntendedWheelPositions[w];
		}
		car.ApplyForce(force,w.GlobalPosition);
	}
	public void ApplyForwardForce(Wheel w){
		
		car.ApplyForce(w.ForwardForce(this),w.LastGripPoint);
	}
    public override void _PhysicsProcess(double delta)
    {
        base._PhysicsProcess(delta);
		bool setspace = false;
		foreach(Wheel wheel in IntendedWheelPositions.Keys)
		{
			if(!setspace)
			globalState = PhysicsServer3D.SpaceGetDirectState(wheel.GetWorld3D().Space);
			setspace = true;
			
		}
    }
}
