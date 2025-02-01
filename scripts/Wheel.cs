using Godot;
using System.Linq;
using System;
using System.Net.WebSockets;
//TODO implement typed GetChildren!!!!!!!!!
public partial class Wheel : Node3D
{
	public float rotationSpeed;
	[Export]
	public float suspension_coefficient;
	[Export]
	public float mass;
	public Vector3 LastGripPoint{ get; set;}
	public Node3D LastUsed{ get; set;}
    public Node3D MostDown(DriveComponent driveComponent)
	{
		var down = Vector3.Down*driveComponent.car.GlobalBasis;
        return (Node3D)GetChildren().OrderBy(e =>
            (-down).Dot(GlobalPosition-((Node3D)e).GlobalPosition)
			).First();
	}
	public Vector3 SuspensionForce(Vector3 intended,DriveComponent driveComponent){
		var down = Vector3.Down*driveComponent.car.Basis;
		down*=(((Node3D)GetChildren().First()).GlobalPosition-GlobalPosition).Length();
		down+=intended;
		var groundQuery = PhysicsRayQueryParameters3D.Create(GlobalPosition,down);
    	var groundResult = DriveComponent.globalState.IntersectRay(groundQuery);
		if(groundResult.Count>0)
		return ((intended-GlobalPosition).Normalized()*suspension_coefficient)/(intended-GlobalPosition).Length();
		return Vector3.Zero;
	}
	public Vector3 ForwardForce(DriveComponent driveComponent){
		var rotationalQuery = PhysicsRayQueryParameters3D.Create(GlobalPosition,GlobalPosition+2*(LastUsed.GlobalPosition-GlobalPosition));
    	var rotationalResult = DriveComponent.globalState.IntersectRay(rotationalQuery);
		var down = Vector3.Down*driveComponent.car.Basis;
		down*=(((Node3D)GetChildren().First()).GlobalPosition-GlobalPosition).Length();
		var groundQuery = PhysicsRayQueryParameters3D.Create(GlobalPosition,GlobalPosition+down);
    	var groundResult = DriveComponent.globalState.IntersectRay(groundQuery);
		if(rotationalResult.Count>0)
		{
			var position = (Vector3)rotationalResult["position"];
			if(groundResult.Count>0)
			{
				LastUsed = MostDown(driveComponent);
				rotationalQuery = PhysicsRayQueryParameters3D.Create(GlobalPosition,GlobalPosition+2*(LastUsed.GlobalPosition-GlobalPosition));
    			rotationalResult = DriveComponent.globalState.IntersectRay(rotationalQuery);
				LastGripPoint = (Vector3)rotationalResult["position"];
				
				return (LastGripPoint-position);
			}
		}
		LastUsed = MostDown(driveComponent);
		rotationalQuery = PhysicsRayQueryParameters3D.Create(GlobalPosition,GlobalPosition+2*(LastUsed.GlobalPosition-GlobalPosition));
    	rotationalResult = DriveComponent.globalState.IntersectRay(rotationalQuery);
		LastGripPoint = (Vector3)rotationalResult["position"];
		return Vector3.Zero;
	}
}
