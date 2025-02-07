using Godot;
using Godot.Collections;
using System.Linq;
//TODO implement typed GetChildren!!!!!!!!!
public partial class Wheel : MeshInstance3D
{
	public MeshInstance3D f;
	public MeshInstance3D t;
	public PhysicsDirectSpaceState3D space;
	//curremnt rotation speed
	
	public float floatiness=.4f;
	public float rotationSpeed;
	public float fall_coefficient=2f;
	public float suspension_coefficient=1f;
	[Export]
	public float mass;
	public Vector3 LastDownPoint{ get; set;}
	public Vector3 LastGripPoint{ get; set;}
	public Node3D LastUsed{ get; set;}
    public override void _Ready()
    {
        base._Ready();
    }
    public override void _PhysicsProcess(double delta)
    {
        base._PhysicsProcess(delta);
    }
    public Node3D MostDown(DriveComponent driveComponent)
	{
		var down = Vector3.Down*driveComponent.car.GlobalBasis;
        return (Node3D)GetChildren().OrderBy(e =>
            (-down).Dot(GlobalPosition-((Node3D)e).GlobalPosition)
			).First();
	}
	public Vector3 SuspensionForce(Vector3 intend,DriveComponent driveComponent){
		var parent = driveComponent.car;
		Vector3 intended = parent.GlobalBasis.Z * intend.Z + parent.GlobalBasis.X * intend.X + parent.GlobalBasis.Y * intend.Y + parent.GlobalPosition;
		var wtc = driveComponent.car.GlobalPosition-intended;

		var up = parent.GlobalBasis.Y.Normalized();
		var force = Vector3.Zero;
		var wheelLen = (((Node3D)this.GetChild(0)).GlobalPosition-this.GlobalPosition).Length();
		var offset = wheelLen*-up;

        var groundQuery = PhysicsRayQueryParameters3D.Create(intended + up*wheelLen, intended+offset,exclude: new Array<Rid>(new Rid[1] {driveComponent.car.GetRid()}) );
    	var groundResult = DriveComponent.globalState.IntersectRay(groundQuery);
		if(f == null)
		 f = Draw3D.Point(intended+up*20,1,Colors.Red);
		else
		 f.GlobalPosition = intended+up*20;
		if(t == null)
		 t = Draw3D.Point(intended + offset,1,Colors.Beige);
		else
		 t.GlobalPosition = intended + offset;
		if(groundResult.Count>0){
			force = suspension_coefficient*((((Vector3)groundResult["position"]-(intended+offset)).Length()*wtc)*floatiness+(1-floatiness)*((Vector3)groundResult["position"]-(intended+offset)));
		}
		else{
			driveComponent.car.AngularVelocity = Vector3.Zero;
		}
			//force = ((Vector3)groundResult["position"]-(intended+offset)).Length()*suspension_coefficient*((((Vector3)groundResult["position"]-(intended+offset)).Length()*wtc)*floatiness+(1-floatiness)*((Vector3)groundResult["position"]-(intended+offset)));
		
		if(groundResult.Count>0)
		{
			GlobalPosition=(Vector3)groundResult["position"]-offset;
		}
		else
		{
			GlobalPosition=driveComponent.IntendedWheelPositions[this];
		}
		//apply the forces to the car
		GD.Print(Name,force);
		return force;
	}

	public Vector3 ForwardForce(DriveComponent driveComponent){
		var rotationalQuery = PhysicsRayQueryParameters3D.Create(GlobalPosition,GlobalPosition+2*(LastUsed.GlobalPosition-GlobalPosition));
    	var rotationalResult = DriveComponent.globalState.IntersectRay(rotationalQuery);
		var down = Vector3.Down*driveComponent.car.Basis;
		down*=4.4f;
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
				
				return (LastGripPoint-position)-driveComponent.car.LinearVelocity;
			}
		}
		LastUsed = MostDown(driveComponent);
		rotationalQuery = PhysicsRayQueryParameters3D.Create(GlobalPosition,GlobalPosition+2*(LastUsed.GlobalPosition-GlobalPosition));
    	rotationalResult = DriveComponent.globalState.IntersectRay(rotationalQuery);
		LastGripPoint = (Vector3)rotationalResult["position"];
		return Vector3.Zero;
	}
}
