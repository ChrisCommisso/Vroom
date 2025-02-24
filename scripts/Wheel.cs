using Godot;
using Godot.Collections;
using System;
using System.Linq;
//TODO implement typed GetChildren!!!!!!!!!
public partial class Wheel : MeshInstance3D
{
	public MeshInstance3D f;
	public MeshInstance3D t;
	public PhysicsDirectSpaceState3D space;
	//current rotation speed
	private float wheelRad = 0;
	public float floatiness=.8f;
	public float rotationSpeed;
	public float fall_coefficient=2f;
	public float suspension_coefficient=20000f;
	[Export]
	public float mass;
	public Vector3 LastDownPoint{ get; set;}
	public Vector3 LastGripPoint{ get; set;}
	public Node3D LastUsed{ get; set;}
    public override void _Ready()
    {
        base._Ready();
		wheelRad = (((Node3D)this.GetChild(0)).GlobalPosition-this.GlobalPosition).Length();
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
		Vector3 intended = parent.GlobalBasis.Z * intend.Z + parent.GlobalBasis.X * intend.X + 
		parent.GlobalBasis.Y * intend.Y + parent.GlobalPosition;
		//var wtc = driveComponent.car.GlobalPosition-intended;

		var up = parent.GlobalBasis.Y.Normalized();
		var force = Vector3.Zero;
		var offset = wheelRad*-up;

        var groundQuery = PhysicsRayQueryParameters3D.Create(intended + up*wheelRad, intended+offset,
		exclude: new Array<Rid>(new Rid[1] {driveComponent.car.GetRid()}) );
    	var groundResult = DriveComponent.globalState.IntersectRay(groundQuery);
		
		if(groundResult.Count>0){
			
			force = suspension_coefficient*((Vector3)groundResult["position"]-(intended+offset));//suspension_coefficient*((((Vector3)groundResult["position"]-(intended+offset)).Length()*wtc)*floatiness+(1-floatiness)*suspension_coefficient*new Vector3(1,1,1)*((Vector3)groundResult["normal"]).Normalized()*((Vector3)groundResult["position"]-(intended+offset)).Length()*((Vector3)groundResult["position"]-(intended+offset)).LengthSquared());
			GD.Print(Name+" "+((intended + up*wheelRad)-((Vector3)groundResult["position"])).Length());
			//force = suspension_coefficient*(GlobalPosition-intended)*(GlobalPosition-intended).LengthSquared();
			force *= new Vector3(1,1, 1);

			LastDownPoint = (Vector3)groundResult["position"];
			GlobalPosition = (Vector3)groundResult["position"]-offset;
		}
		else{
			
			//      force = suspension_coefficient*(-1*up);//*(intended-GlobalPosition).LengthSquared();
			//force *= new Vector3(1,1,1);
			LastDownPoint=intended;
			GlobalPosition=intended;
		}
			//force = ((Vector3)groundResult["position"]-(intended+offset)).Length()*suspension_coefficient*((((Vector3)groundResult["position"]-(intended+offset)).Length()*wtc)*floatiness+(1-floatiness)*((Vector3)groundResult["position"]-(intended+offset)));
		if(f == null)
		 		f = Draw3D.Point(intended+up*20,4,Colors.Red);
			else
		 		f.GlobalPosition = intended-offset;
			if(t == null)
		 		t = Draw3D.Point(intended + offset,5,Colors.Beige);
			else
		 		t.GlobalPosition = intended-offset + force/1000;
		
		
		//apply the forces to the car

		return force;
		
	}

	public Vector3 ForwardForce(DriveComponent driveComponent,float wheelRpm,double delta){
		
		var parent = driveComponent.car;
		var up = parent.GlobalBasis.Y.Normalized();
		var force = Vector3.Zero;
		var offset = wheelRad*-up;
		var intend = driveComponent.IntendedWheelPositions[this];
		Vector3 intended = parent.GlobalBasis.Z * intend.Z + parent.GlobalBasis.X * intend.X + parent.GlobalBasis.Y * intend.Y + parent.GlobalPosition;
       
       
		var groundQuery = PhysicsRayQueryParameters3D.Create(intended + up*wheelRad, intended+offset,exclude: new Array<Rid>(new Rid[1] {driveComponent.car.GetRid()}) );
    	var groundResult = DriveComponent.globalState.IntersectRay(groundQuery);
		if(groundResult.Count>0){
			var direction = ((Vector3)groundResult["normal"]).Cross(GlobalBasis.Z).Normalized();
			if(direction.Dot(driveComponent.car.GlobalBasis.Z.Normalized())<0){
				direction*=-1;
			}
			force = 5*direction*wheelRpm*60*2*Mathf.Pi*(float)delta;
			
		}
		//driveComponent.car.AngularVelocity += driveComponent.car.Basis.Y.Normalized()*(driveComponent.car.Basis.X.Normalized().Dot((force)/(driveComponent.car.LinearVelocity.Length())))/(2*MathF.PI);//car.AngularVelocity.Rotated(car.GlobalBasis.Y.Normalized(),(car.Basis.X.Normalized().Dot(lookTo.Normalized())*car.LinearVelocity.Length()*.01f)/(2*MathF.PI));

		
		return force;
	}
}
