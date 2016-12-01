package org.firstinspires.ftc.teamcode.trajectory;

public class Segment{
		
	double pos, vel, accel, time;	
	
	public Segment(double pos, double vel, double accel, double time){
		this.pos = pos;
		this.vel = vel;
		this.accel = accel;			
		this.time = time;
	}
		
	public double getPos(){
		return pos;
	}
		
	public double getVel(){
		return vel;
	}
	
	public double getAccel(){
		return accel;
	}
		
	public double getTime(){
		return time;
	}

}
