package org.firstinspires.ftc.teamcode.trajectory;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;

import static org.firstinspires.ftc.teamcode.opmodes.TelemetryHolder.telemetry;
public class TrajectoryFollower{
	
	private double kP, kI, kD, kV, kA, dt;
	double PID, error, sumError=0, changeError=0, prevError=0;
	private Segment[] traj;
	private int curr_segment;
	private double output, feedForwardValue, feedBackValue;
		
	public TrajectoryFollower(){

	}

	public void setTrajectory(Segment[] traj){
		this.traj = traj;
	}

	public void setGains(double kV, double kA, double kP, double kI, double kD){
		this.kV = kV;
		this.kA = kA;
		this.kP = kP;
		this.kI = kI;
		this.kD = kD;
	}

	public void setLoopTime(double dt){
		this.dt = dt;
	}

	public void resetController(){
		sumError = 0;
		changeError = 0;
		prevError = 0;	
		error = 0;
		curr_segment = 0;
	}
	
	private double calcFeedForward(double curr_vel, double curr_accel){
		//System.out.println("KV" + kV);
		//System.out.println("FF " + (kV*curr_vel + kA*curr_accel));
		return kV*curr_vel + kA*curr_accel;
	}	

	private double calcFeedBack(Segment s, double setpoint_pos, double curr_pos){
		error = setpoint_pos-curr_pos;
		sumError+=error;
		changeError = (error-prevError)/dt-s.getVel();
		prevError = error;
		PID = kP*error + kI*sumError + kD*changeError;
		return PID;
	}
	
	public boolean isFinished(){
		return curr_segment >= traj.length;
	}

	public double calcMotorOutput(double curr_actual_dist){
			Segment s = traj[curr_segment];
            telemetry.addData("vel", s.getVel());
            telemetry.addData("pos", s.getPos());
			feedForwardValue = calcFeedForward(s.vel, s.accel);
			feedBackValue = calcFeedBack(s, s.pos, curr_actual_dist);
			output = feedForwardValue + feedBackValue;
            telemetry.addData("output", output);
            telemetry.addData("current_seg", curr_segment);
			telemetry.addData("encoders", DriveTrain.getInstance().ticksToInches(DriveTrain.getInstance().getAverageEncoderValue()));
			curr_segment++;
			output = Range.clip(output, -1, 1);
			//System.out.println(Double.toString(curr_actual_dist) + "," + Double.toString(s.pos) + "," + Double.toString(s.vel) + "," + Double.toString(s.accel) +","+ Double.toString(output));
		telemetry.update();
        return output;
	}
	
}
