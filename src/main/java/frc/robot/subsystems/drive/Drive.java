package frc.robot.subsystems.drive;

import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drive extends SubsystemBase {
    public static final Drive mInstance = new Drive();

    public Pose2d getPose(){
        return Pose2d.kZero;
    }

    public void resetPose(Pose2d pose){
    }

	public void followChoreoTrajectory(SwerveSample sample) {
	}


}
