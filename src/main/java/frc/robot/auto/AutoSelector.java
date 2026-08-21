package frc.robot.auto;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.auto.autos.ExampleAuto;

public class AutoSelector {
    private AutoChooser mAutoChooser = new AutoChooser();

    private Pose2d startPose = new Pose2d();

    public AutoSelector(AutoFactory autoFactory) {
        mAutoChooser.addRoutine("Example Auto", () -> generateAuto(new ExampleAuto(autoFactory)));
        mAutoChooser.addRoutine("Do Nothing", () -> autoFactory.newRoutine("Do Nothing"));

        SmartDashboard.putData(mAutoChooser);
    }

    private AutoRoutine generateAuto(AutoModeBase auto) {
        startPose = auto.getInitialPose();
        return auto.getRoutine();
    }

    public Command getSelectedCommand() {
        return mAutoChooser.selectedCommand();
    }

    public AutoChooser getAutoChooser() {
        return mAutoChooser;
    }

    public Pose2d getSelectedAutoStartingPose() {
        return startPose;
    }
}
