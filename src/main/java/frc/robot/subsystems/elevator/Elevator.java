package frc.robot.subsystems.elevator;

import frc.lib.bases.ServoMotorSubsystem;
import frc.lib.io.MotorIO;
import frc.lib.io.MotorIO.Setpoint;

public class Elevator extends ServoMotorSubsystem<MotorIO> {
    // Named setpoints first, built from *Constants values.
    public static final Setpoint HOME = Setpoint
            .withMotionMagicSetpoint(ElevatorConstants.kHomePosition);
    public static final Setpoint LEVEL_1 = Setpoint
            .withMotionMagicSetpoint(ElevatorConstants.kLevel1Position);
    public static final Setpoint LEVEL_2 = Setpoint
            .withMotionMagicSetpoint(ElevatorConstants.kLevel2Position);
    public static final Setpoint LEVEL_3 = Setpoint
            .withMotionMagicSetpoint(ElevatorConstants.kLevel3Position);

    // Singleton instance.
    public static final Elevator mInstance = new Elevator();

    public Elevator() {
        super(
                ElevatorConstants.getMotorIO(),
                "Elevator",
                ElevatorConstants.kEpsilonThreshold,
                ElevatorConstants.getServoHomingConfig());
    }
}
