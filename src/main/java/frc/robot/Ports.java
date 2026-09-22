package frc.robot;

import org.wpilib.hardware.bus.CANPort;

import com.ctre.phoenix6.CANBus;

public enum Ports {
    FRONT_LEFT_DRIVE(0, RobotConstants.S1),
    FRONT_LEFT_TURN(1, RobotConstants.S1),
    FRONT_RIGHT_DRIVE(2, RobotConstants.S1),
    FRONT_RIGHT_TURN(3, RobotConstants.S1),
    BACK_LEFT_DRIVE(4, RobotConstants.S1),
    BACK_LEFT_TURN(5, RobotConstants.S1),
    BACK_RIGHT_DRIVE(6, RobotConstants.S1),
    BACK_RIGHT_TURN(7, RobotConstants.S1),
    PIGEON(8, RobotConstants.S1),
    EXAMPLE_SUBSYSTEM(9, RobotConstants.S1);

    public final int id;
    public CANBus bus;
    public CANPort canPort; // TODO fix the stupid vendor incompatibilities

    private Ports(int id, CANBus bus) {
        this.id = id;
        this.bus = bus;
    }
    
    private Ports(int id, CANPort canPort) {
        this.id = id;
        this.canPort = canPort;
    }
}
