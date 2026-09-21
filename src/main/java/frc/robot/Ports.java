package frc.robot;

import com.ctre.phoenix6.CANBus;

public enum Ports {
    EXAMPLE_SUBSYSTEM(5, RobotConstants.systemCore);

    public final int id;
    public final CANBus bus;

    private Ports(int id, CANBus bus) {
        this.id = id;
        this.bus = bus;
    }
}
