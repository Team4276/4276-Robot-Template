package frc.robot;

import com.ctre.phoenix6.CANBus;

public enum Ports {
    EXAMPLE_SUBSYSTEM(8, RobotConstants.rio);

    public final int id;
    public final CANBus bus;

    private Ports(int id, CANBus bus){
        this.id = id;
        this.bus = bus;
    }
}
