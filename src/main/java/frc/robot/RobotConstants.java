package frc.robot;

import com.ctre.phoenix6.CANBus;

public class RobotConstants {
    public static enum Mode {
        /** Running on a real robot. */
        REAL,

        /** Running a physics simulator. */
        SIM,

        /** Replaying from a log file. */
        REPLAY,
    }

    public static Mode getMode() {
        return mode;
    }

    public static enum RobotType {
        COMPBOT,
        SIMBOT
    }

    public static Mode mode = Mode.SIM;

    public static RobotType getType() {
        return switch (mode) {
            case REAL -> RobotType.COMPBOT;
            case REPLAY -> RobotType.COMPBOT;
            case SIM -> RobotType.SIMBOT;
        };
    }

    public static final CANBus canivore1 = new CANBus("canivore1");
    public static final CANBus rio = new CANBus();

    public static final boolean isTuning = true;

    public static final boolean simulateVision = false;
}
