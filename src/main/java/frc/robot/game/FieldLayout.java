package frc.robot.game;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;

public class FieldLayout {
    public static final AprilTagFieldLayout kApriltagLayout = AprilTagFieldLayout
            .loadField(AprilTagFields.k2026RebuiltWelded);

    public static final Distance kFieldLength = Meters.of(kApriltagLayout.getFieldLength());
    public static final Distance kFieldWidth = Meters.of(kApriltagLayout.getFieldWidth());

    // Point of interest the robot aims at. Defined in blue-origin field
    // coordinates so it auto-flips for the red alliance.
    public static final Translation2d kPOI = new Translation2d(10.0, 5.0);

    public static class LinesVertical {
        public static final Distance kCenter = kFieldLength.div(2.0);
    }

    public static class LinesHorizontal {
        public static final Distance kCenter = kFieldWidth.div(2.0);
    }

    public static AprilTag[] getAprilTagArrayFromIDs(int ids[]) {
        AprilTag buffer[] = new AprilTag[ids.length];
        int offset = 0;
        for (AprilTag tag : kApriltagLayout.getTags()) {
            for (int id : ids)
                if (id == tag.ID) {
                    buffer[offset++] = tag;
                    break;
                }
        }
        return buffer;
    }
}
