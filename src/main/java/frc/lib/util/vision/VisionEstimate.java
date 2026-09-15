package frc.lib.util.vision;

import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.Seconds;

import org.wpilib.vision.apriltag.AprilTag;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.units.BaseUnits;
import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.Time;
import org.wpilib.system.Timer;
import frc.robot.game.FieldLayout;
// import frc.robot.subsystems.drive.Drive;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;

public class VisionEstimate {

    private final AprilTag tags[];
    private final Pose2d pose;
    private final Time timestamp;
    private Optional<Distance> averageDistance = Optional.empty();

    public VisionEstimate(Pose2d pose, Time timestamp, AprilTag tags[]) {
        this.pose = pose;
        this.timestamp = timestamp;
        this.tags = tags;
    }

    public VisionEstimate(Pose2d pose, Time timestamp, int tagIDs[]) {
        this(pose, timestamp, FieldLayout.getAprilTagArrayFromIDs(tagIDs));
    }

    public VisionEstimate(Pose2d pose, int tagIDs[]) {
        this(pose, Seconds.of(Timer.getMonotonicTimestamp()), tagIDs);
    }

    public VisionEstimate(Pose2d pose, AprilTag tags[]) {
        this(pose, Seconds.of(Timer.getMonotonicTimestamp()), tags);
    }

    public Pose2d getPose() {
        return pose;
    }

    public Time getTimestamp() {
        return timestamp;
    }

    public AprilTag[] getTags() {
        return tags;
    }

    public VisionEstimate withAverageDistance(Distance averageDistance) {
        this.averageDistance = Optional.of(averageDistance);
        return this;
    }

    public Distance getAverageDistance() {
        // return averageDistance.orElseGet(() -> {
        // Distance average = BaseUnits.DistanceUnit.zero();
        // if (tags.length > 0) {
        // for (AprilTag tag : tags)
        // average = average.plus(Meters.of(tag.pose
        // .getTranslation()
        // .toTranslation2d()
        // .getDistance(Drive.mInstance.getPose().getTranslation())));
        // average = average.div(tags.length);
        // }
        // return average;
        // });
        return Distance.ofBaseUnits(0, Meters);
    }

    public void log(String key) {
        Logger.recordOutput(key + "/Pose", pose);
    }
}
