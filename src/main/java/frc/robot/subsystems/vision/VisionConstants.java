package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

public final class VisionConstants {
    // Vision Standard Deviations
    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);

    public static final AprilTagFieldLayout kTagLayout = AprilTagFields.kDefaultField.loadAprilTagLayoutField();
    public static final Transform3d unicornLeftTransform = new Transform3d(new Translation3d(Units.inchesToMeters(-11.302), Units.inchesToMeters(-12.747), Units.inchesToMeters(8.613)), new Rotation3d(0, Units.degreesToRadians(-28.125), Units.degreesToRadians(210)));
    public static final Transform3d meowRightTransform = new Transform3d(new Translation3d(Units.inchesToMeters(-11.302), Units.inchesToMeters(12.747), Units.inchesToMeters(8.613)), new Rotation3d(0, Units.degreesToRadians(-28.125), Units.degreesToRadians(150)));

    public static boolean inField(Pose3d pose) {
        return (pose.getX() > 0
            && pose.getX() < Units.inchesToMeters(651.223)
            && pose.getY() > 0
            && pose.getY() < Units.inchesToMeters(323.277)
        );
    }
}
