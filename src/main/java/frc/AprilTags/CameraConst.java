package frc.AprilTags;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public class CameraConst {
  static final Transform3d robotToCam = new Transform3d(
      new Translation3d(0.00, .38, 0.15),
      new Rotation3d(0, 0, 0));

}
