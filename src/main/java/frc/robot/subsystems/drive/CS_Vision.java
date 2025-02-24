package frc.robot.subsystems.drive;

import frc.robot.subsystems.drive.CS_VisionConstants.CS_Cameras;
import java.util.ArrayList;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

public class CS_Vision {
  private ArrayList<PhotonPoseEstimator> poseEstimators;
  private ArrayList<CS_Cameras> cameras = new ArrayList<>();
  private double lastEstTimestamp = 0;

  public void CS_Vision() {
    for (CS_VisionConstants.CS_Cameras camera : CS_VisionConstants.CS_Cameras.values()) {
      cameras.add(camera);
      poseEstimators.add(
          new PhotonPoseEstimator(
              CS_VisionConstants.kTagLayout,
              PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
              camera.getCameraToRobot()));
    }

    // photonEstimator =
    // new PhotonPoseEstimator(
    // CS_VisionConstants.kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, ATCamera,
    // CameraPose);
    // photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

  }

  public void getEstimatedpose() {

    // Do nothing
  }
}
    // Correct pose estimate with vision measurements
    // var backVisionEst = backVision.getEstimatedGlobalPose();
    // var frontVisionEst = frontVision.getEstimatedGlobalPose();

    // backVisionEst.ifPresentOrElse(
    //     est -> {
    //       var estPose = est.estimatedPose.toPose2d();
    //       // Change our trust in the measurement based on the tags we can see
    //       var estStdDevs = backVision.getEstimationStdDevs(estPose);

    //       swerveDrive.addVisionMeasurement(
    //           est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
    //     },
    //     () ->
    //         frontVisionEst.ifPresent(
    //             est -> {
    //               if (MathUtil.isNear(Presets.kStow.getExtInches(), m_armExt.getExtInches(), 1)
    //                   && MathUtil.isNear(
    //                       Presets.kStow.getRotDegrees(), m_armRot.getRotDegrees(), 5)) {
    //                 var estPose = est.estimatedPose.toPose2d();
    //                 // Change our trust in the measurement based on the tags we can see
    //                 var estStdDevs = frontVision.getEstimationStdDevs(estPose);

    //                 swerveDrive.addVisionMeasurement(
    //                     est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
    //               }
    //             }));
