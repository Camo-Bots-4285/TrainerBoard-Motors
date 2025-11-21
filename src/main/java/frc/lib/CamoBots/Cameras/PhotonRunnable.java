//DO NOT TOUCH sould not need to update if verion is off see Photon Lib instead
package frc.lib.CamoBots.Cameras;


import java.util.LinkedList;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import java.util.concurrent.atomic.AtomicReference;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;


/**
 * Runnable that gets AprilTag data from PhotonVision.
 */
public class PhotonRunnable implements Runnable {
  private String CameraName;


  private final PhotonPoseEstimator photonPoseEstimator;
  private final PhotonCamera photonCamera;
  private final AtomicReference<EstimatedRobotPose> atomicEstimatedRobotPose = new AtomicReference<EstimatedRobotPose>();
  public List<PhotonPipelineResult> photonResults;

  // Keep track of the latest valid targets for adaptive std dev calculation
  private volatile List<PhotonTrackedTarget> latestValidTargets = List.of();

  //Filter Out unwanted tags
  private static final Set<Integer> IGNORED_TAG_IDS = Set.of(0, 3, 7);

  //You can tune these values as you see fit.
  private static final double BASE_STD_DEV = 0.02; // meters
  private static final double BASE_THETA_STD_DEV = Math.toRadians(3.75); // radians

  public static final double FIELD_LENGTH_METERS = 16.542;
  public static final double FIELD_WIDTH_METERS = 8.2042;

  /** Minimum target ambiguity. Targets with higher ambiguity will be discarded */
  // Raise this value to reject less accurate poses 0.2 recommended by photon
  // vision
  public static final double APRILTAG_AMBIGUITY_THRESHOLD = 0.2;

  // Initialize logging values
    List<Pose3d> allTagPoses = new LinkedList<>();
    List<Pose3d> allRobotPosesAccepted = new LinkedList<>();
    List<Pose3d> allRobotPosesRejected = new LinkedList<>();

    private AprilTagFieldLayout aprilTagLayout;

  public PhotonRunnable(String CameraName, Transform3d cameraToRobot) {
    this.CameraName=CameraName;
    this.photonCamera = new PhotonCamera(CameraName);
    PhotonPoseEstimator photonPoseEstimator = null;
    aprilTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
      // PV estimates will always be blue, they'll get flipped by robot thread
      aprilTagLayout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
      if (photonCamera != null) {
        photonPoseEstimator = new PhotonPoseEstimator(
          aprilTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, cameraToRobot);
      }
    this.photonPoseEstimator = photonPoseEstimator;
  }

  // @Override
  // public void run() {      
  //   // Get AprilTag data
  //   if (photonPoseEstimator != null && photonCamera != null /*&& !RobotState.isAutonomous()*/) {
  //     PhotonPipelineResult result = photonCamera.getLatestResult();
  //       // Iterate over the results
  //         if (result.hasTargets() && (result.targets.size() > 1 || result.targets.get(0).getPoseAmbiguity() < APRILTAG_AMBIGUITY_THRESHOLD)) {
  //           photonPoseEstimator.update(result).ifPresent(estimatedRobotPose -> {
  //             var estimatedPose = estimatedRobotPose.estimatedPose;
  //             // Make sure the measurement is on the field
  //             if (estimatedPose.getX() > 0.0 && estimatedPose.getX() <= FIELD_LENGTH_METERS
  //                 && estimatedPose.getY() > 0.0 && estimatedPose.getY() <= FIELD_WIDTH_METERS) {
  //               atomicEstimatedRobotPose.set(estimatedRobotPose);
  //               latestValidTargets = result.targets;
  //             }
  //           }); 
  //     }
  //   }  
  // }

  // @Override
  // public void run() {      
  //   // Get AprilTag data
  //   if (photonPoseEstimator != null && photonCamera != null /*&& !RobotState.isAutonomous()*/) {
  //     photonResults = photonCamera.getAllUnreadResults();
  //     if (!photonResults.isEmpty()) {
  //       // Iterate over the results
  //       for (PhotonPipelineResult result : photonResults) {
  //         if (result.hasTargets() && (result.targets.size() > 1 || result.targets.get(0).getPoseAmbiguity() < APRILTAG_AMBIGUITY_THRESHOLD)) {
  //           photonPoseEstimator.update(result).ifPresent(estimatedRobotPose -> {
  //             var estimatedPose = estimatedRobotPose.estimatedPose;
  //             // Make sure the measurement is on the field
  //             if (estimatedPose.getX() > 0.0 && estimatedPose.getX() <= FIELD_LENGTH_METERS
  //                 && estimatedPose.getY() > 0.0 && estimatedPose.getY() <= FIELD_WIDTH_METERS) {
  //               atomicEstimatedRobotPose.set(estimatedRobotPose);
  //               latestValidTargets =result.targets;
  //             }
  //           });
  //           break; //can add this in to stop in the first valid one
  //         }
  //       }  
  //     }
  //   }  
  // }


@Override
public void run() {   
  //Check for an estimate and a camera   
  if (photonPoseEstimator != null && photonCamera != null /*&& !RobotState.isAutonomous()*/) {
    
    //Grab all unread runs
    photonResults = photonCamera.getAllUnreadResults();
    
    //Make sure there is unread estimate to read
    if (!photonResults.isEmpty()) {
      //Goes thought each unread estimate one by one
      for (PhotonPipelineResult result : photonResults) {
        //Move on to nest estimate if there are not any targets
        if (!result.hasTargets()) continue;

        //Logs all tags that where seen in this result
          for (PhotonTrackedTarget target : result.getTargets()) {
              Optional<Pose3d> tagPose = aprilTagLayout.getTagPose(target.getFiducialId());
              if (tagPose.isPresent()) {
                  allTagPoses.add(tagPose.get());
              }
          }

        // Filter out ignored tag IDs
        var validTargets = result.targets.stream()
            .filter(t -> !IGNORED_TAG_IDS.contains(t.getFiducialId()))
            .toList();

        // If no valid targets, skip this result
        if (validTargets.isEmpty()) continue;
        // Check ambiguity and target count
        if (validTargets.size() > 1 || validTargets.get(0).getPoseAmbiguity() < APRILTAG_AMBIGUITY_THRESHOLD) {
          // Optional: log which tag you're using
          // System.out.println("Using tag ID: " + validTargets.get(0).getFiducialId());

          // Estimate pose using original result; estimator will use best available target
          photonPoseEstimator.update(result).ifPresent(estimatedRobotPose -> {
            var estimatedPose = estimatedRobotPose.estimatedPose;
            if (estimatedPose.getX() > 0.0 && estimatedPose.getX() <= FIELD_LENGTH_METERS
                && estimatedPose.getY() > 0.0 && estimatedPose.getY() <= FIELD_WIDTH_METERS) {
              atomicEstimatedRobotPose.set(estimatedRobotPose);
              latestValidTargets = validTargets;
              allRobotPosesAccepted.add(estimatedRobotPose.estimatedPose);
            }
            else{allRobotPosesRejected.add(estimatedRobotPose.estimatedPose);}
          });
          break; // stop after first valid result
        }
      }
    }
  }
}

  /**
   * Gets the latest robot pose. Calling this will only return the pose once. If it returns a non-null value, it is a
   * new estimate that hasn't been returned before.
   * This pose will always be for the BLUE alliance. It must be flipped if the current alliance is RED.
   * @return latest estimated pose
   */
  public EstimatedRobotPose grabLatestEstimatedPose() {
    return atomicEstimatedRobotPose.getAndSet(null);
  }

  public Matrix<N3, N1> getAdaptiveStdDevs() {
    return computeStdDevsFromTargets(latestValidTargets);
  }
  
  public Matrix<N3, N1> computeStdDevsFromTargets(List<PhotonTrackedTarget> targets) {
      if (targets.isEmpty()) {
          // fallback if somehow empty
          return VecBuilder.fill(0.5, 0.5, Math.toRadians(20));
      }
  
      double totalDist = 0;
      double totalAmbiguity = 0;
  
      for (PhotonTrackedTarget target : targets) {
          double distance = target.getBestCameraToTarget().getTranslation().getNorm(); // meters
          double ambiguity = target.getPoseAmbiguity();

        if (distance < 1.5) {
            distance = 1.5;
        }

          totalDist += distance;
          totalAmbiguity += ambiguity;
      }
  
      int count = targets.size();
      double avgDist = totalDist / count;
      double avgAmbiguity = totalAmbiguity / count;
  
      double scale = (avgDist * avgDist * (1 + avgAmbiguity)) / count;
  
      double linearStdDev = BASE_STD_DEV * scale;
      double thetaStdDev = BASE_THETA_STD_DEV * scale;
  
      return VecBuilder.fill(linearStdDev, linearStdDev, thetaStdDev);
  }

  public void logCamera(){
    Logger.recordOutput(
        "7_Vision/Photon/"+ CameraName + "/TagPoses", allTagPoses.toArray(new Pose3d[allTagPoses.size()]));
    Logger.recordOutput(
        "7_Vision/Photon/"+ CameraName + "/RobotPosesAccepted",
        allRobotPosesAccepted.toArray(new Pose3d[allRobotPosesAccepted.size()]));
    Logger.recordOutput(
        "7_Vision/Photon/"+ CameraName + "/RobotPosesRejected",
        allRobotPosesRejected.toArray(new Pose3d[allRobotPosesRejected.size()]));

        //After recording data clears it for next run
        allTagPoses.clear();
        allRobotPosesAccepted.clear();
        allRobotPosesRejected.clear();
  }
}
