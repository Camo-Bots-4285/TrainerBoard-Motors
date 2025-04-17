package frc.robot.subsystems;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSink;
import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Cameras.DriveCamera;

public class CameraServor extends SubsystemBase{
// UsbCamera camera1;
// UsbCamera camera2;
// VideoSink server;

//     public static DriveCamera main = new DriveCamera(
//         0,
//         60,
//         480,
//         320
//     );

//     public static DriveCamera second = new DriveCamera(
//         1,
//         60,
//         480,
//         320
//     );


// public CameraServor() {
//     camera1 = CameraServer.startAutomaticCapture(0);
//     camera2 = CameraServer.startAutomaticCapture(1);
//     server = CameraServer.getServer();
//     camera1.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
//     camera2.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
//}
@Override
public void periodic() {
    // if (joy1.getTriggerPressed()) {
    //     System.out.println("Setting camera 2");
    //     server.setSource(camera2);
    // } else if (joy1.getTriggerReleased()) {
    //     System.out.println("Setting camera 1");
    //     server.setSource(camera1);
    // }
}
}
