package frc.robot.lib.Cameras;

//Works
import org.opencv.core.Mat;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cameraserver.CameraServer;
import org.opencv.imgcodecs.Imgcodecs;

public class StaticImageStream {
    private final CvSource stream;
    private final Mat image;

    public StaticImageStream(String streamName, String imagePath, int width, int height) {
        image = Imgcodecs.imread(imagePath);
        if (image.empty()) {
            throw new RuntimeException("Failed to load image: " + imagePath);
        }

        stream = CameraServer.putVideo(streamName, width, height);
    }

    public void update() {
        if (stream != null && !image.empty()) {
            stream.putFrame(image);
        }
    }
}


//Try this will not update as much
// import org.opencv.core.Mat;
// import edu.wpi.first.cscore.CvSource;
// import edu.wpi.first.cameraserver.CameraServer;
// import org.opencv.imgcodecs.Imgcodecs;

// public class StaticImageStream {
//     private final CvSource stream;
//     private final Mat image;

//     public StaticImageStream(String streamName, String imagePath, int width, int height) {
//         // Load the static image from file
//         image = Imgcodecs.imread(imagePath);
//         if (image.empty()) {
//             throw new RuntimeException("Failed to load image: " + imagePath);
//         }

//         // Create an MJPEG stream for the dashboard
//         stream = CameraServer.putVideo(streamName, width, height);

//         // Start background thread to push the static image every 5 seconds
//         new Thread(() -> {
//             try {
//                 while (!Thread.currentThread().isInterrupted()) {
//                     stream.putFrame(image);
//                     Thread.sleep(5000); // Push every 5 seconds
//                 }
//             } catch (InterruptedException e) {
//                 Thread.currentThread().interrupt(); // Clean exit on interrupt
//             }
//         }, streamName + "-Thread").start();
//     }

//     // Optional: manually trigger a frame push if needed
//     public void updateOnce() {
//         if (stream != null && !image.empty()) {
//             stream.putFrame(image);
//         }
//     }
// }

