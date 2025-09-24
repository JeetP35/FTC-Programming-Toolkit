package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;


//@TeleOp(name = "OpenCV Testing")

public class opencv{
    double cX = 0;
    double cY = 0;
    double width = 0;
    double height = 0 ;

    double theta = 0;
    double wrappedTheta = 0;

    private OpenCvCamera controlHubCam;  // Use OpenCvCamera class from FTC SDK
    private static final int CAMERA_WIDTH = 640; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 360; // height of wanted camera resolution

    // Calculate the distance using the formula
//    public static final double objectWidthInRealWorldUnits = 3.5;  // Replace with the actual width of the object in real-world units
//    public static final double focalLength = 720;  // Replace with the focal length of the camera in pixels

    private final double objW = 3.5;
    private final double objH = 1.5;

    private Mat emptyMat = new Mat();
    List<Point> points;

    double yMaxX = 0;

    double minDiff = 0;
    double maxDiff = 0;


    double focalLength = 3.67; // mm
    double realHeight = 88.9; //mm
    double realWidth = 38.1; //mm
    double sensorWidth = 8.466; //mm
    double sensorHeight = 8.466; //mm
    double imageWidth = 640; // px
    double imageHeight = 360; // px

    // outlier checker
    ElapsedTime derivRuntime = new ElapsedTime();
    ElapsedTime detectionRuntime = new ElapsedTime();

    double hz = 8; //5
    double distanceDerivMax = 100;
    boolean firstDetection = true;

    Mat persistentDetectionMatrix = new Mat();
    Mat persistentHsvMatrix = new Mat();

    boolean detecting = false;

    double horizontalFOV = 90; // degrees

    double camCenterOffset = 4.625; //inches

    public void init(HardwareMap hwMap)
    {
        initOpenCV(hwMap);
    }

    /*
    Real -> Projected
    20 -> 10 = 2
    10 -> 5 = 2
     */

    private void initOpenCV(HardwareMap hwMap) {
        // Create an instance of the camera
        int cameraMonitorViewId = hwMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hwMap.appContext.getPackageName());

        // Use OpenCvCameraFactory class from FTC SDK to create camera instance
        controlHubCam = OpenCvCameraFactory.getInstance().createWebcam(
                hwMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        controlHubCam.setPipeline(new BlobDetectionPipeline());

        controlHubCam.openCameraDevice();
        controlHubCam.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
    }
    class BlobDetectionPipeline extends OpenCvPipeline {
        @Override
        public Mat processFrame(Mat input) {
            if (!firstDetection)
            {
                return persistentDetectionMatrix;
            }

            // Find contours of the detected yellow regions
            List<MatOfPoint> contours = new ArrayList<>();
//            Mat hierarchy = new Mat();
            Imgproc.findContours(preprocessFrame(input), contours, emptyMat, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            // Find the largest yellow contour (blob)
            MatOfPoint largestContour = findLargestContour(contours);

            if (largestContour != null) {
                // Draw a red outline around the largest detected object
                Imgproc.drawContours(input, contours, contours.indexOf(largestContour), new Scalar(255, 0, 0), 2);
                // Calculate the width of the bounding box
                width = calculateWidth(largestContour);
                height = calculateHeight(largestContour);

                String widthLabel = "Width: " + (int) width + " pixels";
                Imgproc.putText(input, widthLabel, new Point(cX + 10, cY + 20), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0), 2);

                String heightLabel = "Height: " + (int) height + " pixels";
                Imgproc.putText(input, heightLabel, new Point(cX + 10, cY + 60), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0), 2);
//                Display the Distance
                String distanceLabel = "Distance: " + GetDistance() + " inches";
                Imgproc.putText(input, distanceLabel, new Point(cX + 10, cY + 100), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0), 2);
//                 Calculate the centroid of the largest contour
                Moments moments = Imgproc.moments(largestContour);

                cX = moments.get_m10() / moments.get_m00();
                cY = moments.get_m01() / moments.get_m00();

                double numerator = 2 * moments.get_mu11();
                double denominator = moments.get_mu20() - moments.get_mu02();

                theta = Math.atan2(numerator,denominator) / 2;
                theta *= 180/Math.PI;

                if (theta > 0)
                {
                    theta = 90 - theta + 90;
                }
                theta = Math.abs(theta);
            }

            persistentDetectionMatrix = input;

            return persistentDetectionMatrix;
        }

        private Mat preprocessFrame(Mat frame) {
            if (!firstDetection)
            {
                return persistentHsvMatrix;
            }

            Mat hsvFrame = new Mat();
            Imgproc.cvtColor(frame, hsvFrame, Imgproc.COLOR_BGR2HSV);

            // tuned values
            Scalar lowerYellow = new Scalar(78, 75, 95); // 91, 150, 0 | 78, 75, 95
            Scalar upperYellow = new Scalar(110, 255, 255); // 114, 255, 255 | 110, 255, 255

            Scalar lowerBlue = new Scalar(0,67,0);
            Scalar upperBlue = new Scalar(20,255,255);

            Scalar lowerRed = new Scalar(109,63,0);
            Scalar upperRed = new Scalar(143,254,255);

            Mat mask = new Mat();
            Core.inRange(hsvFrame, lowerYellow, upperYellow, mask);

            Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
            Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_OPEN, kernel);
            Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_CLOSE, kernel);

            persistentHsvMatrix = mask;
            firstDetection = false;

            return persistentHsvMatrix;
        }

        private MatOfPoint findLargestContour(@NonNull List<MatOfPoint> contours) {
            double maxArea = 0;
            MatOfPoint largestContour = null;

            for (MatOfPoint contour : contours) {
                double area = Imgproc.contourArea(contour);
                if (area > maxArea) {
                    maxArea = area;
                    largestContour = contour;
                }
            }
            return largestContour;
        }
        private double calculateWidth(MatOfPoint contour) {
            Rect boundingRect = Imgproc.boundingRect(contour);
            return boundingRect.width;
        }

        private double calculateHeight(MatOfPoint contour) {
            Rect boundingRect = Imgproc.boundingRect(contour);
            return boundingRect.height;
        }
    }

    public void StartStream(Telemetry tel)
    {
//        FtcDashboard dashboard = FtcDashboard.getInstance();
//        tel = new MultipleTelemetry(tel, dashboard.getTelemetry());
//        FtcDashboard.getInstance().startCameraStream(controlHubCam, 30);
    }

    public void StopStream()
    {
//        controlHubCam.stopStreaming();
    }

    public int GetCX()
    {
        return (int)cX;
    }

    public List<Point> GetPoints()
    {
        return points;
    }

    public double GetWrappedTheta()
    {
        return wrappedTheta;
    }

    public double GetMinDiff()
    {
        return minDiff;
    }

    public double GetMaxDiff()
    {
        return maxDiff;
    }

    public void SetDetecting(boolean input)
    {
        detecting = input;
    }

    public boolean GetDetecting()
    {
        return detecting;
    }

    public int GetCameraWidth()
    {
        return CAMERA_WIDTH;
    }

    public int GetCameraHeight()
    {
        return CAMERA_HEIGHT;
    }

    public double GetDistance()
    {
        double distanceH = focalLength * realHeight * imageHeight / (height * sensorHeight);
        double distanceW = focalLength * realWidth * imageWidth / (width * sensorWidth);
        double average = (distanceH + distanceW) / 2;

        // convert mm to inches
        average *= 0.0394;

        double coef = 2;

        return coef * average;
    }

    public double GetXOffset()
    {
        // y tan theta = x

        // gets the direction and magnitude on a normalized scale of the angle
        double theta = (cX - imageWidth/2) / (imageWidth / 2);

        // converts normalized scale to degrees
        theta *= (horizontalFOV/2);

        double xOffset = GetDistance() * Math.tan(theta * Math.PI/180);

        // move from cam center to real center
        xOffset -= camCenterOffset;

        return xOffset;
    }

    private boolean IsOutlierDetected(double distanceValue, double lastDistance)
    {
        double derivative = (distanceValue - lastDistance) / derivRuntime.seconds();

        return derivative > distanceDerivMax;
    }

    private double LowPass(double average, double newValue) {
        average = (average * 0.85) + (0.15 * newValue);

        return average;
    }

    public double GetTheta()
    {
        return theta;
    }

    public void SetFirstDetection(boolean input)
    {
        firstDetection = input;
    }
    public boolean GetFirstDetection() {return firstDetection;}
}
