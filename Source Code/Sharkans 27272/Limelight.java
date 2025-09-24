package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import java.util.List;

public class Limelight {
    Limelight3A limelight;

    SparkFunOTOS.Pose2D lastPosition = new SparkFunOTOS.Pose2D();
    boolean isValid = true;

    double averageX = 0;
    double averageY = 0;

    public void init(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(30); // This sets how often we ask Limelight for data (15 times per second)
        limelight.start(); // This tells Limelight to start looking!
        limelight.pipelineSwitch(0);
    }

    public SparkFunOTOS.Pose2D GetLimelightData(boolean redAlliance, double orientation)
    {
        limelight.updateRobotOrientation(orientation);

//        LLResult result = limelight.getLatestResult();
//        LLResultTypes.FiducialResult fiducialResult = limelight.getLatestResult();
//        Pose3D pose = result.getBotpose();

        List<LLResultTypes.FiducialResult> fiducialResult = limelight.getLatestResult().getFiducialResults();

        SparkFunOTOS.Pose2D output = new SparkFunOTOS.Pose2D(0,0,0);

        if (fiducialResult.isEmpty()) {
            isValid = false;
            return output;
        }
        else
        {
            isValid = true;
        }

        Pose3D pose = fiducialResult.get(0).getRobotPoseFieldSpace();

        output.x = pose.getPosition().x;
        output.y = pose.getPosition().y;

        output = ProcessCoordinates(redAlliance, output);

//        output.x = LowPass(averageX, output.x);
//        output.y = LowPass(averageY, output.y);

        if (output.x != 0 && output.y != 0)
        {
            lastPosition = output;
        }

        return output;
    }

    private SparkFunOTOS.Pose2D ProcessCoordinates(boolean redAlliance, SparkFunOTOS.Pose2D pos)
    {
        SparkFunOTOS.Pose2D processedPosition = new SparkFunOTOS.Pose2D();
        processedPosition.x = MtoIn(pos.x);
        processedPosition.y = 2 - Math.abs(pos.y);

        processedPosition.y = MtoIn(processedPosition.y);

        if (!redAlliance)
        {
            processedPosition.x *= -1;
        }
        else
        {
            processedPosition.y *= -1;
        }

        processedPosition.x -= 3;
        processedPosition.y -= 10;

//        processedPosition.x = LowPass(averageX, processedPosition.x);
//        processedPosition.y = LowPass(averageY, processedPosition.y);

        return processedPosition;
    }

    private double LowPass(double average, double value)
    {
        average = (average * 0.5) + (value * 0.5);
        return average;
    }

    private double MtoIn(double input)
    {
        return input / 0.0254;
    }

    public boolean GetIsValid()
    {
        return isValid;
    }

    public SparkFunOTOS.Pose2D GetLastPosition()
    {
        return lastPosition;
    }
}
