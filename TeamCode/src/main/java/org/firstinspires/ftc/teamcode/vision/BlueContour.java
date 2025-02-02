package org.firstinspires.ftc.teamcode.vision;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.opencv.core.*;
@Disabled
public class BlueContour extends OpenCvPipeline {
    private Mat hsvImage = new Mat();
    private Mat blueMask = new Mat();
    private List<MatOfPoint> contours = new ArrayList<>();

    @Override
    public Mat processFrame(Mat input) {
        // Convert input image to HSV color space
        Imgproc.cvtColor(input, hsvImage, Imgproc.COLOR_BGR2HSV);

        // Define HSV range for detecting blue
        Scalar lowerBlue = new Scalar(100, 150, 70); // Adjust as needed
        Scalar upperBlue = new Scalar(140, 255, 255); // Adjust as needed

        // Create a mask for blue regions
        Core.inRange(hsvImage, lowerBlue, upperBlue, blueMask);

        // Apply morphological operations to clean up the mask
        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
        Imgproc.morphologyEx(blueMask, blueMask, Imgproc.MORPH_OPEN, kernel);
        Imgproc.morphologyEx(blueMask, blueMask, Imgproc.MORPH_CLOSE, kernel);

        // Find contours in the blue mask
        contours.clear();
        Mat hierarchy = new Mat();
        Imgproc.findContours(blueMask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        // Draw contours on the input image
        for (MatOfPoint contour : contours) {
            Imgproc.drawContours(input, contours, contours.indexOf(contour), new Scalar(0, 255, 0), 2);
        }

        // Return the input frame with contours drawn
        return input;
    }
}
