package org.firstinspires.ftc.teamcode.vision;


import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;

import org.opencv.core.Point;

import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class OpenCVSIM extends OpenCvPipeline {
    double cX = 0;
    double cY = 0;

    double width = 0;
        @Override
        public Mat processFrame(Mat input) {
            // Preprocess the frame to detect Blue regions
            Mat BlueMask = preprocessFrame(input);

            // Find contours of the detected Blue regions
            List<MatOfPoint> contours = new ArrayList<>();
            Mat hierarchy = new Mat();
            Imgproc.findContours(BlueMask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            MatOfPoint largestContour = findLargestContour(contours);

            if (largestContour != null) {
                // Draw a red outline around the largest detected object
                Imgproc.drawContours(input, contours, contours.indexOf(largestContour), new Scalar(0, 255, 255), 2);
                Moments moments = Imgproc.moments(largestContour);
                cX = moments.get_m10() / moments.get_m00();
                cY = moments.get_m01() / moments.get_m00();
            }
            return input;
        }
        private Mat preprocessFrame(Mat frame) {
            Mat hsvFrame = new Mat();
            Imgproc.cvtColor(frame, hsvFrame, Imgproc.COLOR_BGR2HSV);
            //blue samples
            Scalar lowerBlue = new Scalar(0, 100, 100);
            Scalar upperBlue = new Scalar(50, 255, 255);


            Mat BlueMask = new Mat();
            Core.inRange(hsvFrame, lowerBlue, upperBlue, BlueMask);
            Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
            Imgproc.morphologyEx(BlueMask, BlueMask, Imgproc.MORPH_OPEN, kernel);
            Imgproc.morphologyEx(BlueMask, BlueMask, Imgproc.MORPH_CLOSE, kernel);
            return BlueMask;
        }

    private MatOfPoint findLargestContour(List<MatOfPoint> contours) {
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
}