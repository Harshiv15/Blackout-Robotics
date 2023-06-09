package org.firstinspires.ftc.teamcode.vision.pipelines;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

public class ContourDetectionPipeline extends OpenCvPipeline {
    @Override
    public Mat processFrame(Mat input) {

        //blur and convert to YCrCb color space
        Mat ycrcb = new Mat();
        Imgproc.GaussianBlur(input, input, new Size(5, 5), 0);
        Imgproc.cvtColor(input, ycrcb, Imgproc.COLOR_RGB2YCrCb);

        //thresholding values, may need to be further tuned
        Scalar lowThresh = new Scalar(0, 50, 10);
        Scalar highThresh = new Scalar(255, 180, 95);

        //thresholding for yellow objects
        Mat thresh = new Mat();
        Core.inRange(ycrcb, lowThresh, highThresh, thresh);

        //close to connect broken edges
        //scale everything by 5-6
        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(1, 3));
        Imgproc.morphologyEx(thresh, thresh, Imgproc.MORPH_CLOSE, kernel);

        //find contours
        ArrayList<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(thresh, contours, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        Imgproc.drawContours(input, contours, -1, new Scalar(0,255,0), 2);

        thresh.release();
        ycrcb.release();
        kernel.release();

        return input;
    }
}