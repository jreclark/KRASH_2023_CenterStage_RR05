package org.firstinspires.ftc.teamcode.processors;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import java.util.ArrayList;
import java.util.List;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.*;
//import org.opencv.features2d.
import org.opencv.imgproc.*;

public class ContourTSEProcessor implements VisionProcessor {
    public enum Selected {
        NONE,
        LEFT,
        MIDDLE,
        RIGHT}

    //640 pixels wide
    private final double LEFT_LIMIT = 150;
    private final double RIGHT_LIMIT = 490;

    //Outputs
    private Mat cvResizeOutput = new Mat();
    private Mat blurOutput = new Mat();
    private Mat hsvThresholdOutput = new Mat();
    private Mat cvErodeOutput = new Mat();
    private Mat cvDilateOutput = new Mat();
    private ArrayList<MatOfPoint> findContoursOutput = new ArrayList<MatOfPoint>();
    private ArrayList<MatOfPoint> filterContoursOutput = new ArrayList<MatOfPoint>();

    private MatOfPoint selectedContour = new MatOfPoint();

    public enum Alliance {
        RED,
        BLUE}

    private Alliance alliance = Alliance.BLUE;

    private Selected selection = Selected.LEFT;

    /**
     * This is the primary method that runs the entire pipeline and updates the outputs.
     */
    public void process(Mat source0) {
        // Step CV_resize0:
        Mat cvResizeSrc = source0;
//        Size cvResizeDsize = new Size(0, 0);
//        double cvResizeFx = 1;  // Changed back to 1 in order to deal with scaling Rect later
//        double cvResizeFy = 1;
//        int cvResizeInterpolation = Imgproc.INTER_LINEAR;
//        cvResize(cvResizeSrc, cvResizeDsize, cvResizeFx, cvResizeFy, cvResizeInterpolation, cvResizeOutput);
        cvResizeOutput = cvResizeSrc;

        // Step Blur0:
        Mat blurInput = cvResizeOutput;
        BlurType blurType = BlurType.get("Gaussian Blur");
        double blurRadius = 5.4;
        blur(blurInput, blurType, blurRadius, blurOutput);

        // Step HSV_Threshold0:
        Mat hsvThresholdInput = blurOutput;
        final double[] HSVTHRESHOLDHUEBLUE = {95, 120};
        final double[] HSVTHRESHOLDHUERED = {150, 180};
        double[] hsvThresholdSaturation = {90, 255.0};
        double[] hsvThresholdValue = {0.0, 255.0};
        double[] hsvThresholdHue = (alliance == Alliance.BLUE) ? HSVTHRESHOLDHUEBLUE:HSVTHRESHOLDHUERED;

        hsvThreshold(hsvThresholdInput, hsvThresholdHue, hsvThresholdSaturation, hsvThresholdValue, hsvThresholdOutput);

        // Step CV_erode0:
        Mat cvErodeSrc = hsvThresholdOutput;
        Mat cvErodeKernel = new Mat();
        Point cvErodeAnchor = new Point(-1, -1);
        double cvErodeIterations = 5.0;
        int cvErodeBordertype = Core.BORDER_CONSTANT;
        Scalar cvErodeBordervalue = new Scalar(-1);
        cvErode(cvErodeSrc, cvErodeKernel, cvErodeAnchor, cvErodeIterations, cvErodeBordertype, cvErodeBordervalue, cvErodeOutput);

        // Step CV_dilate0:
        Mat cvDilateSrc = cvErodeOutput;
        Mat cvDilateKernel = new Mat();
        Point cvDilateAnchor = new Point(-1, -1);
        double cvDilateIterations = 5.0;
        int cvDilateBordertype = Core.BORDER_CONSTANT;
        Scalar cvDilateBordervalue = new Scalar(-1);
        cvDilate(cvDilateSrc, cvDilateKernel, cvDilateAnchor, cvDilateIterations, cvDilateBordertype, cvDilateBordervalue, cvDilateOutput);

        // Step Find_Contours0:
        Mat findContoursInput = cvDilateOutput;
        boolean findContoursExternalOnly = true;
        findContours(findContoursInput, findContoursExternalOnly, findContoursOutput);

        //cvDilateOutput = annotateFinalImage(cvDilateOutput);

//        // Step Filter_Contours0:
//        ArrayList<MatOfPoint> filterContoursContours = findContoursOutput;
//        double filterContoursMinArea = 10000.0;
//        double filterContoursMinPerimeter = 0;
//        double filterContoursMinWidth = 0;
//        double filterContoursMaxWidth = 1000;
//        double filterContoursMinHeight = 0;
//        double filterContoursMaxHeight = 1000;
//        double[] filterContoursSolidity = {0, 100};
//        double filterContoursMaxVertices = 1000000;
//        double filterContoursMinVertices = 0;
//        double filterContoursMinRatio = 0;
//        double filterContoursMaxRatio = 1000;
//        filterContours(filterContoursContours, filterContoursMinArea, filterContoursMinPerimeter, filterContoursMinWidth, filterContoursMaxWidth, filterContoursMinHeight, filterContoursMaxHeight, filterContoursSolidity, filterContoursMaxVertices, filterContoursMinVertices, filterContoursMinRatio, filterContoursMaxRatio, filterContoursOutput);

    }

    /**
     * This method is a generated getter for the output of a CV_resize.
     * @return Mat output from CV_resize.
     */
    public Mat cvResizeOutput() {
        return cvResizeOutput;
    }

    /**
     * This method is a generated getter for the output of a Blur.
     * @return Mat output from Blur.
     */
    public Mat blurOutput() {
        return blurOutput;
    }

    /**
     * This method is a generated getter for the output of a HSV_Threshold.
     * @return Mat output from HSV_Threshold.
     */
    public Mat hsvThresholdOutput() {
        return hsvThresholdOutput;
    }

    /**
     * This method is a generated getter for the output of a CV_erode.
     * @return Mat output from CV_erode.
     */
    public Mat cvErodeOutput() {
        return cvErodeOutput;
    }

    /**
     * This method is a generated getter for the output of a CV_dilate.
     * @return Mat output from CV_dilate.
     */
    public Mat cvDilateOutput() {
        return cvDilateOutput;
    }

    /**
     * This method is a generated getter for the output of a Find_Contours.
     * @return ArrayList<MatOfPoint> output from Find_Contours.
     */
    public ArrayList<MatOfPoint> findContoursOutput() {
        return findContoursOutput;
    }

    /**
     * This method is a generated getter for the output of a Filter_Contours.
     * @return ArrayList<MatOfPoint> output from Filter_Contours.
     */
    public ArrayList<MatOfPoint> filterContoursOutput() {
        return filterContoursOutput;
    }


    /**
     * Resizes an image.
     * @param src The image to resize.
     * @param dSize size to set the image.
     * @param fx scale factor along X axis.
     * @param fy scale factor along Y axis.
     * @param interpolation type of interpolation to use.
     * @param dst output image.
     */
    private void cvResize(Mat src, Size dSize, double fx, double fy, int interpolation,
                          Mat dst) {
        if (dSize==null) {
            dSize = new Size(0,0);
        }
        Imgproc.resize(src, dst, dSize, fx, fy, interpolation);
    }

    /**
     * An indication of which type of filter to use for a blur.
     * Choices are BOX, GAUSSIAN, MEDIAN, and BILATERAL
     */
    enum BlurType{
        BOX("Box Blur"), GAUSSIAN("Gaussian Blur"), MEDIAN("Median Filter"),
        BILATERAL("Bilateral Filter");

        private final String label;

        BlurType(String label) {
            this.label = label;
        }

        public static BlurType get(String type) {
            if (BILATERAL.label.equals(type)) {
                return BILATERAL;
            }
            else if (GAUSSIAN.label.equals(type)) {
                return GAUSSIAN;
            }
            else if (MEDIAN.label.equals(type)) {
                return MEDIAN;
            }
            else {
                return BOX;
            }
        }

        @Override
        public String toString() {
            return this.label;
        }
    }

    /**
     * Softens an image using one of several filters.
     * @param input The image on which to perform the blur.
     * @param type The blurType to perform.
     * @param doubleRadius The radius for the blur.
     * @param output The image in which to store the output.
     */
    private void blur(Mat input, BlurType type, double doubleRadius,
                      Mat output) {
        int radius = (int)(doubleRadius + 0.5);
        int kernelSize;
        switch(type){
            case BOX:
                kernelSize = 2 * radius + 1;
                Imgproc.blur(input, output, new Size(kernelSize, kernelSize));
                break;
            case GAUSSIAN:
                kernelSize = 6 * radius + 1;
                Imgproc.GaussianBlur(input,output, new Size(kernelSize, kernelSize), radius);
                break;
            case MEDIAN:
                kernelSize = 2 * radius + 1;
                Imgproc.medianBlur(input, output, kernelSize);
                break;
            case BILATERAL:
                Imgproc.bilateralFilter(input, output, -1, radius, radius);
                break;
        }
    }

    /**
     * Segment an image based on hue, saturation, and value ranges.
     *
     * @param input The image on which to perform the HSL threshold.
     * @param hue The min and max hue
     * @param sat The min and max saturation
     * @param val The min and max value
     * @param out The image in which to store the output.
     */
    private void hsvThreshold(Mat input, double[] hue, double[] sat, double[] val,
                              Mat out) {
        Imgproc.cvtColor(input, out, Imgproc.COLOR_RGB2HSV);
        Core.inRange(out, new Scalar(hue[0], sat[0], val[0]),
                new Scalar(hue[1], sat[1], val[1]), out);
    }

    /**
     * Expands area of lower value in an image.
     * @param src the Image to erode.
     * @param kernel the kernel for erosion.
     * @param anchor the center of the kernel.
     * @param iterations the number of times to perform the erosion.
     * @param borderType pixel extrapolation method.
     * @param borderValue value to be used for a constant border.
     * @param dst Output Image.
     */
    private void cvErode(Mat src, Mat kernel, Point anchor, double iterations,
                         int borderType, Scalar borderValue, Mat dst) {
        if (kernel == null) {
            kernel = new Mat();
        }
        if (anchor == null) {
            anchor = new Point(-1,-1);
        }
        if (borderValue == null) {
            borderValue = new Scalar(-1);
        }
        Imgproc.erode(src, dst, kernel, anchor, (int)iterations, borderType, borderValue);
    }

    /**
     * Expands area of higher value in an image.
     * @param src the Image to dilate.
     * @param kernel the kernel for dilation.
     * @param anchor the center of the kernel.
     * @param iterations the number of times to perform the dilation.
     * @param borderType pixel extrapolation method.
     * @param borderValue value to be used for a constant border.
     * @param dst Output Image.
     */
    private void cvDilate(Mat src, Mat kernel, Point anchor, double iterations,
                          int borderType, Scalar borderValue, Mat dst) {
        if (kernel == null) {
            kernel = new Mat();
        }
        if (anchor == null) {
            anchor = new Point(-1,-1);
        }
        if (borderValue == null){
            borderValue = new Scalar(-1);
        }
        Imgproc.dilate(src, dst, kernel, anchor, (int)iterations, borderType, borderValue);
    }

    /**
     * Find color contours in the image.
     * @param input The image on which to perform the Distance Transform.
     * @param externalOnly Find only external contours,
     * @param contours list of resulting contours.
     */
    private void findContours(Mat input, boolean externalOnly,
                              List<MatOfPoint> contours) {
        Mat hierarchy = new Mat();
        contours.clear();
        int mode;
        if (externalOnly) {
            mode = Imgproc.RETR_EXTERNAL;
        }
        else {
            mode = Imgproc.RETR_LIST;
        }
        int method = Imgproc.CHAIN_APPROX_SIMPLE;
        Imgproc.findContours(input, contours, hierarchy, mode, method);
    }


//    /**
//     * Filters out contours that do not meet certain criteria.
//     * @param inputContours is the input list of contours
//     * @param output is the the output list of contours
//     * @param minArea is the minimum area of a contour that will be kept
//     * @param minPerimeter is the minimum perimeter of a contour that will be kept
//     * @param minWidth minimum width of a contour
//     * @param maxWidth maximum width
//     * @param minHeight minimum height
//     * @param maxHeight maximimum height
//     * @param solidity the minimum and maximum solidity of a contour
//     * @param minVertexCount minimum vertex Count of the contours
//     * @param maxVertexCount maximum vertex Count
//     * @param minRatio minimum ratio of width to height
//     * @param maxRatio maximum ratio of width to height
//     */
//    private void filterContours(List<MatOfPoint> inputContours, double minArea,
//                                double minPerimeter, double minWidth, double maxWidth, double minHeight, double
//                                        maxHeight, double[] solidity, double maxVertexCount, double minVertexCount, double
//                                        minRatio, double maxRatio, List<MatOfPoint> output) {
//        final MatOfInt hull = new MatOfInt();
//        output.clear();
//        //operation
//        for (int i = 0; i < inputContours.size(); i++) {
//            final MatOfPoint contour = inputContours.get(i);
//            final Rect bb = Imgproc.boundingRect(contour);
//            if (bb.width < minWidth || bb.width > maxWidth) continue;
//            if (bb.height < minHeight || bb.height > maxHeight) continue;
//            final double area = Imgproc.contourArea(contour);
//            if (area < minArea) continue;
//            if (Imgproc.arcLength(new MatOfPoint2f(contour.toArray()), true) < minPerimeter) continue;
//            Imgproc.convexHull(contour, hull);
//            MatOfPoint mopHull = new MatOfPoint();
//            mopHull.create((int) hull.size().height, 1, CvType.CV_32SC2);
//            for (int j = 0; j < hull.size().height; j++) {
//                int index = (int)hull.get(j, 0)[0];
//                double[] point = new double[] { contour.get(index, 0)[0], contour.get(index, 0)[1]};
//                mopHull.put(j, 0, point);
//            }
//            final double solid = 100 * area / Imgproc.contourArea(mopHull);
//            if (solid < solidity[0] || solid > solidity[1]) continue;
//            if (contour.rows() < minVertexCount || contour.rows() > maxVertexCount)	continue;
//            final double ratio = bb.width / (double)bb.height;
//            if (ratio < minRatio || ratio > maxRatio) continue;
//            output.add(contour);
//        }
//    }

    private MatOfPoint findLargestContour(List<MatOfPoint> inputContours) {

        double maxArea = 0;
        MatOfPoint largestContour = new MatOfPoint();

        //operation
        for (int i = 0; i < inputContours.size(); i++) {
            final MatOfPoint contour = inputContours.get(i);
            final double area = Imgproc.contourArea(contour);
            if(area > maxArea){
                largestContour = contour;
            }
        }
        return largestContour;
    }

    private double getContourCentroidX(MatOfPoint contour){
        Moments moments =  Imgproc.moments(contour);

        return (moments.m10 / moments.m00);
    }

    private Rect getContourBoundingRect(MatOfPoint contour){
        return Imgproc.boundingRect(contour);
    }

    private android.graphics.Rect makeGraphicsRect(Rect rect, float scaleBmpPxToCanvasPx) {
        int left = Math.round(rect.x * scaleBmpPxToCanvasPx);
        int top = Math.round(rect.y * scaleBmpPxToCanvasPx);
        int right = left + Math.round(rect.width * scaleBmpPxToCanvasPx);
        int bottom = top + Math.round(rect.height * scaleBmpPxToCanvasPx);

        return new android.graphics.Rect(left, top, right, bottom);
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration){
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        process(frame);

        return findLargestContour(findContoursOutput());
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx,
                            float scaleCanvasDensity, Object userContext){

        Paint selectedPaint = new Paint();
        selectedPaint.setColor(Color.RED);
        selectedPaint.setStyle(Paint.Style.STROKE);
        selectedPaint.setStrokeWidth(scaleCanvasDensity * 4);

        Paint nonSelectedPaint = new Paint(selectedPaint);
        nonSelectedPaint.setColor(Color.GREEN);

        selectedContour = (MatOfPoint) userContext;
        Rect boundingBox = getContourBoundingRect(selectedContour);

        android.graphics.Rect drawContourBound = makeGraphicsRect(boundingBox, scaleBmpPxToCanvasPx);
        canvas.drawRect(drawContourBound, selectedPaint);
        canvas.drawText("x = " + getSelectedX(), boundingBox.x, boundingBox.y+boundingBox.height, selectedPaint);
    }

    public double getSelectedX() {
        return getContourCentroidX(selectedContour);
    }

    public Selected getLocation() {
        double x = getSelectedX();
        if (x < LEFT_LIMIT) {
            selection = Selected.LEFT;
        } else if (x > RIGHT_LIMIT) {
            selection = Selected.RIGHT;
        } else {
            selection = Selected.MIDDLE;
        }

        return selection;
    }

    public void setAlliance(Alliance alliance){
        this.alliance = alliance;
    }

    public Selected getSelection() {
        return selection;
    }

    public Mat annotateFinalImage(Mat matSource){
        Rect rect = getContourBoundingRect(selectedContour);
        Imgproc.rectangle(matSource, rect, new Scalar(0, 255, 0));
        Imgproc.putText(matSource,"x = " + getSelectedX(), new Point(rect.x, rect.y), 1, 1, new Scalar(0, 255, 0));
        return matSource;
    }


}
