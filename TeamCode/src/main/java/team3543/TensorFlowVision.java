/*
 * Copyright (c) 2021 Titan Robotics Club (http://www.titanrobotics.com)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package team3543;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.opencv.core.Core;
import org.opencv.core.Point;

import java.util.Comparator;

import TrcCommonLib.trclib.TrcDbgTrace;
import TrcCommonLib.trclib.TrcVisionTargetInfo;
import TrcFtcLib.ftclib.FtcOpMode;
import TrcFtcLib.ftclib.FtcTensorFlow;
import TrcFtcLib.ftclib.FtcVuforia;

/**
 * This class implements TensorFlow Vision that provides the capability to detect learned objects and return their
 * location info.
 */
public class TensorFlowVision
{
    private static final String TFOD_MODEL_ASSET = "FreightFrenzy_BCDM.tflite";
    private static final float TFOD_MIN_CONFIDENCE = 0.2f;
    private static final double TARGET_WIDTH_LOWER_TOLERANCE = 2.0;
    private static final double TARGET_WIDTH_UPPER_TOLERANCE = 3.0;
//    private static final double ASPECT_RATIO_TOLERANCE_LOWER = 0.75;
//    private static final double ASPECT_RATIO_TOLERANCE_UPPER = 1.33;
//    // Target size is area of target rect.
//    private static final double TARGET_SIZE_TOLERANCE_LOWER = 4000.0;
//    private static final double TARGET_SIZE_TOLERANCE_UPPER = 25000.0;

    private final TrcDbgTrace tracer;
    private FtcTensorFlow tensorFlow;

    /**
     * Constructor: Create an instance of the object.
     */
    public TensorFlowVision(FtcVuforia vuforia, TrcDbgTrace tracer)
    {
        System.loadLibrary(Vision.OPENCV_NATIVE_LIBRARY_NAME);
        FtcOpMode opMode = FtcOpMode.getInstance();
        int tfodMonitorViewId = !RobotParams.Preferences.showTensorFlowView ? -1 :
            opMode.hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", opMode.hardwareMap.appContext.getPackageName());
        //
        // If no TFOD monitor view ID, do not activate camera monitor view to save power.
        //
        TFObjectDetector.Parameters tfodParams =
            tfodMonitorViewId == -1?
                new TFObjectDetector.Parameters() : new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParams.minResultConfidence = TFOD_MIN_CONFIDENCE;
        tfodParams.isModelTensorFlow2 = true;
        tfodParams.inputSize = 320;

        this.tracer = tracer;
        tensorFlow = new FtcTensorFlow(
            vuforia, tfodParams, TFOD_MODEL_ASSET,
            new String[] {Vision.LABEL_BALL, Vision.LABEL_CUBE, Vision.LABEL_DUCK, Vision.LABEL_MARKER},
            RobotParams.cameraRect, RobotParams.worldRect, tracer);
    }   //TensorFlowVision

    /**
     * This method enables/disables TensorFlow.
     *
     * @param enabled specifies true to enable, false to disable.
     */
    public void setEnabled(boolean enabled)
    {
        tensorFlow.setEnabled(enabled);
        if (enabled)
        {
            tensorFlow.setZoom(1.0, 16.0/9.0);
        }
    }   //setEnabled

    /**
     * This method shuts down TensorFlow.
     */
    public void shutdown()
    {
        setEnabled(false);
        if (tensorFlow != null)
        {
            tensorFlow.shutdown();
            tensorFlow = null;
        }
    }   //shutdown

    public void setZoomFactor(double factor)
    {
        tensorFlow.setZoom(factor, 16.0/9.0);
    }   //setZoomFactor

    /**
     * This method returns an array of detected targets from TensorFlow vision.
     *
     * @param label specifies the label of the targets to detect for, can be null for detecting any target.
     * @param filter specifies the filter to call to filter out false positive targets.
     * @param comparator specifies the comparator to sort the array if provided, can be null if not provided.
     * @param traceTargets specifies true to display info for all detected targets.
     * @return array of detected target info.
     */
    public TrcVisionTargetInfo<FtcTensorFlow.DetectedObject>[] getDetectedTargetsInfo(
        String label, FtcTensorFlow.FilterTarget filter,
        Comparator<? super TrcVisionTargetInfo<FtcTensorFlow.DetectedObject>> comparator, boolean traceTargets)
    {
        final String funcName = "getDetectedTargetsInfo";
        TrcVisionTargetInfo<FtcTensorFlow.DetectedObject>[] targets =
            tensorFlow.getDetectedTargetsInfo(label, filter, comparator);

        if (targets != null && traceTargets)
        {
            for (int i = 0; i < targets.length; i++)
            {
                tracer.traceInfo(funcName, "[%d] Target=%s", i, targets[i]);
            }
        }

        return targets;
    }   //getDetectedTargetsInfo

    /**
     * This method returns an array of detected ducks from TensorFlow vision.
     *
     * @return array of detected target info.
     */
    public TrcVisionTargetInfo<FtcTensorFlow.DetectedObject>[] getDetectedDucksInfo()
    {
        return getDetectedTargetsInfo(Vision.LABEL_DUCK, null, this::compareConfidence, false);
    }   //getDetectedTargetsInfo

    /**
     * This method maps a camera screen point to a real-world point.
     *
     * @param point specifies the camera screen point.
     * @return real-world point.
     */
    public Point mapPoint(Point point)
    {
        return tensorFlow != null? tensorFlow.mapPoint(point): null;
    }   //mapPoint

    /**
     * This method maps a camera screen point to a real-world point.
     *
     * @param x specifies the camera screen point x.
     * @param y specifies the camera screen point y.
     * @return real-world point.
     */
    public Point mapPoint(double x, double y)
    {
        return mapPoint(new Point(x, y));
    }   //mapPoint

    /**
     * This method is called by the Arrays.sort to sort the target object by decreasing confidence.
     *
     * @param a specifies the first target
     * @param b specifies the second target.
     * @return negative value if a has higher confidence than b, 0 if a and b have equal confidence, positive value
     *         if a has lower confidence than b.
     */
    private int compareConfidence(
        TrcVisionTargetInfo<FtcTensorFlow.DetectedObject> a, TrcVisionTargetInfo<FtcTensorFlow.DetectedObject> b)
    {
        return (int)((b.detectedObj.confidence - a.detectedObj.confidence)*100);
    }   //compareConfidence

    /**
     * This method is called to validate the detected target as a duck.
     * To valid a valid duck, it must have:
     *  - correct aspect ratio
     *  - correct size
     *  - at expected location(s).
     *
     * @param target specifies the target to be validated.
     * @return true if target is valid, false if false positive.
     */
    public boolean validateDuck(Recognition target)
    {
        TrcVisionTargetInfo<FtcTensorFlow.DetectedObject> targetInfo = tensorFlow.getTargetInfo(target);
//            double aspectRatio = (double)targetInfo.rect.width/(double)targetInfo.rect.height;
//            double area = targetInfo.rect.width*targetInfo.rect.height;
//            double distanceYTolerance = targetInfo.imageHeight/6.0;
//            boolean isValid = targetInfo.label.equals(LABEL_DUCK) &&
//                              aspectRatio <= ASPECT_RATIO_TOLERANCE_UPPER &&
//                              aspectRatio >= ASPECT_RATIO_TOLERANCE_LOWER &&
//                              targetInfo.rect.x > 20 && targetInfo.rect.x < targetInfo.imageWidth - 20;
        boolean isValid = targetInfo.detectedObj.label.equals(Vision.LABEL_DUCK) &&
                          targetInfo.targetWidth >= TARGET_WIDTH_LOWER_TOLERANCE &&
                          targetInfo.targetWidth <= TARGET_WIDTH_UPPER_TOLERANCE;
        tracer.traceInfo("validateDuck", "<<<<< valid=%s, duckInfo=%s", isValid, targetInfo);

        return isValid;
//            return targetInfo.label.equals(LABEL_DUCK) &&
//                   aspectRatio <= ASPECT_RATIO_TOLERANCE_UPPER &&
//                   aspectRatio >= ASPECT_RATIO_TOLERANCE_LOWER &&
//                   targetInfo.rect.x > 20 && targetInfo.rect.x < targetInfo.imageWidth - 20;
//                   area <= TARGET_SIZE_TOLERANCE_UPPER &&
//                   area >= TARGET_SIZE_TOLERANCE_LOWER &&
//                   Math.abs(targetInfo.distanceFromImageCenter.y) <= distanceYTolerance;
    }   //validateDuck

    /**
     * This method is called to validate the detected target as either a cube or a ball.
     *
     * @param target specifies the target to be validated.
     * @return true if target is valid, false if false positive.
     */
    public boolean validateFreight(Recognition target)
    {
        TrcVisionTargetInfo<FtcTensorFlow.DetectedObject> targetInfo = tensorFlow.getTargetInfo(target);

        return targetInfo.detectedObj.label.equals(Vision.LABEL_CUBE) ||
               targetInfo.detectedObj.label.equals(Vision.LABEL_BALL);
    }   //validateFreight

}   //class TensorFlowVision
