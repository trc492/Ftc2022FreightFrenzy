/*
 * Copyright (c) 2022 Titan Robotics Club (http://www.titanrobotics.com)
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

import org.opencv.core.KeyPoint;
import org.opencv.core.Mat;
import org.opencv.core.MatOfKeyPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;

import TrcCommonLib.trclib.TrcDbgTrace;
import TrcCommonLib.trclib.TrcHomographyMapper;
import TrcCommonLib.trclib.TrcOpenCVDetector;
import TrcFtcLib.ftclib.FtcEocvDetector;

/**
 * This class implements EOCV Vision that provides the capability to detect color blobs and return their location
 * info.
 */
public class EocvVision extends FtcEocvDetector
{
    private final GripPipeline gripPipeline;
    private TrcOpenCVDetector.DetectedObject[] detectedObjects = null;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param imageWidth specifies the camera image width.
     * @param imageHeight specifies the camera image height.
     * @param cameraRect specifies the homography camera pixel rectangle, can be null if not provided.
     * @param worldRect specifies the homography world coordinate rectangle, can be null if not provided.
     * @param openCvCam specifies the OpenCV camera object.
     * @param cameraRotation specifies the camera orientation.
     */
    public EocvVision(
        String instanceName, int imageWidth, int imageHeight,
        TrcHomographyMapper.Rectangle cameraRect, TrcHomographyMapper.Rectangle worldRect,
        OpenCvCamera openCvCam, OpenCvCameraRotation cameraRotation)
    {
        super(instanceName, imageWidth, imageHeight, cameraRect, worldRect, openCvCam, cameraRotation, null);

        gripPipeline = new GripPipeline();
        openCvCam.setPipeline(this);
    }   //EocvVision

    //
    // Implements FtcEocvDetector abstract methods.
    //

    /**
     * This method is called by EasyOpenCV.OpenCVPipeline to process an image frame. It calls the grip pipeline to
     * process the image and converts the detected object into an array of TrcOpenCvDetector.DetectedObject. It also
     * annotates the original image with rectangles around the detected objects.
     *
     * @param input specifies the image frame.
     * @return annotated image frame.
     */
    @Override
    public Mat processFrame(Mat input)
    {
        final String funcName = "processFrame";
        TrcOpenCVDetector.DetectedObject[] targets = null;

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.CALLBK);
        }

        gripPipeline.process(input);
        //
        // Process the image to detect the targets we are looking for and put them into targetRects.
        //
        MatOfKeyPoint detectedTargets = gripPipeline.findBlobsOutput();

        if (detectedTargets != null)
        {
            KeyPoint[] targetPoints = detectedTargets.toArray();
            targets = new TrcOpenCVDetector.DetectedObject[targetPoints.length];
            for (int i = 0; i < targets.length; i++)
            {
                double radius = targetPoints[i].size/2;
                targets[i] = new TrcOpenCVDetector.DetectedObject(
                    new Rect((int)(targetPoints[i].pt.x - radius), (int)(targetPoints[i].pt.y - radius),
                             (int)targetPoints[i].size, (int)targetPoints[i].size),
                    targetPoints[i].angle, targetPoints[i].response, targetPoints[i].octave, targetPoints[i].class_id);
            }
            detectedTargets.release();
            TrcOpenCVDetector.drawRectangles(input, targets, new Scalar(0, 255, 0), 0);
            synchronized (this)
            {
                detectedObjects = targets;
            }
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.CALLBK, "=%s", targets != null);
        }

        return input;
    }   //processFrame

    /**
     * This method returns the currently detect objects in a thread safe manner.
     *
     * @return array of detected objects.
     */
    public synchronized TrcOpenCVDetector.DetectedObject[] getDetectedObjects()
    {
        TrcOpenCVDetector.DetectedObject[] targets = detectedObjects;
        detectedObjects = null;
        return targets;
    }   //getDetectedObjects

}   //class GripVision
