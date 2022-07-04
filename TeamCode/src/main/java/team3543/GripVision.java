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

import org.opencv.core.KeyPoint;
import org.opencv.core.Mat;
import org.opencv.core.MatOfKeyPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;

import TrcCommonLib.trclib.TrcOpenCV;
import TrcFtcLib.ftclib.FtcVuforia;

/**
 * This class implements GRIP Vision that provides the capability to detect color blobs and return their
 * location info.
 */
public class GripVision extends TrcOpenCV
{
    private final FtcVuforia vuforia;
    private final GripPipeline gripPipeline;
    private boolean videoOutEnabled = false;

    public GripVision(String instanceName, FtcVuforia vuforia)
    {
        super(instanceName, vuforia, 2, RobotParams.IMAGE_WIDTH, RobotParams.IMAGE_HEIGHT,
              RobotParams.cameraRect, RobotParams.worldRect, null);
        this.vuforia = vuforia;
        gripPipeline = new GripPipeline();
    }   //GripVision

    /**
     * This method enables/disables the video out stream.
     *
     * @param enabled specifies true to enable video out stream, false to disable.
     */
    public void setVideoOutEnabled(boolean enabled)
    {
        videoOutEnabled = enabled;
    }   //setVideoOutEnabled

    /**
     * This method is called to grab an image frame from the video input.
     *
     * @param image specifies the frame buffer to hold the captured image.
     * @return true if frame is successfully captured, false otherwise.
     */
    @Override
    public boolean grabFrame(Mat image)
    {
        return vuforia.getFrame(image);
    }   //grabFrame

    /**
     * This method is called to process an image frame to detect objects in the acquired frame.
     *
     * @param image specifies the image to be processed.
     * @return detected objects, null if none detected.
     */
    @Override
    public DetectedObject[] processFrame(Mat image)
    {
        DetectedObject[] targets = null;
        MatOfKeyPoint detectedTargets;
        //
        // Process the image to detect the targets we are looking for and put them into targetRects.
        //
        gripPipeline.process(image);
        detectedTargets = gripPipeline.findBlobsOutput();
        if (detectedTargets != null)
        {
            KeyPoint[] targetPoints = detectedTargets.toArray();
            targets = new DetectedObject[targetPoints.length];
            for (int i = 0; i < targets.length; i++)
            {
                double radius = targetPoints[i].size/2;
                targets[i] = new DetectedObject(
                    new Rect((int)(targetPoints[i].pt.x - radius), (int)(targetPoints[i].pt.y - radius),
                             (int)targetPoints[i].size, (int)targetPoints[i].size));
            }

            detectedTargets.release();
        }

        if (videoOutEnabled)
        {
            drawRectangles(image, targets, new Scalar(0, 255, 0), 0);
        }

        return targets;
    }   //processFrame

}   //class GripVision
