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

import TrcCommonLib.trclib.TrcOpenCVDetector;
import TrcFtcLib.ftclib.FtcVuforia;

/**
 * This class implements GRIP Vision that provides the capability to detect color blobs and return their
 * location info.
 */
public class GripVision extends TrcOpenCVDetector
{
    private static final int NUM_IMAGE_BUFFERS = 2;
    private final FtcVuforia vuforia;
    private final GripPipeline gripPipeline;

    public GripVision(String instanceName, FtcVuforia vuforia)
    {
        super(instanceName, NUM_IMAGE_BUFFERS, RobotParams.IMAGE_WIDTH, RobotParams.IMAGE_HEIGHT,
              RobotParams.cameraRect, RobotParams.worldRect, null);
        this.vuforia = vuforia;
        gripPipeline = new GripPipeline();
    }   //GripVision

    //
    // Implements TrcVisionTask.VisionProcessor interface.
    //

    /**
     * This method takes a snapshot of the video frame.
     *
     * @param frame specifies the frame buffer to hold the video snapshot.
     * @return true if successful, false otherwise.
     */
    @Override
    public boolean getFrame(Mat frame)
    {
        return vuforia.getFrame(frame);
    }   //getFrame

    /**
     * This method displays a frame buffer to the display surface.
     *
     * @param frame specifies the video frame to be displayed.
     */
    @Override
    public void putFrame(Mat frame)
    {
        vuforia.putFrame(frame);
    }   //putFrame

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
                             (int)targetPoints[i].size, (int)targetPoints[i].size),
                    targetPoints[i].angle, targetPoints[i].response, targetPoints[i].octave, targetPoints[i].class_id);
            }

            detectedTargets.release();
        }

        return targets;
    }   //processFrame

    /**
     * This method is called to overlay rectangles of the detected objects on an image.
     *
     * @param image specifies the frame to be rendered to the video output.
     * @param detectedObjects specifies the detected objects.`
     */
    @Override
    public void annotateFrame(Mat image, DetectedObject[] detectedObjects)
    {
        drawRectangles(image, detectedObjects, new Scalar(0, 255, 0), 0);
        vuforia.putFrame(image);
    }   //annotateFrame

}   //class GripVision
