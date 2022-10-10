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
import TrcCommonLib.trclib.TrcOpenCvColorBlobPipeline;
import TrcCommonLib.trclib.TrcOpenCvDetector;
import TrcCommonLib.trclib.TrcUtil;
import TrcFtcLib.ftclib.FtcEocvColorBlobPipeline;
import TrcFtcLib.ftclib.FtcEocvDetector;

/**
 * This class implements EOCV Vision that provides the capability to detect color blobs and return their location
 * info.
 */
public class EocvVision extends FtcEocvDetector
{
    private static final double[] colorThresholdsTSE = {15.0, 80.0, 128.0, 255.0, 220.0, 255.0};
    private final FtcEocvColorBlobPipeline tsePipeline;

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
     * @param showEocvView specifies true to show the annotated image on robot controller screen, false to hide the
     *        image.
     * @param tracer specifies the tracer for trace info, null if none provided.
     */
    public EocvVision(
        String instanceName, int imageWidth, int imageHeight,
        TrcHomographyMapper.Rectangle cameraRect, TrcHomographyMapper.Rectangle worldRect,
        OpenCvCamera openCvCam, OpenCvCameraRotation cameraRotation, boolean showEocvView, TrcDbgTrace tracer)
    {
        super(instanceName, openCvCam, imageWidth, imageHeight, cameraRotation, showEocvView, cameraRect, worldRect,
              tracer);

        TrcOpenCvColorBlobPipeline.FilterContourParams filterContourParams =
            new TrcOpenCvColorBlobPipeline.FilterContourParams()
                .setMinArea(100.0)
                .setMinPerimeter(100.0)
                .setWidthRange(10.0, 1000.0)
                .setHeightRange(100.0, 1000.0)
                .setSolidityRange(0.0, 100.0)
                .setVerticesRange(0.0, 1000.0)
                .setAspectRatioRange(0.0, 1000.0);
        tsePipeline = new FtcEocvColorBlobPipeline(
            "tsePipeline", false, colorThresholdsTSE, filterContourParams, tracer);
        setPipeline(tsePipeline);
    }   //EocvVision

}   //class EocvVision
