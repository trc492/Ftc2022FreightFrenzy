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

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

import TrcCommonLib.trclib.TrcDbgTrace;
import TrcCommonLib.trclib.TrcOpenCV;
import TrcCommonLib.trclib.TrcRevBlinkin;
import TrcCommonLib.trclib.TrcVisionTargetInfo;
import TrcFtcLib.ftclib.FtcOpMode;
import TrcFtcLib.ftclib.FtcTensorFlow;
import TrcFtcLib.ftclib.FtcVuforia;

/**
 * This class implements Vuforia/TensorFlow Vision for the game season. It creates and initializes all the vision
 * target info as well as providing info for the robot, camera and the field. It also provides methods to get the
 * location of the robot and detected targets.
 */
public class Vision
{
    public static final String OPENCV_NATIVE_LIBRARY_NAME = "opencv_java3";
    public static final String BLUE_STORAGE_NAME = "Blue Storage";
    public static final String BLUE_ALLIANCE_WALL_NAME = "Blue Alliance Wall";
    public static final String RED_STORAGE_NAME = "Red Storage";
    public static final String RED_ALLIANCE_WALL_NAME = "Red Alliance Wall";
    public static final String LABEL_BALL = "Ball";
    public static final String LABEL_CUBE = "Cube";
    public static final String LABEL_DUCK = "Duck";
    public static final String LABEL_MARKER = "Marker";
    public static final String DUCK_POS_1 = "DuckPos1";
    public static final String DUCK_POS_2 = "DuckPos2";
    public static final String DUCK_POS_3 = "DuckPos3";
    public static final String GOT_TARGET = "GotTarget";
    public static final String SAW_TARGET = "SawTarget";

    private final TrcRevBlinkin.Pattern[] ledPatternPriorities = {
        new TrcRevBlinkin.Pattern(DUCK_POS_1, TrcRevBlinkin.RevLedPattern.SolidRed),
        new TrcRevBlinkin.Pattern(DUCK_POS_2, TrcRevBlinkin.RevLedPattern.SolidGreen),
        new TrcRevBlinkin.Pattern(DUCK_POS_3, TrcRevBlinkin.RevLedPattern.SolidBlue),
        new TrcRevBlinkin.Pattern(SAW_TARGET, TrcRevBlinkin.RevLedPattern.SolidViolet),
        new TrcRevBlinkin.Pattern(GOT_TARGET, TrcRevBlinkin.RevLedPattern.SolidAqua),
        new TrcRevBlinkin.Pattern(RED_STORAGE_NAME, TrcRevBlinkin.RevLedPattern.FixedStrobeRed),
        new TrcRevBlinkin.Pattern(BLUE_STORAGE_NAME, TrcRevBlinkin.RevLedPattern.FixedStrobeBlue),
        new TrcRevBlinkin.Pattern(RED_ALLIANCE_WALL_NAME, TrcRevBlinkin.RevLedPattern.FixedLightChaseRed),
        new TrcRevBlinkin.Pattern(BLUE_ALLIANCE_WALL_NAME, TrcRevBlinkin.RevLedPattern.FixedLightChaseBlue)};

    private final Robot robot;
    private final TrcDbgTrace tracer;
    public VuforiaVision vuforiaVision;
    public TensorFlowVision tensorFlowVision;
    public GripVision gripVision;

    private int lastDuckPosition = 0;

    /**
     * Constructor: Create an instance of the object. Vision is required by both Vuforia and TensorFlow and must be
     * instantiated if either is used. However, to use either Vuforia or TensorFlow, one must explicitly initialize
     * them by calling the initVuforia or initTensorFlow methods respectively.
     *
     * @param robot specifies the robot object.
     * @param useVuforia specifies true to use Vuforia Vision, false otherwise.
     * @param useTensorFlow specifies true to use TensorFlow Vision, false otherwise.
     * @param useGripPipeline specifies true to use GRIP Vision, false otherwise.
     */
    public Vision(Robot robot, boolean useVuforia, boolean useTensorFlow, boolean useGripPipeline)
    {
        this.robot = robot;
        this.tracer = TrcDbgTrace.getGlobalTracer();
        final String VUFORIA_LICENSE_KEY =
            "ARbBwjf/////AAABmZijKPKUWEY+uNSzCuTOUFgm7Gr5irDO55gtIOjsOXmhLzLEILJp45qdPrwMfoBV2Yh7F+Wh8iEjnSA" +
            "NnnRKiJNHy1T9Pr2uufETE40YJth10Twv0sTNSEqxDPhg2t4PJXwRImMaEsTE53fmcm08jT9qMso2+1h9eNk2b4x6DVKgBt" +
            "Tv5wocDs949Gkh6lRt5rAxATYYO9esmyKyfyzfFLMMpfq7/uvQQrSibNBqa13hJRmmHoM2v0Gfk8TCTTfP044/XsOm54u8k" +
            "dv0HfeMBC91uQ/NvWHVV5XCh8pZAzmL5sry1YwG8FSRNVlSAZ1zN/m6jAe98q6IxpwQxP0da/TpJoqDI7x4RGjOs1Areunf";
        FtcOpMode opMode = FtcOpMode.getInstance();
        int cameraViewId = !RobotParams.Preferences.showVuforiaView ? -1 :
            opMode.hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", opMode.hardwareMap.appContext.getPackageName());
        //
        // If no camera view ID, do not activate camera monitor view to save power.
        //
        VuforiaLocalizer.Parameters vuforiaParams =
            cameraViewId == -1? new VuforiaLocalizer.Parameters(): new VuforiaLocalizer.Parameters(cameraViewId);

        vuforiaParams.vuforiaLicenseKey = VUFORIA_LICENSE_KEY;
        vuforiaParams.cameraName = opMode.hardwareMap.get(WebcamName.class, RobotParams.HWNAME_WEBCAM);
        vuforiaParams.useExtendedTracking = false;
        vuforiaParams.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;
        FtcVuforia vuforia = new FtcVuforia(vuforiaParams);

        vuforiaVision = useVuforia? new VuforiaVision(vuforia, robot.blinkin): null;
        tensorFlowVision = useTensorFlow? new TensorFlowVision(vuforia, null): null;

        if (useGripPipeline)
        {
            vuforia.configVideoSource(RobotParams.IMAGE_WIDTH, RobotParams.IMAGE_HEIGHT, 1);
            System.loadLibrary(OPENCV_NATIVE_LIBRARY_NAME);
            gripVision = new GripVision("GripVision", vuforia);
        }
    }   //Vision

    /**
     * This method sets up the Blinkin with a priority pattern list and a pattern name map.
     */
    public void setupBlinkin()
    {
        robot.blinkin.setPatternPriorities(ledPatternPriorities);
    }   //setupBlinkin

    /**
     * This method shuts down TensorFlow.
     */
    public void tensorFlowShutdown()
    {
        tensorFlowVision.shutdown();
        tensorFlowVision = null;
    }   //tensorFlowShutdown

    /**
     * This method determines the duck's barcode position 1, 2, or 3 (0 if no valid duck found).
     *
     * @param targetInfo specifies the detected duck's target info.
     * @return duck's barcode position.
     */
    public int determineDuckPosition(TrcVisionTargetInfo<?> targetInfo)
    {
        int pos = 0;

        if (targetInfo != null)
        {
            double oneSixthImageWidth = targetInfo.imageWidth/6.0;

            if (targetInfo.distanceFromImageCenter.x <= -oneSixthImageWidth)
            {
                pos = 1;
            }
            else if (targetInfo.distanceFromImageCenter.x >= oneSixthImageWidth)
            {
                pos = 3;
            }
            else
            {
                pos = 2;
            }

            if (robot.blinkin != null)
            {
                // Turn off previous detection indication.
                robot.blinkin.setPatternState(Vision.DUCK_POS_1, false);
                robot.blinkin.setPatternState(Vision.DUCK_POS_2, false);
                robot.blinkin.setPatternState(Vision.DUCK_POS_3, false);

                switch (pos)
                {
                    case 1:
                        robot.blinkin.setPatternState(Vision.DUCK_POS_1, true);
                        break;

                    case 2:
                        robot.blinkin.setPatternState(Vision.DUCK_POS_2, true);
                        break;

                    case 3:
                        robot.blinkin.setPatternState(Vision.DUCK_POS_3, true);
                        break;
                }
            }
        }

        return pos;
    }   //determineDuckPosition

    /**
     * This method returns an array of detected duck info.
     *
     * @return array of detected duck info.
     */
    public TrcVisionTargetInfo<?>[] getDetectedDucksInfo()
    {
        TrcVisionTargetInfo<?>[] targetInfo = null;

        if (tensorFlowVision != null)
        {
            targetInfo = tensorFlowVision.getDetectedDucksInfo();
        }
        else if (gripVision != null)
        {
            targetInfo = gripVision.getDetectedTargetsInfo(null, this::compareObjectSize);
        }

        return targetInfo;
    }   //getDetectedDucksInfo

    /**
     * This method calls vision to detect all the ducks and returns their barcode positions in an array.
     *
     * @return duck barcode position array 1, 2, or 3, null if none found.
     */
    public int[] getDetectedDuckPositions()
    {
        final String funcName = "getDetectedDuckPositions";
        int[] duckPositions = null;
        TrcVisionTargetInfo<?>[] targetInfo = getDetectedDucksInfo();

        if (targetInfo != null && targetInfo.length > 0)
        {
            duckPositions = new int[targetInfo.length];
            //
            // Target array is sorted with highest confidence first. Therefore, we process the array backward so
            // that the highest confidence target will be processed last. Therefore, the LED state will show the
            // highest confidence target position.
            //
            for (int i = targetInfo.length - 1; i >= 0; i--)
            {
                duckPositions[i] = determineDuckPosition(targetInfo[i]);
                if (tracer != null)
                {
                    tracer.traceInfo(funcName, "[%d] targetInfo=%s, duckPos=%d", i, targetInfo[i], duckPositions[i]);
                }
            }

            lastDuckPosition = duckPositions[0];
        }

        return duckPositions;
    }   //getDetectedDuckPositions

    /**
     * This method returns the info of the detected duck.
     *
     * @return detected duck info.
     */
    public TrcVisionTargetInfo<?> getDetectedDuckInfo()
    {
        TrcVisionTargetInfo<?>[] targetInfo = getDetectedDucksInfo();
        return targetInfo != null? targetInfo[0]: null;
    }   //getDetectedDuckInfo

    /**
     * This method returns target info of the closest detected duck to image center.
     *
     * @return closest duck target info.
     */
    public TrcVisionTargetInfo<?> getClosestDuckInfo()
    {
        TrcVisionTargetInfo<?>[] targets = null;

        if (tensorFlowVision != null)
        {
            targets = tensorFlowVision.getDetectedTargetsInfo(
                LABEL_DUCK, tensorFlowVision::validateDuck, this::compareDistanceToCenter, false);
        }
        else if (gripVision != null)
        {
            targets = gripVision.getDetectedTargetsInfo(null, this::compareObjectSize);
        }

        return targets != null? targets[0]: null;
    }   //getClosestDuckInfo

    /**
     * This method returns target info of the closest freight detected.
     *
     * @return closest freight target info.
     */
    public TrcVisionTargetInfo<?> getClosestFreightInfo()
    {
        TrcVisionTargetInfo<?>[] targets = null;

        if (tensorFlowVision != null)
        {
            targets = tensorFlowVision.getDetectedTargetsInfo(
                null, tensorFlowVision::validateFreight, this::compareCameraAngle, false);
        }
        else if (gripVision != null)
        {
            targets = gripVision.getDetectedTargetsInfo(null, this::compareObjectSize);
        }

        return targets != null? targets[0]: null;
    }   //getClosestFreightInfo

    /**
     * This method returns the last detected duck barcode position.
     *
     * @return last detected duck barcode position, 0 if no duck detected.
     */
    public int getLastDuckPosition()
    {
        return lastDuckPosition;
    }   //getLastDuckPosition

    /**
     * This method is called by the Arrays.sort to sort the target object by increasing distance to image center.
     *
     * @param a specifies the first target
     * @param b specifies the second target.
     * @return negative value if a has smaller distance to image center than b, 0 if a and b have equal distance to
     * image center, positive value if a has larger distance to image center than b.
     */
    private int compareDistanceToCenter(TrcVisionTargetInfo<?> a, TrcVisionTargetInfo<?> b)
    {
        return (int)((Math.abs(a.distanceFromImageCenter.x) - Math.abs(b.distanceFromImageCenter.x))*1000);
    }   //compareDistanceToCenter

    /**
     * This method is called by the Arrays.sort to sort the target object by increasing camera angle.
     *
     * @param a specifies the first target
     * @param b specifies the second target.
     * @return negative value if a has smaller camera angle than b, 0 if a and b have equal camera angle, positive
     *         value if a has larger camera angle than b.
     */
    private int compareCameraAngle(
        TrcVisionTargetInfo<FtcTensorFlow.DetectedObject> a, TrcVisionTargetInfo<FtcTensorFlow.DetectedObject> b)
    {
        return (int)((Math.abs(a.detectedObj.angle) - Math.abs(b.detectedObj.angle))*1000);
    }   //compareCameraAngle

    /**
     * This method is called by the Arrays.sort to sort the target object by decreasing object size.
     *
     * @param a specifies the first target
     * @param b specifies the second target.
     * @return negative value if a has smaller area than b, 0 if a and b have equal area, positive value if a has
     *         larger area than b.
     */
    private int compareObjectSize(
        TrcVisionTargetInfo<TrcOpenCV.DetectedObject> a, TrcVisionTargetInfo<TrcOpenCV.DetectedObject> b)
    {
        return a.detectedObj.rect.width * a.detectedObj.rect.height -
               b.detectedObj.rect.width * b.detectedObj.rect.height;
    }   //compareObjectSize

}   //class Vision
