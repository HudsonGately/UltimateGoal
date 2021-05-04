package org.firstinspires.ftc.teamcode.pipelines;

import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.*;
import java.util.stream.Collectors;

public class UGBasicHighGoalPipeline extends OpenCvPipeline {
    // Thresholds for filtering red and blue contours such as High Goal
    public static double MIN_COLOR_THRESHOLD = 155;
    public static double MAX_COLOR_THRESHOLD = 256;

    // Color to target/process
    public enum Mode {
        RED_ONLY, BLUE_ONLY, BOTH
    }

    protected double centerX;
    protected double centerY;

    // YCbCr Channel numbers (constants)
    // Wikipedia article: https://en.wikipedia.org/wiki/YCbCr
    private final int YCBCR_Y_CHANNEL = 0; // grayscale
    private final int YCBCR_CB_CHANNEL = 1; // Blue-chroma difference
    private final int YCBCR_CR_CHANNEL = 2; // Red-chroma difference

    private Mode currentMode;

    // YCrCb color space to extract red, blue, and grayscale easier
    private Mat adjustedColorSpace;
    // Channel and Threshold mat (for tracking)
    private Mat currentChannel, currentThreshold;

    // List to read contours into
    private List<MatOfPoint> currentContours, interiorContours;
    private MatOfPoint biggestContour;
    private Rect blueGoal, redGoal;

    public UGBasicHighGoalPipeline() {
        this(Mode.BOTH);
    }

    public UGBasicHighGoalPipeline(Mode mode) {
        this.currentMode = mode;

        adjustedColorSpace = new Mat();

        currentChannel = new Mat();
        currentThreshold = new Mat();

        currentContours = new ArrayList<>(3);
        interiorContours = new ArrayList<>(3);

        biggestContour = new MatOfPoint();

        blueGoal = new Rect();
        redGoal = new Rect();
    }

    @Override
    public void init(Mat mat) {
        super.init(mat);
        centerX = ((double) mat.width() / 2) - 0.5;
        centerY = ((double) mat.height() / 2) - 0.5;
    }

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, adjustedColorSpace, Imgproc.COLOR_RGB2YCrCb);

        if(currentMode == Mode.RED_ONLY || currentMode == Mode.BOTH)
        {
            // Extract necessary Channels to apropritae Mats
            Core.extractChannel(adjustedColorSpace, currentChannel, YCBCR_CB_CHANNEL);
            redGoal = findTarget(currentChannel);
            if (redGoal != null) {
                Imgproc.rectangle(input, redGoal, new Scalar(255, 0, 0), 3);
            }
        }

        if(currentMode == Mode.BLUE_ONLY || currentMode == Mode.BOTH)
        {
            // Extract necessary Channels to apropritae Mats
            Core.extractChannel(adjustedColorSpace, currentChannel, YCBCR_CR_CHANNEL);
            blueGoal = findTarget(currentChannel);
            if (blueGoal != null) {
                Imgproc.rectangle(input, blueGoal, new Scalar(0, 0, 255), 3);
            }
        }
        // Convert RGB input to YCrCB, this is for better Red and blue identification
        currentThreshold.release();
        currentChannel.release();

        return input;
    }

    /**
     * @return red high goal as a Rect (null if not visible)
     */
    public Rect getRedRect() {
        return redGoal;
    }

    /**
     * @return blue high goal as a Rect (null if not visible)
     */
    public Rect getBlueRect() {
        return blueGoal;
    }

    /**
     * @return whether the red high goal is visible
     */
    public boolean isRedVisible() {
        return (redGoal != null);
    }

    /**
     * @return whether the blue high goal is visible
     */
    public boolean isBlueVisible() {
        return (blueGoal != null);
    }

    /**
     *
     * @param channel The Cr or Cb channel to filter out red & blue high goal
     * @return High goal target
     */
    private Rect findTarget(Mat channel) {
        // Clears current contour lists
        currentContours.clear();
        interiorContours.clear();

        // Filters channel based on threshold - giving us a binary image
        Imgproc.threshold(channel, currentThreshold, MIN_COLOR_THRESHOLD, MAX_COLOR_THRESHOLD, Imgproc.THRESH_BINARY);
        // Finding the contours along with its heirarchy (so we can find interior/child contours)
        Mat hierarchy = new Mat();
        Imgproc.findContours(currentThreshold, currentContours, hierarchy, Imgproc.RETR_CCOMP, Imgproc.CHAIN_APPROX_NONE);

        // Creating new list of contours to add suitable high goal candidates (ones that have a child and fit aspect ratio)
        List<MatOfPoint> filteredContours = new ArrayList<>();

        for (int i = 0; i < currentContours.size(); i++) {
            //Filtering out all countours that don't have children.
            if(hierarchy.get(0, i)[2] != -1) {
                MatOfPoint currentContour = currentContours.get(i);

                //Further filtering out contours that don't pass this apsect ratio.
                if (isPossibleContour(currentContour)) {
                    filteredContours.add(currentContour);
                }
            }
        }

        if (filteredContours.isEmpty()) {
            return null;
        } else {
            // Comparing width instead of area because wobble goals that are close to the camera tend to have a large area
            biggestContour = Collections.max(filteredContours, Comparator.comparingDouble(t0 -> Imgproc.minAreaRect(new MatOfPoint2f(t0.toArray())).size.area()));
            return Imgproc.boundingRect(biggestContour);
        }
    }

    /**
     * @param contour to be checked
     * @return whether the contour fits high goal shape
     */
    private boolean isPossibleContour(MatOfPoint contour) {
        Rect boundingRect = Imgproc.boundingRect(contour);
        return Imgproc.contourArea(contour) > 50  && boundingRect.width / boundingRect.height > 0.6 && boundingRect.width /boundingRect.height < 1.8;
    }

    public Point getCenterofRect(Rect rect) {
        if (rect == null) {
            return new Point(centerX, centerY);
        }
        return new Point(rect.x + rect.width / 2.0, rect.y + rect.height / 2.0);
    }
}