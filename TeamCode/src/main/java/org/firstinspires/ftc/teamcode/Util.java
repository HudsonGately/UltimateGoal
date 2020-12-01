package org.firstinspires.ftc.teamcode;

public class Util {
    /**
     * Returns modulus of error where error is the difference between the reference
     * and a measurement.
     *
     * @param reference Reference input of a controller.
     * @param measurement The current measurement.
     * @param minimumInput The minimum value expected from the input.
     * @param maximumInput The maximum value expected from the input.
     */
    public static double getModulusError(double reference, double measurement, double minimumInput,
                                         double maximumInput) {
        double error = reference - measurement;
        double modulus = maximumInput - minimumInput;

        // Wrap error above maximum input
        int numMax = (int) ((error + maximumInput) / modulus);
        error -= numMax * modulus;

        // Wrap error below minimum input
        int numMin = (int) ((error + minimumInput) / modulus);
        error -= numMin * modulus;

        return error;
    }
}
