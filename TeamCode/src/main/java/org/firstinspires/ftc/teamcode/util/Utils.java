package org.firstinspires.ftc.teamcode.util;

/**
 * This Class is used to house any custom static methods you may need to implement
 */
public class Utils {

    /**
     * Computes a scaling factor to adjust powers if any value is outside of -1:1
     * scaleFactor maintains the ratios of each power to the others while
     * capping the maximum magnitude at 1.
     * @param p1 Motor power 1
     * @param p2 Motor power 2
     * @param p3 Motor power 3
     * @param p4 Motor power 4
     * @return
     */

    public static double scalePower(double p1, double p2, double p3, double p4)
    {
        //Returns a scaling factor to normalize the input powers p1-p4 to a maximum magnitude of 1

        double maxValue = Math.abs(p1);
        double scaleFactor = 1;

        //Search for the largest power magnitude
        if (Math.abs(p2) > maxValue) {
            maxValue = Math.abs(p2);
        }
        if (Math.abs(p3) > maxValue) {
            maxValue = Math.abs(p3);
        }
        if (Math.abs(p4) > maxValue) {
            maxValue = Math.abs(p4);
        }

        //If maxValue is larger than 1 return a scale factor to limit it to +1 or -1
        if (maxValue > 1 )
        {
            scaleFactor = 1 / maxValue;
        }

        return  scaleFactor;
    }

    /**
     * This is a duplication of the sleep method available in LinearOpMode
     * Placing an implementation here allows more general use in other Classes
     * Note that sleep is blocking and will freeze the current thread until milliseconds
     * has passed.  This can especially cause problems in teleOp.
     * @param milliseconds
     */

    public static void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
}
