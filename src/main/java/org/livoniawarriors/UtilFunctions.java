package org.livoniawarriors;

import edu.wpi.first.math.MathUtil;

public class UtilFunctions {
    /**
     * This function takes a joystick input from -1 to 1 and removes the center of the stick.
     * This is because Xbox joysticks have awful centering
     * @param input Joystick input to manipulate
     * @param deadband How much of the center we need to remove (Xbox 360 controllers was around 0.2, Xbox One 0.13)
     * @return A value between -1 to 1 that will not drift with stick drift
     */
    public static double deadband(double input, double deadband) {
        double abs = Math.abs(input);

        if (abs > deadband) {
            return Math.signum(input) * ((abs-deadband)/(1-deadband));
        } else {
            return 0;
        }
    }

    /**
     * This function takes a input in degrees and makes it -180 to 180*.
     * If you are in radians, use MathUtil.angleModulus() from WpiLib
     * @param degAngle Angle to reduce
     * @return A value between -180 to 180*
     */
    public static double degreeMod(double degAngle) {
        return MathUtil.inputModulus(degAngle,-180,180);
    }
}
