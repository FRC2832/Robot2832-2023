package org.livoniawarriors;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Preferences;

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

    /**
     * This uses the Preferences API to save settings over power cycles.
     * This is different in that you don't have to set the default value, it will set it for you.
     * @param key The parameter you want to get (slashes are allowed)
     * @param backup The value to use if the key is missing
     * @return The value in NetworkTables if it exists, the backup if missing
     */
    public static double getSetting(String key, double backup) {
        if(Preferences.containsKey(key)) {
            //key exists, return the value
            return Preferences.getDouble(key, backup);
        } else {
            //key missing, set default
            Preferences.initDouble(key, backup);
            return backup;
        }
    }
}
