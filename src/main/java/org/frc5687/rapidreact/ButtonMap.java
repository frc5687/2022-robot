package org.frc5687.rapidreact;

import edu.wpi.first.wpilibj.Joystick.AxisType;

/**
 * ButtonMap is used to abstract button and controller mapping out of the OI class, and make it easier to
 * change button assignment without breaking the OI code.
 * 
 * The class is broken into three parts:
 * * The Controllers class holds the port assignments for each controller.  Note that if a controller is to be unplugged
 *   you should set the port to -1.  For the most part this will cause OI to ignore it.
 * 
 * * The Axes class has a subclass for each axis (or group of axes) in use.  Each axis subclass should have a constant defining
 *   which controller it's mapped to and a constant for each axis in the group.
 * 
 * * The Buttons class has a subclass for each button in use.  Each button subclass should have a constant defining
 *   which controller it's mapped to and a constant for the button number itself.
 */
public class ButtonMap {

    public static class Controllers {
        /** 
         * USB ports of joysticks or gamepads.
         * -1 means not in use.
        */
        public static final int DRIVER_JOYSTICK = 0;
        public static final int OPERATOR_JOYSTICK = 1;
        public static final int OPERATOR_GAMEPAD = -1;
    }


    public static class Axes {
        public static class Translation {
            public static int Controller = Controllers.DRIVER_JOYSTICK;
            public static int X = AxisType.kX.value;
            public static int Y = AxisType.kY.value;
        }

        /*  To move the Rotation control from the operator joystick to a gamepad, you'd need to change the
         *  OPERATOR_GAMEPAD constant above to a valid port and change Rotation.Controller to reference Controllers.OPERATOR_GAMEPAD.
         *  You could also change Rotation.Twist to a different axis number if needed.
         * 
         *  See commented code below. NOTE: Twist for gamepad is using Y axis instead of X axis.
         */
        public static class Rotation {
            // Joystick control of rotation
            public static int Controller = Controllers.OPERATOR_JOYSTICK;
            public static int Twist = AxisType.kX.value;

            // Gamepad control of rotation
            // public static int Controller = Controllers.OPERATOR_GAMEPAD;
            // public static int Twist = AxisType.kY.value;
        }
    }

    public static class Buttons {
        public static class SHOOT {
            public static int Controller = Controllers.DRIVER_JOYSTICK;
            public static int Button = 0;
        }

        public static class RESET_NAVX {
            public static int Controller = Controllers.DRIVER_JOYSTICK;
            public static int Button = 5;
        }

    }
}
