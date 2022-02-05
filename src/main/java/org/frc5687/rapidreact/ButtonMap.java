package org.frc5687.rapidreact;

import edu.wpi.first.wpilibj.Joystick.AxisType;

public class ButtonMap {
    public static class Controllers {
        public static final int DRIVER_JOYSTICK = 0;
        public static final int OPERATOR_JOYSTICK = 1;
        public static final int OPERATOR_GAMEPAD = 2;
    }


    public static class Axes {
        public static class Translation {
            public static int Controller = Controllers.DRIVER_JOYSTICK;
            public static int X = AxisType.kX.value;
            public static int Y = AxisType.kY.value;
        }

        public static class Rotation {
            public static int Controller = Controllers.OPERATOR_JOYSTICK;
            public static int Twist = AxisType.kX.value;
        }
    }

    public static class Buttons {
        public static class SHOOT {
            public static int Controller = Controllers.DRIVER_JOYSTICK;
            public static int Button = 0;
        }
    }
}
