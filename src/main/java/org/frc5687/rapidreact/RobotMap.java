/* Team 5687 (C)2020-2021 */
package org.frc5687.rapidreact;

public class RobotMap {

    /**
     * There should be an entry here for each CAN device, preferrably grouped by device type and
     * then in numerical order. Note that for CAN, ids must be unique per device type, but not
     * across types. Thus, you cannot have two SparkMax controllers with Id 0, but you can have a
     * SparkMax with Id 0 and a TalonSRX with Id 0.
     */
    public static class CAN {

        public static class TALONFX {
            /**
             * Pulled in from 2021-Proto-Bot
             *          N
             *          |
             *      E -- -- W
             *          |
             *          S
             */
            public static final int BL_RIGHT_FALCON = 4;
            public static final int BL_LEFT_FALCON = 3;
            public static final int BR_RIGHT_FALCON = 6;
            public static final int BR_LEFT_FALCON = 5;
            public static final int FL_RIGHT_FALCON = 1;
            public static final int FL_LEFT_FALCON = 2;
            public static final int FR_RIGHT_FALCON = 8;
            public static final int FR_LEFT_FALCON = 7;
        }

        public static class SPARKMAX {
            public static final int SPRING_BABY_NEO = 1;
            public static final int WINCH_BABY_NEO = 2;

        }
    }

    /**
     * There should be an entry here for each PWM port, preferrably in numerical order. Note that
     * for PWM only one device can connect to each port, so the numbers should be unique.
     */
    public static class PWM {}

    /**
     * There should be an entry here for each PCM port, preferrably in numerical order. Note that
     * for PCH only one device can connect to each port, so the numbers should be unique.
     * This is the REVRobotics Pneumatic Control Hub.
     */
    public static class PCH {
        public static int RELEASE_PIN_HIGH = 0;
        public static int RELEASE_PIN_LOW = 1;

    }

    /**
     * There should be an entry here for each PDP breaker, preferrably in numerical order. Note that
     * only on device can be connected to each breaker, so the numbers should be unique.
     */
    public static class PDP {}

    /**
     * There should be an entry here for each Analgo port, preferrably in numerical order. Note that
     * for Analog only one device can connect to each port, so the numbers should be unique.
     */
    public static class Analog {}

    /**
     * There should be an entry here for each DIO port, preferrably in numerical order. Note that
     * for DIO only one device can connect to each port, so the numbers should be unique.
     */
    public static class DIO {
        public static final int ENCODER_FR = 4; //DIO 4
        public static final int ENCODER_FL = 3; //DIO 3
        public static final int ENCODER_BR = 1; //DIO 1
        public static final int ENCODER_BL = 2; //DIO 2
    }
}
