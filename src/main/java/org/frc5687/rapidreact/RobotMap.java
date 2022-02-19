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
        /**
         *          N
         *          |
         *      E -- -- W
         *          |
         *          S
         */

        public static class TALONFX {
            public static final int NORTH_EAST_OUTER = 1;
            public static final int NORTH_EAST_INNER = 2;
            public static final int SOUTH_EAST_INNER = 3;
            public static final int SOUTH_EAST_OUTER = 4;
            public static final int SOUTH_WEST_OUTER = 5;
            public static final int SOUTH_WEST_INNER = 6;
            public static final int NORTH_WEST_INNER = 7;
            public static final int NORTH_WEST_OUTER = 8;
        }

        public static class SPARKMAX {
            public static final int SPRING_BABY_NEO = 9;
            public static final int WINCH_BABY_NEO = 10;
            public static final int INTAKE_ROLLER = 13;
        }
    }

    /**
     * There should be an entry here for each PWM port, preferrably in numerical order. Note that
     * for PWM only one device can connect to each port, so the numbers should be unique.
     */
    public static class PWM {
        public static final int INTAKE_STOPPER = 9;
    }

    /**
     * There should be an entry here for each PCM port, preferrably in numerical order. Note that
     * for PCH only one device can connect to each port, so the numbers should be unique.
     * This is the REVRobotics Pneumatic Control Hub.
     */
    public static class PCH {
        public static int RELEASE_PIN_HIGH = 7;
        public static int RELEASE_PIN_LOW = 8;

        public static final int INTAKE_HIGH = 10;
        public static final int INTAKE_LOW = 5;
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
        public static final int NORTH_EAST = 0;
        public static final int SOUTH_EAST = 1;
        public static final int SOUTH_WEST = 2;
        public static final int NORTH_WEST = 3;

        public static final int SPRING_HALL_EFFECT = 4;
        public static final int ARM_HALL_EFFECT = 5;
        public static final int INTAKE_HALL_EFFECT = 22;
    }
}
