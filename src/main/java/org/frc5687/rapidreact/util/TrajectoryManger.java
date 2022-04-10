/* Team 5687 (C)2022 */
package org.frc5687.rapidreact.util;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import java.util.HashMap;
import org.frc5687.rapidreact.config.Auto;
import org.frc5687.rapidreact.subsystems.DriveTrain;

public class TrajectoryManger {

    private final HashMap<Trajectories, Trajectory> _trajectories;
    private final DriveTrain _driveTrain;

    public TrajectoryManger(DriveTrain driveTrain) {
        _driveTrain = driveTrain;
        _trajectories = new HashMap<>();
    }

    public void generateTrajectories() {
        _trajectories.put(
                Trajectories.POS_THREE_TO_BALL_TWO,
                TrajectoryGenerator.generateTrajectory(
                        Auto.TrajectoryPoints.PositionThreeToBallTwo.waypoints,
                        _driveTrain.getConfig()));
        _trajectories.put(
                Trajectories.BALL_TWO_TO_BALL_FOUR,
                TrajectoryGenerator.generateTrajectory(
                        Auto.TrajectoryPoints.BallTwoToBallFour.waypoints,
                        _driveTrain.getConfig()));
        _trajectories.put(
                Trajectories.BALL_FOUR_TO_FIELD_SHOT,
                TrajectoryGenerator.generateTrajectory(
                        Auto.TrajectoryPoints.BallFourToFieldShot.waypoints,
                        _driveTrain.getConfig()));
        _trajectories.put(
                Trajectories.POS_ONE_TO_BALL_ONE,
                TrajectoryGenerator.generateTrajectory(
                        Auto.TrajectoryPoints.PositionOneToBallOne.waypoints,
                        _driveTrain.getConfig()));
        _trajectories.put(
                Trajectories.POS_TWO_TO_BALL_ONE,
                TrajectoryGenerator.generateTrajectory(
                        Auto.TrajectoryPoints.PositionTwoToBallOne.waypoints,
                        _driveTrain.getConfig()));
        _trajectories.put(
                Trajectories.POS_FOUR_TO_BALL_THREE,
                TrajectoryGenerator.generateTrajectory(
                        Auto.TrajectoryPoints.PositionFourToBallThree.waypoints,
                        _driveTrain.getConfig()));
    }

    public Trajectory getTrajectory(Trajectories trajectory) {
        return _trajectories.get(trajectory);
    }

    public enum Trajectories {
        POS_THREE_TO_BALL_TWO(0),
        BALL_TWO_TO_BALL_FOUR(1),
        BALL_FOUR_TO_FIELD_SHOT(2),
        POS_ONE_TO_BALL_ONE(3),
        POS_TWO_TO_BALL_ONE(4),
        POS_FOUR_TO_BALL_THREE(5);

        private final int _value;

        Trajectories(int value) {
            _value = value;
        }

        public int getValue() {
            return _value;
        }
    }
}
