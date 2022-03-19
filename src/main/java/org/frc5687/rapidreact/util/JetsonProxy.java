package org.frc5687.rapidreact.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.frc5687.rapidreact.commands.Drive;

import java.io.IOException;
import java.io.InputStream;
import java.net.*;
import java.util.Timer;
import java.util.TimerTask;

import static org.frc5687.rapidreact.Constants.UDPJetson.BUFFER;

public class JetsonProxy {

    public static final int JETSON_PORT = 27002;
    public static final int RIO_PORT = 27001;
    public static final int PERIOD = 10;

    DatagramSocket outgoingSocket;
    private Thread _listenerThread;

    private int _period = PERIOD;
    private int _jetsonPort = JETSON_PORT;
    private int _rioPort = RIO_PORT;

    private Frame _latestFrame;

    private long trackingMillis = System.currentTimeMillis();
    private JetsonListener _jetsonListener;
    private Timer _jetsonTimer;

    private String _data = "";

    public JetsonProxy(int period) {
        _period = period;
        try {
            outgoingSocket = new DatagramSocket();
        } catch (IOException ioe) {
            outgoingSocket = null;
        }

        _jetsonListener = new JetsonListener(this, _rioPort);
        _listenerThread = new Thread(_jetsonListener);
        _listenerThread.start();

        _jetsonTimer = new Timer();

        _jetsonTimer.schedule(new JetsonTimerTask(this), _period, _period);
    }
    synchronized protected void collect() {
        long rioMillis = System.currentTimeMillis();
        // Send the heartbeat to the pi
        if (_jetsonListener != null) {
            InetAddress jetsonAddress = _jetsonListener.getJetsonAddress();
            if (jetsonAddress != null) {
                byte[] sendData = new byte[BUFFER];
                sendData = _data.getBytes();
                DatagramPacket sendPacket = new DatagramPacket(sendData, _data.length(), jetsonAddress, _jetsonPort);
                try {
                    outgoingSocket.send(sendPacket);
                } catch (IOException ioe) {
                }
            }
        }
        // latestFrame = _piListener==null?null:_piListener.getLatestFrame();
        // trackingPose = latestFrame==null?null: (OutliersPose)poseTracker.getRaw(latestFrame.adjustedMillis);
    }
    protected synchronized void setLatestFrame(Frame frame) {
        _latestFrame = frame;
    }

    public synchronized Frame getLatestFrame() {
        return _latestFrame;
    }

    public void setData(Data... data) {
        StringBuilder buffer = new StringBuilder();
        for (Data d : data) {
            buffer.append(d.toString());
            buffer.append(";");
        }
        _data = buffer.toString();
    }

    public static class Data {
        private Double _double = null;
        private Long _long = null;
        private String _string = null;
        private Boolean _boolean = null;
        private Integer _int = null;

        public Data(double data) {
            _double = data;
        }
        public Data(long data) {
            _long = data;
        }
        public Data(String data) {
            _string = data;
        }
        public Data(Boolean data) {
            _boolean = data;
        }
        public Data(int data) {
            _int = data;
        }

        @Override
        public String toString() {
            if (_double != null) {
                return _double.toString();
            } else if (_long != null) {
                return _long.toString();
            } else if (_string != null) {
                return _string;
            } else if (_boolean != null) {
                return _boolean.toString();
            } else if (_int != null) {
                return _int.toString();
            }
            return null;
        }
    }
    public class Frame {
        private long _millis;
        private double _estimatedX;
        private double _estimatedY;
        private double _estimatedHeading;
        private boolean _hasTarget;
        private double _goalDistance;
        private double _goalAngle;
        private double _target_x;
        private double _target_y;
        private double _target_z;
        private double _target_vx;
        private double _target_vy;
        private double _target_vz;
        private double _blue_ball_yaw;
        private double _red_ball_yaw;

        public Frame(String packet) {
//            DriverStation.reportError("string is: " + packet, false);
            if (!packet.equals("nan")) {
                String[] a = packet.split(";");
                _millis = Long.parseLong(a[0]);
                _estimatedX = Double.parseDouble(a[1]);
                _estimatedY = Double.parseDouble(a[2]);
                _estimatedHeading = Double.parseDouble(a[3]);
                _hasTarget = Boolean.parseBoolean(a[4]);
                _goalDistance = Double.parseDouble(a[5]);
                _goalAngle = Double.parseDouble(a[6]);
                _target_x = Double.parseDouble(a[7]);
                _target_y = Double.parseDouble(a[8]);
                _target_z = Double.parseDouble(a[9]);
                _target_vx = Double.parseDouble(a[10]);
                _target_vy = Double.parseDouble(a[11]);
                _target_vz = Double.parseDouble(a[12]);
                _blue_ball_yaw = Double.parseDouble(a[13]);
                _red_ball_yaw = Double.parseDouble(a[14]);
            }
        }

        public long getMillis() { return _millis; }
        public double getEstimatedX() { return _estimatedX; }
        public double getEstimatedY() { return _estimatedY; }
        public double getTargetDistance() { return _goalDistance; }
        public double getTargetAngle() { return _goalAngle; }
        public boolean hasTarget() { return _hasTarget; }
        public double[] targetPosition() {
            return new double[]{_target_x, _target_y, _target_z};
        }
        public double[] targetVelocity() {
            return new double[]{_target_vx, _target_vy, _target_vz};
        }
        public double getBlueBallYaw() { return _blue_ball_yaw; }
        public double getRedBallYaw() { return _red_ball_yaw; }
    }
    protected class JetsonTimerTask extends TimerTask {
        private JetsonProxy _proxy;

        protected JetsonTimerTask(JetsonProxy proxy) {
            _proxy = proxy;
        }

        @Override
        public void run() {
            _proxy.collect();
        }
    }
    protected class JetsonListener implements Runnable {
        private JetsonProxy _proxy;
        private InetAddress _jetsonAddress = null;
        private int _rioPort;
        private long _prevTime;

        protected JetsonListener(JetsonProxy proxy, int rioPort) {
            _proxy = proxy;
            _rioPort = rioPort;
        }

        @Override
        public void run() {
            DatagramSocket incomingSocket;
            byte[] receiveData = new byte[BUFFER];
            try {
                incomingSocket = new DatagramSocket(_rioPort);
                while (true) {
                    DatagramPacket receivePacket = new DatagramPacket(receiveData, receiveData.length);
                    incomingSocket.receive(receivePacket);
                    if (receivePacket == null) {

                    } else {
                        synchronized (this) {
                            _jetsonAddress = receivePacket.getAddress();
                        }
                        String raw = new String(receivePacket.getData(), 0, receivePacket.getLength());
                        Frame frame = new Frame(raw);
                        _proxy.setLatestFrame(frame);
                    }
                }
            } catch (IOException ioe) {
                DriverStation.reportError("IOE Exception getting frame", false);
            } catch (Exception e) {
                DriverStation.reportError("Exception getting frame [Error]: " + e.getMessage(), false);
            }
        }
        public synchronized InetAddress getJetsonAddress() {
            return _jetsonAddress;
        }
    }
}