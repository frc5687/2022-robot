package org.frc5687.rapidreact.util;

import edu.wpi.first.wpilibj.DriverStation;

import java.io.*;
import java.net.*;
import java.util.Timer;
import java.util.TimerTask;

import static java.lang.Double.NaN;
import static org.frc5687.rapidreact.Constants.UDPJetson.BUFFER;

public class JetsonProxy {

    public static final int JETSON_PORT = 27002;
    public static final int RIO_PORT = 27001;
    public static final int PERIOD = 10;
    public static final String JETSON_IP = "10.56.87.59";

    private Socket _socket;
    private DataInputStream _input;
    private DataOutputStream _output;


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
            _socket = new Socket(JETSON_IP, JETSON_PORT);
            _input = new DataInputStream(_socket.getInputStream());
            _output = new DataOutputStream(_socket.getOutputStream());
        } catch (IOException ioe) {
            DriverStation.reportError(ioe.getMessage(), false);
        }

        _jetsonListener = new JetsonListener(this, _input, _rioPort);
        _listenerThread = new Thread(_jetsonListener);
        _listenerThread.start();

        _jetsonTimer = new Timer();

        _jetsonTimer.schedule(new JetsonTimerTask(this), _period, _period);
    }
    synchronized protected void collect() {
        // Send the heartbeat to the pi
        if (_jetsonListener != null) {
            try {
//                DriverStation.reportError("sending data", false);
                _output.writeUTF(_data);
                _output.flush();
            } catch (IOException ioe) {

            }
        }
        // latestFrame = _piListener==null?null:_piListener.getLatestFrame();
        // trackingPose = latestFrame==null?null: (OutliersPose)poseTracker.getRaw(latestFrame.adjustedMillis);
    }

    protected synchronized void setLatestFrame(Frame frame) {
//        DriverStation.reportError("Setting frame", false);
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
        private double _targetDistance;
        private double _targetAngle;

        public Frame(String packet) {
//            DriverStation.reportError("string is: " + packet, false);
            String[] a = packet.split(";");
            _millis = Long.parseLong(a[0]);
            _estimatedX = Double.parseDouble(a[1]);
            _estimatedY = Double.parseDouble(a[2]);
            _estimatedHeading = Double.parseDouble(a[3]);
            _hasTarget = Boolean.parseBoolean(a[4]);
            _targetDistance = Double.parseDouble(a[5]);
            if (a[6].equals("nan")) {
                _targetAngle = NaN;
            } else {
                _targetAngle = Double.parseDouble(a[6]);
            }
        }

        public long getMillis() { return _millis; }
        public double getEstimatedX() { return _estimatedX; }
        public double getEstimatedY() { return _estimatedY; }
        public boolean hasTarget() { return _hasTarget; }
        public double getTargetDistance() { return _targetDistance; }
        public double getTargetAngle() { return _targetAngle; }


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
        private DataInputStream _input;

        protected JetsonListener(JetsonProxy proxy, DataInputStream input, int rioPort) {
            _proxy = proxy;
            _input = input;
            _rioPort = rioPort;
        }

        @Override
        public void run() {
            try {
                byte[] data = new byte[BUFFER];
                while (true) {
                    if (_input.read(data)> 0) {
                        String raw = new String(data);
//                    DriverStation.reportError(raw, false);
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
    }
}