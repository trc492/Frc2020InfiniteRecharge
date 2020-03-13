package team492.simulator;

import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.RealVector;
import trclib.TrcDriveBase;
import trclib.TrcEvent;
import trclib.TrcHolonomicPurePursuitDrive;
import trclib.TrcPath;
import trclib.TrcPidController;
import trclib.TrcPidDrive;
import trclib.TrcPose2D;
import trclib.TrcUtil;
import trclib.TrcWaypoint;

import javax.swing.*;
import java.awt.*;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class Simulator
{
    private final double xSize;
    private final double ySize;
    private final double scaleFactor;
    private int robotSize;
    private final TrcDriveBase driveBase;
    private final List<TrcPath> paths;
    private JFrame frame;
    private Thread t;

    /**
     * @param xSize       In meters.
     * @param ySize       In meters.
     * @param scaleFactor Pixels per meter.
     * @param driveBase   Drivebase to simulate.
     */
    public Simulator(double xSize, double ySize, double scaleFactor, double robotSize, TrcDriveBase driveBase)
    {
        this.xSize = xSize;
        this.ySize = ySize;
        this.scaleFactor = scaleFactor;
        this.robotSize = round(robotSize * scaleFactor);
        this.driveBase = driveBase;
        paths = new ArrayList<>();
    }

    /**
     * Utility method to block the current thread until the event is signalled.
     *
     * @param event The event to wait for. The method does not clear this event.
     * @return True if the event was waited for successfully. False if the thread was interrupted in the process.
     */
    public boolean blockForEvent(TrcEvent event)
    {
        do
        {
            try
            {
                Thread.sleep(100);
            }
            catch (InterruptedException e)
            {
                return false;
            }
        } while (!event.isSignaled());
        return true;
    }

    public void stop()
    {
        try
        {
            t.interrupt();
            t.join();
        }
        catch (InterruptedException e)
        {
            e.printStackTrace();
        }
        frame.dispose();
        frame = null;
    }

    public void start()
    {
        frame = new JFrame();
        frame.add(new SimulationPanel());
        frame.pack();
        frame.setVisible(true);
        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        t = new Thread(() -> {
            while (!Thread.interrupted())
            {
                frame.repaint();
                try
                {
                    Thread.sleep(20);
                }
                catch (InterruptedException e)
                {
                    break;
                }
            }
        });
        t.start();
    }

    public void addPath(TrcPath path)
    {
        paths.add(path);
    }

    public void clearPaths()
    {
        paths.clear();
    }

    public void removePath(TrcPath path)
    {
        paths.remove(path);
    }

    public List<TrcPath> getPaths()
    {
        return paths;
    }

    private int round(double d)
    {
        return (int) Math.round(d);
    }

    private class SimulationPanel extends JPanel
    {
        public SimulationPanel()
        {
            setPreferredSize(new Dimension((int) (xSize * scaleFactor), (int) (ySize * scaleFactor)));
        }

        private void drawRobot(Graphics g)
        {
            int robotX = round((driveBase.getFieldPosition().x + xSize / 2) * scaleFactor);
            int robotY =
                (int) (ySize * scaleFactor) - round((driveBase.getFieldPosition().y + ySize / 2) * scaleFactor);
            double headingDeg = driveBase.getFieldPosition().angle;
            int halfLength = robotSize / 2;
            int[][] points = new int[][] { { -halfLength, halfLength }, { 0, halfLength * 5 / 4 },
                { halfLength, halfLength }, { halfLength, -halfLength }, { -halfLength, -halfLength } };
            for (int[] point : points)
            {
                int x = point[0];
                int y = point[1];
                RealVector vec = TrcUtil.rotateCW(new ArrayRealVector(new double[] { x, y }), headingDeg);
                point[0] = round(vec.getEntry(0)) + robotX;
                point[1] = -round(vec.getEntry(1)) + robotY;
            }
            g.setColor(Color.RED);
            g.fillPolygon(Arrays.stream(points).mapToInt(p -> p[0]).toArray(),
                Arrays.stream(points).mapToInt(p -> p[1]).toArray(), points.length);
            g.setColor(Color.GREEN);
            int velX = round(driveBase.getFieldVelocity().x * scaleFactor);
            int velY = round(driveBase.getFieldVelocity().y * scaleFactor);
            g.drawLine(robotX, robotY, robotX + velX, robotY - velY);
        }

        @Override
        protected void paintComponent(Graphics g)
        {
            super.paintComponent(g);
            drawRobot(g);

            g.setColor(Color.BLUE);
            for (TrcPath path : paths)
            {
                int pointSize = robotSize / 8;
                TrcWaypoint[] points = path.getAllWaypoints();
                for (int i = 0; i < points.length - 1; i++)
                {
                    int pointX = round((points[i].x + xSize / 2) * scaleFactor);
                    int pointY = (int) (ySize * scaleFactor) - round((points[i].y + ySize / 2) * scaleFactor);
                    g.fillOval(pointX - pointSize / 2, pointY - pointSize / 2, pointSize, pointSize);
                    int point2X = round((points[i + 1].x + xSize / 2) * scaleFactor);
                    int point2Y = (int) (ySize * scaleFactor) - round((points[i + 1].y + ySize / 2) * scaleFactor);
                    g.drawLine(pointX, pointY, point2X, point2Y);
                    g.fillOval(point2X - pointSize / 2, point2Y - pointSize / 2, pointSize, pointSize);
                }
            }
        }
    }
}
