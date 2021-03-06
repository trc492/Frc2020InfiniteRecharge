/*
 * Copyright (c) 2020 Titan Robotics Club (http://www.titanrobotics.com)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package team492;

import hallib.HalDashboard;
import trclib.TrcRobot;
import trclib.TrcRobot.RunMode;

public class FrcDisabled implements TrcRobot.RobotMode
{
    private static final String disableSensorsKey = "DisableSensorsToggle";

    private final Robot robot;

    public FrcDisabled(Robot robot)
    {
        this.robot = robot;
    }   // FrcDisabled

    //
    // Implements TrcRobot.RunMode interface.
    //

    @Override
    public void startMode(RunMode prevMode, RunMode nextMode)
    {
        robot.alignment.enableRanging();
        if (robot.preferences.useVision)
        {
            robot.vision.setEnabled(true);
        }
    }   // startMode

    @Override
    public void stopMode(RunMode prevMode, RunMode nextMode)
    {
    }   // stopMode

    @Override
    public void runPeriodic(double elapsedTime)
    {
        robot.updateDashboard(RunMode.DISABLED_MODE);
        if (HalDashboard.getBoolean(disableSensorsKey, false))
        {
            robot.alignment.disableRanging();
            if (robot.preferences.useVision)
            {
                robot.vision.setEnabled(false);
            }
        }
        else
        {
            robot.alignment.enableRanging();
            if (robot.preferences.useVision)
            {
                robot.vision.setEnabled(true);
            }
        }
    }   // runPeriodic

    @Override
    public void runContinuous(double elapsedTime)
    {
    }   // runContinuous

}   // class FrcDisabled
