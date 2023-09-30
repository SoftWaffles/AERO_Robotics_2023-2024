/*
 * Copyright (c) 2019 OpenFTC Team
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
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.vision;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import java.util.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;
public class WebcamExample extends OpenCvPipeline
    {


        private int width = 176;


        boolean viewportPaused;


        /*
         * NOTE: if you wish to use additional Mat objects in your processing pipeline, it is
         * highly recommended to declare them here as instance variables and re-use them for
         * each invocation of processFrame(), rather than declaring them as new local variables
         * each time through processFrame(). This removes the danger of causing a memory leak
         * by forgetting to call mat.release(), and it also reduces memory pressure by not
         * constantly allocating and freeing large chunks of memory.
         */

        @Override
        public Mat processFrame(Mat input)
        {
            /*
             * IMPORTANT NOTE: the input Mat that is passed in as a parameter to this method
             * will only dereference to the same image for the duration of this particular
             * invocation of this method. That is, if for some reason you'd like to save a copy
             * of this particular frame for later use, you will need to either clone it or copy
             * it to another Mat.
             */

            /*
             * Draw a simple box around the middle 1/2 of the entire frame
             */
            Mat mat = new Mat();
            Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV); // making img HSV

            Scalar lowHSV = new Scalar(20,150,150);
            Scalar highHSV = new Scalar(30,255,255);
            Mat thresh = new Mat();

            Core.inRange(mat, lowHSV, highHSV, thresh); // creates img with white as yellow black as other colors

            Mat edges = new Mat();
            Imgproc.Canny(thresh, edges, 100, 300); // creates img with edges

            List<MatOfPoint> contours = new ArrayList<>();
            Mat hierarchy = new Mat();
            Imgproc.findContours(edges, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

            MatOfPoint2f[] contoursPoly  = new MatOfPoint2f[contours.size()];
            Rect[] boundRect = new Rect[contours.size()];
            for (int i = 0; i < contours.size(); i++) {
                contoursPoly[i] = new MatOfPoint2f();
                Imgproc.approxPolyDP(new MatOfPoint2f(contours.get(i).toArray()), contoursPoly[i], 3, true);
                boundRect[i] = Imgproc.boundingRect(new MatOfPoint(contoursPoly[i].toArray()));
            }


            double left_x = 0.25 * width;
            double right_x = 0.75 * width;
            String position = "";
            for (int i = 0; i != boundRect.length; i++) {
                Imgproc.rectangle(mat, boundRect[i], new Scalar(255, 255, 255),5);

              //  Imgproc.drawMarker(mat, new Point(boundRect[i].x, boundRect[i].y), new Scalar(255,255,255), Imgproc.MARKER_SQUARE, 155, 8, Imgproc.LINE_8);


              if (boundRect[i].x < left_x) {
                  position = "left";
              }
              else if (boundRect[i].x > right_x) {
                  position = "right";
              }
              else position = "center";
               Imgproc.putText(mat, position, new Point(boundRect[i].x, boundRect[i].y), 0 , 0.5, new Scalar(255, 255, 255), 2);
            }


/*
            // Iterate and check whether the bounding boxes
            // cover left and/or right side of the image
            double left_x = 0.25 * width;
            double right_x = 0.75 * width;
            boolean left = false; // true if regular stone found on the left side
            boolean right = false; // "" "" on the right side
            for (int i = 0; i != boundRect.length; i++) {
                if (boundRect[i].x < left_x)
                    left = true;
                if (boundRect[i].x + boundRect[i].width > right_x)
                    right = true;

                // draw red bounding rectangles on mat
                // the mat has been converted to HSV so we need to use HSV as well
                Imgproc.rectangle(mat, boundRect[i], new Scalar(0.5, 76.9, 89.8),5);
            }


            // if there is no yellow regions on a side
            // that side should be a Skystone
            if (!left) telemetry.addData("Side", "LEFT");
            else if (!right) telemetry.addData("Side", "RIGHT");
                // if both are true, then there's no Skystone in front.
                // since our team's camera can only detect two at a time
                // we will need to scan the next 2 stones
            else telemetry.addData("Side", "NONE");

            telemetry.update();

*/

            Imgproc.rectangle(
                    mat,
                    new Point(
                            input.cols()/4,
                            input.rows()/4),
                    new Point(
                            input.cols()*(3f/4f),
                            input.rows()*(3f/4f)),
                    new Scalar(255, 255, 255), 4);

            /**
             * NOTE: to see how to get data from your pipeline to your OpMode as well as how
             * to change which stage of the pipeline is rendered to the viewport when it is
             * tapped, please see {@link PipelineStageSwitchingExample}
             */

            return mat;
        }

        @Override
        public void onViewportTapped()
        {
            /*
             * The viewport (if one was specified in the constructor) can also be dynamically "paused"
             * and "resumed". The primary use case of this is to reduce CPU, memory, and power load
             * when you need your vision pipeline running, but do not require a live preview on the
             * robot controller screen. For instance, this could be useful if you wish to see the live
             * camera preview as you are initializing your robot, but you no longer require the live
             * preview after you have finished your initialization process; pausing the viewport does
             * not stop running your pipeline.
             *
             * Here we demonstrate dynamically pausing/resuming the viewport when the user taps it
             */

            viewportPaused = !viewportPaused;
        /*
        if(viewportPaused)
        {
            webcam.pauseViewport();
        }
        else
        {
            webcam.resumeViewport();
        }

         */
        }
    }
