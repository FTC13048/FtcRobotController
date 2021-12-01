package org.firstinspires.ftc.teamcode.VisionStuff;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.List;

public class VisionWrapper {
    private OpenCvWebcam webcam;
    private GripPipeline grip;

    private static final int CAMERA_WIDTH = 1280;

    public VisionWrapper(){ grip = new GripPipeline(); }

    public void init(HardwareMap hmap){ initVision(hmap); }

    private void initVision(HardwareMap hmap){
        int cameraMonitorViewId = hmap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hmap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hmap.get(WebcamName.class, "pp2"), cameraMonitorViewId);

        webcam.setPipeline(grip);
        webcam.setMillisecondsPermissionTimeout(2500); // Timeout for obtaining permission is configurable. Set before opening.

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode){ }
        });
    }

    private DetectionLevel updateDetermination(){
        List<MatOfPoint> contours = grip.filterContoursOutput();
        for(int i = 0; i < contours.size(); i++){
            Rect boundingRect = Imgproc.boundingRect(contours.get(i));
            if(boundingRect.area() < 20){
                contours.remove(i);
                i--;
            }
        }

        if(contours.size() == 0){
            return DetectionLevel.LEVEL_THREE;
        }else{
            contours.size();
            double areaSum = 0;
            double xAvg = 0;
            //double yAvg = 0;
            for (int i = 0; i < contours.size(); i++) {
                Rect boundingRect = Imgproc.boundingRect(contours.get(i));
                double areaTotal = boundingRect.area();
                areaSum = areaSum + areaTotal;
                xAvg = xAvg + areaTotal * (boundingRect.x + boundingRect.width / 2d);
                //yAvg = yAvg + areaLol*(boundingRect.y + boundingRect.height / 2d);
            }
            xAvg = xAvg / areaSum;

            if (xAvg > CAMERA_WIDTH/2) {
                return DetectionLevel.LEVEL_TWO;
            } else {
                return DetectionLevel.LEVEL_ONE;
            }
        }
    }

    public DetectionLevel currentDetermination(){ return this.updateDetermination(); }

    public void stop(){ webcam.stopStreaming(); }

    public enum DetectionLevel{
        LEVEL_ONE("1 - Lowest level"), LEVEL_TWO("2 - Middle level"),
        LEVEL_THREE("3 - Top level"), UNKNOWN("Unknown");

        private final String level;

        DetectionLevel(String str) {
            this.level = str;
        }

        @Override
        public String toString() { return this.level; }
    }
}
