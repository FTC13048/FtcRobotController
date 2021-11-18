package org.firstinspires.ftc.teamcode.test_pipeline_stuff;

import org.firstinspires.ftc.teamcode.test_pipeline_stuff.ElementPosition;
import org.openftc.easyopencv.OpenCvPipeline;

public abstract class ACustomPipeline extends OpenCvPipeline {
    public abstract ElementPosition lastDetectedPosition();
}
