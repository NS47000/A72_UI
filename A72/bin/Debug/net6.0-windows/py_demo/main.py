from MLSDK import Colorimeter
import time

ml= Colorimeter()

# Colorimeter Func
# ml.IQ_SetExposure(1000)
# ml.CLRMTF_CameraFocus()
# ml.IQ_BeginAndImgCrop()
# ml.IQ_SwitchNDFilter('ND3')
# ml.IQ_SwitchXYZFilter('Y')
# ml.CLRMTF_SelectProfiles()
# ml.CLRMTF_OutPutResult()

# Spectroradiometer func
# ml.SPEC_Open()
# ml.SPEC_TargetON()
# ml.SPEC_TargetOFF()
# ml.SPEC_ConfigSpectroradiometerInfo(59995,121,0,10)
# ml.SPEC_OutputXYZResult()
# ml.SPEC_SwitchCalibrationfile(0)
# ml.SPEC_StartMeasurement()
# ml.SPEC_StopMeasurement()
# ml.SPEC_SetROI(1000,2000,500,300)
# ml.SPEC_CompareChromaticity()

# Motion and safety control
# ml.MASC_InitMotor()
# ml.MASC_Move2TestPos()
# ml.MASC_CheckSensors()

# add cycle test
ml.SetExposureTime(1000)
time.sleep(2)

ml.SetCylinderMirror(-1, 45)
time.sleep(2)

ml.SetCameraBinning(1)
time.sleep(2)

ml.SwitchNDFilter("ND3")
time.sleep(2)

ml.SwitchXYZFilter('Y')
time.sleep(2)

ml.GoToTestPosition()
time.sleep(2)

ml.SetSpectroradiometerConfigByIndex(1)
time.sleep(2)

for index in range(0, 9):
    ml.GetChromaXYZImage(0)
    time.sleep(2)

    ml.SetElectricMirrorState(1)
    time.sleep(2)

    ml.SpectroradiometerStartMeasurement()
    time.sleep(2)

    ml.SetElectricMirrorState(0)
    time.sleep(2)

    ml.CLRMTF_GetXYZImg()
    time.sleep(2)

    ml.GetSpectroradiometerXYZResult()
    time.sleep(2)

    ml.SetResultCompareROI(1000, 2000, 500, 300)
    time.sleep(2)

    ml.CompareChromaticity()
    time.sleep(2)

    ml.GoToEyeBoxLocationByIndex(1, index)
    time.sleep(2)

ml.GoToLoadPosition()

ml.SwitchNDFilter('ND1')