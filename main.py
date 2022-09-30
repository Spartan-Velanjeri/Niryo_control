import NiryoMobileController

demo = NiryoMobileController.NiryoMobileController()

demo.initialize()

while(True):
    demo.readSensorData()
    demo.calculateOrientation()
    # demo.controlNiryoJoints()

    