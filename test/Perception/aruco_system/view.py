import robotic as ry

C = ry.Config()
C.addFile('scene.yml')
C.view(True)

cams = []
for i in range(4):
    cams.append(C.getFrame(f'cam{i}'))

V = ry.CameraView(C)
for _ in range(20):
    C.setRandom()
    for c in cams:
        V.setCamera(c)
        rgb, depth = V.computeImageAndDepth(C)
        C.get_viewer().setQuad(0, rgb, 0., 0., .3)
        C.view(True)