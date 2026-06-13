import robotic as ry
import drawsvg as draw

Vi = ry.ConfigurationViewer()
d = draw.Drawing(600, 600)
for k in range(5):
    I = ry.getArucoImage(k)
    Vi.setQuad(0, I, 0., 0., 1.)
    Vi.view()
    d.append(draw.Text(f'{50+k}', 5, 9+k*100, 9+k*100, text_anchor='end', font_family='Arial Black'))
    for i in range(I.shape[0]):
        for j in range(I.shape[1]):
            if I[i,j]==0:
                d.append(draw.Rectangle(k*100+10+j*10, k*100+10+i*10, 10, 10))
d.save_svg('example.svg')
exit(0)

C = ry.Config()
# C.addFile('scene.yml')
C.addFile('scene_bimanual.yml')
C.view(True)

cams = []
for i in range(6):
    cams.append(C.getFrame(f'cam{i}'))

V = ry.CameraView(C)
for _ in range(20):
    C.setRandom()
    for c in cams:
        V.setCamera(c)
        rgb, depth = V.computeImageAndDepth(C)
        C.get_viewer().setQuad(0, rgb, 0., 0., .3)
        C.view(True)