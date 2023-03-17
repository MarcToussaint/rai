#!/usr/bin/python3

import sys
from lxml import etree

inFile = sys.argv[1]
xmlData = etree.parse(inFile)

useCollisionShapes = False

def writeShape(link):
    elem = link.find('origin')
    if elem is not None:
        xyz = elem.attrib.get('xyz')
        rpy = elem.attrib.get('rpy')
        if rpy=='0 0 0':
            rpy=None
        if xyz=='0 0 0':
            xyz=None
        if xyz is not None and rpy is not None:
            print(' rel: <t(%s) E(%s)>,' % (xyz, rpy), end='')
        else:
            if rpy is not None:
                print(' rel: <E(%s)>,' % (rpy), end='')
            if xyz is not None:
                print(' rel: [%s],' % (xyz), end='')

    elem = link.find('geometry/box')
    if elem is not None:
        print(' shape: box\n  size: [%s 0],' % elem.attrib['size'], end='')

    elem = link.find('geometry/sphere')
    if elem is not None:
        print(' shape: sphere\n  size: [0 0 0 %s],' % elem.attrib['radius'], end='')

    elem = link.find('geometry/cylinder')
    if elem is not None:
        print(' shape: cylinder\n  size: [0 0 %s %s],' % (elem.attrib['length'], elem.attrib['radius']), end='')

    elem = link.find('geometry/mesh')
    if elem is not None:
        filename = elem.attrib['filename'].replace('package://','',1)
        print(' mesh: \'%s\',' % filename, end='')
        if elem.find('scale') is not None:
            print(' meshscale: [%s],' % elem.attrib['scale'], end='')

    elem = link.find('material/color')
    if elem is not None:
        print(' color: [%s],' % elem.attrib['rgba'], end='')

#    elem = link.find('material')
#    if elem is not None:
#        if elem.attrib['name'] is not None:
#            print('colorName:%s' % elem.attrib['name'])


links = xmlData.findall('/link')
for link in links:
    name = link.attrib['name']
    print('%s: {' % name, end='')

    elem = link.find('inertial/mass')
    if elem is not None:
        print(' mass: %s,' % elem.attrib['value'], end='')

    elem = link.find('inertial/inertia')
    if elem is not None:
        print(' inertia: [%s %s %s %s %s %s],' % (
            elem.attrib['ixx'],
            elem.attrib['ixy'],
            elem.attrib['ixz'],
            elem.attrib['iyy'],
            elem.attrib['iyz'],
            elem.attrib['izz']), end='')

    print('}') # end of body

    # visual shape
    for visual in link.findall('visual'):
        print('%s_0 (%s): {' % (name, name), end='')
        writeShape(visual)
        print(' visual: true }') # end of shape

    # collision shape
    if useCollisionShapes:
        for collision in link.findall('collision'):
            print('%s_1 (%s): {' % (name, name), end='')
            print(' color: [.8 .2 .2 .5],', end='')
            writeShape(collision)
            print(' contact: -2 }') # end of shape


joints = xmlData.findall('/joint')
for joint in joints:
    name = joint.attrib['name']
    if joint.find('child') is not None:
        print('%s (%s %s): {' % (name,
                                   joint.find('parent').attrib['link'],
                                   joint.find('child').attrib['link']), end=''),

        # figure out joint type
        att = joint.attrib.get('type')
        
        if att in ['revolute', 'continuous']:
            elem = joint.find('axis')
            if elem is not None:
                axis = elem.attrib['xyz']
                if axis=='1 0 0':
                    print(' joint: hingeX,', end='')
                elif axis=='0 1 0':
                    print(' joint: hingeY,', end='')
                elif axis=='0 0 1':
                    print(' joint: hingeZ,', end='')
                else:
                    raise Exception('CAN ONLY PROCESS X Y Z prismatic joints, not', axis)
            else:
                print(' joint: hingeX,', end='')
                
        if att == 'prismatic':
            elem = joint.find('axis')
            if elem is not None:
                axis = elem.attrib['xyz']
                if axis=='1 0 0':
                    print(' joint: transX,', end='')
                elif axis=='0 1 0':
                    print(' joint: transY,', end='')
                elif axis=='0 -1 0':
                    print(' joint: transY, joint_scale: -1,', end='')
                elif axis=='0 0 1':
                    print(' joint: transZ,', end='')
                else:
                    raise Exception('CAN ONLY PROCESS X Y Z prismatic joints, not', axis)
            else:
                print(' joint: transX,', end='')
                
        if att == 'fixed':
            print(' joint: rigid,', end='')

        elem = joint.find('mimic')
        if elem is not None:
            print(' mimic: %s,' % elem.attrib['joint'], end='')

        #elem = joint.find('axis')
        #if elem is not None:
        #    print('axis:[%s]' % elem.attrib['xyz'])

        elem = joint.find('origin')
        if elem is not None:
            xyz = elem.attrib.get('xyz')
            rpy = elem.attrib.get('rpy')
            if rpy=='0 0 0':
                rpy=None
            if xyz=='0 0 0':
                xyz=None
            if xyz is not None and rpy is not None:
                print(' pre: <t(%s) E(%s)>,' % (xyz, rpy), end='')
            else:
                if rpy is not None:
                    print(' pre: <E(%s)>,' % (rpy), end='')
                if xyz is not None:
                    print(' pre: [%s],' % (xyz), end='')

        elem = joint.find('limit')
        if elem is not None:
            lo = elem.attrib.get('lower')
            up = elem.attrib.get('upper')
            eff = elem.attrib.get('effort')
            vel = elem.attrib.get('velocity')
            if eff=='0':
                eff=None
            if vel=='0':
                vel=None
            if lo is not None:
                print(' limits: [%s %s],' % (lo, up), end='')
            if vel is not None:
                print(' ctrl_limits: [%s -1 %s],' % (vel, eff), end='') #the 2nd value is an acceleration limit
        else:
            elem = joint.find('safety_controller')
            if elem is not None:
                lo = elem.attrib.get('soft_lower_limit')
                up = elem.attrib.get('soft_upper_limit')
                if lo is not None:
                    print(' limits: [%s %s],' % (lo, up), end='')

        print('}')

#print(etree.tostring(links[22]))
#print(etree.tostring(joints[0]))
