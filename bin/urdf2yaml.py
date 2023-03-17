#!/usr/bin/python3

import sys
from lxml import etree
import yaml

inFile = sys.argv[1]
xmlData = etree.parse(inFile)

def writeShape(link, dict):
    elem = link.find('origin')
    if elem is not None:
        xyz = elem.attrib.get('xyz')
        rpy = elem.attrib.get('rpy')
        if rpy=='0 0 0':
            rpy=None
        if xyz=='0 0 0':
            xyz=None
        if xyz is not None and rpy is not None:
            dict['rel'] = '<t(%s) E(%s)>' % (xyz, rpy)
        else:
            if rpy is not None:
                dict['rel'] = '<E(%s)>' % (rpy)
            if xyz is not None:
                dict['rel'] = [float(i) for i in xyz.split()]

    elem = link.find('geometry/box')
    if elem is not None:
        dict['shape'] = 'box'
        dict['size'] = float(elem.attrib['size'])

    elem = link.find('geometry/sphere')
    if elem is not None:
        dict['shape'] = 'sphere'
        dict['size'] = float(elem.attrib['radius'])

    elem = link.find('geometry/cylinder')
    if elem is not None:
        dict['shape'] = 'cylinder'
        dict['size'] = '[0 0 %s %s]' % (elem.attrib['length'], elem.attrib['radius'])

    elem = link.find('geometry/mesh')
    if elem is not None:
        filename = elem.attrib['filename'].replace('package://','',1)
        dict['mesh'] = filename
        if elem.find('scale') is not None:
            dict['meshscale'] = float(elem.attrib['scale'])

    elem = link.find('material/color')
    if elem is not None:
        dict['color'] = [float(i) for i in elem.attrib['rgba'].split()]

#    elem = link.find('material')
#    if elem is not None:
#        if elem.attrib['name'] is not None:
#            print('colorName:%s' % elem.attrib['name'])


links = xmlData.findall('/link')
all = {}
for link in links:
    name = link.attrib['name']
    dict = {}
    all[name] = dict

    elem = link.find('inertial/mass')
    if elem is not None:
        dict['mass'] = float(elem.attrib['value'])

    elem = link.find('inertial/inertia')
    if elem is not None:
        dict['inertia'] = [float(elem.attrib['ixx']),
                           float(elem.attrib['ixy']),
                           float(elem.attrib['ixz']),
                           float(elem.attrib['iyy']),
                           float(elem.attrib['iyz']),
                           float(elem.attrib['izz'])]

    # visual shape
    for visual in link.findall('visual'):
        tmp = {}
        all['%s_1 (%s)' % (name, name)] = tmp
        writeShape(visual, tmp)
        tmp['visual'] = True

    # collision shape
    for collision in link.findall('collision'):
        tmp = {}
        all['%s_0 (%s)' % (name, name)] = tmp
        tmp['color'] = [.8, .2, .2, .5]
        writeShape(collision, tmp)
        tmp['contact'] = -2


joints = xmlData.findall('/joint')
for joint in joints:
    name = joint.attrib['name']
    if joint.find('child') is not None:
        dict = {}
        all['%s (%s %s)' % (name,
                            joint.find('parent').attrib['link'],
                            joint.find('child').attrib['link'])] = dict

        # figure out joint type
        att = joint.attrib.get('type')
        
        if att in ['revolute', 'continuous']:
            elem = joint.find('axis')
            if elem is not None:
                axis = elem.attrib['xyz']
                if axis=='1 0 0':
                    dict['joint'] = 'hingeX'
                elif axis=='0 1 0':
                    dict['joint'] = 'hingeY'
                elif axis=='0 0 1':
                    dict['joint'] = 'hingeZ'
                else:
                    raise Exception('CAN ONLY PROCESS X Y Z prismatic joints, not', axis)
            else:
                dict['joint'] = 'hingeX'
                
        if att == 'prismatic':
            elem = joint.find('axis')
            if elem is not None:
                axis = elem.attrib['xyz']
                if axis=='1 0 0':
                    dict['joint'] = 'transX'
                elif axis=='0 1 0':
                    dict['joint'] = 'transY'
                elif axis=='0 -1 0':
                    dict['joint'] = 'transY'
                    dict['joint_scale'] = -1
                elif axis=='0 0 1':
                    dict['joint'] = 'transZ'
                else:
                    raise Exception('CAN ONLY PROCESS X Y Z prismatic joints, not', axis)
            else:
                dict['joint'] = 'transX'
                
        if att == 'fixed':
            dict['joint'] = 'rigid'

        elem = joint.find('mimic')
        if elem is not None:
            dict['mimic'] = elem.attrib['joint']

        #elem = joint.find('axis')
        #if elem is not None:
        #    dict['axis:[%s]' % elem.attrib['xyz'])

        elem = joint.find('origin')
        if elem is not None:
            xyz = elem.attrib.get('xyz')
            rpy = elem.attrib.get('rpy')
            if rpy=='0 0 0':
                rpy=None
            if xyz=='0 0 0':
                xyz=None
            if xyz is not None and rpy is not None:
                dict['pre'] = '<t(%s) E(%s)>' % (xyz, rpy)
            else:
                if rpy is not None:
                    dict['pre'] = '<E(%s)>' % (rpy)
                if xyz is not None:
                    dict['pre'] = [float(i) for i in xyz.split()]

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
                dict['limits'] = [float(lo), float(up)]
            if vel is not None:
                dict['ctrl_limits'] = [float(vel), -1, float(eff)] #the 2nd value is an acceleration limit
        else:
            elem = joint.find('safety_controller')
            if elem is not None:
                lo = elem.attrib.get('soft_lower_limit')
                up = elem.attrib.get('soft_upper_limit')
                if lo is not None:
                    dict['limits'] = [float(lo), float(up)]

#print(all)
print(yaml.dump(all, default_flow_style=True, width=10000, sort_keys=False))
#print(etree.tostring(links[22]))
#print(etree.tostring(joints[0]))
