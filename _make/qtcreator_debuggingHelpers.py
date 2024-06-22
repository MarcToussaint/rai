#IN NEW VERSIONS of qtcreator:
    # add this file in Options -> Debugger -> GDB -> Extra Dumper Files

#IN OLD VERSIONS, this is loaded via .gdbinit and you need the python and end flags!
#python

#import sys
#sys.path.insert(0, '/home/mtoussai/opt/qtcreator-4.2.0-rc1/share/qtcreator/debugger')

from dumper import *

import math

def qdump__rai__String(d, value):
    p = value["p"]
    N = int(value["N"])
    s = "'"
    for i in range(0, N):
        s += "%c" % int(p.dereference())
        p += 1
    s += "' [%i]" % N
    d.putValue(s)
    d.putNumChild(1)
    if d.isExpanded():
        with Children(d):
            d.putSubItem("N", value["N"])
            d.putSubItem("p", value["p"])

def qdump__rai__Enum(d, value):
    d.putItem(value["x"])

def qdump__LIST(d, value):
    p = value["p"]
    N = int(value["N"])
    s = "<%i>" %N
    d.putValue(s)
    m=N
    if m>10:
        m=10
    d.putNumChild(1)
    if d.isExpanded():
        with Children(d):
            for i in range(0, m):
                s = "(%i)" % i
                d.putSubItem(s, p.dereference())
                p += 1

def qdump__rai__Array(d, value):
    p = value["p"]
    N = int(value["N"])
    nd = int(value["nd"])
    d0 = int(value["d0"])
    d1 = int(value["d1"])
    d2 = int(value["d2"])
    M = int(value["M"])
    if nd==0:
        s = "<>"
    if nd==1:
        s = "<%i>" %d0
    if nd==2:
        s = "<%i %i>"%(d0,d1)
    if nd==3:
        s = "<%i %i %i>"%(d0,d1,d2)
    d.putValue(s)
    m=N
    if m>10:
        m=10
    d.putNumChild(1)
    if d.isExpanded():
        with Children(d):
            d.putSubItem("N", value["N"])
            for i in range(0, m):
                if nd==1:
                    s = "(%i)" %(i)
                if nd==2:
                    s = "(%i,%i)"%(i/d1,i%d1)
                if nd==3:
                    s = "(%i,%i,%i)"%(i/(d1*d2),(i/d2)%d1,i%d2)
                d.putSubItem(s, p.dereference())
                p += 1
            d.putSubItem("p", value["p"])
            d.putSubItem("isReference", value["isReference"])
            d.putSubItem("M", value["M"])
            d.putSubItem("special", value["special"])
            d.putSubItem("jac", value["jac"])
            
def qdump__rai__ArrayDouble(d, value):
    qdump__rai__Array(d, value)

def qdump__rai__Node_typed(d, value):
    pars_N = int(value["parents"]["N"])
    pars_p = value["parents"]["p"]
    s = ""
    string = value["key"]
    string_N = int(string["N"])
    if string_N>0:
        string_p = string["p"]
        for j in range(0, string_N):
            s += "%c" % int(string_p.dereference())
            string_p += 1
    else:
        s += "(%i)" % int(value["index"]);
    s += "("
    for i in range(0, pars_N):
        par = pars_p.dereference()
        string = par["key"]
        string_N = int(string["N"])
        if string_N>0:
            string_p = string["p"]
            for j in range(0, string_N):
                s += "%c" % int(string_p.dereference())
                string_p += 1
        else:
            s += "(%i)" % int(par["index"]);
        pars_p += 1
        if(i<pars_N-1):
            s += " "
    s += ")"
    d.putValue(s)
    d.putNumChild(1)
    if d.isExpanded():
        with Children(d):
            d.putSubItem("value", value["value"])
            d.putSubItem("key", value["key"])
            d.putSubItem("parents", value["parents"])
            d.putSubItem("numChildren", value["numChildren"])
            d.putSubItem("children", value["children"])
            d.putSubItem("index", value["index"])
            d.putSubItem("container", value["container"])

def qdump__rai__NodeL(d, value):
    qdump__LIST(d, value)

def qdump__rai__Graph(d, value):
    p = value["p"]
    N = int(value["N"])
    s = "<%i>" % N
    d.putValue(s)
    m=N
    if m>10:
        m=10
    d.putNumChild(1)
    if d.isExpanded():
        with Children(d):
            for i in range(0, m):
                s = "(%i)" %i
                d.putSubItem(s, p.dereference())
                p += 1
            d.putSubItem("isNodeOfGraph", value["isNodeOfGraph"])
            d.putSubItem("isIndexed", value["isIndexed"])
            d.putSubItem("isDoubleLinked", value["isDoubleLinked"])

def qdump__rai__Vector(d, value):
    x = value["x"].to('d')
    y = value["y"].to('d')
    z = value["z"].to('d')
    s = "[%.3g %.3g %.3g]" % (x,y,z)
    d.putValue(s)
    d.putNumChild(1)
    if d.isExpanded():
        with Children(d):
            d.putSubItem("x", value["x"])
            d.putSubItem("y", value["y"])
            d.putSubItem("z", value["z"])
            d.putSubItem("isZero", value["isZero"])
            with SubItem(d, "length"):
                d.putValue(math.sqrt(x*x+y*y+z*z))
                d.putType("float")
                d.putNumChild(0)

def qdump__rai__Quaternion(d, value):
    w = value["w"].to('d')
    x = value["x"].to('d')
    y = value["y"].to('d')
    z = value["z"].to('d')
    s = "[%.3f %.3f %.3f %.3f]" % (w,x,y,z)
    d.putValue(s)
    d.putNumChild(1)
    if d.isExpanded():
        with Children(d):
            d.putSubItem("w", value["w"])
            d.putSubItem("x", value["x"])
            d.putSubItem("y", value["y"])
            d.putSubItem("z", value["z"])
            d.putSubItem("isZero", value["isZero"])
            with SubItem(d, "degrees"):
                d.putValue(180.0/math.pi*math.acos(w))
                d.putType("float")
                d.putNumChild(0)

def qdump__rai__Transformation(d, value):
    px = value["pos"]["x"].to('d')
    py = value["pos"]["y"].to('d')
    pz = value["pos"]["z"].to('d')
    qw = value["rot"]["w"].to('d')
    qx = value["rot"]["x"].to('d')
    qy = value["rot"]["y"].to('d')
    qz = value["rot"]["z"].to('d')
    s = "[%.1f %.1f %.1f %.1f %.1f %.1f %.1f]" % (px,py,pz,qw,qx,qy,qz)
    d.putValue(s)
    d.putNumChild(1)
    if d.isExpanded():
        with Children(d):
            d.putSubItem("pos", value["pos"])
            d.putSubItem("rot", value["rot"])

def qdump__rai__Frame(d, value):
    ID = int(value["ID"])
    name = value["name"]
    s = "(%i) " % ID
    string_N = int(name["N"])
    string_p = name["p"]
    for j in range(0, string_N):
        s += "%c" % int(string_p.dereference())
        string_p += 1
    d.putValue(s)
    d.putNumChild(1)
    if d.isExpanded():
        with Children(d):
            d.putSubItem("ID", value["ID"])
            d.putSubItem("name", value["name"])
            d.putSubItem("parent", value["parent"])
            d.putSubItem("prev", value["prev"])
            d.putSubItem("children", value["children"])
            d.putSubItem("Q", value["Q"])
            d.putSubItem("X", value["X"])
            d.putSubItem("tau", value["tau"])
            d.putSubItem("joint", value["joint"])
            d.putSubItem("shape", value["shape"])
            d.putSubItem("inertia", value["inertia"])
            d.putSubItem("forces", value["forces"])
            d.putSubItem("ats", value["ats"])
            d.putSubItem("_state_X_isGood", value["_state_X_isGood"])
            d.putSubItem("C", value["C"])

#end
