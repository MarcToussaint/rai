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
    for i in xrange(0, N):
        s += "%c" % int(p.dereference())
        p += 1
    s += "' [%i]" % N
    d.putValue(s)
    d.putNumChild(2)
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
    d.putNumChild(m+1)
    if d.isExpanded():
        with Children(d):
            for i in xrange(0, m):
                s = "(%i)" % i
                d.putSubItem(s, p.dereference())
                p += 1
                i += 1
#            d.putSubItem("p", p)


def qdump__rai__Array(d, value):
    p = value["p"]
    N = int(value["N"])
    nd = int(value["nd"])
    d0 = int(value["d0"])
    d1 = int(value["d1"])
    d2 = int(value["d2"])
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
    d.putNumChild(m+4)
    if d.isExpanded():
        with Children(d):
            d.putSubItem("N", value["N"])
            for i in xrange(0, m):
                if nd==1:
                    s = "(%i)" %(i)
                if nd==2:
                    s = "(%i,%i)"%(i/d1,i%d1)
                if nd==3:
                    s = "(%i,%i,%i)"%(i/(d1*d2),(i/d2)%d1,i%d2)
                d.putSubItem(s, p.dereference())
                p += 1
                i += 1
            d.putSubItem("p", value["p"])
            d.putSubItem("isReference", value["isReference"])
            d.putSubItem("special", value["special"])
            
def qdump__Node_typed(d, value):
    keys_N = int(value["keys"]["N"])
    keys_p = value["keys"]["p"]
    pars_N = int(value["parents"]["N"])
    pars_p = value["parents"]["p"]
    s = ""
    for i in xrange(0, keys_N):
        string = keys_p.dereference()
        string_N = int(string["N"])
        string_p = string["p"]
        for j in xrange(0, string_N):
            s += "%c" % int(string_p.dereference())
            string_p += 1
        keys_p += 1
        if(i<keys_N):
            s += " "
    s += "("
    for i in xrange(0, pars_N):
        par = pars_p.dereference()
        parkeys_N = int(par["keys"]["N"])
        parkeys_p = par["keys"]["p"]
        if(parkeys_N>0):
            string = (parkeys_p+(parkeys_N-1)).dereference()
            string_N = int(string["N"])
            string_p = string["p"]
            for j in xrange(0, string_N):
                s += "%c" % int(string_p.dereference())
                string_p += 1
        else:
            s += "(%i)" % int(par["index"]);
        pars_p += 1
        if(i<pars_N-1):
            s += " "
    s += ")"
    d.putValue(s)
    d.putNumChild(4)
    if d.isExpanded():
        with Children(d):
            d.putSubItem("value", value["value"])
            d.putSubItem("keys", value["keys"])
            d.putSubItem("parents", value["parents"])
            d.putSubItem("numChildren", value["numChildren"])
            d.putSubItem("parentOf", value["parentOf"])
            d.putSubItem("index", value["index"])
            d.putSubItem("container", value["container"])

def qdump__NodeL(d, value):
    qdump__LIST(d, value)

def qdump__Graph(d, value):
    p = value["p"]
    N = int(value["N"])
    s = "<%i>" % N
    d.putValue(s)
    m=N
    if m>10:
        m=10
    d.putNumChild(m+1)
    if d.isExpanded():
        with Children(d):
            for i in xrange(0, m):
                s = "(%i)" %i
                d.putSubItem(s, p.dereference())
                p += 1
            d.putSubItem("isNodeOfGraph", value["isNodeOfGraph"])
            d.putSubItem("isIndexed", value["isIndexed"])
#            d.putSubItem("p", value["p"])

def qdump__BodyL(d, value):
    qdump__LIST(d,value)

def qdump__ShapeL(d, value):
    qdump__LIST(d,value)

def qdump__JointL(d, value):
    qdump__LIST(d,value)

def qdump__ProxyL(d, value):
    qdump__LIST(d,value)

#def qdump__rai__Vector(d, value):
#    x=value["x"]
#    y=value["y"]
#    z=value["z"]
#    s = "[%g %g %g]" % (x,y,z)
#    d.putValue(s)
#    d.putNumChild(1)
#    if d.isExpanded():
#        with Children(d):
#            d.putSubItem("isZero", value["isZero"])
#            with SubItem(d, "length"):
#                d.putValue(math.sqrt(x*x+y*y+z*z))
#                d.putType("float")
#                d.putNumChild(0)

#def qdump__rai__Quaternion(d, value):
#    w=value["w"]
#    x=value["x"]
#    y=value["y"]
#    z=value["z"]
#    s = "[%d %d %d %d]" % (w,x,y,z)
#    d.putValue(s)
#    d.putNumChild(1)
#    if d.isExpanded():
#        with Children(d):
#            d.putSubItem("isZero", value["isZero"])
#            with SubItem(d, "degrees"):
#                d.putValue(360.0/math.pi*math.acos(w))
#                d.putType("float")
#                d.putNumChild(0)

def qdump__rai__Frame(d, value):
    ID = int(value["ID"])
    name = value["name"]
    s = "(%i) " % ID
    string_N = int(name["N"])
    string_p = name["p"]
    for j in xrange(0, string_N):
        s += "%c" % int(string_p.dereference())
        string_p += 1
    d.putValue(s)
    d.putNumChild(1)
    if d.isExpanded():
        with Children(d):
            d.putSubItem("ID", value["ID"])
            d.putSubItem("name", value["name"])
            d.putSubItem("parent", value["parent"])
            d.putSubItem("parentOf", value["parentOf"])
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
