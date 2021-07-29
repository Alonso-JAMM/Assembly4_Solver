# This library is free software; you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation; either
# version 2.1 of the License, or (at your option) any later version.
#
# This library is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public
# License along with this library; if not, write to the Free Software
# Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA


import os
from math import pi
import FreeCAD as App


def getResourcesDir():
    """Returns the directory of the icon folder"""
    return os.path.join(os.path.dirname(__file__), "Resources")


def getRotationVal(objName, axis):
    """
    Gets an axis from the Rotation placement of an object in the current file
    or from a child datum of a linked object
    """
    rot = None
    val = None
    if "." in objName:
        parent, datum = objName.split(".")
        linkedObject = App.ActiveDocument.getObject(parent).getLinkedObject()

        parentPla = App.ActiveDocument.getObject(parent).Placement
        datumPla = linkedObject.Document.getObject(datum).Placement
        absDatumPla = parentPla*datumPla
        rot = absDatumPla.Rotation.toEuler()
    else:
        rot = App.ActiveDocument.getObject(objName) \
                                .Placement.Rotation.toEuler()

    if axis == "x":
        val = rot[2] * pi/180
    elif axis == "y":
        val = rot[1] * pi/180
    elif axis == "z":
        val = rot[0] * pi/180

    # We mostly prefer positive angles
    if val < 0:
        val = 2*pi + val

    return val


def getBaseVal(objName, axis):
    """
    Gets an axis from the Base placement of an object in the current file
    or from a child datum of a linked object
    """
    val = None
    if "." in objName:
        parent, datum = objName.split(".")
        linkedObject = App.ActiveDocument.getObject(parent).getLinkedObject()

        parentPla = App.ActiveDocument.getObject(parent).Placement
        datumPla = linkedObject.Document.getObject(datum).Placement
        absDatumPla = parentPla*datumPla
        val = getattr(absDatumPla.Base, axis)
    else:
        val = getattr(App.ActiveDocument.getObject(objName)
                      .Placement.Base, axis)
    return val
