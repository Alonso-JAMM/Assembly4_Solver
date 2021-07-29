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
import time
from math import pi
import FreeCAD as App
from asm4_solver.solver import solve_constraint_system
from ..features import getResourcesDir
from .ConstraintSystem import ConstraintSystem as CS


class SolveSystemCmd:
    def GetResources(self):
        return {
            "MenuText": "Add system object",
            "ToolTip": "Creates a system object into the assembly",
            "Pixmap": os.path.join(getResourcesDir(), "Edit_OK.svg")
        }

    def IsActive(self):
        constraintObject = App.ActiveDocument.getObject(CS.name)
        if (App.ActiveDocument and constraintObject is not None):
            return (True)
        else:
            return (False)

    def Activated(self):
        t = time.time()
        App.Console.PrintMessage("Solving the system...")
        CS.updateSystem()
        objects = CS.getObjects()
        constraintNames = CS.getConstraintNames()
        constraintParams = CS.getConstraintParameters()
        new_objects, success = solve_constraint_system(objects,
                                                       constraintNames,
                                                       constraintParams)
        if not success:
            App.Console.PrintError("Couldn't solve the system!")
            return

        for objName, new_vals in new_objects.items():
            obj = App.ActiveDocument.getObject(objName)
            obj.Placement.Base.x = new_vals["x"]
            obj.Placement.Base.y = new_vals["y"]
            obj.Placement.Base.z = new_vals["z"]
            newRotation = App.Rotation(new_vals["psi"]*180/pi,
                                       new_vals["theta"]*180/pi,
                                       new_vals["phi"]*180/pi)
            obj.Placement.Rotation = newRotation
        App.Console.PrintMessage("Solved the system successfully!")
        App.ActiveDocument.recompute()
        timeUsed = time.time() - t
        print(f"solver took {timeUsed}")
