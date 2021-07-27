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
from PySide import QtGui
import FreeCAD as App
import FreeCADGui as Gui
import libAsm4 as asm4
from ..features import getResourcesDir


class LockConstraintCmd:

    def GetResources(self):
        return {
            "MenuText": "Add Lock constraint",
            "ToolTip": "Creates new Lock constraint into the assembly",
            "Pixmap": os.path.join(getResourcesDir(), "Constraint_Lock.svg")
        }

    def IsActive(self):
        if App.ActiveDocument:
            return(True)
        else:
            return(False)

    def Activated(self):
        panel = LockPanel()
        Gui.Control.showDialog(panel)


class LockPanel:
    def __init__(self):
        self.type = "Lock_Constraint"
        self.form = Gui.PySideUic.loadUi(
            os.path.join(
                getResourcesDir(),
                "ui/TaskPanel_LockConstraint.ui"))
        self.addObjects()

    def accept(self):
        objName = self.form.objectList.selectedItems()[0].text()
        qty = App.Units.Quantity
        constraint_params = {}
        if not objName:
            print("Select an object first")
            return
        if self.form.xCheck.isChecked():
            value = qty(self.form.xVal.text()).Value
            constraint_params["x"] = value
        if self.form.yCheck.isChecked():
            value = qty(self.form.yVal.text()).Value
            constraint_params["y"] = value
        if self.form.zCheck.isChecked():
            value = qty(self.form.zVal.text()).Value
            constraint_params["z"] = value
        if self.form.xrotCheck.isChecked():
            value = qty(self.form.xrotVal.text()).Value
            constraint_params["phi"] = value
        if self.form.yrotCheck.isChecked():
            value = qty(self.form.yrotVal.text()).Value
            constraint_params["theta"] = value
        if self.form.zrotCheck.isChecked():
            value = qty(self.form.zrotVal.text()).Value
            constraint_params["psi"] = value

        newConstraint = App.ActiveDocument.addObject("App::FeaturePython", self.type)
        LockConstraint(newConstraint, objName, self.type, constraint_params)
        Gui.Control.closeDialog()
        App.ActiveDocument.recompute()

    def addObjects(self):
        for obj in App.ActiveDocument.Objects:
            if obj.TypeId not in asm4.datumTypes:
                continue
            newItem = QtGui.QListWidgetItem()
            newItem.setText(obj.Name)
            newItem.setIcon(obj.ViewObject.Icon)
            self.form.objectList.addItem(newItem)


class LockConstraint:
    def __init__(self, obj, objName, constraintType, constraint_params):
        obj.Proxy = self
        obj.addProperty("App::PropertyString", "Type", "", "", 1)
        obj.Type = constraintType
        obj.addProperty("App::PropertyString", "Object")
        obj.Object = objName
        obj.addProperty("App::PropertyInteger", "reduced_DoF", "", "", 1)
        obj.addProperty("App::PropertyBool", "Base_x", "Placement")
        obj.addProperty("App::PropertyFloat", "Base_x_val", "Placement")
        obj.addProperty("App::PropertyBool", "Base_y", "Placement")
        obj.addProperty("App::PropertyFloat", "Base_y_val", "Placement")
        obj.addProperty("App::PropertyBool", "Base_z", "Placement")
        obj.addProperty("App::PropertyFloat", "Base_z_val", "Placement")
        obj.addProperty("App::PropertyBool", "Rotation_x", "Placement")
        obj.addProperty("App::PropertyFloat", "Rotation_x_val", "Placement")
        obj.addProperty("App::PropertyBool", "Rotation_y", "Placement")
        obj.addProperty("App::PropertyFloat", "Rotation_y_val", "Placement")
        obj.addProperty("App::PropertyBool", "Rotation_z", "Placement")
        obj.addProperty("App::PropertyFloat", "Rotation_z_val", "Placement")
        obj.addProperty("App::PropertyPythonObject", "Parameters", "", "", 4)
        obj.Parameters = constraint_params
        for variable in constraint_params:
            val = constraint_params[variable]
            # Name of the property to put the value
            if variable == "x":
                setattr(obj, "Base_x", True)
                setattr(obj, "Base_x_val", val)
            if variable == "y":
                setattr(obj, "Base_y", True)
                setattr(obj, "Base_y_val", val)
            if variable == "z":
                setattr(obj, "Base_z", True)
                setattr(obj, "Base_z_val", val)
            if variable == "phi":
                setattr(obj, "Rotation_x", True)
                setattr(obj, "Rotation_x_val", val)
            if variable == "theta":
                setattr(obj, "Rotation_y", True)
                setattr(obj, "Rotation_y_val", val)
            if variable == "psi":
                setattr(obj, "Rotation_z", True)
                setattr(obj, "Rotation_z_val", val)
        App.ActiveDocument.Constraints.addObject(obj)

    def onChanged(self, obj, prop):
        if prop in "Base_x_val":
            self.changeParameterValue(obj, "x", "Base_x")
        if prop in "Base_y_val":
            self.changeParameterValue(obj, "y", "Base_y")
        if prop in "Base_z_val":
            self.changeParameterValue(obj, "z", "Base_z")
        if prop in "Rotation_x_val":
            self.changeParameterValue(obj, "phi", "Rotation_x")
        if prop in ("Rotation_y", "Rotation_y_val"):
            self.changeParameterValue(obj, "theta", "Rotation_y")
        if prop in "Rotation_z_val":
            self.changeParameterValue(obj, "psi", "Rotation_z")

    @staticmethod
    def changeParameterValue(obj, param, prop):
        valueProp = prop + "_val"
        if not getattr(obj, prop):
            obj.Parameters.pop(param, None)
        else:
            obj.Parameters[param] = getattr(obj, valueProp)
        # update the reduced degrees of freedom from this constraint
        obj.reduced_DoF = len(obj.Parameters)
