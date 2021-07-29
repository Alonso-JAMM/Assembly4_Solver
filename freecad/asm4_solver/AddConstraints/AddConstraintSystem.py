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
import networkx as nx
import FreeCAD as App
from ..features import getResourcesDir, getRotationVal, getBaseVal


class ConstraintSystemCmd:
    def GetResources(self):
        return {
            "MenuText": "Add system object",
            "ToolTip": "Creates a system object into the assembly",
            "Pixmap": os.path.join(getResourcesDir(),
                                   "Assembly_Assembly_Create_New.svg")
        }

    def IsActive(self):
        constraintObject = App.ActiveDocument.getObject(ConstraintSystem.name)
        if (App.ActiveDocument and constraintObject is None):
            return (True)
        else:
            return (False)

    def Activated(self):
        newConstraintSystem = App.ActiveDocument.addObject(
            "App::FeaturePython", ConstraintSystem.name)
        ConstraintSystem(newConstraintSystem)
        App.ActiveDocument.recompute()


class ConstraintSystem:
    name = "Constraint_System"

    def __init__(self, obj):
        obj.addProperty("App::PropertyString", "Type", "", "", 1)
        obj.Type = ConstraintSystem.name
        obj.addProperty("App::PropertyPythonObject", "System_Data")
        obj.Proxy = self
        App.ActiveDocument.Constraints.addObject(obj)

    @staticmethod
    def getSystemObject():
        """
        Returns the system object inside this assembly
        """
        systemObject = App.ActiveDocument.getObject(ConstraintSystem.name)
        if systemObject:
            return systemObject
        else:
            print("No system object in assembly")

    @staticmethod
    def updateSystem():
        system = ConstraintSystem.getSystemObject()
        if system is None:
            return
        systemGraph = nx.MultiGraph()
        for f in App.ActiveDocument.Constraints.Group:
            if f.Name == ConstraintSystem.name:
                continue
            u = None
            v = None
            label = f.Name
            weight = f.reduced_DoF
            constraintData = f.Parameters
            constraintType = f.Type
            if f.Type == "Lock_Constraint":
                u = f.Object
                v = "LOCK_NODE"
            systemGraph.add_edge(u, v, weight=weight, label=label,
                                 parameters=constraintData,
                                 constraintType=constraintType)

        system.System_Data = nx.to_dict_of_dicts(systemGraph)
        # ConstraintSystem.addLinkedObject()

    # @staticmethod
    # def addLinkedObject():
        # """
        # Adds a node representing the linked objects that have datums
        # constrained in the assembly
        # """
        # system = ConstraintSystem.getSystemObject()
        # if system is None:
            # return
        # systemGraph = nx.from_dict_of_dicts(system.System_Data,
                                            # multigraph_input=True,
                                            # create_using=nx.MultiGraph)
        # for v in list(systemGraph.nodes()):
            # if "." in v:
                # parentName, datumName = v.split(".")
                # if parentName not in systemGraph.nodes():
                    # label = "Fix_Constraint_" + v
                    # constraintType = "Virtual_Fix_Constraint"
                    # components = FixComponents(parentName, v)
                    # systemGraph.add_edge(v, parentName, weight=6,
                                         # components=components, label=label,
                                         # constraintType=constraintType)
                    # system.System_Data = nx.to_dict_of_dicts(systemGraph)

    @staticmethod
    def getObjects():
        """
        Returns a dictionary containing all the names of all objects in the
        system with their position and rotation values
        """
        system = ConstraintSystem.getSystemObject()
        if system is None:
            return
        systemGraph = nx.from_dict_of_dicts(system.System_Data)
        objects = {}
        for objName in systemGraph.nodes:
            if objName == "LOCK_NODE":
                continue
            objects[objName] = {}
            objects[objName]["x"] = getBaseVal(objName, "x")
            objects[objName]["y"] = getBaseVal(objName, "y")
            objects[objName]["z"] = getBaseVal(objName, "z")
            objects[objName]["phi"] = getRotationVal(objName, "x")
            objects[objName]["theta"] = getRotationVal(objName, "y")
            objects[objName]["psi"] = getRotationVal(objName, "z")
        return objects

    @staticmethod
    def getConstraintNames():
        """
        Returns a dictionary containing all the names of of the constraints in
        the system and the respective objects they constraint
        """
        system = ConstraintSystem.getSystemObject()
        if system is None:
            return
        systemGraph = nx.from_dict_of_dicts(system.System_Data,
                                            multigraph_input=True,
                                            create_using=nx.MultiGraph)
        constraintNames = {}
        for obj1Name, obj2Name, data in systemGraph.edges(data=True):
            if data["constraintType"] == "Lock_Constraint":
                fName = data["label"]
                objName = None
                if obj1Name == "LOCK_NODE":
                    objName = obj2Name
                else:
                    objName = obj1Name
                constraintNames[fName] = {"Object": objName}
        return constraintNames

    @staticmethod
    def getConstraintParameters():
        """
        Returns a dictionary containing all the parameters of the constraints
        in the system.
        """
        system = ConstraintSystem.getSystemObject()
        if system is None:
            return
        systemGraph = nx.from_dict_of_dicts(system.System_Data,
                                            multigraph_input=True,
                                            create_using=nx.MultiGraph)
        constraintParameters = {}
        for _, _, data in systemGraph.edges(data=True):
            if data["constraintType"] == "Lock_Constraint":
                fName = data["label"]
                constraintParameters[fName] = data["parameters"]

        return constraintParameters
