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
from ..features import getResourcesDir


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
