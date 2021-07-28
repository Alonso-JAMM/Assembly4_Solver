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
import FreeCADGui as Gui
import FreeCAD as App


class Assembly4Solver(Gui.Workbench):
    """
    Solver Workbench for Assembly4
    """

    MenuText = "Assembly 4 Solver"
    ToolTip = "Adds and solves constraints for Assembly 4"
    commands = [
        "LockConstraintCmd",
        "ConstraintSystemCmd",
    ]

    def GetClassName(self):
        return "Gui::PythonWorkbench"

    def Initialize(self):
        App.Console.PrintMessage("Starting Assembly4 Solver")
        from .AddConstraints.AddLockConstraint import LockConstraintCmd
        from .AddConstraints.AddConstraintSystem import ConstraintSystemCmd

        self.appendToolbar("asm4_solver", self.commands)
        self.appendMenu("asm4_solver", self.commands)

        Gui.addCommand("LockConstraintCmd", LockConstraintCmd())
        Gui.addCommand("ConstraintSystemCmd", ConstraintSystemCmd())

    def Activated(self):
        pass

    def Deactivated(self):
        pass


Gui.addWorkbench(Assembly4Solver())
