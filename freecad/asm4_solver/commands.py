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


import FreeCAD as App
import FreeCADGui as Gui


from asm4_solver.solver import test_fn


class TestCommand(object):
    NAME = "test function"

    def IsActive(self):
        return True

    def Activated(self):
        App.Console.PrintMessage("Starting test function")
        result = test_fn(1, 2)
        App.Console.PrintMessage(f"Result = {result}\n")

    def GetResources(self):
        return {"MenuText": "Test Function",
                "ToolTop": "Click here to see result of 1 + 2"}
