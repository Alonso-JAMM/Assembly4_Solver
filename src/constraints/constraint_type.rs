// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.
//
// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public
// License along with this library; if not, write to the Free Software
// Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA


use ndarray::{Array1, Array2};
use crate::constraints::{Constraint, fix_base_constraint};
use crate::system_object::SystemObject;

// Used to group all types of constraints so they can be used in a single vector
#[derive(Debug)]
pub enum ConstraintType {
    FixBaseConstraint(fix_base_constraint::FixBaseConstraint),
}

impl ConstraintType {
    pub fn evaluate(
            &mut self,
            sys_objects: &Vec<SystemObject>
    ) {
        match self {
            Self::FixBaseConstraint(fix) => fix.evaluate(sys_objects),
        }
    }

    pub fn get_value(&self) -> f64 {
        match self {
            Self::FixBaseConstraint(fix) => fix.get_value()
        }
    }

    pub fn get_gradient(
            &self,
            sys_grad: &mut Array1<f64>,
            sys_objects: &Vec<SystemObject>,
    ) {
        match self {
            Self::FixBaseConstraint(fix) => fix.get_gradient(sys_grad, sys_objects)
        }
    }

    pub fn get_diff(
            &mut self,
    ) -> f64 {
        match self {
            Self::FixBaseConstraint(fix) => fix.get_diff()
        }
    }

    pub fn get_hessian(
            &self,
            sys_hess: &mut Array2<f64>,
            sys_objects: &Vec<SystemObject>,
    ) {
        match self {
            Self::FixBaseConstraint(fix) => fix.get_hessian(sys_hess, sys_objects)
        }
    }
}
