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


use std::collections::HashMap;

use crate::system::{Variable, ObjectIndices};


/// This function adds the lock constraints to the variables being locked.
pub fn set_up_locks(
        object: &ObjectIndices,
        variables: &mut Vec<Variable>,
        c_params: &HashMap<&str, f64>,
) {
    let mut k: usize;

    for variable in ["x", "y", "z", "phi", "theta", "psi"].iter() {
        match c_params.get(variable) {
            Some(_) => {
                // We want to lock this variable
                k = object.get_index(variable);
                variables[k].locked = true;
                // WARNING: do we need to enable the variable if it is going to be
                // constant anyways?
                variables[k].enabled = true;
            },
            None => ()
        }
    }
}
