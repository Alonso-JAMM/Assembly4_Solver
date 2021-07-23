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

use crate::system_object::{SystemObject, VariableName as VN};

/// This function adds the lock constraints to the variables being locked.
pub fn set_up_locks(
        c_params: &HashMap<&str, f64>,
        sys_object: &mut SystemObject,
) {
    let mut locked_variables: Vec<&str> = Vec::new();
    let var_names_str = ["x", "y", "z", "phi", "theta", "psi"];

    for (variable_str, variable_name) in var_names_str.iter().zip(VN::get_variable_iter()) {
        match c_params.get(variable_str) {
            Some(value) => {
                locked_variables.push(variable_str);
                sys_object.get_mut_variable(variable_name).value = *value;
            },
            None => ()
        }
    }
    sys_object.lock_variables(&locked_variables);
    sys_object.enable_variables(&locked_variables);

    // WARNING: we are enabling both the rotation quaternion and position vector
    // of the object, in some cases we should not enable them (it may slow things down
    // by making unnecessary updates to the quaternion and the vector)
    sys_object.q_enable = true;
    sys_object.v_enable = true;
}
