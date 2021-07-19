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

use crate::system_object::{SystemObject, VariableIndex};


/// This function adds the objects being constrained to the system and their corresponding
/// variables. Then it adds the corresponding equality constraints to the variables of
/// the objects.
pub fn set_up_equalities<>(
        c_params: &HashMap<&str, f64>,
        sys_object1_idx: usize,
        sys_object2_idx: usize,
        sys_objects: &mut Vec<SystemObject>,
) {
    let mut var_idx: VariableIndex;
    let mut equal_variables: Vec<&str> = Vec::new();
    let mut equal_indices: Vec<(&str, (usize, VariableIndex))> = Vec::new();
    // now we add the indices of the equal variables
    // NOTE: we assume that there are not chained equality constraints (they should
    // be removed by the constraint front-end)
    for variable in ["x", "y", "z", "phi", "theta", "psi"].iter() {
        match c_params.get(variable) {
            Some(_) => {
                var_idx = VariableIndex::get_from_str(variable);
                equal_variables.push(variable);
                equal_indices.push((variable, (sys_object1_idx, var_idx)));
            }
            None => (),
        }
    }
    sys_objects[sys_object1_idx].enable_variables(&equal_variables);
    sys_objects[sys_object2_idx].enable_variables(&equal_variables);
    sys_objects[sys_object2_idx].add_equal_indices(&equal_indices);

    // WARNING: we are enabling both rotation quaternion and position vector of the
    // system objects. This may cause unnecessary updates on the quaternion or the vector
    // we only need to enable them if one of their variables are enabled
    sys_objects[sys_object1_idx].q_enable = true;
    sys_objects[sys_object1_idx].v_enable = true;
    sys_objects[sys_object2_idx].q_enable = true;
    sys_objects[sys_object2_idx].v_enable = true;
}
