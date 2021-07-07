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


/// This function adds the objects being constrained to the system and their corresponding
/// variables. Then it adds the corresponding equality constraints to the variables of
/// the objects.
pub fn set_up_equalities<>(
        object1: &ObjectIndices,
        object2: &ObjectIndices,
        variables: &mut Vec<Variable>,
        c_params: &HashMap<&str, f64>,
) {
    let mut k: usize;
    let mut j: usize;

    // now we need to add the indices of the equal variables
    for variable in ["x", "y", "z", "phi", "theta", "psi"].iter() {
        match c_params.get(variable) {
            Some(_) => {
                // The reference variable may also be equal to some other variable
                // and we need to have the index of the "original" variable.
                // This method ensures that all equal variables point to the same
                // solver variable.
                k = object1.get_index(variable);
                j = object2.get_index(variable);
                match variables[k].equal {
                    // Object1's variable is equal to some other variable.
                    // So we get the index of the other ( the"original") variable
                    // and put it into the object2's variable.
                    // NOTE: We assume object1's variable is enabled
                    Some(i) => {
                        variables[j].equal = Some(i);
                    },
                    // Object1's variable is the "original" variable (it is not
                    // equal to other variable). So we then add its index to
                    // object2's variable.
                    // Object1's variable may or may not be enabled, so we
                    // just simply enable it either way since we need it enabled.
                    // Then, we need to add the index of the object1's variable
                    // to the object2's variable.
                    None => {
                        variables[k].enabled = true;
                        variables[j].equal = Some(k);
                        variables[j].enabled= true;
                    },
                }
            },
            None => ()
        }
    }
}
