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

use pyo3::prelude::*;
use pyo3::wrap_pyfunction;

mod constraints;
use constraints::ConstraintType;

mod system;
use system::System;


#[pymodule]
fn solver(_py: Python<'_>, m: &PyModule) -> PyResult<()> {
    m.add_wrapped(wrap_pyfunction!(build_constraints))?;

    Ok(())
}


/// Set-up the constraints functions
///
/// objects: map of all objects in the system with their current placement values.
///     This map is returned with the resulting values after solving the system.
/// constraint_names: map of all constraints with the name of constrained objects
/// constraint_parameters: map of all constraints parameters. For example the
///     values of the axis to lock for a Lock constraint. Axis not enabled in a
///     constraint will be omitted in this map (if a lock constraint does not
///     lock the x-axis, then it will not be included in constraint_parameters)
#[pyfunction]
fn build_constraints(objects: HashMap<&str, HashMap<&str, f64>>,
                     constraint_names: HashMap<&str, HashMap<&str, &str>>,
                     constraint_parameters: HashMap<&str, HashMap<&str, f64>>) {
    // Here we store the system information.
    let mut system = System::new();

     for (c, object_names) in &constraint_names {
        if c.contains("FixBase") {
            let obj_name = object_names.get("Object").unwrap();
            let ref_name = object_names.get("Reference").unwrap();

            let obj_params = objects.get(obj_name).unwrap();
            let ref_params = objects.get(ref_name).unwrap();

            // constraint parameters of this fix constraint
            let c_params = constraint_parameters.get(c).unwrap();

            // we add object to be fixed and the reference object to the system
            // and create variables
            system.add_object(obj_name, obj_params);
            system.add_object(ref_name, ref_params);

            // Finally, add the fix constraint. Note that a Fix constraint is
            // broken into fix base and fix rotation
            let fix_base_constraint = constraints::FixBaseConstraint::new(
                &mut system, obj_name, ref_name, c_params
            );
            system.constraints.push(ConstraintType::FixBaseConstraint(fix_base_constraint));
        }
        // TODO: make a fix_rotation_constraint
        // TODO: make a lock_constraint
        // TODO: make a equality_constraint
     }

    // At this point we have all the constraints and variables. We can proceed to
    // solve this system of constraints.
    // TODO: solve the system of constraints
    //      1. Create array of variables used by the solver (enabled variables)
    //      2. Assign a index of the solver variables to the corresponding
    //         system variables
    //      3. Construct problem type that will be given to the solver.



    // check we have all the objects
//     for (obj_name, obj_params) in &system.objects {
//         println!("{}: {:?}", obj_name, obj_params);
//     }
//     for var in system.variables.iter() {
//         println!("{:?}", var);
//     }
//
//     for constraint in system.constraints.iter_mut() {
//         constraint.evaluate(&system.variables);
//     }

        // Test this to evaluate the constraint function
//     for _ in 0..30 {
//         system.constraints[0].evaluate(&system.variables);
//     }
}

