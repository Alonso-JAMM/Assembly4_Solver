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

mod geometry;
mod system;
use system::System;
mod system_object;

use optimization::TrustNCG;

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
fn build_constraints(
    objects: HashMap<&str, HashMap<&str, f64>>,
    constraint_names: HashMap<&str, HashMap<&str, &str>>,
    constraint_parameters: HashMap<&str, HashMap<&str, f64>>,
) {
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

            // indices of the reference and object in the SystemObject vector
            let ref_idx = *system.sys_objects_idx.get(ref_name).unwrap();
            let obj_idx = *system.sys_objects_idx.get(obj_name).unwrap();

            // Finally, add the fix constraint. Note that a Fix constraint is
            // broken into fix base and fix rotation
            let fix_base_constraint =
                constraints::FixBaseConstraint::new(
                    &mut system.sys_objects,
                    c_params,
                    obj_idx,
                    ref_idx
                );
            system
                .constraints
                .push(ConstraintType::FixBaseConstraint(fix_base_constraint));
        }
        // TODO: make a fix_rotation_constraint
        // TODO: make a lock_constraint
        if c.contains("Lock") {
            // WARNING: It is assumed that at this point any chained equality
            // constraints with some locked constraint applied to any of the
            // chained variables is already decomposed into multiple simple locked
            // constraints.
            let obj_name = object_names.get("Object").unwrap();

            let obj_params = objects.get(obj_name).unwrap();
            system.add_object(obj_name, obj_params);

            let sys_obj_idx = *system.sys_objects_idx.get(obj_name).unwrap();
            let sys_object = &mut system.sys_objects[sys_obj_idx];

            let c_params = constraint_parameters.get(c).unwrap();
            constraints::lock_constraint::set_up_locks(
                &c_params,
                sys_object,
            );
        }
        // TODO: make a equality_constraint
        if c.contains("Equality") {
            // Now we have to have a way of stating which variables are equal
            // NOTE: equal system variables point to the same solver variables. This
            // way other constraints (like fix_base and fix_rotation) will be able
            // to use any equal variable without any extra work (they would end up
            // updating the correct gradient and hessian indices). Basically,
            // equal variables are treated as only one variable.

            let obj1_name = object_names.get("Object1").unwrap();
            let obj2_name = object_names.get("Object2").unwrap();

            let obj1_params = objects.get(obj1_name).unwrap();
            let obj2_params = objects.get(obj2_name).unwrap();

            system.add_object(obj1_name, obj1_params);
            system.add_object(obj2_name, obj2_params);

            let object1_idx = *system.sys_objects_idx.get(obj1_name).unwrap();
            let object2_idx = *system.sys_objects_idx.get(obj2_name).unwrap();
            let c_params = constraint_parameters.get(c).unwrap();
            constraints::equality_constraint::set_up_equalities(
                &c_params,
                object1_idx,
                object2_idx,
                &mut system.sys_objects,
            );
        }
    }

    // Un-comment this part in order to solve the problem (it is faster than the
    // implementation in python
//         system.add_indices();
//         let x0 = system.start_position();
//
//         let mut min = TrustNCG::new();
//         min.i_max = 11;
//
//         let sol = min.minimize(&x0, &mut system);
//
//         println!("Solution succeeded?: {}, iterations: {}, function evaluations: {}, \
//         gradient evaluations: {}", sol.success, sol.iter_num, sol.f_evals, sol.f_grad_evals);
//         println!("solution x: {}", sol.x);

}
