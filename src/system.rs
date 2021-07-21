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
use crate::constraints::*;
use crate::system_object::{SystemObject, VariableName as VN};
use ndarray::{Array1, Array2};

use optimization::problem::{Objective, Gradient, Hessian};

/// A Variable represents one of the six values used to determine an object in
/// 3D space. It is used internally to keep track of the placement of constrained
/// objects through the solving procedure.
#[derive(Debug)]
pub struct Variable {
    /// index of this variable in the solver array
    pub index: usize,
    /// value of the variable during iteration process
    pub value: f64,
    /// value of the variable at the start of the iteration process
    pub initial_value: f64,
    /// States whether the value of this variable is locked. If set to true,
    /// then the initial_value will be used during the iteration process without
    /// changing it.
    pub locked: bool,
    /// determines whether the variable is enabled or not. If not enabled, then
    /// this variable will be ignored
    pub enabled: bool,
    /// contains the index of the variable that is equal to this variable or none
    /// if an equality constraint is not applied
    pub equal: Option<(usize, VN)>,
}

impl Variable {
    pub fn new() -> Variable {
        Variable {
            index: 0,
            value: 0.0,
            initial_value: 0.0,
            locked: false,
            enabled: false,
            equal: None,
        }
    }
}


/// Represents the entire system. This struct contains all the variables, objects,
/// and constraints in the system.
#[derive(Debug)]
pub struct System<'a> {
    /// Contains all the constraints in the system. When evaluating the objective
    /// function we are evaluating all the constraints of this vector.
    pub constraints: Vec<ConstraintType>,
    /// Contains all the objects in the system
    pub sys_objects: Vec<SystemObject>,
    /// Contains the indices of the system objects in sys_objects
    pub sys_objects_idx: HashMap<&'a str, usize>,
}


impl<'a> System<'a> {
    pub fn new() -> System<'a> {
        System {
            constraints: Vec::new(),
            sys_objects: Vec::new(),
            sys_objects_idx: HashMap::new(),
        }
    }

    /// Adds a new to the system. If new_object already exists, then nothing will
    /// be done. It also adds 6 new variables to the system since these variables
    /// represent the placement of the new_object.
    pub fn add_object(
            &mut self,
            new_object_name: &'a str,
            object_params: &HashMap<&str, f64>,
    ) {
        match self.sys_objects_idx.get(new_object_name) {
            None => {
                let mut new_object = SystemObject::new();

                // initial value of each variable
                let mut x: f64;
                let var_names_str = ["x", "y", "z", "phi", "theta", "psi"];

                for (var_name_str, var_name) in var_names_str.iter().zip(VN::get_variable_iter()) {
                    let mut new_var = new_object.get_mut_variable(var_name);
                    x = *object_params.get(var_name_str).unwrap();
                    new_var.value = x;
                    new_var.initial_value = x;
                }
                self.sys_objects.push(new_object);
                // object index in the system object HashMap
                let n = self.sys_objects_idx.len();
                self.sys_objects_idx.insert(new_object_name, n);
            },
            Some(_) => ()
        }
    }


    /// Adds indices to the enabled variables in the system
    pub fn add_indices(&mut self) {
        let mut i = 0;
        for obj in self.sys_objects.iter_mut() {
            for variable in &mut obj.get_variables_mut_iter() {
                if variable.enabled {
                    match variable.equal {
                        // we add indices of equal variables later
                        Some(_) => (),
                        None => {
                            // Only add indices to unlocked variable
                            if !variable.locked {
                                variable.index = i;
                                i += 1;
                            }
                        }
                    }
                }
            }
        }
        // We add indices to equal variables here because variables default with
        // and index of 0 which means that we don't know if the other variable
        // has an index of 0 or if its index is not defined yet.
        // TODO: Would using Option<usize> for indices be better than defaulting
        // to an index of 0?
        // NOTE: we use indices since we need to have both immutable and mutable
        // access to self.variables
        for i in 0..self.sys_objects.len() {
            for var_name in VN::get_variable_iter() {
                if let Some((j, j_var_name)) = self.sys_objects[i].get_variable(var_name).equal {
                    self.sys_objects[i].get_mut_variable(var_name).index = self.sys_objects[j].get_variable(j_var_name).index;
                    self.sys_objects[i].get_mut_variable(var_name).initial_value = self.sys_objects[j].get_variable(j_var_name).initial_value;
                }
            }

        }

    }

    /// Returns the number of enabled variables
    pub fn get_enabled_size(&self) -> usize {
        let mut i = 0;
        for obj in self.sys_objects.iter() {
            for variable in obj.get_variables_iter() {
                if variable.enabled {
                    i += 1;
                }
            }
        }
        i
    }

    /// Returns the starting point for the solver
    pub fn start_position(&self) -> Array1<f64> {
        let n = self.get_enabled_size();
        let mut output = Array1::zeros(n);
        for obj in self.sys_objects.iter() {
            for variable in obj.get_variables_iter() {
                if variable.enabled {
                    output[variable.index] = variable.initial_value;
                }
            }
        }
        output
    }
}


impl<'a> Objective for System<'a> {
    fn eval(&mut self) {
        for constraint in &mut self.constraints {
            constraint.evaluate(&self.sys_objects);
        }
    }

    fn eval_real(&mut self) -> f64 {
        self.eval();
        let mut value = 0.0;
        for constraint in &self.constraints {
            value += constraint.get_value();
        }
        value
    }

    fn update_x(&mut self, x: &Array1<f64>) {
        for obj in &mut self.sys_objects {
            for variable in &mut obj.get_variables_mut_iter() {
                if variable.enabled && !variable.locked {
                    variable.value = x[variable.index];
                }
            }
            if obj.q_enable {
                obj.update_q();
            }
            if obj.v_enable {
                obj.update_v();
            }
        }
    }

    fn move_step(&mut self, _x: &Array1<f64>, _p: &Array1<f64>, _alpha: f64) {
        // we dont need this function, (at least for TrustNCG)
        // If we implement an optimization method that requires to calculate the step
        // then we need a method of updating the step. and to take partial derivatives
        // with respect to alpha. Note that we could simply add a new method to the constraint
        // functions that updates the internal variables by adding the corresponding step
        // to the variables since the system itself doesn't know about the number system used
    }
}

impl<'a> Gradient for System<'a> {
    fn grad(&mut self, output: &mut Array1<f64>) {
        // HACK: This should be done in the library
        output.fill(0.0);
        for constraint in &mut self.constraints {
            constraint.get_gradient(output, &self.sys_objects);
        }
    }

    fn diff(&mut self) -> f64 {
        0.0
    }
}


impl<'a> Hessian for System<'a> {
    fn hess(&mut self, output: &mut Array2<f64>) {
        // HACK: This should be done in the library
        output.fill(0.0);
        for constraint in &mut self.constraints {
            constraint.get_hessian(output, &self.sys_objects)
        }

    }

}
