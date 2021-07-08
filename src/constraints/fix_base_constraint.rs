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

use ndarray::{Array1, Array2};

use optimization::geometry::{HDQuaternion, HDVector};
use optimization::number_system::HyperDualScalar as HDual;

use crate::system::{Variable, System, ObjectIndices};
use crate::constraints::Constraint;


/// The values to fix the 3 axis of the object relative to the reference object
#[derive(Debug)]
struct FixParameters {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl FixParameters {
    pub fn new() -> FixParameters {
        FixParameters {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        }
    }

    /// Adds value to the parameters
    pub fn set_parameter(&mut self, variable: &str, value: f64) {
        match variable {
            "x" => self.x = value,
            "y" => self.y = value,
            "z" => self.z = value,
            _ => ()
        }
    }
}


/// Contains the system and local indices of each variable.
#[derive(Debug)]
struct VariableIndex {
    pub sys: usize,
    pub local: usize,
}

impl VariableIndex {
    pub fn new() -> VariableIndex {
        VariableIndex {
            sys: 0,
            local: 0,
        }
    }
}


/// The system and internal indices of the variables are stored here in order to
/// know exactly which variable in the variable vector represents each variable in
/// in the constraint function.
#[derive(Debug)]
struct Object {
    pub x: VariableIndex,
    pub y: VariableIndex,
    pub z: VariableIndex,
    pub phi: VariableIndex,
    pub theta: VariableIndex,
    pub psi: VariableIndex,
}

impl Object {
    pub fn new() -> Object {
        Object {
            x: VariableIndex::new(),
            y: VariableIndex::new(),
            z: VariableIndex::new(),
            phi: VariableIndex::new(),
            theta: VariableIndex::new(),
            psi: VariableIndex::new(),
        }
    }

    pub fn get_variable(&self, variable: &str) -> &VariableIndex {
        match variable {
            "x" => &self.x,
            "y" => &self.y,
            "z" => &self.z,
            "phi" => &self.phi,
            "theta" => &self.theta,
            "psi" => &self.psi,
            _ => unreachable!(),
        }
    }

    pub fn get_mut_variable(&mut self, variable: &str) -> &mut VariableIndex {
        match variable {
            "x" => &mut self.x,
            "y" => &mut self.y,
            "z" => &mut self.z,
            "phi" => &mut self.phi,
            "theta" => &mut self.theta,
            "psi" => &mut self.psi,
            _ => unreachable!(),
        }
    }
}


/// Fixes the 3D position (no rotation) of one object with respect to another
///
/// Calculates f(x)^2 where f(x) represents the constraint function. Internally
/// the constraint function is represented by a function phi(y) where y is a
/// vector of only the variables used by this constraint whereas x is a vector
/// of all variables in the constraint system. This terminology is taken from
/// "Numerical Optimization" second edition written by Jorge Nocedal and Stephen
/// J. Wright from chapter 7.4 (Partially separable functions).
#[derive(Debug)]
pub struct FixBaseConstraint {
    /// value of phi(y)^2
    value: f64,
    /// gradient vector of phi(y)^2
    grad: Array1<f64>,
    /// hessian matrix of phi(y)^2
    hess: Array2<f64>,
    /// system variables indices of the internal variables. These are the
    /// indices of the variables in the system variable vector. The size of this
    /// vector may change depending on the amount of axis enabled.
    index_list: Vec<usize>,
    /// Fix constraint values for the 3 position axis. These values represent
    /// "how far away" we are fixing the object with respect to the local coordinate
    /// system of the reference object.
    parameters: FixParameters,
    /// fixed object indices. They are used to make the vectors and
    /// quaternions needed for calculating the error of this constraint.
    /// Note that we only require the x, y, and z components of obj_indices since
    /// we are only fixing the 3 position axes.
    object_indices: Object,
    /// reference object indices. They are used to make the vectors and
    /// quaternions needed for calculating the error of this constraint
    reference_indices: Object,
    // contains the variables used to evaluate the constraint function. These values
    // are going to be used to evaluate the constraint function.
    variables: Vec<HDual>,
}


impl Constraint for FixBaseConstraint {

    fn evaluate(&mut self, sys_variables: &Vec<Variable>) {
        // sys_variables is already updated. However, the local variables are
        // not updated yet.
        self.update_variables(sys_variables);

        // now we can proceed and fill the hessian, the gradient, and the
        // constraint function value
        let mut fn_eval = HDual::new();
        let mut var: &Variable;
        for i in 0..self.index_list.len() {
            // we only need to evaluate the upper half of the hessian matrix
            for j in i..self.index_list.len() {
                var = &sys_variables[self.index_list[i]];
                // Only work with enabled and unlocked variables
                if var.enabled && !var.locked {
                    self.variables[i].e1 = 1.0;
                }
                var = &sys_variables[self.index_list[j]];
                if var.enabled && !var.locked {
                    self.variables[j].e2 = 1.0;
                }

                // here we set the hessian matrix
                fn_eval = self.eval(sys_variables);
                self.hess[[i,j]] = fn_eval.e1e2;
                self.hess[[j,i]] = fn_eval.e1e2;

                self.variables[i].e1 = 0.0;
                self.variables[j].e2 = 0.0;
            }
            // here we set the gradient (equivalent of evaluating the function n times)
            // we get this evaluation "free" from evaluating the hessian.
            self.grad[i] = fn_eval.e1;
        }
        // All evaluations give the constraint function error but we only need
        // to assign it once to the value field.
        self.value = fn_eval.re;
    }

     fn get_value(&self) -> f64 {
        self.value
     }

     fn get_gradient(
            &self,
            system_grad: &mut Array1<f64>,
            sys_variables: &Vec<Variable>
    ) {
        let mut k: usize;    // variable index
        let mut ks: usize;   // solver variable index
        let mut var: &Variable;
        for i in 0..self.index_list.len() {
            k = self.index_list[i];
            var = &sys_variables[k];
            ks = var.index;
            if var.enabled && !var.locked {
                system_grad[ks] += self.grad[i];
            }
        }
     }

     fn get_diff(
            &mut self,
            sys_variables: &Vec<Variable>,
     ) -> f64 {
        self.eval(sys_variables).e1
     }

    fn get_hessian(
            &self,
            system_hess: &mut Array2<f64>,
            sys_variables: &Vec<Variable>
    ) {
        // system indices of the variables
        let mut k: usize;
        let mut ks: usize;  // solver variable index
        let mut l: usize;
        let mut ls: usize;  // solver variable index
        let mut var_k: &Variable;
        let mut var_l: &Variable;
        for i in 0..self.index_list.len() {
            for j in 0..self.index_list.len() {
                k = self.index_list[i];
                var_k = &sys_variables[k];
                ks = var_k.index;
                l = self.index_list[j];
                var_l = &sys_variables[l];
                ls = var_l.index;
                if (var_k.enabled && !var_k.locked) && (var_l.enabled && !var_l.locked) {
                    system_hess[[ks, ls]] += self.hess[[i,j]];
                }
            }
        }
    }
}


impl FixBaseConstraint {
    pub fn new(
        system: &mut System,
        obj_name: &str,
        ref_name: &str,
        constraint_parameters: &HashMap<&str, f64>
    ) -> FixBaseConstraint {

        // Assuming that we already added the object and reference on the system
        let object = system.objects.get(obj_name).unwrap();
        let reference = system.objects.get(ref_name).unwrap();

        // Enable the position variables for both the reference and the object being fixed
        // and the 3 rotation variables of the reference. It is assumed that at this point
        // that at least one of the 3 position variables is enabled (otherwise we wouldn't
        // be creating this constraint).
        // Note that the enabled position variables may vary between 1 and 3 (for each
        // object). On the other hand, all of the 3 rotation variables of the reference will
        // always be enabled.
        // Also note that the variables are enabled in the vector of variables of the
        // system
        enable_position_variables(object, constraint_parameters, &mut system.variables);
        enable_position_variables(reference, constraint_parameters, &mut system.variables);
        enable_rotation_variables(reference, &mut system.variables);

        // Add the position and rotation variables to the object and reference
        // indices. Here we add all the indices of all the variables even the
        // disabled variables since we need to know their values when evaluating
        // the constraint function
        let mut object_indices = Object::new();
        let mut reference_indices = Object::new();
        let mut index_list = Vec::new();
        add_position_variables(object, &mut index_list, &mut object_indices);
        add_position_variables(reference, &mut index_list, &mut reference_indices);
        add_rotation_variables(reference, &mut index_list, &mut reference_indices);

        // Adds the "offset" values used in the constraint function. Note that
        // the parameters of the disabled axes will be set to a value of 0.
        // However, these values will not be used when evaluating the constraint
        // function.
        let mut parameters = FixParameters::new();
        add_parameters(&mut parameters, constraint_parameters);

        // variables that will be used to evaluate the constraint function
        let variables = vec![HDual::new(); 9];

        // TODO: is there a method of only enabling the needed rotation variables?
        //      For example if a LCS is constrained on the x-axis of another LCS,
        //      then rotation about the x-axis of the reference LCS is not required.
        //      However, this only works when the LCS is constrained exactly on the
        //      x-axis of the reference. Otherwise, the rotation about the x-axis of
        //      the reference LCS is required.
        FixBaseConstraint {
            value: 0.0,
            grad: Array1::zeros(9),
            hess: Array2::zeros((9,9)),
            index_list,
            parameters,
            object_indices,
            reference_indices,
            variables,
        }
    }

    /// Updates the local variables with the new values of the system variables
    fn update_variables(&mut self, sys_variables: &Vec<Variable>) {
        for (k_local, k_sys) in self.index_list.iter().enumerate() {
            self.variables[k_local].re = sys_variables[*k_sys].value;
        }
    }

    /// This is the actual constraint function error. It is intended to be called
    /// by the method evaluate() from the Constraint trait. This constraint function
    /// doesn't care about the partial derivatives of the hyperdual scalar numbers.
    fn eval(&self, sys_variables: &Vec<Variable>) -> HDual {
        let obj_px_enabled = sys_variables[self.object_indices.x.sys].enabled;
        let obj_py_enabled = sys_variables[self.object_indices.y.sys].enabled;
        let obj_pz_enabled = sys_variables[self.object_indices.z.sys].enabled;

        let p = self.get_obj();
        let rp = self.get_ref();
        let ref_rot = self.get_ref_rot().inv();

        let f_base = self.get_f_base(obj_px_enabled, obj_py_enabled, obj_pz_enabled);

        let v = p - rp;

        let base_eval = ref_rot.mul_vec(&v) - f_base;

        let mut result = HDual::new();
        //TODO: addasign operator
        if obj_px_enabled {
            result = result + base_eval.x.powi(2);
        }
        if obj_py_enabled{
            result = result + base_eval.y.powi(2);
        }
        if obj_pz_enabled{
            result = result + base_eval.z.powi(2);
        }
        result
    }

    /// Gets the vector f_base used in evaluating the constraint function
    fn get_f_base(
            &self,
            obj_px_enabled: bool,
            obj_py_enabled: bool,
            obj_pz_enabled: bool,
    ) -> HDVector {
        let mut f_base = HDVector::new();
        if obj_px_enabled {
            f_base.x.re = self.parameters.x;
        }
        else {
            f_base.x = self.variables[self.object_indices.x.local];
        }
        if obj_py_enabled {
            f_base.y.re = self.parameters.y;
        }
        else {
            f_base.y = self.variables[self.object_indices.y.local];
        }
        if obj_pz_enabled {
            f_base.z.re = self.parameters.z;
        }
        else {
            f_base.z = self.variables[self.object_indices.z.local];
        }
        f_base
    }

    /// Gets the vector representing the object to be fixed
    fn get_obj(&self) -> HDVector {
        HDVector {
            x: self.variables[self.object_indices.x.local],
            y: self.variables[self.object_indices.y.local],
            z: self.variables[self.object_indices.z.local],
        }
    }

    /// Gets the vector representing the reference object
    fn get_ref(&self) -> HDVector {
        HDVector {
            x: self.variables[self.reference_indices.x.local],
            y: self.variables[self.reference_indices.y.local],
            z: self.variables[self.reference_indices.z.local],
        }
    }

    /// Gets the quaternion that represents the rotation of the reference object
    fn get_ref_rot(&self) -> HDQuaternion {
        let ref_phi = self.variables[self.reference_indices.phi.local];
        let ref_theta = self.variables[self.reference_indices.theta.local];
        let ref_psi = self.variables[self.reference_indices.psi.local];
        HDQuaternion::from_angles(ref_phi, ref_theta, ref_psi)
    }
}


/// Fills the parameters of the fix constraint
fn add_parameters(
        parameters: &mut FixParameters,
        constraint_parameters: &HashMap<&str, f64>,
) {
    for variable in ["x", "y", "z"].iter() {
        match constraint_parameters.get(variable) {
            Some(value) => parameters.set_parameter(variable, *value),
            None => ()
        }
    }
}


/// Adds the phi, theta, psi variables to the indices
/// Note that we only add these variables to the reference object.
fn add_rotation_variables(
        object: &ObjectIndices,
        index_list: &mut Vec<usize>,
        object_indices: &mut Object,
) {
    let mut k: usize;
    let mut n: usize = index_list.len();
    let mut object_variable: &mut VariableIndex;
    for variable in ["phi", "theta", "psi"].iter() {
        k = object.get_index(variable);
        index_list.push(k);
        object_variable = object_indices.get_mut_variable(variable);
        object_variable.sys = k;
        object_variable.local = n;
        n += 1;
    }
}


/// Adds the x, y, and z variables to the indices.
fn add_position_variables(
        object: &ObjectIndices,
        index_list: &mut Vec<usize>,
        object_indices: &mut Object,
) {
    let mut k: usize;
    let mut n: usize = index_list.len();
    let mut object_variable: &mut VariableIndex;
    for variable in ["x", "y", "z"].iter() {
        k = object.get_index(variable);
        index_list.push(k);
        object_variable = object_indices.get_mut_variable(variable);
        object_variable.sys = k;
        object_variable.local = n;
        n += 1;
    }
}


/// Enables the position variables that are constrained by this constraint function
fn enable_position_variables(
        object: &ObjectIndices,
        constraint_parameters: &HashMap<&str, f64>,
        variables: &mut Vec<Variable>,
) {
    let mut k: usize;
    // The position variables may or may not be enabled.
    for variable in ["x", "y", "z"].iter() {
        match constraint_parameters.get(variable) {
            Some(_) => {
                k = object.get_index(variable);
                variables[k].enabled = true;
            },
            None => ()

        }
    }
}


/// Enables the rotation variables that are constrained by this constraint function
fn enable_rotation_variables(
        reference: &ObjectIndices,
        variables: &mut Vec<Variable>,
) {
    let mut k: usize;
    // The rotation variables will always be enabled.
    for variable in ["phi", "theta", "psi"].iter() {
        k = reference.get_index(variable);
        variables[k].enabled = true;
    }
}
