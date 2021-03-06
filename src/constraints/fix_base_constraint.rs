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

use crate::system::Variable;
use crate::system_object::{SystemObject, VariableName as VN};
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

/// Fixes the 3D position (no rotation) of one object with respect to another
///
/// Calculates f(x)^2 where f(x) represents the constraint function. Internally
/// the constraint function is represented by a function phi(y) where y is a
/// vector of only the variables used by this constraint whereas x is a vector
/// of all variables in the constraint system. This terminology is taken from
/// "Numerical Optimization" second edition written by Jorge Nocedal and Stephen
/// J. Wright from chapter 7.4 (Partially separable functions).
///
/// NOTE: the indices of the local variables used in this constraint are the following:
/// 0 -> object.x
/// 1 -> object.y
/// 2 -> object.z
/// 3 -> reference.x
/// 4 -> reference.y
/// 5 -> reference.z
/// 6 -> reference.phi
/// 7 -> reference.theta
/// 8 -> reference.psi
#[derive(Debug)]
pub struct FixBaseConstraint {
    /// value of phi(y)^2
    value: f64,
    /// gradient vector of phi(y)^2
    grad: [f64; 9],
    /// hessian matrix of phi(y)^2
    hess: [[f64; 9]; 9],
    /// Fix constraint values for the 3 position axis. These values represent
    /// "how far away" we are fixing the object with respect to the local coordinate
    /// system of the reference object.
    parameters: FixParameters,
    /// Index of the object in the vector of system objects
    obj_index: usize,
    /// Index of the reference in the vector of system objects
    ref_index: usize,
}


impl Constraint for FixBaseConstraint {

    fn evaluate(
            &mut self,
            sys_objects: &Vec<SystemObject>
    ) {
        let object = &sys_objects[self.obj_index];
        let reference = &sys_objects[self.ref_index];

        // The first 3 variables are the object variables, then the next 6 variables
        // are the reference variables so we need a way of offsetting them
        let offset = 3;

        // function evaluation
        let mut fn_eval = HDual::new();

        // vector representing the position of the object and the reference
        let mut p: HDVector;
        let mut rp: HDVector;
        // quaternion representing the rotation of the reference
        let mut rq: HDQuaternion;


        // Start with the partial derivatives with respect to only the object variables
        // The object variables are: x, y, z

        // Initially the vector and quaternion of the reference are not required
        // for the evaluation of the partial derivatives with respect to only
        // the variables of the object being fixed
        rp = reference.get_vector(None, None); // this
        rq = reference.get_quaternion(None, None); // no evaluate the reference variables
        for (i, var1) in VN::get_position_iter().enumerate() {
            // Now find the other partial derivatives with respect to the object
            // (we find the partial derivatives with respect to all the combinations
            // of x, y, z for the object)
            for (j, var2) in VN::get_position_iter().enumerate().skip(i) {
                p = object.get_vector(Some(var1), Some(var2));
                fn_eval = self.eval(object, p, rp, rq);
                self.hess[i][j] = fn_eval.e1e2;
                self.hess[j][i] = fn_eval.e1e2;
            }
            // now we add the first partial derivatives with respect to the variables
            // of the object
            self.grad[i] = fn_eval.e1;
        }

        // Now find the partial derivatives with respect to the variables of both
        // the object and the reference
        for (i, var1) in VN::get_position_iter().enumerate() {
            // the first variable is an object variable
            p = object.get_vector(Some(var1), None);
            for (j, var2) in VN::get_variable_iter().enumerate() {
                // the second variable is a reference variable
                rp = reference.get_vector(None, Some(var2));
                rq = reference.get_quaternion(None, Some(var2));
                fn_eval = self.eval(object, p, rp, rq);
                self.hess[i][j+offset] = fn_eval.e1e2;
                self.hess[j+offset][i] = fn_eval.e1e2;
            }
        }

        // Then do the partial derivatives with respect to the variables of the
        // reference object

        // The position vector for the object being fixed remain constant over
        // the evaluation of the partial derivatives with respect to the reference
        // object's variables.
        p = object.get_vector(None, None);
        for (i, var1) in VN::get_variable_iter().enumerate() {
            for (j, var2) in VN::get_variable_iter().enumerate().skip(i) {
                rp = reference.get_vector(Some(var1), Some(var2));
                rq = reference.get_quaternion(Some(var1), Some(var2));
                fn_eval = self.eval(object, p, rp, rq);
                self.hess[i+offset][j+offset] = fn_eval.e1e2;
                self.hess[j+offset][i+offset] = fn_eval.e1e2;
            }

            // now add the gradients with respect to the reference variables
            self.grad[i+offset] = fn_eval.e1;
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
            sys_objects: &Vec<SystemObject>,
    ) {
        let object = &sys_objects[self.obj_index];
        let reference = &sys_objects[self.ref_index];
        let mut var: &Variable;
        let offset = 3; // offset between object variables and reference variables
        // add the gradient values from object variables
        for (i, var_name) in VN::get_position_iter().enumerate() {
            var = object.get_variable(var_name);
            if let Some(k) = var.index {
                system_grad[k] += self.grad[i];
            }
        }
        // add the gradient values from the reference variables
        for (i, var_name) in VN::get_variable_iter().enumerate() {
            var = reference.get_variable(var_name);
            if let Some(k) = var.index {
                system_grad[k] += self.grad[i+offset];
            }
        }
     }

     fn get_diff(
            &mut self,
     ) -> f64 {
        1.0
     }

    fn get_hessian(
            &self,
            system_hess: &mut Array2<f64>,
            sys_objects: &Vec<SystemObject>,
    ) {
        // system indices of the variables
        let object = &sys_objects[self.obj_index];
        let reference = &sys_objects[self.ref_index];
        let mut variable1: &Variable;
        let mut variable2: &Variable;
        let offset = 3; // offset between object variables and reference variables

        // get the derivatives with respect to only the variables of the object to
        // be fixed
        for (i, var1) in VN::get_position_iter().enumerate() {
            variable1 = object.get_variable(var1);
            if let Some(k) = variable1.index {
                for (j, var2) in VN::get_position_iter().enumerate() {
                    variable2 = object.get_variable(var2);
                    if let Some(l) = variable2.index {
                        system_hess[[k, l]] += self.hess[i][j];
                    }
                }
            }
        }

        // Get the derivatives with respect to both the object variables and the
        // reference variables
        for (i, var1) in VN::get_position_iter().enumerate() {
            variable1 = object.get_variable(var1);
            if let Some(k) = variable1.index {
                for (j, var2) in VN::get_variable_iter().enumerate() {
                    variable2 = reference.get_variable(var2);
                    if let Some(l) = variable2.index {
                        system_hess[[k, l]] += self.hess[i][j+offset];
                        system_hess[[l, k]] += self.hess[j+offset][i];
                    }
                }
            }
        }

        // Get the derivatives with respect to only the reference variables
        for (i, var1) in VN::get_variable_iter().enumerate() {
            variable1 = reference.get_variable(var1);
            if let Some(k) = variable1.index {
                for (j, var2) in VN::get_variable_iter().enumerate() {
                    variable2 = reference.get_variable(var2);
                    if let Some(l) = variable2.index {
                        system_hess[[k, l]] += self.hess[i+offset][j+offset];
                    }
                }
            }
        }
    }
}


impl FixBaseConstraint {
    pub fn new(
        system_objects: &mut Vec<SystemObject>,
        constraint_parameters: &HashMap<&str, f64>,
        obj_index: usize,
        ref_index: usize,
    ) -> FixBaseConstraint {
        // Enable the position variables for both the reference and the object being fixed
        // and the 3 rotation variables of the reference. It is assumed that at this point
        // that at least one of the 3 position variables is enabled (otherwise we wouldn't
        // be creating this constraint).
        // Note that the enabled position variables may vary between 1 and 3 (for each
        // object). On the other hand, all of the 3 rotation variables of the reference will
        // always be enabled.
        // Also note that the variables are enabled in the vector of variables of the
        // system
        {
            let sys_object = &mut system_objects[obj_index];
            sys_object.enable_variables_from_params(constraint_parameters);
            sys_object.v_enable = true;
        }
        {
            let sys_reference = &mut system_objects[ref_index];
            sys_reference.enable_variables_from_params(constraint_parameters);
            // make sure we enable the rotation angles of the reference object
            sys_reference.enable_variables(&["phi", "theta", "psi"]);
            sys_reference.v_enable = true;
            sys_reference.q_enable = true;
        }

        // Adds the "offset" values used in the constraint function. Note that
        // the parameters of the disabled axes will be set to a value of 0.
        // However, these values will not be used when evaluating the constraint
        // function.
        let mut parameters = FixParameters::new();
        add_parameters(&mut parameters, constraint_parameters);

        FixBaseConstraint {
            value: 0.0,
            grad: [0.0; 9],
            hess: [[0.0; 9]; 9],
            parameters,
            obj_index,
            ref_index,
        }
    }

    /// This is the actual constraint function error. It is intended to be called
    /// by the method evaluate() from the Constraint trait.
    fn eval(
            &self,
            object: &SystemObject,
            p: HDVector,
            rp: HDVector,
            rq: HDQuaternion,
    ) -> HDual {
        let obj_px_enabled = object.get_variable(VN::x).enabled;
        let obj_py_enabled = object.get_variable(VN::y).enabled;
        let obj_pz_enabled = object.get_variable(VN::z).enabled;

        let f_base = self.get_f_base(obj_px_enabled, obj_py_enabled, obj_pz_enabled, &p);

        let v = p - rp;

        let base_eval = rq.inv().mul_vec(&v) - f_base;

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

    /// Gets the vector f_base used in evaluating the constraint function.
    /// p is the position vector of the fixed object
    fn get_f_base(
            &self,
            obj_px_enabled: bool,
            obj_py_enabled: bool,
            obj_pz_enabled: bool,
            p: &HDVector,
    ) -> HDVector {
        let mut f_base = HDVector::new();
        if obj_px_enabled {
            f_base.x.re = self.parameters.x;
        }
        else {
            f_base.x = p.x;
        }
        if obj_py_enabled {
            f_base.y.re = self.parameters.y;
        }
        else {
            f_base.y = p.y;
        }
        if obj_pz_enabled {
            f_base.z.re = self.parameters.z;
        }
        else {
            f_base.z = p.z;
        }
        f_base

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
