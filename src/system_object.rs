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


use std::ops::{Index, IndexMut};
use std::collections::HashMap;

use optimization::geometry::{HDVector, HDQuaternion};
use crate::system::Variable;
use crate::geometry::{Quaternion, Vector};


/// Represents an object in the constraint system.
///
/// A SystemObject contains all data regarding the placement of an object in 3D space
/// including the variables used to represent the object. An object will be used by
/// constraint functions.
#[derive(Debug)]
pub struct SystemObject{
    // Stores the actual variables that represent the position and rotation of this
    // object in 3D space.
    pub vars: ObjectVariables,
    /// This field stores the quaternion rotation information about this object.
    /// The quaternion information is used by constraints functions in other to
    /// calculate the error.
    q_vals: Quaternion,
    /// When enabled, it means that q_vals will be updated at each iteration. If
    /// disabled, then q_vals will not be updated
    pub q_enable: bool,
    /// This field stores the position vector information about this object.
    /// The vector contains the partial derivatives with respect to the variables
    /// x, y, and z of this object.
    v_vals: Vector,
    /// When enabled, it means that v_vals will be updated at each iteration.
    pub v_enable: bool,
}


/// Stores the 6 variables of an object
#[derive(Debug)]
pub struct ObjectVariables {
    /// This variable represents the global x-axis position of this object
    pub x: Variable,
    /// This variable represents the global y-axis position of this object
    pub y: Variable,
    /// This variable represents the global z-axis position of this object
    pub z: Variable,
    /// This variable represents the global rotation angle about the x-axis of this object
    pub phi: Variable,
    /// This variable represents the global rotation angle about the y-axis of this object
    pub theta: Variable,
    /// This variable represents the global rotation angle about the z-axis of this object
    pub psi: Variable,
}


/// Object variable indices. This enum represents the indices of a variable
/// inside an Object.
#[allow(non_camel_case_types)]
#[derive(Debug, Copy, Clone)]
pub enum VariableName {
    x,
    y,
    z,
    phi,
    theta,
    psi,
}


impl SystemObject {
    pub fn new() -> SystemObject {
        SystemObject {
            vars: ObjectVariables::new(),
            q_vals: Quaternion::new(),
            q_enable: false,
            v_vals: Vector::new(),
            v_enable: false,
        }
    }

    /// Enables the variables in this object.
    ///
    /// variables: all variables names that are going to be enabled
    pub fn enable_variables(&mut self, variables: &[&str]){
        for variable in variables {
            self.vars.get_mut_variable(variable).enabled = true;
        }
    }

    /// Enables variables from a constraint parameters hashmap
    ///
    /// Many (pretty much all of them) constraint functions enable their variables
    /// from a constraint parameters hashmap where the keys are the enabled variables
    /// so this function helps to enable the variables from a hashmap in order to avoid
    /// repeating code.
    pub fn enable_variables_from_params(&mut self, c_params: &HashMap<&str, f64>) {
        for variable in c_params.keys() {
            self.vars.get_mut_variable(variable).enabled = true;
        }
    }


    /// Locks the variables in this object.
    ///
    /// variables: slice containing the names of all the variables to be locked
    /// in this object.
    pub fn lock_variables(&mut self, variables: &[&str]) {
        for variable in variables {
            self.vars.get_mut_variable(variable).locked = true;
        }
    }

    /// Adds the equality indices.
    ///
    /// equal_indices: contains all the necessary information to add the equality
    /// indices to the variables in the object. It contains the variable names and the
    /// equality indices (equality indices are of the form of (usize, VariableName))
    pub fn add_equal_indices(&mut self, equal_indices: &[(&str, (usize, VariableName))]) {
        for (variable, indices) in equal_indices {
            self.vars.get_mut_variable(variable).equal = Some(*indices);
        }
    }

    /// updates the rotation quaternion of the object
    ///
    /// NOTE: call this function after updating the object variables!
    pub fn update_q(&mut self) {
        self.q_vals.evaluate_quaternion(&self.vars.phi, &self.vars.theta, &self.vars.psi);
    }

    /// updates the position vector of the object
    ///
    /// NOTE: call this function after updating the object variables
    pub fn update_v(&mut self) {
        self.v_vals.evaluate_vector(&self.vars.x, &self.vars.y, &self.vars.z);
    }

    /// Returns the position vector with the given enabled variables.
    ///
    /// The two passed variables represent the enabled object's placement variables
    /// x, y, z, phi, theta, psi.
    pub fn get_vector(&self, var1: &str, var2: &str) -> HDVector {
        match var1 {
            x if x == "x" => match var2 {
                x if x == "x" => self.v_vals.get_x_x(),
                y if y == "y" => self.v_vals.get_x_y(),
                z if z == "z" => self.v_vals.get_x_z(),
                _ => self.v_vals.get_x_const(),
            },
            y  if y == "y" => match var2 {
                x if x == "x" => self.v_vals.get_y_x(),
                y if y == "y" => self.v_vals.get_y_y(),
                z if z == "z" => self.v_vals.get_y_z(),
                _ => self.v_vals.get_y_const(),
            },
            z if z == "z" => match var2 {
                x if x == "x" => self.v_vals.get_z_x(),
                y if y == "y" => self.v_vals.get_z_y(),
                z if z == "z" => self.v_vals.get_z_z(),
                _ => self.v_vals.get_z_const(),
            },
            _ => match var2 {
                x if x == "x" => self.v_vals.get_const_x(),
                y if y == "y" => self.v_vals.get_const_y(),
                z if z == "z" => self.v_vals.get_const_z(),
                _ => self.v_vals.get_const_const(),
            }
        }
    }

    /// Returns the rotation quaternion with the given enabled variables.
    ///
    /// The two passed variables represent the enabled object's placement variables
    /// x, y, z, phi, theta, psi. And the returning quaternion will contain the
    /// partial derivatives with respect of these two variables
    pub fn get_quaternion(&self, var1: &str, var2: &str) -> HDQuaternion {
        match var1 {
            phi if phi == "phi" => match var2 {
                phi if phi == "phi" => self.q_vals.get_phi_phi(),
                theta if theta == "theta" => self.q_vals.get_phi_theta(),
                psi if psi == "psi" => self.q_vals.get_phi_psi(),
                _ => self.q_vals.get_phi_const(),
            },
            theta if theta == "theta" => match var2 {
                phi if phi == "phi" => self.q_vals.get_theta_phi(),
                theta if theta == "theta" => self.q_vals.get_theta_theta(),
                psi if psi == "psi" => self.q_vals.get_theta_psi(),
                _ => self.q_vals.get_theta_const(),
            },
            psi if psi == "psi" => match var2 {
                phi if phi == "phi" => self.q_vals.get_psi_phi(),
                theta if theta == "theta" => self.q_vals.get_psi_theta(),
                psi if psi == "psi" => self.q_vals.get_psi_psi(),
                _ => self.q_vals.get_psi_const(),
            }
            _ => match var2 {
                phi if phi == "phi" => self.q_vals.get_const_phi(),
                theta if theta == "theta" => self.q_vals.get_const_theta(),
                psi if psi == "psi" => self.q_vals.get_const_psi(),
                _ => self.q_vals.get_const_const(),
            }
        }
    }
}


impl ObjectVariables {
    pub fn new() -> ObjectVariables {
        ObjectVariables {
            x: Variable::new(),
            y: Variable::new(),
            z: Variable::new(),
            phi: Variable::new(),
            theta: Variable::new(),
            psi: Variable::new(),
        }
    }

    pub fn iter(&self) -> ObjectVariablesIter<'_> {
        ObjectVariablesIter {
            index: 0,
            vars: self,
        }
    }

    pub fn iter_mut(&mut self) -> ObjectVariablesMutIter<'_> {
        ObjectVariablesMutIter {
            index: 0,
            vars: self,
        }
    }

    /// returns a reference to a variable by name
    pub fn get_variable(&self, var_name: &str) -> &Variable {
        match var_name {
            "x" => &self.x,
            "y" => &self.y,
            "z" => &self.z,
            "phi" => &self.phi,
            "theta" => &self.theta,
            "psi" => &self.psi,
            // we should never call something else than the previous variable names!
            _ => unreachable!(),
        }
    }

    /// returns a mutable reference to a variable by name
    pub fn get_mut_variable(&mut self, var_name: &str) -> &mut Variable {
        match var_name {
            "x" => &mut self.x,
            "y" => &mut self.y,
            "z" => &mut self.z,
            "phi" => &mut self.phi,
            "theta" => &mut self.theta,
            "psi" => &mut self.psi,
            // we should never call something else than the previous variable names!
            _ => unreachable!(),
        }
    }
}



impl Index<VariableName> for ObjectVariables {
    type Output = Variable;
    fn index(&self, idx: VariableName) -> &Self::Output {
        match idx {
            VariableName::x => &self.x,
            VariableName::y => &self.y,
            VariableName::z => &self.z,
            VariableName::phi => &self.phi,
            VariableName::theta => &self.theta,
            VariableName::psi=> &self.psi,
        }
    }
}


impl IndexMut<VariableName> for ObjectVariables {
    fn index_mut(&mut self, idx: VariableName) -> &mut Self::Output {
        match idx {
            VariableName::x => &mut self.x,
            VariableName::y => &mut self.y,
            VariableName::z => &mut self.z,
            VariableName::phi => &mut self.phi,
            VariableName::theta => &mut self.theta,
            VariableName::psi=> &mut self.psi,
        }
    }
}



pub struct ObjectVariablesIter<'a> {
    index: u8,
    vars: &'a ObjectVariables,
}

impl<'a> Iterator for ObjectVariablesIter<'a> {
    type Item = &'a Variable;
    fn next(&mut self) -> Option<Self::Item> {
        let var = match self.index {
            0 => &self.vars.x,
            1 => &self.vars.y,
            2 => &self.vars.z,
            3 => &self.vars.phi,
            4 => &self.vars.theta,
            5 => &self.vars.psi,
            _ => return None,
        };
        self.index += 1;
        Some(var)
    }
}

pub struct ObjectVariablesMutIter<'a> {
    index: u8,
    vars: &'a mut ObjectVariables,
}

impl<'a> Iterator for ObjectVariablesMutIter<'a> {
    type Item = &'a mut Variable;
    fn next(&mut self) -> Option<Self::Item> {
        let var = unsafe {
            // We update index each time next is called. This means we never
            // return the same mutable reference more than once.
            // https://stackoverflow.com/questions/61978903/how-do-i-create-mutable-iterator-over-struct-fields
            match self.index {
                0 => &mut *(&mut self.vars.x as *mut _),
                1 => &mut *(&mut self.vars.y as *mut _),
                2 => &mut *(&mut self.vars.z as *mut _),
                3 => &mut *(&mut self.vars.phi as *mut _),
                4 => &mut *(&mut self.vars.theta as *mut _),
                5 => &mut *(&mut self.vars.psi as *mut _),
                _ => return None,
            }
        };
        self.index += 1;
        Some(var)
    }
}


impl VariableName {
    /// Returns a VariableName from an input variable name str. For example
    /// if the input variable is "x" then this function will return VariableName::x
    pub fn get_from_str(variable: &str) -> VariableName {
        match variable {
            "x" => VariableName::x,
            "y" => VariableName::y,
            "z" => VariableName::z,
            "phi" => VariableName::phi,
            "theta" => VariableName::theta,
            "psi" => VariableName::psi,
            _ => unreachable!(),
        }
    }

    /// Returns an iterator over all the different options of VariableName
    pub fn get_variable_iter() -> VariableNameIter {
        VariableNameIter {
            index: 0,
        }
    }

    /// Returns an iterator over the position variables (x, y, z)
    pub fn get_position_iter() -> PositionVariableNameIter {
        PositionVariableNameIter {
            index: 0,
        }
    }

    /// Returns an iterator over the rotation variables (phi, theta, psi)
    pub fn get_rotation_iter() -> RotationVariableNameIter {
        RotationVariableNameIter {
            index: 0,
        }
    }
}


pub struct VariableNameIter {
    index: u8,
}

impl Iterator for VariableNameIter {
    type Item = VariableName;
    fn next(&mut self) -> Option<Self::Item> {
        let var_idx = match self.index {
            0 => VariableName::x,
            1 => VariableName::y,
            2 => VariableName::z,
            3 => VariableName::phi,
            4 => VariableName::theta,
            5 => VariableName::psi,
            _ => return None,
        };
        self.index += 1;
        Some(var_idx)
    }
}


pub struct PositionVariableNameIter {
    index: u8,
}

impl Iterator for PositionVariableNameIter {
    type Item = VariableName;
    fn next(&mut self) -> Option<Self::Item> {
        let var_idx = match self.index {
            0 => VariableName::x,
            1 => VariableName::y,
            2 => VariableName::z,
            _ => return None
        };
        self.index += 1;
        Some(var_idx)
    }
}


pub struct RotationVariableNameIter {
    index: u8,
}

impl Iterator for RotationVariableNameIter {
    type Item = VariableName;
    fn next(&mut self) -> Option<Self::Item> {
        let var_idx = match self.index {
            0 => VariableName::phi,
            1 => VariableName::theta,
            2 => VariableName::psi,
            _ => return None
        };
        self.index += 1;
        Some(var_idx)
    }
}
