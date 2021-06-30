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


/// This struct stores the indices of all the variables (6 variables) that
/// represents an object in 3D space. It is used to obtain the variable from
/// the vector of all the variables in the system.
#[derive(Debug, Clone)]
pub struct ObjectIndices {
    pub x: usize,
    pub y: usize,
    pub z: usize,
    pub phi: usize,
    pub theta: usize,
    pub psi: usize,
}

impl ObjectIndices {
    /// Creates a new empty container. It is assumed that the indices of the
    /// variables will be filled after creating a new ObjectIndices.
    pub fn new() -> ObjectIndices {
        ObjectIndices {
            x: 0,
            y: 0,
            z: 0,
            phi: 0,
            theta: 0,
            psi: 0,
        }
    }

    /// fill a index of a variable.
    pub fn set_index(&mut self, variable: &str, index: usize) {
        match variable {
            "x" => self.x = index,
            "y" => self.y = index,
            "z" => self.z = index,
            "phi" => self.phi = index,
            "theta" => self.theta = index,
            "psi" => self.psi = index,
            _ => ()
        };
    }

    /// obtains the index of a variable from a str
    pub fn get_index(&self, variable: &str) -> usize {
        match variable {
            "x" => self.x,
            "y" => self.y,
            "z" => self.z,
            "phi" => self.phi,
            "theta" => self.theta,
            "psi" => self.psi,
            // this should never happen. Only "x", "y", "z", "phi", "theta", "psi"
            // should be used when dealing with ObjectIndices. Any other variable
            // should be invalid. If an invalid variable is passed, then the code
            // calling this function is doing something wrong and should be fixed!
            _ => unreachable!()
        }
    }
}


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
}

impl Variable {
    pub fn new() -> Variable {
        Variable {
            index: 0,
            value: 0.0,
            initial_value: 0.0,
            locked: false,
            enabled: false,
        }
    }
}


/// Represents the entire system. This struct contains all the variables, objects,
/// and constraints in the system.
#[derive(Debug)]
pub struct System<'a> {
    /// Contains all the objects in the system. The keys are the objects names
    /// obtained from FreeCAD.
    pub objects: HashMap<&'a str, ObjectIndices>,
    /// Contains all the variables in the system. The objects represented by
    /// these variables will contain the indices of the variables in this vector.
    pub variables: Vec<Variable>,
    /// Contains all the constraints in the system. When evaluating the objective
    /// function we are evaluating all the constraints of this vector.
    pub constraints: Vec<ConstraintType>,
}


impl<'a> System<'a> {
    pub fn new() -> System<'a> {
        System {
            objects: HashMap::new(),
            variables: Vec::new(),
            constraints: Vec::new(),
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
        match self.objects.get(new_object_name) {
            None => {
                let n = self.variables.len();
                for _ in 0..6 {
                    let new_variable = Variable::new();
                    self.variables.push(new_variable);
                }

                let mut k: usize;   // variable index in the system variable vector
                let mut x: f64;     // initial value of each variable
                // The new variables are appended at the end of the vector
                // so their indices will start at "n"
                let mut new_object = ObjectIndices::new();
                for (i, variable) in ["x", "y", "z", "phi", "theta", "psi"].iter().enumerate() {
                    k = n+i;
                    new_object.set_index(variable, k);
                    x = *object_params.get(variable).unwrap();
                    self.variables[k].initial_value = x;
                    self.variables[k].value = x;
                }
                self.objects.insert(new_object_name, new_object);
            },
            Some(_) => ()
        }
    }
}
