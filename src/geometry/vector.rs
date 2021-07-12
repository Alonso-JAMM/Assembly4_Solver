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


use optimization::{
    number_system::HyperDualScalar as HDual,
    geometry::HDVector,
};

use crate::system::Variable;


/// This object holds a vector with its partial derivatives.
///
/// Vector is similar to Quaternion in which it creates hyper dualvectors representing the
/// position of an object including the partial derivatives with respect to the variables
/// of the position of the object. This object is useful for reusing the same position vectors
/// across all constraint functions. This way, constraint functions don't have to
/// to create their own vectors; they can retrieve them from this object.
#[derive(Debug)]
pub struct Vector {
    // These are the vectors containing all of the different partial derivatives
    // with respect to the variables x, y, and z
    x_x: HDVector,
    x_y: HDVector,
    x_z: HDVector,
    y_y: HDVector,
    y_z: HDVector,
    z_z: HDVector,
}


impl Vector {
    pub fn new() -> Vector {
        Vector {
            x_x: HDVector::new(),
            x_y: HDVector::new(),
            x_z: HDVector::new(),
            y_y: HDVector::new(),
            y_z: HDVector::new(),
            z_z: HDVector::new(),
        }
    }

    pub fn evaluate_vector(
            &mut self,
            x_var: &Variable,
            y_var: &Variable,
            z_var: &Variable,
    ) {
        let mut x = HDual::new();
        x.re = x_var.value;
        let mut y = HDual::new();
        y.re = y_var.value;
        let mut z = HDual::new();
        z.re = z_var.value;

        // Vector that represents a vector made out of constant values
        let const_const = HDVector{x, y, z};

        // Find the partial derivatives with respect to x-x, x-y, and x-z
        if x_var.locked || !x_var.enabled {
            // Treat x as a constant value
            self.x_x = const_const;

            // try to find the partial derivatives with respect to x-y
            if y_var.locked || !y_var.enabled {
                // x and y are constant
                self.x_y = const_const;
            }
            else {
                // x is a constant and y is a variable
                y.e2 = 1.0;
                self.x_y = HDVector{x, y, z};
                y.e2 = 0.0;
            }

            // try to find the partial derivatives with respect to x-z
            if z_var.locked || !z_var.enabled {
                // x and z are constant
                self.x_z = const_const;
            }
            else {
                // x is a constant and z is a variable
                z.e2 = 1.0;
                self.x_z = HDVector{x, y, z};
                z.e2 = 0.0;
            }

        }
        else {
            // x is a variable, its partial derivatives matter!
            x.e1 = 1.0;
            x.e2 = 1.0;
            self.x_x = HDVector{x, y, z};
            x.e1 = 0.0;
            x.e2 = 0.0;

            // try to find the partial derivatives with respect to x-y
            if y_var.locked || !y_var.enabled {
                // x is a variable and y is a constant
                self.x_y = self.x_x;
                remove_e2(&mut self.x_y)

            }
            else {
                // x and y are variables
                x.e1 = 1.0;
                y.e2 = 1.0;
                self.x_y = HDVector{x, y , z};
                x.e1 = 0.0;
                y.e2 = 0.0;
            }

            // try to find the partial derivatives with respect to x-z
            if z_var.locked || !z_var.enabled {
                // x is a variable and z is a constant
                self.x_z = self.x_x;
                remove_e2(&mut self.x_z);
            }
            else {
                // x and z are variables;
                x.e1 = 1.0;
                z.e2 = 1.0;
                self.x_z = HDVector{x, y, z};
                x.e1 = 0.0;
                z.e2 = 0.0;
            }
        }

        // Find the partial derivatives with respect to y-y, y-z
        if y_var.locked || !y_var.enabled {
            // Treat y as a constant value
            self.y_y = const_const;

            // try to find the partial derivatives with respect to y-z
            if z_var.locked || !z_var.enabled {
                // y and z are constants
                self.y_z = const_const;
            }
            else {
                // y is a constant and z is a variable
                z.e2 = 1.0;
                self.y_z = HDVector{x, y, z};
                z.e2 = 0.0;
            }
        }
        else {
            // y is a variable
            y.e1 = 1.0;
            y.e2 = 1.0;
            self.y_y = HDVector{x, y, z};
            y.e1 = 0.0;
            y.e2 = 0.0;

            // try to find the partial derivatives with respect to y-z
            if z_var.locked || !z_var.enabled {
                // y is a variable and z is a constant
                self.y_z = self.y_y;
                remove_e2(&mut self.y_z);
            }
            else {
                // y and z are variables
                y.e1 = 1.0;
                z.e2 = 1.0;
                self.y_z = HDVector{x, y, z};
                y.e1 = 0.0;
                z.e2 = 0.0;
            }
        }

        // We only have left the partial derivatives with respect to z-z
        if z_var.locked || !z_var.enabled {
            // z is a constant
            self.z_z = const_const;
        }
        else {
            // z is a variable
            z.e1 = 1.0;
            z.e2 = 1.0;
            self.z_z = HDVector{x, y, z};
            z.e1 = 0.0;
            z.e2 = 0.0;
        }
    }

    /// Returns a vector with the partial derivatives with respect to x and x
    ///
    /// e1 corresponds to x and e2 corresponds to x
    pub fn get_x_x(&self) -> HDVector {
        self.x_x
    }

    /// Returns a vector with the partial derivatives with respect to x and y
    ///
    /// e1 corresponds to x and e2 corresponds to y
    pub fn get_x_y(&self) -> HDVector {
        self.x_y
    }

    /// Returns a vector with the partial derivatives with respect to x and z
    ///
    /// e1 corresponds to x and e2 corresponds to z
    pub fn get_x_z(&self) -> HDVector {
        self.x_z
    }

    /// Returns a vector with the partial derivatives with respect to x and a constant
    ///
    /// e1 corresponds to x and e2 corresponds to a constant
    pub fn get_x_const(&self) -> HDVector {
        let mut x_const = self.x_x;
        remove_e2(&mut x_const);
        x_const
    }

    /// Returns a vector with the partial derivatives with respect to y and x
    ///
    /// e1 corresponds to y and e2 corresponds to x
    pub fn get_y_x(&self) -> HDVector {
        let mut y_x = self.x_y;
        flip_e1_e2(&mut y_x);
        y_x
    }

    /// Returns a vector with the partial derivatives with respect to y and y
    ///
    /// e1 corresponds to y and e2 corresponds to y
    pub fn get_y_y(&self) -> HDVector {
        self.y_y
    }

    /// Returns a vector with the partial derivatives with respect to y and z
    ///
    /// e1 corresponds to y and e2 corresponds to z
    pub fn get_y_z(&self) -> HDVector {
        self.y_z
    }

    /// Returns a vector with the partial derivatives with respect to y and a constant
    ///
    /// e1 corresponds to y and e2 corresponds to a constant
    pub fn get_y_const(&self) -> HDVector {
        let mut y_const = self.y_y;
        remove_e2(&mut y_const);
        y_const
    }

    /// Returns a vector with the partial derivatives with respect to z and x
    ///
    /// e1 corresponds to z and e2 corresponds to x
    pub fn get_z_x(&self) -> HDVector {
        let mut z_x = self.x_z;
        flip_e1_e2(&mut z_x);
        z_x
    }

    /// Returns a vector with the partial derivatives with respect to z and y
    ///
    /// e1 corresponds to z and e2 corresponds to y
    pub fn get_z_y(&self) -> HDVector {
        let mut z_y = self.y_z;
        flip_e1_e2(&mut z_y);
        z_y
    }

    /// Returns a vector with the partial derivatives with respect to z and z
    ///
    /// e1 corresponds to z and e2 corresponds to z
    pub fn get_z_z(&self) -> HDVector {
        self.z_z
    }

    /// Returns a vector with the partial derivatives with respect to z and a constant
    ///
    /// e1 corresponds to z and e2 corresponds to a constant
    pub fn get_z_const(&self) -> HDVector {
        let mut z_const = self.z_z;
        remove_e2(&mut z_const);
        z_const
    }

    /// Returns a vector with the partial derivatives with respect to a constant and x
    ///
    /// e1 corresponds to a constant and e2 corresponds to x
    pub fn get_const_x(&self) -> HDVector {
        let mut const_x = self.x_x;
        remove_e1(&mut const_x);
        const_x
    }

    /// Returns a vector with the partial derivatives with respect to a constant and x
    ///
    /// e1 corresponds to a constant and e2 corresponds to x
    pub fn get_const_y(&self) -> HDVector {
        let mut const_y = self.y_y;
        remove_e1(&mut const_y);
        const_y
    }

    /// Returns a vector with the partial derivatives with respect to a constant and z
    ///
    /// e1 corresponds to a constant and e2 corresponds to z
    pub fn get_const_z(&self) -> HDVector {
        let mut const_z = self.z_z;
        remove_e1(&mut const_z);
        const_z
    }

    /// Returns a vector with the partial derivatives with respect to a constant and
    /// a constant
    ///
    /// e1 corresponds to a constant and e2 corresponds to a constant
    pub fn get_const_const(&self) -> HDVector {
        let mut const_const = self.x_x;
        remove_e1_e2(&mut const_const);
        const_const
    }
}


/// Helper function that removes the components of the partial derivatives of the first
/// variable.
///
/// This function sets e1 and e1e2 to zero for the input vector. When e1 represents a
/// constant, then it and e1e2 has to be 0 since the partial derivatives with respect
/// to a constant are zero.
fn remove_e1(v: &mut HDVector) {
    v.x.e1 = 0.0;
    v.y.e1 = 0.0;
    v.z.e1 = 0.0;
    v.x.e1e2 = 0.0;
    v.y.e1e2 = 0.0;
    v.z.e1e2 = 0.0;
}

/// Helper function that removes the components of the partial derivatives of the
/// second variable.
///
/// The second variable represents a constant value so its partial derivatives have
/// to be zero (e2 and e1e2 are set to zero)
fn remove_e2(v: &mut HDVector) {
    v.x.e2 = 0.0;
    v.y.e2 = 0.0;
    v.z.e2 = 0.0;
    v.x.e1e2 = 0.0;
    v.y.e1e2 = 0.0;
    v.z.e1e2 = 0.0;
}

/// Helper function that removes the components of the partial derivatives of the
/// first and second variables.
///
/// The first and second variables are constants so the partial derivatives with
/// respect to both variables are zero (e1, e2, e1e2 are set to zero)
fn remove_e1_e2(v: &mut HDVector) {
    v.x.e1 = 0.0;
    v.y.e1 = 0.0;
    v.z.e1 = 0.0;
    v.x.e2 = 0.0;
    v.y.e2 = 0.0;
    v.z.e2 = 0.0;
    v.x.e1e2 = 0.0;
    v.y.e1e2 = 0.0;
    v.z.e1e2 = 0.0;
}

/// Helper function that flips the values of e1 and e2 for the input vector.
///
/// This function is useful when the partial derivatives with respect to e1 should
/// we moved to e2 or vice-versa.
fn flip_e1_e2(v: &mut HDVector) {
    let mut old_e1: f64;
    old_e1 = v.x.e1;
    v.x.e1 = v.x.e2;
    v.x.e2 = old_e1;
    old_e1 = v.y.e1;
    v.y.e1 = v.y.e2;
    v.y.e2 = old_e1;
    old_e1 = v.z.e1;
    v.y.e1 = v.z.e2;
    v.z.e2 = old_e1;
}

