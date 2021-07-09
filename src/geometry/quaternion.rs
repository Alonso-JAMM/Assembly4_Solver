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
    geometry::HDQuaternion,
};

use crate::system::Variable;



/// This object holds a quaternion with its partial derivatives.
///
/// Quaternion is intended to be used by constraints instead of generating a quaternion
/// themselves. This is useful since different constraints may require the same quaternion
/// and building it outside of the constraint allows the reuse of the quaternion computation.
/// Moreover, it also helps to reduce the number of calculations of partial derivatives.
#[derive(Debug)]
pub struct Quaternion {
    // These are the quaternions containing all of the different partial derivatives
    // with respect to the variables phi, theta, and psi
    phi_phi: HDQuaternion,
    phi_theta: HDQuaternion,
    phi_psi: HDQuaternion,
    theta_theta: HDQuaternion,
    theta_psi: HDQuaternion,
    psi_psi: HDQuaternion,
    indices: QIndices,
}

#[derive(Debug)]
struct QIndices {
    phi: usize,
    theta: usize,
    psi: usize,
}

impl Quaternion {
    pub fn new(phi: usize, theta: usize, psi: usize) -> Quaternion {
        Quaternion {
            phi_phi: HDQuaternion::new(),
            phi_theta: HDQuaternion::new(),
            phi_psi: HDQuaternion::new(),
            theta_theta: HDQuaternion::new(),
            theta_psi: HDQuaternion::new(),
            psi_psi: HDQuaternion::new(),
            indices: QIndices{phi, theta, psi},
        }
    }

    /// Updates the values of the quaternion from the system variables
    pub fn update(&mut self, sys_variables: &Vec<Variable>) {
        let phi_var = &sys_variables[self.indices.phi];
        let theta_var = &sys_variables[self.indices.theta];
        let psi_var = &sys_variables[self.indices.psi];
        self.evaluate_quaternion(phi_var, theta_var, psi_var);
    }

    /// Evaluates all the quaternions with the different partial derivatives
    fn evaluate_quaternion(
            &mut self,
            phi_var: &Variable,
            theta_var: &Variable,
            psi_var: &Variable
    ) {
        let mut phi = HDual::new();
        phi.re = phi_var.value;
        let mut theta = HDual::new();
        theta.re = theta_var.value;
        let mut psi = HDual::new();
        psi.re = psi_var.value;

        // Quaternion that represents a quaternion with constant angles
        let const_const = HDQuaternion::from_angles(phi, theta, psi);

        // Find the partial derivatives with respect to phi
        // phi-phi, phi-theta, phi-psi
        if phi_var.locked || !phi_var.enabled {
            // phi is either locked or disabled, in this case we treat this variable
            // as a constant (partial derivatives are zero)
            self.phi_phi = const_const;

            // now we find phi-theta
            if theta_var.locked || !theta_var.enabled {
                // theta is a constant so its partial derivatives are zero
                self.phi_theta = const_const;
            }
            else {
                theta.e2 = 1.0;
                self.phi_theta = HDQuaternion::from_angles(phi, theta, psi);
                theta.e2 = 0.0;
            }

            // find phi-psi
            if psi_var.locked || !psi_var.enabled {
                self.phi_psi = const_const;
            }
            else {
                psi.e2 = 1.0;
                self.phi_psi = HDQuaternion::from_angles(phi, theta, psi);
                psi.e2 = 0.0;
            }

        }
        else {
            // phi is enabled and not locked, we have to account for the partial
            // derivatives with respect to this variable
            phi.e1 = 1.0;
            phi.e2 = 1.0;
            self.phi_phi = HDQuaternion::from_angles(phi, theta, psi);
            phi.e1 = 0.0;
            phi.e2 = 0.0;

            // now we try to find phi-theta
            if theta_var.locked || !theta_var.enabled {
                self.phi_theta = self.phi_phi;
                remove_e2(&mut self.phi_theta);
            }
            else {
                phi.e1 = 1.0;
                theta.e2 = 1.0;
                self.phi_theta = HDQuaternion::from_angles(phi, theta, psi);
                theta.e2 = 0.0;
                phi.e1 = 0.0;
            }

            // find phi-psi
            if psi_var.locked || !psi_var.enabled {
                self.phi_psi = self.phi_phi;
                remove_e2(&mut self.phi_psi);
            }
            else {
                phi.e1 = 1.0;
                psi.e2 = 1.0;
                self.phi_psi = HDQuaternion::from_angles(phi, theta, psi);
                psi.e2 = 0.0;
                phi.e1 = 0.0;
            }
        }


        // Find the partial derivatives with respect to theta
        // find theta-theta, theta-psi
        if theta_var.locked || !theta_var.enabled {
            self.theta_theta = const_const;

            // find theta-psi
            if psi_var.locked || !psi_var.enabled {
                self.theta_psi = const_const;
            }
            else {
                psi.e2 = 1.0;
                self.theta_psi = HDQuaternion::from_angles(phi, theta, psi);
                psi.e2 = 0.0;
            }
        }
        else {
            theta.e1 = 1.0;
            theta.e2 = 1.0;
            self.theta_theta = HDQuaternion::from_angles(phi, theta, psi);
            theta.e1 = 0.0;
            theta.e2 = 0.0;

            // find theta-psi
            if psi_var.locked || !psi_var.enabled {
                self.theta_psi = self.theta_theta;
                remove_e2(&mut self.theta_psi);
            }
            else {
                theta.e1 = 1.0;
                psi.e2 = 1.0;
                self.theta_psi = HDQuaternion::from_angles(phi, theta, psi);
                psi.e2 = 0.0;
                theta.e1 = 0.0;
            }
        }

        // Now find the partial derivatives with respect to psi and only psi since
        // all the other partial derivatives are already found.
        // We find psi-psi
        if psi_var.locked || !phi_var.enabled {
            self.psi_psi = const_const;
        }
        else {
            psi.e1 = 1.0;
            psi.e2 = 1.0;
            self.psi_psi = HDQuaternion::from_angles(phi, theta, psi);
            psi.e1 = 0.0;
            psi.e2 = 0.0;
        }
    }

    /// Returns a quaternion with the partial derivatives with respect to phi and phi
    ///
    /// e1 corresponds to phi and e2 corresponds to phi
    pub fn get_phi_phi(&self) -> HDQuaternion {
        self.phi_phi
    }

    /// Returns a quaternion with the partial derivatives with respect to phi and theta
    ///
    /// e1 corresponds to phi and e2 corresponds to theta
    pub fn get_phi_theta(&self) -> HDQuaternion {
        self.phi_theta
    }

    /// Returns a quaternion with the partial derivatives with respect to phi and psi
    ///
    /// e1 corresponds to phi and e2 corresponds to psi
    pub fn get_phi_psi(&self) -> HDQuaternion {
        self.phi_psi
    }

    /// Returns a quaternion with the partial derivatives with respect to phi and a constant
    ///
    /// e1 corresponds to phi and e2 corresponds to a constant value
    pub fn get_phi_const(&self) -> HDQuaternion {
        let mut phi_const = self.phi_phi;
        // remove partial derivatives with respect to the constant
        remove_e2(&mut phi_const);
        phi_const
    }

    /// Returns a quaternion with the partial derivatives with respect to theta and phi
    ///
    /// e1 corresponds to theta and e2 corresponds to phi
    pub fn get_theta_phi(&self) -> HDQuaternion {
        let mut theta_phi = self.phi_theta;
        // Flip the partial derivatives of the components of the quaternion
        flip_e1_e2(&mut theta_phi);
        theta_phi
    }

    /// Returns a quaternion with the partial derivatives with respect to theta and theta
    ///
    /// e1 corresponds to theta and e2 corresponds to theta
    pub fn get_theta_theta(&self) -> HDQuaternion {
        self.theta_theta
    }

    /// Returns a quaternion with the partial derivatives with respect to theta and psi
    ///
    /// e1 corresponds to theta and e2 corresponds to psi
    pub fn get_theta_psi(&self) -> HDQuaternion {
        self.theta_psi
    }

    /// Returns a quaternion with the partial derivatives with respect to theta and a constant
    ///
    /// e1 corresponds to theta and e2 corresponds to a constant
    pub fn get_theta_const(&self) -> HDQuaternion {
        let mut theta_const = self.theta_theta;
        // remove partial derivatives with respect to the constant
        remove_e2(&mut theta_const);
        theta_const
    }

    /// Returns a quaternion with the partial derivatives with respect to psi and phi
    ///
    /// e1 corresponds to psi and e2 corresponds to phi
    pub fn get_psi_phi(&self) -> HDQuaternion {
        let mut psi_phi = self.phi_theta;
        // Flip the partial derivatives of the components of the quaternion
        flip_e1_e2(&mut psi_phi);
        psi_phi
    }

    /// Returns a quaternion with the partial derivatives with respect to psi and theta
    ///
    /// e1 corresponds to psi and e2 corresponds to theta
    pub fn get_psi_theta(&self) -> HDQuaternion {
        let mut psi_theta = self.theta_psi;
        // Flip the partial derivatives of the components of the quaternion
        flip_e1_e2(&mut psi_theta);
        psi_theta
    }

    /// Returns a quaternion with the partial derivatives with respect to psi and psi
    ///
    /// e1 corresponds to psi and e2 corresponds to psi
    pub fn get_psi_psi(&self) -> HDQuaternion {
        self.psi_psi
    }

    /// Returns a quaternion with the partial derivatives with respect to psi and a constant
    ///
    /// e1 corresponds to psi and e2 corresponds to a constant
    pub fn get_psi_const(&self) -> HDQuaternion {
        let mut psi_const = self.psi_psi;
        // remove partial derivatives with respect to the constant
        remove_e2(&mut psi_const);
        psi_const
    }

    /// Returns a quaternion with the partial derivatives with respect to a constant and phi
    ///
    /// e1 corresponds to a constant and e2 corresponds to phi
    pub fn get_const_phi(&self) -> HDQuaternion {
        let mut const_phi = self.phi_phi;
        // remove partial derivatives with respect to the constant
        remove_e1(&mut const_phi);
        const_phi
    }

    /// Returns a quaternion with the partial derivatives with respect to a constant and theta
    ///
    /// e1 corresponds to a constant and e2 corresponds to theta
    pub fn get_const_theta(&self) -> HDQuaternion {
        let mut const_theta = self.theta_theta;
        // remove partial derivatives with respect to the constant
        remove_e1(&mut const_theta);
        const_theta
    }


    /// Returns a quaternion with the partial derivatives with respect to a constant and psi
    ///
    /// e1 corresponds to a constant and e2 corresponds to psi
    pub fn get_const_psi(&self) -> HDQuaternion {
        let mut const_psi = self.psi_psi;
        // remove partial derivatives with respect to the constant
        remove_e1(&mut const_psi);
        const_psi
    }

    /// Returns a quaternion with the partial derivatives with respect to a constant and a constant
    ///
    /// e1 corresponds to a constant and e2 corresponds to a constant
    pub fn get_const_const(&self) -> HDQuaternion {
        let mut const_const = self.phi_phi;
        // remove partial derivatives with respect to the constants
        remove_e1_e2(&mut const_const);
        const_const
    }
}

///  Helper function that removes the components of the partial derivatives of e1
///
/// This function sets e1 and e1e2 to zero for the input quaternion. It is used
/// to remove the partial derivatives with respect to a constant when e1 represents
/// the constant.
fn remove_e1(q: &mut HDQuaternion) {
    q.q0.e1 = 0.0;
    q.q1.e1 = 0.0;
    q.q2.e1 = 0.0;
    q.q3.e1 = 0.0;
    q.q0.e1e2 = 0.0;
    q.q1.e1e2 = 0.0;
    q.q2.e1e2 = 0.0;
    q.q3.e1e2 = 0.0;
}


/// Helper function that removes the components of the partial derivatives of e2
///
/// This function sets e2 and e1e2 to zero for the input quaternion. It is used to
/// remove the partial derivatives with respect to a constant when e2 represents
/// the constant
fn remove_e2(q: &mut HDQuaternion) {
    q.q0.e2 = 0.0;
    q.q1.e2 = 0.0;
    q.q2.e2 = 0.0;
    q.q3.e2 = 0.0;
    q.q0.e1e2 = 0.0;
    q.q1.e1e2 = 0.0;
    q.q2.e1e2 = 0.0;
    q.q3.e1e2 = 0.0;
}


/// Helper function that removes the components of all the partial derivatives
///
/// This function sets e1, e2, and e1e2 to zero for the input quaternion. It is
/// used to remove all partial derivatives of the quaternion when e1 and e2 represent
/// constant values.
fn remove_e1_e2(q: &mut HDQuaternion) {
    q.q0.e1 = 0.0;
    q.q1.e1 = 0.0;
    q.q2.e1 = 0.0;
    q.q3.e1 = 0.0;
    q.q0.e2 = 0.0;
    q.q1.e2 = 0.0;
    q.q2.e2 = 0.0;
    q.q3.e2 = 0.0;
    q.q0.e1e2 = 0.0;
    q.q1.e1e2 = 0.0;
    q.q2.e1e2 = 0.0;
    q.q3.e1e2 = 0.0;
}


/// Helper function that flips the values of e1 and e2 for the input quaternion.
///
/// This function is useful when the partial derivatives with respect to e1 should
/// we moved to e2 or vice-versa.
fn flip_e1_e2(q: &mut HDQuaternion) {
    let mut old_e1: f64;
    old_e1 = q.q0.e1;
    q.q0.e1 = q.q0.e2;
    q.q0.e2 = old_e1;
    old_e1 = q.q1.e1;
    q.q1.e1 = q.q1.e2;
    q.q1.e2 = old_e1;
    old_e1 = q.q2.e1;
    q.q2.e1 = q.q2.e2;
    q.q2.e2 = old_e1;
    old_e1 = q.q3.e1;
    q.q3.e1 = q.q3.e2;
    q.q3.e2 = old_e1;
}
