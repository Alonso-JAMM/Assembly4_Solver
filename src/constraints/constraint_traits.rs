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


use ndarray::{Array1, Array2};
use crate::system::Variable;


/// General constraint methods used by the solver.
///
/// This trait is used as a way to interface the distinct constraint functions
/// with the solver. This way the constraint functions are free to choose any
/// way to calculate the constraint errors.
pub trait Constraint {
    /// Evaluates the square of the constraint function f(x)^2
    /// This method is intended to set the calculated gradients and hessians in
    /// internal variables that then are obtained by calling get_gradient and
    /// get_hessian. sys_variables should be the updated variables for the
    /// iteration.
    fn evaluate(&mut self, sys_variables: &Vec<Variable>);

    /// Gets the real value of the square of constraint function
    fn get_value(&self) -> f64;

    /// Gets the gradient of the square of the constraint function. This method
    /// adds the gradient contribution of this constraint to the system gradient.
    fn get_gradient(&self, sys_grad: &mut Array1<f64>, sys_variables: &Vec<Variable>);

    /// Gets the one-dimensional first derivative of the constraint function
    fn get_diff(&mut self, sys_variables: &Vec<Variable>) -> f64;

    /// Gets the hessian matrix of the square of the constraint function. This
    /// method adds the hessian contribution of this constraint to the system
    /// hessian.
    fn get_hessian(&self, sys_hess: &mut Array2<f64>, sys_variables: &Vec<Variable>);
}
