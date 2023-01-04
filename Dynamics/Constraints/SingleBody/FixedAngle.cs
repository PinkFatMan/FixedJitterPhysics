/* Copyright (C) <2009-2011> <Thorben Linneweber, Jitter Physics>
* 
*  This software is provided 'as-is', without any express or implied
*  warranty.  In no event will the authors be held liable for any damages
*  arising from the use of this software.
*
*  Permission is granted to anyone to use this software for any purpose,
*  including commercial applications, and to alter it and redistribute it
*  freely, subject to the following restrictions:
*
*  1. The origin of this software must not be misrepresented; you must not
*      claim that you wrote the original software. If you use this software
*      in a product, an acknowledgment in the product documentation would be
*      appreciated but is not required.
*  2. Altered source versions must be plainly marked as such, and must not be
*      misrepresented as being the original software.
*  3. This notice may not be removed or altered from any source distribution. 
*/

#region Using Statements
using System;
using System.Collections.Generic;

using Jitter.Dynamics;
using Jitter.LinearMath;
using Jitter.Collision.Shapes;
#endregion

namespace Jitter.Dynamics.Constraints.SingleBody
{

    #region Constraint Equations
    // Constraint formulation:
    // 
    // C_1 = R1_x - R2_x
    // C_2 = ...
    // C_3 = ...
    //
    // Derivative:
    //
    // dC_1/dt = w1_x - w2_x
    // dC_2/dt = ...
    // dC_3/dt = ...
    //
    // Jacobian:
    // 
    // dC/dt = J*v+b
    //
    // v = (v1x v1y v1z w1x w1y w1z v2x v2y v2z w2x w2y w2z)^(T) 
    //
    //     v1x v1y v1z w1x w1y w1z v2x v2y v2z w2x w2y w2z
    //     -------------------------------------------------
    // J = 0   0   0   1   0    0   0   0   0   -1   0   0   <- dC_1/dt
    //     0   0   0   0   1    0   0   0   0    0  -1   0   <- ...  
    //     0   0   0   0   0    1   0   0   0    0   0  -1   <- ...
    //
    // Effective Mass:
    //
    // 1/m_eff = [J^T * M^-1 * J] = I1^(-1) + I2^(-1)
    #endregion

    /// <summary>
    /// The body stays at a fixed angle relative to
    /// world space.
    /// </summary>
    public class FixedAngle : Constraint
    {

        private JFix64 biasFactor = (5 * JFix64.EN2);
        private JFix64 softness = JFix64.Zero;

        private JMatrix orientation;
        private JVector accumulatedImpulse;

        /// <summary>
        /// Constraints two bodies to always have the same relative
        /// orientation to each other. Combine the AngleConstraint with a PointOnLine
        /// Constraint to get a prismatic joint.
        /// </summary>
        public FixedAngle(RigidBody body1)
            : base(body1, null)
        {
            orientation = body1.orientation;
        }

        /// <summary>
        /// Defines how big the applied impulses can get.
        /// </summary>
        public JFix64 Softness { get { return softness; } set { softness = value; } }

        /// <summary>
        /// Defines how big the applied impulses can get which correct errors.
        /// </summary>
        public JFix64 BiasFactor { get { return biasFactor; } set { biasFactor = value; } }

        public JMatrix InitialOrientation { get { return orientation; } set { orientation = value; } }

        JMatrix effectiveMass;
        JVector bias;
        JFix64 softnessOverDt;

        /// <summary>
        /// Called once before iteration starts.
        /// </summary>
        /// <param name="timestep">The 5simulation timestep</param>
        public override void PrepareForIteration(JFix64 timestep)
        {
            effectiveMass = body1.invInertiaWorld;

            softnessOverDt = softness / timestep;

            effectiveMass.M11 += softnessOverDt;
            effectiveMass.M22 += softnessOverDt;
            effectiveMass.M33 += softnessOverDt;

            JMatrix.Inverse(ref effectiveMass, out effectiveMass);

            JMatrix q = JMatrix.Transpose(orientation) * body1.orientation;
            JVector axis;

            JFix64 x = q.M32 - q.M23;
            JFix64 y = q.M13 - q.M31;
            JFix64 z = q.M21 - q.M12;

            JFix64 r = JFix64Math.Sqrt(x * x + y * y + z * z);
            JFix64 t = q.M11 + q.M22 + q.M33;

            JFix64 angle = JFix64Math.Atan2(r, t - JFix64.One);
            axis = new JVector(x, y, z) * angle;

            if (r != JFix64.Zero) axis = axis * (JFix64.One / r);

            bias = axis * biasFactor * (-JFix64.One / timestep);

            // Apply previous frame solution as initial guess for satisfying the constraint.
            if (!body1.IsStatic) body1.angularVelocity += JVector.Transform(accumulatedImpulse, body1.invInertiaWorld);
        }

        /// <summary>
        /// Iteratively solve this constraint.
        /// </summary>
        public override void Iterate()
        {
            JVector jv = body1.angularVelocity;

            JVector softnessVector = accumulatedImpulse * softnessOverDt;

            JVector lambda = -JFix64.One * JVector.Transform(jv + bias + softnessVector, effectiveMass);

            accumulatedImpulse += lambda;

            if (!body1.IsStatic) body1.angularVelocity += JVector.Transform(lambda, body1.invInertiaWorld);
        }

    }
}
