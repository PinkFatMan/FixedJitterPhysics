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

namespace Jitter.Dynamics.Constraints
{

    #region Constraint Equations
    // Constraint formulation:
    // 
    // C = |(p1-p2) x l|
    //
    #endregion

    /// <summary>
    /// Constraints a point on a body to be fixed on a line
    /// which is fixed on another body.
    /// </summary>
    public class PointOnLine : Constraint
    {
        private JVector lineNormal;

        private JVector localAnchor1, localAnchor2;
        private JVector r1, r2;

        private JFix64 biasFactor = JFix64.Half;
        private JFix64 softness = JFix64.Zero;

        /// <summary>
        /// Constraints a point on a body to be fixed on a line
        /// which is fixed on another body.
        /// </summary>
        /// <param name="body1"></param>
        /// <param name="body2"></param>
        /// <param name="lineStartPointBody1"></param>
        /// <param name="lineDirection"></param>
        /// <param name="pointBody2"></param>
        public PointOnLine(RigidBody body1, RigidBody body2,
            JVector lineStartPointBody1, JVector pointBody2) : base(body1,body2)
        {

            JVector.Subtract(ref lineStartPointBody1, ref body1.position, out localAnchor1);
            JVector.Subtract(ref pointBody2, ref body2.position, out localAnchor2);

            JVector.Transform(ref localAnchor1, ref body1.invOrientation, out localAnchor1);
            JVector.Transform(ref localAnchor2, ref body2.invOrientation, out localAnchor2);

            lineNormal = JVector.Normalize(lineStartPointBody1 - pointBody2);
        }

        public JFix64 AppliedImpulse { get { return accumulatedImpulse; } }

        /// <summary>
        /// Defines how big the applied impulses can get.
        /// </summary>
        public JFix64 Softness { get { return softness; } set { softness = value; } }

        /// <summary>
        /// Defines how big the applied impulses can get which correct errors.
        /// </summary>
        public JFix64 BiasFactor { get { return biasFactor; } set { biasFactor = value; } }

        JFix64 effectiveMass = JFix64.Zero;
        JFix64 accumulatedImpulse = JFix64.Zero;
        JFix64 bias;
        JFix64 softnessOverDt;

        JVector[] jacobian = new JVector[4];

        /// <summary>
        /// Called once before iteration starts.
        /// </summary>
        /// <param name="timestep">The simulation timestep</param>
        public override void PrepareForIteration(JFix64 timestep)
        {
            JVector.Transform(ref localAnchor1, ref body1.orientation, out r1);
            JVector.Transform(ref localAnchor2, ref body2.orientation, out r2);

            JVector p1, p2, dp;
            JVector.Add(ref body1.position, ref r1, out p1);
            JVector.Add(ref body2.position, ref r2, out p2);

            JVector.Subtract(ref p2, ref p1, out dp);

            JVector l = JVector.Transform(lineNormal, body1.orientation);
            l.Normalize();

            JVector t = (p1 - p2) % l;
            if(t.LengthSquared() != JFix64.Zero) t.Normalize();
            t = t % l;

            jacobian[0] = t;                      // linearVel Body1
            jacobian[1] = (r1 + p2 - p1) % t;     // angularVel Body1
            jacobian[2] = -JFix64.One * t;              // linearVel Body2
            jacobian[3] = -JFix64.One * r2 % t;         // angularVel Body2

            effectiveMass = body1.inverseMass + body2.inverseMass
                + JVector.Transform(jacobian[1], body1.invInertiaWorld) * jacobian[1]
                + JVector.Transform(jacobian[3], body2.invInertiaWorld) * jacobian[3];

            softnessOverDt = softness / timestep;
            effectiveMass += softnessOverDt;

            if(effectiveMass != 0) effectiveMass = JFix64.One / effectiveMass;

            bias = - (l % (p2-p1)).Length() * biasFactor * (JFix64.One / timestep);

            if (!body1.isStatic)
            {
                body1.linearVelocity += body1.inverseMass * accumulatedImpulse * jacobian[0];
                body1.angularVelocity += JVector.Transform(accumulatedImpulse * jacobian[1], body1.invInertiaWorld);
            }

            if (!body2.isStatic)
            {
                body2.linearVelocity += body2.inverseMass * accumulatedImpulse * jacobian[2];
                body2.angularVelocity += JVector.Transform(accumulatedImpulse * jacobian[3], body2.invInertiaWorld);
            }
        }

        /// <summary>
        /// Iteratively solve this constraint.
        /// </summary>
        public override void Iterate()
        {
            JFix64 jv =
                body1.linearVelocity * jacobian[0] +
                body1.angularVelocity * jacobian[1] +
                body2.linearVelocity * jacobian[2] +
                body2.angularVelocity * jacobian[3];

            JFix64 softnessScalar = accumulatedImpulse * softnessOverDt;

            JFix64 lambda = -effectiveMass * (jv + bias + softnessScalar);

            accumulatedImpulse += lambda;

            if (!body1.isStatic)
            {
                body1.linearVelocity += body1.inverseMass * lambda * jacobian[0];
                body1.angularVelocity += JVector.Transform(lambda * jacobian[1], body1.invInertiaWorld);
            }

            if (!body2.isStatic)
            {
                body2.linearVelocity += body2.inverseMass * lambda * jacobian[2];
                body2.angularVelocity += JVector.Transform(lambda * jacobian[3], body2.invInertiaWorld);
            }
        }

        public override void DebugDraw(IDebugDrawer drawer)
        {
            drawer.DrawLine(body1.position + r1,
                body1.position + r1 + JVector.Transform(lineNormal, body1.orientation) * (100 * JFix64.One));
        }

    }
}
