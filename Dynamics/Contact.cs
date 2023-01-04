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
using Jitter.Dynamics.Constraints;
#endregion

namespace Jitter.Dynamics
{

    #region public class ContactSettings
    public class ContactSettings
    {
        public enum MaterialCoefficientMixingType { TakeMaximum, TakeMinimum, UseAverage }

        internal JFix64 maximumBias = (10 * JFix64.One);
        internal JFix64 bias = (25 * JFix64.EN2);
        internal JFix64 minVelocity = JFix64.EN3;
        internal JFix64 allowedPenetration = JFix64.EN2;
        internal JFix64 breakThreshold = JFix64.EN2;

        internal MaterialCoefficientMixingType materialMode = MaterialCoefficientMixingType.UseAverage;

        public JFix64 MaximumBias { get { return maximumBias; } set { maximumBias = value; } }

        public JFix64 BiasFactor { get { return bias; } set { bias = value; } }

        public JFix64 MinimumVelocity { get { return minVelocity; } set { minVelocity = value; } }

        public JFix64 AllowedPenetration { get { return allowedPenetration; } set { allowedPenetration = value; } }

        public JFix64 BreakThreshold { get { return breakThreshold; } set { breakThreshold = value; } }

        public MaterialCoefficientMixingType MaterialCoefficientMixing { get { return materialMode; } set { materialMode = value; } }
    }
    #endregion


    /// <summary>
    /// </summary>
    public class Contact : IConstraint
    {
        private ContactSettings settings;

        internal RigidBody body1, body2;

        internal JVector normal, tangent;

        internal JVector realRelPos1, realRelPos2;
        internal JVector relativePos1, relativePos2;
        internal JVector p1, p2;

        internal JFix64 accumulatedNormalImpulse = JFix64.Zero;
        internal JFix64 accumulatedTangentImpulse = JFix64.Zero;

        internal JFix64 penetration = JFix64.Zero;
        internal JFix64 initialPen = JFix64.Zero;

        private JFix64 staticFriction, dynamicFriction, restitution;
        private JFix64 friction = JFix64.Zero;

        private JFix64 massNormal = JFix64.Zero, massTangent = JFix64.Zero;
        private JFix64 restitutionBias = JFix64.Zero;

        private bool newContact = false;

        private bool treatBody1AsStatic = false;
        private bool treatBody2AsStatic = false;


        bool body1IsMassPoint; bool body2IsMassPoint;

        JFix64 lostSpeculativeBounce = JFix64.Zero;
        JFix64 speculativeVelocity = JFix64.Zero;

        /// <summary>
        /// A contact resource pool.
        /// </summary>
        public static readonly ResourcePool<Contact> Pool =
            new ResourcePool<Contact>();

        private JFix64 lastTimeStep = JFix64.PositiveInfinity;

        #region Properties
        public JFix64 Restitution
        {
            get { return restitution; }
            set { restitution = value; }
        }

        public JFix64 StaticFriction
        {
            get { return staticFriction; }
            set { staticFriction = value; }
        }

        public JFix64 DynamicFriction
        {
            get { return dynamicFriction; }
            set { dynamicFriction = value; }
        }

        /// <summary>
        /// The first body involved in the contact.
        /// </summary>
        public RigidBody Body1 { get { return body1; } }

        /// <summary>
        /// The second body involved in the contact.
        /// </summary>
        public RigidBody Body2 { get { return body2; } }

        /// <summary>
        /// The penetration of the contact.
        /// </summary>
        public JFix64 Penetration { get { return penetration; } }

        /// <summary>
        /// The collision position in world space of body1.
        /// </summary>
        public JVector Position1 { get { return p1; } }

        /// <summary>
        /// The collision position in world space of body2.
        /// </summary>
        public JVector Position2 { get { return p2; } }

        /// <summary>
        /// The contact tangent.
        /// </summary>
        public JVector Tangent { get { return tangent; } }

        /// <summary>
        /// The contact normal.
        /// </summary>
        public JVector Normal { get { return normal; } }
        #endregion

        /// <summary>
        /// Calculates relative velocity of body contact points on the bodies.
        /// </summary>
        /// <param name="relVel">The relative velocity of body contact points on the bodies.</param>
        public JVector CalculateRelativeVelocity()
        {
            JFix64 x, y, z;

            x = (body2.angularVelocity.Y * relativePos2.Z) - (body2.angularVelocity.Z * relativePos2.Y) + body2.linearVelocity.X;
            y = (body2.angularVelocity.Z * relativePos2.X) - (body2.angularVelocity.X * relativePos2.Z) + body2.linearVelocity.Y;
            z = (body2.angularVelocity.X * relativePos2.Y) - (body2.angularVelocity.Y * relativePos2.X) + body2.linearVelocity.Z;

            JVector relVel;
            relVel.X = x - (body1.angularVelocity.Y * relativePos1.Z) + (body1.angularVelocity.Z * relativePos1.Y) - body1.linearVelocity.X;
            relVel.Y = y - (body1.angularVelocity.Z * relativePos1.X) + (body1.angularVelocity.X * relativePos1.Z) - body1.linearVelocity.Y;
            relVel.Z = z - (body1.angularVelocity.X * relativePos1.Y) + (body1.angularVelocity.Y * relativePos1.X) - body1.linearVelocity.Z;

            return relVel;
        }

        /// <summary>
        /// Solves the contact iteratively.
        /// </summary>
        public void Iterate()
        {
            //body1.linearVelocity = JVector.Zero;
            //body2.linearVelocity = JVector.Zero;
            //return;

            if (treatBody1AsStatic && treatBody2AsStatic) return;

            JFix64 dvx, dvy, dvz;

            dvx = body2.linearVelocity.X - body1.linearVelocity.X;
            dvy = body2.linearVelocity.Y - body1.linearVelocity.Y;
            dvz = body2.linearVelocity.Z - body1.linearVelocity.Z;

            if (!body1IsMassPoint)
            {
                dvx = dvx - (body1.angularVelocity.Y * relativePos1.Z) + (body1.angularVelocity.Z * relativePos1.Y);
                dvy = dvy - (body1.angularVelocity.Z * relativePos1.X) + (body1.angularVelocity.X * relativePos1.Z);
                dvz = dvz - (body1.angularVelocity.X * relativePos1.Y) + (body1.angularVelocity.Y * relativePos1.X);
            }

            if (!body2IsMassPoint)
            {
                dvx = dvx + (body2.angularVelocity.Y * relativePos2.Z) - (body2.angularVelocity.Z * relativePos2.Y);
                dvy = dvy + (body2.angularVelocity.Z * relativePos2.X) - (body2.angularVelocity.X * relativePos2.Z);
                dvz = dvz + (body2.angularVelocity.X * relativePos2.Y) - (body2.angularVelocity.Y * relativePos2.X);
            }

            // this gets us some performance
            if (dvx * dvx + dvy * dvy + dvz * dvz < settings.minVelocity * settings.minVelocity)
            { return; }

            JFix64 vn = normal.X * dvx + normal.Y * dvy + normal.Z * dvz;
            JFix64 normalImpulse = massNormal * (-vn + restitutionBias + speculativeVelocity);

            JFix64 oldNormalImpulse = accumulatedNormalImpulse;
            accumulatedNormalImpulse = oldNormalImpulse + normalImpulse;
            if (accumulatedNormalImpulse < JFix64.Zero) accumulatedNormalImpulse = JFix64.Zero;
            normalImpulse = accumulatedNormalImpulse - oldNormalImpulse;

            JFix64 vt = dvx * tangent.X + dvy * tangent.Y + dvz * tangent.Z;
            JFix64 maxTangentImpulse = friction * accumulatedNormalImpulse;
            JFix64 tangentImpulse = massTangent * (-vt);

            JFix64 oldTangentImpulse = accumulatedTangentImpulse;
            accumulatedTangentImpulse = oldTangentImpulse + tangentImpulse;
            if (accumulatedTangentImpulse < -maxTangentImpulse) accumulatedTangentImpulse = -maxTangentImpulse;
            else if (accumulatedTangentImpulse > maxTangentImpulse) accumulatedTangentImpulse = maxTangentImpulse;

            tangentImpulse = accumulatedTangentImpulse - oldTangentImpulse;

            // Apply contact impulse
            JVector impulse;
            impulse.X = normal.X * normalImpulse + tangent.X * tangentImpulse;
            impulse.Y = normal.Y * normalImpulse + tangent.Y * tangentImpulse;
            impulse.Z = normal.Z * normalImpulse + tangent.Z * tangentImpulse;

            if (!treatBody1AsStatic)
            {
                body1.linearVelocity.X -= (impulse.X * body1.inverseMass);
                body1.linearVelocity.Y -= (impulse.Y * body1.inverseMass);
                body1.linearVelocity.Z -= (impulse.Z * body1.inverseMass);

                if (!body1IsMassPoint)
                {
                    JFix64 num0, num1, num2;
                    num0 = relativePos1.Y * impulse.Z - relativePos1.Z * impulse.Y;
                    num1 = relativePos1.Z * impulse.X - relativePos1.X * impulse.Z;
                    num2 = relativePos1.X * impulse.Y - relativePos1.Y * impulse.X;

                    JFix64 num3 =
                        (((num0 * body1.invInertiaWorld.M11) +
                        (num1 * body1.invInertiaWorld.M21)) +
                        (num2 * body1.invInertiaWorld.M31));
                    JFix64 num4 =
                        (((num0 * body1.invInertiaWorld.M12) +
                        (num1 * body1.invInertiaWorld.M22)) +
                        (num2 * body1.invInertiaWorld.M32));
                    JFix64 num5 =
                        (((num0 * body1.invInertiaWorld.M13) +
                        (num1 * body1.invInertiaWorld.M23)) +
                        (num2 * body1.invInertiaWorld.M33));

                    body1.angularVelocity.X -= num3;
                    body1.angularVelocity.Y -= num4;
                    body1.angularVelocity.Z -= num5;
                }
            }

            if (!treatBody2AsStatic)
            {

                body2.linearVelocity.X += (impulse.X * body2.inverseMass);
                body2.linearVelocity.Y += (impulse.Y * body2.inverseMass);
                body2.linearVelocity.Z += (impulse.Z * body2.inverseMass);

                if (!body2IsMassPoint)
                {

                    JFix64 num0, num1, num2;
                    num0 = relativePos2.Y * impulse.Z - relativePos2.Z * impulse.Y;
                    num1 = relativePos2.Z * impulse.X - relativePos2.X * impulse.Z;
                    num2 = relativePos2.X * impulse.Y - relativePos2.Y * impulse.X;

                    JFix64 num3 =
                        (((num0 * body2.invInertiaWorld.M11) +
                        (num1 * body2.invInertiaWorld.M21)) +
                        (num2 * body2.invInertiaWorld.M31));
                    JFix64 num4 =
                        (((num0 * body2.invInertiaWorld.M12) +
                        (num1 * body2.invInertiaWorld.M22)) +
                        (num2 * body2.invInertiaWorld.M32));
                    JFix64 num5 =
                        (((num0 * body2.invInertiaWorld.M13) +
                        (num1 * body2.invInertiaWorld.M23)) +
                        (num2 * body2.invInertiaWorld.M33));

                    body2.angularVelocity.X += num3;
                    body2.angularVelocity.Y += num4;
                    body2.angularVelocity.Z += num5;

                }
            }

        }

        public JFix64 AppliedNormalImpulse { get { return accumulatedNormalImpulse; } }
        public JFix64 AppliedTangentImpulse { get { return accumulatedTangentImpulse; } }

        /// <summary>
        /// The points in wolrd space gets recalculated by transforming the
        /// local coordinates. Also new penetration depth is estimated.
        /// </summary>
        public void UpdatePosition()
        {
            if (body1IsMassPoint)
            {
                JVector.Add(ref realRelPos1, ref body1.position, out p1);
            }
            else
            {
                JVector.Transform(ref realRelPos1, ref body1.orientation, out p1);
                JVector.Add(ref p1, ref body1.position, out p1);
            }

            if (body2IsMassPoint)
            {
                JVector.Add(ref realRelPos2, ref body2.position, out p2);
            }
            else
            {
                JVector.Transform(ref realRelPos2, ref body2.orientation, out p2);
                JVector.Add(ref p2, ref body2.position, out p2);
            }


            JVector dist; JVector.Subtract(ref p1, ref p2, out dist);
            penetration = JVector.Dot(ref dist, ref normal);
        }

        /// <summary>
        /// An impulse is applied an both contact points.
        /// </summary>
        /// <param name="impulse">The impulse to apply.</param>
        public void ApplyImpulse(ref JVector impulse)
        {
            #region INLINE - HighFrequency
            //JVector temp;

            if (!treatBody1AsStatic)
            {
                body1.linearVelocity.X -= (impulse.X * body1.inverseMass);
                body1.linearVelocity.Y -= (impulse.Y * body1.inverseMass);
                body1.linearVelocity.Z -= (impulse.Z * body1.inverseMass);

                JFix64 num0, num1, num2;
                num0 = relativePos1.Y * impulse.Z - relativePos1.Z * impulse.Y;
                num1 = relativePos1.Z * impulse.X - relativePos1.X * impulse.Z;
                num2 = relativePos1.X * impulse.Y - relativePos1.Y * impulse.X;

                JFix64 num3 =
                    (((num0 * body1.invInertiaWorld.M11) +
                    (num1 * body1.invInertiaWorld.M21)) +
                    (num2 * body1.invInertiaWorld.M31));
                JFix64 num4 =
                    (((num0 * body1.invInertiaWorld.M12) +
                    (num1 * body1.invInertiaWorld.M22)) +
                    (num2 * body1.invInertiaWorld.M32));
                JFix64 num5 =
                    (((num0 * body1.invInertiaWorld.M13) +
                    (num1 * body1.invInertiaWorld.M23)) +
                    (num2 * body1.invInertiaWorld.M33));

                body1.angularVelocity.X -= num3;
                body1.angularVelocity.Y -= num4;
                body1.angularVelocity.Z -= num5;
            }

            if (!treatBody2AsStatic)
            {

                body2.linearVelocity.X += (impulse.X * body2.inverseMass);
                body2.linearVelocity.Y += (impulse.Y * body2.inverseMass);
                body2.linearVelocity.Z += (impulse.Z * body2.inverseMass);

                JFix64 num0, num1, num2;
                num0 = relativePos2.Y * impulse.Z - relativePos2.Z * impulse.Y;
                num1 = relativePos2.Z * impulse.X - relativePos2.X * impulse.Z;
                num2 = relativePos2.X * impulse.Y - relativePos2.Y * impulse.X;

                JFix64 num3 =
                    (((num0 * body2.invInertiaWorld.M11) +
                    (num1 * body2.invInertiaWorld.M21)) +
                    (num2 * body2.invInertiaWorld.M31));
                JFix64 num4 =
                    (((num0 * body2.invInertiaWorld.M12) +
                    (num1 * body2.invInertiaWorld.M22)) +
                    (num2 * body2.invInertiaWorld.M32));
                JFix64 num5 =
                    (((num0 * body2.invInertiaWorld.M13) +
                    (num1 * body2.invInertiaWorld.M23)) +
                    (num2 * body2.invInertiaWorld.M33));

                body2.angularVelocity.X += num3;
                body2.angularVelocity.Y += num4;
                body2.angularVelocity.Z += num5;
            }


            #endregion
        }

        public void ApplyImpulse(JVector impulse)
        {
            #region INLINE - HighFrequency
            //JVector temp;

            if (!treatBody1AsStatic)
            {
                body1.linearVelocity.X -= (impulse.X * body1.inverseMass);
                body1.linearVelocity.Y -= (impulse.Y * body1.inverseMass);
                body1.linearVelocity.Z -= (impulse.Z * body1.inverseMass);

                JFix64 num0, num1, num2;
                num0 = relativePos1.Y * impulse.Z - relativePos1.Z * impulse.Y;
                num1 = relativePos1.Z * impulse.X - relativePos1.X * impulse.Z;
                num2 = relativePos1.X * impulse.Y - relativePos1.Y * impulse.X;

                JFix64 num3 =
                    (((num0 * body1.invInertiaWorld.M11) +
                    (num1 * body1.invInertiaWorld.M21)) +
                    (num2 * body1.invInertiaWorld.M31));
                JFix64 num4 =
                    (((num0 * body1.invInertiaWorld.M12) +
                    (num1 * body1.invInertiaWorld.M22)) +
                    (num2 * body1.invInertiaWorld.M32));
                JFix64 num5 =
                    (((num0 * body1.invInertiaWorld.M13) +
                    (num1 * body1.invInertiaWorld.M23)) +
                    (num2 * body1.invInertiaWorld.M33));

                body1.angularVelocity.X -= num3;
                body1.angularVelocity.Y -= num4;
                body1.angularVelocity.Z -= num5;
            }

            if (!treatBody2AsStatic)
            {

                body2.linearVelocity.X += (impulse.X * body2.inverseMass);
                body2.linearVelocity.Y += (impulse.Y * body2.inverseMass);
                body2.linearVelocity.Z += (impulse.Z * body2.inverseMass);

                JFix64 num0, num1, num2;
                num0 = relativePos2.Y * impulse.Z - relativePos2.Z * impulse.Y;
                num1 = relativePos2.Z * impulse.X - relativePos2.X * impulse.Z;
                num2 = relativePos2.X * impulse.Y - relativePos2.Y * impulse.X;

                JFix64 num3 =
                    (((num0 * body2.invInertiaWorld.M11) +
                    (num1 * body2.invInertiaWorld.M21)) +
                    (num2 * body2.invInertiaWorld.M31));
                JFix64 num4 =
                    (((num0 * body2.invInertiaWorld.M12) +
                    (num1 * body2.invInertiaWorld.M22)) +
                    (num2 * body2.invInertiaWorld.M32));
                JFix64 num5 =
                    (((num0 * body2.invInertiaWorld.M13) +
                    (num1 * body2.invInertiaWorld.M23)) +
                    (num2 * body2.invInertiaWorld.M33));

                body2.angularVelocity.X += num3;
                body2.angularVelocity.Y += num4;
                body2.angularVelocity.Z += num5;
            }


            #endregion
        }

        /// <summary>
        /// PrepareForIteration has to be called before <see cref="Iterate"/>.
        /// </summary>
        /// <param name="timestep">The timestep of the simulation.</param>
        public void PrepareForIteration(JFix64 timestep)
        {
            JFix64 dvx, dvy, dvz;

            dvx = (body2.angularVelocity.Y * relativePos2.Z) - (body2.angularVelocity.Z * relativePos2.Y) + body2.linearVelocity.X;
            dvy = (body2.angularVelocity.Z * relativePos2.X) - (body2.angularVelocity.X * relativePos2.Z) + body2.linearVelocity.Y;
            dvz = (body2.angularVelocity.X * relativePos2.Y) - (body2.angularVelocity.Y * relativePos2.X) + body2.linearVelocity.Z;

            dvx = dvx - (body1.angularVelocity.Y * relativePos1.Z) + (body1.angularVelocity.Z * relativePos1.Y) - body1.linearVelocity.X;
            dvy = dvy - (body1.angularVelocity.Z * relativePos1.X) + (body1.angularVelocity.X * relativePos1.Z) - body1.linearVelocity.Y;
            dvz = dvz - (body1.angularVelocity.X * relativePos1.Y) + (body1.angularVelocity.Y * relativePos1.X) - body1.linearVelocity.Z;

            JFix64 kNormal = JFix64.Zero;

            JVector rantra = JVector.Zero;
            if (!treatBody1AsStatic)
            {
                kNormal += body1.inverseMass;

                if (!body1IsMassPoint)
                {

                    // JVector.Cross(ref relativePos1, ref normal, out rantra);
                    rantra.X = (relativePos1.Y * normal.Z) - (relativePos1.Z * normal.Y);
                    rantra.Y = (relativePos1.Z * normal.X) - (relativePos1.X * normal.Z);
                    rantra.Z = (relativePos1.X * normal.Y) - (relativePos1.Y * normal.X);

                    // JVector.Transform(ref rantra, ref body1.invInertiaWorld, out rantra);
                    JFix64 num0 = ((rantra.X * body1.invInertiaWorld.M11) + (rantra.Y * body1.invInertiaWorld.M21)) + (rantra.Z * body1.invInertiaWorld.M31);
                    JFix64 num1 = ((rantra.X * body1.invInertiaWorld.M12) + (rantra.Y * body1.invInertiaWorld.M22)) + (rantra.Z * body1.invInertiaWorld.M32);
                    JFix64 num2 = ((rantra.X * body1.invInertiaWorld.M13) + (rantra.Y * body1.invInertiaWorld.M23)) + (rantra.Z * body1.invInertiaWorld.M33);

                    rantra.X = num0; rantra.Y = num1; rantra.Z = num2;

                    //JVector.Cross(ref rantra, ref relativePos1, out rantra);
                    num0 = (rantra.Y * relativePos1.Z) - (rantra.Z * relativePos1.Y);
                    num1 = (rantra.Z * relativePos1.X) - (rantra.X * relativePos1.Z);
                    num2 = (rantra.X * relativePos1.Y) - (rantra.Y * relativePos1.X);

                    rantra.X = num0; rantra.Y = num1; rantra.Z = num2;
                }
            }

            JVector rbntrb = JVector.Zero;
            if (!treatBody2AsStatic)
            {
                kNormal += body2.inverseMass;

                if (!body2IsMassPoint)
                {

                    // JVector.Cross(ref relativePos1, ref normal, out rantra);
                    rbntrb.X = (relativePos2.Y * normal.Z) - (relativePos2.Z * normal.Y);
                    rbntrb.Y = (relativePos2.Z * normal.X) - (relativePos2.X * normal.Z);
                    rbntrb.Z = (relativePos2.X * normal.Y) - (relativePos2.Y * normal.X);

                    // JVector.Transform(ref rantra, ref body1.invInertiaWorld, out rantra);
                    JFix64 num0 = ((rbntrb.X * body2.invInertiaWorld.M11) + (rbntrb.Y * body2.invInertiaWorld.M21)) + (rbntrb.Z * body2.invInertiaWorld.M31);
                    JFix64 num1 = ((rbntrb.X * body2.invInertiaWorld.M12) + (rbntrb.Y * body2.invInertiaWorld.M22)) + (rbntrb.Z * body2.invInertiaWorld.M32);
                    JFix64 num2 = ((rbntrb.X * body2.invInertiaWorld.M13) + (rbntrb.Y * body2.invInertiaWorld.M23)) + (rbntrb.Z * body2.invInertiaWorld.M33);

                    rbntrb.X = num0; rbntrb.Y = num1; rbntrb.Z = num2;

                    //JVector.Cross(ref rantra, ref relativePos1, out rantra);
                    num0 = (rbntrb.Y * relativePos2.Z) - (rbntrb.Z * relativePos2.Y);
                    num1 = (rbntrb.Z * relativePos2.X) - (rbntrb.X * relativePos2.Z);
                    num2 = (rbntrb.X * relativePos2.Y) - (rbntrb.Y * relativePos2.X);

                    rbntrb.X = num0; rbntrb.Y = num1; rbntrb.Z = num2;
                }
            }

            if (!treatBody1AsStatic) kNormal += rantra.X * normal.X + rantra.Y * normal.Y + rantra.Z * normal.Z;
            if (!treatBody2AsStatic) kNormal += rbntrb.X * normal.X + rbntrb.Y * normal.Y + rbntrb.Z * normal.Z;

            massNormal = JFix64.One / kNormal;

            JFix64 num = dvx * normal.X + dvy * normal.Y + dvz * normal.Z;

            tangent.X = dvx - normal.X * num;
            tangent.Y = dvy - normal.Y * num;
            tangent.Z = dvz - normal.Z * num;

            num = tangent.X * tangent.X + tangent.Y * tangent.Y + tangent.Z * tangent.Z;

            if (num != JFix64.Zero)
            {
                num = JFix64Math.Sqrt(num);
                tangent.X /= num;
                tangent.Y /= num;
                tangent.Z /= num;
            }

            JFix64 kTangent = JFix64.Zero;

            if (treatBody1AsStatic) rantra.MakeZero();
            else
            {
                kTangent += body1.inverseMass;
  
                if (!body1IsMassPoint)
                {
                    // JVector.Cross(ref relativePos1, ref normal, out rantra);
                    rantra.X = (relativePos1.Y * tangent.Z) - (relativePos1.Z * tangent.Y);
                    rantra.Y = (relativePos1.Z * tangent.X) - (relativePos1.X * tangent.Z);
                    rantra.Z = (relativePos1.X * tangent.Y) - (relativePos1.Y * tangent.X);

                    // JVector.Transform(ref rantra, ref body1.invInertiaWorld, out rantra);
                    JFix64 num0 = ((rantra.X * body1.invInertiaWorld.M11) + (rantra.Y * body1.invInertiaWorld.M21)) + (rantra.Z * body1.invInertiaWorld.M31);
                    JFix64 num1 = ((rantra.X * body1.invInertiaWorld.M12) + (rantra.Y * body1.invInertiaWorld.M22)) + (rantra.Z * body1.invInertiaWorld.M32);
                    JFix64 num2 = ((rantra.X * body1.invInertiaWorld.M13) + (rantra.Y * body1.invInertiaWorld.M23)) + (rantra.Z * body1.invInertiaWorld.M33);

                    rantra.X = num0; rantra.Y = num1; rantra.Z = num2;

                    //JVector.Cross(ref rantra, ref relativePos1, out rantra);
                    num0 = (rantra.Y * relativePos1.Z) - (rantra.Z * relativePos1.Y);
                    num1 = (rantra.Z * relativePos1.X) - (rantra.X * relativePos1.Z);
                    num2 = (rantra.X * relativePos1.Y) - (rantra.Y * relativePos1.X);

                    rantra.X = num0; rantra.Y = num1; rantra.Z = num2;
                }

            }

            if (treatBody2AsStatic) rbntrb.MakeZero();
            else
            {
                kTangent += body2.inverseMass;

                if (!body2IsMassPoint)
                {
                    // JVector.Cross(ref relativePos1, ref normal, out rantra);
                    rbntrb.X = (relativePos2.Y * tangent.Z) - (relativePos2.Z * tangent.Y);
                    rbntrb.Y = (relativePos2.Z * tangent.X) - (relativePos2.X * tangent.Z);
                    rbntrb.Z = (relativePos2.X * tangent.Y) - (relativePos2.Y * tangent.X);

                    // JVector.Transform(ref rantra, ref body1.invInertiaWorld, out rantra);
                    JFix64 num0 = ((rbntrb.X * body2.invInertiaWorld.M11) + (rbntrb.Y * body2.invInertiaWorld.M21)) + (rbntrb.Z * body2.invInertiaWorld.M31);
                    JFix64 num1 = ((rbntrb.X * body2.invInertiaWorld.M12) + (rbntrb.Y * body2.invInertiaWorld.M22)) + (rbntrb.Z * body2.invInertiaWorld.M32);
                    JFix64 num2 = ((rbntrb.X * body2.invInertiaWorld.M13) + (rbntrb.Y * body2.invInertiaWorld.M23)) + (rbntrb.Z * body2.invInertiaWorld.M33);

                    rbntrb.X = num0; rbntrb.Y = num1; rbntrb.Z = num2;

                    //JVector.Cross(ref rantra, ref relativePos1, out rantra);
                    num0 = (rbntrb.Y * relativePos2.Z) - (rbntrb.Z * relativePos2.Y);
                    num1 = (rbntrb.Z * relativePos2.X) - (rbntrb.X * relativePos2.Z);
                    num2 = (rbntrb.X * relativePos2.Y) - (rbntrb.Y * relativePos2.X);

                    rbntrb.X = num0; rbntrb.Y = num1; rbntrb.Z = num2;
                }
            }

            if (!treatBody1AsStatic) kTangent += JVector.Dot(ref rantra, ref tangent);
            if (!treatBody2AsStatic) kTangent += JVector.Dot(ref rbntrb, ref tangent);
            massTangent = JFix64.One / kTangent;

            restitutionBias = lostSpeculativeBounce;

            speculativeVelocity = JFix64.Zero;

            JFix64 relNormalVel = normal.X * dvx + normal.Y * dvy + normal.Z * dvz; //JVector.Dot(ref normal, ref dv);

            if (Penetration > settings.allowedPenetration)
            {
                restitutionBias = settings.bias * (JFix64.One / timestep) * JFix64Math.Max(JFix64.Zero, Penetration - settings.allowedPenetration);
                restitutionBias = JFix64Math.Clamp(restitutionBias, JFix64.Zero, settings.maximumBias);
              //  body1IsMassPoint = body2IsMassPoint = false;
            }
      

            JFix64 timeStepRatio = timestep / lastTimeStep;
            accumulatedNormalImpulse *= timeStepRatio;
            accumulatedTangentImpulse *= timeStepRatio;

            {
                // Static/Dynamic friction
                JFix64 relTangentVel = -(tangent.X * dvx + tangent.Y * dvy + tangent.Z * dvz);
                JFix64 tangentImpulse = massTangent * relTangentVel;
                JFix64 maxTangentImpulse = -staticFriction * accumulatedNormalImpulse;

                if (tangentImpulse < maxTangentImpulse) friction = dynamicFriction;
                else friction = staticFriction;
            }

            JVector impulse;

            // Simultaneos solving and restitution is simply not possible
            // so fake it a bit by just applying restitution impulse when there
            // is a new contact.
            if (relNormalVel < -JFix64.One && newContact)
            {
                restitutionBias = JFix64Math.Max(-restitution * relNormalVel, restitutionBias);
            }

            // Speculative Contacts!
            // if the penetration is negative (which means the bodies are not already in contact, but they will
            // be in the future) we store the current bounce bias in the variable 'lostSpeculativeBounce'
            // and apply it the next frame, when the speculative contact was already solved.
            if (penetration < -settings.allowedPenetration)
            {
                speculativeVelocity = penetration / timestep;

                lostSpeculativeBounce = restitutionBias;
                restitutionBias = JFix64.Zero;
            }
            else
            {
                lostSpeculativeBounce = JFix64.Zero;
            }

            impulse.X = normal.X * accumulatedNormalImpulse + tangent.X * accumulatedTangentImpulse;
            impulse.Y = normal.Y * accumulatedNormalImpulse + tangent.Y * accumulatedTangentImpulse;
            impulse.Z = normal.Z * accumulatedNormalImpulse + tangent.Z * accumulatedTangentImpulse;

            if (!treatBody1AsStatic)
            {
                body1.linearVelocity.X -= (impulse.X * body1.inverseMass);
                body1.linearVelocity.Y -= (impulse.Y * body1.inverseMass);
                body1.linearVelocity.Z -= (impulse.Z * body1.inverseMass);

                if (!body1IsMassPoint)
                {
                    JFix64 num0, num1, num2;
                    num0 = relativePos1.Y * impulse.Z - relativePos1.Z * impulse.Y;
                    num1 = relativePos1.Z * impulse.X - relativePos1.X * impulse.Z;
                    num2 = relativePos1.X * impulse.Y - relativePos1.Y * impulse.X;

                    JFix64 num3 =
                        (((num0 * body1.invInertiaWorld.M11) +
                        (num1 * body1.invInertiaWorld.M21)) +
                        (num2 * body1.invInertiaWorld.M31));
                    JFix64 num4 =
                        (((num0 * body1.invInertiaWorld.M12) +
                        (num1 * body1.invInertiaWorld.M22)) +
                        (num2 * body1.invInertiaWorld.M32));
                    JFix64 num5 =
                        (((num0 * body1.invInertiaWorld.M13) +
                        (num1 * body1.invInertiaWorld.M23)) +
                        (num2 * body1.invInertiaWorld.M33));

                    body1.angularVelocity.X -= num3;
                    body1.angularVelocity.Y -= num4;
                    body1.angularVelocity.Z -= num5;

                }
            }

            if (!treatBody2AsStatic)
            {

                body2.linearVelocity.X += (impulse.X * body2.inverseMass);
                body2.linearVelocity.Y += (impulse.Y * body2.inverseMass);
                body2.linearVelocity.Z += (impulse.Z * body2.inverseMass);

                if (!body2IsMassPoint)
                {

                    JFix64 num0, num1, num2;
                    num0 = relativePos2.Y * impulse.Z - relativePos2.Z * impulse.Y;
                    num1 = relativePos2.Z * impulse.X - relativePos2.X * impulse.Z;
                    num2 = relativePos2.X * impulse.Y - relativePos2.Y * impulse.X;

                    JFix64 num3 =
                        (((num0 * body2.invInertiaWorld.M11) +
                        (num1 * body2.invInertiaWorld.M21)) +
                        (num2 * body2.invInertiaWorld.M31));
                    JFix64 num4 =
                        (((num0 * body2.invInertiaWorld.M12) +
                        (num1 * body2.invInertiaWorld.M22)) +
                        (num2 * body2.invInertiaWorld.M32));
                    JFix64 num5 =
                        (((num0 * body2.invInertiaWorld.M13) +
                        (num1 * body2.invInertiaWorld.M23)) +
                        (num2 * body2.invInertiaWorld.M33));

                    body2.angularVelocity.X += num3;
                    body2.angularVelocity.Y += num4;
                    body2.angularVelocity.Z += num5;
                }
            }

            lastTimeStep = timestep;

            newContact = false;
        }

        public void TreatBodyAsStatic(RigidBodyIndex index)
        {
            if (index == RigidBodyIndex.RigidBody1) treatBody1AsStatic = true;
            else treatBody2AsStatic = true;
        }


        /// <summary>
        /// Initializes a contact.
        /// </summary>
        /// <param name="body1">The first body.</param>
        /// <param name="body2">The second body.</param>
        /// <param name="point1">The collision point in worldspace</param>
        /// <param name="point2">The collision point in worldspace</param>
        /// <param name="n">The normal pointing to body2.</param>
        /// <param name="penetration">The estimated penetration depth.</param>
        public void Initialize(RigidBody body1, RigidBody body2, ref JVector point1, ref JVector point2, ref JVector n,
            JFix64 penetration, bool newContact, ContactSettings settings)
        {
            this.body1 = body1;  this.body2 = body2;
            this.normal = n; normal.Normalize();
            this.p1 = point1; this.p2 = point2;

            this.newContact = newContact;

            JVector.Subtract(ref p1, ref body1.position, out relativePos1);
            JVector.Subtract(ref p2, ref body2.position, out relativePos2);
            JVector.Transform(ref relativePos1, ref body1.invOrientation, out realRelPos1);
            JVector.Transform(ref relativePos2, ref body2.invOrientation, out realRelPos2);

            this.initialPen = penetration;
            this.penetration = penetration;

            body1IsMassPoint = body1.isParticle;
            body2IsMassPoint = body2.isParticle;

            // Material Properties
            if (newContact)
            {
                treatBody1AsStatic = body1.isStatic;
                treatBody2AsStatic = body2.isStatic;

                accumulatedNormalImpulse = JFix64.Zero;
                accumulatedTangentImpulse = JFix64.Zero;

                lostSpeculativeBounce = JFix64.Zero;

                switch (settings.MaterialCoefficientMixing)
                {
                    case ContactSettings.MaterialCoefficientMixingType.TakeMaximum:
                        staticFriction = JFix64Math.Max(body1.material.staticFriction, body2.material.staticFriction);
                        dynamicFriction = JFix64Math.Max(body1.material.kineticFriction, body2.material.kineticFriction);
                        restitution = JFix64Math.Max(body1.material.restitution, body2.material.restitution);
                        break;
                    case ContactSettings.MaterialCoefficientMixingType.TakeMinimum:
                        staticFriction = JFix64Math.Min(body1.material.staticFriction, body2.material.staticFriction);
                        dynamicFriction = JFix64Math.Min(body1.material.kineticFriction, body2.material.kineticFriction);
                        restitution = JFix64Math.Min(body1.material.restitution, body2.material.restitution);
                        break;
                    case ContactSettings.MaterialCoefficientMixingType.UseAverage:
                        staticFriction = (body1.material.staticFriction + body2.material.staticFriction) / (2 * JFix64.One);
                        dynamicFriction = (body1.material.kineticFriction + body2.material.kineticFriction) / (2 * JFix64.One);
                        restitution = (body1.material.restitution + body2.material.restitution) / (2 * JFix64.One);
                        break;
                }

            }

            this.settings = settings;
            


        }
    }
}
