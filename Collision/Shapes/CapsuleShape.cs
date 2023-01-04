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

namespace Jitter.Collision.Shapes
{

    /// <summary>
    /// A <see cref="Shape"/> representing a capsule.
    /// </summary>
    public class CapsuleShape : Shape
    {
        private JFix64 length, radius;

        /// <summary>
        /// Gets or sets the length of the capsule (exclusive the round endcaps).
        /// </summary>
        public JFix64 Length { get { return length; } set { length = value; UpdateShape(); } }

        /// <summary>
        /// Gets or sets the radius of the endcaps.
        /// </summary>
        public JFix64 Radius { get { return radius; } set { radius = value; UpdateShape(); } }

        /// <summary>
        /// Create a new instance of the capsule.
        /// </summary>
        /// <param name="length">The length of the capsule (exclusive the round endcaps).</param>
        /// <param name="radius">The radius of the endcaps.</param>
        public CapsuleShape(JFix64 length,JFix64 radius)
        {
            this.length = length;
            this.radius = radius;
            UpdateShape();
        }

        /// <summary>
        /// 
        /// </summary>
        public override void CalculateMassInertia()
        {
            JFix64 massSphere = ((4 * JFix64.One) / (3 * JFix64.One)) * JFix64Math.Pi * radius * radius * radius;
            JFix64 massCylinder = JFix64Math.Pi * radius * radius * length;

            mass = massCylinder + massSphere;

            this.inertia.M11 = (JFix64.One / (4 * JFix64.One)) * massCylinder * radius * radius + (JFix64.One / (12 * JFix64.One)) * massCylinder * length * length + ((2 * JFix64.One) / (5 * JFix64.One)) * massSphere * radius * radius + (JFix64.One / (4 * JFix64.One)) * length * length * massSphere;
            this.inertia.M22 = (JFix64.One / (2 * JFix64.One)) * massCylinder * radius * radius + ((2 * JFix64.One) / (5 * JFix64.One)) * massSphere * radius * radius;
            this.inertia.M33 = (JFix64.One / (4 * JFix64.One)) * massCylinder * radius * radius + (JFix64.One / (12 * JFix64.One)) * massCylinder * length * length + ((2 * JFix64.One) / (5 * JFix64.One)) * massSphere * radius * radius + (JFix64.One / (4 * JFix64.One)) * length * length * massSphere;

            //this.inertia.M11 = (1.0f / 4.0f) * mass * radius * radius + (1.0f / 12.0f) * mass * height * height;
            //this.inertia.M22 = (1.0f / 2.0f) * mass * radius * radius;
            //this.inertia.M33 = (1.0f / 4.0f) * mass * radius * radius + (1.0f / 12.0f) * mass * height * height;
        }


        /// <summary>
        /// SupportMapping. Finds the point in the shape furthest away from the given direction.
        /// Imagine a plane with a normal in the search direction. Now move the plane along the normal
        /// until the plane does not intersect the shape. The last intersection point is the result.
        /// </summary>
        /// <param name="direction">The direction.</param>
        /// <param name="result">The result.</param>
        public override void SupportMapping(ref JVector direction, out JVector result)
        {
            JFix64 r = JFix64Math.Sqrt(direction.X * direction.X + direction.Z * direction.Z);

            if (JFix64Math.Abs(direction.Y) > JFix64.Zero)
            {
                JVector dir; JVector.Normalize(ref direction, out dir);
                JVector.Multiply(ref dir, radius, out result);
                result.Y += JFix64Math.Sign(direction.Y) * JFix64.Half * length;              
            }
            else if (r > JFix64.Zero)
            {
                result.X = direction.X / r * radius;
                result.Y = JFix64.Zero;
                result.Z = direction.Z / r * radius;
            }
            else
            {
                result.X = JFix64.Zero;
                result.Y = JFix64.Zero;
                result.Z = JFix64.Zero;
            }
        }
    }
}
