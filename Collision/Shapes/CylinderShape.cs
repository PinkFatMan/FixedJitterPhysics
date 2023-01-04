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
    /// A <see cref="Shape"/> representing a cylinder.
    /// </summary>
    public class CylinderShape : Shape
    {
        private JFix64 height, radius;

        /// <summary>
        /// Sets the height of the cylinder.
        /// </summary>
        public JFix64 Height { get { return height; } set { height = value; UpdateShape(); } }

        /// <summary>
        /// Sets the radius of the cylinder.
        /// </summary>
        public JFix64 Radius { get { return radius; } set { radius = value; UpdateShape(); } }

        /// <summary>
        /// Initializes a new instance of the CylinderShape class.
        /// </summary>
        /// <param name="height">The height of the cylinder.</param>
        /// <param name="radius">The radius of the cylinder.</param>
        public CylinderShape(JFix64 height, JFix64 radius)
        {
            this.height = height;
            this.radius = radius;
            UpdateShape();
        }

        /// <summary>
        /// 
        /// </summary>
        public override void CalculateMassInertia()
        {
            this.mass = JFix64Math.Pi * radius * radius * height;
            this.inertia.M11 = (JFix64.One / (4 * JFix64.One)) * mass * radius * radius + (JFix64.One / (12 * JFix64.One)) * mass * height * height;
            this.inertia.M22 = (JFix64.One / (2 * JFix64.One)) * mass * radius * radius;
            this.inertia.M33 = (JFix64.One / (4 * JFix64.One)) * mass * radius * radius + (JFix64.One / (12 * JFix64.One)) * mass * height * height;
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
            JFix64 sigma = JFix64Math.Sqrt((direction.X * direction.X + direction.Z * direction.Z));

            if (sigma > JFix64.Zero)
            {
                result.X = direction.X / sigma * radius;
                result.Y = JFix64Math.Sign(direction.Y) * height * JFix64.Half;
                result.Z = direction.Z / sigma * radius;
            }
            else
            {
                result.X = JFix64.Zero;
                result.Y = JFix64Math.Sign(direction.Y) * height * JFix64.Half;
                result.Z = JFix64.Zero;
            }
        }
    }
}
