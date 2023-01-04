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

namespace Jitter.LinearMath
{

    /// <summary>
    /// A Quaternion representing an orientation. Member of the math 
    /// namespace, so every method has it's 'by reference' equivalent
    /// to speed up time critical math operations.
    /// </summary>
    public struct JQuaternion
    {

        /// <summary>The X component of the quaternion.</summary>
        public JFix64 X;
        /// <summary>The Y component of the quaternion.</summary>
        public JFix64 Y;
        /// <summary>The Z component of the quaternion.</summary>
        public JFix64 Z;
        /// <summary>The W component of the quaternion.</summary>
        public JFix64 W;

        static JQuaternion()
        {
        }

        /// <summary>
        /// Initializes a new instance of the JQuaternion structure.
        /// </summary>
        /// <param name="x">The X component of the quaternion.</param>
        /// <param name="y">The Y component of the quaternion.</param>
        /// <param name="z">The Z component of the quaternion.</param>
        /// <param name="w">The W component of the quaternion.</param>
        public JQuaternion(JFix64 x, JFix64 y, JFix64 z, JFix64 w)
        {
            this.X = x;
            this.Y = y;
            this.Z = z;
            this.W = w;
        }

        /// <summary>
        /// Quaternions are added.
        /// </summary>
        /// <param name="quaternion1">The first quaternion.</param>
        /// <param name="quaternion2">The second quaternion.</param>
        /// <returns>The sum of both quaternions.</returns>
        #region public static JQuaternion Add(JQuaternion quaternion1, JQuaternion quaternion2)
        public static JQuaternion Add(JQuaternion quaternion1, JQuaternion quaternion2)
        {
            JQuaternion result;
            JQuaternion.Add(ref quaternion1, ref quaternion2, out result);
            return result;
        }

        public static void CreateFromYawPitchRoll(JFix64 yaw, JFix64 pitch, JFix64 roll, out JQuaternion result)
        {
            JFix64 num9 = roll * JFix64.Half;
            JFix64 num6 = JFix64Math.Sin(num9);
            JFix64 num5 = JFix64Math.Cos(num9);
            JFix64 num8 = pitch * JFix64.Half;
            JFix64 num4 = JFix64Math.Sin(num8);
            JFix64 num3 = JFix64Math.Cos(num8);
            JFix64 num7 = yaw * JFix64.Half;
            JFix64 num2 = JFix64Math.Sin(num7);
            JFix64 num = JFix64Math.Cos(num7);
            result.X = ((num * num4) * num5) + ((num2 * num3) * num6);
            result.Y = ((num2 * num3) * num5) - ((num * num4) * num6);
            result.Z = ((num * num3) * num6) - ((num2 * num4) * num5);
            result.W = ((num * num3) * num5) + ((num2 * num4) * num6);
        }

 


        /// <summary>
        /// Quaternions are added.
        /// </summary>
        /// <param name="quaternion1">The first quaternion.</param>
        /// <param name="quaternion2">The second quaternion.</param>
        /// <param name="result">The sum of both quaternions.</param>
        public static void Add(ref JQuaternion quaternion1, ref JQuaternion quaternion2, out JQuaternion result)
        {
            result.X = quaternion1.X + quaternion2.X;
            result.Y = quaternion1.Y + quaternion2.Y;
            result.Z = quaternion1.Z + quaternion2.Z;
            result.W = quaternion1.W + quaternion2.W;
        }
        #endregion

        public static JQuaternion Conjugate(JQuaternion value)
        {
            JQuaternion quaternion;
            quaternion.X = -value.X;
            quaternion.Y = -value.Y;
            quaternion.Z = -value.Z;
            quaternion.W = value.W;
            return quaternion;
        }

        /// <summary>
        /// Quaternions are subtracted.
        /// </summary>
        /// <param name="quaternion1">The first quaternion.</param>
        /// <param name="quaternion2">The second quaternion.</param>
        /// <returns>The difference of both quaternions.</returns>
        #region public static JQuaternion Subtract(JQuaternion quaternion1, JQuaternion quaternion2)
        public static JQuaternion Subtract(JQuaternion quaternion1, JQuaternion quaternion2)
        {
            JQuaternion result;
            JQuaternion.Subtract(ref quaternion1, ref quaternion2, out result);
            return result;
        }

        /// <summary>
        /// Quaternions are subtracted.
        /// </summary>
        /// <param name="quaternion1">The first quaternion.</param>
        /// <param name="quaternion2">The second quaternion.</param>
        /// <param name="result">The difference of both quaternions.</param>
        public static void Subtract(ref JQuaternion quaternion1, ref JQuaternion quaternion2, out JQuaternion result)
        {
            result.X = quaternion1.X - quaternion2.X;
            result.Y = quaternion1.Y - quaternion2.Y;
            result.Z = quaternion1.Z - quaternion2.Z;
            result.W = quaternion1.W - quaternion2.W;
        }
        #endregion

        /// <summary>
        /// Multiply two quaternions.
        /// </summary>
        /// <param name="quaternion1">The first quaternion.</param>
        /// <param name="quaternion2">The second quaternion.</param>
        /// <returns>The product of both quaternions.</returns>
        #region public static JQuaternion Multiply(JQuaternion quaternion1, JQuaternion quaternion2)
        public static JQuaternion Multiply(JQuaternion quaternion1, JQuaternion quaternion2)
        {
            JQuaternion result;
            JQuaternion.Multiply(ref quaternion1, ref quaternion2, out result);
            return result;
        }

        /// <summary>
        /// Multiply two quaternions.
        /// </summary>
        /// <param name="quaternion1">The first quaternion.</param>
        /// <param name="quaternion2">The second quaternion.</param>
        /// <param name="result">The product of both quaternions.</param>
        public static void Multiply(ref JQuaternion quaternion1, ref JQuaternion quaternion2, out JQuaternion result)
        {
            JFix64 x = quaternion1.X;
            JFix64 y = quaternion1.Y;
            JFix64 z = quaternion1.Z;
            JFix64 w = quaternion1.W;
            JFix64 num4 = quaternion2.X;
            JFix64 num3 = quaternion2.Y;
            JFix64 num2 = quaternion2.Z;
            JFix64 num = quaternion2.W;
            JFix64 num12 = (y * num2) - (z * num3);
            JFix64 num11 = (z * num4) - (x * num2);
            JFix64 num10 = (x * num3) - (y * num4);
            JFix64 num9 = ((x * num4) + (y * num3)) + (z * num2);
            result.X = ((x * num) + (num4 * w)) + num12;
            result.Y = ((y * num) + (num3 * w)) + num11;
            result.Z = ((z * num) + (num2 * w)) + num10;
            result.W = (w * num) - num9;
        }
        #endregion

        /// <summary>
        /// Scale a quaternion
        /// </summary>
        /// <param name="quaternion1">The quaternion to scale.</param>
        /// <param name="scaleFactor">Scale factor.</param>
        /// <returns>The scaled quaternion.</returns>
        #region public static JQuaternion Multiply(JQuaternion quaternion1, JFix64 scaleFactor)
        public static JQuaternion Multiply(JQuaternion quaternion1, JFix64 scaleFactor)
        {
            JQuaternion result;
            JQuaternion.Multiply(ref quaternion1, scaleFactor, out result);
            return result;
        }

        /// <summary>
        /// Scale a quaternion
        /// </summary>
        /// <param name="quaternion1">The quaternion to scale.</param>
        /// <param name="scaleFactor">Scale factor.</param>
        /// <param name="result">The scaled quaternion.</param>
        public static void Multiply(ref JQuaternion quaternion1, JFix64 scaleFactor, out JQuaternion result)
        {
            result.X = quaternion1.X * scaleFactor;
            result.Y = quaternion1.Y * scaleFactor;
            result.Z = quaternion1.Z * scaleFactor;
            result.W = quaternion1.W * scaleFactor;
        }
        #endregion

        /// <summary>
        /// Sets the length of the quaternion to one.
        /// </summary>
        #region public void Normalize()
        public void Normalize()
        {
            JFix64 num2 = (((this.X * this.X) + (this.Y * this.Y)) + (this.Z * this.Z)) + (this.W * this.W);
            JFix64 num = JFix64.One / (JFix64Math.Sqrt(num2));
            this.X *= num;
            this.Y *= num;
            this.Z *= num;
            this.W *= num;
        }
        #endregion

        /// <summary>
        /// Creates a quaternion from a matrix.
        /// </summary>
        /// <param name="matrix">A matrix representing an orientation.</param>
        /// <returns>JQuaternion representing an orientation.</returns>
        #region public static JQuaternion CreateFromMatrix(JMatrix matrix)
        public static JQuaternion CreateFromMatrix(JMatrix matrix)
        {
            JQuaternion result;
            JQuaternion.CreateFromMatrix(ref matrix, out result);
            return result;
        }

        /// <summary>
        /// Creates a quaternion from a matrix.
        /// </summary>
        /// <param name="matrix">A matrix representing an orientation.</param>
        /// <param name="result">JQuaternion representing an orientation.</param>
        public static void CreateFromMatrix(ref JMatrix matrix, out JQuaternion result)
        {
            JFix64 num8 = (matrix.M11 + matrix.M22) + matrix.M33;
            if (num8 > JFix64.Zero)
            {
                JFix64 num = JFix64Math.Sqrt((num8 + JFix64.One));
                result.W = num * JFix64.Half;
                num = JFix64.Half / num;
                result.X = (matrix.M23 - matrix.M32) * num;
                result.Y = (matrix.M31 - matrix.M13) * num;
                result.Z = (matrix.M12 - matrix.M21) * num;
            }
            else if ((matrix.M11 >= matrix.M22) && (matrix.M11 >= matrix.M33))
            {
                JFix64 num7 = JFix64Math.Sqrt((((JFix64.One + matrix.M11) - matrix.M22) - matrix.M33));
                JFix64 num4 = JFix64.Half / num7;
                result.X = JFix64.Half * num7;
                result.Y = (matrix.M12 + matrix.M21) * num4;
                result.Z = (matrix.M13 + matrix.M31) * num4;
                result.W = (matrix.M23 - matrix.M32) * num4;
            }
            else if (matrix.M22 > matrix.M33)
            {
                JFix64 num6 = JFix64Math.Sqrt((((JFix64.One + matrix.M22) - matrix.M11) - matrix.M33));
                JFix64 num3 = JFix64.Half / num6;
                result.X = (matrix.M21 + matrix.M12) * num3;
                result.Y = JFix64.Half * num6;
                result.Z = (matrix.M32 + matrix.M23) * num3;
                result.W = (matrix.M31 - matrix.M13) * num3;
            }
            else
            {
                JFix64 num5 = JFix64Math.Sqrt((((JFix64.One + matrix.M33) - matrix.M11) - matrix.M22));
                JFix64 num2 = JFix64.Half / num5;
                result.X = (matrix.M31 + matrix.M13) * num2;
                result.Y = (matrix.M32 + matrix.M23) * num2;
                result.Z = JFix64.Half * num5;
                result.W = (matrix.M12 - matrix.M21) * num2;
            }
        }
        #endregion

        /// <summary>
        /// Multiply two quaternions.
        /// </summary>
        /// <param name="value1">The first quaternion.</param>
        /// <param name="value2">The second quaternion.</param>
        /// <returns>The product of both quaternions.</returns>
        #region public static JFix64 operator *(JQuaternion value1, JQuaternion value2)
        public static JQuaternion operator *(JQuaternion value1, JQuaternion value2)
        {
            JQuaternion result;
            JQuaternion.Multiply(ref value1, ref value2,out result);
            return result;
        }
        #endregion

        /// <summary>
        /// Add two quaternions.
        /// </summary>
        /// <param name="value1">The first quaternion.</param>
        /// <param name="value2">The second quaternion.</param>
        /// <returns>The sum of both quaternions.</returns>
        #region public static JFix64 operator +(JQuaternion value1, JQuaternion value2)
        public static JQuaternion operator +(JQuaternion value1, JQuaternion value2)
        {
            JQuaternion result;
            JQuaternion.Add(ref value1, ref value2, out result);
            return result;
        }
        #endregion

        /// <summary>
        /// Subtract two quaternions.
        /// </summary>
        /// <param name="value1">The first quaternion.</param>
        /// <param name="value2">The second quaternion.</param>
        /// <returns>The difference of both quaternions.</returns>
        #region public static JFix64 operator -(JQuaternion value1, JQuaternion value2)
        public static JQuaternion operator -(JQuaternion value1, JQuaternion value2)
        {
            JQuaternion result;
            JQuaternion.Subtract(ref value1, ref value2, out result);
            return result;
        }
        #endregion

    }
}
