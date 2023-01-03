using System;

namespace Jitter.LinearMath {

    /// <summary>
    /// Contains common math operations.
    /// </summary>
    public sealed class JFix64Math {

        /// <summary>
        /// PI constant.
        /// </summary>
        public static JFix64 Pi = JFix64.Pi;

        /**
        *  @brief PI over 2 constant.
        **/
        public static JFix64 PiOver2 = JFix64.PiOver2;

        /// <summary>
        /// A small value often used to decide if numeric 
        /// results are zero.
        /// </summary>
		public static JFix64 Epsilon = JFix64.Epsilon;

        /**
        *  @brief Degree to radians constant.
        **/
        public static JFix64 Deg2Rad = JFix64.Deg2Rad;

        /**
        *  @brief Radians to degree constant.
        **/
        public static JFix64 Rad2Deg = JFix64.Rad2Deg;


        /**
         * @brief JFix64 infinity.
         * */
        public static JFix64 Infinity = JFix64.MaxValue;

        /// <summary>
        /// Gets the square root.
        /// </summary>
        /// <param name="number">The number to get the square root from.</param>
        /// <returns></returns>
        #region public static JFix64 Sqrt(JFix64 number)
        public static JFix64 Sqrt(JFix64 number) {
            return JFix64.Sqrt(number);
        }
        #endregion

        /// <summary>
        /// Gets the maximum number of two values.
        /// </summary>
        /// <param name="val1">The first value.</param>
        /// <param name="val2">The second value.</param>
        /// <returns>Returns the largest value.</returns>
        #region public static JFix64 Max(JFix64 val1, JFix64 val2)
        public static JFix64 Max(JFix64 val1, JFix64 val2) {
            return (val1 > val2) ? val1 : val2;
        }
        #endregion

        /// <summary>
        /// Gets the minimum number of two values.
        /// </summary>
        /// <param name="val1">The first value.</param>
        /// <param name="val2">The second value.</param>
        /// <returns>Returns the smallest value.</returns>
        #region public static JFix64 Min(JFix64 val1, JFix64 val2)
        public static JFix64 Min(JFix64 val1, JFix64 val2) {
            return (val1 < val2) ? val1 : val2;
        }
        #endregion

        /// <summary>
        /// Gets the maximum number of three values.
        /// </summary>
        /// <param name="val1">The first value.</param>
        /// <param name="val2">The second value.</param>
        /// <param name="val3">The third value.</param>
        /// <returns>Returns the largest value.</returns>
        #region public static JFix64 Max(JFix64 val1, JFix64 val2,JFix64 val3)
        public static JFix64 Max(JFix64 val1, JFix64 val2, JFix64 val3) {
            JFix64 max12 = (val1 > val2) ? val1 : val2;
            return (max12 > val3) ? max12 : val3;
        }
        #endregion

        /// <summary>
        /// Returns a number which is within [min,max]
        /// </summary>
        /// <param name="value">The value to clamp.</param>
        /// <param name="min">The minimum value.</param>
        /// <param name="max">The maximum value.</param>
        /// <returns>The clamped value.</returns>
        #region public static JFix64 Clamp(JFix64 value, JFix64 min, JFix64 max)
        public static JFix64 Clamp(JFix64 value, JFix64 min, JFix64 max) {
            if (value < min)
            {
                value = min;
                return value;
            }
            if (value > max)
            {
                value = max;
            }
            return value;
        }
        #endregion

        /// <summary>
        /// Returns a number which is within [JFix64.Zero, JFix64.One]
        /// </summary>
        /// <param name="value">The value to clamp.</param>
        /// <returns>The clamped value.</returns>
        public static JFix64 Clamp01(JFix64 value)
        {
            if (value < JFix64.Zero)
                return JFix64.Zero;

            if (value > JFix64.One)
                return JFix64.One;

            return value;
        }

        /// <summary>
        /// Changes every sign of the matrix entry to '+'
        /// </summary>
        /// <param name="matrix">The matrix.</param>
        /// <param name="result">The absolute matrix.</param>
        #region public static void Absolute(ref JMatrix matrix,out JMatrix result)
        
        public static void Absolute(ref JMatrix matrix, out JMatrix result) {
            result.M11 = JFix64.Abs(matrix.M11);
            result.M12 = JFix64.Abs(matrix.M12);
            result.M13 = JFix64.Abs(matrix.M13);
            result.M21 = JFix64.Abs(matrix.M21);
            result.M22 = JFix64.Abs(matrix.M22);
            result.M23 = JFix64.Abs(matrix.M23);
            result.M31 = JFix64.Abs(matrix.M31);
            result.M32 = JFix64.Abs(matrix.M32);
            result.M33 = JFix64.Abs(matrix.M33);
        }
        #endregion

        /// <summary>
        /// Returns the sine of value.
        /// </summary>
        public static JFix64 Sin(JFix64 value) {
            return JFix64.Sin(value);
        }

        /// <summary>
        /// Returns the cosine of value.
        /// </summary>
        public static JFix64 Cos(JFix64 value) {
            return JFix64.Cos(value);
        }

        /// <summary>
        /// Returns the tan of value.
        /// </summary>
        public static JFix64 Tan(JFix64 value) {
            return JFix64.Tan(value);
        }

        /// <summary>
        /// Returns the arc sine of value.
        /// </summary>
        public static JFix64 Asin(JFix64 value) {
            return JFix64.Asin(value);
        }

        /// <summary>
        /// Returns the arc cosine of value.
        /// </summary>
        public static JFix64 Acos(JFix64 value) {
            return JFix64.Acos(value);
        }

        /// <summary>
        /// Returns the arc tan of value.
        /// </summary>
        public static JFix64 Atan(JFix64 value) {
            return JFix64.Atan(value);
        }

        /// <summary>
        /// Returns the arc tan of coordinates x-y.
        /// </summary>
        public static JFix64 Atan2(JFix64 y, JFix64 x) {
            return JFix64.Atan2(y, x);
        }

        /// <summary>
        /// Returns the largest integer less than or equal to the specified number.
        /// </summary>
        public static JFix64 Floor(JFix64 value) {
            return JFix64.Floor(value);
        }

        /// <summary>
        /// Returns the smallest integral value that is greater than or equal to the specified number.
        /// </summary>
        public static JFix64 Ceiling(JFix64 value) {
            return value;
        }

        /// <summary>
        /// Rounds a value to the nearest integral value.
        /// If the value is halfway between an even and an uneven value, returns the even value.
        /// </summary>
        public static JFix64 Round(JFix64 value) {
            return JFix64.Round(value);
        }

        /// <summary>
        /// Returns a number indicating the sign of a Fix64 number.
        /// Returns 1 if the value is positive, 0 if is 0, and -1 if it is negative.
        /// </summary>
        public static int Sign(JFix64 value) {
            return JFix64.Sign(value);
        }

        /// <summary>
        /// Returns the absolute value of a Fix64 number.
        /// Note: Abs(Fix64.MinValue) == Fix64.MaxValue.
        /// </summary>
        public static JFix64 Abs(JFix64 value) {
            return JFix64.Abs(value);                
        }

        public static JFix64 Barycentric(JFix64 value1, JFix64 value2, JFix64 value3, JFix64 amount1, JFix64 amount2) {
            return value1 + (value2 - value1) * amount1 + (value3 - value1) * amount2;
        }

        public static JFix64 CatmullRom(JFix64 value1, JFix64 value2, JFix64 value3, JFix64 value4, JFix64 amount) {
            // Using formula from http://www.mvps.org/directx/articles/catmull/
            // Internally using JFix64s not to lose precission
            JFix64 amountSquared = amount * amount;
            JFix64 amountCubed = amountSquared * amount;
            return (JFix64)(0.5 * (2.0 * value2 +
                                 (value3 - value1) * amount +
                                 (2.0 * value1 - 5.0 * value2 + 4.0 * value3 - value4) * amountSquared +
                                 (3.0 * value2 - value1 - 3.0 * value3 + value4) * amountCubed));
        }

        public static JFix64 Distance(JFix64 value1, JFix64 value2) {
            return JFix64.Abs(value1 - value2);
        }

        public static JFix64 Hermite(JFix64 value1, JFix64 tangent1, JFix64 value2, JFix64 tangent2, JFix64 amount) {
            // All transformed to JFix64 not to lose precission
            // Otherwise, for high numbers of param:amount the result is NaN instead of Infinity
            JFix64 v1 = value1, v2 = value2, t1 = tangent1, t2 = tangent2, s = amount, result;
            JFix64 sCubed = s * s * s;
            JFix64 sSquared = s * s;

            if (amount == 0f)
                result = value1;
            else if (amount == 1f)
                result = value2;
            else
                result = (2 * v1 - 2 * v2 + t2 + t1) * sCubed +
                         (3 * v2 - 3 * v1 - 2 * t1 - t2) * sSquared +
                         t1 * s +
                         v1;
            return (JFix64)result;
        }

        public static JFix64 Lerp(JFix64 value1, JFix64 value2, JFix64 amount) {
            return value1 + (value2 - value1) * Clamp01(amount);
        }

        public static JFix64 InverseLerp(JFix64 value1, JFix64 value2, JFix64 amount) {
            if (value1 != value2)
                return Clamp01((amount - value1) / (value2 - value1));
            return JFix64.Zero;
        }

        public static JFix64 SmoothStep(JFix64 value1, JFix64 value2, JFix64 amount) {
            // It is expected that 0 < amount < 1
            // If amount < 0, return value1
            // If amount > 1, return value2
            JFix64 result = Clamp(amount, 0f, 1f);
            result = Hermite(value1, 0f, value2, 0f, result);
            return result;
        }


        /// <summary>
        /// Returns 2 raised to the specified power.
        /// Provides at least 6 decimals of accuracy.
        /// </summary>
        internal static JFix64 Pow2(JFix64 x)
        {
            if (x.RawValue == 0)
            {
                return JFix64.One;
            }

            // Avoid negative arguments by exploiting that exp(-x) = 1/exp(x).
            bool neg = x.RawValue < 0;
            if (neg)
            {
                x = -x;
            }

            if (x == JFix64.One)
            {
                return neg ? JFix64.One / (JFix64)2 : (JFix64)2;
            }
            if (x >= JFix64.Log2Max)
            {
                return neg ? JFix64.One / JFix64.MaxValue : JFix64.MaxValue;
            }
            if (x <= JFix64.Log2Min)
            {
                return neg ? JFix64.MaxValue : JFix64.Zero;
            }

            /* The algorithm is based on the power series for exp(x):
             * http://en.wikipedia.org/wiki/Exponential_function#Formal_definition
             * 
             * From term n, we get term n+1 by multiplying with x/n.
             * When the sum term drops to zero, we can stop summing.
             */

            int integerPart = (int)Floor(x);
            // Take fractional part of exponent
            x = JFix64.FromRaw(x.RawValue & 0x00000000FFFFFFFF);

            var result = JFix64.One;
            var term = JFix64.One;
            int i = 1;
            while (term.RawValue != 0)
            {
                term = JFix64.FastMul(JFix64.FastMul(x, term), JFix64.Ln2) / (JFix64)i;
                result += term;
                i++;
            }

            result = JFix64.FromRaw(result.RawValue << integerPart);
            if (neg)
            {
                result = JFix64.One / result;
            }

            return result;
        }

        /// <summary>
        /// Returns the base-2 logarithm of a specified number.
        /// Provides at least 9 decimals of accuracy.
        /// </summary>
        /// <exception cref="ArgumentOutOfRangeException">
        /// The argument was non-positive
        /// </exception>
        internal static JFix64 Log2(JFix64 x)
        {
            if (x.RawValue <= 0)
            {
                throw new ArgumentOutOfRangeException("Non-positive value passed to Ln", "x");
            }

            // This implementation is based on Clay. S. Turner's fast binary logarithm
            // algorithm (C. S. Turner,  "A Fast Binary Logarithm Algorithm", IEEE Signal
            //     Processing Mag., pp. 124,140, Sep. 2010.)

            long b = 1U << (JFix64.FRACTIONAL_PLACES - 1);
            long y = 0;

            long rawX = x.RawValue;
            while (rawX < JFix64.ONE)
            {
                rawX <<= 1;
                y -= JFix64.ONE;
            }

            while (rawX >= (JFix64.ONE << 1))
            {
                rawX >>= 1;
                y += JFix64.ONE;
            }

            var z = JFix64.FromRaw(rawX);

            for (int i = 0; i < JFix64.FRACTIONAL_PLACES; i++)
            {
                z = JFix64.FastMul(z, z);
                if (z.RawValue >= (JFix64.ONE << 1))
                {
                    z = JFix64.FromRaw(z.RawValue >> 1);
                    y += b;
                }
                b >>= 1;
            }

            return JFix64.FromRaw(y);
        }

        /// <summary>
        /// Returns the natural logarithm of a specified number.
        /// Provides at least 7 decimals of accuracy.
        /// </summary>
        /// <exception cref="ArgumentOutOfRangeException">
        /// The argument was non-positive
        /// </exception>
        public static JFix64 Ln(JFix64 x)
        {
            return JFix64.FastMul(Log2(x), JFix64.Ln2);
        }

        /// <summary>
        /// Returns a specified number raised to the specified power.
        /// Provides about 5 digits of accuracy for the result.
        /// </summary>
        /// <exception cref="DivideByZeroException">
        /// The base was zero, with a negative exponent
        /// </exception>
        /// <exception cref="ArgumentOutOfRangeException">
        /// The base was negative, with a non-zero exponent
        /// </exception>
        public static JFix64 Pow(JFix64 b, JFix64 exp)
        {
            if (b == JFix64.One)
            {
                return JFix64.One;
            }

            if (exp.RawValue == 0)
            {
                return JFix64.One;
            }

            if (b.RawValue == 0)
            {
                if (exp.RawValue < 0)
                {
                    //throw new DivideByZeroException();
                    return JFix64.MaxValue;
                }
                return JFix64.Zero;
            }

            JFix64 log2 = Log2(b);
            return Pow2(exp * log2);
        }

        public static JFix64 MoveTowards(JFix64 current, JFix64 target, JFix64 maxDelta)
        {
            if (Abs(target - current) <= maxDelta)
                return target;
            return (current + (Sign(target - current)) * maxDelta);
        }

        public static JFix64 Repeat(JFix64 t, JFix64 length)
        {
            return (t - (Floor(t / length) * length));
        }

        public static JFix64 DeltaAngle(JFix64 current, JFix64 target)
        {
            JFix64 num = Repeat(target - current, (JFix64)360f);
            if (num > (JFix64)180f)
            {
                num -= (JFix64)360f;
            }
            return num;
        }

        public static JFix64 MoveTowardsAngle(JFix64 current, JFix64 target, float maxDelta)
        {
            target = current + DeltaAngle(current, target);
            return MoveTowards(current, target, maxDelta);
        }

        public static JFix64 SmoothDamp(JFix64 current, JFix64 target, ref JFix64 currentVelocity, JFix64 smoothTime, JFix64 maxSpeed)
        {
            JFix64 deltaTime = JFix64.EN2;
            return SmoothDamp(current, target, ref currentVelocity, smoothTime, maxSpeed, deltaTime);
        }

        public static JFix64 SmoothDamp(JFix64 current, JFix64 target, ref JFix64 currentVelocity, JFix64 smoothTime)
        {
            JFix64 deltaTime = JFix64.EN2;
            JFix64 positiveInfinity = -JFix64.MaxValue;
            return SmoothDamp(current, target, ref currentVelocity, smoothTime, positiveInfinity, deltaTime);
        }

        public static JFix64 SmoothDamp(JFix64 current, JFix64 target, ref JFix64 currentVelocity, JFix64 smoothTime, JFix64 maxSpeed, JFix64 deltaTime)
        {
            smoothTime = Max(JFix64.EN4, smoothTime);
            JFix64 num = (JFix64)2f / smoothTime;
            JFix64 num2 = num * deltaTime;
            JFix64 num3 = JFix64.One / (((JFix64.One + num2) + (((JFix64)0.48f * num2) * num2)) + ((((JFix64)0.235f * num2) * num2) * num2));
            JFix64 num4 = current - target;
            JFix64 num5 = target;
            JFix64 max = maxSpeed * smoothTime;
            num4 = Clamp(num4, -max, max);
            target = current - num4;
            JFix64 num7 = (currentVelocity + (num * num4)) * deltaTime;
            currentVelocity = (currentVelocity - (num * num7)) * num3;
            JFix64 num8 = target + ((num4 + num7) * num3);
            if (((num5 - current) > JFix64.Zero) == (num8 > num5))
            {
                num8 = num5;
                currentVelocity = (num8 - num5) / deltaTime;
            }
            return num8;
        }
    }
}
