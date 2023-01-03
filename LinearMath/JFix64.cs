using System;
using System.IO;

namespace Jitter.LinearMath
{
     /// <summary>
    /// Represents a Q31.32 fixed-point number.
    /// </summary>
    [Serializable]
    public partial struct JFix64 : IEquatable<JFix64>, IComparable<JFix64> {
         public long SerializedValue;

		public const long MAX_VALUE = long.MaxValue;
		public const long MIN_VALUE = long.MinValue;
		public const int NUM_BITS = 64;
		public const int FRACTIONAL_PLACES = 32;
		public const long ONE = 1L << FRACTIONAL_PLACES;
		public const long TEN = 10L << FRACTIONAL_PLACES;
		public const long HALF = 1L << (FRACTIONAL_PLACES - 1);
		public const long PI_TIMES_2 = 0x6487ED511;
		public const long PI = 0x3243F6A88;
		public const long PI_OVER_2 = 0x1921FB544;
        public const long LN2 = 0xB17217F7;
        public const long LOG2MAX = 0x1F00000000;
        public const long LOG2MIN = -0x2000000000;
        public const int LUT_SIZE = (int)(PI_OVER_2 >> 15);

        // Precision of this type is 2^-32, that is 2,3283064365386962890625E-10
        public static readonly decimal Precision = (decimal)(new JFix64(1L));//0.00000000023283064365386962890625m;
        public static readonly JFix64 MaxValue = new JFix64(MAX_VALUE-1);
        public static readonly JFix64 MinValue = new JFix64(MIN_VALUE+2);
        public static readonly JFix64 One = new JFix64(ONE);
		public static readonly JFix64 Ten = new JFix64(TEN);
        public static readonly JFix64 Half = new JFix64(HALF);

        public static readonly JFix64 Zero = new JFix64();
        public static readonly JFix64 PositiveInfinity = new JFix64(MAX_VALUE);
        public static readonly JFix64 NegativeInfinity = new JFix64(MIN_VALUE+1);
        public static readonly JFix64 NaN = new JFix64(MIN_VALUE);

        public static readonly JFix64 EN1 = JFix64.One / 10;
        public static readonly JFix64 EN2 = JFix64.One / 100;
        public static readonly JFix64 EN3 = JFix64.One / 1000;
        public static readonly JFix64 EN4 = JFix64.One / 10000;
        public static readonly JFix64 EN5 = JFix64.One / 100000;
        public static readonly JFix64 EN6 = JFix64.One / 1000000;
        public static readonly JFix64 EN7 = JFix64.One / 10000000;
        public static readonly JFix64 EN8 = JFix64.One / 100000000;
        public static readonly JFix64 Epsilon = JFix64.EN4;

        /// <summary>
        /// The value of Pi
        /// </summary>
        public static readonly JFix64 Pi = new JFix64(PI);
        public static readonly JFix64 PiOver2 = new JFix64(PI_OVER_2);
        public static readonly JFix64 PiTimes2 = new JFix64(PI_TIMES_2);
        public static readonly JFix64 PiInv = (JFix64)0.3183098861837906715377675267M;
        public static readonly JFix64 PiOver2Inv = (JFix64)0.6366197723675813430755350535M;

        public static readonly JFix64 Deg2Rad = Pi / new JFix64(180);

        public static readonly JFix64 Rad2Deg = new JFix64(180) / Pi;

		public static readonly JFix64 LutInterval = (JFix64)(LUT_SIZE - 1) / PiOver2;

        public static readonly JFix64 Log2Max = new JFix64(LOG2MAX);
        public static readonly JFix64 Log2Min = new JFix64(LOG2MIN);
        public static readonly JFix64 Ln2 = new JFix64(LN2);

        /// <summary>
        /// Returns a number indicating the sign of a Fix64 number.
        /// Returns 1 if the value is positive, 0 if is 0, and -1 if it is negative.
        /// </summary>
        public static int Sign(JFix64 value) {
            return
                value.SerializedValue < 0 ? -1 :
                value.SerializedValue > 0 ? 1 :
                0;
        }


        /// <summary>
        /// Returns the absolute value of a Fix64 number.
        /// Note: Abs(Fix64.MinValue) == Fix64.MaxValue.
        /// </summary>
        public static JFix64 Abs(JFix64 value) {
            if (value.SerializedValue == MIN_VALUE) {
                return MaxValue;
            }

            // branchless implementation, see http://www.strchr.com/optimized_abs_function
            var mask = value.SerializedValue >> 63;
            JFix64 result;
            result.SerializedValue = (value.SerializedValue + mask) ^ mask;
            return result;
            //return new FP((value.SerializedValue + mask) ^ mask);
        }

        /// <summary>
        /// Returns the absolute value of a Fix64 number.
        /// FastAbs(Fix64.MinValue) is undefined.
        /// </summary>
        public static JFix64 FastAbs(JFix64 value) {
            // branchless implementation, see http://www.strchr.com/optimized_abs_function
            var mask = value.SerializedValue >> 63;
            JFix64 result;
            result.SerializedValue = (value.SerializedValue + mask) ^ mask;
            return result;
            //return new FP((value.SerializedValue + mask) ^ mask);
        }


        /// <summary>
        /// Returns the largest integer less than or equal to the specified number.
        /// </summary>
        public static JFix64 Floor(JFix64 value) {
            // Just zero out the fractional part
            JFix64 result;
            result.SerializedValue = (long)((ulong)value.SerializedValue & 0xFFFFFFFF00000000);
            return result;
            //return new FP((long)((ulong)value.SerializedValue & 0xFFFFFFFF00000000));
        }

        /// <summary>
        /// Returns the smallest integral value that is greater than or equal to the specified number.
        /// </summary>
        public static JFix64 Ceiling(JFix64 value) {
            var hasFractionalPart = (value.SerializedValue & 0x00000000FFFFFFFF) != 0;
            return hasFractionalPart ? Floor(value) + One : value;
        }

        /// <summary>
        /// Rounds a value to the nearest integral value.
        /// If the value is halfway between an even and an uneven value, returns the even value.
        /// </summary>
        public static JFix64 Round(JFix64 value) {
            var fractionalPart = value.SerializedValue & 0x00000000FFFFFFFF;
            var integralPart = Floor(value);
            if (fractionalPart < 0x80000000) {
                return integralPart;
            }
            if (fractionalPart > 0x80000000) {
                return integralPart + One;
            }
            // if number is halfway between two values, round to the nearest even number
            // this is the method used by System.Math.Round().
            return (integralPart.SerializedValue & ONE) == 0
                       ? integralPart
                       : integralPart + One;
        }

        /// <summary>
        /// Adds x and y. Performs saturating addition, i.e. in case of overflow, 
        /// rounds to MinValue or MaxValue depending on sign of operands.
        /// </summary>
        public static JFix64 operator +(JFix64 x, JFix64 y) {
            JFix64 result;
            result.SerializedValue = x.SerializedValue + y.SerializedValue;
            return result;
            //return new FP(x.SerializedValue + y.SerializedValue);
        }

        /// <summary>
        /// Adds x and y performing overflow checking. Should be inlined by the CLR.
        /// </summary>
        public static JFix64 OverflowAdd(JFix64 x, JFix64 y) {
            var xl = x.SerializedValue;
            var yl = y.SerializedValue;
            var sum = xl + yl;
            // if signs of operands are equal and signs of sum and x are different
            if (((~(xl ^ yl) & (xl ^ sum)) & MIN_VALUE) != 0) {
                sum = xl > 0 ? MAX_VALUE : MIN_VALUE;
            }
            JFix64 result;
            result.SerializedValue = sum;
            return result;
            //return new FP(sum);
        }

        /// <summary>
        /// Adds x and y witout performing overflow checking. Should be inlined by the CLR.
        /// </summary>
        public static JFix64 FastAdd(JFix64 x, JFix64 y) {
            JFix64 result;
            result.SerializedValue = x.SerializedValue + y.SerializedValue;
            return result;
            //return new FP(x.SerializedValue + y.SerializedValue);
        }

        /// <summary>
        /// Subtracts y from x. Performs saturating substraction, i.e. in case of overflow, 
        /// rounds to MinValue or MaxValue depending on sign of operands.
        /// </summary>
        public static JFix64 operator -(JFix64 x, JFix64 y) {
            JFix64 result;
            result.SerializedValue = x.SerializedValue - y.SerializedValue;
            return result;
            //return new FP(x.SerializedValue - y.SerializedValue);
        }

        /// <summary>
        /// Subtracts y from x witout performing overflow checking. Should be inlined by the CLR.
        /// </summary>
        public static JFix64 OverflowSub(JFix64 x, JFix64 y) {
            var xl = x.SerializedValue;
            var yl = y.SerializedValue;
            var diff = xl - yl;
            // if signs of operands are different and signs of sum and x are different
            if ((((xl ^ yl) & (xl ^ diff)) & MIN_VALUE) != 0) {
                diff = xl < 0 ? MIN_VALUE : MAX_VALUE;
            }
            JFix64 result;
            result.SerializedValue = diff;
            return result;
            //return new FP(diff);
        }

        /// <summary>
        /// Subtracts y from x witout performing overflow checking. Should be inlined by the CLR.
        /// </summary>
        public static JFix64 FastSub(JFix64 x, JFix64 y) {
            return new JFix64(x.SerializedValue - y.SerializedValue);
        }

        static long AddOverflowHelper(long x, long y, ref bool overflow) {
            var sum = x + y;
            // x + y overflows if sign(x) ^ sign(y) != sign(sum)
            overflow |= ((x ^ y ^ sum) & MIN_VALUE) != 0;
            return sum;
        }

        public static JFix64 operator *(JFix64 x, JFix64 y) {
            var xl = x.SerializedValue;
            var yl = y.SerializedValue;

            var xlo = (ulong)(xl & 0x00000000FFFFFFFF);
            var xhi = xl >> FRACTIONAL_PLACES;
            var ylo = (ulong)(yl & 0x00000000FFFFFFFF);
            var yhi = yl >> FRACTIONAL_PLACES;

            var lolo = xlo * ylo;
            var lohi = (long)xlo * yhi;
            var hilo = xhi * (long)ylo;
            var hihi = xhi * yhi;

            var loResult = lolo >> FRACTIONAL_PLACES;
            var midResult1 = lohi;
            var midResult2 = hilo;
            var hiResult = hihi << FRACTIONAL_PLACES;

            var sum = (long)loResult + midResult1 + midResult2 + hiResult;
            JFix64 result;// = default(FP);
            result.SerializedValue = sum;
            return result;
        }

        /// <summary>
        /// Performs multiplication without checking for overflow.
        /// Useful for performance-critical code where the values are guaranteed not to cause overflow
        /// </summary>
        public static JFix64 OverflowMul(JFix64 x, JFix64 y) {
            var xl = x.SerializedValue;
            var yl = y.SerializedValue;

            var xlo = (ulong)(xl & 0x00000000FFFFFFFF);
            var xhi = xl >> FRACTIONAL_PLACES;
            var ylo = (ulong)(yl & 0x00000000FFFFFFFF);
            var yhi = yl >> FRACTIONAL_PLACES;

            var lolo = xlo * ylo;
            var lohi = (long)xlo * yhi;
            var hilo = xhi * (long)ylo;
            var hihi = xhi * yhi;

            var loResult = lolo >> FRACTIONAL_PLACES;
            var midResult1 = lohi;
            var midResult2 = hilo;
            var hiResult = hihi << FRACTIONAL_PLACES;

            bool overflow = false;
            var sum = AddOverflowHelper((long)loResult, midResult1, ref overflow);
            sum = AddOverflowHelper(sum, midResult2, ref overflow);
            sum = AddOverflowHelper(sum, hiResult, ref overflow);

            bool opSignsEqual = ((xl ^ yl) & MIN_VALUE) == 0;

            // if signs of operands are equal and sign of result is negative,
            // then multiplication overflowed positively
            // the reverse is also true
            if (opSignsEqual) {
                if (sum < 0 || (overflow && xl > 0)) {
                    return MaxValue;
                }
            } else {
                if (sum > 0) {
                    return MinValue;
                }
            }

            // if the top 32 bits of hihi (unused in the result) are neither all 0s or 1s,
            // then this means the result overflowed.
            var topCarry = hihi >> FRACTIONAL_PLACES;
            if (topCarry != 0 && topCarry != -1 /*&& xl != -17 && yl != -17*/) {
                return opSignsEqual ? MaxValue : MinValue;
            }

            // If signs differ, both operands' magnitudes are greater than 1,
            // and the result is greater than the negative operand, then there was negative overflow.
            if (!opSignsEqual) {
                long posOp, negOp;
                if (xl > yl) {
                    posOp = xl;
                    negOp = yl;
                } else {
                    posOp = yl;
                    negOp = xl;
                }
                if (sum > negOp && negOp < -ONE && posOp > ONE) {
                    return MinValue;
                }
            }
            JFix64 result;
            result.SerializedValue = sum;
            return result;
            //return new FP(sum);
        }

        /// <summary>
        /// Performs multiplication without checking for overflow.
        /// Useful for performance-critical code where the values are guaranteed not to cause overflow
        /// </summary>
        public static JFix64 FastMul(JFix64 x, JFix64 y) {
            var xl = x.SerializedValue;
            var yl = y.SerializedValue;

            var xlo = (ulong)(xl & 0x00000000FFFFFFFF);
            var xhi = xl >> FRACTIONAL_PLACES;
            var ylo = (ulong)(yl & 0x00000000FFFFFFFF);
            var yhi = yl >> FRACTIONAL_PLACES;

            var lolo = xlo * ylo;
            var lohi = (long)xlo * yhi;
            var hilo = xhi * (long)ylo;
            var hihi = xhi * yhi;

            var loResult = lolo >> FRACTIONAL_PLACES;
            var midResult1 = lohi;
            var midResult2 = hilo;
            var hiResult = hihi << FRACTIONAL_PLACES;

            var sum = (long)loResult + midResult1 + midResult2 + hiResult;
            JFix64 result;// = default(FP);
			result.SerializedValue = sum;
			return result;
            //return new FP(sum);
        }

        //[MethodImplAttribute(MethodImplOptions.AggressiveInlining)] 
        public static int CountLeadingZeroes(ulong x) {
            int result = 0;
            while ((x & 0xF000000000000000) == 0) { result += 4; x <<= 4; }
            while ((x & 0x8000000000000000) == 0) { result += 1; x <<= 1; }
            return result;
        }

        public static JFix64 operator /(JFix64 x, JFix64 y) {
            var xl = x.SerializedValue;
            var yl = y.SerializedValue;

            if (yl == 0) {
                return MAX_VALUE;
                //throw new DivideByZeroException();
            }

            var remainder = (ulong)(xl >= 0 ? xl : -xl);
            var divider = (ulong)(yl >= 0 ? yl : -yl);
            var quotient = 0UL;
            var bitPos = NUM_BITS / 2 + 1;


            // If the divider is divisible by 2^n, take advantage of it.
            while ((divider & 0xF) == 0 && bitPos >= 4) {
                divider >>= 4;
                bitPos -= 4;
            }

            while (remainder != 0 && bitPos >= 0) {
                int shift = CountLeadingZeroes(remainder);
                if (shift > bitPos) {
                    shift = bitPos;
                }
                remainder <<= shift;
                bitPos -= shift;

                var div = remainder / divider;
                remainder = remainder % divider;
                quotient += div << bitPos;

                // Detect overflow
                if ((div & ~(0xFFFFFFFFFFFFFFFF >> bitPos)) != 0) {
                    return ((xl ^ yl) & MIN_VALUE) == 0 ? MaxValue : MinValue;
                }

                remainder <<= 1;
                --bitPos;
            }

            // rounding
            ++quotient;
            var result = (long)(quotient >> 1);
            if (((xl ^ yl) & MIN_VALUE) != 0) {
                result = -result;
            }

            JFix64 r;
            r.SerializedValue = result;
            return r;
            //return new FP(result);
        }

        public static JFix64 operator %(JFix64 x, JFix64 y) {
            JFix64 result;
            result.SerializedValue = x.SerializedValue == MIN_VALUE & y.SerializedValue == -1 ?
                0 :
                x.SerializedValue % y.SerializedValue;
            return result;
            //return new FP(
            //    x.SerializedValue == MIN_VALUE & y.SerializedValue == -1 ?
            //    0 :
            //    x.SerializedValue % y.SerializedValue);
        }

        /// <summary>
        /// Performs modulo as fast as possible; throws if x == MinValue and y == -1.
        /// Use the operator (%) for a more reliable but slower modulo.
        /// </summary>
        public static JFix64 FastMod(JFix64 x, JFix64 y) {
            JFix64 result;
            result.SerializedValue = x.SerializedValue % y.SerializedValue;
            return result;
            //return new FP(x.SerializedValue % y.SerializedValue);
        }

        public static JFix64 operator -(JFix64 x) {
            return x.SerializedValue == MIN_VALUE ? MaxValue : new JFix64(-x.SerializedValue);
        }

        public static bool operator ==(JFix64 x, JFix64 y) {
            return x.SerializedValue == y.SerializedValue;
        }

        public static bool operator !=(JFix64 x, JFix64 y) {
            return x.SerializedValue != y.SerializedValue;
        }

        public static bool operator >(JFix64 x, JFix64 y) {
            return x.SerializedValue > y.SerializedValue;
        }

        public static bool operator <(JFix64 x, JFix64 y) {
            return x.SerializedValue < y.SerializedValue;
        }

        public static bool operator >=(JFix64 x, JFix64 y) {
            return x.SerializedValue >= y.SerializedValue;
        }

        public static bool operator <=(JFix64 x, JFix64 y) {
            return x.SerializedValue <= y.SerializedValue;
        }


        /// <summary>
        /// Returns the square root of a specified number.
        /// </summary>
        /// <exception cref="ArgumentOutOfRangeException">
        /// The argument was negative.
        /// </exception>
        public static JFix64 Sqrt(JFix64 x) {
            var xl = x.SerializedValue;
            if (xl < 0) {
                // We cannot represent infinities like Single and Double, and Sqrt is
                // mathematically undefined for x < 0. So we just throw an exception.
                throw new ArgumentOutOfRangeException("Negative value passed to Sqrt", "x");
            }

            var num = (ulong)xl;
            var result = 0UL;

            // second-to-top bit
            var bit = 1UL << (NUM_BITS - 2);

            while (bit > num) {
                bit >>= 2;
            }

            // The main part is executed twice, in order to avoid
            // using 128 bit values in computations.
            for (var i = 0; i < 2; ++i) {
                // First we get the top 48 bits of the answer.
                while (bit != 0) {
                    if (num >= result + bit) {
                        num -= result + bit;
                        result = (result >> 1) + bit;
                    } else {
                        result = result >> 1;
                    }
                    bit >>= 2;
                }

                if (i == 0) {
                    // Then process it again to get the lowest 16 bits.
                    if (num > (1UL << (NUM_BITS / 2)) - 1) {
                        // The remainder 'num' is too large to be shifted left
                        // by 32, so we have to add 1 to result manually and
                        // adjust 'num' accordingly.
                        // num = a - (result + 0.5)^2
                        //       = num + result^2 - (result + 0.5)^2
                        //       = num - result - 0.5
                        num -= result;
                        num = (num << (NUM_BITS / 2)) - 0x80000000UL;
                        result = (result << (NUM_BITS / 2)) + 0x80000000UL;
                    } else {
                        num <<= (NUM_BITS / 2);
                        result <<= (NUM_BITS / 2);
                    }

                    bit = 1UL << (NUM_BITS / 2 - 2);
                }
            }
            // Finally, if next bit would have been 1, round the result upwards.
            if (num > result) {
                ++result;
            }

            JFix64 r;
            r.SerializedValue = (long)result;
            return r;
            //return new FP((long)result);
        }

        /// <summary>
        /// Returns the Sine of x.
        /// This function has about 9 decimals of accuracy for small values of x.
        /// It may lose accuracy as the value of x grows.
        /// Performance: about 25% slower than Math.Sin() in x64, and 200% slower in x86.
        /// </summary>
        public static JFix64 Sin(JFix64 x) {
            bool flipHorizontal, flipVertical;
            var clampedL = ClampSinValue(x.SerializedValue, out flipHorizontal, out flipVertical);
            var clamped = new JFix64(clampedL);

            // Find the two closest values in the LUT and perform linear interpolation
            // This is what kills the performance of this function on x86 - x64 is fine though
            var rawIndex = FastMul(clamped, LutInterval);
            var roundedIndex = Round(rawIndex);
            var indexError = 0;//FastSub(rawIndex, roundedIndex);

            var nearestValue = new JFix64(SinLut[flipHorizontal ?
                SinLut.Length - 1 - (int)roundedIndex :
                (int)roundedIndex]);
            var secondNearestValue = new JFix64(SinLut[flipHorizontal ?
                SinLut.Length - 1 - (int)roundedIndex - Sign(indexError) :
                (int)roundedIndex + Sign(indexError)]);

            var delta = FastMul(indexError, FastAbs(FastSub(nearestValue, secondNearestValue))).SerializedValue;
            var interpolatedValue = nearestValue.SerializedValue + (flipHorizontal ? -delta : delta);
            var finalValue = flipVertical ? -interpolatedValue : interpolatedValue;

            //FP a2 = new FP(finalValue);
            JFix64 a2;
            a2.SerializedValue = finalValue;
            return a2;
        }

        /// <summary>
        /// Returns a rough approximation of the Sine of x.
        /// This is at least 3 times faster than Sin() on x86 and slightly faster than Math.Sin(),
        /// however its accuracy is limited to 4-5 decimals, for small enough values of x.
        /// </summary>
        public static JFix64 FastSin(JFix64 x) {
            bool flipHorizontal, flipVertical;
            var clampedL = ClampSinValue(x.SerializedValue, out flipHorizontal, out flipVertical);

            // Here we use the fact that the SinLut table has a number of entries
            // equal to (PI_OVER_2 >> 15) to use the angle to index directly into it
            var rawIndex = (uint)(clampedL >> 15);
            if (rawIndex >= LUT_SIZE) {
                rawIndex = LUT_SIZE - 1;
            }
            var nearestValue = SinLut[flipHorizontal ?
                SinLut.Length - 1 - (int)rawIndex :
                (int)rawIndex];

            JFix64 result;
            result.SerializedValue = flipVertical ? -nearestValue : nearestValue;
            return result;
            //return new FP(flipVertical ? -nearestValue : nearestValue);
        }



        //[MethodImplAttribute(MethodImplOptions.AggressiveInlining)] 
        public static long ClampSinValue(long angle, out bool flipHorizontal, out bool flipVertical) {
            // Clamp value to 0 - 2*PI using modulo; this is very slow but there's no better way AFAIK
            var clamped2Pi = angle % PI_TIMES_2;
            if (angle < 0) {
                clamped2Pi += PI_TIMES_2;
            }

            // The LUT contains values for 0 - PiOver2; every other value must be obtained by
            // vertical or horizontal mirroring
            flipVertical = clamped2Pi >= PI;
            // obtain (angle % PI) from (angle % 2PI) - much faster than doing another modulo
            var clampedPi = clamped2Pi;
            while (clampedPi >= PI) {
                clampedPi -= PI;
            }
            flipHorizontal = clampedPi >= PI_OVER_2;
            // obtain (angle % PI_OVER_2) from (angle % PI) - much faster than doing another modulo
            var clampedPiOver2 = clampedPi;
            if (clampedPiOver2 >= PI_OVER_2) {
                clampedPiOver2 -= PI_OVER_2;
            }
            return clampedPiOver2;
        }

        /// <summary>
        /// Returns the cosine of x.
        /// See Sin() for more details.
        /// </summary>
        public static JFix64 Cos(JFix64 x) {
            var xl = x.SerializedValue;
            var rawAngle = xl + (xl > 0 ? -PI - PI_OVER_2 : PI_OVER_2);
            JFix64 a2 = Sin(new JFix64(rawAngle));
            return a2;
        }

        /// <summary>
        /// Returns a rough approximation of the cosine of x.
        /// See FastSin for more details.
        /// </summary>
        public static JFix64 FastCos(JFix64 x) {
            var xl = x.SerializedValue;
            var rawAngle = xl + (xl > 0 ? -PI - PI_OVER_2 : PI_OVER_2);
            return FastSin(new JFix64(rawAngle));
        }

        /// <summary>
        /// Returns the tangent of x.
        /// </summary>
        /// <remarks>
        /// This function is not well-tested. It may be wildly inaccurate.
        /// </remarks>
        public static JFix64 Tan(JFix64 x) {
            var clampedPi = x.SerializedValue % PI;
            var flip = false;
            if (clampedPi < 0) {
                clampedPi = -clampedPi;
                flip = true;
            }
            if (clampedPi > PI_OVER_2) {
                flip = !flip;
                clampedPi = PI_OVER_2 - (clampedPi - PI_OVER_2);
            }

            var clamped = new JFix64(clampedPi);

            // Find the two closest values in the LUT and perform linear interpolation
            var rawIndex = FastMul(clamped, LutInterval);
            var roundedIndex = Round(rawIndex);
            var indexError = FastSub(rawIndex, roundedIndex);

            var nearestValue = new JFix64(TanLut[(int)roundedIndex]);
            var secondNearestValue = new JFix64(TanLut[(int)roundedIndex + Sign(indexError)]);

            var delta = FastMul(indexError, FastAbs(FastSub(nearestValue, secondNearestValue))).SerializedValue;
            var interpolatedValue = nearestValue.SerializedValue + delta;
            var finalValue = flip ? -interpolatedValue : interpolatedValue;
            JFix64 a2 = new JFix64(finalValue);
            return a2;
        }

        /// <summary>
        /// Returns the arctan of of the specified number, calculated using Euler series
        /// This function has at least 7 decimals of accuracy.
        /// </summary>
        public static JFix64 Atan(JFix64 z)
        {
            if (z.RawValue == 0) return Zero;

            // Force positive values for argument
            // Atan(-z) = -Atan(z).
            var neg = z.RawValue < 0;
            if (neg)
            {
                z = -z;
            }

            JFix64 result;
            var two = (JFix64)2;
            var three = (JFix64)3;

            bool invert = z > One;
            if (invert) z = One / z;

            result = One;
            var term = One;

            var zSq = z * z;
            var zSq2 = zSq * two;
            var zSqPlusOne = zSq + One;
            var zSq12 = zSqPlusOne * two;
            var dividend = zSq2;
            var divisor = zSqPlusOne * three;

            for (var i = 2; i < 30; ++i)
            {
                term *= dividend / divisor;
                result += term;

                dividend += zSq2;
                divisor += zSq12;

                if (term.RawValue == 0) break;
            }

            result = result * z / zSqPlusOne;

            if (invert)
            {
                result = PiOver2 - result;
            }

            if (neg)
            {
                result = -result;
            }
            return result;
        }

        public static JFix64 Atan2(JFix64 y, JFix64 x) {
            var yl = y.SerializedValue;
            var xl = x.SerializedValue;
            if (xl == 0) {
                if (yl > 0) {
                    return PiOver2;
                }
                if (yl == 0) {
                    return Zero;
                }
                return -PiOver2;
            }
            JFix64 atan;
            var z = y / x;

            JFix64 sm = JFix64.EN2 * 28;
            // Deal with overflow
            if (One + sm * z * z == MaxValue) {
                return y < Zero ? -PiOver2 : PiOver2;
            }

            if (Abs(z) < One) {
                atan = z / (One + sm * z * z);
                if (xl < 0) {
                    if (yl < 0) {
                        return atan - Pi;
                    }
                    return atan + Pi;
                }
            } else {
                atan = PiOver2 - z / (z * z + sm);
                if (yl < 0) {
                    return atan - Pi;
                }
            }
            return atan;
        }

        public static JFix64 Asin(JFix64 value) {
            return FastSub(PiOver2, Acos(value));
        }

        /// <summary>
        /// Returns the arccos of of the specified number, calculated using Atan and Sqrt
        /// This function has at least 7 decimals of accuracy.
        /// </summary>
        public static JFix64 Acos(JFix64 x)
        {
            if (x < -One || x > One)
            {
                throw new ArgumentOutOfRangeException("Must between -FP.One and FP.One", "x");
            }

            if (x.RawValue == 0) return PiOver2;

            var result = Atan(Sqrt(One - x * x) / x);
            return x.RawValue < 0 ? result + Pi : result;
        }

        public static implicit operator JFix64(long value) {
            JFix64 result;
            result.SerializedValue = value * ONE;
            return result;
            //return new FP(value * ONE);
        }

        public static explicit operator long(JFix64 value) {
            return value.SerializedValue >> FRACTIONAL_PLACES;
        }

        public static implicit operator JFix64(float value) {
            JFix64 result;
            result.SerializedValue = (long)(value * ONE);
            return result;
            //return new FP((long)(value * ONE));
        }

        public static explicit operator float(JFix64 value) {
            return (float)value.SerializedValue / ONE;
        }

        public static implicit operator JFix64(double value) {
            JFix64 result;
            result.SerializedValue = (long)(value * ONE);
            return result;
            //return new FP((long)(value * ONE));
        }

        public static explicit operator double(JFix64 value) {
            return (double)value.SerializedValue / ONE;
        }

        public static explicit operator JFix64(decimal value) {
            JFix64 result;
            result.SerializedValue = (long)(value * ONE);
            return result;
            //return new FP((long)(value * ONE));
        }

        public static implicit operator JFix64(int value) {
            JFix64 result;
            result.SerializedValue = value * ONE;
            return result;
            //return new FP(value * ONE);
        }

        public static explicit operator decimal(JFix64 value) {
            return (decimal)value.SerializedValue / ONE;
        }

        public float AsFloat() {
            return (float) this;
        }

        public int AsInt() {
            return (int) this;
        }

        public long AsLong() {
            return (long)this;
        }

        public double AsDouble() {
            return (double)this;
        }

        public decimal AsDecimal() {
            return (decimal)this;
        }

        public static float ToFloat(JFix64 value) {
            return (float)value;
        }

        public static int ToInt(JFix64 value) {
            return (int)value;
        }

        public static JFix64 FromFloat(float value) {
            return (JFix64)value;
        }

        public static bool IsInfinity(JFix64 value) {
            return value == NegativeInfinity || value == PositiveInfinity;
        }

        public static bool IsNaN(JFix64 value) {
            return value == NaN;
        }

        public override bool Equals(object obj) {
            return obj is JFix64 && ((JFix64)obj).SerializedValue == SerializedValue;
        }

        public override int GetHashCode() {
            return SerializedValue.GetHashCode();
        }

        public bool Equals(JFix64 other) {
            return SerializedValue == other.SerializedValue;
        }

        public int CompareTo(JFix64 other) {
            return SerializedValue.CompareTo(other.SerializedValue);
        }

        public override string ToString() {
            return ((float)this).ToString();
        }

        public string ToString(IFormatProvider provider) {
            return ((float)this).ToString(provider);
        }
        public string ToString(string format) {
            return ((float)this).ToString(format);
        }

        public static JFix64 FromRaw(long rawValue) {
            return new JFix64(rawValue);
        }

        internal static void GenerateAcosLut() {
            using (var writer = new StreamWriter("JFix64AcosLut.cs")) {
                writer.Write(
@"namespace Jitter.LinearMath {
    partial struct JFix64 {
        public static readonly long[] AcosLut = new[] {");
                int lineCounter = 0;
                for (int i = 0; i < LUT_SIZE; ++i) {
                    var angle = i / ((float)(LUT_SIZE - 1));
                    if (lineCounter++ % 8 == 0) {
                        writer.WriteLine();
                        writer.Write("            ");
                    }
                    var acos = Math.Acos(angle);
                    var rawValue = ((JFix64)acos).SerializedValue;
                    writer.Write(string.Format("0x{0:X}L, ", rawValue));
                }
                writer.Write(
@"
        };
    }
}");
            }
        }

        internal static void GenerateSinLut() {
            using (var writer = new StreamWriter("JFix64SinLut.cs")) {
                writer.Write(
@"namespace Jitter.LinearMath {
    partial struct JFix64 {
        public static readonly long[] SinLut = new[] {");
                int lineCounter = 0;
                for (int i = 0; i < LUT_SIZE; ++i) {
                    var angle = i * Math.PI * 0.5 / (LUT_SIZE - 1);
                    if (lineCounter++ % 8 == 0) {
                        writer.WriteLine();
                        writer.Write("            ");
                    }
                    var sin = Math.Sin(angle);
                    var rawValue = ((JFix64)sin).SerializedValue;
                    writer.Write(string.Format("0x{0:X}L, ", rawValue));
                }
                writer.Write(
@"
        };
    }
}");
            }
        }

        internal static void GenerateTanLut() {
            using (var writer = new StreamWriter("JFix64TanLut.cs")) {
                writer.Write(
@"namespace Jitter.LinearMath {
    partial struct JFix64 {
        public static readonly long[] TanLut = new[] {");
                int lineCounter = 0;
                for (int i = 0; i < LUT_SIZE; ++i) {
                    var angle = i * Math.PI * 0.5 / (LUT_SIZE - 1);
                    if (lineCounter++ % 8 == 0) {
                        writer.WriteLine();
                        writer.Write("            ");
                    }
                    var tan = Math.Tan(angle);
                    if (tan > (double)MaxValue || tan < 0.0) {
                        tan = (double)MaxValue;
                    }
                    var rawValue = (((decimal)tan > (decimal)MaxValue || tan < 0.0) ? MaxValue : (JFix64)tan).SerializedValue;
                    writer.Write(string.Format("0x{0:X}L, ", rawValue));
                }
                writer.Write(
@"
        };
    }
}");
            }
        }

        /// <summary>
        /// The underlying integer representation
        /// </summary>
        public long RawValue { get { return SerializedValue; } }

        /// <summary>
        /// This is the constructor from raw value; it can only be used interally.
        /// </summary>
        /// <param name="rawValue"></param>
        JFix64(long rawValue) {
            SerializedValue = rawValue;
        }

        public JFix64(int value) {
            SerializedValue = value * ONE;
        }
    }
}
