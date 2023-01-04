using System;
using System.Collections.Generic;
using System.Text;
using Jitter.LinearMath;

namespace Jitter.Dynamics
{

    // TODO: Check values, Documenation
    // Maybe some default materials, aka Material.Soft?
    public class Material
    {

        internal JFix64 kineticFriction = (3 * JFix64.EN1);
        internal JFix64 staticFriction = (6 * JFix64.EN1);
        internal JFix64 restitution = JFix64.Zero;

        public Material() { }

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

        public JFix64 KineticFriction
        {
            get { return kineticFriction; }
            set { kineticFriction = value; }
        }

    }
}
