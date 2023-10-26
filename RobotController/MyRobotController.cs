using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using System.Security.Cryptography.X509Certificates;
using System.Text;

namespace RobotController
{
    public struct MyQuat
    {
        public float x;
        public float y;
        public float z;
        public float w;


        public MyQuat(float _x, float _y, float _z, float _w)
        {
            x = _x;
            y = _y;
            z = _z;
            w = _w;
        }

        public static MyQuat operator +(MyQuat a, MyQuat b)
        {
            return new MyQuat(a.x + b.x, a.y + b.y, a.z * b.z, a.w + b.w);
        }
        public static MyQuat operator *(MyQuat a, MyQuat b)
        {
            MyQuat r = new MyQuat();
            r.w = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z;
            r.x = a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y;
            r.y = a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x;
            r.z = a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w;

            return r;
        }
        public static MyQuat operator *(MyQuat a, float b)
        {
            MyQuat r = new MyQuat();
            r.w = a.w * b;
            r.x = a.x * b;
            r.y = a.y * b;
            r.z = a.z * b;


            return r;
        }
        public static MyQuat operator *(float b, MyQuat a)
        {
            MyQuat r = new MyQuat();
            r.w = a.w * b;
            r.x = a.x * b;
            r.y = a.y * b;
            r.z = a.z * b;

            return r;
        }
        public MyQuat Conjugate()
        {
            x = -x;
            y = -y;
            z = -z;
            return this;
        }

        public static MyQuat Cross(MyQuat a, MyQuat b)
        {
            return a * b.Conjugate();
        }
        public float Magnitude()
        {
            return (float)Math.Sqrt((x * x) + (y * y) + (z * z) + (w * w));
        }
        public MyQuat Inverse()
        {
            float num = 1f / ((float)Math.Pow(x, 2.0) + (float)Math.Pow(y, 2.0) + (float)Math.Pow(z, 2.0) + (float)Math.Pow(w, 2.0));
            return Conjugate() * num;
        }
        public static MyQuat VectoQuad(MyVec v)
        {
            return new MyQuat(v.x, v.y, v.z, 1);
        }

        public static MyQuat VectoQuad(MyVec v, float w)
        {
            return new MyQuat(v.x, v.y, v.z, w);
        }
        public static MyQuat Normalize(MyQuat v)
        {
            float n = v.Magnitude();
            return new MyQuat(v.x / n, v.y / n, v.z / n, v.w / n);
        }
        public MyQuat Rotate(MyVec v, float angle)
        {
            MyQuat axis = VectoQuad(v) * (float)Math.Sin(angle / 2);
            axis.w = (float)Math.Cos(angle / 2);
            return Normalize(this * axis);
        }
    }

    public struct MyVec
    {
        public float x;
        public float y;
        public float z;
        public MyVec(float _x, float _y, float _z)
        {
            x = _x;
            y = _y;
            z = _z;
        }
        public static MyVec operator +(MyVec a, MyVec b)
        {
            return new MyVec(a.x + b.x, a.y + b.y, a.z + b.z);
        }
        public static MyVec operator -(MyVec a, MyVec b)
        {
            return new MyVec(a.x - b.x, a.y - b.y, a.z - b.z);
        }
        public static MyVec operator *(MyVec a, float b)
        {
            return new MyVec(a.x + b, a.y + b, a.z + b);
        }
        public static MyVec operator *(MyVec a, MyVec b)
        {
            return new MyVec(a.x * b.x, a.y * b.y, a.z * b.y);
        }
        public static float Dot(MyVec a, MyVec b)
        {
            return a.x * b.x + a.y * b.y + a.z * b.z;
        }

    }
    public class MyRobotController
    {
        float startTimeAnimation1 = 0;
        float startTimeAnimation2 = 0;
        bool canPlayExercice2 = true;
        bool canPlayExercice3 = true;

        private static MyQuat twist;
        private static MyQuat swing;

        #region public methods

        public string Hi()
        {

            string s = "Robot Controller Pablo y Dani";
            return s;

        }


        //EX1: this function will place the robot in the initial position

        public void PutRobotStraight(out MyQuat rot0, out MyQuat rot1, out MyQuat rot2, out MyQuat rot3)
        {

            canPlayExercice2 = true;
            canPlayExercice3 = true;

            rot0 = NullQ;
            rot0 = Rotate(rot0, new MyVec(0, 1, 0), 74);
            rot1 = Rotate(rot0, new MyVec(1, 0, 0), -7);
            rot2 = Rotate(rot1, new MyVec(1, 0, 0), 76);
            rot3 = Rotate(rot2, new MyVec(1, 0, 0), 32);
        }

        //EX2: this function will interpolate the rotations necessary to move the arm of the robot until its end effector collides with the target (called Stud_target)
        //it will return true until it has reached its destination. The main project is set up in such a way that when the function returns false, the object will be droped and fall following gravity.

        public bool PickStudAnim(out MyQuat rot0, out MyQuat rot1, out MyQuat rot2, out MyQuat rot3)
        {

            if (!canPlayExercice2)
            {
                rot0 = NullQ;
                rot1 = NullQ;
                rot2 = NullQ;
                rot3 = NullQ;
                return false;
            }

            float animationDuration = 1000;
            if (startTimeAnimation1 <= 0)
            {
                startTimeAnimation1 = TimeSinceMidnight;
            }


            float lerpValue = (TimeSinceMidnight - startTimeAnimation1) / animationDuration;

            if (lerpValue < 1)
            {
                //todo: add your code here
                rot0 = NullQ;
                rot0 = Rotate(rot0, new MyVec(0f, 1f, 0f), (41 - 74) * lerpValue + 74);
                rot1 = Rotate(rot0, new MyVec(1, 0, 0), (6 + 7) * lerpValue - 7);
                rot2 = Rotate(rot1, new MyVec(1, 0, 0), (65 - 76) * lerpValue + 76);
                rot3 = Rotate(rot2, new MyVec(1, 0, 0), (30 - 32) * lerpValue + 32);

                return true;
            }

            startTimeAnimation1 = 0;
            canPlayExercice2 = false;
            canPlayExercice3 = true;
            rot0 = NullQ;
            rot1 = NullQ;
            rot2 = NullQ;
            rot3 = NullQ;

            return false;
        }


        //EX3: this function will calculate the rotations necessary to move the arm of the robot until its end effector collides with the target (called Stud_target)
        //it will return true until it has reached its destination. The main project is set up in such a way that when the function returns false, the object will be droped and fall following gravity.
        //the only difference wtih exercise 2 is that rot3 has a swing and a twist, where the swing will apply to joint3 and the twist to joint4

        public bool PickStudAnimVertical(out MyQuat rot0, out MyQuat rot1, out MyQuat rot2, out MyQuat rot3)
        {
            bool myCondition = false;
            //todo: add a check for your condition



            while (myCondition)
            {
                //todo: add your code here


            }

            //todo: remove this once your code works.
            rot0 = NullQ;
            rot1 = NullQ;
            rot2 = NullQ;
            rot3 = NullQ;

            return false;
        }


        public static MyQuat GetSwing(MyQuat rot3)
        {
            return MyQuat.Normalize(twist.Inverse() * rot3);
        }
        public static MyQuat GetTwist(MyQuat rot3)
        {
            return MyQuat.Normalize(swing.Inverse() * rot3);

        }
        #endregion

        #region private and internal methods

        internal float TimeSinceMidnight { get { return (DateTime.Now.Hour * 3600000) + (DateTime.Now.Minute * 60000) + (DateTime.Now.Second * 1000) + DateTime.Now.Millisecond; } }

        private static MyVec Up
        {
            get
            {
                return new MyVec(0, 1, 0);
            }
        }
        private static MyVec Rigth
        {
            get
            {
                return new MyVec(1, 0, 0);
            }
        }
        private static MyVec Forward
        {
            get
            {
                return new MyVec(0, 0, 1);
            }
        }


        private static MyQuat NullQ
        {
            get
            {
                MyQuat a;
                a.w = 1;
                a.x = 0;
                a.y = 0;
                a.z = 0;
                return a;

            }
        }

        internal MyQuat Multiply(MyQuat q1, MyQuat q2)
        {
            return q1 * q2;
        }

        internal MyQuat Rotate(MyQuat currentRotation, MyVec axis, float angle)
        {
            return currentRotation.Rotate(axis, angle * (float)Math.PI / 180);
        }

        //todo: add here all the functions needed

        #endregion


    }
}
