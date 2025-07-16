#region Using

using System;
using System.Collections;
using System.Runtime.CompilerServices;
using System.Text;

#endregion

namespace Collision2D;

#region Random

public static class Random
{
    private static System.Random random = new System.Random();
    private static readonly float twoPi = 2f * MathF.PI;

    #region Seed

    public static void SetSeed(int seed)
    {
        random = new System.Random(seed);
    }
    /// <summary>
    /// randomize de seed of the random, allow to have diffenrent random number at each launch of the game
    /// </summary>
    public static void SetRandomSeed()
    {
        SetSeed(Environment.TickCount);
    }

    #endregion

    #region Random Value and vector

    /// <returns> A random integer between a and b, [|a, b|]</returns>
    public static int Rand(int a, int b) => random.Next(a, b + 1);
    /// <returns> A random float between 0 and 1, [0, 1]</returns>
    public static float Rand() => (float)random.Next(int.MaxValue) / (int.MaxValue - 1);
    /// <returns> A random float between a and b, [a, b]</returns>
    public static float Rand(float a, float b) => Rand() * MathF.Abs(b - a) + a;
    /// <returns> A random int between a and b, [|a, b|[</returns>
    public static int RandExclude(int a, int b) => random.Next(a, b);
    /// <returns> A random double between a and b, [a, b[</returns>
    public static float RandExclude(float a, float b) => (float)random.NextDouble() * (MathF.Abs(b - a)) + a;
    public static float RandExclude() => (float)random.NextDouble();
    public static Vector2 Vector2()
    {
        float angle = RandExclude(0f, twoPi);
        return new Vector2(MathF.Cos(angle), MathF.Sin(angle));
    }
    /// <returns> A random Vector2 with de magnitude in param</returns>
    public static Vector2 Vector2(float magnitude)
    {
        float angle = RandExclude(0f, twoPi);
        return new Vector2(magnitude * MathF.Cos(angle), magnitude * MathF.Sin(angle));
    }
    /// <returns> A random Vector2 with a randoml magnitude</returns>
    public static Vector2 Vector2(float minMagnitude, float maxMagnitude)
    {
        float angle = RandExclude(0f, twoPi);
        float magnitude = Rand(minMagnitude, maxMagnitude);
        return new Vector2(magnitude * MathF.Cos(angle), magnitude * MathF.Sin(angle));
    }

    public static Vector2 PointInCircle(Circle circle) => PointInCircle(circle.center, circle.radius);
    public static Vector2 PointInCircle(Vector2 center, float radius)
    {
        float x, y;
        while(true)
        {
            x = Rand() * 2f * radius - radius;
            y = Rand() * 2f * radius - radius;
            if(x * x + (y * y) <= radius * radius)
                return new Vector2(center.x + x, center.y + y);
        }
    }
    public static Vector2 PointInRectangle(Hitbox rec) => PointInRectangle(rec.center, rec.size);
    public static Vector2 PointInRectangle(Vector2 center, Vector2 size)
    {
        return new Vector2(center.x + (Rand() - 0.5f) * size.x, center.y + (Rand() - 0.5f) * size.y);
    }

    public static Vector2 PointInCapsule(Capsule capsule)
    {
        float areaCircles = capsule.circle1.radius * capsule.circle1.radius * MathF.PI;
        float areaRec = capsule.hitbox.size.y * capsule.hitbox.size.x;
        if(Rand(0f, areaCircles + areaRec) <= areaRec)
        {
            return PointInRectangle(capsule.hitbox.center, capsule.hitbox.size);
        }
        else
        {
            Vector2 v = capsule.circle1.center - capsule.hitbox.center;
            Vector2 rand = PointInCircle(Collision2D.Vector2.zero, capsule.circle1.radius);
            if(v.Dot(rand) >= 0f)
            {
                return capsule.circle1.center + rand;
            }
            return capsule.circle2.center + rand;
        }
    }

    #endregion
}

#endregion

#region ICloneable<T>

public interface ICloneable<T>
{
    public T Clone();
}

#endregion

#region Useful

public static class Useful
{
    #region Vector and Maths

    public static bool Approximately(float a, float b)
    {
        return MathF.Abs(b - a) < MathF.Max(1E-06f * MathF.Max(MathF.Abs(a), MathF.Abs(b)), float.Epsilon * 8f);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool IsOdd(this int number) => (number & 1) != 0;
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool IsEven(this int number) => (number & 1) == 0;

    //Vector2
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static float SqrDistance(in this Vector2 v, Vector2 a) => (a.x - v.x) * (a.x - v.x) + (a.y - v.y) * (a.y - v.y);
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static float Distance(in this Vector2 v, Vector2 a) => MathF.Sqrt(v.SqrDistance(a));
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool IsCollinear(this Vector2 a, Vector2 v) => MathF.Abs((v.x / a.x) - (v.y / a.y)) < 1e-3f;
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static float Dot(in this Vector2 v1, Vector2 v) => v1.x * v.x + v1.y * v.y;
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector2 Vector2FromAngle(float angle) => new Vector2(MathF.Cos(angle), MathF.Sin(angle));
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector2 Vector2FromAngle(float angle, float length) => new Vector2(length * MathF.Cos(angle), length * MathF.Sin(angle));


    /// <returns>the orthogonal normalised vector of v</returns>
    public static Vector2 NormalVector(in this Vector2 v)
    {
        if (!Approximately(v.x, 0f))
        {
            float y = MathF.Sqrt(1f / (((v.y * v.y) / (v.x * v.x)) + 1f));
            return new Vector2(-v.y * y / v.x, y);
        }
        else if (!Approximately(v.y, 0f))
        {
            float x = MathF.Sqrt(1f / (1f + (v.x * v.x) / (v.y * v.y)));
            return new Vector2(x, -v.x * x / v.y);
        }
        else
        {
            return v;
        }
    }

    /// <summary>
    /// 
    /// </summary>
    /// <param name="a">le point de début du vecteur</param>
    /// <param name="b">le point de fin du vecteur</param>
    /// <returns>l'angle en rad entre 0 et 2pi entre le vecteur (1,0) et (b-a) </returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static float AngleHori(this Vector2 a, Vector2 b) => MathF.Atan2(a.y - b.y, a.x - b.x) + MathF.PI;
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static float Angle(this in Vector2 a, in Vector2 b) => ClampModulo(-MathF.PI, MathF.PI, AngleHori(Vector2.zero, a) + AngleHori(Vector2.zero, b));
    /// <summary>
    /// Renvoie l'angle minimal entre le segment [ca] et [cb]
    /// </summary>
    public static float Angle(Vector2 c, Vector2 a, Vector2 b)
    {
        float ang1 = AngleHori(c, a);
        float ang2 = AngleHori(c, b);
        float diff = MathF.Abs(ang1 - ang2);
        return MathF.Min(diff, 2f * MathF.PI - diff);
    }

    public static float ClampModulo(float start, float end, float value)
    {
        if (end < start)
            return ClampModulo(end, start, value);
        if (end - start < float.Epsilon)
            return start;

        if (value < end && value >= start)
            return value;
        else
        {
            float modulo = end - start;
            float result = ((value - start) % modulo) + start;
            if (result >= end)
                return result - modulo;
            if (result < start)
                return result + modulo;
            return result;
        }
    }

    #endregion

    #region Collection

    public static T[] Merge<T>(this T[] arr, T[] other) where T : ICloneable<T>
    {
        T[] res = new T[arr.Length + other.Length];
        for (int i = 0; i < arr.Length; i++)
        {
            res[i] = arr[i].Clone();
        }
        for (int i = arr.Length; i < res.Length; i++)
        {
            res[i] = other[i - arr.Length].Clone();
        }
        return res;
    }

    public static List<T> Distinct<T>(this List<T> lst)
    {
        List<T> result = new List<T>(lst.Count);
        foreach (T item in lst)
        {
            if(!result.Contains(item))
                result.Add(item);
        }
        return result;
    }

    public static List<T> Clone<T>(this List<T> lst) where T : ICloneable<T>
    {
        List<T> result = new List<T>(lst.Count);
        foreach (T item in lst)
        {
            result.Add(item.Clone());
        }
        return result;
    }

    #endregion
}

#endregion