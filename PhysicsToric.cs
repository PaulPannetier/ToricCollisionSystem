

namespace Collision2D
{
    #region Struct

    public struct ToricRaycastHit2D : ICloneable<ToricRaycastHit2D>
    {
        //
        // Summary:
        //     The world space position where the physics query shape intersected with the detected Collider2D surface.
        public Vector2 point;
        //
        // Summary:
        //     The world space centroid (center) of the physics query shape when it intersects.
        public Vector2 centroid;
        //
        // Summary:
        //     The Collider2D detected by the physics query.
        public Collider2D? collider;
        //
        // Summary:
        //     The surface normal of the detected Collider2D.
        public Vector2 normal;
        //
        // Summary:
        //     The distance the physics query traversed before it detected a Collider2D.
        public float distance;

        public ToricRaycastHit2D(Vector2 point, Vector2 centroid, Collider2D? collider, Vector2 normal, float distance)
        {
            this.point = point;
            this.centroid = centroid;
            this.collider = collider;
            this.normal = normal;
            this.distance = distance;
        }

        public ToricRaycastHit2D Clone()
        {
            return new ToricRaycastHit2D(point, centroid, collider, normal, distance);
        }

        public static implicit operator bool(ToricRaycastHit2D hit)
        {
            return hit.collider != null;
        }
    }

    #endregion

    public static class PhysicsToric
    {
        #region Camera and General Things

        internal static Hitbox mapHitbox = new Hitbox(Vector2.zero, Vector2.zero);
        internal static Line2D[] mapSides = new Line2D[4];
        internal static Vector2[] mapOffset = new Vector2[4];

        internal static HashSet<Collider2D> colliders = new HashSet<Collider2D>();

        //Call this method to change the size of the toric map
        public static void OnMapChange(Vector2 size)
        {
            Vector2 halfSize = size * 0.5f;
            mapHitbox = new Hitbox(Vector2.zero, size);
            mapSides[0] = new Line2D(new Vector2(-halfSize.x, halfSize.y), halfSize);
            mapSides[1] = new Line2D(-halfSize, new Vector2(halfSize.x, -halfSize.y));
            mapSides[2] = new Line2D(new Vector2(-halfSize.x, halfSize.y), -halfSize);
            mapSides[3] = new Line2D(halfSize, new Vector2(halfSize.x, -halfSize.y));
            mapOffset[0] = new Vector2(0f, size.y);
            mapOffset[1] = new Vector2(0f, -size.y);
            mapOffset[2] = new Vector2(-size.x, 0f);
            mapOffset[3] = new Vector2(size.x, 0f);
        }

        public static bool IsPointInsideBound(Vector2 point) => -mapHitbox.size.x * 0.5f <= point.x && mapHitbox.size.x * 0.5f >= point.x && -mapHitbox.size.y * 0.5f <= point.y && mapHitbox.size.y * 0.5f >= point.y;

        /// <param name="point"></param>
        /// <returns>Le point visible au coor donn�e en param dans l'espace torique</returns>
        public static Vector2 GetPointInsideBounds(Vector2 point)
        {
            return new Vector2(Useful.ClampModulo(-mapHitbox.size.x * 0.5f, mapHitbox.size.x * 0.5f, point.x),
                Useful.ClampModulo(-mapHitbox.size.y * 0.5f, mapHitbox.size.y * 0.5f, point.y));
        }

        public static Vector2 GetComplementaryPoint(Vector2 point)
        {
            Vector2 step = new Vector2(point.x - mapHitbox.center.x > 0f ? 0.001f : -0.001f, point.y - mapHitbox.center.y > 0f ? 0.001f : -0.001f);
            while (mapHitbox.Contains(point))
            {
                point += step;
            }
            point += step;
            return GetPointInsideBounds(point);
        }

        public static float Distance(Vector2 p1, Vector2 p2)
        {
            p1 = GetPointInsideBounds(p1);
            p2 = GetPointInsideBounds(p2);

            float toricX = p1.x + MathF.Sign(p2.x - p1.x) * mapHitbox.size.x;
            float x = MathF.Abs(p1.x - p2.x) < MathF.Abs(toricX - p2.x) ? p1.x : toricX;
            float toricY = p1.y + MathF.Sign(p2.y - p1.y) * mapHitbox.size.y;
            float y = MathF.Abs(p1.y - p2.y) < MathF.Abs(toricY - p2.y) ? p1.y : toricY;

            return p2.Distance(new Vector2(x, y));
        }

        public static Vector2 Direction(Vector2 from, Vector2 to)
        {
            Vector2[] possibleA = new Vector2[5]
            {
                from,
                new Vector2(from.x + mapHitbox.size.x , from.y),
                new Vector2(from.x - mapHitbox.size.x, from.y),
                new Vector2(from.x, from.y + mapHitbox.size.y),
                new Vector2(from.x, from.y - mapHitbox.size.y)
            };

            Vector2[] possibleB = new Vector2[5]
            {
                to,
                new Vector2(to.x + mapHitbox.size.x , to.y),
                new Vector2(to.x - mapHitbox.size.x, to.y),
                new Vector2(to.x, to.y + mapHitbox.size.y),
                new Vector2(to.x, to.y - mapHitbox.size.y)
            };

            Vector2 aKeep = new Vector2(), bKeep = new Vector2();
            float minSqrMag = float.MaxValue;
            float sqrMag;
            for (int i = 0; i < 5; i++)
            {
                for (int j = 0; j < 5; j++)
                {
                    sqrMag = possibleA[i].SqrDistance(possibleB[j]);
                    if (sqrMag < minSqrMag)
                    {
                        minSqrMag = sqrMag;
                        aKeep = possibleA[i];
                        bKeep = possibleB[j];
                    }
                }
            }

            if (minSqrMag <= 1e-5f)
            {
                return Vector2.zero;
            }

            return (bKeep - aKeep) * (1f / MathF.Sqrt(minSqrMag));
        }

        public static Tuple<Vector2, float> DirectionAndDistance(Vector2 from, Vector2 to)
        {
            Vector2[] possibleA = new Vector2[5]
            {
                from,
                new Vector2(from.x + mapHitbox.size.x , from.y),
                new Vector2(from.x - mapHitbox.size.x, from.y),
                new Vector2(from.x, from.y + mapHitbox.size.y),
                new Vector2(from.x, from.y - mapHitbox.size.y)
            };

            Vector2[] possibleB = new Vector2[5]
            {
                to,
                new Vector2(to.x + mapHitbox.size.x , to.y),
                new Vector2(to.x - mapHitbox.size.x, to.y),
                new Vector2(to.x, to.y + mapHitbox.size.y),
                new Vector2(to.x, to.y - mapHitbox.size.y)
            };

            Vector2 aKeep = new Vector2(), bKeep = new Vector2();
            float minSqrMag = float.MaxValue;
            float sqrMag;
            for (byte i = 0; i < 5; i++)
            {
                for (byte j = 0; j < 5; j++)
                {
                    sqrMag = possibleA[i].SqrDistance(possibleB[j]);
                    if (sqrMag < minSqrMag)
                    {
                        minSqrMag = sqrMag;
                        aKeep = possibleA[i];
                        bKeep = possibleB[j];
                    }
                }
            }

            if (minSqrMag <= 1e-5f)
            {
                return new Tuple<Vector2, float>(Vector2.zero, 0f);
            }

            float distance = MathF.Sqrt(minSqrMag);
            return new Tuple<Vector2, float>((bKeep - aKeep) * (1f / distance), distance);
        }

        public static bool GetToricIntersection(Vector2 from, Vector2 end, out Vector2 inter)
        {
            for (byte i = 0; i < mapHitbox.vertices.Length; i++)
            {
                if (Collider2D.CollideLines(from, end, mapHitbox.vertices[i], mapHitbox.vertices[(i + 1) % mapHitbox.vertices.Length], out inter))
                {
                    return true;
                }
            }
            inter = Vector2.zero;
            return false;
        }

        #endregion

        #region Overlap

        public static Collider2D? OverlapPoint(Vector2 point)
        {
            point = GetPointInsideBounds(point);
            foreach (Collider2D col in colliders)
            {
                if (col.Contains(point))
                {
                    return col;
                }
            }
            return default(Collider2D);
        }

        public static Collider2D[] OverlapPointAll(Vector2 point)
        {
            point = GetPointInsideBounds(point);
            List<Collider2D> res = new List<Collider2D>(colliders.Count);
            foreach (Collider2D col in colliders)
            {
                if (col.Contains(point))
                {
                    res.Add(col);
                }
            }
            return res.ToArray();
        }

        public static Collider2D? OverlapCircle(Circle circle) => OverlapCircle(circle.center, circle.radius);

        public static Collider2D? OverlapCircle(Vector2 center, float radius)
        {
            Circle circle = new Circle(GetPointInsideBounds(center), radius);
            bool[] collideWithCamSide = new bool[4];
            for (int i = 0; i < 4; i++)
            {
                if (Collider2D.CollideCircleLine(circle, mapSides[i]))
                {
                    collideWithCamSide[i] = true;
                }
            }

            foreach (Collider2D col in colliders)
            {
                if (Collider2D.Collide(col, circle))
                    return col;

                for (int i = 0; i < 4; i++)
                {
                    if (collideWithCamSide[i])
                    {
                        circle.MoveAt(circle.center - mapOffset[i]);
                        if (Collider2D.Collide(col, circle))
                            return col;
                        circle.MoveAt(circle.center + mapOffset[i]);
                    }
                }
            }
            return null;
        }

        public static Collider2D[] OverlapCircleAll(Circle circle) => OverlapCircleAll(circle.center, circle.radius);

        public static Collider2D[] OverlapCircleAll(Vector2 center, float radius)
        {
            Circle circle = new Circle(GetPointInsideBounds(center), radius);

            bool[] collideWithCamSide = new bool[4];
            for (int i = 0; i < 4; i++)
            {
                if (Collider2D.CollideCircleLine(circle, mapSides[i]))
                {
                    collideWithCamSide[i] = true;
                }
            }

            List<Collider2D> resToAdd = new List<Collider2D>(colliders.Count);
            foreach (Collider2D col in colliders)
            {
                if (Collider2D.Collide(col, circle))
                {
                    resToAdd.Add(col);
                }

                for (int i = 0; i < 4; i++)
                {
                    if (collideWithCamSide[i])
                    {
                        circle.MoveAt(circle.center - mapOffset[i]);
                        if (Collider2D.Collide(col, circle))
                        {
                            resToAdd.Add(col);
                        }
                        circle.MoveAt(circle.center + mapOffset[i]);
                    }
                }
            }
            return resToAdd.ToArray();
        }

        public static Collider2D? OverlapBox(Hitbox hitbox) => OverlapBox(hitbox.center, hitbox.size, hitbox.AngleHori());

        public static Collider2D? OverlapBox(Vector2 point, Vector2 size, float angle)
        {
            Hitbox hitbox = new Hitbox(GetPointInsideBounds(point), size);
            hitbox.Rotate(angle);

            bool[] collideWithCamSide = new bool[4];
            for (int i = 0; i < 4; i++)
            {
                if (Collider2D.CollideHitboxLine(hitbox, mapSides[i]))
                {
                    collideWithCamSide[i] = true;
                }
            }

            foreach (Collider2D col in colliders)
            {
                if (Collider2D.Collide(col, hitbox))
                    return col;

                for (int i = 0; i < 4; i++)
                {
                    if (collideWithCamSide[i])
                    {
                        hitbox.MoveAt(hitbox.center - mapOffset[i]);
                        if (Collider2D.Collide(col, hitbox))
                            return col;

                        hitbox.MoveAt(hitbox.center + mapOffset[i]);
                    }
                }
            }

            return null;
        }

        public static Collider2D[] OverlapBoxAll(Hitbox hitbox) => OverlapBoxAll(hitbox.center, hitbox.size, hitbox.AngleHori());

        /// <summary>
        /// 
        /// </summary>
        /// <param name="point"></param>
        /// <param name="size"></param>
        /// <param name="angle">The rotation of the hitbox in radian</param>
        /// <param name="layerMask"></param>
        /// <returns></returns>
        public static Collider2D[] OverlapBoxAll(Vector2 point, Vector2 size, float angle)
        {
            Hitbox hitbox = new Hitbox(GetPointInsideBounds(point), size);
            hitbox.Rotate(angle);

            bool[] collideWithCamSide = new bool[4];
            for (int i = 0; i < 4; i++)
            {
                if (Collider2D.CollideHitboxLine(hitbox, mapSides[i]))
                {
                    collideWithCamSide[i] = true;
                }
            }

            List<Collider2D> res = new List<Collider2D>();
            foreach (Collider2D col in colliders)
            {
                if (Collider2D.Collide(col, hitbox))
                {
                    res.Add(col);
                }

                for (int i = 0; i < 4; i++)
                {
                    if (collideWithCamSide[i])
                    {
                        hitbox.MoveAt(hitbox.center - mapOffset[i]);
                        if (Collider2D.Collide(col, hitbox))
                        {
                            res.Add(col);
                        }
                        hitbox.MoveAt(hitbox.center - mapOffset[i]);
                    }
                }
            }
            return res.ToArray();
        }

        /// <summary>
        /// 
        /// </summary>
        /// <param name="center"></param>
        /// <param name="size"></param>
        /// <param name="angle">The rotation of the capsule in radian</param>
        /// <param name="layerMask"></param>
        /// <returns></returns>
        public static Collider2D? OverlapCapsule(Vector2 center, Vector2 size, float angle)
        {
            Capsule c = new Capsule(center, size);
            if (MathF.Abs(angle) > 1e-5f)
                c.Rotate(angle);
            return OverlapCapsule(c);
        }

        public static Collider2D? OverlapCapsule(Capsule capsule)
        {
            Collider2D? res = PhysicsToric.OverlapBox(capsule.hitbox);
            if (res != null)
                return res;
            res = PhysicsToric.OverlapCircle(capsule.circle1);
            if (res != null)
                return res;
            return PhysicsToric.OverlapCircle(capsule.circle2);
        }

        public static Collider2D[] OverlapCapsuleAll(Vector2 center, Vector2 size, float angle)
        {
            Capsule c = new Capsule(center, size);
            if (MathF.Abs(angle) > 1e-5f)
                c.Rotate(angle);
            return OverlapCapsuleAll(c);
        }

        public static Collider2D[] OverlapCapsuleAll(Capsule capsule)
        {
            Collider2D[] res = OverlapBoxAll(capsule.hitbox);
            res = res.Merge(OverlapCircleAll(capsule.circle1));
            return res.Merge(OverlapCircleAll(capsule.circle2)).Distinct().ToArray();
        }

        #endregion

        #region Cast

        #region Raycast

        public static Vector2[] RaycastToricIntersections(Vector2 from, Vector2 dir, float length)
        {
            Vector2 end = from + (dir * length);
            List<Vector2> inters = new List<Vector2>(3);
            while (GetToricIntersection(from, end, out Vector2 inter) && length > 1e-3f)
            {
                inters.Add(inter);
                length -= from.Distance(inter);
                from = GetComplementaryPoint(inter);
                end = from + (dir * length);
            }

            return inters.ToArray();
        }

        public static ToricRaycastHit2D Raycast(Vector2 from, Vector2 direction, float distance)
        {
            List<Vector2> _ = new List<Vector2>();
            ToricRaycastHit2D raycast = RaycastRecur(GetPointInsideBounds(from), direction.normalized, distance, 0f, ref _);
            return raycast;
        }

        public static ToricRaycastHit2D Raycast(Vector2 from, Vector2 direction, float distance, out Vector2[] toricHitboxIntersectionsPoints)
        {
            List<Vector2> points = new List<Vector2>();
            ToricRaycastHit2D raycast = RaycastRecur(GetPointInsideBounds(from), direction.normalized, distance, 0f, ref points);
            toricHitboxIntersectionsPoints = points.ToArray();
            return raycast;
        }

        private static ToricRaycastHit2D Physics2DRaycast(Vector2 from, Vector2 end)
        {
            float minSqrDist = float.MaxValue;
            Vector2 minCp = Vector2.zero;
            Vector2 minN = Vector2.zero;
            Collider2D? res = null;

            Ray2D ray = new Ray2D(from, end);
            foreach (Collider2D col in colliders)
            {
                if (Collider2D.CollideRay(col, ray, out Vector2 cp, out Vector2 n))
                {
                    float sqrDist = from.SqrDistance(cp);
                    if (sqrDist < minSqrDist)
                    {
                        minSqrDist = sqrDist;
                        minCp = cp;
                        res = col;
                        minN = n;
                    }
                }
            }

            return new ToricRaycastHit2D(minCp, minCp, res, minN, MathF.Sqrt(minSqrDist));
        }

        private static List<ToricRaycastHit2D> Physics2DRaycastAll(Vector2 from, Vector2 end)
        {
            List<ToricRaycastHit2D> res = new List<ToricRaycastHit2D>();
            Ray2D ray = new Ray2D(from, end);
            foreach (Collider2D col in colliders)
            {
                if (Collider2D.CollideRay(col, ray, out Vector2 cp, out Vector2 n))
                {
                    res.Add(new ToricRaycastHit2D(cp, cp, col, n, from.Distance(cp)));
                }
            }
            return res;
        }

        private static ToricRaycastHit2D RaycastRecur(Vector2 from, Vector2 direction, float distance, float reachDistance, ref List<Vector2> points)
        {
            Vector2 end = from + direction * distance;

            if (GetToricIntersection(from, end, out Vector2 inter))
            {
                float currentDistance = from.Distance(inter);
                ToricRaycastHit2D res = Physics2DRaycast(from, inter);

                if (res.collider == null)
                {
                    reachDistance += currentDistance;
                    points.Add(inter);
                    inter = GetComplementaryPoint(inter);
                    return RaycastRecur(inter, direction, distance - currentDistance, reachDistance, ref points);
                }
                else
                {
                    res.distance += reachDistance;
                    return res;
                }
            }
            else
            {
                //ez case
                ToricRaycastHit2D res = Physics2DRaycast(from, end);
                res.distance += reachDistance;
                return res;
            }
        }

        public static ToricRaycastHit2D[] RaycastAll(Vector2 from, Vector2 dir, float distance)
        {
            return RaycastAllRecur(GetPointInsideBounds(from), dir.normalized, distance, 0f);
        }

        public static ToricRaycastHit2D[] RaycastAll(Vector2 from, Vector2 dir, float distance, out Vector2[][] toricHitboxIntersectionsPoints)
        {
            from = GetPointInsideBounds(from);
            List<List<Vector2>> interPoints = new List<List<Vector2>>();
            List<Vector2> globalInterPoints = new List<Vector2>();
            ToricRaycastHit2D[] raycasts = RaycastAllRecur(from, dir.normalized, distance, 0f, ref globalInterPoints, ref interPoints);

            toricHitboxIntersectionsPoints = new Vector2[interPoints.Count][];
            for (int i = 0; i < toricHitboxIntersectionsPoints.Length; i++)
            {
                toricHitboxIntersectionsPoints[i] = new Vector2[interPoints[i].Count];
                for (int j = 0; j < toricHitboxIntersectionsPoints[i].Length; j++)
                {
                    toricHitboxIntersectionsPoints[i][j] = interPoints[i][j];
                }
            }

            return raycasts;
        }

        private static ToricRaycastHit2D[] RaycastAllRecur(Vector2 from, Vector2 direction, float distance, float reachDistance)
        {
            Vector2 end = from + (distance * direction);
            if (GetToricIntersection(from, end, out Vector2 inter))
            {
                float currentDist = from.Distance(inter);
                List<ToricRaycastHit2D> resList = Physics2DRaycastAll(from, inter);

                ToricRaycastHit2D[] res = new ToricRaycastHit2D[resList.Count];
                for (int i = 0; i < resList.Count; i++)
                {
                    res[i] = resList[i];
                    res[i].distance += reachDistance;
                }

                reachDistance += currentDist;
                return res.Merge(RaycastAllRecur(GetComplementaryPoint(inter), direction, distance - currentDist, reachDistance));

            }
            else
            {

                List<ToricRaycastHit2D> resList = Physics2DRaycastAll(from, end);
                ToricRaycastHit2D[] res = new ToricRaycastHit2D[resList.Count];
                for (int i = 0; i < resList.Count; i++)
                {
                    res[i] = resList[i];
                    res[i].distance += reachDistance;
                }
                return res;
            }
        }

        private static ToricRaycastHit2D[] RaycastAllRecur(Vector2 from, Vector2 direction, float distance, float reachDistance, ref List<Vector2> globalInterPoints, ref List<List<Vector2>> interPoints)
        {
            Vector2 end = from + (distance * direction);
            if (GetToricIntersection(from, end, out Vector2 inter))
            {
                float currentDist = from.Distance(inter);
                List<ToricRaycastHit2D> resList = Physics2DRaycastAll(from, inter);

                ToricRaycastHit2D[] res = new ToricRaycastHit2D[resList.Count];
                for (int i = 0; i < resList.Count; i++)
                {
                    res[i] = resList[i];
                    res[i].distance += reachDistance;
                    interPoints.Add(globalInterPoints.Clone());
                }

                reachDistance += currentDist;
                globalInterPoints.Add(inter);
                return res.Merge(RaycastAllRecur(GetComplementaryPoint(inter), direction, distance - currentDist, reachDistance, ref globalInterPoints, ref interPoints));
            }
            else
            {
                List<ToricRaycastHit2D> resList = Physics2DRaycastAll(from, end);
                ToricRaycastHit2D[] res = new ToricRaycastHit2D[resList.Count];
                for (int i = 0; i < resList.Count; i++)
                {
                    res[i] = resList[i];
                    res[i].distance += reachDistance;
                    interPoints.Add(globalInterPoints.Clone());
                }

                return res;
            }
        }

        #endregion

        #region CircleCast

        #region Physics2D

        #region Dico

        private static readonly Dictionary<Type, Func<Vector2, Vector2, float, float, float, Collider2D, ToricRaycastHit2D>> circleCastFunction = new Dictionary<Type, Func<Vector2, Vector2, float, float, float, Collider2D, ToricRaycastHit2D>>()
        {
            { typeof(Circle), (Vector2 start, Vector2 dir, float radius, float distance, float bestDistanceFound, Collider2D circle) => Physics2DCircleCastCircle(start, dir, radius, distance, bestDistanceFound, (Circle)circle) },
            { typeof(Hitbox), (Vector2 start, Vector2 dir, float radius, float distance, float bestDistanceFound, Collider2D hitbox) => Physics2DCircleCastHitbox(start, dir, radius, distance, bestDistanceFound, (Hitbox)hitbox) },
            { typeof(Polygone), (Vector2 start, Vector2 dir, float radius, float distance, float bestDistanceFound, Collider2D poly) => Physics2DCircleCastPolygone(start, dir, radius, distance, bestDistanceFound, (Polygone)poly) },
            { typeof(Capsule), (Vector2 start, Vector2 dir, float radius, float distance, float bestDistanceFound, Collider2D capsule) => Physics2DCircleCastCapsule(start, dir, radius, distance, bestDistanceFound, (Capsule)capsule) }
        };

        #endregion

        #region Circle Cast Single

        private static ToricRaycastHit2D Physics2DCircleCast(Vector2 start, Vector2 dir, float radius, float distance)
        {
            ToricRaycastHit2D raycast;
            ToricRaycastHit2D res = new ToricRaycastHit2D();
            bool collide = false;
            float bestDistanceFound = -1f;

            foreach (Collider2D col in colliders)
            {
                raycast = circleCastFunction[col.GetType()](start, dir, radius, distance, bestDistanceFound, col);

                if (raycast.distance >= 0f && (!collide || raycast.distance < res.distance))
                {
                    collide = true;
                    res = raycast;
                    res.collider = col;
                    bestDistanceFound = res.distance;
                }
            }

            return res;
        }

        #region CircleCast Circle

        private static ToricRaycastHit2D Physics2DCircleCastCircle(Vector2 start, Vector2 dir, float radius, float distance, float bestDistanceFound, Circle circle)
        {
            Vector2 end = start + dir * distance;
            if ((bestDistanceFound >= 0f && start.SqrDistance(circle.center) > (bestDistanceFound + circle.radius + radius) * (bestDistanceFound + circle.radius + radius)) ||
                Line2D.Distance(start, end, circle.center) > circle.radius + radius)
            {
                return new ToricRaycastHit2D(Vector2.zero, Vector2.zero, null, Vector2.zero, -1f);
            }

            Vector2 maxCenter = StraightLine2D.OrthogonalProjection(circle.center, start, end);
            maxCenter = maxCenter.SqrDistance(start) > distance * distance ? end : maxCenter;

            void FindCenterRecur(Circle circle, Vector2 minCenter, Vector2 maxCenter, float sumRadiusSqr, ref Vector2? lastCollideCenter)
            {
                if (minCenter.SqrDistance(maxCenter) < 1e-5f || maxCenter.SqrDistance(circle.center) > sumRadiusSqr)
                    return;

                Vector2 avgCenter = (minCenter + maxCenter) * 0.5f;
                if (avgCenter.SqrDistance(circle.center) <= sumRadiusSqr)
                {
                    lastCollideCenter = avgCenter;
                    FindCenterRecur(circle, minCenter, avgCenter, sumRadiusSqr, ref lastCollideCenter);
                    return;
                }
                FindCenterRecur(circle, avgCenter, maxCenter, sumRadiusSqr, ref lastCollideCenter);
            }

            Vector2? bestCenter = null;
            FindCenterRecur(circle, start, maxCenter, (radius + circle.radius) * (radius + circle.radius), ref bestCenter);

            if (bestCenter.HasValue)
            {
                Vector2 n = (bestCenter.Value - circle.center).normalized;
                Vector2 point = bestCenter.Value - n * radius;
                return new ToricRaycastHit2D(point, bestCenter.Value, null, n, start.Distance(point));
            }

            return new ToricRaycastHit2D(Vector2.zero, Vector2.zero, null, Vector2.zero, -1f);
        }

        #endregion

        #region CircleCast Polygone

        private static ToricRaycastHit2D Physics2DCircleCastPolygone(Vector2 start, Vector2 dir, float radius, float distance, float bestDistanceFound, Polygone polygone)
        {
            float cache = polygone.center.SqrDistance(start);
            if ((bestDistanceFound >= 0f && MathF.Sqrt(cache) - polygone.inclusiveCircle.radius > bestDistanceFound) ||
                cache > (radius + distance + polygone.inclusiveCircle.radius) * (radius + distance + polygone.inclusiveCircle.radius))
            {
                return new ToricRaycastHit2D(Vector2.zero, Vector2.zero, null, Vector2.zero, -1f);
            }

            if (Collider2D.CollideCirclePolygone(new Circle(start, radius), polygone, out Vector2 point, out Vector2 _, out Vector2 n))
            {
                return new ToricRaycastHit2D(point, start, null, n, 0f);
            }

            List<Line2D> sides = new List<Line2D>();
            cache = (distance + radius) * (distance + radius);
            Vector2 offset = dir.NormalVector() * radius;
            StraightLine2D diameterStraightLine = new StraightLine2D(start + offset, start - offset);

            float minX, maxX, minY, maxY;
            if (diameterStraightLine.A.x <= diameterStraightLine.B.x)
            {
                minX = diameterStraightLine.A.x;
                maxX = diameterStraightLine.B.x;
            }
            else
            {
                minX = diameterStraightLine.B.x;
                maxX = diameterStraightLine.A.x;
            }
            if (diameterStraightLine.A.y <= diameterStraightLine.B.y)
            {
                minY = diameterStraightLine.A.y;
                maxY = diameterStraightLine.B.y;
            }
            else
            {
                minY = diameterStraightLine.B.y;
                maxY = diameterStraightLine.A.y;
            }

            Line2D side;
            Vector2 point1, point2;
            for (int i = 0; i < polygone.vertices.Length; i++)
            {
                side = new Line2D(polygone.vertices[i], polygone.vertices[(i + 1) % polygone.vertices.Length]);
                if (dir.Dot(side.A - start) < 0f && dir.Dot(side.B - start) < 0f)
                    continue;

                point1 = diameterStraightLine.OrthogonalProjection(side.A);
                point2 = diameterStraightLine.OrthogonalProjection(side.B);
                if (point1.SqrDistance(side.A) > cache && point2.SqrDistance(side.B) > cache)
                    continue;

                bool isCircleCastPossible = minX < point1.x && point1.x <= maxX && minY <= point1.y && point1.y <= maxY;
                isCircleCastPossible = isCircleCastPossible || (minX < point2.x && point2.x <= maxX && minY <= point2.y && point2.y <= maxY);
                isCircleCastPossible = isCircleCastPossible || MathF.Sign(offset.Dot(point1 - start)) != MathF.Sign(offset.Dot(point2 - start));
                if (isCircleCastPossible)
                {
                    sides.Add(side);
                }
            }

            if (sides.Count <= 0)
                return new ToricRaycastHit2D(Vector2.zero, Vector2.zero, null, Vector2.zero, -1f);

            StraightLine2D straightLine1 = new StraightLine2D(diameterStraightLine.A, diameterStraightLine.A + dir);
            StraightLine2D straightLine2 = new StraightLine2D(diameterStraightLine.B, diameterStraightLine.B + dir);

            List<Line2D> sides2 = new List<Line2D>();
            StraightLine2D sideStraightLine;
            Vector2 a, b, c;
            for (int i = 0; i < sides.Count; i++)
            {
                side = sides[i];
                sideStraightLine = new StraightLine2D(side.A, side.B);
                if (!Collider2D.CollideStraightLines(straightLine1, sideStraightLine, out point1))
                    point1 = side.A;
                if (!Collider2D.CollideStraightLines(straightLine2, sideStraightLine, out point2))
                    point2 = side.A;

                if (MathF.Abs(side.A.x - point1.x) < 1e-3f && MathF.Abs(side.B.x - point2.x) < 1e-3f && MathF.Abs(side.A.x - side.B.x) < 1e-3f)
                {
                    (a, b, c) = RemoveMin(side.A, side.B, point1, point2, false);
                    (point1, point2) = RemoveMax(a, b, c, false);
                }
                else
                {
                    (a, b, c) = RemoveMin(side.A, side.B, point1, point2, true);
                    (point1, point2) = RemoveMax(a, b, c, true);
                }
                (Vector2, Vector2, Vector2) RemoveMin(Vector2 a, Vector2 b, Vector2 c, Vector2 d, bool x)
                {
                    if (x)
                    {
                        if (a.x < b.x && a.x < c.x && a.x < d.x)
                            return (b, c, d);
                        if (b.x < c.x && b.x < d.x)
                            return (a, c, d);
                        return c.x < d.x ? (a, b, d) : (a, b, c);
                    }
                    else
                    {
                        if (a.y < b.y && a.y < c.y && a.y < d.y)
                            return (b, c, d);
                        if (b.y < c.y && b.y < d.y)
                            return (a, c, d);
                        return c.y < d.y ? (a, b, d) : (a, b, c);
                    }
                }
                (Vector2, Vector2) RemoveMax(Vector2 a, Vector2 b, Vector2 c, bool x)
                {
                    if (x)
                    {
                        if (a.x > b.x && a.x > c.x)
                            return (b, c);
                        return b.x > c.x ? (a, c) : (a, b);
                    }
                    else
                    {
                        if (a.y > b.y && a.y > c.y)
                            return (b, c);
                        return b.y > c.y ? (a, c) : (a, b);
                    }
                }

                if (dir.Dot(point1 - start) > 0f || dir.Dot(point2 - start) > 0f)
                {
                    sides2.Add(new Line2D(point1, point2));
                }
            }
            sides.Clear();

            Vector2 minCenter, maxCenter;
            straightLine1.A = start;
            straightLine1.B = start + dir;
            ToricRaycastHit2D? res = null;

            for (int i = 0; i < sides2.Count; i++)
            {
                void FindCenterRecur(Vector2 start, Line2D line, Vector2 minCenter, Vector2 maxCenter, float radius, float maxDistanceSqr, ref ToricRaycastHit2D? lastCollideHit)
                {
                    if (minCenter.SqrDistance(maxCenter) < 1e-5f || minCenter.SqrDistance(start) > maxDistanceSqr)
                        return;

                    Vector2 avgCenter = (minCenter + maxCenter) * 0.5f;
                    if (Collider2D.CollideCircleLine(new Circle(avgCenter, radius), line, out Vector2 inter, out Vector2 n))
                    {
                        lastCollideHit = new ToricRaycastHit2D(inter, avgCenter, null, -n, start.SqrDistance(avgCenter));
                        FindCenterRecur(start, line, minCenter, avgCenter, radius, maxDistanceSqr, ref lastCollideHit);
                        return;
                    }
                    FindCenterRecur(start, line, avgCenter, maxCenter, radius, maxDistanceSqr, ref lastCollideHit);
                }

                side = sides2[i];
                minCenter = straightLine1.OrthogonalProjection(side.A);
                maxCenter = straightLine1.OrthogonalProjection(side.B);

                if (start.SqrDistance(minCenter) > start.SqrDistance(maxCenter))
                {
                    a = minCenter;
                    minCenter = maxCenter;
                    maxCenter = a;
                }

                minCenter -= radius * dir;
                float d = start.SqrDistance(minCenter);
                if ((res.HasValue && d > res.Value.distance) || (bestDistanceFound >= 0f && d > bestDistanceFound * bestDistanceFound))
                {
                    continue;
                }

                ToricRaycastHit2D? bestHit = null;
                FindCenterRecur(start, side, minCenter, maxCenter, radius, cache, ref bestHit);
                if (bestHit.HasValue)
                {
                    if (!res.HasValue || bestHit.Value.distance < res.Value.distance)
                    {
                        res = bestHit.Value;
                    }
                }
            }

            if (!res.HasValue)
                return new ToricRaycastHit2D(Vector2.zero, Vector2.zero, null, Vector2.zero, -1f);

            ToricRaycastHit2D toricRaycastHit = res.Value;
            toricRaycastHit.distance = MathF.Sqrt(toricRaycastHit.distance);
            return toricRaycastHit;
        }

        #endregion

        #region CircleCast Hitbox

        private static ToricRaycastHit2D Physics2DCircleCastHitbox(Vector2 start, Vector2 dir, float radius, float distance, float bestDistanceFound, Hitbox hitbox)
        {
            return Physics2DCircleCastPolygone(start, dir, radius, distance, bestDistanceFound, hitbox.ToPolygone());
        }

        #endregion

        #region CircleCast Capsule

        private static ToricRaycastHit2D Physics2DCircleCastCapsule(Vector2 start, Vector2 dir, float radius, float distance, float bestDistanceFound, Capsule capsule)
        {
            float cache = capsule.center.SqrDistance(start);
            if (cache > (radius + distance + capsule.inclusiveCircle.radius) * (radius + distance + capsule.inclusiveCircle.radius) ||
                (bestDistanceFound >= 0f && cache > (bestDistanceFound + capsule.inclusiveCircle.radius + radius) * (bestDistanceFound + capsule.inclusiveCircle.radius + radius)))
            {
                return new ToricRaycastHit2D(Vector2.zero, Vector2.zero, null, Vector2.zero, -1f);
            }

            ToricRaycastHit2D rayC1 = Physics2DCircleCastCircle(start, dir, radius, distance, bestDistanceFound, capsule.circle1);
            ToricRaycastHit2D rayC2 = Physics2DCircleCastCircle(start, dir, radius, distance, bestDistanceFound, capsule.circle2);
            ToricRaycastHit2D rayHitbox = Physics2DCircleCastHitbox(start, dir, radius, distance, bestDistanceFound, capsule.hitbox);

            if (rayC1.distance >= 0f || rayC2.distance >= 0f || rayHitbox.distance >= 0f)
            {
                if (rayC1.distance >= 0f && (rayC1.distance <= rayC2.distance || rayC2.distance < 0f) && (rayC1.distance <= rayHitbox.distance || rayHitbox.distance < 0f))
                    return rayC1;

                if (rayC2.distance >= 0f && (rayC2.distance <= rayHitbox.distance || rayHitbox.distance < 0f))
                    return rayC2;
                return rayHitbox;
            }

            return new ToricRaycastHit2D(Vector2.zero, Vector2.zero, null, Vector2.zero, -1f);
        }

        #endregion

        #endregion

        #region Circle Cast All

        private static ToricRaycastHit2D[] Physics2DCircleCastAll(Vector2 start, Vector2 dir, float radius, float distance)
        {
            ToricRaycastHit2D raycast;
            List<ToricRaycastHit2D> res = new List<ToricRaycastHit2D>();
            foreach (Collider2D col in colliders)
            {
                raycast = circleCastFunction[col.GetType()](start, dir, radius, distance, -1f, col);

                if (raycast.distance >= 0f)
                {
                    raycast.collider = col;
                    res.Add(raycast);
                }
            }

            int CompareToricRaycastHit2D(ToricRaycastHit2D hit1, ToricRaycastHit2D hit2)
            {
                return MathF.Sign(hit1.distance - hit2.distance);
            }

            res.Sort(CompareToricRaycastHit2D);
            return res.ToArray();
        }

        #endregion

        #endregion

        #region Circle Cast Single

        public static ToricRaycastHit2D CircleCast(Vector2 from, Vector2 dir, float radius, float distance)
        {
            List<Vector2> _ = new List<Vector2>();
            ToricRaycastHit2D raycasts = CircleCastRecur(GetPointInsideBounds(from), dir.normalized, radius, distance, 0f, ref _);
            return raycasts;
        }

        public static ToricRaycastHit2D CircleCast(Vector2 from, Vector2 dir, float radius, float distance, out Vector2[] toricIntersections)
        {
            List<Vector2> inter = new List<Vector2>();
            ToricRaycastHit2D raycasts = CircleCastRecur(GetPointInsideBounds(from), dir.normalized, radius, distance, 0f, ref inter);
            toricIntersections = inter.ToArray();
            return raycasts;
        }

        private static ToricRaycastHit2D CircleCastRecur(Vector2 from, Vector2 direction, float radius, float distance, float reachDistance, ref List<Vector2> points)
        {
            ToricRaycastHit2D circleCast;
            Vector2 end = from + direction * distance;
            if (GetToricIntersection(from, end, out Vector2 inter))
            {
                float currentDistance = from.Distance(inter);
                circleCast = Physics2DCircleCast(from, direction, radius, currentDistance);

                if (circleCast.collider == null)
                {
                    reachDistance += currentDistance;
                    points.Add(inter);
                    inter = GetComplementaryPoint(inter);
                    return RaycastRecur(inter, direction, distance - currentDistance, reachDistance, ref points);
                }
                else
                {
                    if (!mapHitbox.Contains(circleCast.point))
                    {
                        reachDistance += currentDistance;
                        points.Add(inter);
                        inter = GetComplementaryPoint(inter);
                        return RaycastRecur(inter, direction, distance - currentDistance, reachDistance, ref points);
                    }

                    circleCast.distance += reachDistance;
                    return circleCast;
                }
            }
            else //ez case
            {
                circleCast = Physics2DCircleCast(from, direction, radius, distance);

                if (circleCast.collider != null)
                {
                    if (!mapHitbox.Contains(circleCast.point))
                        return default(ToricRaycastHit2D);

                    circleCast.distance += reachDistance;
                }
                return circleCast;
            }
        }

        #endregion

        #region Circle Cast All

        public static ToricRaycastHit2D[] CircleCastAll(Vector2 from, Vector2 dir, float radius, float distance)
        {
            ToricRaycastHit2D[] raycasts = CircleCastRecurAll(GetPointInsideBounds(from), dir.normalized, radius, distance, 0f);
            return raycasts;
        }

        public static ToricRaycastHit2D[] CircleCastAll(Vector2 from, Vector2 dir, float radius, float distance, out Vector2[][] toricIntersections)
        {
            List<List<Vector2>> interPoints = new List<List<Vector2>>();
            List<Vector2> globalInterPoints = new List<Vector2>();
            ToricRaycastHit2D[] raycasts = CircleCastRecurAll(GetPointInsideBounds(from), dir.normalized, radius, distance, 0f, ref globalInterPoints, ref interPoints);

            toricIntersections = new Vector2[interPoints.Count][];
            for (int i = 0; i < toricIntersections.Length; i++)
            {
                toricIntersections[i] = new Vector2[interPoints[i].Count];
                for (int j = 0; j < toricIntersections[i].Length; j++)
                {
                    toricIntersections[i][j] = interPoints[i][j];
                }
            }
            return raycasts;
        }

        private static ToricRaycastHit2D[] CircleCastRecurAll(Vector2 from, Vector2 direction, float radius, float distance, float reachDistance)
        {
            Vector2 end = from + (distance * direction);
            if (GetToricIntersection(from, end, out Vector2 inter))
            {
                float currentDist = from.Distance(inter);
                ToricRaycastHit2D[] circlesCasts = Physics2DCircleCastAll(from, direction, radius, currentDist);

                List<ToricRaycastHit2D> resList = new List<ToricRaycastHit2D>(circlesCasts.Length);
                for (int i = 0; i < circlesCasts.Length; i++)
                {
                    if (mapHitbox.Contains(circlesCasts[i].point))
                    {
                        resList.Add(circlesCasts[i]);
                    }
                }

                ToricRaycastHit2D[] res = new ToricRaycastHit2D[resList.Count];
                for (int i = 0; i < resList.Count; i++)
                {
                    res[i] = resList[i];
                    res[i].distance += reachDistance;
                }

                reachDistance += currentDist;
                return res.Merge(CircleCastRecurAll(GetComplementaryPoint(inter), direction, radius, distance - currentDist, reachDistance));
            }
            else
            {
                ToricRaycastHit2D[] circlesCasts = Physics2DCircleCastAll(from, direction, radius, distance);

                List<ToricRaycastHit2D> resList = new List<ToricRaycastHit2D>();
                for (int i = 0; i < circlesCasts.Length; i++)
                {
                    if (mapHitbox.Contains(circlesCasts[i].point))
                    {
                        resList.Add(circlesCasts[i]);
                    }
                }

                ToricRaycastHit2D[] res = new ToricRaycastHit2D[resList.Count];
                for (int i = 0; i < resList.Count; i++)
                {
                    res[i] = resList[i];
                    res[i].distance += reachDistance;
                }
                return res;
            }
        }

        private static ToricRaycastHit2D[] CircleCastRecurAll(Vector2 from, Vector2 direction, float radius, float distance, float reachDistance, ref List<Vector2> globalInterPoints, ref List<List<Vector2>> interPoints)
        {
            Vector2 end = from + (distance * direction);
            if (GetToricIntersection(from, end, out Vector2 inter))
            {
                float currentDist = from.Distance(inter);
                ToricRaycastHit2D[] circlesCasts = Physics2DCircleCastAll(from, direction, radius, currentDist);

                List<ToricRaycastHit2D> resList = new List<ToricRaycastHit2D>(circlesCasts.Length);
                for (int i = 0; i < circlesCasts.Length; i++)
                {
                    if (mapHitbox.Contains(circlesCasts[i].point))
                    {
                        resList.Add(circlesCasts[i]);
                    }
                }

                ToricRaycastHit2D[] res = new ToricRaycastHit2D[resList.Count];
                for (int i = 0; i < resList.Count; i++)
                {
                    res[i] = resList[i];
                    res[i].distance += reachDistance;
                    interPoints.Add(globalInterPoints.Clone());
                }

                reachDistance += currentDist;
                globalInterPoints.Add(inter);
                return res.Merge(CircleCastRecurAll(GetComplementaryPoint(inter), direction, radius, distance - currentDist, reachDistance, ref globalInterPoints, ref interPoints));
            }
            else
            {
                ToricRaycastHit2D[] circlesCasts = Physics2DCircleCastAll(from, direction, radius, distance);
                List<ToricRaycastHit2D> resList = new List<ToricRaycastHit2D>(circlesCasts.Length);
                for (int i = 0; i < circlesCasts.Length; i++)
                {
                    if (mapHitbox.Contains(circlesCasts[i].point))
                    {
                        resList.Add(circlesCasts[i]);
                    }
                }

                ToricRaycastHit2D[] res = new ToricRaycastHit2D[resList.Count];
                for (int i = 0; i < resList.Count; i++)
                {
                    res[i] = resList[i];
                    res[i].distance += reachDistance;
                    interPoints.Add(globalInterPoints.Clone());
                }

                return res;
            }
        }

        #endregion

        #endregion

        #endregion
    }

}

