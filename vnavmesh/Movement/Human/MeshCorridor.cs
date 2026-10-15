using DotRecast.Core.Numerics;
using DotRecast.Detour;
using Navmesh.NavVolume;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Threading;

namespace Navmesh.Movement.Human;

// Asks the walking mesh for a route and hands back its corners, for VoxelCorridor to follow
// through the volume when the volume search on its own fell short. See ADR 0010.
//
// Detour is happy to answer with a partial route when the two ends are not connected on the mesh,
// and where it stops matters: a partial route from the start side may end at a cliff edge right
// next to the goal, which is one easy flight away, or it may end at the near side of the map with
// the whole problem still ahead. So when the route is partial, the reverse route is asked for too
// and the one that leaves the shorter gap is used.
public static class MeshCorridor
{
    // how far around an end point to look for mesh. a flying start can be well above the ground,
    // or over ground too steep to carry any mesh for a long way around, so the box is wide and tall
    private static readonly RcVec3f Reach = new(150, 400, 150);

    // how many nearby mesh points to try before giving up on an end point, and how many expansions
    // each try may spend. the nearest point is usually the one, but a start hovering over a slope
    // grazes it on the way down and needs a few hundred expansions to get round. tries are spread
    // over distinct cells of this size, so one unreachable patch of mesh cannot use them all up
    private const int Candidates = 32;
    private const int CandidateBudget = 2000;
    private const float CandidateSpacing = 16;

    // node expansions one mesh query may spend. a route Detour cannot complete is a flood of the whole
    // mesh otherwise, which on a large zone is a few hundred milliseconds spent on top of a volume
    // search that already failed. the sliced query hands back the best partial route when this runs out
    public const int Iterations = 20000;

    // null when either end has no mesh it can fly to or Detour cannot make a route at all
    public static List<Vector3>? Corners(DtNavMeshQuery query, IDtQueryFilter filter, VoxelPathfind search, Vector3 from, Vector3 to, bool useRaycast, CancellationToken cancel)
    {
        var start = NearestReachable(query, filter, search, from, useRaycast, cancel);
        var end = NearestReachable(query, filter, search, to, useRaycast, cancel);
        if (start == null || end == null)
            return null;

        var forward = Route(query, filter, start.Value, end.Value);
        if (forward == null)
            return null;
        if (forward.Value.complete)
            return forward.Value.corners;

        var backward = Route(query, filter, end.Value, start.Value);
        if (backward is { } b && b.gap < forward.Value.gap)
        {
            b.corners.Reverse();
            return b.corners;
        }
        return forward.Value.corners;
    }

    // the nearest point on the mesh that the volume agrees can be flown to. the nearest polygon by
    // distance alone can be the floor of a cavern directly under a flier and 300 y of rock away,
    // which no corridor leg could reach; so each candidate is tried with a short search, nearest first
    private static (long poly, RcVec3f point)? NearestReachable(DtNavMeshQuery query, IDtQueryFilter filter, VoxelPathfind search, Vector3 p, bool useRaycast, CancellationToken cancel)
    {
        var volume = search.Volume;
        var fromVoxel = VoxelSearch.FindNearestEmptyVoxel(volume, p, new(5, 5, 5));
        if (fromVoxel == VoxelMap.InvalidVoxel)
            return null;

        var polys = new Collector();
        query.QueryPolygons(p.SystemToRecast(), Reach, filter, polys);
        var candidates = new List<(long poly, RcVec3f point, float dist)>(polys.Refs.Count);
        foreach (var poly in polys.Refs)
        {
            if (query.ClosestPointOnPoly(poly, p.SystemToRecast(), out var closest, out _).Failed())
                continue;
            candidates.Add((poly, closest, Vector3.DistanceSquared(closest.RecastToSystem(), p)));
        }
        candidates.Sort((a, b) => a.dist.CompareTo(b.dist));

        var settings = search.Settings;
        search.Settings = settings with { MaxSteps = Math.Min(settings.MaxSteps, CandidateBudget) };
        try
        {
            var tried = new HashSet<(int, int, int)>();
            foreach (var (poly, point, _) in candidates)
            {
                if (tried.Count >= Candidates)
                    break;
                var target = point.RecastToSystem();
                var cell = ((int)MathF.Floor(target.X / CandidateSpacing), (int)MathF.Floor(target.Y / CandidateSpacing), (int)MathF.Floor(target.Z / CandidateSpacing));
                if (!tried.Add(cell))
                    continue;
                var toVoxel = VoxelSearch.FindNearestEmptyVoxel(volume, target, new(5, 5, 5));
                if (toVoxel == VoxelMap.InvalidVoxel)
                    continue;
                if (fromVoxel == toVoxel)
                    return (poly, point);
                cancel.ThrowIfCancellationRequested();
                var lifted = VoxelSearch.FindClosestVoxelPoint(volume, toVoxel, target);
                var leg = search.FindPath(fromVoxel, toVoxel, p, lifted, useRaycast, false, cancel);
                if (leg.Count > 0 && leg[^1].voxel == toVoxel)
                    return (poly, point);
            }
            return null;
        }
        finally
        {
            search.Settings = settings;
        }
    }

    private static (List<Vector3> corners, bool complete, float gap)? Route(DtNavMeshQuery query, IDtQueryFilter filter, (long poly, RcVec3f point) start, (long poly, RcVec3f point) end)
    {
        var status = query.InitSlicedFindPath(start.poly, end.poly, start.point, end.point, filter, 0);
        for (var left = Iterations; status.InProgress() && left > 0;)
        {
            status = query.UpdateSlicedFindPath(Math.Min(left, 1000), out var done);
            left -= Math.Max(done, 1);
        }
        var polys = new List<long>();
        status = query.FinalizeSlicedFindPath(ref polys); // a query still in progress finalizes to its best partial route
        if (status.Failed() || polys.Count == 0)
            return null;

        var straight = new List<DtStraightPath>();
        if (query.FindStraightPath(start.point, end.point, polys, ref straight, 1024, 0).Failed() || straight.Count == 0)
            return null;

        var corners = straight.Select(s => s.pos.RecastToSystem()).ToList();
        return (corners, !status.IsPartial(), RcVec3f.Distance(corners[^1].SystemToRecast(), end.point));
    }

    private sealed class Collector : IDtPolyQuery
    {
        public readonly List<long> Refs = [];
        public void Process(DtMeshTile tile, DtPoly poly, long refs) => Refs.Add(refs);
    }
}
