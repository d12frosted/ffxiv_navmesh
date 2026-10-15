using System;
using System.Collections.Generic;
using System.Numerics;
using System.Threading;

namespace Navmesh.NavVolume;

// Follows a chain of corner points through the volume, one short search per leg.
//
// The volume A* has no idea of the shape of the zone beyond the voxel in front of it, so a route
// that has to go away from the goal first (out of a burrow, along a tunnel, round a mountain) can
// cost millions of expansions, and on any budget it hands back a path that stops short. Something
// else may already know the shape: in the plugin that is the walking mesh, which Detour searches in
// milliseconds. This takes that route's corners and joins them through the volume, so each search
// is a short one between two points that are known to be close. See ADR 0010.
//
// Corners come from a floor, so the voxel containing one is usually the floor itself; each corner is
// lifted into the nearest empty voxel first. Legs that are in line of sight are taken as they are.
// Any leg that cannot be finished gives up on the whole corridor, since a chain with a gap in it is
// exactly the partial path this exists to replace.
public static class VoxelCorridor
{
    // how far around a corner to look for empty space
    private static readonly Vector3 CornerReach = new(5, 5, 5);

    // expansions any one leg may spend. consecutive corners of a walking route see each other along
    // the floor, so a leg between them is short and mostly in line of sight a little way up; one that
    // needs more than this is a leg with a problem, and giving up on it early is what keeps a hopeless
    // corridor cheap. the caller's own budget still applies when it is smaller
    public const int LegBudget = 20000;

    // null when the corridor cannot be followed; the caller then keeps whatever it had
    public static List<(ulong voxel, Vector3 p)>? Follow(VoxelPathfind search, (ulong voxel, Vector3 p) from, (ulong voxel, Vector3 p) to, IReadOnlyList<Vector3> corners, bool useRaycast, CancellationToken cancel, Vector3? avoidCenter = null, float avoidRadius = 0)
    {
        var volume = search.Volume;
        var stops = new List<(ulong voxel, Vector3 p)>(corners.Count + 2) { from };
        foreach (var corner in corners)
        {
            var voxel = VoxelSearch.FindNearestEmptyVoxel(volume, corner, CornerReach);
            if (voxel == VoxelMap.InvalidVoxel)
                return null; // a corner with no empty space around it is not one the volume can pass through
            if (voxel == stops[^1].voxel)
                continue; // same cell as the previous stop, so a hop to it would be a hop to nowhere
            stops.Add((voxel, VoxelSearch.FindClosestVoxelPoint(volume, voxel, corner)));
        }
        if (stops.Count > 1 && stops[^1].voxel == to.voxel)
            stops.RemoveAt(stops.Count - 1); // the requested end point wins over a corner in the same cell
        stops.Add(to);

        var path = new List<(ulong voxel, Vector3 p)> { from };
        var avoiding = avoidRadius > 0 && avoidCenter.HasValue;
        var settings = search.Settings;
        search.Settings = settings with { MaxSteps = Math.Min(settings.MaxSteps, LegBudget) };
        try
        {
            for (var i = 1; i < stops.Count; ++i)
            {
                cancel.ThrowIfCancellationRequested();
                var a = stops[i - 1];
                var b = stops[i];
                if (a.voxel == b.voxel || !avoiding && Visible(volume, a, b))
                {
                    path.Add(b);
                    continue;
                }
                var leg = search.FindPath(a.voxel, b.voxel, a.p, b.p, useRaycast, false, cancel, avoidCenter, avoidRadius);
                if (leg.Count == 0 || leg[^1].voxel != b.voxel)
                    return null;
                for (var j = 1; j < leg.Count; ++j)
                    path.Add(leg[j]);
            }
            return path;
        }
        finally
        {
            search.Settings = settings;
        }
    }

    private static bool Visible(VoxelMap volume, (ulong voxel, Vector3 p) a, (ulong voxel, Vector3 p) b)
    {
        try
        {
            return VoxelSearch.LineOfSight(volume, a.voxel, b.voxel, a.p, b.p);
        }
        catch (PathfindLoopException)
        {
            return false; // the walk along the line stalled, so nothing is known about it; searching is always safe
        }
    }
}
