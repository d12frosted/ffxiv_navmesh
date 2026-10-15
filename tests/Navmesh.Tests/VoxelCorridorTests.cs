using System.Numerics;
using Navmesh.NavVolume;
using Xunit;

namespace Navmesh.Tests;

// Covers the mesh-guided fallback from ADR 0010. The corridor is a chain of corner points that
// some other source (in the plugin, the walking mesh) already knows leads to the goal; the helper
// turns it into a volume path leg by leg. Every test that returns a path also checks that no hop
// passes through geometry, since a fallback that cuts corners would be worse than none.
public class VoxelCorridorTests
{
    private static VoxelPathfindTuning Tuning(int maxSteps = 1000000)
        => new() { HeuristicWeight = 1, SearchRaycast = true, RaycastRange = 0, MaxSteps = maxSteps, RandomnessMultiplier = 0 };

    // a wall across the whole world at z=15..16 with a gap at the far x end. the straight line
    // from start to goal is blocked, and the only way round is away from the goal first, which
    // is the shape of the burrow route the fallback exists for
    private static VoxelWorld Cave()
    {
        var w = new VoxelWorld(8, 2, 2); // 32 units a side
        w.SolidBox(0, 0, 15, 31, 31, 16);
        w.EmptyBox(28, 0, 15, 31, 31, 16);
        return w;
    }

    private static readonly (int x, int y, int z) Start = (2, 2, 2);
    private static readonly (int x, int y, int z) Goal = (2, 2, 30);
    private static readonly Vector3[] CaveCorners = [new(2.5f, 2.5f, 12.5f), new(29.5f, 2.5f, 12.5f), new(29.5f, 2.5f, 19.5f), new(2.5f, 2.5f, 19.5f)];

    private static List<(ulong voxel, Vector3 p)>? Follow(VoxelWorld w, VoxelPathfind pf, (int x, int y, int z) from, (int x, int y, int z) to, IReadOnlyList<Vector3> corners)
        => VoxelCorridor.Follow(pf, w.At(from.x, from.y, from.z), w.At(to.x, to.y, to.z), corners, true, default);

    private static void AssertEveryHopIsClear(VoxelWorld w, List<(ulong voxel, Vector3 p)> path)
    {
        for (var i = 1; i < path.Count; ++i)
            Assert.True(w.Visible(path[i - 1], path[i]), $"hop {path[i - 1].p} -> {path[i].p} passes through geometry");
    }

    [Fact]
    public void ThePlainSearchRunsOutOfBudgetInTheCave()
    {
        var w = Cave();
        var pf = new VoxelPathfind(w.Map) { Settings = Tuning(maxSteps: 40) };
        var a = w.At(Start.x, Start.y, Start.z);
        var b = w.At(Goal.x, Goal.y, Goal.z);

        var path = pf.FindPath(a.voxel, b.voxel, a.p, b.p, true, false, default);

        // this is the failure the fallback exists for: a partial path that stops short of the goal
        Assert.NotEmpty(path);
        Assert.NotEqual(b.voxel, path[^1].voxel);
    }

    [Fact]
    public void FollowingTheCornersReachesTheGoalOnTheSameBudget()
    {
        var w = Cave();
        var pf = new VoxelPathfind(w.Map) { Settings = Tuning(maxSteps: 40) };

        var path = Follow(w, pf, Start, Goal, CaveCorners);

        Assert.NotNull(path);
        Assert.Equal(w.At(Start.x, Start.y, Start.z).p, path[0].p);
        Assert.Equal(w.At(Goal.x, Goal.y, Goal.z).voxel, path[^1].voxel);
        Assert.Equal(w.At(Goal.x, Goal.y, Goal.z).p, path[^1].p);
        AssertEveryHopIsClear(w, path);
    }

    [Fact]
    public void CornersInLineOfSightNeedNoSearch()
    {
        var w = new VoxelWorld(8, 2, 2);
        var pf = new VoxelPathfind(w.Map) { Settings = Tuning(maxSteps: 1) }; // a budget that cannot find anything
        Vector3[] corners = [new(10.5f, 2.5f, 2.5f), new(10.5f, 2.5f, 20.5f)];

        var path = Follow(w, pf, Start, Goal, corners);

        Assert.NotNull(path);
        Assert.Equal([w.At(Start.x, Start.y, Start.z).p, corners[0], corners[1], w.At(Goal.x, Goal.y, Goal.z).p], path.Select(p => p.p));
    }

    [Fact]
    public void ACornerOnTheFloorIsLiftedIntoEmptySpace()
    {
        var w = new VoxelWorld(8, 2, 2);
        w.SolidBox(0, 0, 0, 31, 0, 31); // a floor, which is where a mesh corner always sits
        var pf = new VoxelPathfind(w.Map) { Settings = Tuning() };
        Vector3[] corners = [new(16.5f, 0.5f, 16.5f)]; // inside the floor cell

        var path = Follow(w, pf, (2, 2, 2), (30, 2, 30), corners);

        Assert.NotNull(path);
        var lifted = path.Single(p => MathF.Abs(p.p.X - 16.5f) < 0.01f && MathF.Abs(p.p.Z - 16.5f) < 0.01f);
        Assert.True(w.Map.IsEmpty(lifted.voxel));
        Assert.True(lifted.p.Y >= 1, $"corner stayed at {lifted.p.Y}");
        AssertEveryHopIsClear(w, path);
    }

    [Fact]
    public void ACornerBuriedInGeometryDeclines()
    {
        var w = new VoxelWorld(8, 2, 2);
        w.SolidBox(8, 0, 8, 23, 23, 23); // nothing empty within reach of its centre
        var pf = new VoxelPathfind(w.Map) { Settings = Tuning() };
        Vector3[] corners = [new(15.5f, 11.5f, 15.5f)];

        var path = Follow(w, pf, Start, Goal, corners);

        Assert.Null(path);
    }

    [Fact]
    public void ALegThatCannotBeFinishedDeclines()
    {
        var w = Cave();
        w.SolidBox(0, 0, 15, 31, 31, 16); // close the gap again, so no corridor can work
        var pf = new VoxelPathfind(w.Map) { Settings = Tuning(maxSteps: 2000) };

        var path = Follow(w, pf, Start, Goal, CaveCorners);

        Assert.Null(path);
    }

    [Fact]
    public void TheSearchSettingsAreLeftAsTheyWere()
    {
        var w = Cave();
        var tuning = Tuning(maxSteps: 1000000);
        var pf = new VoxelPathfind(w.Map) { Settings = tuning };

        Follow(w, pf, Start, Goal, CaveCorners);

        Assert.Equal(tuning, pf.Settings);
    }

    [Fact]
    public void AHopelessLegGivesUpWithinTheLegBudget()
    {
        // pillars everywhere so the world is leaf cells rather than a few coarse ones, and a goal
        // sealed in a box; the last leg can only flood, and the flood must stop at the leg budget
        // rather than at the caller's
        var w = new VoxelWorld(8, 4, 2); // 64 units a side
        for (var x = 3; x < 62; x += 6)
            for (var z = 3; z < 62; z += 6)
                w.SolidBox(x, 0, z, x + 1, 63, z + 1);
        w.SolidBox(40, 0, 40, 46, 6, 46);
        w.EmptyBox(42, 1, 42, 44, 4, 44);
        var pf = new VoxelPathfind(w.Map) { Settings = Tuning(maxSteps: 1000000) };

        var path = Follow(w, pf, (1, 2, 1), (43, 2, 43), [new(20.5f, 2.5f, 20.5f)]);

        Assert.Null(path);
        Assert.True(pf.NodeSpan.Length < 2 * VoxelCorridor.LegBudget, $"the failing leg visited {pf.NodeSpan.Length} nodes");
    }

    [Fact]
    public void CornersInTheSameCellDoNotProduceEmptyHops()
    {
        var w = new VoxelWorld(8, 2, 2);
        var pf = new VoxelPathfind(w.Map) { Settings = Tuning() };
        Vector3[] corners = [new(10.2f, 2.5f, 10.2f), new(10.8f, 2.5f, 10.8f), new(10.5f, 2.5f, 10.5f)];

        var path = Follow(w, pf, Start, Goal, corners);

        Assert.NotNull(path);
        for (var i = 1; i < path.Count; ++i)
            Assert.NotEqual(path[i - 1].voxel, path[i].voxel);
    }

    [Fact]
    public void NoCornersIsJustOneLeg()
    {
        var w = new VoxelWorld(8, 2, 2);
        var pf = new VoxelPathfind(w.Map) { Settings = Tuning() };

        var path = Follow(w, pf, Start, Goal, []);

        Assert.NotNull(path);
        Assert.Equal(w.At(Goal.x, Goal.y, Goal.z).voxel, path[^1].voxel);
        AssertEveryHopIsClear(w, path);
    }
}
