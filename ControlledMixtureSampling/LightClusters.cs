using SeeSharp.Shading.Emitters;

namespace OptimalCV;

/// <summary>
/// Builds a quick-and-dirty clustering of light sources, inspired by the lightcuts tree construction.
/// A sort of Monte Carlo greedy best first bottom up construction: picks random pairs of clusters and
/// merges the pair with highest similarity.
/// </summary>
class LightClusters {
    class Cluster {
        public HashSet<int> Lights;
        public BoundingCone DirectionBounds;
        public BoundingBox PositionBounds;
        public float Power;
    }

    List<Cluster> clusters;
    Dictionary<int, int> emitterToCluster;
    Dictionary<Emitter, int> emitterObjToCluster;
    float spatioDirectionalTradeoff;
    Scene scene;

    float EvalSimilarity(Cluster a, Cluster b)
    {
        // Taken from the lightcuts paper
        float power = a.Power + b.Power;
        var bounds = a.PositionBounds.GrowToContain(b.PositionBounds);
        var cone = a.DirectionBounds.GrowToContain(b.DirectionBounds);
        float cos = 1.0f - MathF.Cos(cone.HalfAngle);

        return power * (bounds.Diagonal.LengthSquared() + spatioDirectionalTradeoff * cos * cos);
    }

    Cluster Merge(Cluster a, Cluster b) {
        Cluster c = new() {
            Power = a.Power + b.Power,
            DirectionBounds = a.DirectionBounds.GrowToContain(b.DirectionBounds),
            PositionBounds = a.PositionBounds.GrowToContain(b.PositionBounds),
            Lights = a.Lights
        };
        c.Lights.UnionWith(b.Lights);
        return c;
    }

    void MergeClusters(int targetLightCount) {
        // Randomly combine similar lights until the target count is reached
        RNG rng = new(1337);

        while (clusters.Count > targetLightCount) {
            int i = rng.NextInt(0, clusters.Count);

            // Compare similarity to 100 random other clusters, and merge the best combination
            float bestVal = float.MaxValue;
            int bestIdx = 0;
            for (int k = 0; k < 100; ++k) {
                int j = rng.NextInt(0, clusters.Count);
                if (j == i) continue;
                float s = EvalSimilarity(clusters[i], clusters[j]);
                if (s < bestVal) {
                    bestVal = s;
                    bestIdx = j;
                }
            }

            clusters.Add(Merge(clusters[i], clusters[bestIdx]));
            clusters.RemoveAt(Math.Max(i, bestIdx));
            clusters.RemoveAt(Math.Min(i, bestIdx));
        }
    }

    public LightClusters(Scene scene, int targetLightCount = 100) {
        this.scene = scene;

        clusters = new();
        for (int i = 0; i < scene.Emitters.Count; ++i) {
            var l = scene.Emitters[i];
            var n = l.Mesh.FaceNormals[0];
            var bounds = BoundingBox.Empty
                .GrowToContain(l.Mesh.Vertices[0])
                .GrowToContain(l.Mesh.Vertices[1])
                .GrowToContain(l.Mesh.Vertices[2]);
            clusters.Add(new() {
                Lights = new() { i },
                DirectionBounds = new BoundingCone(n),
                PositionBounds = bounds,
                Power = l.ComputeTotalPower().Average
            });
        }

        if (scene.Emitters.Count <= targetLightCount)
            return;

        spatioDirectionalTradeoff = scene.Bounds.Diagonal.LengthSquared();

        MergeClusters(targetLightCount);

        emitterToCluster = new();
        emitterObjToCluster = new();
        for (int i = 0; i < clusters.Count; ++i) {
            foreach (var lightIdx in clusters[i].Lights)
            {
                emitterToCluster.Add(lightIdx, i);
                emitterObjToCluster.Add(scene.Emitters[lightIdx], i);
            }
        }
    }

    public int GetClusterIdx(int emitterIdx)
    => emitterToCluster?[emitterIdx] ?? emitterIdx;

    public int GetClusterIdx(Emitter emitter)
    => emitterObjToCluster?[emitter] ?? scene.GetEmitterIndex(emitter);

    public int GetNumLightsInCluster(int clusterIdx) => clusters[clusterIdx].Lights.Count;

    public int NumClusters => clusters.Count;
}
