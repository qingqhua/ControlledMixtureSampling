using SeeSharp.Shading.Emitters;

namespace OptimalCV;

class NaivePerLightCV
{
    public NaivePerLightCV(int numLights)
    {
        numPartitioned = numLights;
        contribs = new RgbColor[numLights];
        coefficients = new RgbColor[numLights];
    }

    public RgbColor SumCoefficients { get; private set; }

    public int numPartitioned;

    public void Solve()
    {
        if (numSamples == 0) return;

        SumCoefficients = RgbColor.Black;
        for (int i = 0; i < numPartitioned; ++i)
        {
            coefficients[i] = contribs[i] / numSamples;
            SumCoefficients += coefficients[i];
            Debug.Assert(float.IsFinite(coefficients[i].Average));
        }
    }

    RgbColor[] contribs;
    RgbColor[] coefficients;
    int numSamples;

    public void Update(RgbColor estimate, int lightIdx)
    {
        lock (contribs)
        {
            contribs[lightIdx] += estimate;
            numSamples++;
            Debug.Assert(float.IsFinite(estimate.Average));
        }
    }

    public RgbColor Evaluate(int lightIdx, float pdf) => -1 * coefficients[lightIdx] / pdf;

    public RgbColor this[int lightIdx] => coefficients[lightIdx];
}

/// <summary>
/// This integrator only takes per light cv, not include bsdf or guiding into cv
/// </summary>
partial class PerLightCVRenderer : PathTracer {
    #region PARAMETERS

    /// <summary>
    /// If false, renders with classic mixture sampling / balance heuristic MIS
    /// </summary>
    public bool EnableCV = true;

    /// <summary>
    /// Number of samples before the first CV computation
    /// </summary>
    public int TrainingSamples = 2;

    /// <summary>
    /// Maximum number of lights for which to compute the CV. If there are more lights in the scene, apply
    /// a simple clustering to reduce the number to this threshold.
    /// </summary>
    public int MaxNumLights = 500;

    /// <summary>
    /// Specify num of grid resolution. If not specify, use adaptive grid. 
    /// </summary>
    public int MaxNumVoxels = 0;

    /// <summary>
    /// If true, uses OpenPGL path guiding.
    /// If false, uses Bsdf importance sampling as CV.
    /// </summary>
    public bool EnablePathGuiding = true;

    public bool RenderPathLengths = false;

    #endregion PARAMETERS

    SparseGrid<NaivePerLightCV> cvOptimizer;
    bool cvReady;
    LightClusters clusters;
    int numGuidingLobes = 0;

    List<RgbLayer> pathDepthImages;

    /// <summary>
    /// Decides the resolution of the sparse grid for the current scene based on a pixel footprint
    /// heuristic. Similar to the initial radius of the photon mapper.
    /// </summary>
    /// <returns>Maximum number of voxels along the widest axis of the scene</returns>
    int AdaptGridResolution() {
        int numSamples = 100;

        // Sample a small number of primary rays and compute the average pixel footprint area
        var resolution = new Vector2(scene.FrameBuffer.Width, scene.FrameBuffer.Height);
        float averageArea = 0;
        int numHits = 0;
        RNG dummyRng = new RNG();
        for (int i = 0; i < numSamples; ++i) {
            Ray ray = scene.Camera.GenerateRay(dummyRng.NextFloat2D() * resolution, dummyRng).Ray;
            var hit = scene.Raytracer.Trace(ray);
            if (!hit) continue;

            float areaToAngle = scene.Camera.SurfaceAreaToSolidAngleJacobian(hit.Position, hit.Normal);
            float angleToPixel = scene.Camera.SolidAngleToPixelJacobian(ray.Direction);
            float pixelFootprintArea = 1 / (angleToPixel * areaToAngle);
            averageArea += pixelFootprintArea;
            numHits++;
        }

        if (numHits == 0) {
            Logger.Warning("Error determining pixel footprint: no intersections reported." +
                "Falling back to default of 64.");
            return 64;
        }

        averageArea /= numHits;

        // Compute the radius based on the approximated average footprint area
        // Our heuristic aims to have each photon cover roughly a 16 x 16 region on the image
        float width = MathF.Sqrt(averageArea) * 16;
        float max = Math.Max(Math.Max(scene.Bounds.Diagonal.X, scene.Bounds.Diagonal.Y), scene.Bounds.Diagonal.Z);
        return (int)(max / width);
    }

    protected override void OnPrepareRender()
    {
        // Bound the number of lights in the scene through semi-random clustering, just to keep the
        // overhead in check. Becomes redundant if proper clustering (e.g., lightcuts) is used.
        clusters = new(scene, MaxNumLights);

        // Evaluate the grid size heuristic to adapt the resolution to the scene
        int maxNumVoxels = MaxNumVoxels==0 ? AdaptGridResolution() : MaxNumVoxels;
        Logger.Log($"Grid resolution: {maxNumVoxels}");

        int numLights = clusters.NumClusters;
        if (scene.Emitters.Count == 0)
            numLights = 0;
        if (scene.Background != null) // Account for env map tobe the additional cluster
            numLights += 1;
        if(NumShadowRays==0)
            numLights = 0;

        cvOptimizer = new(scene, () => new NaivePerLightCV(numLights), maxNumVoxels);
        cvReady = false;

        if (RenderPathLengths)
        {
            pathDepthImages = new();
            for (int i = 2; i <= MaxDepth; ++i)
            {
                pathDepthImages.Add(new());
                scene.FrameBuffer.AddLayer($"depth{i}", pathDepthImages[^1]);
            }
        }
    }

    protected override void OnPostIteration(uint iterIdx)
    {
        if (!EnableCV || iterIdx < TrainingSamples) return;

        // Only optimize once. TODO multiple upates in exponential steps may be beneficial
        if (cvReady) return;

        var timer = Stopwatch.StartNew();
        cvOptimizer.ParallelForAllCells(cvOpt =>
        {
            cvOpt.Solve();
        });
        Logger.Log($"Solved all systems in {timer.ElapsedMilliseconds}ms");
        cvReady = true;

        // this can be used to see only the CV rendering without noise from initial iterations
        // scene.FrameBuffer.Reset();
    }

    protected override RgbColor PerformBackgroundNextEvent(in Ray ray, in SurfacePoint hit, PathState state)
    {
        if (scene.Background == null)
            return RgbColor.Black; // There is no background

        var sample = scene.Background.SampleDirection(state.Rng.NextFloat2D());
        bool IsLeftScene = scene.Raytracer.LeavesScene(hit, sample.Direction);

        // Prevent NaN / Inf
        if (/*pdfBsdf == 0 ||*/ sample.Pdf == 0)
            return RgbColor.Black;

        var cvOpt = cvOptimizer.Query(hit.Position);

        RgbColor estimate = RgbColor.Black;
        RgbColor contrib = RgbColor.Black;
        if (IsLeftScene)
        {
            var bsdfTimesCosine = hit.Material.EvaluateWithCosine(hit, -ray.Direction, sample.Direction, false);
            float pdfBsdf = DirectionPdf(hit, -ray.Direction, sample.Direction, state);
            contrib = sample.Weight * bsdfTimesCosine / NumShadowRays;
            float misWeight = EnableBsdfDI ? 1 / (1.0f + pdfBsdf / (sample.Pdf * NumShadowRays)) : 1;
            Debug.Assert(float.IsFinite(contrib.Average));
            Debug.Assert(float.IsFinite(misWeight));

            RegisterSample(state.Pixel, contrib * state.Throughput, misWeight, state.Depth + 1, true);
            OnNextEventResult(ray, hit, state, misWeight, contrib);
            estimate = misWeight * contrib;
        }

        cvOpt.Update(estimate, cvOpt.numPartitioned - 1);
        var cv = -1 * cvOpt[cvOpt.numPartitioned - 1];
        if (scene.Emitters.Count == 0)
            cv += cvOpt.SumCoefficients;
        return estimate + cv;
    }

    protected override RgbColor PerformNextEventEstimation(in Ray ray, in SurfacePoint hit, PathState state)
    {
        if (scene.Emitters.Count == 0)
            return RgbColor.Black;

        // Select a light source
        int idx = state.Rng.NextInt(0, scene.Emitters.Count);
        var light = scene.Emitters[idx];
        float lightSelectProb = 1.0f / scene.Emitters.Count;

        // Sample a point on the light source
        var lightSample = light.SampleUniformArea(state.Rng.NextFloat2D());
        float pdfNextEvt = lightSample.Pdf * lightSelectProb * NumShadowRays;

        int clusterIdx = clusters.GetClusterIdx(idx);
        var cvOpt = cvOptimizer.Query(hit.Position);
        float clusterSelectProb = clusters.GetNumLightsInCluster(clusterIdx) / (float)scene.Emitters.Count;

        // Evaluate balance heuristic weighted MC estimator
        RgbColor estimate = RgbColor.Black;
        float misWeight = 0;
        if (!scene.Raytracer.IsOccluded(hit, lightSample.Point)) {
            Vector3 lightToSurface = Vector3.Normalize(hit.Position - lightSample.Point.Position);
            var bsdfCos = hit.Material.EvaluateWithCosine(hit, -ray.Direction, -lightToSurface, false);
            var emission = light.EmittedRadiance(lightSample.Point, lightToSurface);

            // MIS weighting
            float pdfBsdfSolidAngle = DirectionPdf(hit, -ray.Direction, -lightToSurface, state);
            float jacobian = SampleWarp.SurfaceAreaToSolidAngle(hit, lightSample.Point);
            float pdfBsdf = pdfBsdfSolidAngle * jacobian;
            misWeight = EnableBsdfDI ? 1.0f / (pdfBsdf / pdfNextEvt + 1) : 1;

            // Avoid Inf / NaN
            if (jacobian == 0) return RgbColor.Black;
            Debug.Assert(pdfBsdf != 0 || bsdfCos == RgbColor.Black,
                "Non-zero BSDF value not sampled by forward path tracing!");

            estimate = misWeight * emission * jacobian * bsdfCos / pdfNextEvt;
        }
        OnNextEventResult(ray, hit, state, misWeight, estimate);
        cvOpt.Update(estimate, clusterIdx);
        return estimate + cvOpt.SumCoefficients - cvOpt[clusterIdx] / clusterSelectProb;
    }

    protected override RadianceEstimate EstimateIncidentRadiance(in Ray ray, PathState state) {
        // Trace the next ray
        if (state.Depth > MaxDepth)
            return RadianceEstimate.Absorbed;
        var hit = scene.Raytracer.Trace(ray);

        RgbColor directHitContrib = RgbColor.Black;
        float nextEventPdf = 0;

        if (!hit && state.Depth >= MinDepth) {
            (directHitContrib, nextEventPdf) = OnBackgroundHit(ray, state);
            return new RadianceEstimate {
                Emitted = directHitContrib,
                NextEventPdf = nextEventPdf
            };
        } else if (!hit) {
            return RadianceEstimate.Absorbed;
        }

        OnHit(ray, hit, state);

        if (state.Depth == 1 && EnableDenoiser) {
            var albedo = ((SurfacePoint)hit).Material.GetScatterStrength(hit);
            denoiseBuffers.LogPrimaryHit(state.Pixel, albedo, hit.ShadingNormal);
        }

        // Check if a light source was hit.
        Emitter light = scene.QueryEmitter(hit);
        if (light != null && state.Depth >= MinDepth) {
            (directHitContrib, nextEventPdf) = OnLightHit(ray, hit, state, light);
        }

        RgbColor nextEventContrib = RgbColor.Black;
        // Perform next event estimation
        if (state.Depth + 1 >= MinDepth && state.Depth < MaxDepth) {
            for (int i = 0; i < NumShadowRays; ++i) {
                nextEventContrib += PerformBackgroundNextEvent(ray, hit, state);
                nextEventContrib += PerformNextEventEstimation(ray, hit, state);
            }
        }

        // Terminate early if this is the last desired bounce
        if (state.Depth >= MaxDepth) {
            return new RadianceEstimate {
                Emitted = directHitContrib,
                NextEventPdf = nextEventPdf,
                Reflected = nextEventContrib
            };
        }

        // Sample a direction to continue the random walk
        (var bsdfRay, float bsdfPdf, var bsdfSampleWeight) = SampleDirection(ray, hit, state);
        if (bsdfPdf == 0 || bsdfSampleWeight == RgbColor.Black)
            return new RadianceEstimate {
                Emitted = directHitContrib,
                NextEventPdf = nextEventPdf,
                Reflected = nextEventContrib
            };

        // Recursively estimate the incident radiance and log the result
        var curThroughput = state.Throughput;
        int depth = (int)state.Depth;
        state.Throughput *= bsdfSampleWeight;
        state.Depth += 1;
        state.PreviousHit = hit;
        state.PreviousPdf = bsdfPdf;
        var nested = EstimateIncidentRadiance(bsdfRay, state);

        if (RenderPathLengths)
        {
            var contrib = curThroughput * (nextEventContrib + nested.Emitted * bsdfSampleWeight);
            pathDepthImages[depth - 1].Splat(state.Pixel.Col, state.Pixel.Row, contrib);
        }

        return new RadianceEstimate {
            Emitted = directHitContrib,
            NextEventPdf = nextEventPdf,
            Reflected = nextEventContrib + nested.Outgoing * bsdfSampleWeight
        };
    }
}