using SeeSharp.Shading.Emitters;
using SeeSharp.Shading.Background;

namespace OptimalCV;

class CVRenderer : PathTracer {
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
    /// If true, each light source sampling PDF is used as one control variate in the mixture.
    /// If false, the combined PDF over all lights is used as a single control variate.
    /// </summary>
    public bool UsePerLightCV = false;

    /// <summary>
    /// If true, heeds the prefix weights when optimizing the local CV coefficients.
    /// </summary>
    public bool UsePrefixAwareCV = false;

    /// <summary>
    /// If true, each lobe of the guiding mixture is used as a control variate.
    /// If false, the full guiding PDF defines a single control variate.
    /// </summary>
    public bool UseMisCompensation = false;

    /// <summary>
    /// If true, divides each sample weight by a surrogate reference value of the corresponding pixel.
    /// </summary>
    public bool UseRelative = false;

    /// <summary>
    /// Specify num of grid resolution. If not specify, use adaptive grid.
    /// </summary>
    public int MaxNumVoxels = 0;

    public bool RenderPathLengths = false;

    #endregion PARAMETERS

    SparseGrid<SparseCVOptimizer> cvOptimizer;
    bool cvReady;
    LightClusters clusters;
    int numGuidingLobes = 0;
    int numBsdfLobes = 1;

    RgbImage denoisedImage;

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

    List<RgbLayer> pathDepthImages;
    RgbLayer cvCoeffs;

    class PathStateCV : PathState
    {
        public RgbColor PreviousBsdfCos { get; set; }
        public RgbColor PreviousThroughput { get; set; }
        public float PreviousBsdfPdf { get; set; }
    }

    protected override PathState MakePathState() => new PathStateCV()
    {
    };

    protected override void OnPrepareRender()
    {
        // Bound the number of lights in the scene through semi-random clustering, just to keep the
        // overhead in check. Becomes redundant if proper clustering (e.g., lightcuts) is used.
        clusters = new(scene, 5000);

        // Evaluate the grid size heuristic to adapt the resolution to the scene
        int maxNumVoxels = MaxNumVoxels == 0 ? AdaptGridResolution() : MaxNumVoxels;
        Logger.Log($"Grid resolution: {maxNumVoxels}");
        Logger.Log($"Number of clusters: {clusters.NumClusters}");
        Logger.Log($"Number of emissive triangles: {scene.Emitters.Count}");

        if (scene.Background is EnvironmentMap)
            scene.Background = new EnvironmentMap((scene.Background as EnvironmentMap).Image, UseMisCompensation);

        numBsdfLobes = 1;

        // Use per light cv or merge into single CV
        int numNeeTechs = UsePerLightCV ? clusters.NumClusters : 1;
        if (scene.Emitters.Count == 0)
            numNeeTechs = 0;
        if (scene.Background != null) // Account for env map tobe the additional cluster
            numNeeTechs += 1;
        if(NumShadowRays==0)
            numNeeTechs = 0;

        // Bsdf sampling used.
        int numTechOthers = numBsdfLobes;

        cvOptimizer = new(scene, () => new SparseCVOptimizer(numNeeTechs, numTechOthers, UsePrefixAwareCV), maxNumVoxels);
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

        cvCoeffs = new();
        scene.FrameBuffer.AddLayer($"cv", cvCoeffs);
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
        //scene.FrameBuffer.Reset();
    }

    protected override void PostprocessIteration(uint iterIdx)
    {
        // Only update the denoised ground truth once (it's expensive)
        if (iterIdx == 1 && UseRelative)
        {
            denoiseBuffers.Denoise();
            denoisedImage = scene.FrameBuffer.GetLayer("denoised").Image as RgbImage;

            // Clear all estimates because we now compute a different solution
            cvOptimizer.ParallelForAllCells(cvOpt =>
            {
                cvOpt.Clear();
            });
        }
    }

    protected override (RgbColor, float) OnBackgroundHit(in Ray ray, PathState state)
    {
        if (scene.Background == null && !EnableCV)
            return (RgbColor.Black, 0);

        float misWeight = 1.0f;
        float pdfNextEvent = 0;
        var emission = RgbColor.Black;
        if (scene.Background != null)
        {
            emission = scene.Background.EmittedRadiance(ray.Direction);
            pdfNextEvent = scene.Background.DirectionPdf(ray.Direction) * NumShadowRays;
        }

        if (state.Depth > 1)
        {
            // Compute the balance heuristic MIS weight

            misWeight = 1 / (1 + pdfNextEvent / state.PreviousPdf);
            var estimate = misWeight * emission * (state as PathStateCV).PreviousBsdfCos / state.PreviousPdf;

            if (!EnableCV)
                return (estimate, pdfNextEvent);

            // Update the CV estimates
            var cvOpt = cvOptimizer.Query(state.PreviousHit.Value.Position);

            float bsdfPdf = (state as PathStateCV).PreviousBsdfPdf;

            Span<float> otherPdfs = stackalloc float[cvOpt.numOther];

            // Bsdf sampling
            otherPdfs[0] = state.PreviousPdf;

            float effectivePdf = pdfNextEvent + state.PreviousPdf;

            //TODO: Prevent divide by zero in pure nee with no background case
            //if (estimate != RgbColor.Black && pdfNextEvent != 0) //

            // Index of envmap is the last one of all the clusters
            int envIdx = NumShadowRays == 0 ? 0: cvOpt.numPartitioned - 1;
            RgbColor throughput = (state as PathStateCV).PreviousThroughput;

            cvOpt.Update(estimate, pdfNextEvent, envIdx, otherPdfs, effectivePdf,
                throughput / (0.01f * RgbColor.White + denoisedImage?[(int)state.Pixel.Col, (int)state.Pixel.Row] ?? RgbColor.White));

            // Evaluate the CV
            RgbColor cv = RgbColor.Black;

            if (cvReady)
            {
                cv += cvOpt.Evaluate(pdfNextEvent, envIdx, otherPdfs) / effectivePdf;
            }

            Debug.Assert(float.IsFinite(cv.Average));
            OnHitLightResult(ray, state, misWeight, emission, true);

            return (estimate + cv, pdfNextEvent);
        }

        RegisterSample(state.Pixel, emission * state.Throughput, misWeight, state.Depth, false);
        OnHitLightResult(ray, state, misWeight, emission, true);
        return (misWeight * emission, pdfNextEvent);
    }

    protected override RgbColor PerformBackgroundNextEvent(in Ray ray, in SurfacePoint hit, PathState state)
    {
        if (scene.Background == null)
            return RgbColor.Black; // There is no background

        var sample = scene.Background.SampleDirection(state.Rng.NextFloat2D());
        bool IsLeftScene = scene.Raytracer.LeavesScene(hit, sample.Direction);

        Span<float> guidePdfs = stackalloc float[numGuidingLobes];
        Span<float> guideLobeWeighs = stackalloc float[numGuidingLobes];
        float pdfBsdf = 0;

        // Prevent NaN / Inf
        if (sample.Pdf == 0) return RgbColor.Black;

        RgbColor estimate = RgbColor.Black;
        if (IsLeftScene)
        {
            pdfBsdf = DirectionPdf(hit, -ray.Direction, sample.Direction, state);
            float misWeight = EnableBsdfDI ? 1 / (1.0f + pdfBsdf / (sample.Pdf * NumShadowRays)) : 1;
            var bsdfTimesCosine = hit.Material.EvaluateWithCosine(hit, -ray.Direction, sample.Direction, false);
            var contrib = sample.Weight * bsdfTimesCosine / NumShadowRays;

            Debug.Assert(float.IsFinite(contrib.Average));
            Debug.Assert(float.IsFinite(misWeight));

            RegisterSample(state.Pixel, contrib * state.Throughput, misWeight, state.Depth + 1, true);
            OnNextEventResult(ray, hit, state, misWeight, contrib);
            estimate = misWeight * contrib;
        }

        if (!EnableCV)
            return estimate;

        // Update the CV estimates
        var cvOpt = cvOptimizer.Query(hit.Position);

        Span<float> otherPdfs = stackalloc float[cvOpt.numOther];

        // Bsdf samping
        otherPdfs[0] = pdfBsdf;

        float effectivePdf = sample.Pdf * NumShadowRays + pdfBsdf;

        // Index of envmap is the last one in the cluster on its own
        cvOpt.Update(estimate, sample.Pdf * NumShadowRays, cvOpt.numPartitioned - 1, otherPdfs, effectivePdf,
            state.Throughput / (0.01f * RgbColor.White + denoisedImage?[(int)state.Pixel.Col, (int)state.Pixel.Row] ?? RgbColor.White));

        // Evaluate the CV
        RgbColor cv = RgbColor.Black;

        if (cvReady)
        {
            cv += cvOpt.Evaluate(sample.Pdf, cvOpt.numPartitioned - 1, otherPdfs) / effectivePdf;
        }
        Debug.Assert(float.IsFinite(cv.Average));

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

        // Direction sampling PDFs are initially zero and only computed if the shadow ray is not occluded
        // (direction sampling cannot sample occluded points)
        Span<float> pdfGuideSolidAngle = stackalloc float[numGuidingLobes];
        Span<float> guideLobeWeights = stackalloc float[numGuidingLobes];
        float pdfBsdfSolidAngle = 0;
        float jacobian = 0;
        float pdfBsdf = 0;

        // Evaluate balance heuristic weighted MC estimator
        RgbColor estimate = RgbColor.Black;
        if (!scene.Raytracer.IsOccluded(hit, lightSample.Point)) {
            Vector3 lightToSurface = Vector3.Normalize(hit.Position - lightSample.Point.Position);
            var emission = light.EmittedRadiance(lightSample.Point, lightToSurface);
            var bsdfCos = hit.Material.EvaluateWithCosine(hit, -ray.Direction, -lightToSurface, false);

            jacobian = SampleWarp.SurfaceAreaToSolidAngle(hit, lightSample.Point);
            pdfBsdfSolidAngle = DirectionPdf(hit, -ray.Direction, -lightToSurface, state);
            pdfBsdf = pdfBsdfSolidAngle * jacobian;

            float misWeight = EnableBsdfDI ? 1.0f / (pdfBsdf / pdfNextEvt + 1) : 1;
            estimate = misWeight * emission * jacobian * bsdfCos / pdfNextEvt;

/*            Debug.Assert(pdfBsdf != 0 || bsdfCos == RgbColor.Black,
                "Non-zero BSDF value not sampled by forward path tracing!");*/

            OnNextEventResult(ray, hit, state, misWeight, estimate);
        }

        if (!EnableCV)
            return estimate;

        // Update the CV estimates
        var cvOpt = cvOptimizer.Query(hit.Position);

        Span<float> otherPdfs = stackalloc float[cvOpt.numOther];

        // Bsdf sampling

        otherPdfs[0] = pdfBsdfSolidAngle * jacobian;


        float cvFunc = pdfNextEvt;
        int sampleIdx = 0;

        if (UsePerLightCV)
        {
            int clusterIdx = clusters.GetClusterIdx(idx);
            int clusterSize = clusters.GetNumLightsInCluster(clusterIdx);
            float clusterSelectProb = clusterSize / (float)scene.Emitters.Count; // assumes uniform selection

            // Account for the fact that the CV is a mixture of all lights in the cluster.
            cvFunc = lightSample.Pdf / clusterSize;
            sampleIdx = clusterIdx;
        }

        float effectivePdf = pdfNextEvt + pdfBsdf;

        cvOpt.Update(estimate, cvFunc, sampleIdx, otherPdfs, effectivePdf,
            state.Throughput / (0.01f * RgbColor.White + denoisedImage?[(int)state.Pixel.Col, (int)state.Pixel.Row] ?? RgbColor.White));

        // Evaluate the CV
        RgbColor cv = RgbColor.Black;
        if (cvReady)
        {
            cv += cvOpt.Evaluate(cvFunc, sampleIdx, otherPdfs) / effectivePdf;
        }
        Debug.Assert(float.IsFinite(cv.Average));

        return estimate + cv;
    }

    protected override (RgbColor, float) OnLightHit(in Ray ray, in SurfacePoint hit, PathState state, Emitter light)
    {
        float misWeight = 1.0f;
        float pdfNextEvt = 0;
        var emission = light.EmittedRadiance(hit, -ray.Direction);

        if (state.Depth > 1)
        { // directly visible emitters are not explicitely connected
          // Compute the solid angle pdf of next event
            var jacobian = SampleWarp.SurfaceAreaToSolidAngle(state.PreviousHit.Value, hit);
            pdfNextEvt = light.PdfUniformArea(hit) / scene.Emitters.Count * NumShadowRays / jacobian;

            // Compute balance heuristic MIS weights
            float pdfRatio = pdfNextEvt / state.PreviousPdf;
            misWeight = EnableBsdfDI ? 1 / (pdfRatio + 1) : 0;
            RgbColor estimate = misWeight * emission * (state as PathStateCV).PreviousBsdfCos / state.PreviousPdf;
            if (!EnableCV)
                return (estimate, pdfNextEvt);

            // Update the CV estimates
            var cvOpt = cvOptimizer.Query(state.PreviousHit.Value.Position);

            Span<float> otherPdfs = stackalloc float[cvOpt.numOther];

            // Bsdf sampling
            {
                otherPdfs[0] = state.PreviousPdf;
            }

            float cvFunc = pdfNextEvt;
            float sampleCount = NumShadowRays;
            int idx = 0;
            float effectivePdf = pdfNextEvt + state.PreviousPdf;

            if (UsePerLightCV)
            {
                int clusterIdx = clusters.GetClusterIdx(light);
                int clusterSize = clusters.GetNumLightsInCluster(clusterIdx);
                float clusterSelectProb = clusterSize / (float)scene.Emitters.Count; // assumes uniform selection
                // Account for the fact that the CV is a mixture of all lights in the cluster.
                cvFunc = light.PdfUniformArea(hit) / jacobian / clusterSize;
                sampleCount *= clusterSelectProb;
                idx = clusterIdx;
            }

            RgbColor throughput = (state as PathStateCV).PreviousThroughput;
            cvOpt.Update(estimate, cvFunc, idx, otherPdfs, effectivePdf,
                throughput / (0.01f * RgbColor.White + denoisedImage?[(int)state.Pixel.Col, (int)state.Pixel.Row] ?? RgbColor.White));

            // Evaluate the CV
            RgbColor cv = RgbColor.Black;

            if (cvReady)
            {
                cv += cvOpt.Evaluate(cvFunc, idx, otherPdfs) / effectivePdf;
            }
            Debug.Assert(float.IsFinite(cv.Average));

            OnHitLightResult(ray, state, misWeight, emission, false);

            return (estimate + cv, pdfNextEvt);
        }

        RegisterSample(state.Pixel, emission * state.Throughput, misWeight, state.Depth, false);
        OnHitLightResult(ray, state, misWeight, emission, false);
        return (misWeight * emission, pdfNextEvt);
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
        if (light != null && state.Depth >= MinDepth)
        {
            (directHitContrib, nextEventPdf) = OnLightHit(ray, hit, state, light);
        }
        else if (state.Depth > 1) // Hit something but not light
        {
            // In this sample, we can only sample bsdf, thus nee samples have zero pdfs
            var cvOpt = cvOptimizer.Query(state.PreviousHit.Value.Position);

            Span<float> otherPdfs = stackalloc float[cvOpt.numOther];

            otherPdfs[0] = state.PreviousPdf;
            

            float effectivePdf = state.PreviousPdf;
            RgbColor throughput = (state as PathStateCV).PreviousThroughput;

            cvOpt.Update(RgbColor.Black, 0, 0, otherPdfs, effectivePdf,
            throughput / (0.01f * RgbColor.White + denoisedImage?[(int)state.Pixel.Col, (int)state.Pixel.Row] ?? RgbColor.White));

            // Evaluate the CV
            RgbColor cv = RgbColor.Black;
            if (cvReady)
            {
                cv += cvOpt.Evaluate(0, 0, otherPdfs) / effectivePdf;
            }
            Debug.Assert(float.IsFinite(cv.Average));

            directHitContrib += cv;
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
        (var bsdfRay, float pdf, var bsdfSampleWeight) =
            SampleDirection(ray, hit, state);


        // If the CV is not yet enabled, we can still terminate on zero-valued samples
        bool earlyExit = bsdfSampleWeight == RgbColor.Black && (!cvReady);

        if (pdf == 0 || earlyExit)
            return new RadianceEstimate {
                Emitted = directHitContrib,
                NextEventPdf = nextEventPdf,
                Reflected = nextEventContrib
            };

        // Recursively estimate the incident radiance and log the result
        var curThroughput = state.Throughput;
        int depth = (int)state.Depth;
        (state as PathStateCV).PreviousBsdfCos = bsdfSampleWeight * pdf;
        (state as PathStateCV).PreviousThroughput = state.Throughput;
        (state as PathStateCV).PreviousBsdfPdf = pdf;
        state.Throughput *= bsdfSampleWeight;
        state.Depth += 1;
        state.PreviousHit = hit;
        state.PreviousPdf = pdf;
        var nested = EstimateIncidentRadiance(bsdfRay, state);

        var reflected = nextEventContrib + nested.Reflected * bsdfSampleWeight + nested.Emitted;
        var sumCoeff = RgbColor.Black;

        if (cvReady)
        {
            var cvOpt = cvOptimizer.Query(hit.Position);
            sumCoeff = cvOpt.GetSumCoefficients(cvOpt.numOther);

            cvCoeffs.Splat(state.Pixel.Col, state.Pixel.Row, sumCoeff);
        }

        if (RenderPathLengths)
        {
            var contrib = curThroughput * (nextEventContrib + nested.Emitted + sumCoeff);
            pathDepthImages[depth - 1].Splat(state.Pixel.Col, state.Pixel.Row, contrib);
        }

        return new RadianceEstimate {
            Emitted = directHitContrib,
            NextEventPdf = nextEventPdf,
            Reflected = reflected + sumCoeff
        };
    }
}