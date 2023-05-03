using SeeSharp.Shading;
using System.Text.Json.Nodes;
using System.Text.Json;
using static System.Runtime.InteropServices.JavaScript.JSType;
using System.Buffers.Text;
using System.Runtime.Intrinsics.Arm;

namespace OptimalCV;

class PathTracerExperiment : Experiment
{
    public int numSamples = 131313131;
    public int numCVTrainSamples = 8;
    public int maxTime = 36000;
    int numShadowRays = 1;
    public int numVoxels = 0; // 0 if want to use adaptive grid
    public override List<Method> MakeMethods() => new() {
    new("Baseline", new PathTracer() {
            TotalSpp = numSamples,
            MaximumRenderTimeMs = maxTime,
            NumShadowRays = numShadowRays,
            EnableDenoiser = true,
            EnableBsdfDI = true,
        }),
        new("16/Vevoda et al.", new PerLightCVRenderer() {
            TotalSpp = numSamples,
            MaximumRenderTimeMs = maxTime,
            NumShadowRays = numShadowRays,
            EnableDenoiser = true,
            EnableBsdfDI = true,
            MaxNumVoxels = 16,

            #region CvParameters
            EnableCV = true,
            EnablePathGuiding = false,
            TrainingSamples = numCVTrainSamples,
            RenderPathLengths = false,
            #endregion CvParameters
        }),
        new("32/Vevoda et al.", new PerLightCVRenderer() {
            TotalSpp = numSamples,
            MaximumRenderTimeMs = maxTime,
            NumShadowRays = numShadowRays,
            EnableDenoiser = true,
            EnableBsdfDI = true,
            MaxNumVoxels = 32,

            #region CvParameters
            EnableCV = true,
            EnablePathGuiding = false,
            TrainingSamples = numCVTrainSamples,
            RenderPathLengths = false,
            #endregion CvParameters
        }),
        new("64/Vevoda et al.", new PerLightCVRenderer() {
            TotalSpp = numSamples,
            MaximumRenderTimeMs = maxTime,
            NumShadowRays = numShadowRays,
            EnableDenoiser = true,
            EnableBsdfDI = true,
            MaxNumVoxels = 64,

            #region CvParameters
            EnableCV = true,
            EnablePathGuiding = false,
            TrainingSamples = numCVTrainSamples,
            RenderPathLengths = false,
            #endregion CvParameters
        }),
        new("256/Vevoda et al.", new PerLightCVRenderer() {
            TotalSpp = numSamples,
            MaximumRenderTimeMs = maxTime,
            NumShadowRays = numShadowRays,
            EnableDenoiser = true,
            EnableBsdfDI = true,
            MaxNumVoxels = 256,

            #region CvParameters
            EnableCV = true,
            EnablePathGuiding = false,
            TrainingSamples = numCVTrainSamples,
            RenderPathLengths = false,
            #endregion CvParameters
        }),
        new("16/Ours", new CVRenderer() {
            TotalSpp = numSamples,
            MaximumRenderTimeMs = maxTime,
            NumShadowRays = numShadowRays,
            EnableDenoiser = true,
            EnableBsdfDI = true,
            MaxNumVoxels = 16,

            #region CvParameters
            EnableCV = true,
            UsePerLightCV = true,
            UseMisCompensation = false,
            UsePrefixAwareCV = true,
            UseRelative = true,
            TrainingSamples = numCVTrainSamples,
            RenderPathLengths = false
            #endregion CvParameters
        }),
        new("32/Ours", new CVRenderer() {
            TotalSpp = numSamples,
            MaximumRenderTimeMs = maxTime,
            NumShadowRays = numShadowRays,
            EnableDenoiser = true,
            EnableBsdfDI = true,
            MaxNumVoxels = 32,

            #region CvParameters
            EnableCV = true,
            UsePerLightCV = true,
            UseMisCompensation = false,
            UsePrefixAwareCV = true,
            UseRelative = true,
            TrainingSamples = numCVTrainSamples,
            RenderPathLengths = false
            #endregion CvParameters
        }),
        new("64/Ours", new CVRenderer() {
            TotalSpp = numSamples,
            MaximumRenderTimeMs = maxTime,
            NumShadowRays = numShadowRays,
            EnableDenoiser = true,
            EnableBsdfDI = true,
            MaxNumVoxels = 64,

            #region CvParameters
            EnableCV = true,
            UsePerLightCV = true,
            UseMisCompensation = false,
            UsePrefixAwareCV = true,
            UseRelative = true,
            TrainingSamples = numCVTrainSamples,
            RenderPathLengths = false
            #endregion CvParameters
        }),
        new("256/Ours", new CVRenderer() {
            TotalSpp = numSamples,
            MaximumRenderTimeMs = maxTime,
            NumShadowRays = numShadowRays,
            EnableDenoiser = true,
            EnableBsdfDI = true,
            MaxNumVoxels = 256,

            #region CvParameters
            EnableCV = true,
            UsePerLightCV = true,
            UseMisCompensation = false,
            UsePrefixAwareCV = true,
            UseRelative = true,
            TrainingSamples = numCVTrainSamples,
            RenderPathLengths = false
            #endregion CvParameters
        }),
    };
}