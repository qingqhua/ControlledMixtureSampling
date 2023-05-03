using OptimalCV;
using System.Runtime.CompilerServices;

class Program
{
    static void RunEqualSppExperiment()
    {

        List<SceneConfig> scenes = new();
        scenes.Add(SceneRegistry.LoadScene("VeachMIS", maxDepth: 5));

        scenes.Add(SceneRegistry.LoadScene("DiningRoom2", maxDepth: 5));
        scenes.Add(SceneRegistry.LoadScene("RGBSofa", maxDepth: 5));
        scenes.Add(SceneRegistry.LoadScene("ModernHall", maxDepth: 5));
        scenes.Add(SceneRegistry.LoadScene("VeachMIS", maxDepth: 5));
        scenes.Add(SceneRegistry.LoadScene("Bathroom", maxDepth: 5));

        Benchmark benchmark = new(new PathTracerExperiment()
        {
            numSamples = 128,
            numCVTrainSamples = 16,
            maxTime = int.MaxValue,
        }, scenes,
        $"Results/PT/EqualSpp/", 640, 480, FrameBuffer.Flags.SendToTev);
        benchmark.Run(skipReference: false);
    }
    static void RunEqualTimeExperiment()
    {

        List<SceneConfig> scenes = new();
        scenes.Add(SceneRegistry.LoadScene("VeachMIS", maxDepth: 5));

        scenes.Add(SceneRegistry.LoadScene("DiningRoom2", maxDepth: 5));
        scenes.Add(SceneRegistry.LoadScene("RGBSofa", maxDepth: 5));
        scenes.Add(SceneRegistry.LoadScene("ModernHall", maxDepth: 5));
        scenes.Add(SceneRegistry.LoadScene("VeachMIS", maxDepth: 5));
        scenes.Add(SceneRegistry.LoadScene("Bathroom", maxDepth: 5));

        Benchmark benchmark = new(new PathTracerExperiment()
        {
            numSamples = int.MaxValue,
            numCVTrainSamples = 16,
            maxTime = 36000,
        }, scenes,
        $"Results/PT/EqualTime/", 640, 480, FrameBuffer.Flags.SendToTev);
        benchmark.Run(skipReference: false);
    }

    static public void Main(String[] args)
    {
        // Add the scenes folder in the parent directory of this source file to the path.
        string GetThisFilePath([CallerFilePath] string path = null) => path;
        var path = Path.GetDirectoryName(GetThisFilePath());
        SceneRegistry.AddSource(Path.Join(path, "..", "Scenes"));

        RunEqualTimeExperiment();
        RunEqualSppExperiment();
    }

}


