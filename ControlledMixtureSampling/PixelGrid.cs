namespace OptimalCV;

/// <summary>
/// A regular hash grid spanning the full scene. Cells are initialized sparsely and on the fly, using atomics.
/// </summary>
public class PixelGrid<T> where T : class {
    Scene scene;
    int numX, numY;
    T[] pixels;
    InitializeCell initFn;

    public delegate T InitializeCell();

    public PixelGrid(Scene scene, InitializeCell initFn) {
        this.scene = scene;
        this.initFn = initFn;

        numX = scene.FrameBuffer.Width;
        numY = scene.FrameBuffer.Height;

        pixels = new T[numX * numY];
    }

    int PixelToCell(Vector2 Pixel)
    {
        int pixelIdx = (int)(Math.Floor(Pixel.Y)) * scene.FrameBuffer.Width + (int)Math.Floor(Pixel.X);
        pixelIdx = Math.Clamp(pixelIdx, 0, scene.FrameBuffer.Width * scene.FrameBuffer.Height - 1);

        return pixelIdx;
    }

    /// <summary>
    /// Retrieves the voxel at the given position. If it is not initialized yet, the voxel is created using
    /// the callback passed to the constructor.
    /// Thread-safe and lock-free.
    /// </summary>
    /// <param name="pos">Position in the scene</param>
    /// <returns>The voxel that contains the position</returns>
    public T Query(Vector2 Pixel) {
        int idx = PixelToCell(Pixel);
        if (pixels[idx] == null)
        {
            // Create a new distribution and try to write it. If another thread was quicker, instead use
            // the one written by that thread.
            T obj = initFn();
            Interlocked.CompareExchange<T>(ref pixels[idx], obj, null);
        }

        return pixels[idx];
    }

    /// <summary>
    /// Invokes a function on each initialized cell in the grid.
    /// </summary>
    /// <param name="fn">The function to invoke</param>
    public void ForAllCells(Action<T> fn) {
        foreach (var cell in pixels)
            if (cell != null) fn(cell);
    }

    /// <summary>
    /// Invokes a function on each initialized cell in the grid in parallel.
    /// </summary>
    /// <param name="fn">The function to invoke</param>
    public void ParallelForAllCells(Action<T> fn) {
        Parallel.ForEach(pixels, cell => {
            if (cell != null) fn(cell);
        });
    }
}