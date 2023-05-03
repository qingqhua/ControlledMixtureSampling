namespace OptimalCV;

/// <summary>
/// A regular hash grid spanning the full scene. Cells are initialized sparsely and on the fly, using atomics.
/// </summary>
public class SparseGrid<T> where T : class {
    Scene scene;
    int numX, numY, numZ;
    T[] voxels;
    InitializeCell initFn;
    Vector3 invDiagonal;
    Vector3 boundsMin;

    public delegate T InitializeCell();

    public SparseGrid(Scene scene, InitializeCell initFn, int maxVoxels = 64) {
        this.scene = scene;
        this.initFn = initFn;

        // Compute the size of the scene along the widest axis
        float bmax = 0.0f;
        var max = scene.Bounds.Diagonal;
        if (max.X > max.Y && max.X > max.Z) bmax = max.X;
        else if (max.Y > max.Z) bmax = max.Y;
        else bmax = max.Z;

        var diag = scene.Bounds.Diagonal;
        numX = (int)Math.Max(1, Math.Round(diag.X / bmax * maxVoxels));
        numY = (int)Math.Max(1, Math.Round(diag.Y / bmax * maxVoxels));
        numZ = (int)Math.Max(1, Math.Round(diag.Z / bmax * maxVoxels));

        voxels = new T[numX * numY * numZ];

        // Cache frequently used values
        invDiagonal = Vector3.One / scene.Bounds.Diagonal;
        boundsMin = scene.Bounds.Min;
    }

    /// <summary>
    /// Retrieves the voxel at the given position. If it is not initialized yet, the voxel is created using
    /// the callback passed to the constructor.
    /// Thread-safe and lock-free.
    /// </summary>
    /// <param name="pos">Position in the scene</param>
    /// <returns>The voxel that contains the position</returns>
    public T Query(Vector3 pos) {
        var relPos = (pos - boundsMin) * invDiagonal;

        int x = Math.Clamp((int)(relPos.X * numX), 0, numX - 1);
        int y = Math.Clamp((int)(relPos.Y * numY), 0, numY - 1);
        int z = Math.Clamp((int)(relPos.Z * numZ), 0, numZ - 1);
        
        int idx = z * numY * numX + y * numX + x;

        if (voxels[idx] == null)
        {
            // Create a new distribution and try to write it. If another thread was quicker, instead use
            // the one written by that thread.
            T obj = initFn();
            Interlocked.CompareExchange<T>(ref voxels[idx], obj, null);
        }

        return voxels[idx];
    }

    /// <summary>
    /// Invokes a function on each initialized cell in the grid.
    /// </summary>
    /// <param name="fn">The function to invoke</param>
    public void ForAllCells(Action<T> fn) {
        foreach (var cell in voxels)
            if (cell != null) fn(cell);
    }

    /// <summary>
    /// Invokes a function on each initialized cell in the grid in parallel.
    /// </summary>
    /// <param name="fn">The function to invoke</param>
    public void ParallelForAllCells(Action<T> fn) {
        Parallel.ForEach(voxels, cell => {
            if (cell != null) fn(cell);
        });
    }
}