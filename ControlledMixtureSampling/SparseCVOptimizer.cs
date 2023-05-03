//#define NAIVESOLVER
#define ATOMICUPDATE
using M = MathNet.Numerics.LinearAlgebra.Matrix<float>;
using V = MathNet.Numerics.LinearAlgebra.Vector<float>;

namespace OptimalCV;

/// <summary>
/// Optimal CV computations for a huge set of sparse techniques (e.g., per-light CVs)
/// Separates techniques into those that are mutually exclusive / non overlapping (i.e., their PDFs from a
/// partitioning of the domain), and those that are not.
///
/// TODO assumes equal sample count / selection weight for everything
/// </summary>
public class SparseCVOptimizer {
    float[] partitionedDiagonal;
    float[][] partitionedDotOther;
    // float[][] otherDotOther;
    float[] otherDotOther;

    float[] contributionsRed;
    float[] contributionsGreen;
    float[] contributionsBlue;

    public int numPartitioned { get; init; }
    public int numOther { get; init; }
    int numTechs;

    float[] coeffsR, coeffsG, coeffsB;
    int numSamples;
    int numNonZeroSamples;
    bool heedPrefix;
    int remainingUpdates;


    RgbColor sumCoeffsPartitioned;

    /// <summary>
    ///
    /// </summary>
    /// <param name="numPartitioned">
    /// Number of techniques with mutually non-overlapping PDFs. These don't overlap each other, but can
    /// still overlap other techniques in the full set.
    /// </param>
    /// <param name="numOther">
    /// Number of techniques with (potentially) overlapping PDFs
    /// </param>
    /// <param name="heedPrefix">
    /// If false, ignores the prefix weight
    /// </param>
    public SparseCVOptimizer(int numPartitioned, int numOther, bool heedPrefix, int maxNumUpdates = 16) {
        partitionedDiagonal = new float[numPartitioned];
        partitionedDotOther = new float[numPartitioned][];
        for (int i = 0; i < numPartitioned; ++i) {
            partitionedDotOther[i] = new float[numOther];
        }
        otherDotOther = new float[(numOther * (numOther + 1)) / 2];

        contributionsRed = new float[numPartitioned + numOther];
        contributionsGreen = new float[numPartitioned + numOther];
        contributionsBlue = new float[numPartitioned + numOther];

        this.numOther = numOther;
        this.numPartitioned = numPartitioned;
        numTechs = numOther + numPartitioned;

        this.heedPrefix = heedPrefix;
        remainingUpdates = maxNumUpdates;
    }

    public void Clear() {
        Array.Clear(partitionedDiagonal);
        for (int i = 0; i < numPartitioned; ++i)
            Array.Clear(partitionedDotOther[i]);
        Array.Clear(otherDotOther);
        Array.Clear(contributionsRed);
        Array.Clear(contributionsGreen);
        Array.Clear(contributionsBlue);
    }

    /// <summary>
    /// Updates the estimates given a new sample
    /// </summary>
    /// <param name="balanceEstimate">The balance heuristic weighted estimate of the current technique</param>
    /// <param name="partPdf">The value of the only non-zero partitioned PDF</param>
    /// <param name="partIdx">The 0-based index of the only non-zero partitioned PDF</param>
    /// <param name="otherPdfs">The values of all other non-partitioned PDFs</param>
    /// <param name="samplingPdf">Effective PDF (mixture / balance) used to compute the CV</param>
    /// <param name="throughputWeight">Throughput estimate (g(x) / p(x)) of the prefix path.</param>
    public void Update(RgbColor balanceEstimate, float partPdf, int partIdx, ReadOnlySpan<float> otherPdfs,
                       float samplingPdf, RgbColor throughputWeight)
    {
        if (remainingUpdates <= 0)
            return;

        float sumPdfs = samplingPdf; // Assumes that we use the same technique for sampling at all times

        // for simplicity, we consider only the average in the matrix
        // TODO test with one matrix for each color band
        //      not even sure if we should square and average or average first and then square...
        float prefixWeight = 1.0f;
        if (heedPrefix)
        {
            balanceEstimate *= (throughputWeight * throughputWeight).Average;
            prefixWeight = (throughputWeight * throughputWeight).Average;
        }

        Debug.Assert(float.IsFinite(prefixWeight));

#if ATOMICUPDATE
        float invSumPdf = 1.0f / sumPdfs;
        var e = balanceEstimate * (partPdf * invSumPdf);
        Atomic.AddFloat(ref contributionsRed[partIdx], e.R);
        Atomic.AddFloat(ref contributionsGreen[partIdx], e.G);
        Atomic.AddFloat(ref contributionsBlue[partIdx], e.B);
        for (int i = numPartitioned; i < numTechs; ++i) {
            e = balanceEstimate * (otherPdfs[i - numPartitioned] * invSumPdf);
            Atomic.AddFloat(ref contributionsRed[i], e.R);
            Atomic.AddFloat(ref contributionsGreen[i], e.G);
            Atomic.AddFloat(ref contributionsBlue[i], e.B);
        }

        int idx = 0;
        float r = prefixWeight * invSumPdf / samplingPdf;
        for (int i = 0; i < numOther; ++i)
        {
            for (int j = i; j < numOther; ++j)
            {
                Atomic.AddFloat(ref otherDotOther[idx++], otherPdfs[j] * otherPdfs[i] * r);
            }
        }

        Interlocked.Increment(ref numSamples);
        if (balanceEstimate.Average > 0)
            Interlocked.Increment(ref numNonZeroSamples);

        if (numPartitioned == 0)
            return;

        r = prefixWeight * partPdf * invSumPdf / samplingPdf;
        Atomic.AddFloat(ref partitionedDiagonal[partIdx], partPdf * r);
        for (int i = 0; i < numOther; ++i)
        {
            Atomic.AddFloat(ref partitionedDotOther[partIdx][i], otherPdfs[i] * r);
        }
#else
        lock (this)
        {
            float invSumPdf = 1.0f / sumPdfs;

            contributionsRed[partIdx] += balanceEstimate.R * partPdf * invSumPdf;
            contributionsGreen[partIdx] += balanceEstimate.G * partPdf * invSumPdf;
            contributionsBlue[partIdx] += balanceEstimate.B * partPdf * invSumPdf;
            for (int i = numPartitioned; i < numTechs; ++i)
            {
                float ratio = otherPdfs[i - numPartitioned] * invSumPdf;
                contributionsRed[i] += balanceEstimate.R * ratio;
                contributionsGreen[i] += balanceEstimate.G * ratio;
                contributionsBlue[i] += balanceEstimate.B * ratio;
            }

            int idx = 0;
            for (int i = 0; i < numOther; ++i)
            {
                for (int j = i; j < numOther; ++j)
                {
                    otherDotOther[idx++] += prefixWeight * otherPdfs[j] * otherPdfs[i] * invSumPdf / samplingPdf;
                }
            }

            numSamples++;

            if (balanceEstimate.Average > 0)
                numNonZeroSamples++;

            // If the sample balance is much difference than average balance in this optimizer, we will not solve this optimizer.

            if (numPartitioned == 0)
                return;

            partitionedDiagonal[partIdx] += prefixWeight * partPdf * partPdf * invSumPdf / samplingPdf;
            for (int i = 0; i < numOther; ++i)
            {
                partitionedDotOther[partIdx][i] += prefixWeight * partPdf * otherPdfs[i] * invSumPdf / samplingPdf;
            }
        }
#endif
    }

    /// <summary>
    /// Solves the linear system to compute the optimal coefficients for the CV.
    /// </summary>
    public void Solve() {
        // Analytic solution if there are only partitioned techniques

        // if (numSamples < 10) return;
        // coeffsR = new float[numPartitioned];
        // coeffsG = new float[numPartitioned];
        // coeffsB = new float[numPartitioned];
        // for (int i = 0; i < numPartitioned; ++i)
        // {
        //     coeffsR[i] = partitionedDiagonal[i] == 0 ? 0 : contributionsRed[i] / partitionedDiagonal[i];
        //     coeffsG[i] = partitionedDiagonal[i] == 0 ? 0 : contributionsGreen[i] / partitionedDiagonal[i];
        //     coeffsB[i] = partitionedDiagonal[i] == 0 ? 0 : contributionsBlue[i] / partitionedDiagonal[i];
        // }
        // return;

        // TODO: test revert to balance (0 coeffs) if
        //       - too high variation in the value
        //       - any NaNs in the result (matrix not invertible)

        //if (numNonZeroSamples < 100) return; // TODO configure / test

        // Naive full-system solver
#if NAIVESOLVER


        // Populate the matrix
        M techCovar = M.Build.Sparse(numTechs, numTechs);
         for (int i = 0; i < numPartitioned; ++i)
             techCovar[i, i] = partitionedDiagonal[i];
         for (int i = 0; i < numPartitioned; ++i) {
             for (int j = 0; j < numOther; ++j) {
                 techCovar[i, j + numPartitioned] = partitionedDotOther[i][j];
                 techCovar[j + numPartitioned, i] = partitionedDotOther[i][j];
             }
         }

         for (int i = 0; i < numOther; ++i) {
             for (int j = i; j < numOther; ++j) {
                 techCovar[i + numPartitioned, j + numPartitioned] = otherDotOther[i][j - i];
                 techCovar[j + numPartitioned, i + numPartitioned] = otherDotOther[i][j - i];
             }
         }

         // Inversion is expensive, so we only do it once and then multiply the result on each color vector
         var inverse = techCovar.Inverse();
         V r = V.Build.DenseOfArray(contributionsRed);
         V g = V.Build.DenseOfArray(contributionsGreen);
         V b = V.Build.DenseOfArray(contributionsBlue);
         coeffsR = inverse.Multiply(r).ToArray();
         coeffsG = inverse.Multiply(g).ToArray();
         coeffsB = inverse.Multiply(b).ToArray();

#else
        // Fancy, specialized solver

        // Compute a numOther x numOther matrix that marginalizes over the partitioned techniques
        float[][] newMatrix = new float[numOther][];
        for (int i = 0; i < numOther; ++i)
        {
            newMatrix[i] = new float[numOther];
            for (int j = 0; j < numOther; ++j)
            {
                float sumRatios = 0.0f;
                for (int k = 0; k < numPartitioned; ++k)
                    sumRatios += partitionedDotOther[k][i] * partitionedDotOther[k][j] / partitionedDiagonal[k];

                // pick the right element of the upper triagonal matrix representation
                // float other = i > j ? otherDotOther[j][i - j] : otherDotOther[i][j - i];

                int row = i;
                int col = j;
                if (row > col) (row, col) = (col, row);
                float other = otherDotOther[row * numOther - (row * (row + 1)) / 2 + col];

                newMatrix[i][j] = other - sumRatios;
            }
        }

        float[] SolveColor(float[] contribs)
        {
            float[] newVector = new float[numOther];
            for (int i = 0; i < numOther; ++i)
            {
                float sumWeights = 0.0f;
                for (int k = 0; k < numPartitioned; ++k)
                {
                    if (partitionedDiagonal[k] == 0)
                        continue;
                    sumWeights += partitionedDotOther[k][i] * contribs[k] / partitionedDiagonal[k];
                }
                newVector[i] = contribs[i + numPartitioned] - sumWeights;
            }

            V Y = null;
            if (numOther > 0)
                Y = M.Build.DenseOfRowArrays(newMatrix).Solve(V.Build.DenseOfArray(newVector));

            float[] X = new float[numPartitioned];
            for (int i = 0; i < numPartitioned; ++i)
            {
                if (partitionedDiagonal[i] == 0)
                    X[i] = 0;
                else
                {
                    float sum = 0;
                    for (int k = 0; k < numOther; ++k)
                    {
                        sum += partitionedDotOther[i][k] * Y[k];
                    }
                    X[i] = (contribs[i] - sum) / partitionedDiagonal[i];
                }
            }

            float[] result = new float[numTechs];
            for (int i = 0; i < numPartitioned; ++i)
                result[i] = X[i];
            for (int i = 0; i < numOther; ++i)
                result[i + numPartitioned] = Y[i];
            return result;
        }

        coeffsR = SolveColor(contributionsRed);
        coeffsG = SolveColor(contributionsGreen);
        coeffsB = SolveColor(contributionsBlue);
#endif

        // Precompute the sum of coefficients for the partitioned techniques.
        sumCoeffsPartitioned = RgbColor.Black;
        for (int i = 0; i < numPartitioned; ++i)
            sumCoeffsPartitioned += new RgbColor(coeffsR[i], coeffsG[i], coeffsB[i]);

        var sumCoeffsOther = RgbColor.Black;
        for (int i = numPartitioned; i < numTechs; ++i)
            sumCoeffsOther += new RgbColor(coeffsR[i], coeffsG[i], coeffsB[i]);

        if (!float.IsFinite(sumCoeffsPartitioned.Average) || !float.IsFinite(sumCoeffsOther.Average))
        {
            // revert to zero if solving failed
            coeffsR = new float[numTechs];
            coeffsG = new float[numTechs];
            coeffsB = new float[numTechs];
            sumCoeffsPartitioned = RgbColor.Black;
            sumCoeffsOther = RgbColor.Black;
            //Logger.Warning("Ignoring bad coefficients");
        }
        else
            remainingUpdates--;
    }

    public RgbColor GetSumCoefficients(int localNumOthers) {
        if (coeffsR == null)
            return RgbColor.Black;

        RgbColor sum = sumCoeffsPartitioned;

        // We manually add the other techniques here, since they may not all be present at all points
        for (int i = 0; i < localNumOthers; ++i)
            sum += new RgbColor(
                coeffsR[i + numPartitioned],
                coeffsG[i + numPartitioned],
                coeffsB[i + numPartitioned]
            );

        //Debug.Assert(float.IsFinite(sum.Average));

        return sum;
    }

    public RgbColor GetSumOtherCoefficients(int localNumOthers)
    {
        if (coeffsR == null)
            return RgbColor.Black;

        RgbColor sum = RgbColor.Black;

        // We manually add the other techniques here, since they may not all be present at all points
        for (int i = 0; i < localNumOthers; ++i)
            sum += new RgbColor(
                coeffsR[i + numPartitioned],
                coeffsG[i + numPartitioned],
                coeffsB[i + numPartitioned]
            );

        //Debug.Assert(float.IsFinite(sum.Average));

        return sum;
    }

    /// <summary>
    /// Evaluates the control variate, i.e., computes g(x)
    /// </summary>
    public RgbColor Evaluate(float partPdf, int partIdx, ReadOnlySpan<float> otherPdfs) {
        if (coeffsR == null)
            return RgbColor.Black;

        var cv = new RgbColor(
            -coeffsR[partIdx],
            -coeffsG[partIdx],
            -coeffsB[partIdx]
        ) * partPdf;

        for (int i = 0; i < otherPdfs.Length; ++i)
            cv += new RgbColor(
                -coeffsR[i + numPartitioned],
                -coeffsG[i + numPartitioned],
                -coeffsB[i + numPartitioned]
            ) * otherPdfs[i];

        //Debug.Assert(float.IsFinite(cv.Average));

        return cv;
    }
}