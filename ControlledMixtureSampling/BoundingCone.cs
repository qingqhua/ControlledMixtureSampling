namespace OptimalCV;

record struct BoundingCone(Vector3 Center, float HalfAngle = 0.0f) {
    public static BoundingCone Merge(BoundingCone a, BoundingCone b) {
        // [Han et al. 2021: Tight Normal Cone Merging...]
        // Early exit if the cone is already a full sphere
        if (a.HalfAngle > Math.PI) return a;
        if (b.HalfAngle > Math.PI) return b;

        // Compute the cross section angles
        float phi11 = -a.HalfAngle;
        float phi12 = a.HalfAngle;

        float bProj = MathF.Acos(Vector3.Dot(a.Center, b.Center));
        float phi21 = bProj - b.HalfAngle;
        float phi22 = bProj + b.HalfAngle;

        // Check if b inside a
        if (phi21 < phi12 && phi22 < phi12 && phi21 > phi11 && phi22 > phi11)
            return a;
        // or a inside b
        if (phi11 < phi22 && phi12 < phi22 && phi11 > phi21 && phi12 > phi21)
            return b;

        float alpha = 0.5f * (Math.Max(phi22, phi12) + Math.Min(phi21, phi11));
        float theta = 0.5f * (Math.Max(phi22, phi12) - Math.Min(phi21, phi11));

        if (theta > MathF.PI) return new(a.Center, MathF.PI);
        if (!float.IsFinite(theta)) return new(a.Center, MathF.PI);
        if (theta == 0) return a;

        var q = Quaternion.Slerp(new(a.Center, 0.0f), new(b.Center, 0.0f), alpha / bProj);
        return new BoundingCone(new Vector3(q.X, q.Y, q.Z), theta);
    }

    public BoundingCone GrowToContain(BoundingCone other) => Merge(this, other);
}
