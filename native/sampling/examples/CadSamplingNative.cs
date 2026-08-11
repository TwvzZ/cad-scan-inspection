using System;
using System.Runtime.InteropServices;

namespace CadSampling
{
    public enum Status : int
    {
        Success = 0,
        InvalidArgument = 1,
        FileReadFailed = 2,
        NoShape = 3,
        MeshFailed = 4,
        BufferTooSmall = 5,
        InternalError = 6
    }

    public enum SurfaceMode : uint
    {
        AllFaces = 0,
        OuterShell = 1,
        VisibleSurface = 2
    }

    public enum ProjectionMode : uint
    {
        Perspective = 0,
        Orthographic = 1
    }

    [StructLayout(LayoutKind.Sequential)]
    internal struct NativeOptions
    {
        public uint StructSize;
        public uint AbiVersion;
        public ulong TargetPointCount;
        public double LinearDeflection;
        public double AngularDeflectionDeg;
        public uint RelativeDeflection;
        public uint ParallelMeshing;
        public uint RandomSeed;
        public double VoxelSize;
        public SurfaceMode SurfaceMode;
        public ProjectionMode ProjectionMode;
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 3)]
        public double[] CameraPosition;
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 3)]
        public double[] ViewDirection;
        public double MaxIncidenceAngleDeg;
        public double VisibilityTolerance;
        public uint VisibilityOversampleFactor;
    }

    [StructLayout(LayoutKind.Sequential)]
    internal struct NativeBuffers
    {
        public uint StructSize;
        public uint AbiVersion;
        public ulong CapacityPoints;
        public IntPtr Xyz;
        public IntPtr Normals;
        public IntPtr FaceIds;
    }

    [StructLayout(LayoutKind.Sequential, CharSet = CharSet.Ansi)]
    internal struct NativeResult
    {
        public uint StructSize;
        public uint AbiVersion;
        public Status Status;
        public ulong RequiredPointCapacity;
        public ulong PointCount;
        public ulong FaceCount;
        public ulong TriangleCount;
        public ulong SelectedTriangleCount;
        public double SurfaceArea;
        public double SelectedSurfaceArea;
        public double LoadElapsedMs;
        public double TriangulationElapsedMs;
        public double GenerationElapsedMs;
        public double VoxelElapsedMs;
        public double VisibilityElapsedMs;
        public double SampleElapsedMs;
        public uint MeshCacheHit;
        public uint SampleCacheHit;

        [MarshalAs(UnmanagedType.ByValTStr, SizeConst = 512)]
        public string ErrorMessage;
    }

    public sealed class SampleResult
    {
        public double[] Xyz { get; internal set; } = Array.Empty<double>();
        public double[] Normals { get; internal set; } = Array.Empty<double>();
        public uint[] FaceIds { get; internal set; } = Array.Empty<uint>();
        public ulong FaceCount { get; internal set; }
        public ulong TriangleCount { get; internal set; }
        public double SurfaceArea { get; internal set; }
        public double LoadElapsedMs { get; internal set; }
        public double TriangulationElapsedMs { get; internal set; }
        public double GenerationElapsedMs { get; internal set; }
        public double VoxelElapsedMs { get; internal set; }
        public double SampleElapsedMs { get; internal set; }
        public double VisibilityElapsedMs { get; internal set; }
        public bool MeshCacheHit { get; internal set; }
        public bool SampleCacheHit { get; internal set; }
    }

    public static class StepSampler
    {
        private const string DllName = "cad_sampling";

        [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
        private static extern IntPtr cadsample_create();

        [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
        private static extern void cadsample_destroy(IntPtr handle);

        [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
        private static extern void cadsample_default_options(
            ref NativeOptions options);

        [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
        private static extern void cadsample_init_buffers(
            ref NativeBuffers buffers);

        [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
        private static extern void cadsample_init_result(
            ref NativeResult result);

        [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
        private static extern Status cadsample_load_step(
            IntPtr handle,
            [MarshalAs(UnmanagedType.LPUTF8Str)] string path,
            ref NativeResult result);

        [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
        private static extern Status cadsample_generate(
            IntPtr handle,
            ref NativeOptions options,
            IntPtr buffers,
            ref NativeResult result);

        [DllImport(DllName, CallingConvention = CallingConvention.Cdecl,
            EntryPoint = "cadsample_generate")]
        private static extern Status cadsample_generate_with_buffers(
            IntPtr handle,
            ref NativeOptions options,
            ref NativeBuffers buffers,
            ref NativeResult result);

        public static SampleResult Sample(
            string stepPath,
            ulong pointCount = 100000,
            double linearDeflection = 0.1,
            double angularDeflectionDeg = 10.0,
            uint randomSeed = 1,
            double voxelSize = 0.0,
            SurfaceMode surfaceMode = SurfaceMode.OuterShell,
            double cameraX = 0.0,
            double cameraY = 0.0,
            double cameraZ = 0.0,
            ProjectionMode projectionMode = ProjectionMode.Orthographic,
            double viewX = 0.0,
            double viewY = 0.0,
            double viewZ = -1.0,
            double maxIncidenceAngleDeg = 75.0)
        {
            if (string.IsNullOrWhiteSpace(stepPath))
                throw new ArgumentException("STEP path is required.", nameof(stepPath));
            if (pointCount == 0 || pointCount > int.MaxValue / 3)
                throw new ArgumentOutOfRangeException(nameof(pointCount));

            IntPtr handle = cadsample_create();
            if (handle == IntPtr.Zero)
                throw new InvalidOperationException("Could not create sampler.");
            try
            {
                var nativeResult = new NativeResult();
                cadsample_init_result(ref nativeResult);
                Status status = cadsample_load_step(
                    handle, stepPath, ref nativeResult);
                ThrowOnFailure(status, nativeResult);
                double loadElapsed = nativeResult.LoadElapsedMs;

                var options = new NativeOptions {
                    CameraPosition = new double[3],
                    ViewDirection = new double[3]
                };
                cadsample_default_options(ref options);
                options.TargetPointCount = pointCount;
                options.LinearDeflection = linearDeflection;
                options.AngularDeflectionDeg = angularDeflectionDeg;
                options.RandomSeed = randomSeed;
                options.VoxelSize = voxelSize;
                options.SurfaceMode = surfaceMode;
                options.ProjectionMode = projectionMode;
                options.CameraPosition = new[] { cameraX, cameraY, cameraZ };
                options.ViewDirection = new[] { viewX, viewY, viewZ };
                options.MaxIncidenceAngleDeg = maxIncidenceAngleDeg;

                status = cadsample_generate(
                    handle, ref options, IntPtr.Zero, ref nativeResult);
                if (status != Status.BufferTooSmall)
                    ThrowOnFailure(status, nativeResult);
                double triangulationElapsed =
                    nativeResult.TriangulationElapsedMs;
                double generationElapsed = nativeResult.GenerationElapsedMs;
                double voxelElapsed = nativeResult.VoxelElapsedMs;
                double queryElapsed = nativeResult.SampleElapsedMs;
                bool queryMeshCacheHit = nativeResult.MeshCacheHit != 0;
                bool querySampleCacheHit = nativeResult.SampleCacheHit != 0;

                int count = checked((int)nativeResult.RequiredPointCapacity);
                var xyz = new double[checked(count * 3)];
                var normals = new double[checked(count * 3)];
                var faceIds = new uint[count];
                GCHandle xyzPin = default;
                GCHandle normalPin = default;
                GCHandle facePin = default;
                try
                {
                    xyzPin = GCHandle.Alloc(xyz, GCHandleType.Pinned);
                    normalPin = GCHandle.Alloc(normals, GCHandleType.Pinned);
                    facePin = GCHandle.Alloc(faceIds, GCHandleType.Pinned);
                    var buffers = new NativeBuffers();
                    cadsample_init_buffers(ref buffers);
                    buffers.CapacityPoints = (ulong)count;
                    buffers.Xyz = xyzPin.AddrOfPinnedObject();
                    buffers.Normals = normalPin.AddrOfPinnedObject();
                    buffers.FaceIds = facePin.AddrOfPinnedObject();
                    status = cadsample_generate_with_buffers(
                        handle, ref options, ref buffers, ref nativeResult);
                    ThrowOnFailure(status, nativeResult);
                }
                finally
                {
                    if (facePin.IsAllocated) facePin.Free();
                    if (normalPin.IsAllocated) normalPin.Free();
                    if (xyzPin.IsAllocated) xyzPin.Free();
                }

                return new SampleResult {
                    Xyz = xyz,
                    Normals = normals,
                    FaceIds = faceIds,
                    FaceCount = nativeResult.FaceCount,
                    TriangleCount = nativeResult.TriangleCount,
                    SurfaceArea = nativeResult.SurfaceArea,
                    LoadElapsedMs = loadElapsed,
                    TriangulationElapsedMs = triangulationElapsed,
                    GenerationElapsedMs = generationElapsed,
                    VoxelElapsedMs = voxelElapsed,
                    VisibilityElapsedMs =
                        nativeResult.VisibilityElapsedMs,
                    SampleElapsedMs =
                        queryElapsed + nativeResult.SampleElapsedMs,
                    MeshCacheHit = queryMeshCacheHit,
                    SampleCacheHit = querySampleCacheHit
                };
            }
            finally
            {
                cadsample_destroy(handle);
            }
        }

        private static void ThrowOnFailure(Status status, NativeResult result)
        {
            if (status != Status.Success)
                throw new InvalidOperationException(
                    $"CAD sampling failed ({status}): {result.ErrorMessage}");
        }
    }
}
