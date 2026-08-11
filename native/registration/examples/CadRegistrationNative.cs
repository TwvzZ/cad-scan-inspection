using System;
using System.Runtime.InteropServices;

namespace CadRegistration
{
    public enum Status : int
    {
        Success = 0,
        NotConverged = 1,
        InvalidArgument = 2,
        EmptyAfterDownsampling = 3,
        InternalError = 4,
        QualityRejected = 5,
        Ambiguous = 6
    }

    public enum Mode : int
    {
        Single = 0,
        Cascade = 1,
        Ensemble = 2
    }

    [Flags]
    public enum Strategy : uint
    {
        Initial = 1,
        Pca = 2,
        FpfhRansac = 4
    }

    [StructLayout(LayoutKind.Sequential)]
    internal struct NativePointCloud
    {
        public IntPtr Xyz;
        public ulong PointCount;
        public ulong XyzStrideBytes;
    }

    [StructLayout(LayoutKind.Sequential)]
    internal struct NativeIcpLevel
    {
        public double VoxelSize;
        public double MaxCorrespondenceDistance;
        public uint MaxIterations;
        public uint Reserved;
    }

    [StructLayout(LayoutKind.Sequential)]
    internal struct NativeOptions
    {
        public uint StructSize;
        public uint AbiVersion;
        public double MaxCorrespondenceDistance;
        public uint MaxIterations;
        public double VoxelSize;
        public Mode Mode;
        public Strategy StrategyMask;
        public double FeatureVoxelSize;
        public uint RansacMaxIterations;
        public double MinInlierRatio;
        public double MaxRmse;
        public double AmbiguityScoreMargin;
        public uint IcpLevelCount;

        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 4)]
        public NativeIcpLevel[] IcpLevels;

        public uint RansacAttempts;
        public uint MaxCandidatesPerStrategy;
        public uint EnableTargetCoverage;
        public uint MaxRefinedCandidatesPerStrategy;
        public double MinTargetCoverage;
    }

    [StructLayout(LayoutKind.Sequential, CharSet = CharSet.Ansi)]
    internal struct NativeCandidateDiagnostic
    {
        public Strategy Strategy;
        public uint Converged;
        public uint Accepted;
        public uint Rank;

        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 16)]
        public double[] SourceToTarget;

        public double Rmse;
        public double InlierRatio;
        public double TargetCoverage;
        public double Score;
        public double SharedCoarseMs;
        public double RefinementMs;
        public double QualityMs;
        public double CandidateElapsedMs;
    }

    [StructLayout(LayoutKind.Sequential, CharSet = CharSet.Ansi)]
    internal struct NativeResult
    {
        public uint StructSize;
        public uint AbiVersion;
        public Status Status;
        public uint Converged;

        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 16)]
        public double[] SourceToTarget;

        public double Rmse;
        public double InlierRatio;
        public double ElapsedMs;
        public ulong SourcePointsUsed;
        public ulong TargetPointsUsed;
        public Strategy SelectedStrategy;
        public uint CandidateCount;
        public uint AcceptedCandidateCount;
        public double Score;
        public double SecondBestScore;
        public double TargetCoverage;
        public uint TargetCacheHit;
        public uint Reserved;
        public double PreprocessingMs;
        public double CoarseRegistrationMs;
        public double RefinementMs;
        public double QualityMs;
        public uint DiagnosticCount;
        public uint DiagnosticReserved;

        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 16)]
        public NativeCandidateDiagnostic[] Diagnostics;

        [MarshalAs(UnmanagedType.ByValTStr, SizeConst = 512)]
        public string ErrorMessage;
    }

    public sealed class CandidateDiagnostic
    {
        public Strategy Strategy { get; internal set; }
        public bool Converged { get; internal set; }
        public bool Accepted { get; internal set; }
        public uint Rank { get; internal set; }
        public double[] SourceToTarget { get; internal set; } = new double[16];
        public double Rmse { get; internal set; }
        public double InlierRatio { get; internal set; }
        public double TargetCoverage { get; internal set; }
        public double Score { get; internal set; }
        public double SharedCoarseMs { get; internal set; }
        public double RefinementMs { get; internal set; }
        public double QualityMs { get; internal set; }
        public double CandidateElapsedMs { get; internal set; }
    }

    public sealed class RegistrationResult
    {
        public Status Status { get; internal set; }
        public bool Converged { get; internal set; }
        public double[] SourceToTarget { get; internal set; } = new double[16];
        public double Rmse { get; internal set; }
        public double InlierRatio { get; internal set; }
        public double ElapsedMs { get; internal set; }
        public string ErrorMessage { get; internal set; } = "";
        public Strategy SelectedStrategy { get; internal set; }
        public uint CandidateCount { get; internal set; }
        public double Score { get; internal set; }
        public double TargetCoverage { get; internal set; }
        public bool TargetCacheHit { get; internal set; }
        public double PreprocessingMs { get; internal set; }
        public double CoarseRegistrationMs { get; internal set; }
        public double RefinementMs { get; internal set; }
        public double QualityMs { get; internal set; }
        public CandidateDiagnostic[] Candidates { get; internal set; } =
            Array.Empty<CandidateDiagnostic>();
    }

    public sealed class RegistrationContext : IDisposable
    {
        private const string DllName = "cad_registration";
        private IntPtr handle;

        [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
        private static extern void cadreg_default_options(ref NativeOptions options);

        [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
        private static extern void cadreg_init_result(ref NativeResult result);

        [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
        private static extern IntPtr cadreg_create();

        [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
        private static extern void cadreg_destroy(IntPtr handle);

        [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
        private static extern Status cadreg_set_target(
            IntPtr handle, ref NativePointCloud target);

        [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
        private static extern Status cadreg_register_source(
            IntPtr handle,
            ref NativePointCloud source,
            IntPtr initialSourceToTarget,
            ref NativeOptions options,
            ref NativeResult result);

        public RegistrationContext(double[] targetXyz)
        {
            ValidateCloud(targetXyz, nameof(targetXyz));
            handle = cadreg_create();
            if (handle == IntPtr.Zero)
                throw new InvalidOperationException(
                    "Could not create registration context.");
            GCHandle targetPin = default;
            try
            {
                targetPin = GCHandle.Alloc(targetXyz, GCHandleType.Pinned);
                var target = new NativePointCloud {
                    Xyz = targetPin.AddrOfPinnedObject(),
                    PointCount = (ulong)(targetXyz.Length / 3),
                    XyzStrideBytes = 0
                };
                Status status = cadreg_set_target(handle, ref target);
                if (status != Status.Success)
                    throw new InvalidOperationException(
                        $"Could not prepare target point cloud: {status}");
            }
            catch
            {
                Dispose();
                throw;
            }
            finally
            {
                if (targetPin.IsAllocated) targetPin.Free();
            }
        }

        public RegistrationResult Register(
            double[] sourceXyz,
            double[]? initialSourceToTarget = null,
            double maxCorrespondenceDistance = 5.0,
            uint maxIterations = 100,
            double voxelSize = 1.0,
            Mode mode = Mode.Cascade,
            Strategy strategies = Strategy.Initial | Strategy.Pca |
                                  Strategy.FpfhRansac)
        {
            ValidateCloud(sourceXyz, nameof(sourceXyz));
            if (handle == IntPtr.Zero)
                throw new ObjectDisposedException(nameof(RegistrationContext));
            if (initialSourceToTarget != null &&
                initialSourceToTarget.Length != 16)
                throw new ArgumentException("Initial matrix must contain 16 values.");

            var options = new NativeOptions {
                IcpLevels = new NativeIcpLevel[4]
            };
            cadreg_default_options(ref options);
            options.MaxCorrespondenceDistance = maxCorrespondenceDistance;
            options.MaxIterations = maxIterations;
            options.VoxelSize = voxelSize;
            options.Mode = mode;
            options.StrategyMask = strategies;
            options.IcpLevelCount = 3;
            options.IcpLevels[0] = new NativeIcpLevel {
                VoxelSize = voxelSize * 6.0,
                MaxCorrespondenceDistance =
                    maxCorrespondenceDistance * 12.0,
                MaxIterations = Math.Min(maxIterations, 40)
            };
            options.IcpLevels[1] = new NativeIcpLevel {
                VoxelSize = voxelSize * 2.5,
                MaxCorrespondenceDistance =
                    maxCorrespondenceDistance * 4.0,
                MaxIterations = Math.Min(maxIterations, 60)
            };
            options.IcpLevels[2] = new NativeIcpLevel {
                VoxelSize = voxelSize,
                MaxCorrespondenceDistance = maxCorrespondenceDistance,
                MaxIterations = maxIterations
            };

            var nativeResult = new NativeResult {
                SourceToTarget = new double[16],
                Diagnostics = new NativeCandidateDiagnostic[16]
            };
            for (int i = 0; i < nativeResult.Diagnostics.Length; ++i)
                nativeResult.Diagnostics[i].SourceToTarget = new double[16];
            cadreg_init_result(ref nativeResult);

            GCHandle sourcePin = default;
            GCHandle matrixPin = default;
            try
            {
                sourcePin = GCHandle.Alloc(sourceXyz, GCHandleType.Pinned);
                IntPtr matrixPointer = IntPtr.Zero;
                if (initialSourceToTarget != null)
                {
                    matrixPin = GCHandle.Alloc(
                        initialSourceToTarget, GCHandleType.Pinned);
                    matrixPointer = matrixPin.AddrOfPinnedObject();
                }

                var source = new NativePointCloud {
                    Xyz = sourcePin.AddrOfPinnedObject(),
                    PointCount = (ulong)(sourceXyz.Length / 3),
                    XyzStrideBytes = 0
                };
                cadreg_register_source(handle, ref source, matrixPointer,
                                       ref options, ref nativeResult);
            }
            finally
            {
                if (matrixPin.IsAllocated) matrixPin.Free();
                if (sourcePin.IsAllocated) sourcePin.Free();
            }

            int diagnosticCount = checked((int)Math.Min(
                nativeResult.DiagnosticCount, 16u));
            var diagnostics = new CandidateDiagnostic[diagnosticCount];
            for (int i = 0; i < diagnosticCount; ++i)
            {
                NativeCandidateDiagnostic item = nativeResult.Diagnostics[i];
                diagnostics[i] = new CandidateDiagnostic {
                    Strategy = item.Strategy,
                    Converged = item.Converged != 0,
                    Accepted = item.Accepted != 0,
                    Rank = item.Rank,
                    SourceToTarget = item.SourceToTarget,
                    Rmse = item.Rmse,
                    InlierRatio = item.InlierRatio,
                    TargetCoverage = item.TargetCoverage,
                    Score = item.Score,
                    SharedCoarseMs = item.SharedCoarseMs,
                    RefinementMs = item.RefinementMs,
                    QualityMs = item.QualityMs,
                    CandidateElapsedMs = item.CandidateElapsedMs
                };
            }
            return new RegistrationResult {
                Status = nativeResult.Status,
                Converged = nativeResult.Converged != 0,
                SourceToTarget = nativeResult.SourceToTarget,
                Rmse = nativeResult.Rmse,
                InlierRatio = nativeResult.InlierRatio,
                ElapsedMs = nativeResult.ElapsedMs,
                ErrorMessage = nativeResult.ErrorMessage ?? "",
                SelectedStrategy = nativeResult.SelectedStrategy,
                CandidateCount = nativeResult.CandidateCount,
                Score = nativeResult.Score,
                TargetCoverage = nativeResult.TargetCoverage,
                TargetCacheHit = nativeResult.TargetCacheHit != 0,
                PreprocessingMs = nativeResult.PreprocessingMs,
                CoarseRegistrationMs = nativeResult.CoarseRegistrationMs,
                RefinementMs = nativeResult.RefinementMs,
                QualityMs = nativeResult.QualityMs,
                Candidates = diagnostics
            };
        }

        public void Dispose()
        {
            if (handle != IntPtr.Zero)
            {
                cadreg_destroy(handle);
                handle = IntPtr.Zero;
            }
            GC.SuppressFinalize(this);
        }

        ~RegistrationContext()
        {
            Dispose();
        }

        internal static void ValidateCloud(double[] xyz, string name)
        {
            if (xyz == null || xyz.Length < 9 || xyz.Length % 3 != 0)
                throw new ArgumentException(
                    "Point cloud must contain at least three XYZ points.", name);
        }
    }

    public static class Registration
    {
        public static RegistrationResult Run(
            double[] sourceXyz,
            double[] targetXyz,
            double[]? initialSourceToTarget = null,
            double maxCorrespondenceDistance = 5.0,
            uint maxIterations = 100,
            double voxelSize = 1.0,
            Mode mode = Mode.Cascade,
            Strategy strategies = Strategy.Initial | Strategy.Pca |
                                  Strategy.FpfhRansac)
        {
            using var context = new RegistrationContext(targetXyz);
            return context.Register(
                sourceXyz, initialSourceToTarget,
                maxCorrespondenceDistance, maxIterations,
                voxelSize, mode, strategies);
        }
    }
}
