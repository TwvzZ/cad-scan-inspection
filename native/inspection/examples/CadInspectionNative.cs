using System;
using System.Runtime.InteropServices;

namespace CadInspection
{
    public enum Status : int
    {
        Success = 0, InvalidArgument = 1, EmptyDetectionRoi = 2,
        InsufficientReference = 3, PlaneRejected = 4,
        BufferTooSmall = 5, InternalError = 6
    }

    public enum DefectType : int { Positive = 1, Negative = 2 }
    public enum ReferenceMode : int { NarrowRoi = 0, WideMultiplane = 1 }

    [StructLayout(LayoutKind.Sequential)]
    internal struct NativeFrame
    {
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 3)] public double[] Origin;
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 3)] public double[] AxisU;
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 3)] public double[] AxisV;
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 3)] public double[] NominalNormal;
    }

    [StructLayout(LayoutKind.Sequential)]
    internal struct NativeOptions
    {
        public uint StructSize, AbiVersion;
        public NativeFrame Frame;
        public ReferenceMode ReferenceMode;
        public uint MaxPlaneCandidates;
        public double MinCandidatePointRatio;
        public double UMin, UMax, VMin, VMax;
        public double DetectionNormalMin, DetectionNormalMax;
        public double ReferenceNormalMin, ReferenceNormalMax, EdgeMargin;
        public uint RansacMaxIterations, RandomSeed;
        public uint RansacEvaluationLimit;
        public double RansacConfidence;
        public double PlaneInlierDistance, MaxNormalAngleDeg;
        public ulong MinReferencePoints;
        public double MinReferenceInlierRatio;
        public uint CoverageGridU, CoverageGridV;
        public double MinReferenceGridCoverage, MaxReferenceRmse;
        public double PositiveDefectThreshold, NegativeDefectThreshold;
        public double DefectClusterCellSize;
        public ulong MinDefectPoints;
        public double MinDefectArea;
        public uint MaxOutputDefects;
        public double FlatnessTrimFraction;
        public double FlatnessWorkingGridSize;
        public uint MinimumZoneMaxIterations;
        public double MinimumZoneTolerance;
    }

    [StructLayout(LayoutKind.Sequential)]
    internal struct NativeCloud { public IntPtr Xyz; public ulong PointCount, XyzStrideBytes; }

    [StructLayout(LayoutKind.Sequential)]
    internal struct NativePlane
    {
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 4)] public double[] Coefficients;
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 3)] public double[] Normal;
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 3)] public double[] Centroid;
        public double NormalAngleDeg, NominalOffset, Rmse, MeanAbsError, MaxAbsError;
        public ulong CandidateCount, InlierCount;
        public double InlierRatio;
        public uint OccupiedGridCells, TotalGridCells;
        public double GridCoverage;
        public uint Reliable;
    }

    [StructLayout(LayoutKind.Sequential)]
    internal struct NativePlaneCandidate
    {
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 4)] public double[] Coefficients;
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 3)] public double[] Normal;
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 3)] public double[] Centroid;
        public double NormalAngleDeg, NominalOffset, Rmse, MeanAbsError, MaxAbsError;
        public ulong InlierCount;
        public double PointRatio, GridCoverage;
        public uint Accepted, Selected;
    }

    [StructLayout(LayoutKind.Sequential)]
    internal struct NativeFlatness
    {
        public ulong EvaluatedPointCount;
        public double LeastSquaresPeakToValley, LeastSquaresMinDeviation, LeastSquaresMaxDeviation;
        public double RobustPeakToValley, RobustMinDeviation, RobustMaxDeviation;
        public double MinimumZoneFlatness;
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 3)] public double[] MinimumZoneNormal;
        public uint MinimumZoneConverged, MinimumZoneIterations;
    }

    [StructLayout(LayoutKind.Sequential)]
    internal struct NativeDefect
    {
        public uint Id;
        public DefectType Type;
        public ulong PointCount;
        public double ProjectedArea, MaximumHeight, MinimumHeight, MeanHeight, EstimatedVolume;
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 3)] public double[] Centroid;
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 3)] public double[] LocalBoundsMin;
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 3)] public double[] LocalBoundsMax;
    }

    [StructLayout(LayoutKind.Sequential)]
    internal struct NativeBuffers
    {
        public uint StructSize, AbiVersion, DefectCapacity;
        public IntPtr Defects;
        public ulong PointLabelCapacity;
        public IntPtr PointLabels;
    }

    [StructLayout(LayoutKind.Sequential, CharSet = CharSet.Ansi)]
    internal struct NativeResult
    {
        public uint StructSize, AbiVersion;
        public Status Status;
        public ulong InputPointCount, FinitePointCount, DetectionRoiPointCount,
                     ReferenceRoiPointCount, RejectedEdgePointCount;
        public uint PlaneCandidateCount, SelectedPlaneCandidate;
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 8)]
        public NativePlaneCandidate[] PlaneCandidates;
        public NativePlane ReferencePlane;
        public NativeFlatness Flatness;
        public uint DefectCount, RequiredDefectCapacity;
        public ulong PositiveDefectPointCount, NegativeDefectPointCount;
        public uint RansacIterationsUsed, RansacEvaluationPointCount;
        public ulong FlatnessWorkingPointCount;
        public double RoiElapsedMs, PlaneElapsedMs, FlatnessElapsedMs,
                      DefectElapsedMs, ElapsedMs;
        [MarshalAs(UnmanagedType.ByValTStr, SizeConst = 512)] public string ErrorMessage;
    }

    public sealed class InspectionConfig
    {
        public ReferenceMode ReferenceMode = ReferenceMode.NarrowRoi;
        public uint MaxPlaneCandidates = 4;
        public double MinCandidatePointRatio = 0.03;
        public double[] Origin = { 0, 0, 0 };
        public double[] AxisU = { 1, 0, 0 };
        public double[] AxisV = { 0, 1, 0 };
        public double[] Normal = { 0, 0, 1 };
        public double UMin = -50, UMax = 50, VMin = -50, VMax = 50;
        public double DetectionMin = -5, DetectionMax = 20;
        public double ReferenceMin = -1, ReferenceMax = 0.5;
        public double EdgeMargin = 1, PlaneInlierDistance = 0.15;
        public double MaxNormalAngleDeg = 5, MaxReferenceRmse = 0.1;
        public ulong MinReferencePoints = 100, MinDefectPoints = 5;
        public double MinReferenceInlierRatio = 0.5, MinGridCoverage = 0.2;
        public double PositiveThreshold = 0.3, NegativeThreshold = 0.3;
        public double ClusterCellSize = 0.5, MinDefectArea = 0.25;
        public uint RansacIterations = 1000, RandomSeed = 1,
                    RansacEvaluationLimit = 5000, MaxDefects = 256;
        public double RansacConfidence = 0.999, FlatnessWorkingGridSize = 0.5;
    }

    public sealed class Defect
    {
        public uint Id; public DefectType Type; public ulong PointCount;
        public double Area, MaximumHeight, MinimumHeight, MeanHeight, Volume;
        public double[] Centroid = Array.Empty<double>();
        public double[] BoundsMin = Array.Empty<double>(), BoundsMax = Array.Empty<double>();
    }

    public sealed class PlaneCandidate
    {
        public double[] Coefficients = Array.Empty<double>();
        public double[] Normal = Array.Empty<double>();
        public double[] Centroid = Array.Empty<double>();
        public double NormalAngleDeg, NominalOffset, Rmse, PointRatio,
                      GridCoverage;
        public ulong InlierCount;
        public bool Accepted, Selected;
    }

    public sealed class InspectionResult
    {
        public Status Status; public string Error = "";
        public double[] Plane = Array.Empty<double>();
        public double PlaneRmse, PlaneInlierRatio, ReferenceCoverage;
        public bool ReferenceReliable;
        public double TotalFlatness, RobustFlatness, MinimumZoneFlatness;
        public Defect[] Defects = Array.Empty<Defect>();
        public PlaneCandidate[] PlaneCandidates = Array.Empty<PlaneCandidate>();
        public uint SelectedPlaneCandidate;
        public uint[] PointLabels = Array.Empty<uint>();
        public double ElapsedMs;
    }

    public static class Inspector
    {
        private const string Dll = "cad_inspection";
        [DllImport(Dll, CallingConvention = CallingConvention.Cdecl)]
        private static extern void cadinspect_default_options(ref NativeOptions value);
        [DllImport(Dll, CallingConvention = CallingConvention.Cdecl)]
        private static extern void cadinspect_init_buffers(ref NativeBuffers value);
        [DllImport(Dll, CallingConvention = CallingConvention.Cdecl)]
        private static extern void cadinspect_init_result(ref NativeResult value);
        [DllImport(Dll, CallingConvention = CallingConvention.Cdecl)]
        private static extern Status cadinspect_analyze(ref NativeCloud cloud,
            ref NativeOptions options, ref NativeBuffers buffers, ref NativeResult result);

        private static double[] V(double[] value, string name)
        {
            if (value == null || value.Length != 3) throw new ArgumentException(name + " must have 3 values");
            return (double[])value.Clone();
        }

        public static InspectionResult Analyze(double[] xyz, InspectionConfig c)
        {
            if (xyz == null || xyz.Length < 9 || xyz.Length % 3 != 0)
                throw new ArgumentException("XYZ must contain at least three points");
            var o = new NativeOptions {
                Frame = new NativeFrame { Origin=new double[3], AxisU=new double[3], AxisV=new double[3], NominalNormal=new double[3] }
            };
            cadinspect_default_options(ref o);
            o.Frame = new NativeFrame { Origin=V(c.Origin,"Origin"), AxisU=V(c.AxisU,"AxisU"), AxisV=V(c.AxisV,"AxisV"), NominalNormal=V(c.Normal,"Normal") };
            o.ReferenceMode=c.ReferenceMode;o.MaxPlaneCandidates=c.MaxPlaneCandidates;
            o.MinCandidatePointRatio=c.MinCandidatePointRatio;
            o.UMin=c.UMin;o.UMax=c.UMax;o.VMin=c.VMin;o.VMax=c.VMax;
            o.DetectionNormalMin=c.DetectionMin;o.DetectionNormalMax=c.DetectionMax;
            o.ReferenceNormalMin=c.ReferenceMin;o.ReferenceNormalMax=c.ReferenceMax;
            o.EdgeMargin=c.EdgeMargin;o.PlaneInlierDistance=c.PlaneInlierDistance;
            o.MaxNormalAngleDeg=c.MaxNormalAngleDeg;o.MaxReferenceRmse=c.MaxReferenceRmse;
            o.MinReferencePoints=c.MinReferencePoints;o.MinReferenceInlierRatio=c.MinReferenceInlierRatio;
            o.MinReferenceGridCoverage=c.MinGridCoverage;o.PositiveDefectThreshold=c.PositiveThreshold;
            o.NegativeDefectThreshold=c.NegativeThreshold;o.DefectClusterCellSize=c.ClusterCellSize;
            o.MinDefectPoints=c.MinDefectPoints;o.MinDefectArea=c.MinDefectArea;
            o.RansacMaxIterations=c.RansacIterations;o.RandomSeed=c.RandomSeed;o.MaxOutputDefects=c.MaxDefects;
            o.RansacEvaluationLimit=c.RansacEvaluationLimit;
            o.RansacConfidence=c.RansacConfidence;
            o.FlatnessWorkingGridSize=c.FlatnessWorkingGridSize;
            int defectSize=Marshal.SizeOf<NativeDefect>();
            IntPtr defectMemory=Marshal.AllocHGlobal(checked(defectSize*(int)c.MaxDefects));
            var labels=new uint[xyz.Length/3];GCHandle xyzPin=default,labelPin=default;
            try {
                xyzPin=GCHandle.Alloc(xyz,GCHandleType.Pinned);labelPin=GCHandle.Alloc(labels,GCHandleType.Pinned);
                var cloud=new NativeCloud { Xyz=xyzPin.AddrOfPinnedObject(),PointCount=(ulong)labels.Length };
                var buffers=new NativeBuffers();cadinspect_init_buffers(ref buffers);buffers.Defects=defectMemory;buffers.DefectCapacity=c.MaxDefects;buffers.PointLabels=labelPin.AddrOfPinnedObject();buffers.PointLabelCapacity=(ulong)labels.Length;
                var planeCandidates=new NativePlaneCandidate[8];
                for(int i=0;i<planeCandidates.Length;++i) planeCandidates[i]=new NativePlaneCandidate{Coefficients=new double[4],Normal=new double[3],Centroid=new double[3]};
                var result=new NativeResult { PlaneCandidates=planeCandidates,ReferencePlane=new NativePlane { Coefficients=new double[4],Normal=new double[3],Centroid=new double[3]},Flatness=new NativeFlatness {MinimumZoneNormal=new double[3]} };cadinspect_init_result(ref result);
                Status status=cadinspect_analyze(ref cloud,ref o,ref buffers,ref result);
                var defects=new Defect[result.DefectCount];
                for(int i=0;i<defects.Length;++i){var d=Marshal.PtrToStructure<NativeDefect>(IntPtr.Add(defectMemory,i*defectSize));defects[i]=new Defect{Id=d.Id,Type=d.Type,PointCount=d.PointCount,Area=d.ProjectedArea,MaximumHeight=d.MaximumHeight,MinimumHeight=d.MinimumHeight,MeanHeight=d.MeanHeight,Volume=d.EstimatedVolume,Centroid=d.Centroid,BoundsMin=d.LocalBoundsMin,BoundsMax=d.LocalBoundsMax};}
                var candidates=new PlaneCandidate[result.PlaneCandidateCount];
                for(int i=0;i<candidates.Length;++i){var p=result.PlaneCandidates[i];candidates[i]=new PlaneCandidate{Coefficients=p.Coefficients,Normal=p.Normal,Centroid=p.Centroid,NormalAngleDeg=p.NormalAngleDeg,NominalOffset=p.NominalOffset,Rmse=p.Rmse,PointRatio=p.PointRatio,GridCoverage=p.GridCoverage,InlierCount=p.InlierCount,Accepted=p.Accepted!=0,Selected=p.Selected!=0};}
                return new InspectionResult{Status=status,Error=result.ErrorMessage??"",Plane=result.ReferencePlane.Coefficients,PlaneRmse=result.ReferencePlane.Rmse,PlaneInlierRatio=result.ReferencePlane.InlierRatio,ReferenceCoverage=result.ReferencePlane.GridCoverage,ReferenceReliable=result.ReferencePlane.Reliable!=0,TotalFlatness=result.Flatness.LeastSquaresPeakToValley,RobustFlatness=result.Flatness.RobustPeakToValley,MinimumZoneFlatness=result.Flatness.MinimumZoneFlatness,Defects=defects,PlaneCandidates=candidates,SelectedPlaneCandidate=result.SelectedPlaneCandidate,PointLabels=labels,ElapsedMs=result.ElapsedMs};
            } finally { if(labelPin.IsAllocated)labelPin.Free();if(xyzPin.IsAllocated)xyzPin.Free();Marshal.FreeHGlobal(defectMemory); }
        }
    }
}
