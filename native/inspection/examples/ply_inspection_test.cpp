#include "cad_inspection.h"

#include <array>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
struct Point { double x, y, z; };
struct ColoredPoint { double x, y, z; uint8_t r, g, b; };
using Cloud = std::vector<Point>;
enum class Scalar { I8, U8, I16, U16, I32, U32, F32, F64 };

Scalar ParseScalar(const std::string& s) {
  if (s=="char"||s=="int8") return Scalar::I8;
  if (s=="uchar"||s=="uint8") return Scalar::U8;
  if (s=="short"||s=="int16") return Scalar::I16;
  if (s=="ushort"||s=="uint16") return Scalar::U16;
  if (s=="int"||s=="int32") return Scalar::I32;
  if (s=="uint"||s=="uint32") return Scalar::U32;
  if (s=="float"||s=="float32") return Scalar::F32;
  if (s=="double"||s=="float64") return Scalar::F64;
  throw std::runtime_error("unsupported PLY scalar: " + s);
}
template<class T> double Read(std::istream& in) {
  T v{}; in.read(reinterpret_cast<char*>(&v), sizeof(v));
  if (!in) throw std::runtime_error("unexpected end of binary PLY");
  return static_cast<double>(v);
}
double ReadScalar(std::istream& in, Scalar s) {
  switch(s) {
    case Scalar::I8:return Read<int8_t>(in); case Scalar::U8:return Read<uint8_t>(in);
    case Scalar::I16:return Read<int16_t>(in); case Scalar::U16:return Read<uint16_t>(in);
    case Scalar::I32:return Read<int32_t>(in); case Scalar::U32:return Read<uint32_t>(in);
    case Scalar::F32:return Read<float>(in); case Scalar::F64:return Read<double>(in);
  }
  throw std::runtime_error("invalid PLY scalar");
}
Cloud LoadPly(const std::filesystem::path& path) {
  std::ifstream in(path, std::ios::binary);
  if (!in) throw std::runtime_error("cannot open: " + path.u8string());
  std::string line; std::getline(in,line); if (!line.empty()&&line.back()=='\r') line.pop_back();
  if(line!="ply") throw std::runtime_error("not a PLY: " + path.u8string());
  bool ascii=false,binary=false,in_vertex=false,ended=false; uint64_t count=0;
  std::vector<std::pair<Scalar,std::string>> props;
  while(std::getline(in,line)) {
    if(!line.empty()&&line.back()=='\r') line.pop_back();
    std::istringstream ss(line); std::string key; ss>>key;
    if(key=="format") { std::string f; ss>>f; ascii=f=="ascii"; binary=f=="binary_little_endian"; }
    else if(key=="element") { std::string n; uint64_t c; ss>>n>>c; in_vertex=n=="vertex"; if(in_vertex) count=c; }
    else if(key=="property"&&in_vertex) { std::string t,n; ss>>t; if(t=="list") throw std::runtime_error("vertex list property unsupported"); ss>>n; props.push_back({ParseScalar(t),n}); }
    else if(key=="end_header") { ended=true; break; }
  }
  if(!ended||(!ascii&&!binary)||count==0) throw std::runtime_error("invalid PLY header");
  int xi=-1,yi=-1,zi=-1; for(size_t i=0;i<props.size();++i){if(props[i].second=="x")xi=(int)i;if(props[i].second=="y")yi=(int)i;if(props[i].second=="z")zi=(int)i;}
  if(xi<0||yi<0||zi<0) throw std::runtime_error("PLY has no XYZ");
  Cloud out; out.reserve((size_t)count); std::vector<double> v(props.size());
  for(uint64_t row=0;row<count;++row){for(size_t p=0;p<props.size();++p){if(ascii){if(!(in>>v[p]))throw std::runtime_error("truncated ASCII PLY");}else v[p]=ReadScalar(in,props[p].first);} Point q{v[xi],v[yi],v[zi]};if(std::isfinite(q.x)&&std::isfinite(q.y)&&std::isfinite(q.z))out.push_back(q);}
  return out;
}
void SavePly(const std::filesystem::path& path,const std::vector<ColoredPoint>& pts){
  std::ofstream out(path); if(!out) throw std::runtime_error("cannot create: "+path.u8string());
  out<<"ply\nformat ascii 1.0\ncomment darkgray=aligned_scan red=detection green=reference blue=fitted_plane\nelement vertex "<<pts.size()<<"\nproperty double x\nproperty double y\nproperty double z\nproperty uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n"<<std::setprecision(10);
  for(const auto&p:pts)out<<p.x<<' '<<p.y<<' '<<p.z<<' '<<(int)p.r<<' '<<(int)p.g<<' '<<(int)p.b<<'\n';
}
struct Face { const char* name; std::array<double,3> c; std::array<double,2> size; };
const Face faces[] = {
 {"face_01",{-23.0363725810,-20.4486690076,-39.95},{67.6858083221,54.5679517115}},
 {"face_02",{-174.5370423392,-20.4460942780,-39.95},{67.6858181972,54.5679463133}},
 {"face_03",{-326.0370423395,-20.4460942782,-39.95},{67.6858181972,54.5679463133}},
 {"face_04",{-477.5370423391,-20.4460942780,-39.95},{67.6858181972,54.5679463132}},
 {"face_05",{-629.0370423399,-20.4460942783,-39.95},{67.6858181971,54.5679463135}},
 {"face_06",{-780.5370423917,-20.4460943379,-39.95},{67.6858172253,54.5679466839}}
};
double Dot(const Point&a,const std::array<double,3>&b){return a.x*b[0]+a.y*b[1]+a.z*b[2];}
Point Delta(const Point&p,const Face&f){return {p.x-f.c[0],p.y-f.c[1],p.z-f.c[2]};}
void Configure(CadInspectOptions& o,const Face& f){
  cadinspect_default_options(&o); o.reference_mode=CADINSPECT_REFERENCE_NARROW_ROI;
  for(int i=0;i<3;++i)o.frame.origin[i]=f.c[i];
  o.frame.axis_u[0]=0;o.frame.axis_u[1]=1;o.frame.axis_u[2]=0;
  o.frame.axis_v[0]=1;o.frame.axis_v[1]=0;o.frame.axis_v[2]=0;
  o.frame.nominal_normal[0]=0;o.frame.nominal_normal[1]=0;o.frame.nominal_normal[2]=-1;
  o.u_min=-f.size[0]/2;o.u_max=f.size[0]/2;o.v_min=-f.size[1]/2;o.v_max=f.size[1]/2;
  o.detection_normal_min=-1;o.detection_normal_max=1;o.reference_normal_min=-0.4;o.reference_normal_max=0.4;
  o.plane_inlier_distance=0.15;
}
bool Local(const Point&p,const Face&f,const CadInspectOptions&o,double&u,double&v,double&w){
  Point d=Delta(p,f); u=d.y; v=d.x; w=-d.z;
  return u>=o.u_min&&u<=o.u_max&&v>=o.v_min&&v<=o.v_max&&w>=o.detection_normal_min&&w<=o.detection_normal_max;
}
void AddPlane(std::vector<ColoredPoint>& out,const Face&f,const CadInspectOptions&o,const CadInspectResult&r){
  const auto& c=r.reference_plane.coefficients; if(!r.reference_plane.reliable)return;
  const int nu=40,nv=32; for(int i=0;i<=nu;++i)for(int j=0;j<=nv;++j){double u=o.u_min+(o.u_max-o.u_min)*i/nu;double v=o.v_min+(o.v_max-o.v_min)*j/nv;double x=f.c[0]+v,y=f.c[1]+u;double z=std::abs(c[2])>1e-12?-(c[0]*x+c[1]*y+c[3])/c[2]:f.c[2];out.push_back({x,y,z,0,80,255});}
}
int Run(int argc,char**argv){
  if(argc!=4){std::cout<<"Usage: "<<argv[0]<<" standard_cad.ply aligned_scan.ply output_directory\n";return 2;}
  Cloud cad=LoadPly(argv[1]),scan=LoadPly(argv[2]);std::vector<double>xyz(scan.size()*3);for(size_t i=0;i<scan.size();++i){xyz[3*i]=scan[i].x;xyz[3*i+1]=scan[i].y;xyz[3*i+2]=scan[i].z;}
  std::cout<<"standard_points="<<cad.size()<<" aligned_scan_points="<<scan.size()<<"\n";
  std::filesystem::path dir=argv[3];std::filesystem::create_directories(dir);bool all_ok=true;
  for(const Face&f:faces){CadInspectOptions o{};Configure(o,f);std::vector<CadInspectDefect> defects(o.max_output_defects);std::vector<uint32_t>labels(scan.size());CadInspectBuffers b{};cadinspect_init_buffers(&b);b.defects=defects.data();b.defect_capacity=(uint32_t)defects.size();b.point_labels=labels.data();b.point_label_capacity=labels.size();CadInspectResult r{};cadinspect_init_result(&r);CadInspectPointCloud cloud{xyz.data(),scan.size(),0};auto status=cadinspect_analyze(&cloud,&o,&b,&r);all_ok&=status==CADINSPECT_STATUS_SUCCESS;
    std::ostringstream log;
    log<<f.name<<" status="<<(int)status<<" message="<<r.error_message<<" detection="<<r.detection_roi_point_count<<" reference="<<r.reference_roi_point_count<<" flatness_total="<<r.flatness.least_squares_peak_to_valley<<" flatness_robust="<<r.flatness.robust_peak_to_valley<<" flatness_min_zone="<<r.flatness.minimum_zone_flatness<<" defects="<<r.defect_count<<" plane=["<<r.reference_plane.coefficients[0]<<","<<r.reference_plane.coefficients[1]<<","<<r.reference_plane.coefficients[2]<<","<<r.reference_plane.coefficients[3]<<"] plane_rmse="<<r.reference_plane.rmse<<" plane_inlier="<<r.reference_plane.inlier_ratio<<" plane_coverage="<<r.reference_plane.grid_coverage<<" normal_angle="<<r.reference_plane.normal_angle_deg;
    std::cout<<log.str()<<"\n";
    std::ofstream result_log(dir/"inspection_result.txt", std::ios::app);
    result_log<<log.str()<<"\n";
    std::vector<ColoredPoint>vis;vis.reserve(scan.size()+1500);
    std::vector<ColoredPoint>roi;roi.reserve((size_t)r.detection_roi_point_count);
    std::vector<ColoredPoint>detection_roi;detection_roi.reserve((size_t)r.detection_roi_point_count);
    std::vector<ColoredPoint>reference_roi;reference_roi.reserve((size_t)r.reference_roi_point_count);
    for(size_t i=0;i<scan.size();++i){const auto&p=scan[i];uint8_t rr=75,gg=75,bb=75;double u,v,w;bool inside=Local(p,f,o,u,v,w);if(inside){rr=230;gg=35;bb=35;detection_roi.push_back({p.x,p.y,p.z,230,35,35});bool edge=u<o.u_min+o.edge_margin||u>o.u_max-o.edge_margin||v<o.v_min+o.edge_margin||v>o.v_max-o.edge_margin;if(!edge&&w>=o.reference_normal_min&&w<=o.reference_normal_max){rr=30;gg=210;bb=70;reference_roi.push_back({p.x,p.y,p.z,30,210,70});}roi.push_back({p.x,p.y,p.z,rr,gg,bb});}vis.push_back({p.x,p.y,p.z,rr,gg,bb});}
    std::vector<ColoredPoint>plane;plane.reserve(1500);AddPlane(plane,f,o,r);
    vis.insert(vis.end(),plane.begin(),plane.end());
    const std::string prefix=std::string(f.name);
    SavePly(dir/(prefix+"_inspection_colored.ply"),vis);
    SavePly(dir/(prefix+"_roi_only.ply"),roi);
    SavePly(dir/(prefix+"_detection_roi_only.ply"),detection_roi);
    SavePly(dir/(prefix+"_reference_roi_only.ply"),reference_roi);
    SavePly(dir/(prefix+"_fitted_plane_only.ply"),plane);
  }
  std::cout<<"Outputs per face: *_detection_roi_only.ply, *_reference_roi_only.ply, and *_fitted_plane_only.ply can be overlaid on the aligned scan. Colors: detection ROI=red, reference ROI=green, fitted plane=blue\n";return all_ok?0:1;
}
}
int main(int argc,char**argv){try{return Run(argc,argv);}catch(const std::exception&e){std::cerr<<"error: "<<e.what()<<"\n";return 2;}}
