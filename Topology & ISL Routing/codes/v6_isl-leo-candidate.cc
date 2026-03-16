// =================================================================
// isl-leo-candidate.cc  —  v6
//
// Layer 1 — ISL Routing Plan Generator
//
// v6 修正項目：
//   [V6-A] 增量寫檔：每 step append，中途 crash 不遺失資料
//   [V6-B] summary.json 執行狀態：RUNNING → COMPLETED / FAILED
//   [V6-C] progress.json：每 step 更新，記錄跑到哪裡
//   [V6-D] simTime 修正：真正作為模擬上限，tEnd 不能超過 simTime
//   [V6-E] verify_path 真正實作：根據 routing_plan 輸出真實路徑資訊
//   [V6-F] outDir 預設改為 project 目錄，不用 /tmp
//
// Schedule 架構（來自 v5-sched）：
//   單次 Simulator::Run()
//   採樣用 ScheduledSample() 事件驅動
//   GraphCache 每 step 更新（Layer 1 頻率）
//   可直接整合 Layer 2/3 事件
// =================================================================

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/applications-module.h"
#include "ns3/satellite-module.h"
#include "ns3/simulation-helper.h"
#include "ns3/satellite-topology.h"
#include "ns3/satellite-mobility-model.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <ctime>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <limits>
#include <map>
#include <queue>
#include <set>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

using namespace ns3;

namespace
{

// =================================================================
// 資料結構
// =================================================================

struct AppConfig
{
  std::string mode           = "d1_final";
  std::string scenarioFolder = "constellation-telesat-351-sats";
  std::string statsLevel     = "min";

  // [V6-F] 預設 outDir 改為 project 目錄，不用 /tmp
  std::string outDir  = "./results/run_001";
  std::string pcapDir = "./results/run_001/pcap";

  // [V6-D] simTime 是真正的模擬上限
  // tEnd 不能超過 simTime，否則 fatal error
  // SetSimulationTime() 用 simTime，不再用 tEnd+10
  double simTime    = 300.0;  // ns3 模擬上限（s）
  double tStart     = 0.0;
  double tEnd       = 300.0;  // 採樣結束，必須 <= simTime
  double dt         = 1.0;
  double planWindow = 60.0;

  double refLat = 25.0330;
  double refLon = 121.5654;

  double   elevDeg          = 20.0;
  double   islMaxDistanceKm = 5000.0;
  uint32_t gwAnchorCount    = 3;
  uint32_t gwIndex          = 0;

  bool     enablePcap = false;  // 預設關閉，需要時才開
  uint32_t satA       = 0;
  uint32_t satB       = 1;
};

struct CandidateRow
{
  double      time        = 0.0;
  uint32_t    satId       = 0;
  double      elevRefDeg  = 0.0;
  bool        islToGw     = false;
  uint32_t    pathHops    = 0;
  std::string fullPath;
  bool        candidate   = false;
};

struct IslRow
{
  double   time       = 0.0;
  uint32_t satA       = 0;
  uint32_t satB       = 0;
  double   distanceKm = 0.0;
  bool     active     = false;
};

struct PrunedEdge
{
  double   time       = 0.0;
  uint32_t satA       = 0;
  uint32_t satB       = 0;
  double   distanceKm = 0.0;
};

struct PlanRow
{
  double      timeStart          = 0.0;
  double      timeEnd            = 0.0;
  int32_t     servingSat         = -1;
  std::string fullPath;
  int32_t     hopCount           = -1;
  std::string status             = "NO_PATH";
  uint32_t    gwIndex            = 0;
  std::string reason;
  double      maxElevDeg         = 0.0;
  double      visibleDurationSec = 0.0;
  uint32_t    candidateCount     = 0;
  int32_t     nextCandidateSat   = -1;
  double      nextCandidateElev  = 0.0;
};

using IslAdjMap = std::map<uint32_t, std::vector<uint32_t>>;

struct GraphCache
{
  IslAdjMap          adj;
  std::set<uint32_t> gwAnchorSats;
  double             builtAtTime = -1.0;
};

// =================================================================
// [V6-A][V6-B][V6-C] 增量輸出管理器
//
// 設計原則：
//   - CSV 檔案在第一步寫入 header，之後每步 append
//   - summary.json 在 Run() 開始前寫 RUNNING 狀態
//   - progress.json 每步更新，記錄 stepCount / currentTime / wallTime
//   - Run() 結束後更新 summary.json 為 COMPLETED 或 FAILED
// =================================================================

struct IncrementalWriter
{
  std::string outDir;

  // 檔案是否已寫入 header
  bool candidateHeaderWritten  = false;
  bool islHeaderWritten        = false;
  bool prunedHeaderWritten     = false;

  // wall clock 起始時間
  std::chrono::steady_clock::time_point wallStart;

  void Init(const std::string& dir)
  {
    outDir   = dir;
    wallStart = std::chrono::steady_clock::now();
  }

  double WallElapsedSec() const
  {
    auto now = std::chrono::steady_clock::now();
    return std::chrono::duration<double>(now - wallStart).count();
  }

  // [V6-A] append candidate rows
  void AppendCandidates(const std::vector<CandidateRow>& rows)
  {
    std::string path = outDir + "/candidate_sats.csv";
    std::ofstream os(path, std::ios::app);
    if (!candidateHeaderWritten)
      {
        os << "time,satId,elev_ref_deg,isl_to_gw,"
           << "path_hops,full_path,candidate\n";
        candidateHeaderWritten = true;
      }
    os << std::fixed << std::setprecision(3);
    for (const auto& r : rows)
      os << r.time      << ',' << r.satId      << ','
         << r.elevRefDeg << ',' << (r.islToGw?1:0) << ','
         << r.pathHops   << ',' << r.fullPath   << ','
         << (r.candidate?1:0) << '\n';
  }

  // [V6-A] append ISL rows
  void AppendIsl(const std::vector<IslRow>& rows)
  {
    std::string path = outDir + "/isl_connectivity.csv";
    std::ofstream os(path, std::ios::app);
    if (!islHeaderWritten)
      {
        os << "time,satA,satB,distance_km,active\n";
        islHeaderWritten = true;
      }
    os << std::fixed << std::setprecision(3);
    for (const auto& r : rows)
      os << r.time << ',' << r.satA << ',' << r.satB << ','
         << r.distanceKm << ',' << (r.active?1:0) << '\n';
  }

  // [V6-A] append pruned topology rows
  void AppendPruned(const std::vector<PrunedEdge>& edges)
  {
    std::string path = outDir + "/topology_pruned.csv";
    std::ofstream os(path, std::ios::app);
    if (!prunedHeaderWritten)
      {
        os << "time,satA,satB,distance_km\n";
        prunedHeaderWritten = true;
      }
    os << std::fixed << std::setprecision(3);
    for (const auto& e : edges)
      os << e.time << ',' << e.satA << ',' << e.satB << ','
         << e.distanceKm << '\n';
  }

  // [V6-C] 更新 progress.json（每 step 呼叫）
  void UpdateProgress(int stepCount, int totalSteps,
                      double simTime, double lastCompletedTime,
                      int candidateCount, int islCount)
  {
    std::ofstream os(outDir + "/progress.json");
    double elapsed = WallElapsedSec();
    double eta     = (stepCount > 0 && elapsed > 0)
                     ? elapsed / stepCount * (totalSteps - stepCount)
                     : -1.0;
    os << std::fixed << std::setprecision(3);
    os << "{\n"
       << "  \"status\": \"RUNNING\",\n"
       << "  \"step_count\": "          << stepCount         << ",\n"
       << "  \"total_steps\": "         << totalSteps        << ",\n"
       << "  \"current_sim_time\": "    << simTime           << ",\n"
       << "  \"last_completed_step\": " << stepCount         << ",\n"
       << "  \"last_completed_time\": " << lastCompletedTime << ",\n"
       << "  \"wall_elapsed_sec\": "    << elapsed           << ",\n"
       << "  \"eta_sec\": "             << eta               << ",\n"
       << "  \"candidates_written\": "  << candidateCount    << ",\n"
       << "  \"isl_edges_written\": "   << islCount          << "\n"
       << "}\n";
  }

  // [V6-B] 寫 summary.json（RUNNING 狀態，Run() 開始前呼叫）
  void WriteSummaryRunning(const AppConfig& cfg)
  {
    WriteSummaryWithStatus(cfg, "RUNNING", "", {});
  }

  // [V6-B] 寫 summary.json（COMPLETED）
  void WriteSummaryCompleted(const AppConfig& cfg,
                             const std::vector<std::string>& files,
                             const std::string& gwInfo)
  {
    WriteSummaryWithStatus(cfg, "COMPLETED", gwInfo, files);
  }

  // [V6-B] 寫 summary.json（FAILED）
  void WriteSummaryFailed(const AppConfig& cfg,
                          const std::string& errorMsg)
  {
    WriteSummaryWithStatus(cfg, "FAILED", errorMsg, {});
  }

private:
  void WriteSummaryWithStatus(const AppConfig& cfg,
                              const std::string& status,
                              const std::string& gwInfo,
                              const std::vector<std::string>& files)
  {
    std::ofstream os(outDir + "/summary.json");
    double elapsed = WallElapsedSec();
    os << std::fixed << std::setprecision(3);
    os << "{\n"
       << "  \"layer\": 1,\n"
       << "  \"version\": \"v6\",\n"
       << "  \"status\": \""            << status           << "\",\n"
       << "  \"wall_elapsed_sec\": "    << elapsed          << ",\n"
       << "  \"mode\": \""              << cfg.mode         << "\",\n"
       << "  \"scenarioFolder\": \""    << cfg.scenarioFolder << "\",\n"
       << "  \"simTime\": "             << cfg.simTime      << ",\n"
       << "  \"tStart\": "              << cfg.tStart       << ",\n"
       << "  \"tEnd\": "                << cfg.tEnd         << ",\n"
       << "  \"dt\": "                  << cfg.dt           << ",\n"
       << "  \"planWindow\": "          << cfg.planWindow   << ",\n"
       << "  \"refLat\": "              << cfg.refLat       << ",\n"
       << "  \"refLon\": "              << cfg.refLon       << ",\n"
       << "  \"elevDeg\": "             << cfg.elevDeg      << ",\n"
       << "  \"islMaxDistanceKm\": "    << cfg.islMaxDistanceKm << ",\n"
       << "  \"gwIndex\": "             << cfg.gwIndex      << ",\n"
       << "  \"gwAnchorCount\": "       << cfg.gwAnchorCount << ",\n"
       << "  \"gwInfo\": \""            << gwInfo           << "\",\n"
       << "  \"generated_files\": [\n";
    for (size_t i = 0; i < files.size(); ++i)
      os << "    \"" << files[i] << "\""
         << (i+1<files.size()?",\n":"\n");
    os << "  ]\n}\n";
  }
};

// =================================================================
// SampleState
// =================================================================

struct SampleState
{
  AppConfig             cfg;
  std::vector<uint32_t> satIds;
  std::vector<uint32_t> gwIds;
  int                   totalSteps       = 0;

  // [V6-A] 不再累積，每 step 直接寫檔
  IncrementalWriter*    writer           = nullptr;

  // 仍然累積 candidates 供 BuildRoutingPlan 使用
  std::vector<CandidateRow> candidates;

  // 執行狀態
  int         stepCount        = 0;
  int         totalCandWritten = 0;
  int         totalIslWritten  = 0;
  bool        hasError         = false;
  std::string errorMsg;

  bool doCandidate = true;
  bool doIsl       = true;
};

// =================================================================
// [FIX-A] Static base adjacency cache
// =================================================================

static IslAdjMap g_baseAdj;
static bool      g_baseAdjReady = false;

// =================================================================
// 工具函式
// =================================================================

std::string Bool01(bool v) { return v ? "1" : "0"; }

void EnsureDir(const std::string& p)
{ std::ignore = std::system(("mkdir -p '" + p + "'").c_str()); }

std::string JoinPath(const std::string& a, const std::string& b)
{ return (a.empty()||a.back()=='/')?a+b:a+"/"+b; }

double NowWallSec()
{ return static_cast<double>(std::clock()) / CLOCKS_PER_SEC; }

double DegToRad(double d) { return d * M_PI / 180.0; }
double RadToDeg(double r) { return r * 180.0 / M_PI; }

Vector GeoToEcef(double lat, double lon, double alt)
{
  const double a=6378137.0, e2=6.69437999014e-3;
  double la=DegToRad(lat), lo=DegToRad(lon);
  double s=std::sin(la), c=std::cos(la);
  double N=a/std::sqrt(1.0-e2*s*s);
  return Vector((N+alt)*c*std::cos(lo),
                (N+alt)*c*std::sin(lo),
                (N*(1.0-e2)+alt)*s);
}

double DistM(const Vector& a, const Vector& b)
{ double dx=a.x-b.x,dy=a.y-b.y,dz=a.z-b.z;
  return std::sqrt(dx*dx+dy*dy+dz*dz); }

int    NSteps(double s,double e,double d)
       { return (d<=0)?0:(int)std::round((e-s)/d); }
double T(double s,double d,int n) { return s+n*d; }

// =================================================================
// 輸出函式（routing_plan 仍然最後一次寫）
// =================================================================

void WritePlanCsv(const std::string& f,
                  const std::vector<PlanRow>& rows)
{
  std::ofstream os(f);
  os << "time_start,time_end,serving_sat,full_path,hop_count,"
     << "status,gw_index,reason,max_elev_deg,"
     << "visible_duration_sec,candidate_count,"
     << "next_candidate_sat,next_candidate_elev\n";
  os << std::fixed << std::setprecision(3);
  for (const auto& r : rows)
    os << r.timeStart << ',' << r.timeEnd << ','
       << r.servingSat << ',' << r.fullPath << ','
       << r.hopCount << ',' << r.status << ','
       << r.gwIndex << ',' << r.reason << ','
       << r.maxElevDeg << ',' << r.visibleDurationSec << ','
       << r.candidateCount << ','
       << r.nextCandidateSat << ',' << r.nextCandidateElev << '\n';
}

void WriteText(const std::string& f, const std::string& s)
{ std::ofstream os(f); os << s; }

// =================================================================
// 衛星 / GW 查詢
// =================================================================

std::vector<uint32_t> GetSatIds(const AppConfig&)
{
  std::vector<uint32_t> ids;
  uint32_t n=Singleton<SatTopology>::Get()->GetNOrbiterNodes();
  for (uint32_t i=0;i<n;++i)
    if (Singleton<SatTopology>::Get()->GetOrbiterNode(i))
      ids.push_back(i);
  return ids;
}

std::vector<uint32_t> GetGwIds(const AppConfig&)
{
  std::vector<uint32_t> ids;
  auto gws=Singleton<SatTopology>::Get()->GetGwNodes();
  for (uint32_t i=0;i<gws.GetN();++i)
    if (gws.Get(i)) ids.push_back(i);
  return ids;
}

// =================================================================
// ISL Adjacency
// =================================================================

std::map<uint32_t,uint32_t> NodeToSatMap()
{
  std::map<uint32_t,uint32_t> m;
  uint32_t n=Singleton<SatTopology>::Get()->GetNOrbiterNodes();
  for (uint32_t s=0;s<n;++s)
    { auto nd=Singleton<SatTopology>::Get()->GetOrbiterNode(s);
      if (nd) m[nd->GetId()]=s; }
  return m;
}

bool HasEdge(const IslAdjMap& adj, uint32_t a, uint32_t b)
{
  auto it=adj.find(a);
  if (it==adj.end()) return false;
  return std::find(it->second.begin(),it->second.end(),b)
         !=it->second.end();
}

std::vector<Ptr<NetDevice>> PeerDevs(Ptr<NetDevice> dev)
{
  std::vector<Ptr<NetDevice>> p;
  if (!dev) return p;
  auto ch=dev->GetChannel(); if (!ch) return p;
  for (uint32_t i=0;i<ch->GetNDevices();++i)
    { auto o=ch->GetDevice(i); if (o&&o!=dev) p.push_back(o); }
  return p;
}

const IslAdjMap& BaseAdj()
{
  if (g_baseAdjReady) return g_baseAdj;
  std::cout << "[L1] [FIX-A] building base ISL adjacency (once)...\n";
  auto n2s=NodeToSatMap();
  uint32_t n=Singleton<SatTopology>::Get()->GetNOrbiterNodes();
  for (uint32_t sid=0;sid<n;++sid)
    {
      auto node=Singleton<SatTopology>::Get()->GetOrbiterNode(sid);
      if (!node) continue;
      for (uint32_t d=0;d<node->GetNDevices();++d)
        {
          auto dev=node->GetDevice(d); if (!dev) continue;
          for (auto& pd:PeerDevs(dev))
            {
              auto pn=pd?pd->GetNode():nullptr; if (!pn) continue;
              auto it=n2s.find(pn->GetId());
              if (it==n2s.end()||it->second==sid) continue;
              g_baseAdj[sid].push_back(it->second);
            }
        }
    }
  auto dedup=[](IslAdjMap& m){
    for (auto& kv:m){
      std::sort(kv.second.begin(),kv.second.end());
      kv.second.erase(std::unique(kv.second.begin(),kv.second.end()),
                      kv.second.end());}};
  dedup(g_baseAdj);
  for (const auto& kv:g_baseAdj)
    for (uint32_t v:kv.second)
      if (!HasEdge(g_baseAdj,v,kv.first))
        g_baseAdj[v].push_back(kv.first);
  dedup(g_baseAdj);
  g_baseAdjReady=true;
  std::cout<<"[L1] [FIX-A] done. nodes="<<g_baseAdj.size()<<"\n";
  return g_baseAdj;
}

IslAdjMap DynAdj(const AppConfig& cfg)
{
  IslAdjMap dyn;
  for (const auto& kv:BaseAdj())
    {
      uint32_t a=kv.first;
      auto nA=Singleton<SatTopology>::Get()->GetOrbiterNode(a);
      if (!nA) continue;
      auto mA=nA->GetObject<SatMobilityModel>(); if (!mA) continue;
      auto gA=mA->GetGeoPosition();
      auto eA=GeoToEcef(gA.GetLatitude(),gA.GetLongitude(),gA.GetAltitude());
      for (uint32_t b:kv.second)
        {
          if (b<a) continue;
          auto nB=Singleton<SatTopology>::Get()->GetOrbiterNode(b);
          if (!nB) continue;
          auto mB=nB->GetObject<SatMobilityModel>(); if (!mB) continue;
          auto gB=mB->GetGeoPosition();
          auto eB=GeoToEcef(gB.GetLatitude(),gB.GetLongitude(),gB.GetAltitude());
          if (DistM(eA,eB)/1000.0<=cfg.islMaxDistanceKm)
            { dyn[a].push_back(b); dyn[b].push_back(a); }
        }
    }
  for (auto& kv:dyn)
    { std::sort(kv.second.begin(),kv.second.end());
      kv.second.erase(std::unique(kv.second.begin(),kv.second.end()),
                      kv.second.end()); }
  return dyn;
}

std::set<uint32_t> GwAnchors(const AppConfig& cfg)
{
  std::set<uint32_t> anch;
  auto gws=Singleton<SatTopology>::Get()->GetGwNodes();
  if (!gws.GetN()||cfg.gwIndex>=gws.GetN()) return anch;
  auto gw=gws.Get(cfg.gwIndex); if (!gw) return anch;
  auto gwM=gw->GetObject<SatMobilityModel>(); if (!gwM) return anch;
  auto gwG=gwM->GetGeoPosition();
  auto gwE=GeoToEcef(gwG.GetLatitude(),gwG.GetLongitude(),gwG.GetAltitude());
  uint32_t n=Singleton<SatTopology>::Get()->GetNOrbiterNodes();
  std::vector<std::pair<double,uint32_t>> dl;
  for (uint32_t s=0;s<n;++s)
    {
      auto sn=Singleton<SatTopology>::Get()->GetOrbiterNode(s);
      if (!sn) continue;
      auto mob=sn->GetObject<SatMobilityModel>(); if (!mob) continue;
      auto geo=mob->GetGeoPosition();
      auto ecef=GeoToEcef(geo.GetLatitude(),geo.GetLongitude(),geo.GetAltitude());
      dl.emplace_back(DistM(gwE,ecef),s);
    }
  std::sort(dl.begin(),dl.end());
  uint32_t pick=std::min(cfg.gwAnchorCount,(uint32_t)dl.size());
  for (uint32_t i=0;i<pick;++i) anch.insert(dl[i].second);
  return anch;
}

GraphCache BuildCache(const AppConfig& cfg)
{
  GraphCache c;
  c.adj=DynAdj(cfg);
  c.gwAnchorSats=GwAnchors(cfg);
  c.builtAtTime=Simulator::Now().GetSeconds();
  return c;
}

// =================================================================
// BFS
// =================================================================

bool BFS(const IslAdjMap& adj, uint32_t src,
         const std::set<uint32_t>& dsts,
         std::vector<uint32_t>& out)
{
  std::queue<uint32_t> q;
  std::map<uint32_t,int32_t> par;
  std::set<uint32_t> vis;
  q.push(src); vis.insert(src); par[src]=-1;
  int32_t found=-1;
  while (!q.empty())
    {
      uint32_t u=q.front(); q.pop();
      if (dsts.count(u)){found=(int32_t)u;break;}
      auto it=adj.find(u); if (it==adj.end()) continue;
      for (uint32_t v:it->second)
        if (!vis.count(v)){vis.insert(v);par[v]=(int32_t)u;q.push(v);}
    }
  if (found<0) return false;
  out.clear();
  for (int32_t c=found;c!=-1;c=par[c]) out.push_back((uint32_t)c);
  std::reverse(out.begin(),out.end());
  return true;
}

std::string PathStr(uint32_t src, const std::vector<uint32_t>& p, uint32_t gw)
{
  std::ostringstream o;
  o<<"UT->SAT"<<src;
  for (size_t i=1;i<p.size();++i) o<<"->SAT"<<p[i];
  o<<"->GW"<<gw;
  return o.str();
}

bool PathToGw(const AppConfig& cfg, const GraphCache& cache,
              uint32_t sat, uint32_t& hops, std::string& fp)
{
  if (cache.gwAnchorSats.empty()){hops=0;fp.clear();return false;}
  std::vector<uint32_t> p;
  if (!BFS(cache.adj,sat,cache.gwAnchorSats,p)){hops=0;fp.clear();return false;}
  hops=(p.size()>=2)?(uint32_t)(p.size()-1):0;
  fp=PathStr(sat,p,cfg.gwIndex);
  return true;
}

// =================================================================
// 仰角
// =================================================================

double Elev(const AppConfig& cfg, uint32_t satId)
{
  auto node=Singleton<SatTopology>::Get()->GetOrbiterNode(satId);
  if (!node) return -90.0;
  auto mob=node->GetObject<SatMobilityModel>(); if (!mob) return -90.0;
  auto sg=mob->GetGeoPosition();
  auto gs=GeoToEcef(cfg.refLat,cfg.refLon,0.0);
  auto ss=GeoToEcef(sg.GetLatitude(),sg.GetLongitude(),sg.GetAltitude());
  Vector los(ss.x-gs.x,ss.y-gs.y,ss.z-gs.z);
  double lat=DegToRad(cfg.refLat),lon=DegToRad(cfg.refLon);
  Vector up(std::cos(lat)*std::cos(lon),std::cos(lat)*std::sin(lon),std::sin(lat));
  double n=std::sqrt(los.x*los.x+los.y*los.y+los.z*los.z);
  if (n<=0.0) return -90.0;
  return RadToDeg(std::asin((los.x*up.x+los.y*up.y+los.z*up.z)/n));
}

// =================================================================
// ISL connectivity O(E)
// =================================================================

void EvalIsl(const AppConfig& cfg, const GraphCache& cache,
             double t, const std::set<uint32_t>& cands,
             std::vector<IslRow>& outIsl,
             std::vector<PrunedEdge>& outPruned)
{
  for (const auto& kv:cache.adj)
    {
      uint32_t a=kv.first;
      auto nA=Singleton<SatTopology>::Get()->GetOrbiterNode(a);
      if (!nA) continue;
      auto mA=nA->GetObject<SatMobilityModel>(); if (!mA) continue;
      auto gA=mA->GetGeoPosition();
      auto eA=GeoToEcef(gA.GetLatitude(),gA.GetLongitude(),gA.GetAltitude());
      for (uint32_t b:kv.second)
        {
          if (b<=a) continue;
          auto nB=Singleton<SatTopology>::Get()->GetOrbiterNode(b);
          if (!nB) continue;
          auto mB=nB->GetObject<SatMobilityModel>(); if (!mB) continue;
          auto gB=mB->GetGeoPosition();
          auto eB=GeoToEcef(gB.GetLatitude(),gB.GetLongitude(),gB.GetAltitude());
          double dKm=DistM(eA,eB)/1000.0;
          if (cfg.statsLevel=="full"||outIsl.size()<512)
            { IslRow r; r.time=t;r.satA=a;r.satB=b;
              r.distanceKm=dKm;r.active=true; outIsl.push_back(r); }
          if (cands.count(a)&&cands.count(b))
            { PrunedEdge pe;pe.time=t;pe.satA=a;pe.satB=b;
              pe.distanceKm=dKm; outPruned.push_back(pe); }
        }
    }
}

std::string ResolveGwInfo(const AppConfig& cfg)
{
  AppConfig d; auto ids=GetGwIds(d);
  if (ids.empty()||cfg.gwIndex>=ids.size()) return "GW_UNAVAILABLE";
  std::ostringstream o;
  o<<"GW"<<ids[cfg.gwIndex]<<"(index="<<cfg.gwIndex<<")";
  return o.str();
}

// =================================================================
// CollectAtCurrentTime
// =================================================================

void CollectAtCurrentTime(SampleState* state,
                          std::vector<IslRow>& stepIsl,
                          std::vector<PrunedEdge>& stepPruned)
{
  double tSec=Simulator::Now().GetSeconds();
  GraphCache cache=BuildCache(state->cfg);
  std::set<uint32_t> candSet;

  if (state->doCandidate)
    {
      for (uint32_t satId:state->satIds)
        {
          CandidateRow row;
          row.time=tSec; row.satId=satId;
          row.elevRefDeg=Elev(state->cfg,satId);
          if (row.elevRefDeg>=state->cfg.elevDeg)
            {
              row.islToGw=PathToGw(state->cfg,cache,satId,
                                   row.pathHops,row.fullPath);
              row.candidate=row.islToGw;
              if (row.candidate) candSet.insert(satId);
            }
          state->candidates.push_back(row);
        }
    }

  if (state->doIsl)
    EvalIsl(state->cfg,cache,tSec,candSet,stepIsl,stepPruned);
}

// =================================================================
// ScheduledSample — 核心事件回呼（含增量寫檔）
// =================================================================

void ScheduledSample(SampleState* state, double tSec)
{
  double w0=NowWallSec();
  state->stepCount++;

  // 採樣
  std::vector<IslRow>    stepIsl;
  std::vector<PrunedEdge> stepPruned;
  CollectAtCurrentTime(state, stepIsl, stepPruned);

  double w1=NowWallSec();

  // [V6-A] 每 step 立即 append 寫檔
  size_t base=state->candidates.size()-state->satIds.size();
  std::vector<CandidateRow> stepCand(
    state->candidates.begin()+base,
    state->candidates.end());

  if (state->doCandidate)
    {
      state->writer->AppendCandidates(stepCand);
      state->totalCandWritten += (int)stepCand.size();
    }
  if (state->doIsl)
    {
      state->writer->AppendIsl(stepIsl);
      state->writer->AppendPruned(stepPruned);
      state->totalIslWritten += (int)stepIsl.size();
    }

  // 統計
  uint32_t vis=0, reach=0;
  for (const auto& r:stepCand)
    { if (r.elevRefDeg>=state->cfg.elevDeg)++vis;
      if (r.candidate)++reach; }

  std::cout << "[L1] step=" << state->stepCount
            << "/" << state->totalSteps
            << " t=" << tSec
            << " wall=" << (w1-w0) << "s"
            << " vis=" << vis << " reach=" << reach
            << "\n";

  // [V6-C] 每 step 更新 progress.json
  state->writer->UpdateProgress(
    state->stepCount, state->totalSteps,
    tSec, tSec,
    state->totalCandWritten,
    state->totalIslWritten);

  // 排下一步或停止
  double nextT=tSec+state->cfg.dt;
  if (nextT<=state->cfg.tEnd+1e-9)
    Simulator::Schedule(Seconds(state->cfg.dt),
                        &ScheduledSample, state, nextT);
  else
    {
      std::cout<<"[L1] all samples done at t="<<tSec<<"\n";
      Simulator::Stop();
    }
}

// =================================================================
// BuildRoutingPlan
// =================================================================

std::vector<PlanRow>
BuildRoutingPlan(const AppConfig& cfg,
                 const std::vector<CandidateRow>& cands)
{
  std::map<double,std::vector<CandidateRow>> byT;
  for (const auto& r:cands) byT[r.time].push_back(r);

  struct Score {
    double maxElev=-1e9;
    uint32_t minHops=std::numeric_limits<uint32_t>::max();
    std::string bestPath;
    std::vector<double> vt;
  };

  std::vector<PlanRow> plan;
  int nW=(int)std::ceil((cfg.tEnd-cfg.tStart)/cfg.planWindow);

  for (int wi=0;wi<nW;++wi)
    {
      double ws=cfg.tStart+wi*cfg.planWindow;
      double we=std::min(ws+cfg.planWindow,cfg.tEnd);
      std::map<uint32_t,Score> sc; uint32_t tot=0;

      for (const auto& kv:byT)
        {
          if (kv.first<ws||kv.first>=we) continue;
          for (const auto& r:kv.second)
            {
              if (!r.candidate) continue; ++tot;
              auto& s=sc[r.satId];
              if (r.elevRefDeg>s.maxElev) s.maxElev=r.elevRefDeg;
              if (r.pathHops<s.minHops){s.minHops=r.pathHops;s.bestPath=r.fullPath;}
              s.vt.push_back(kv.first);
            }
        }

      PlanRow pr; pr.timeStart=ws; pr.timeEnd=we;
      pr.gwIndex=cfg.gwIndex; pr.candidateCount=tot;

      if (sc.empty())
        { pr.status="NO_PATH"; pr.reason="no_candidate_in_window";
          plan.push_back(pr); continue; }

      auto best=std::min_element(sc.begin(),sc.end(),
        [](const auto& a,const auto& b){
          if (std::abs(a.second.maxElev-b.second.maxElev)>1e-6)
            return a.second.maxElev>b.second.maxElev;
          return a.second.minHops<b.second.minHops;});

      int32_t nxSat=-1; double nxElev=0.0;
      if (sc.size()>=2)
        {
          auto sec=std::min_element(sc.begin(),sc.end(),
            [&](const auto& a,const auto& b){
              if (a.first==best->first) return false;
              if (b.first==best->first) return true;
              if (std::abs(a.second.maxElev-b.second.maxElev)>1e-6)
                return a.second.maxElev>b.second.maxElev;
              return a.second.minHops<b.second.minHops;});
          if (sec->first!=best->first)
            {nxSat=(int32_t)sec->first;nxElev=sec->second.maxElev;}
        }

      // 最長連續可視區段
      auto& vt=best->second.vt;
      std::sort(vt.begin(),vt.end());
      double longest=0.0,runLen=vt.empty()?0.0:cfg.dt;
      for (size_t i=1;i<vt.size();++i)
        { if (vt[i]-vt[i-1]<=cfg.dt*1.5) runLen+=cfg.dt;
          else{longest=std::max(longest,runLen);runLen=cfg.dt;} }
      longest=std::max(longest,runLen);

      pr.servingSat=(int32_t)best->first;
      pr.fullPath=best->second.bestPath;
      pr.hopCount=(int32_t)best->second.minHops;
      pr.status="OK"; pr.reason="best_elevation_then_min_hops";
      pr.maxElevDeg=best->second.maxElev;
      pr.visibleDurationSec=longest;
      pr.nextCandidateSat=nxSat; pr.nextCandidateElev=nxElev;
      plan.push_back(pr);
    }
  return plan;
}

// =================================================================
// [V6-E] RunVerifyPath — 真正實作
//
// 根據 routing_plan 的 servingSat + fullPath，
// 在 tStart 時刻計算該路徑的實際幾何資訊：
//   - 每個中間節點的位置
//   - 每段 ISL 的實際距離
//   - servingSat 對 refPoint 的仰角
//   - 路徑是否在 ISL 距離門檻內
// =================================================================

void RunVerifyPath(const AppConfig& cfg,
                   const std::vector<PlanRow>& planRows)
{
  EnsureDir(cfg.outDir);

  std::ostringstream report;
  report << std::fixed << std::setprecision(3);
  report << "# verify_path report\n"
         << "# generated at sim_t=" << cfg.tStart << "\n"
         << "# ref_point=(" << cfg.refLat << "," << cfg.refLon << ")\n"
         << "# isl_max_dist_km=" << cfg.islMaxDistanceKm << "\n\n";

  if (planRows.empty())
    {
      report << "NO_PLAN: routing_plan is empty\n";
      WriteText(JoinPath(cfg.outDir,"verify_path.txt"), report.str());
      return;
    }

  // 取第一個 OK 的 window
  const PlanRow* pr = nullptr;
  for (const auto& r : planRows)
    if (r.status == "OK") { pr = &r; break; }

  if (!pr)
    {
      report << "NO_PATH: all windows have status=NO_PATH\n";
      WriteText(JoinPath(cfg.outDir,"verify_path.txt"), report.str());
      return;
    }

  report << "window=[" << pr->timeStart << "," << pr->timeEnd << "]\n"
         << "serving_sat=" << pr->servingSat << "\n"
         << "full_path=" << pr->fullPath << "\n"
         << "hop_count=" << pr->hopCount << "\n"
         << "max_elev_deg=" << pr->maxElevDeg << "\n\n";

  // 解析 fullPath 中的 SAT 節點序列
  // 格式：UT->SAT12->SAT45->SAT89->GW0
  std::vector<uint32_t> satSeq;
  std::string token;
  std::istringstream ss(pr->fullPath);
  while (std::getline(ss, token, '>'))
    {
      if (token.substr(0,3) == "SAT")
        {
          try {
            satSeq.push_back(std::stoul(token.substr(3)));
          } catch (...) {}
        }
    }

  if (satSeq.empty())
    {
      report << "PARSE_ERROR: cannot extract SAT sequence from path\n";
      WriteText(JoinPath(cfg.outDir,"verify_path.txt"), report.str());
      return;
    }

  report << "# node-by-node geometry at t=" << cfg.tStart << "\n";
  report << "# format: NODE lat lon alt_km elev_from_ref\n\n";

  Vector gsEcef = GeoToEcef(cfg.refLat, cfg.refLon, 0.0);

  std::vector<Vector> nodeEcef;
  for (uint32_t sid : satSeq)
    {
      auto node = Singleton<SatTopology>::Get()->GetOrbiterNode(sid);
      if (!node) { report << "SAT" << sid << " NOT_FOUND\n"; continue; }
      auto mob = node->GetObject<SatMobilityModel>();
      if (!mob) { report << "SAT" << sid << " NO_MOBILITY\n"; continue; }
      auto geo = mob->GetGeoPosition();
      Vector ecef = GeoToEcef(geo.GetLatitude(),
                               geo.GetLongitude(),
                               geo.GetAltitude());
      nodeEcef.push_back(ecef);

      // 對參考點的仰角
      Vector los(ecef.x-gsEcef.x,ecef.y-gsEcef.y,ecef.z-gsEcef.z);
      double lat=DegToRad(cfg.refLat),lon=DegToRad(cfg.refLon);
      Vector up(std::cos(lat)*std::cos(lon),
                std::cos(lat)*std::sin(lon),
                std::sin(lat));
      double n=std::sqrt(los.x*los.x+los.y*los.y+los.z*los.z);
      double elev=(n>0)?RadToDeg(std::asin(
        (los.x*up.x+los.y*up.y+los.z*up.z)/n)):-90.0;

      report << "SAT" << sid
             << " lat=" << geo.GetLatitude()
             << " lon=" << geo.GetLongitude()
             << " alt_km=" << geo.GetAltitude()/1000.0
             << " elev_from_ref=" << elev << "deg\n";
    }

  // ISL 每段距離驗證
  report << "\n# ISL segment distances\n";
  bool allValid = true;
  for (size_t i=1; i<satSeq.size() && i<nodeEcef.size(); ++i)
    {
      double dKm = DistM(nodeEcef[i-1], nodeEcef[i]) / 1000.0;
      bool ok = (dKm <= cfg.islMaxDistanceKm);
      if (!ok) allValid = false;
      report << "SAT" << satSeq[i-1] << "->SAT" << satSeq[i]
             << " dist=" << dKm << "km"
             << " [" << (ok?"OK":"EXCEEDS_LIMIT") << "]\n";
    }

  report << "\n# verdict: "
         << (allValid ? "PATH_VALID" : "PATH_INVALID_ISL_DISTANCE")
         << "\n";

  WriteText(JoinPath(cfg.outDir,"verify_path.txt"), report.str());
  std::cout << "[L1] verify_path.txt written ("
            << (allValid?"VALID":"INVALID") << ")\n";
}

// =================================================================
// ParseArgs
// =================================================================

void ParseArgs(int argc, char** argv, AppConfig& cfg)
{
  CommandLine cmd(__FILE__);
  cmd.AddValue("mode",
    "verify_path|candidate_scan|isl_connectivity|plan|d1_final",
    cfg.mode);
  cmd.AddValue("scenarioFolder", "Scenario folder",      cfg.scenarioFolder);
  // [V6-D] simTime 現在真正控制 ns3 模擬上限
  // tEnd 必須 <= simTime，否則 fatal error
  cmd.AddValue("simTime",   "ns3 sim upper bound (s)",   cfg.simTime);
  cmd.AddValue("tStart",    "Sample start (s)",           cfg.tStart);
  cmd.AddValue("tEnd",      "Sample end (s), must<=simTime", cfg.tEnd);
  cmd.AddValue("dt",        "Sample interval (s)",        cfg.dt);
  cmd.AddValue("planWindow","Layer1 window (s)",          cfg.planWindow);
  cmd.AddValue("refLat",    "Ref latitude (deg)",         cfg.refLat);
  cmd.AddValue("refLon",    "Ref longitude (deg)",        cfg.refLon);
  cmd.AddValue("elevDeg",   "Min elevation (deg)",        cfg.elevDeg);
  cmd.AddValue("islMaxDistanceKm","ISL max dist (km)",    cfg.islMaxDistanceKm);
  cmd.AddValue("gwIndex",   "GW index",                   cfg.gwIndex);
  cmd.AddValue("gwAnchorCount","GW anchor count",         cfg.gwAnchorCount);
  cmd.AddValue("statsLevel","min|full",                   cfg.statsLevel);
  cmd.AddValue("outDir",    "Output dir (not /tmp)",      cfg.outDir);
  cmd.AddValue("pcapDir",   "PCAP dir",                   cfg.pcapDir);
  cmd.AddValue("enablePcap","Enable pcap",                cfg.enablePcap);
  cmd.AddValue("satA",      "satA (unused placeholder)",  cfg.satA);
  cmd.AddValue("satB",      "satB (unused placeholder)",  cfg.satB);
  cmd.Parse(argc, argv);
}

} // namespace

// =================================================================
// main
// =================================================================

int main(int argc, char** argv)
{
  AppConfig cfg;
  ParseArgs(argc, argv, cfg);

  // [V6-D] simTime 真正作為上限，tEnd 不能超過 simTime
  if (cfg.tEnd > cfg.simTime)
    NS_FATAL_ERROR("[V6-D] tEnd=" << cfg.tEnd
                   << " exceeds simTime=" << cfg.simTime
                   << ". Set simTime >= tEnd.");

  if (cfg.tStart < 0 || cfg.tStart >= cfg.tEnd)
    NS_FATAL_ERROR("Invalid tStart=" << cfg.tStart
                   << " tEnd=" << cfg.tEnd);

  // [V6-F] 警告使用者不要用 /tmp
  if (cfg.outDir.substr(0,4) == "/tmp")
    std::cout << "[L1] WARNING: outDir=" << cfg.outDir
              << " is under /tmp and may be cleared on reboot.\n"
              << "[L1] Consider using a project directory instead.\n";

  Config::SetDefault("ns3::SatConf::ForwardLinkRegenerationMode",
                     EnumValue(SatEnums::REGENERATION_NETWORK));
  Config::SetDefault("ns3::SatConf::ReturnLinkRegenerationMode",
                     EnumValue(SatEnums::REGENERATION_NETWORK));
  Config::SetDefault("ns3::SatHelper::BeamNetworkAddress",
                     Ipv4AddressValue("20.1.0.0"));
  Config::SetDefault("ns3::SatHelper::GwNetworkAddress",
                     Ipv4AddressValue("10.1.0.0"));
  Config::SetDefault("ns3::SatHelper::UtNetworkAddress",
                     Ipv4AddressValue("250.1.0.0"));
  Config::SetDefault("ns3::SatBbFrameConf::AcmEnabled",
                     BooleanValue(true));
  Config::SetDefault("ns3::SatEnvVariables::EnableSimulationOutputOverwrite",
                     BooleanValue(true));
  Config::SetDefault("ns3::SatHelper::PacketTraceEnabled",
                     BooleanValue(false));

  Ptr<SimulationHelper> simHelper =
      CreateObject<SimulationHelper>("isl-leo-candidate");
  simHelper->LoadScenario(cfg.scenarioFolder);

  // [V6-D] 真正用 cfg.simTime 作為上限
  // 內部加 1.0s 緩衝：讓 t=tEnd 的最後採樣事件一定能觸發
  // 使用者只需設 simTime >= tEnd，緩衝對使用者不可見
  // ScheduledSample 仍在 tEnd 後自行呼叫 Simulator::Stop()
  simHelper->SetSimulationTime(Seconds(cfg.simTime + 1.0));

  if (cfg.scenarioFolder == "constellation-telesat-351-sats")
    simHelper->SetBeamSet({1, 43, 60, 64});
  else
    simHelper->SetBeamSet({43, 30});

  simHelper->SetUserCountPerUt(2);
  simHelper->CreateSatScenario();

  const auto satIds = GetSatIds(cfg);
  const auto gwIds  = GetGwIds(cfg);

  EnsureDir(cfg.outDir);
  if (cfg.enablePcap) EnsureDir(cfg.pcapDir);

  // [V6-B] IncrementalWriter 初始化
  IncrementalWriter writer;
  writer.Init(cfg.outDir);

  std::cout
    << "[L1] ================================================\n"
    << "[L1] ISL Routing Plan Generator  v6\n"
    << "[L1] ================================================\n"
    << "[L1] orbiter=" << Singleton<SatTopology>::Get()->GetNOrbiterNodes()
    << " gw=" << Singleton<SatTopology>::Get()->GetGwNodes().GetN() << "\n"
    << "[L1] mode=" << cfg.mode << "\n"
    << "[L1] simTime=" << cfg.simTime
    << " tStart/tEnd/dt=" << cfg.tStart<<"/"<<cfg.tEnd<<"/"<<cfg.dt<<"\n"
    << "[L1] planWindow=" << cfg.planWindow
    << " elevDeg=" << cfg.elevDeg
    << " islMax=" << cfg.islMaxDistanceKm << "km\n"
    << "[L1] gwIndex=" << cfg.gwIndex
    << " anchorCount=" << cfg.gwAnchorCount << "\n"
    << "[L1] outDir=" << cfg.outDir << "\n"
    << "[L1] [V6] incremental write / live progress / real simTime\n"
    << "[L1] ================================================\n\n";

  std::vector<std::string> generatedFiles;
  std::vector<PlanRow>     planRows;

  if (cfg.mode == "verify_path")
    {
      // verify_path 在沒有 routing_plan 的情況下無法執行
      // 需要先跑 plan 或 d1_final 產生 routing_plan.csv
      std::cout << "[L1] verify_path requires routing_plan.csv\n"
                << "[L1] Run mode=plan or mode=d1_final first\n";
      Simulator::Destroy();
      return 0;
    }

  // [V6-B] Run() 開始前先寫 RUNNING 狀態
  writer.WriteSummaryRunning(cfg);
  generatedFiles.push_back("summary.json");
  generatedFiles.push_back("progress.json");

  // 建立 SampleState
  SampleState state;
  state.cfg        = cfg;
  state.satIds     = satIds;
  state.gwIds      = gwIds;
  state.totalSteps = NSteps(cfg.tStart, cfg.tEnd, cfg.dt) + 1;
  state.writer     = &writer;

  if (cfg.mode == "candidate_scan")
    { state.doCandidate=true;  state.doIsl=false; }
  else if (cfg.mode == "isl_connectivity")
    { state.doCandidate=false; state.doIsl=true;  }
  else
    { state.doCandidate=true;  state.doIsl=true;  }

  if (gwIds.empty() || cfg.gwIndex >= gwIds.size())
    {
      writer.WriteSummaryFailed(cfg, "No valid GW");
      NS_FATAL_ERROR("No valid GW for gwIndex=" << cfg.gwIndex);
    }

  if (state.doCandidate) generatedFiles.push_back("candidate_sats.csv");
  if (state.doIsl)
    {
      generatedFiles.push_back("isl_connectivity.csv");
      generatedFiles.push_back("topology_pruned.csv");
    }

  // 排第一個採樣事件
  Simulator::Schedule(Seconds(cfg.tStart),
                      &ScheduledSample, &state, cfg.tStart);

  std::cout << "[L1] Simulator::Run() starting\n";
  Simulator::Run();
  std::cout << "[L1] Simulator::Run() finished\n";

  if (state.hasError)
    {
      writer.WriteSummaryFailed(cfg, state.errorMsg);
      std::cerr << "[L1] FAILED: " << state.errorMsg << "\n";
      Simulator::Destroy();
      return 1;
    }

  std::cout << "[L1] steps=" << state.stepCount
            << " cand_written=" << state.totalCandWritten
            << " isl_written=" << state.totalIslWritten << "\n";

  // routing_plan（最後一次寫，因為需要所有 step 的資料）
  if (cfg.mode == "plan" || cfg.mode == "d1_final")
    {
      planRows = BuildRoutingPlan(cfg, state.candidates);
      WritePlanCsv(JoinPath(cfg.outDir,"routing_plan.csv"), planRows);
      generatedFiles.push_back("routing_plan.csv");
    }

  // [V6-E] verify_path 在 d1_final 模式下自動執行
  if (cfg.mode == "d1_final" && !planRows.empty())
    {
      RunVerifyPath(cfg, planRows);
      generatedFiles.push_back("verify_path.txt");
    }

  // [V6-B] 正常結束寫 COMPLETED
  writer.WriteSummaryCompleted(cfg, generatedFiles, ResolveGwInfo(cfg));

  // [V6-C] 最終 progress.json
  writer.UpdateProgress(state.stepCount, state.totalSteps,
                        cfg.tEnd, cfg.tEnd,
                        state.totalCandWritten,
                        state.totalIslWritten);

  std::cout << "[L1] done -> " << cfg.outDir << "\n"
            << "[L1] status: COMPLETED\n"
            << "[L1] routing_plan.csv    → Layer 2\n"
            << "[L1] topology_pruned.csv → Layer 2\n"
            << "[L1] verify_path.txt     → path validation\n"
            << "[L1] progress.json       → run status\n\n";

  Simulator::Destroy();
  return 0;
}
