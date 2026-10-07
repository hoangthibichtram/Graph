#include "ExactSolver.h"
#include <fstream>
#include <sstream>
#include <iostream>
#include <iomanip>
static std::vector<std::string> split(const std::string& s, char d){
    std::vector<std::string> v; std::string t; std::istringstream ss(s);
    while (std::getline(ss,t,d)) v.push_back(t); return v; }
static std::vector<double> nums(const std::string& s){
    std::vector<double> v; std::vector<std::string> p=split(s,',');
    for(size_t i=0;i<p.size();++i) v.push_back(atof(p[i].c_str())); return v; }
int main(int argc,char**argv){
    OptimizationProblem prob;
    prob.campaignBudget = (argc>2)? atof(argv[2]) : 7000.0;
    std::ifstream f(argv[1]); std::string line; int id=0;
    while (std::getline(f,line)){
        if(line.empty()) continue;
        std::vector<std::string> p=split(line,'|');
        if(p[0]=="U"){
            UAVOpt u; u.id=id++; u.baseCode=p[1]; u.code=p[1];
            u.explosive=atof(p[2].c_str());
            std::vector<double> a=nums(p[3]);
            for(size_t k=0;k<a.size();++k) u.aij.push_back((int)a[k]);
            u.pij=nums(p[4]); u.cij=nums(p[5]);
            prob.uavs.push_back(u);
        } else if(p[0]=="T"){
            TargetOpt t; t.id=(int)prob.targets.size()+1; t.name=p[1];
            t.explosive_required=atof(p[2].c_str()); t.value=atof(p[3].c_str());
            prob.targets.push_back(t);
        }
    }
    std::cout<<"n = "<<prob.uavs.size()<<" phuong tien, m = "<<prob.targets.size()
             <<" muc tieu, C = "<<prob.campaignBudget<<"\n";
    ExactSolution r = solveExact(prob);
    if(!r.ok){ std::cout<<"KHONG GIAI DUOC: "<<r.note<<"\n"; return 1; }
    std::cout<<std::fixed<<std::setprecision(2)
             <<"F* = "<<r.value<<"  chi phi = "<<r.cost
             <<"  ("<<r.typeQty.size()<<" chung loai, "<<r.nStates<<" trang thai, "
             <<r.nPackages<<" goi, "<<r.ms<<" ms)\n";
    for(size_t j=0;j<r.plan.size();++j){
        bool any=false; for(size_t t=0;t<r.plan[j].size();++t) if(r.plan[j][t]) any=true;
        std::cout<<"  "<<prob.targets[j].name<<" <- ";
        if(!any) std::cout<<"(bo qua)";
        for(size_t t=0;t<r.plan[j].size();++t)
            if(r.plan[j][t]) std::cout<<r.plan[j][t]<<"x"<<r.typeName[t]<<" ";
        std::cout<<"\n";
    }
    return 0;
}
