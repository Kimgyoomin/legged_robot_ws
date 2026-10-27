#include <algorithm>
#include <chrono>
#include <fstream>
#include <iostream>
#include <vector>
#include "legged_navigation/sector_filter.hpp"
int main(int argc,char**argv){
 std::ifstream f(argv[1],std::ios::binary);std::vector<float> a;float v;while(f.read((char*)&v,sizeof(v)))a.push_back(v);
 const std::array<double,6> r{1,0,0,0,1,0};legged_navigation::SectorFilter old(.6,5,.6,0),now(.6,5,.3,60);volatile size_t total=0;
 for(auto*filter:{&old,&now}){std::vector<double> us;size_t n=a.size()/3;
 for(int k=0;k<120;++k){auto t=std::chrono::steady_clock::now();size_t c=0;for(size_t i=0;i<n;++i)c+=filter->keep(a[3*i],a[3*i+1],a[3*i+2],r);total+=c;auto e=std::chrono::steady_clock::now();if(k>=20)us.push_back(std::chrono::duration<double,std::micro>(e-t).count());}
 std::sort(us.begin(),us.end());std::cout<<(filter==&old?"uniform":"sector")<<" points="<<n<<" median_us="<<us[50]<<" p95_us="<<us[95]<<" per_40000_us="<<us[50]*40000/n<<"\n";}
 std::cout<<"checksum "<<total<<"\n";
}
