#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/geometry/Pose3.h>
#include <iostream>
#include <stdexcept>
using namespace gtsam;
void check(bool c,const char*m){if(!c)throw std::runtime_error(m);}
Pose3 pose(double x){return Pose3(Rot3(),Point3(x,0,0));}
void compare(ISAM2&a,ISAM2&b) {
 check(a.calculateEstimate().equals(b.calculateEstimate(),1e-8),"estimate differs");
 check(a.getFactorsUnsafe().equals(b.getFactorsUnsafe(),1e-8),"factor set differs");
 check(a.equals(b,1e-8),"tree/linearization/index differs");
}
int main(){try {
 ISAM2Params p;p.relinearizeSkip=1;p.relinearizeThreshold=.01;
 ISAM2 reference(p),trial(p);
 auto noise=noiseModel::Isotropic::Sigma(6,.1);
 NonlinearFactorGraph g;Values v;
 g.add(PriorFactor<Pose3>(0,pose(0),noise));v.insert(0,pose(0));
 for(int i=1;i<4;i++){g.add(BetweenFactor<Pose3>(i-1,i,pose(1),noise));v.insert(i,pose(i));}
 reference.update(g,v);trial.update(g,v);reference.update();trial.update();compare(reference,trial);
 for(int repeat=0;repeat<3;++repeat){
  trial.backup();NonlinearFactorGraph bad;Values extra;
  bad.add(BetweenFactor<Pose3>(0,3,pose(50),noise));
  bad.add(BetweenFactor<Pose3>(3,4,pose(1),noise));extra.insert(4,pose(4));
  FactorIndices removed;removed.push_back(2);
  trial.update(bad,extra,removed);trial.update();
  check(trial.calculateEstimate().exists(4),"trial did not insert new variable");
  trial.recover();compare(reference,trial);
  trial.update();reference.update();compare(reference,trial);
 }
 g.resize(0);v.clear();g.add(BetweenFactor<Pose3>(3,4,pose(1),noise));v.insert(4,pose(4.1));
 reference.update(g,v);trial.update(g,v);reference.update();trial.update();compare(reference,trial);
 bool threw=false;try{ISAM2 empty;empty.recover();}catch(const std::logic_error&){threw=true;}
 check(threw,"recover without snapshot accepted");
 std::cout<<"PASS snapshot: repeated rollback, factor removal, inserted variable, future update, missing snapshot\n";
 }catch(const std::exception&e){std::cerr<<e.what()<<"\n";return 1;}}
