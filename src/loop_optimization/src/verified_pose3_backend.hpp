// Included inside single_backend. Solver and persistent state are worker-owned.
bool processVerified() {
    if(!state.verified_mode)return false;
    auto chosen=state.verified.end();
    for(auto it=state.verified.begin();it!=state.verified.end();++it) {
        if(it->first.second>=int(keyframes_.size()))continue;
        if(chosen==state.verified.end()||std::make_pair(it->first.second,it->first.first)<std::make_pair(chosen->first.second,chosen->first.first))chosen=it;
    }
    if(chosen==state.verified.end())return false;
    Pair p=chosen->first;auto r=chosen->second;state.verified.erase(chosen);
    if(state.terminal.count(p)){event(p,"duplicate","pose3_already_terminal");return true;}
    auto measurement=factor_archive::readPose(r.get_child("measurement"));
    auto sigmas=factor_archive::numbers(r.get_child("sigmas"));gtsam::Vector6 sig;
    for(int i=0;i<6;++i)sig[i]=sigmas[i];
    auto gaussian=gtsam::noiseModel::Diagonal::Sigmas(sig);
    auto noise=gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Cauchy::Create(1),gaussian);
    gtsam::NonlinearFactorGraph graph;graph.add(gtsam::BetweenFactor<gtsam::Pose3>(p.first,p.second,measurement,noise));
    ++state.transaction;event(p,"trial","verified_pose3");dump("before_trial");
    auto archive=factor_archive::transaction(state.transaction,state.session_id,graph,gtsam::Values(),4,{r.get<std::string>("loop_id")});
    const auto start=std::chrono::steady_clock::now();
    isam_->backup();gtsam::ISAM2Result result;std::string reason;
    double max_loop=0,max_graph=0,max_weighted=0;
    try {
        result=isam_->update(graph,gtsam::Values());for(int i=0;i<4;++i)isam_->update();
        gts_cur_vals_=isam_->calculateEstimate();
        for(auto key:gts_cur_vals_.keys())if(!gts_cur_vals_.at<gtsam::Pose3>(key).matrix().allFinite())throw std::runtime_error("nonfinite_pose");
        const auto& factors=isam_->getFactorsUnsafe();
        for(size_t i=0;i<factors.size();++i) {
            auto f=boost::dynamic_pointer_cast<gtsam::NoiseModelFactor>(factors[i]);if(!f)continue;
            auto model=f->noiseModel();if(auto robust=boost::dynamic_pointer_cast<gtsam::noiseModel::Robust>(model))model=robust->noise();
            double raw=model->whiten(f->unwhitenedError(gts_cur_vals_)).norm()/std::sqrt(double(f->dim()));
            double weighted=std::sqrt(2*f->error(gts_cur_vals_)/f->dim());
            if(!std::isfinite(raw)||!std::isfinite(weighted))throw std::runtime_error("nonfinite_normalized_residual");
            max_graph=std::max(max_graph,raw);
            if(state.factor_owner.count(i)||i==result.newFactorsIndices.at(0)){max_loop=std::max(max_loop,raw);max_weighted=std::max(max_weighted,weighted);}
        }
        if(max_loop>state.max_loop_normalized)reason="loop_normalized_residual";
        else if(max_graph>state.max_graph_normalized)reason="graph_normalized_residual";
#ifdef LTAOM_TRANSACTION_TEST_HOOKS
        if(p.second==state.force_reject)reason="test_forced_rejection";
#endif
    }catch(const std::exception& e){reason=std::string("solver_exception: ")+e.what();}
    state.verified_audit<<"{\"transaction\":"<<state.transaction<<",\"loop_id\":"<<quote(r.get<std::string>("loop_id"))
        <<",\"max_loop_raw_normalized\":"<<max_loop<<",\"max_loop_robust_normalized\":"<<max_weighted
        <<",\"max_graph_raw_normalized\":"<<max_graph<<",\"decision\":"<<quote(reason.empty()?"accepted":reason)<<"}"<<std::endl;
    if(!reason.empty()) {isam_->recover();refresh();reject(p,reason);dump("rollback");recordTiming("pose3_rollback",start);return true;}
    archiveCommit(archive,result);refresh();
    size_t index=result.newFactorsIndices.at(0);state.factor_owner[index]=p;state.terminal[p]="accepted";++state.accepted;
    event(p,"accepted","pose3_committed",{index});
    VOXEL_LOC loc;loc.x=p.first;loc.y=p.second;loc.z=0;lc_pose_uomap.emplace(loc,measurement);
    ++loop_corr_counter;passed_dis_atlast_lc=passed_dis;
    auto v=GTSPoseToPose6D(measurement);
    lc_file<<p.first<<' '<<p.second<<' '<<v.x<<' '<<v.y<<' '<<v.z<<' '<<v.roll<<' '<<v.pitch<<' '<<v.yaw<<std::endl;
    pubLoopClosureVisualization(p.first,p.second);pubPath();dump("accepted");recordTiming("pose3_commit",start);
    return true;
}
