#pragma once
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <deque>
#include <fstream>
#include <functional>
#include <iomanip>
#include <limits>
#include <map>
#include <sstream>
#include <stdexcept>
#include <sys/resource.h>
#include <unistd.h>

// Single-owner application queues. Payload estimates exclude DDS, allocator
// overhead and persistent maps. A rejected push leaves the queue unchanged.
namespace ltaom_budget {
using Clock=std::chrono::steady_clock;
struct Overflow:std::runtime_error {using std::runtime_error::runtime_error;};
inline std::string fatal,output;
inline std::map<std::string,std::function<std::string()>> queues;
inline std::map<std::string,uint64_t> lost_messages;
inline std::map<std::string,std::string> actual_qos;
inline uint64_t publish_calls=0;
inline double max_publish_ms=0;
inline std::function<std::string()> extra=[](){return std::string();};
inline Clock::time_point last_write{};
inline double milliseconds(Clock::duration d){return std::chrono::duration<double,std::milli>(d).count();}
inline std::string quote(const std::string& s){std::ostringstream o;o<<'"';for(unsigned char c:s){if(c=='"'||c=='\\')o<<'\\'<<char(c);else if(c<32)o<<"\\u"<<std::hex<<std::setw(4)<<std::setfill('0')<<int(c);else o<<char(c);}o<<'"';return o.str();}
inline void write(bool force=false){
 if(output.empty())return;auto now=Clock::now();if(!force&&milliseconds(now-last_write)<200)return;last_write=now;
 long pages=0,resident=0;std::ifstream("/proc/self/statm")>>pages>>resident;struct rusage usage{};getrusage(RUSAGE_SELF,&usage);
 std::ofstream f(output+".tmp");f<<std::setprecision(17)<<"{\"schema\":1,\"updated_unix\":"<<std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count()
  <<",\"fatal\":"<<quote(fatal)<<",\"rss_bytes\":"<<resident*sysconf(_SC_PAGESIZE)<<",\"peak_rss_bytes\":"<<usage.ru_maxrss*1024
  <<",\"publish_calls\":"<<publish_calls<<",\"max_publish_ms\":"<<max_publish_ms<<",\"dds_message_lost_events\":{";
 bool first=true;for(const auto& e:lost_messages){f<<(first?"":",")<<quote(e.first)<<":"<<e.second;first=false;}
 f<<"},\"actual_qos\":{";first=true;for(const auto& e:actual_qos){f<<(first?"":",")<<quote(e.first)<<":"<<e.second;first=false;}
 f<<"},\"queues\":{";first=true;for(const auto& q:queues){f<<(first?"":",")<<quote(q.first)<<":"<<q.second();first=false;}
 f<<"}"<<extra()<<"}\n";f.close();if(!f)throw std::runtime_error("queue telemetry write failed");std::rename((output+".tmp").c_str(),output.c_str());
}
inline void fail(const std::string& reason){fatal=reason;write(true);throw Overflow(reason);}
inline int64_t parameter(const rclcpp::Node::SharedPtr& node,const std::string& key,int64_t fallback){
 if(!node->has_parameter(key))node->declare_parameter<int64_t>(key,fallback);
 auto value=node->get_parameter(key).as_int();if(value<=0)throw std::runtime_error(key+" must be positive");return value;
}
template<class T>class Queue {
 struct Entry{T value;size_t charge;Clock::time_point arrival;};
 std::deque<Entry> data;std::string name;std::function<size_t(const T&)> bytes;
 size_t max_count=1,max_bytes=1,current_bytes=0,peak_count=0,peak_bytes=0;
 uint64_t pushed=0,popped=0,cleared=0,overflow=0;double max_wait_ms=0;
public:
 Queue(std::string label,std::function<size_t(const T&)> cost):name(std::move(label)),bytes(std::move(cost)){
  queues.emplace(name,[this](){return json();});
 }
 void configure(const rclcpp::Node::SharedPtr& node,size_t count,size_t limit){
  max_count=parameter(node,"buffers."+name+".max_count",count);max_bytes=parameter(node,"buffers."+name+".max_bytes",limit);
 }
 void push_back(const T& value){
  size_t n=bytes(value);if(data.size()>=max_count||n>max_bytes-current_bytes){++overflow;fail(name+" queue capacity exceeded: count="+std::to_string(data.size())+", bytes="+std::to_string(current_bytes)+", incoming="+std::to_string(n));}
  data.push_back({value,n,Clock::now()});current_bytes+=n;++pushed;peak_count=std::max(peak_count,data.size());peak_bytes=std::max(peak_bytes,current_bytes);write();
 }
 void pop_front(){if(data.empty())throw std::logic_error("pop empty "+name);max_wait_ms=std::max(max_wait_ms,milliseconds(Clock::now()-data.front().arrival));current_bytes-=data.front().charge;data.pop_front();++popped;}
 void clear(){cleared+=data.size();data.clear();current_bytes=0;}
 T& front(){return data.front().value;}const T& front()const{return data.front().value;}
 T& back(){return data.back().value;}const T& back()const{return data.back().value;}
 bool empty()const{return data.empty();}size_t size()const{return data.size();}
 std::string json()const{
  auto wait=data.empty()?0:milliseconds(Clock::now()-data.front().arrival);std::ostringstream o;
  o<<"{\"count\":"<<data.size()<<",\"bytes\":"<<current_bytes<<",\"peak_count\":"<<peak_count<<",\"peak_bytes\":"<<peak_bytes
   <<",\"max_count\":"<<max_count<<",\"max_bytes\":"<<max_bytes<<",\"pushed\":"<<pushed<<",\"popped\":"<<popped<<",\"cleared\":"<<cleared
   <<",\"overflow\":"<<overflow<<",\"oldest_wait_ms\":"<<wait<<",\"max_wait_ms\":"<<std::max(wait,max_wait_ms)
   <<",\"high\":"<<((data.size()>=.75*max_count||current_bytes>=.75*max_bytes)?"true":"false")<<"}";return o.str();
 }
};
}
