/* 
 * This example shows how to interact with gazebo (eg. using the 'selection' message)
 */

#include <cmath>
#include <string>
#include <algorithm>
#include <map>

#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>

#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo_client.hh>

#include <boost/thread/mutex.hpp>
#include <boost/algorithm/string/replace.hpp>

#define DEG2RAD     (M_PI/180.0)
#define RAD2DEG     (180.0/M_PI)

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;

class ApplyForce: public RFModule
{
protected:
    gazebo::transport::NodePtr node;
    gazebo::transport::PublisherPtr pub;
    gazebo::transport::SubscriberPtr sub;
    boost::mutex mx;
    std::string current_link;
    ignition::math::Vector3d current_force;
    BufferedPort<Bottle> force_port;

public:
    ApplyForce(): RFModule(), current_link(""), current_force(0,0,0)
    {
      
    }
    
    bool configure(ResourceFinder &rf)
    {    
        string name=rf.check("name",Value("apply_force")).asString().c_str();
        force_port.open("/portForces:i");
        return true;
    }
    
    bool init_gazebo()
    {
        yInfo("Trying to connect to gazebo...");
    
        gazebo::client::setup();
        node=gazebo::transport::NodePtr(new gazebo::transport::Node());
        node->Init();
        // Subscribe to selection to know whre to apply force
        sub=node->Subscribe("~/selection",&ApplyForce::cbGzSelection,this);
//         std::string link="~/iCub/chest/wrench";
//         pub=node->Advertise<gazebo::msgs::Wrench>(link);
//         gazebo::msgs::Wrench msg;
//         gazebo::msgs::Set(msg.mutable_force(),ignition::math::Vector3d(10000,100000,10000));
//         gazebo::msgs::Set(msg.mutable_torque(),ignition::math::Vector3d(0,0,0));
    //  gazebo::msgs::Set(msg.mutable_force_offset(),ignition::math::Vector3d(0.002,0.075,-0));
//         gazebo::msgs::Set(msg.mutable_force_offset(),ignition::math::Vector3d(0.0,0.0,-0));

//         pub->WaitForConnection();
//         std::cout << "connected !" << std::endl;
//         pub->Publish(msg,true);
//         gazebo::common::Time::MSleep(100);
        return true;
    }
    
    bool close()
    {
      return true;
    }

    double getPeriod()
    {	
        return 0.2;
    }

    bool updateModule()
    {         
      std::string link="";
      static std::string old_link="";
      gazebo::msgs::Wrench msg;
      Bottle *b = force_port.read(false);
      if (b!=NULL) {
        // TODO use  vector,
//         // TODO check size and type
          // data received in *b
        double x=b->get(0).asDouble();
        double y=b->get(1).asDouble();
        double z=b->get(2).asDouble();
        current_force=ignition::math::Vector3d(x,y,z);
        yInfo("New force %f %f %f",x,y,z);
      }      
        mx.lock();
        link=current_link;
        mx.unlock();
        if ((link!="") && (link!=old_link))
        {
          // create new pub if new link and not empty link
//           td::string link="~/iCub/chest/wrench";
          pub=node->Advertise<gazebo::msgs::Wrench>(link);
          
          pub->WaitForConnection();
          yInfo("new link connected !");
          old_link=link;
        }
        
        if (old_link!="")
        {
          gazebo::msgs::Set(msg.mutable_force(),current_force);
          gazebo::msgs::Set(msg.mutable_torque(),ignition::math::Vector3d(0,0,0));
          gazebo::msgs::Set(msg.mutable_force_offset(),ignition::math::Vector3d(0.0,0.0,-0));
          pub->Publish(msg,true);
        }
        
        return true;
    }
    
    void cbGzSelection(ConstSelectionPtr &_msg)
    // Gazebo callback, called when a new selection is made
    {     
      
      if (_msg->selected()==false)
      {
        mx.lock();
        current_link="";
        mx.unlock();
        return;
      }
      else
      {  
        std::size_t sep=_msg->name().rfind("::");
      
        if (sep!=string::npos)
        {
          mx.lock();
          current_link=_msg->name();
          boost::replace_all(current_link, "::", "/");
          current_link="~/"+current_link+"/wrench";
          mx.unlock();
          std::cout << "current_link: "<< current_link << std::endl;
        }
        else
        {
          // don't know what to do here: the whole robot is selected, need a link instead...
          mx.lock();
          current_link="";
          mx.unlock();
        }
      }
      // TODO: better name, it can be icub_0::l_forearm or similar if robot is removed and replaced in gazebo...
      // for now we only check if something called *::(l|r)_forearm is selected.
//       yInfo(_msg->name());
      
//       std::size_t sep=_msg->name().rfind("::");
      /*
      if (sep!=string::npos)
      {              
      
        if (_msg->selected() && (!_msg->name().compare(sep+2,string::npos,"l_forearm")))
        {
            mx.lock();
            selected_part=&client_left;
            mx.unlock();
            yInfo("now moving %s",_msg->name().c_str());
        }
        else if (_msg->selected() && (!_msg->name().compare(sep+2,string::npos,"r_forearm")))
        {
            mx.lock();
            selected_part=&client_right;
            mx.unlock();
            yInfo("now moving %s",_msg->name().c_str());
        }
        else
        {
          mx.lock();
          selected_part=nullptr;
          mx.unlock();
        }
      }*/
    }
};

int main(int argc,char *argv[])
{           
    Network yarp;
    if (!yarp.checkNetwork())
    {
        yError("Can not find yarp Network: is yarpsever running happily ?");
        return 1;
    }

    ResourceFinder rf;
    rf.configure(argc,argv);

    ApplyForce af;
    af.init_gazebo();
    yInfo("OK, will now run YARP module");
    int r=af.runModule(rf);  
    gazebo::client::shutdown();
    return r;
}
