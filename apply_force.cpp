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
public:
    ApplyForce(): RFModule()
    {
      
    }
    
    bool configure(ResourceFinder &rf)
    {    
        string name=rf.check("name",Value("apply_force")).asString().c_str();
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
    std::string link="~/iCub/chest/wrench";
    pub=node->Advertise<gazebo::msgs::Wrench>(link);
    gazebo::msgs::Wrench msg;
    gazebo::msgs::Set(msg.mutable_force(),ignition::math::Vector3d(10000,100000,10000));
    gazebo::msgs::Set(msg.mutable_torque(),ignition::math::Vector3d(0,0,0));
//  gazebo::msgs::Set(msg.mutable_force_offset(),ignition::math::Vector3d(0.002,0.075,-0));
    gazebo::msgs::Set(msg.mutable_force_offset(),ignition::math::Vector3d(0.0,0.0,-0));

    pub->WaitForConnection();
    std::cout << "connected !" << std::endl;
    pub->Publish(msg,true);
    gazebo::common::Time::MSleep(100);

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
      
//       yInfo("plop");
//       msgs::Wrench msg;
//       pub->Publish(msg);

        mx.lock();
//         if (selected_part!=NULL)
        {          
        }
        mx.unlock();
        return true;
    }
    
    void cbGzSelection(ConstSelectionPtr &_msg)
    // Gazebo callback, called when a new selection is made
    {     
      // TODO: better name, it can be icub_0::l_forearm or similar if robot is removed and replaced in gazebo...
      // for now we only check if something called *::(l|r)_forearm is selected.
      yInfo(_msg->name());
      
      std::size_t sep=_msg->name().rfind("::");
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
