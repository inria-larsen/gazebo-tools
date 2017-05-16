/* 
 * This example shows how to interact with gazebo (eg. using the 'selection' message)
 * A force (a bottle containing 3 double) is expected on /portForces:i and then sent
 * continuously to the selected link in gazebo.
 */
#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>

#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo_client.hh>

#include <boost/thread/mutex.hpp>
#include <boost/algorithm/string/replace.hpp>

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
    gazebo::transport::SubscriberPtr sub_sel;
    gazebo::transport::SubscriberPtr sub_pose;
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
        sub_sel=node->Subscribe("~/selection",&ApplyForce::cbGzSelection,this);
        // Subscribe to pose (TODO: or pose local ???)
        sub_pose=node->Subscribe("~/pose",&ApplyForce::cbGzPose,this);
        return true;
    }
    
    bool close()
    {
      return true;
    }

    double getPeriod()
    {	
        return 0.1;
    }

    bool updateModule()
    {         
      std::string link="";
      static std::string old_link="";
      gazebo::msgs::Wrench msg;
      Bottle *b = force_port.read(false);
      if (b!=NULL) {
        // TODO use  vector,
        // TODO check size and type
      
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
          // TODO clean old pub ! 
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
    }
    void cbGzPose(ConstPosePtr &_msg)
    // Gazebo callback, called when a new pose is available
    {     
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
