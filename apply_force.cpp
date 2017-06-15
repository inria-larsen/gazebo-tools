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
    std::string name;
    gazebo::transport::NodePtr node;
    gazebo::transport::PublisherPtr pub;
    gazebo::transport::SubscriberPtr sub_sel;
    gazebo::transport::SubscriberPtr sub_pose;
    boost::mutex mx;
    std::string current_link;
    std::string current_link_topic;
    ignition::math::Vector3d current_force;
    BufferedPort<Bottle> force_port;
    std::map<std::string,ignition::math::Pose3<double> > current_poses;
    
public:
    ApplyForce(): RFModule(), name(""), current_link(""), current_link_topic(""), current_force(0,0,0)
    {
      
    }
    
    bool configure(ResourceFinder &rf)
    {    
        name=rf.check("name",Value("apply_force")).asString().c_str();
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
        sub_pose=node->Subscribe("~/pose/info",&ApplyForce::cbGzPose,this);
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
      std::string link_name="";
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
        link=current_link_topic;
        link_name=current_link;
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
          ignition::math::Pose3<double> pose,root_pose;
          ignition::math::Quaternion<double> rot,root_rot;
          ignition::math::Vector3d f;
          bool pose_ok=false; // true if a known pose can be used
          mx.lock();
          // first find our iCub pose
          // TODO: check name here, could be a parameter ?
          auto found=current_poses.find("iCub");
          if (found!=current_poses.end())
          {
            root_pose=found->second;
            root_rot=root_pose.Rot();            
            found=current_poses.find(link_name);
            if (found!=current_poses.end())
            {
              pose=found->second;
              rot=root_rot*pose.Rot();
              f=rot.RotateVectorReverse(current_force);            
              pose_ok=true;
            }
          }                    
          mx.unlock();                    
          
          if (pose_ok)
          {
            gazebo::msgs::Set(msg.mutable_force(),f);
            gazebo::msgs::Set(msg.mutable_torque(),ignition::math::Vector3d(0,0,0));
            gazebo::msgs::Set(msg.mutable_force_offset(),ignition::math::Vector3d(0.0,0.0,-0));
            pub->Publish(msg,true);
          }
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
        current_link_topic="";
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
          current_link_topic=current_link;
          boost::replace_all(current_link_topic, "::", "/");
          current_link_topic="~/"+current_link_topic+"/wrench";
          mx.unlock();
          std::cout << "current_link: "<< current_link << std::endl;
        }
        else
        {
          // don't know what to do here: the whole robot is selected, need a link instead...
          mx.lock();
          current_link_topic="";
          current_link="";
          mx.unlock();
        }
      }
    }
    void cbGzPose(ConstPosesStampedPtr &_msg)
    // Gazebo callback, called when a new pose is available
    { 
      // TODO: clear map ?
      mx.lock();
      for (int i=0;i<_msg->pose_size();i++)
      {        
        ignition::math::Vector3d pos( _msg->pose(i).position().x(),
                                      _msg->pose(i).position().y(),
                                      _msg->pose(i).position().z());
        
        ignition::math::Quaternion<double> rot(_msg->pose(i).orientation().w(),
                                               _msg->pose(i).orientation().x(),
                                               _msg->pose(i).orientation().y(),
                                               _msg->pose(i).orientation().z());
        
        current_poses[_msg->pose(i).name()]= ignition::math::Pose3<double>(pos,rot);
      }
      mx.unlock();
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
