
#include "kinect2_impl.h"

int main(int argc, char* argv[])
{
	std::string dummy;


	// Register Local Transport
	boost::shared_ptr<RobotRaconteur::LocalTransport> t1 = boost::make_shared<RobotRaconteur::LocalTransport>();
	t1->StartServerAsNodeName("edu.rpi.cats.sensors.kinect2_tracker");
	RobotRaconteur::RobotRaconteurNode::s()->RegisterTransport(t1);

	// Register TCP Transport on port 8888
	boost::shared_ptr<RobotRaconteur::TcpTransport> t = boost::make_shared<RobotRaconteur::TcpTransport>();
	t->StartServer(8899);
	t->EnableNodeAnnounce(RobotRaconteur::IPNodeDiscoveryFlags_LINK_LOCAL |
		RobotRaconteur::IPNodeDiscoveryFlags_NODE_LOCAL |
		RobotRaconteur::IPNodeDiscoveryFlags_SITE_LOCAL);
	RobotRaconteur::RobotRaconteurNode::s()->RegisterTransport(t);

	// Create the Kinect object
	boost::shared_ptr<Kinect2_impl> k = boost::make_shared<Kinect2_impl>();

	k->EnableSensors();

	// Register the service type with Robot Raconteur
	RobotRaconteur::RobotRaconteurNode::s()->RegisterServiceType(boost::make_shared<edu::rpi::cats::sensors::camera_interface::edu__rpi__cats__sensors__camera_interfaceFactory>());
	RobotRaconteur::RobotRaconteurNode::s()->RegisterServiceType(boost::make_shared<edu::rpi::cats::sensors::kinect2_tracker::edu__rpi__cats__sensors__kinect2_trackerFactory>());

	// Register the Kinect object as a service
	RobotRaconteur::RobotRaconteurNode::s()->RegisterService("HandTracker", "edu.rpi.cats.sensors.kinect2_tracker", k);

	std::cout << "Connect to the Kinect v2 Service at: " << std::endl;
	std::cout << "tcp://localhost:8899/edu.rpi.cats.sensors.kinect2_tracker/HandTracker" << std::endl;
	std::cout << "Press enter to finish" << std::endl;
	std::getline(std::cin, dummy);
	k->ShutdownKinect();

	std::getline(std::cin, dummy);
	return 0;
}