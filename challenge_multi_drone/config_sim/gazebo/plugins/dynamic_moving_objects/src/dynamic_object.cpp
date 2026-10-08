#include "dynamic_object.hpp"
// [fix8] logging macros (gzerr/gzmsg) live in gz-common
#include <gz/common/Console.hh>

#include <gz/plugin/Register.hh>
// [fix7] Harmonic: the loader only accepts the GzPluginHook symbol, so the
// deprecated IGNITION_ADD_PLUGIN macro (IgnitionPluginHook) never loads.
GZ_ADD_PLUGIN(
    gzplugin::DynamicMovingObjects,
    gz::sim::System,
	gzplugin::DynamicMovingObjects::ISystemConfigure, //)
	gzplugin::DynamicMovingObjects::ISystemPreUpdate)

// Optional alias for the plugin
GZ_ADD_PLUGIN_ALIAS(gzplugin::DynamicMovingObjects, "gzplugin::DynamicMovingObjects")


#define PI 3.14159265358979323846

// using namespace gz;
namespace gzplugin
{
	
// Constructor
DynamicMovingObjects::DynamicMovingObjects(){}

// Destructor
DynamicMovingObjects::~DynamicMovingObjects() {}

// void DynamicMovingObjects::Load(gz::sim::EntityComponentManager &_ecm, sdf::ElementPtr _sdf) {
  // Implement Configure callback, provided by ISystemConfigure
  // and called once at startup.
void DynamicMovingObjects::Configure(const gz::sim::Entity &_entity,
				const std::shared_ptr<const sdf::Element> &_sdf,
				gz::sim::EntityComponentManager &_ecm,
				gz::sim::EventManager &_eventMgr)
{

	// // Create a GazeboRos node instead of a common ROS node.
	// // Pass it SDF parameters so common options like namespace and remapping
	// // can be handled. 
	// Not needed anymore as ROS not integrated as closely
	// ros_node_ = gazebo_ros::Node::Get(_sdf);

	this->ecm = &_ecm;
	this->em = &_eventMgr;

	// // Obtain the model entity (model pointer is no longer passed directly)
    // gz::sim::Entity modelEntity = gz::sim::Model(_sdf->GetParent());
    this->model = gz::sim::Model(_entity); //std::make_shared<gz::physics::Model>(modelEntity);
	if(!this->model.Valid(_ecm))
	{
		gzerr << "Dynamic Object should be attached to a model entity, failed to initialise" << std::endl;
		return;
	}

	// World is initialised in the postupdate

	// // // Get parameters specified in the sdf file.
	if (_sdf->HasElement("topic")) {
		this->topic = _sdf->Get<std::string>("topic");
	}
	if (_sdf->HasElement("topic_hz")) {
		this->topic_hz = _sdf->Get<double>("topic_hz");
	}

	// [dsh20] 可选的 set_pose 限频参数（不写就用默认 50Hz）
	if (_sdf->HasElement("move_hz")) {
		this->move_hz = _sdf->Get<double>("move_hz");
	}
	if (this->move_hz < 1.0) {
		this->move_hz = 1.0;
	}
	gzmsg << "[dsh20] set_pose service rate = " << this->move_hz
	      << " Hz (sim time)" << std::endl;

	if (_sdf->HasElement("boundary_min_x")) {
		this->boundary_min_x = _sdf->Get<double>("boundary_min_x");
	}
	if (_sdf->HasElement("boundary_min_y")) {
		this->boundary_min_y = _sdf->Get<double>("boundary_min_y");
	}
	if (_sdf->HasElement("boundary_max_x")) {
		this->boundary_max_x = _sdf->Get<double>("boundary_max_x");
	}
	if (_sdf->HasElement("boundary_max_y")) {
		this->boundary_max_y = _sdf->Get<double>("boundary_max_y");
	}

	if (_sdf->HasElement("obj_size_x")) {
		this->obj_size_x = _sdf->Get<float>("obj_size_x");
	}
	if (_sdf->HasElement("obj_size_y")) {
		this->obj_size_y = _sdf->Get<float>("obj_size_y");
	}
	if (_sdf->HasElement("obj_size_z")) {
		this->obj_size_z = _sdf->Get<float>("obj_size_z");
	}
	
	if (_sdf->HasElement("movement_velocity")) {
		this->movement_velocity = _sdf->Get<double>("movement_velocity");
	}

	if (_sdf->HasElement("initial_angle")) {
		this->initial_angle = _sdf->Get<double>("initial_angle");
	}

	this->velocity_direction = this->movement_velocity * gz::math::Vector3d(cos(this->initial_angle), sin(this->initial_angle), 0);  // Initial movement along X-axis
	
	// Create Publisher and Subscriber for ROS2 to interact with this
	this->location_pub = node.Advertise<gz::msgs::Pose>(this->topic);
	if (!this->location_pub)
	{
		gzerr << "Error advertising topic [" << this->topic << "]" << std::endl;
		return;
	} 

	gzerr << "Publisher initialised on " << this->topic<< std::endl;
}

void DynamicMovingObjects::PreUpdate(const gz::sim::UpdateInfo &_info, gz::sim::EntityComponentManager &_ecm)
{
	if (!this->_configured) {
		this->world = gz::sim::World(gz::sim::worldEntity(_ecm));
		// Create Subscriber to the visual_config with world
		std::string world_name = *this->world.Name(_ecm);
		this->pose_service_name = "/world/"+world_name+"/set_pose";
		
		this->_configured = true;
	}

	if (_info.paused)
        return;

	auto poseComp = _ecm.Component<gz::sim::components::Pose>(this->model.Entity());
	if (!poseComp)
	{
		gzerr << "Pose component missing from model." << std::endl;
		return;
	}

	gz::math::Pose3d pose = poseComp->Data();

	// Compute time step (convert nanoseconds to seconds)
	double dt = std::chrono::duration<double>(_info.dt).count();

	// Compute new position
	// [dsh20] 服务调用限频之后，ECS 里的 pose 只会以 move_hz 更新；若还像原来那样
	// 每步从 ECS 读回再 +v*dt，就会变成"每次服务调用才走一步" -> 柱子慢 5 倍。
	// 所以位置改由本插件逐步积分（与原来逐字等价：ECS 里的 pose 本来就是上一步写进去的）。
	if (!this->sim_position_init) {
		this->sim_position = pose.Pos();
		this->sim_position_init = true;
	}
	if ((this->sim_position - pose.Pos()).Length() > 0.5) {
		// 被外部 teleport 过（例如用户手动 gz service set_pose）-> 重新对齐
		this->sim_position = pose.Pos();
	}
	gz::math::Vector3d new_position = this->sim_position + this->velocity_direction * dt;

	// Check boundary conditions and reverse direction if needed
	if (new_position.X() < boundary_min_x || new_position.X() > boundary_max_x)
	{
	this->velocity_direction.X(-this->velocity_direction.X());
	new_position.X(std::clamp(new_position.X(), boundary_min_x, boundary_max_x));
	}
	if (new_position.Y() < boundary_min_y || new_position.Y() > boundary_max_y)
	{
	this->velocity_direction.Y(-this->velocity_direction.Y());
	new_position.Y(std::clamp(new_position.Y(), boundary_min_y, boundary_max_y));
	}

	// Apply new position
	// pose.SetX(new_position.X());
	// pose.SetY(new_position.Y());

	// Set updated pose component
	// _ecm.SetComponentData<gz::sim::components::Pose>(this->model.Entity(), pose);

	// Call service
	gz::msgs::Pose msg;
	msg.set_id(this->model.Entity());
	gz::msgs::Header *header = msg.mutable_header();
	header->mutable_stamp()->set_sec(_info.simTime.count() / 1000000000);  // Convert nanoseconds to seconds
	header->mutable_stamp()->set_nsec(_info.simTime.count() % 1000000000);
	auto frame = header->add_data();
	frame->set_key("frame_id");
	frame->add_value("object_" + std::to_string(this->model.Entity()));
	msg.set_name("object_" + std::to_string(this->model.Entity()));
	gz::msgs::Vector3d *vec = msg.mutable_position();
	vec->set_x(new_position.X());
	vec->set_y(new_position.Y());
	// [fix9] keep z: the old code left position.z at its default 0, so every
	// set_pose call yanked the obstacle down to z=0 (half buried) and the
	// formation could never meet it.
	vec->set_z(new_position.Z());

	// [fix9] preserve the orientation instead of resetting it to identity
	gz::msgs::Quaternion *rot = msg.mutable_orientation();
	rot->set_x(pose.Rot().X());
	rot->set_y(pose.Rot().Y());
	rot->set_z(pose.Rot().Z());
	rot->set_w(pose.Rot().W());
	
	std::function<void(const gz::msgs::Boolean &, const bool)> cb =
	[](const gz::msgs::Boolean &/*_rep*/, const bool _result)
	{
		if (!_result)
		gzerr << "Error setting pose configuration" << std::endl;
	};

	// [dsh20] 按**仿真时间**限频调用 set_pose。原来这一行每个物理步都执行，
	// 是 gz server 单核跑满、real_time_factor ~0.5 的主要嫌疑（见文件头注释）。
	const auto move_period =
		std::chrono::duration_cast<std::chrono::steady_clock::duration>(
			std::chrono::duration<double>(1.0 / this->move_hz));
	if ((_info.simTime - this->last_move_time) >= move_period) {
		this->node.Request(this->pose_service_name, msg, cb);
		this->last_move_time = _info.simTime;
	}
	this->sim_position = new_position;

	// [fix9] Publish gate on SIMULATION time.
	// The old `current_time += dt` never accumulated correctly here and the
	// plugin published on nearly every step (measured 32365 msgs / 5 s).
	const auto pub_period =
		std::chrono::duration_cast<std::chrono::steady_clock::duration>(
			std::chrono::duration<double>(1.0 / this->topic_hz));
	if ((_info.simTime - this->last_publish_time) >= pub_period)
	{
		this->location_pub.Publish(msg);
		this->last_publish_time = _info.simTime;
	}

	
} // end of PreUpdate()

} // namespace gazebo
