#include <algorithm>
#include <memory>
#include <string>
#include <utility>

#include "pf_driver/pf/pf_interface.h"
#include "pf_driver/ros/scan_publisher.h"

bool PFInterface::init()
{
  // This is the first time ROS communicates with the device

  // The scanner might be off during the start up of this node up to 30 seconds.
  // This happens on EAE if the user:
  // - stops the bringup
  // - turns off the key to powercycle the base truck
  // - starts the bringup
  // - turns the key back on some time in the future (can be max 30 seconds, or EAE will power off fully.)
  //
  // It also takes many seconds for the scanner to be accessible after it is powered on.
  const uint max_seconds_to_wait_for_commmunication = 30 + 10;
  ros::Time timestamp_at_which_we_will_give_up =
      ros::Time::now() + ros::Duration(max_seconds_to_wait_for_commmunication);

  auto opi = protocol_interface_->get_protocol_info();
  while (opi.isError && ros::Time::now() < timestamp_at_which_we_will_give_up)
  {
    ROS_WARN_THROTTLE(2, "Unable to communicate with device. Either the IP address is wrong or the scanner is off. "
                         "Waiting for it to come up.");
    ros::Duration(2.0).sleep();
    opi = protocol_interface_->get_protocol_info();
  }
  if (opi.isError)
  {
    ROS_ERROR("Unable to communicate with device. We gave up after re-trying for %d seconds.",
              max_seconds_to_wait_for_commmunication);
    return false;
  }

  //  We  will always expect the scanner to be accessible from now on.
  is_scanner_accessible_ = is_scanner_accessible();
  if (is_scanner_accessible_)
  {
    float scanner_accessible_watchog_period = 2;  // seconds
    scanner_accessible_watchdog_timer_ = nh_.createTimer(ros::Duration(scanner_accessible_watchog_period),
                                                         std::bind(&PFInterface::scanner_accessible_watchdog, this));
  }
  else
  {
    ROS_ERROR("Scanner is not accessible. We received HTTP info from it but it is now not pingable somehow.");
    return false;
  }

  if (opi.protocol_name != "pfsdp")
    return false;
  if (!handle_version(opi.version_major, opi.version_minor))
    return false;
  setup_param_server();

  change_state(PFState::INIT);
  return true;
}

void PFInterface::change_state(PFState state)
{
  if (state_ == state)
    return;
  state_ = state;  // Can use this function later
                   // to check state transitions
  std::string text;
  if (state_ == PFState::UNINIT)
    text = "Uninitialized";
  if (state_ == PFState::INIT)
    text = "Initialized";
  if (state_ == PFState::RUNNING)
    text = "Running";
  if (state_ == PFState::SHUTDOWN)
    text = "Shutdown";
  if (state_ == PFState::ERROR)
    text = "Error";
  ROS_INFO("Device state changed to %s", text.c_str());
}

bool PFInterface::can_change_state(PFState state)
{
  return true;
}

bool PFInterface::handle_version(int major_version, int minor_version)
{
  std::string product_name = "product";
  if (expected_device_ == "R2000")
  {
    protocol_interface_ = std::make_shared<PFSDP_2000>(ip_);
  }
  else if (expected_device_ == "R2300")
  {
    protocol_interface_ = std::make_shared<PFSDP_2300>(ip_);
  }
  product_name = protocol_interface_->get_product();
  if (product_name.find(expected_device_) != std::string::npos)
  {
    ROS_INFO("Device found: %s", product_name.c_str());
    product_ = expected_device_;
    return true;
  }
  ROS_ERROR("Device unsupported");
  return false;
}

void PFInterface::setup_param_server()
{
  if (expected_device_ == "R2000")
  {
    param_server_R2000_ = std::make_unique<dynamic_reconfigure::Server<pf_driver::PFDriverR2000Config>>();
    param_server_R2000_->setCallback(
        boost::bind(&PFInterface::reconfig_callback_r2000, this, boost::placeholders::_1, boost::placeholders::_2));
  }
  else
  {
    param_server_R2300_ = std::make_unique<dynamic_reconfigure::Server<pf_driver::PFDriverR2300Config>>();
    param_server_R2300_->setCallback(
        boost::bind(&PFInterface::reconfig_callback_r2300, this, boost::placeholders::_1, boost::placeholders::_2));
  }
}

bool PFInterface::start_transmission(ScanConfig& config)
{
  if (state_ != PFState::INIT)
    return false;

  if (pipeline_ && pipeline_->is_running())
    return true;

  if (!is_scanner_accessible_)
  {
    ROS_ERROR("Cannot start the transmission because the scanner is not available.");
    return false;
  }

  std::string pkt_type = (expected_device_ == "R2000") ? "C" : "";
  if (transport_type_ == transport_type::tcp)
  {
    info_ = protocol_interface_->request_handle_tcp(port_, pkt_type);
    if (port_.empty())
      port_ = info_.port;
    transport_->set_port(port_);
    transport_->connect();
  }
  else if (transport_type_ == transport_type::udp)
  {
    if (!transport_->connect())
      return false;

    std::string host_ip = transport_->get_host_ip();
    port_ = transport_->get_port();
    info_ = protocol_interface_->request_handle_udp(host_ip, port_, pkt_type);
  }
  if (info_.handle.empty())
    return false;

  config_ = protocol_interface_->get_scanoutput_config(info_.handle);
  config_.start_angle = config.start_angle;
  config_.max_num_points_scan = config.max_num_points_scan;
  params_ = protocol_interface_->get_scan_parameters(config_.start_angle);

  // config_.print();
  // params_.print();

  protocol_interface_->set_scanoutput_config(info_.handle, config_);
  config_ = protocol_interface_->get_scanoutput_config(info_.handle);
  pipeline_ = get_pipeline(config_.packet_type);
  pipeline_->set_scanoutput_config(config_);
  pipeline_->set_scan_params(params_);

  if (!pipeline_->start())
    return false;
  protocol_interface_->start_scanoutput(info_.handle);
  if (config_.watchdog)
    start_watchdog_timer(config_.watchdogtimeout / 1000.0);

  change_state(PFState::RUNNING);
  return true;
}

// What happens to the connection_ obj?
void PFInterface::stop_transmission()
{
  if (state_ != PFState::RUNNING)
    return;

  // Unfortunately this node as it is now cannot join the reader & writer threads properly if the scanner is not
  // accessible. we will just let the process die and let the OS do the cleaning up.
  if (is_scanner_accessible_)
  {
    pipeline_->terminate();
    pipeline_.reset();
    protocol_interface_->stop_scanoutput(info_.handle);
    protocol_interface_->release_handle(info_.handle);
  }
  change_state(PFState::SHUTDOWN);
}

void PFInterface::terminate()
{
  if (!pipeline_)
    return;
  pipeline_->terminate();
  pipeline_.reset();
}

std::unique_ptr<Pipeline<PFPacket>> PFInterface::get_pipeline(std::string packet_type)
{
  std::shared_ptr<Parser<PFPacket>> parser;
  std::shared_ptr<Writer<PFPacket>> writer;
  std::shared_ptr<Reader<PFPacket>> reader;
  if (product_ == "R2000")
  {
    ROS_DEBUG("PacketType is: %s", packet_type.c_str());
    if (packet_type == "A")
    {
      parser = std::unique_ptr<Parser<PFPacket>>(new PFR2000_A_Parser);
    }
    else if (packet_type == "B")
    {
      parser = std::unique_ptr<Parser<PFPacket>>(new PFR2000_B_Parser);
    }
    else if (packet_type == "C")
    {
      parser = std::unique_ptr<Parser<PFPacket>>(new PFR2000_C_Parser);
    }
    reader = std::shared_ptr<Reader<PFPacket>>(new ScanPublisherR2000("/scan", "scanner"));
  }
  else if (product_ == "R2300")
  {
    if (packet_type == "C1")
    {
      parser = std::unique_ptr<Parser<PFPacket>>(new PFR2300_C1_Parser);
    }
    reader = std::shared_ptr<Reader<PFPacket>>(new ScanPublisherR2300("/cloud", "scanner"));
  }
  writer = std::shared_ptr<Writer<PFPacket>>(new PFWriter<PFPacket>(std::move(transport_), parser));
  return std::unique_ptr<Pipeline<PFPacket>>(
      new Pipeline<PFPacket>(writer, reader, std::bind(&PFInterface::on_shutdown, this)));
}

void PFInterface::start_watchdog_timer(float duration)
{
  // dividing the watchdogtimeout by 2 to have a “safe” feed time within the defined timeout
  float feed_time = std::min(duration, 60.0f) / 2.0f;
  feed_watchdog_timer_ =
      nh_.createTimer(ros::Duration(feed_time), std::bind(&PFInterface::feed_watchdog, this, std::placeholders::_1));
}

void PFInterface::feed_watchdog(const ros::TimerEvent& e)
{
  protocol_interface_->feed_watchdog(info_.handle);
}

void PFInterface::on_shutdown()
{
  ROS_INFO("Shutting down pipeline!");
  stop_transmission();
}

void PFInterface::reconfig_callback_r2000(pf_driver::PFDriverR2000Config& config, uint32_t level)
{
  bool watchdog = false;
  uint watchdogtimeout = 0;
  std::string packet_type = "";
  int start_angle = 0;
  uint max_num_points_scan = 0;
  uint skip_scans = 0;
  if (product_ != "R2000")
    return;
  if (state_ != PFState::RUNNING)
    return;
  // Do we want to change the address and port at run-time?
  // if(level == 16){
  //   set_parameter({ KV("address", config.address) });
  // } else if(level == 17)
  // {
  //   set_parameter({ KV("port", config.port) });
  // }
  if (level == 18)
  {
    config_.packet_type = config.packet_type;
  }
  else if (level == 19)
  {
    // this param doesn't exist for R2000
    // set_parameter({ KV("packet_crc", config.packet_crc) });
  }
  else if (level == 20)
  {
    config_.watchdog = (config.watchdog == "on") ? true : false;
  }
  else if (level == 21)
  {
    config_.watchdogtimeout = config.watchdogtimeout;
  }
  else if (level == 22)
  {
    config_.start_angle = config.start_angle;
  }
  else if (level == 23)
  {
    config_.max_num_points_scan = config.max_num_points_scan;
  }
  else if (level == 24)
  {
    config_.skip_scans = config.skip_scans;
  }
  else
  {
    protocol_interface_->handle_reconfig(config, level);
  }
  pipeline_->set_scanoutput_config(config_);
  protocol_interface_->set_scanoutput_config(info_.handle, config_);
  params_ = protocol_interface_->get_scan_parameters(config_.start_angle);
  pipeline_->set_scan_params(params_);
}

void PFInterface::reconfig_callback_r2300(pf_driver::PFDriverR2300Config& config, uint32_t level)
{
  if (product_ != "R2300")
    return;
  if (state_ != PFState::RUNNING)
    return;
  // Do we want to change the address and port at run-time?
  // if(level == 16){
  //   set_parameter({ KV("address", config.address) });
  // } else if(level == 17)
  // {
  //   set_parameter({ KV("port", config.port) });
  // }
  if (level == 18)
  {
    config_.packet_type = config.packet_type;
  }
  else if (level == 19)
  {
    // currently always none for R2300
    // config_.packet_crc = config.packet_crc;
  }
  else if (level == 20)
  {
    config_.watchdog = (config.watchdog == "on") ? true : false;
  }
  else if (level == 21)
  {
    config_.watchdogtimeout = config.watchdogtimeout;
  }
  else if (level == 22)
  {
    config_.start_angle = config.start_angle;
  }
  else if (level == 23)
  {
    config_.max_num_points_scan = config.max_num_points_scan;
  }
  else if (level == 24)
  {
    config_.skip_scans = config.skip_scans;
  }
  else
  {
    protocol_interface_->handle_reconfig(config, level);
  }
  protocol_interface_->set_scanoutput_config(info_.handle, config_);
  config_ = protocol_interface_->get_scanoutput_config(info_.handle);
  pipeline_->set_scanoutput_config(config_);
  params_ = protocol_interface_->get_scan_parameters(config_.start_angle);
  pipeline_->set_scan_params(params_);
}

bool PFInterface::is_scanner_accessible() const
{
  // This is extremely ugly and hacky, but is the most bulletproof way
  // to check if we have access to the scanner.
  // This is a ping command with 1 packet and 1 second timeout,
  // stdout and stderr are piped to null.
  std::string command = "ping -c 1 -W 1 " + ip_ + " > /dev/null 2>&1";
  int result = std::system(command.c_str());
  bool is_accessible =  result == 0;

  if (!is_accessible)
  {
    ROS_ERROR("Scanner just became inaccessible, calling ros::shutdown to stop this node");
    ros::shutdown();
  }

  return is_accessible;
}

void PFInterface::scanner_accessible_watchdog()
{
  is_scanner_accessible_ = is_scanner_accessible();
  if (!is_scanner_accessible_)
  {
    ROS_ERROR("Scanner is not accessible.");
  }
}