// NAV-OFFLINE-R4-A — evaluador OFFLINE del controlador DWB real de Nav2 (Humble).
//
// Carga la clase REAL dwb_core::DWBLocalPlanner (libdwb_core.so instalada, mismos plugins de critics)
// y un Costmap2DROS REAL ("local_costmap"), con los mismos parámetros que controller_server.
// Para cada caso (pose, velocidad actual, path NavFn) llama a la MISMA API que usa controller_server:
//     setPlan(path);  computeVelocityCommands(PoseStamped, Twist, goal_checker=nullptr)
// y registra aceptación/rechazo, tiempo, comando seleccionado y scores por critic.
//
// SEGURIDAD: este proceso NO crea publisher de Twist/cmd_vel ni clientes de acciones. El comando
// seleccionado solo se escribe al archivo de resultados. Publica únicamente /tf (odom->base_link del
// caso evaluado) y los tópicos de depuración propios de DWB/costmap. Exige ROS_DOMAIN_ID=99 y
// ROS_LOCALHOST_ONLY=1.

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <map>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "dwb_core/dwb_local_planner.hpp"
#include "dwb_core/exceptions.hpp"
#include "dwb_core/illegal_trajectory_tracker.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav2_core/exceptions.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_costmap_2d/footprint_collision_checker.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/node_thread.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"

using Clock = std::chrono::steady_clock;

// Subclase SOLO para acceder al flag protegido short_circuit_trajectory_evaluation_ (pasada de análisis).
// No sobreescribe ningún método: el algoritmo evaluado es el de la librería instalada.
class DWBProbe : public dwb_core::DWBLocalPlanner
{
public:
  bool shortCircuit() const {return short_circuit_trajectory_evaluation_;}
  void setShortCircuit(bool v) {short_circuit_trajectory_evaluation_ = v;}
};

struct Case
{
  std::string id;
  double x{0}, y{0}, yaw{0}, vx{0}, wz{0};
  int detail{0}, repeats{1};
  std::vector<std::array<double, 3>> path;
};

static std::string jstr(const std::string & s)
{
  std::ostringstream o;
  o << '"';
  for (char c : s) {
    if (c == '"' || c == '\\') {o << '\\' << c;} else if (c == '\n') {o << "\\n";} else if (
      static_cast<unsigned char>(c) < 0x20) {o << ' ';} else {o << c;}
  }
  o << '"';
  return o.str();
}

static std::string jnum(double v)
{
  if (!std::isfinite(v)) {return "null";}
  std::ostringstream o;
  o << std::setprecision(7) << v;
  return o.str();
}

static std::vector<Case> readCases(const std::string & fn)
{
  std::ifstream in(fn);
  std::vector<Case> cases;
  std::string tag;
  while (in >> tag) {
    if (tag == "CASE") {
      Case c;
      in >> c.id >> c.x >> c.y >> c.yaw >> c.vx >> c.wz >> c.detail >> c.repeats;
      cases.push_back(c);
    } else if (tag == "PATH") {
      size_t n;
      in >> n;
      for (size_t i = 0; i < n; ++i) {
        std::array<double, 3> p;
        in >> p[0] >> p[1] >> p[2];
        cases.back().path.push_back(p);
      }
    }
  }
  return cases;
}

static geometry_msgs::msg::Quaternion yawQ(double yaw)
{
  geometry_msgs::msg::Quaternion q;
  q.z = std::sin(yaw / 2);
  q.w = std::cos(yaw / 2);
  return q;
}

int main(int argc, char ** argv)
{
  const char * dom = std::getenv("ROS_DOMAIN_ID");
  const char * loc = std::getenv("ROS_LOCALHOST_ONLY");
  if (!dom || std::string(dom) != "99" || !loc || std::string(loc) != "1") {
    fprintf(stderr, "dwb_offline_eval requiere ROS_DOMAIN_ID=99 y ROS_LOCALHOST_ONLY=1\n");
    return 2;
  }
  rclcpp::init(argc, argv);
  auto node = std::make_shared<nav2_util::LifecycleNode>("dwb_offline_eval", "");
  const std::string cases_file = node->declare_parameter("cases_file", std::string(""));
  const std::string out_file = node->declare_parameter("out_file", std::string(""));
  const std::string plugin_name = node->declare_parameter("plugin_name", std::string("FollowPath"));
  // Espera tras recentrar el costmap rodante (debe ser > 1 período de update del costmap)
  const int settle_ms = node->declare_parameter("settle_ms", 300);
  if (cases_file.empty() || out_file.empty()) {
    RCLCPP_FATAL(node->get_logger(), "faltan parámetros cases_file / out_file");
    return 2;
  }
  auto cases = readCases(cases_file);
  RCLCPP_INFO(node->get_logger(), "casos leídos: %zu", cases.size());

  // Pose actual del robot simulada SOLO vía TF (odom->base_link). map->odom: TF estática del launch.
  std::mutex pose_mtx;
  double cur_x = cases.empty() ? 0.0 : cases[0].x, cur_y = cases.empty() ? 0.0 : cases[0].y;
  double cur_yaw = cases.empty() ? 0.0 : cases[0].yaw;
  auto tfb = std::make_shared<tf2_ros::TransformBroadcaster>(node);
  auto timer = node->create_wall_timer(
    std::chrono::milliseconds(50), [&]() {
      geometry_msgs::msg::TransformStamped t;
      t.header.stamp = node->now();
      t.header.frame_id = "odom";
      t.child_frame_id = "base_link";
      {
        std::lock_guard<std::mutex> l(pose_mtx);
        t.transform.translation.x = cur_x;
        t.transform.translation.y = cur_y;
        t.transform.rotation = yawQ(cur_yaw);
      }
      tfb->sendTransform(t);
    });
  auto node_thread = std::make_unique<nav2_util::NodeThread>(node->get_node_base_interface());

  // Costmap local REAL, instanciado igual que en nav2_controller::ControllerServer (Humble).
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    "local_costmap", std::string{node->get_namespace()}, "local_costmap");
  auto costmap_thread = std::make_unique<nav2_util::NodeThread>(costmap_ros);
  costmap_ros->configure();

  auto dwb = std::make_shared<DWBProbe>();
  dwb->configure(node, plugin_name, costmap_ros->getTfBuffer(), costmap_ros);
  costmap_ros->activate();
  dwb->activate();
  const bool sc_config = dwb->shortCircuit();
  RCLCPP_INFO(
    node->get_logger(), "DWB configurado (%s), short_circuit(config)=%d", plugin_name.c_str(),
    sc_config);

  std::ofstream out(out_file);
  nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *> fcc(
    costmap_ros->getCostmap());
  const auto footprint = costmap_ros->getRobotFootprint();
  const double half_w = costmap_ros->getCostmap()->getSizeInMetersX() / 2.0;
  const double half_h = costmap_ros->getCostmap()->getSizeInMetersY() / 2.0;
  const double res = costmap_ros->getCostmap()->getResolution();

  // cabecera con meta-datos
  {
    std::ostringstream o;
    o << "{\"meta\":{\"plugin\":" << jstr(plugin_name) << ",\"short_circuit_config\":" << sc_config << ",\"settle_ms\":" << settle_ms <<
      ",\"costmap_frame\":" << jstr(costmap_ros->getGlobalFrameID()) << ",\"costmap_res\":" <<
      jnum(res) << ",\"costmap_size_m\":[" << jnum(2 * half_w) << "," << jnum(2 * half_h) <<
      "],\"footprint_padded\":[";
    for (size_t i = 0; i < footprint.size(); ++i) {
      o << (i ? "," : "") << "[" << jnum(footprint[i].x) << "," << jnum(footprint[i].y) << "]";
    }
    o << "],\"n_cases\":" << cases.size() << "}}";
    out << o.str() << "\n";
  }

  size_t k = 0;
  for (auto & c : cases) {
    if (!rclcpp::ok()) {break;}
    ++k;
    {
      std::lock_guard<std::mutex> l(pose_mtx);
      cur_x = c.x; cur_y = c.y; cur_yaw = c.yaw;
    }
    // Esperar a que el costmap rodante se recentre sobre la pose y complete >=1 ciclo de update.
    auto t0 = Clock::now();
    bool ready = false;
    Clock::time_point matched;
    bool have_match = false;
    while (rclcpp::ok() && Clock::now() - t0 < std::chrono::seconds(5)) {
      geometry_msgs::msg::PoseStamped rp;
      auto * cm = costmap_ros->getCostmap();
      bool ok = costmap_ros->getRobotPose(rp) &&
        std::hypot(rp.pose.position.x - c.x, rp.pose.position.y - c.y) < 1e-3 &&
        std::fabs(cm->getOriginX() + half_w - c.x) <= 1.01 * res &&
        std::fabs(cm->getOriginY() + half_h - c.y) <= 1.01 * res;
      if (ok && !have_match) {have_match = true; matched = Clock::now();}
      if (!ok) {have_match = false;}
      if (have_match && Clock::now() - matched > std::chrono::milliseconds(settle_ms)) {ready = true; break;}
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    nav_msgs::msg::Path path;
    path.header.frame_id = "map";
    path.header.stamp = node->now();
    for (auto & p : c.path) {
      geometry_msgs::msg::PoseStamped ps;
      ps.header = path.header;
      ps.pose.position.x = p[0];
      ps.pose.position.y = p[1];
      ps.pose.orientation = yawQ(p[2]);
      path.poses.push_back(ps);
    }
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = costmap_ros->getGlobalFrameID();   // odom (map->odom identidad)
    pose.header.stamp = node->now();
    pose.pose.position.x = c.x;
    pose.pose.position.y = c.y;
    pose.pose.orientation = yawQ(c.yaw);
    geometry_msgs::msg::Twist vel;
    vel.linear.x = c.vx;
    vel.angular.z = c.wz;

    // Contexto del costmap en la pose inicial
    double center_cost = -1, fp_cost = -1;
    {
      std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap_ros->getCostmap()->getMutex()));
      unsigned int mx, my;
      if (costmap_ros->getCostmap()->worldToMap(c.x, c.y, mx, my)) {
        center_cost = costmap_ros->getCostmap()->getCost(mx, my);
      }
      fp_cost = fcc.footprintCostAtPose(c.x, c.y, c.yaw, footprint);
    }

    // ---- PASADA 1: API exacta de controller_server (config real, short_circuit según config) ----
    std::string status = "ACCEPTED", exc_type, exc_msg;
    geometry_msgs::msg::TwistStamped cmd;
    std::vector<double> times_ms;
    for (int r = 0; r < std::max(1, c.repeats); ++r) {
      dwb->setPlan(path);   // resetea critics (Oscillation) y generador, igual que un nuevo FollowPath
      auto a = Clock::now();
      try {
        cmd = dwb->computeVelocityCommands(pose, vel, nullptr);
        if (r == 0) {status = "ACCEPTED";}
      } catch (const dwb_core::NoLegalTrajectoriesException & e) {
        if (r == 0) {status = "REJECTED"; exc_type = "NoLegalTrajectoriesException"; exc_msg = e.what();}
      } catch (const dwb_core::PlannerTFException & e) {
        if (r == 0) {status = "ERROR"; exc_type = "PlannerTFException"; exc_msg = e.what();}
      } catch (const nav2_core::PlannerException & e) {
        if (r == 0) {status = "REJECTED"; exc_type = "PlannerException"; exc_msg = e.what();}
      } catch (const std::exception & e) {
        if (r == 0) {status = "ERROR"; exc_type = "std::exception"; exc_msg = e.what();}
      }
      times_ms.push_back(std::chrono::duration<double, std::milli>(Clock::now() - a).count());
    }

    // ---- PASADA 2 (análisis): misma llamada con short_circuit=false para scores completos ----
    auto res2 = std::make_shared<dwb_msgs::msg::LocalPlanEvaluation>();
    std::string status2 = "ACCEPTED";
    double cmd2_vx = NAN, cmd2_wz = NAN;
    dwb->setPlan(path);
    dwb->setShortCircuit(false);
    try {
      nav_2d_msgs::msg::Pose2DStamped p2;
      p2.header = pose.header;
      p2.pose.x = c.x; p2.pose.y = c.y; p2.pose.theta = c.yaw;
      nav_2d_msgs::msg::Twist2D v2;
      v2.x = c.vx; v2.theta = c.wz;
      auto c2 = dwb->computeVelocityCommands(p2, v2, res2);
      cmd2_vx = c2.velocity.x; cmd2_wz = c2.velocity.theta;
    } catch (const std::exception & e) {
      status2 = "REJECTED";
    }
    dwb->setShortCircuit(sc_config);

    // Análisis de critics sobre todas las trayectorias
    std::map<std::string, int> illegal_by_critic;
    std::vector<std::string> critic_names;
    int n_legal = 0, best = -1;
    double best_total = 1e300, second_total = 1e300;
    for (size_t i = 0; i < res2->twists.size(); ++i) {
      auto & t = res2->twists[i];
      if (t.total < 0) {
        illegal_by_critic[t.scores.empty() ? std::string("?") : t.scores[0].name]++;
        continue;
      }
      ++n_legal;
      if (critic_names.empty()) {for (auto & s : t.scores) {critic_names.push_back(s.name);}}
      if (t.total < best_total) {second_total = best_total; best_total = t.total; best = i;} else if (
        t.total < second_total) {second_total = t.total;}
    }
    // Critic decisivo: si al quitar su contribución cambia la trayectoria elegida.
    std::vector<std::string> decisive;
    std::string dominant;
    double dominant_w = -1;
    if (best >= 0) {
      for (auto & s : res2->twists[best].scores) {
        double w = s.raw_score * s.scale;
        if (w > dominant_w) {dominant_w = w; dominant = s.name;}
      }
      for (size_t ci = 0; ci < critic_names.size(); ++ci) {
        int arg = -1;
        double mn = 1e300;
        for (size_t i = 0; i < res2->twists.size(); ++i) {
          auto & t = res2->twists[i];
          if (t.total < 0 || t.scores.size() <= ci) {continue;}
          double v = t.total - t.scores[ci].raw_score * t.scores[ci].scale;
          if (v < mn - 1e-9) {mn = v; arg = i;}
        }
        if (arg >= 0 && arg != best) {
          auto & a = res2->twists[arg].traj.velocity, &b = res2->twists[best].traj.velocity;
          if (std::fabs(a.x - b.x) > 1e-6 || std::fabs(a.theta - b.theta) > 1e-6) {
            decisive.push_back(critic_names[ci]);
          }
        }
      }
    }
    // Chequeo de footprint COMPLETO (no evaluado por BaseObstacle, que mira solo el centro)
    int legal_fp_collide = 0;
    double best_fp_max = -1;
    {
      std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap_ros->getCostmap()->getMutex()));
      for (size_t i = 0; i < res2->twists.size(); ++i) {
        auto & t = res2->twists[i];
        if (t.total < 0) {continue;}
        double mxc = 0;
        for (auto & p : t.traj.poses) {
          mxc = std::max(mxc, fcc.footprintCostAtPose(p.x, p.y, p.theta, footprint));
        }
        if (mxc >= nav2_costmap_2d::LETHAL_OBSTACLE) {++legal_fp_collide;}
        if (static_cast<int>(i) == best) {best_fp_max = mxc;}
      }
    }

    std::ostringstream o;
    o << "{\"case\":" << jstr(c.id) << ",\"costmap_ready\":" << ready << ",\"pose\":[" << jnum(c.x) <<
      "," << jnum(c.y) << "," << jnum(c.yaw) << "],\"vel_in\":[" << jnum(c.vx) << "," << jnum(c.wz) <<
      "],\"path_len\":" << c.path.size() << ",\"center_cost\":" << jnum(center_cost) <<
      ",\"footprint_cost_at_pose\":" << jnum(fp_cost) << ",\"status\":" << jstr(status) <<
      ",\"exc_type\":" << jstr(exc_type) << ",\"exc_msg\":" << jstr(exc_msg) << ",\"cmd\":[" <<
      jnum(status == "ACCEPTED" ? cmd.twist.linear.x : NAN) << "," <<
      jnum(status == "ACCEPTED" ? cmd.twist.angular.z : NAN) << "],\"times_ms\":[";
    for (size_t i = 0; i < times_ms.size(); ++i) {o << (i ? "," : "") << jnum(times_ms[i]);}
    o << "],\"pass2_status\":" << jstr(status2) << ",\"pass2_cmd\":[" << jnum(cmd2_vx) << "," <<
      jnum(cmd2_wz) << "],\"n_twists\":" << res2->twists.size() << ",\"n_legal\":" << n_legal <<
      ",\"illegal_by_critic\":{";
    bool first = true;
    for (auto & kv : illegal_by_critic) {
      o << (first ? "" : ",") << jstr(kv.first) << ":" << kv.second; first = false;
    }
    o << "},\"best_total\":" << jnum(best >= 0 ? best_total : NAN) << ",\"second_total\":" <<
      jnum(second_total < 1e299 ? second_total : NAN) << ",\"dominant_critic\":" << jstr(dominant) <<
      ",\"decisive_critics\":[";
    for (size_t i = 0; i < decisive.size(); ++i) {o << (i ? "," : "") << jstr(decisive[i]);}
    o << "],\"best_scores\":{";
    if (best >= 0) {
      first = true;
      for (auto & s : res2->twists[best].scores) {
        o << (first ? "" : ",") << jstr(s.name) << ":[" << jnum(s.raw_score) << "," << jnum(s.scale) << "]";
        first = false;
      }
    }
    o << "},\"legal_footprint_collide\":" << legal_fp_collide << ",\"best_footprint_max_cost\":" <<
      jnum(best_fp_max);
    if (best >= 0) {   // poses de la trayectoria elegida (frame del costmap = odom = map)
      o << ",\"best_traj\":[";
      auto & ps = res2->twists[best].traj.poses;
      for (size_t i = 0; i < ps.size(); ++i) {
        o << (i ? "," : "") << "[" << jnum(ps[i].x) << "," << jnum(ps[i].y) << "," << jnum(ps[i].theta) << "]";
      }
      o << "]";
    }
    if (c.detail && best >= 0) {
      o << ",\"twists\":[";
      for (size_t i = 0; i < res2->twists.size(); ++i) {
        auto & t = res2->twists[i];
        o << (i ? "," : "") << "[" << jnum(t.traj.velocity.x) << "," << jnum(t.traj.velocity.theta) <<
          "," << jnum(t.total) << "]";
      }
      o << "]";
    }
    o << "}";
    out << o.str() << "\n";
    out.flush();
    if (k % 25 == 0) {RCLCPP_INFO(node->get_logger(), "%zu/%zu", k, cases.size());}
  }
  out.close();
  dwb->deactivate();
  costmap_ros->deactivate();
  dwb->cleanup();
  costmap_ros->cleanup();
  costmap_thread.reset();
  node_thread.reset();
  rclcpp::shutdown();
  return 0;
}
