// NAV-OFFLINE-R4-B — simulador cinemático MÍNIMO en lazo cerrado con el DWB REAL de Nav2 (Humble).
//
// Lazo (tiempo SIMULADO; ROS solo para parámetros y /map):
//   control 10 Hz (controller_frequency): pose -> dwb_core::DWBLocalPlanner::computeVelocityCommands (API de
//   controller_server) -> lógica de controller_server replicada (failure_tolerance, SimpleGoalChecker,
//   SimpleProgressChecker) -> cola de latencia -> base FOUR_DIFF (ganancia de tracking, saturación, rampas acc/decel,
//   timeout de comando) integrada a 100 Hz.
// Localización: odom = pose VERDADERA (odometría perfecta); el error de localización entra como map->odom ruidoso
//   (Gauss-Markov). El plan (frame "map") se transforma con ese error; el mapa estático del local costmap se publica en
//   "map_true" (odom->map_true = identidad; árbol map->odom->{base_link,map_true}), como proxy de un LiDAR que ve las paredes en su posición relativa real.
// TF: se escribe directamente en el tf2 Buffer del costmap (setTransform). Costmap: loop propio desactivado
//   (update_frequency 0) y updateMap() síncrono en cada paso.
//
// SEGURIDAD: sin publisher Twist/cmd_vel, sin acciones. Los comandos solo existen en memoria y en el archivo de
// resultados. Exige ROS_DOMAIN_ID=99 y ROS_LOCALHOST_ONLY=1.

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <deque>
#include <fstream>
#include <iomanip>
#include <map>
#include <memory>
#include <random>
#include <set>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "dwb_core/dwb_local_planner.hpp"
#include "dwb_core/exceptions.hpp"
#include "dwb_msgs/msg/local_plan_evaluation.hpp"
#include "nav_2d_utils/conversions.hpp"
#include "nav2_core/exceptions.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_navfn_planner/navfn_planner.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/node_thread.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"

using Clock = std::chrono::steady_clock;

struct Run
{
  std::string id;
  double sx{0}, sy{0}, syaw{0}, gx{0}, gy{0}, gyaw{0};
  unsigned seed{0};
  std::vector<std::array<double, 3>> path;
};

static double wrap(double a) {return std::atan2(std::sin(a), std::cos(a));}

static geometry_msgs::msg::Quaternion yawQ(double yaw)
{
  geometry_msgs::msg::Quaternion q;
  q.z = std::sin(yaw / 2);
  q.w = std::cos(yaw / 2);
  return q;
}

static std::string jnum(double v)
{
  if (!std::isfinite(v)) {return "null";}
  std::ostringstream o;
  o << std::setprecision(6) << v;
  return o.str();
}

static std::vector<Run> readRuns(const std::string & fn)
{
  std::ifstream in(fn);
  std::vector<Run> runs;
  std::string tag;
  while (in >> tag) {
    if (tag == "RUN") {
      Run r;
      in >> r.id >> r.sx >> r.sy >> r.syaw >> r.gx >> r.gy >> r.gyaw >> r.seed;
      runs.push_back(r);
    } else if (tag == "PATH") {
      size_t n;
      in >> n;
      for (size_t i = 0; i < n; ++i) {
        std::array<double, 3> p;
        in >> p[0] >> p[1] >> p[2];
        runs.back().path.push_back(p);
      }
    }
  }
  return runs;
}

// rampa con límites de aceleración/desaceleración (decel_lim negativo en la config)
static double ramp(double cur, double tgt, double acc, double decel, double dt)
{
  const bool speeding_up = std::fabs(tgt) > std::fabs(cur) && (cur * tgt >= 0);
  const double lim = (speeding_up ? acc : std::fabs(decel)) * dt;
  return cur + std::clamp(tgt - cur, -lim, lim);
}

// NAV-OFFLINE-R4-G — serialización de la evaluación DWB de un ciclo (solo con debug_eval_runs).
// Por ciclo: [t, pc_bx, pc_by, pc_bt, best_index, n_twists, {critic ilegal: n}, scales[], legales]
// legales = [[vx, wz, total, c0..cN]] con ci = raw*scale (null si el critic no se evaluó por short-circuit);
// se listan todas si full=true, si no solo la elegida.
static std::string evalJson(
  double t, double bx, double by, double bt, const dwb_msgs::msg::LocalPlanEvaluation * ev,
  const std::vector<std::string> & names, bool full, const std::string & exc)
{
  std::ostringstream o;
  o << "[" << jnum(t) << "," << jnum(bx) << "," << jnum(by) << "," << jnum(bt) << ",";
  if (!ev) {o << "null,0,{},[],[],\"";
    for (char ch : exc) {o << ((ch == '"' || ch == '\\' || ch == '\n') ? ' ' : ch);}
    o << "\"]"; return o.str();}
  std::map<std::string, int> ill;
  std::vector<double> sc(names.size(), NAN);
  std::ostringstream lg;
  bool first = true;
  for (size_t i = 0; i < ev->twists.size(); ++i) {
    const auto & ts = ev->twists[i];
    if (ts.total < 0) {
      bool named = false;
      for (auto & c : ts.scores) {if (c.raw_score < 0) {ill[c.name]++; named = true;}}
      if (!named) {ill["?"]++;}
      continue;
    }
    for (auto & c : ts.scores) {
      for (size_t j = 0; j < names.size(); ++j) {if (names[j] == c.name && std::isnan(sc[j])) {sc[j] = c.scale;}}
    }
    if (!full && static_cast<int>(i) != ev->best_index) {continue;}
    lg << (first ? "" : ",") << "[" << jnum(ts.traj.velocity.x) << "," << jnum(ts.traj.velocity.theta) << "," <<
      jnum(ts.total);
    for (auto & nm : names) {
      double v = NAN;
      for (auto & c : ts.scores) {if (c.name == nm) {v = c.raw_score * c.scale;}}
      lg << "," << jnum(v);
    }
    lg << "]";
    first = false;
  }
  o << ev->best_index << "," << ev->twists.size() << ",{";
  bool f2 = true;
  for (auto & kv : ill) {o << (f2 ? "" : ",") << "\"" << kv.first << "\":" << kv.second; f2 = false;}
  o << "},[";
  for (size_t j = 0; j < sc.size(); ++j) {o << (j ? "," : "") << jnum(sc[j]);}
  o << "],[" << lg.str() << "],\"\"]";
  return o.str();
}

template<typename T>
static T P(nav2_util::LifecycleNode::SharedPtr n, const std::string & name, T def)
{
  if (!n->has_parameter(name)) {n->declare_parameter(name, def);}
  return n->get_parameter(name).get_value<T>();
}

int main(int argc, char ** argv)
{
  const char * dom = std::getenv("ROS_DOMAIN_ID");
  const char * loc = std::getenv("ROS_LOCALHOST_ONLY");
  if (!dom || std::string(dom) != "99" || !loc || std::string(loc) != "1") {
    fprintf(stderr, "dwb_closed_loop_sim requiere ROS_DOMAIN_ID=99 y ROS_LOCALHOST_ONLY=1\n");
    return 2;
  }
  rclcpp::init(argc, argv);
  auto node = std::make_shared<nav2_util::LifecycleNode>("dwb_closed_loop_sim", "");
  const auto runs_file = P<std::string>(node, "runs_file", "");
  const auto out_file = P<std::string>(node, "out_file", "");
  const auto scenario = P<std::string>(node, "scenario", "S1");
  const auto plugin = P<std::string>(node, "plugin_name", "FollowPath");
  // escenario
  const double sigma_xy = P<double>(node, "sigma_xy", 0.0);
  const double sigma_yaw = P<double>(node, "sigma_yaw", 0.0);
  const double noise_tau = P<double>(node, "noise_tau", 1.0);
  const double latency = P<double>(node, "latency", 0.0);
  const double k_v = P<double>(node, "k_v", 1.0);
  const double k_w = P<double>(node, "k_w", 1.0);
  const double cmd_timeout = P<double>(node, "cmd_timeout", 0.6);
  const double dt_phys = P<double>(node, "dt_phys", 0.01);
  const double log_dt = P<double>(node, "log_dt", 0.02);
  const double t_max_factor = P<double>(node, "t_max_factor", 3.0);
  const double canon_x = P<double>(node, "canon_x", 5.0), canon_y = P<double>(node, "canon_y", 5.0);
  // parámetros reales (mismo bloque yaml que controller_server)
  const double ctrl_hz = P<double>(node, "controller_frequency", 20.0);
  const double fail_tol = P<double>(node, "failure_tolerance", 0.0);
  const double min_vx_th = P<double>(node, "min_x_velocity_threshold", 0.0001);
  const double min_w_th = P<double>(node, "min_theta_velocity_threshold", 0.0001);
  const double gc_xy = P<double>(node, "general_goal_checker.xy_goal_tolerance", 0.25);
  const double gc_yaw = P<double>(node, "general_goal_checker.yaw_goal_tolerance", 0.25);
  const bool gc_stateful = P<bool>(node, "general_goal_checker.stateful", true);
  const double pc_radius = P<double>(node, "progress_checker.required_movement_radius", 0.5);
  const double pc_time = P<double>(node, "progress_checker.movement_time_allowance", 10.0);
  const double max_v = P<double>(node, plugin + ".max_vel_x", 0.26);
  const double max_w = P<double>(node, plugin + ".max_vel_theta", 1.0);
  const double acc_v = P<double>(node, plugin + ".acc_lim_x", 2.5);
  const double acc_w = P<double>(node, plugin + ".acc_lim_theta", 3.2);
  const double dec_v = P<double>(node, plugin + ".decel_lim_x", -2.5);
  const double dec_w = P<double>(node, plugin + ".decel_lim_theta", -3.2);
  // R4-G: lista separada por comas de run ids con evaluación DWB detallada (vacío = comportamiento R4-B..R4-F)
  const auto dbg_runs_s = P<std::string>(node, "debug_eval_runs", "");
  const double dbg_full_radius = P<double>(node, "debug_eval_full_radius", 0.35);
  // R4-G0: fidelidad de replanning. fixed_plan (por defecto, comportamiento R4-B..R4-F) | replan_1hz (NavFn real desde la
  // pose estimada cada replan_period, como ComputePathToPose en el BT por defecto) | replan_same_1hz (ablación: reentrega
  // el mismo path). En ambos replan_*: dwb->setPlan(path) + reset del goal checker = ControllerServer::setPlannerPath (Humble).
  const auto plan_mode = P<std::string>(node, "plan_mode", "fixed_plan");
  const double replan_period = P<double>(node, "replan_period", 1.0);
  const auto planner_name = P<std::string>(node, "planner_plugin_name", "GridBased");
  if (plan_mode != "fixed_plan" && plan_mode != "replan_1hz" && plan_mode != "replan_same_1hz") {
    RCLCPP_FATAL(node->get_logger(), "plan_mode desconocido: %s", plan_mode.c_str());
    return 2;
  }
  std::set<std::string> dbg_runs;
  {
    std::stringstream ss(dbg_runs_s);
    std::string tok;
    while (std::getline(ss, tok, ',')) {if (!tok.empty()) {dbg_runs.insert(tok);}}
  }
  if (runs_file.empty() || out_file.empty()) {
    RCLCPP_FATAL(node->get_logger(), "faltan runs_file / out_file");
    return 2;
  }
  auto runs = readRuns(runs_file);

  // esperar /map (transient local) antes de activar el costmap
  bool have_map = false;
  auto qos = rclcpp::QoS(1).transient_local().reliable();
  auto map_sub = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "map", qos, [&](nav_msgs::msg::OccupancyGrid::ConstSharedPtr) {have_map = true;});
  auto node_thread = std::make_unique<nav2_util::NodeThread>(node->get_node_base_interface());

  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    "local_costmap", std::string{node->get_namespace()}, "local_costmap");
  auto costmap_thread = std::make_unique<nav2_util::NodeThread>(costmap_ros);
  costmap_ros->configure();
  auto tf = costmap_ros->getTfBuffer();
  const std::string gframe = costmap_ros->getGlobalFrameID(), bframe = costmap_ros->getBaseFrameID();

  auto setTF = [&](const std::string & parent, const std::string & child, double x, double y, double yaw,
      rclcpp::Time stamp, bool is_static) {
      geometry_msgs::msg::TransformStamped t;
      t.header.stamp = stamp;
      t.header.frame_id = parent;
      t.child_frame_id = child;
      t.transform.translation.x = x;
      t.transform.translation.y = y;
      t.transform.rotation = yawQ(yaw);
      tf->setTransform(t, "r4b_sim", is_static);
    };
  {
    auto now = node->now();
    setTF(gframe, "map_true", 0, 0, 0, now, true);   // odom->map_true (tf2: un solo padre por frame)
    setTF("map", gframe, 0, 0, 0, now, false);
    setTF(gframe, bframe, runs.empty() ? 0 : runs[0].sx, runs.empty() ? 0 : runs[0].sy, 0, now, false);
  }
  auto t0w = Clock::now();
  while (!have_map && Clock::now() - t0w < std::chrono::seconds(20)) {
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(1500));   // callback del static_layer
  // Costmap2DROS::start() (Humble) bloquea hasta que un updateMap() marque initialized_; con update_frequency 0 no
  // hay loop propio, así que se activa en otro hilo y se llama updateMap() desde aquí hasta que termine.
  {
    std::atomic<bool> act_done{false};
    std::thread act([&]() {costmap_ros->activate(); act_done = true;});
    while (!act_done) {
      auto now = node->now();
      setTF("map", gframe, 0, 0, 0, now, false);
      setTF(gframe, bframe, runs.empty() ? 0 : runs[0].sx, runs.empty() ? 0 : runs[0].sy, 0, now, false);
      costmap_ros->updateMap();
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    act.join();
  }

  // Chequeo de cordura: robot junto a una pared conocida -> el costmap debe contener celdas LETHAL.
  int sanity_lethal = 0;
  {
    const double spx = P<double>(node, "sanity_x", 0.5), spy = P<double>(node, "sanity_y", 5.0);
    auto now = node->now();
    setTF("map", gframe, 0, 0, 0, now, false);
    setTF(gframe, bframe, spx, spy, 0, now, false);
    costmap_ros->updateMap();
    auto * cm = costmap_ros->getCostmap();
    for (unsigned i = 0; i < cm->getSizeInCellsX(); ++i) {
      for (unsigned j = 0; j < cm->getSizeInCellsY(); ++j) {
        sanity_lethal += cm->getCost(i, j) == nav2_costmap_2d::LETHAL_OBSTACLE;
      }
    }
    RCLCPP_INFO(node->get_logger(), "sanity: celdas LETHAL con robot en (%.2f, %.2f): %d", spx, spy, sanity_lethal);
    if (sanity_lethal == 0) {RCLCPP_FATAL(node->get_logger(), "costmap sin obstáculos: abortando"); return 3;}
  }
  // R4-G0: global costmap (static+inflation, frame map) + NavFn real, solo en replan_1hz
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> gcm;
  std::unique_ptr<nav2_util::NodeThread> gcm_thread;
  std::shared_ptr<tf2_ros::Buffer> gtf;
  std::shared_ptr<nav2_navfn_planner::NavfnPlanner> navfn;
  std::vector<unsigned char> gpristine;
  long g_restored_cells_total = 0;
  auto setGTF = [&](const std::string & parent, const std::string & child, double x, double y, double yaw,
      rclcpp::Time stamp) {
      if (!gtf) {return;}
      geometry_msgs::msg::TransformStamped tt;
      tt.header.stamp = stamp;
      tt.header.frame_id = parent;
      tt.child_frame_id = child;
      tt.transform.translation.x = x;
      tt.transform.translation.y = y;
      tt.transform.rotation = yawQ(yaw);
      gtf->setTransform(tt, "r4g0_sim", false);
    };
  if (plan_mode == "replan_1hz") {
    gcm = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
      "global_costmap", std::string{node->get_namespace()}, "global_costmap");
    gcm_thread = std::make_unique<nav2_util::NodeThread>(gcm);
    gcm->configure();
    gtf = gcm->getTfBuffer();
    {
      auto now = node->now();
      setGTF("map", gframe, 0, 0, 0, now);
      setGTF(gframe, bframe, runs.empty() ? 0 : runs[0].sx, runs.empty() ? 0 : runs[0].sy, 0, now);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1500));   // callback del static_layer global
    std::atomic<bool> gact_done{false};
    std::thread gact([&]() {gcm->activate(); gact_done = true;});
    while (!gact_done) {
      auto now = node->now();
      setGTF("map", gframe, 0, 0, 0, now);
      setGTF(gframe, bframe, runs.empty() ? 0 : runs[0].sx, runs.empty() ? 0 : runs[0].sy, 0, now);
      gcm->updateMap();
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    gact.join();
    gcm->updateMap();
    // NavfnPlanner::makePlan llama clearRobotCell() -> setCost(FREE_SPACE) sobre el costmap MAESTRO. En Nav2 real la capa
    // de obstáculos del global costmap repinta la zona del robot en cada update; aquí el global es estático y esas celdas
    // quedarían libres para siempre (también entre corridas). Se guarda el costmap prístino y se restaura antes de cada plan.
    {
      auto * gc = gcm->getCostmap();
      gpristine.assign(gc->getCharMap(), gc->getCharMap() + gc->getSizeInCellsX() * gc->getSizeInCellsY());
    }
    navfn = std::make_shared<nav2_navfn_planner::NavfnPlanner>();
    navfn->configure(node, planner_name, gtf, gcm);
    navfn->activate();
    RCLCPP_INFO(node->get_logger(), "R4-G0 replan_1hz: global costmap %ux%u res %.3f, NavFn '%s'",
      gcm->getCostmap()->getSizeInCellsX(), gcm->getCostmap()->getSizeInCellsY(), gcm->getCostmap()->getResolution(),
      planner_name.c_str());
  }
  auto dwb = std::make_shared<dwb_core::DWBLocalPlanner>();
  dwb->configure(node, plugin, tf, costmap_ros);
  dwb->activate();
  RCLCPP_INFO(
    node->get_logger(), "R4B %s plugin=%s runs=%zu map=%d sxy=%.3f syaw=%.3f lat=%.2f kv=%.2f kw=%.2f",
    scenario.c_str(), plugin.c_str(), runs.size(), have_map, sigma_xy, sigma_yaw, latency, k_v, k_w);

  std::ofstream out(out_file);
  {
    std::ostringstream o;
    o << "{\"meta\":{\"scenario\":\"" << scenario << "\",\"plugin\":\"" << plugin << "\",\"sigma_xy\":" <<
      jnum(sigma_xy) << ",\"sigma_yaw\":" << jnum(sigma_yaw) << ",\"noise_tau\":" << jnum(noise_tau) <<
      ",\"latency\":" << jnum(latency) << ",\"k_v\":" << jnum(k_v) << ",\"k_w\":" << jnum(k_w) <<
      ",\"cmd_timeout\":" << jnum(cmd_timeout) << ",\"dt_phys\":" << jnum(dt_phys) << ",\"ctrl_hz\":" <<
      jnum(ctrl_hz) << ",\"failure_tolerance\":" << jnum(fail_tol) << ",\"goal_xy\":" << jnum(gc_xy) <<
      ",\"goal_yaw\":" << jnum(gc_yaw) << ",\"goal_stateful\":" << gc_stateful << ",\"progress_radius\":" <<
      jnum(pc_radius) << ",\"progress_time\":" << jnum(pc_time) << ",\"max_v\":" << jnum(max_v) <<
      ",\"max_w\":" << jnum(max_w) << ",\"acc_v\":" << jnum(acc_v) << ",\"acc_w\":" << jnum(acc_w) <<
      ",\"dec_v\":" << jnum(dec_v) << ",\"dec_w\":" << jnum(dec_w) << ",\"map_received\":" << have_map << ",\"sanity_lethal_cells\":" << sanity_lethal <<
      ",\"n_runs\":" << runs.size();
    if (plan_mode != "fixed_plan") {o << ",\"plan_mode\":\"" << plan_mode << "\",\"replan_period\":" << jnum(replan_period);}
    o << "}}";
    out << o.str() << "\n";
  }

  std::ofstream dbg_out;
  std::vector<std::string> crit_names;
  if (!dbg_runs.empty()) {
    dbg_out.open(out_file + ".eval.jsonl");
    crit_names = P<std::vector<std::string>>(node, plugin + ".critics", std::vector<std::string>{});
  }
  const double dt_ctrl = 1.0 / ctrl_hz;
  const int phys_per_ctrl = static_cast<int>(std::round(dt_ctrl / dt_phys));
  const int log_every = std::max(1, static_cast<int>(std::round(log_dt / dt_phys)));

  size_t k = 0;
  for (auto & r : runs) {
    if (!rclcpp::ok()) {break;}
    ++k;
    std::mt19937 rng(r.seed);
    std::normal_distribution<double> N(0.0, 1.0);
    const double a = noise_tau > 0 ? std::exp(-dt_ctrl / noise_tau) : 0.0;
    const double b = std::sqrt(1 - a * a);
    // estado inicial del ruido en régimen estacionario
    double ex = sigma_xy * N(rng), ey = sigma_xy * N(rng), eth = sigma_yaw * N(rng);

    double path_len = 0;
    for (size_t i = 1; i < r.path.size(); ++i) {
      path_len += std::hypot(r.path[i][0] - r.path[i - 1][0], r.path[i][1] - r.path[i - 1][1]);
    }
    const double t_max = t_max_factor * path_len / max_v + 30.0;

    nav_msgs::msg::Path path;
    path.header.frame_id = "map";
    path.header.stamp = node->now();
    for (auto & p : r.path) {
      geometry_msgs::msg::PoseStamped ps;
      ps.header = path.header;
      ps.pose.position.x = p[0];
      ps.pose.position.y = p[1];
      ps.pose.orientation = yawQ(p[2]);
      path.poses.push_back(ps);
    }
    int start_lethal = 0;
    std::ostringstream ray;
    // Independencia entre corridas: el costmap rodante recentra con truncamiento entero desde el origen anterior
    // (±1 celda según la historia). Se lo lleva primero a una pose canónica fija para que cada corrida dependa solo
    // de su propio recorrido.
    {
      // origen EXACTO (resizeMap) para eliminar la deriva de punto flotante acumulada por los recentrados
      auto * lc = costmap_ros->getLayeredCostmap();
      auto * cm = lc->getCostmap();
      lc->resizeMap(
        cm->getSizeInCellsX(), cm->getSizeInCellsY(), cm->getResolution(),
        canon_x - cm->getSizeInMetersX() / 2.0, canon_y - cm->getSizeInMetersY() / 2.0, false);
      auto now = node->now();
      setTF("map", gframe, 0, 0, 0, now, false);
      setTF(gframe, bframe, canon_x, canon_y, 0, now, false);
      costmap_ros->updateMap();
      now = node->now();
      setTF(gframe, bframe, r.sx, r.sy, r.syaw, now, false);
      costmap_ros->updateMap();
      // costos del costmap a lo largo del rumbo inicial (diagnóstico: celdas INSCRIBED/LETHAL delante del robot)
      ray.str("");
      for (int i = 0; i <= 29; ++i) {
        unsigned mx, my;
        const double px = r.sx + 0.05 * i * std::cos(r.syaw), py = r.sy + 0.05 * i * std::sin(r.syaw);
        ray << (i ? "," : "") << (cm->worldToMap(px, py, mx, my) ? static_cast<int>(cm->getCost(mx, my)) : -1);
      }
      start_lethal = 0;
      for (unsigned i = 0; i < cm->getSizeInCellsX(); ++i) {
        for (unsigned j = 0; j < cm->getSizeInCellsY(); ++j) {
          start_lethal += cm->getCost(i, j) == nav2_costmap_2d::LETHAL_OBSTACLE;
        }
      }
    }
    dwb->setPlan(path);
    // R4-G0: registro de setPlan [t, ok, n_poses, largo, x0, y0, d(x0,pose_est), causa]
    std::ostringstream SP;
    bool firstSP = true;
    int n_setplan = 1, n_plan_fail = 0;
    SP << "[0,1," << path.poses.size() << "," << jnum(path_len) << "," << jnum(r.path.empty() ? NAN : r.path[0][0]) << "," <<
      jnum(r.path.empty() ? NAN : r.path[0][1]) << ",null,\"inicial\"]";
    firstSP = false;
    std::ostringstream PLAN0;   // primer plan NavFn completo (validación vs paths_R3)
    double next_replan = plan_mode == "replan_1hz" ? 0.0 : replan_period;
    size_t same_idx = 0;

    // estado verdadero y base
    double x = r.sx, y = r.sy, th = r.syaw, v = 0, w = 0;
    struct QCmd {double t, v, w;};
    std::deque<QCmd> q;
    double tgt_cmd_v = 0, tgt_cmd_w = 0, last_rx = 0;
    bool have_rx = false;
    // controller_server replicado
    double last_valid = 0;
    bool gc_check_xy = true;
    double pc_bx = x, pc_by = y, pc_bt = 0;
    std::string status = "TIMEOUT", reason;
    double t = 0, t_goal = NAN;
    int n_fail = 0, n_ctrl = 0;
    double dwb_ms_sum = 0, dwb_ms_max = 0, um_sum = 0, um_max = 0;
    auto wall0 = Clock::now();
    std::ostringstream L;   // log control-rate
    std::ostringstream H;   // log alta tasa de la pose verdadera
    const bool dbg_on = dbg_runs.count(r.id) > 0;
    std::ostringstream D;   // R4-G: evaluación DWB por ciclo
    bool firstD = true;
    bool firstL = true, firstH = true;
    int phys_i = 0;
    bool done = false;
    int stop_steps = 0;

    auto physStep = [&]() {
        while (!q.empty() && q.front().t <= t + 1e-9) {
          tgt_cmd_v = q.front().v; tgt_cmd_w = q.front().w; last_rx = t; have_rx = true;
          q.pop_front();
        }
        double tv = 0, tw = 0;
        if (have_rx && t - last_rx <= cmd_timeout) {
          tv = std::clamp(k_v * tgt_cmd_v, -max_v, max_v);
          tw = std::clamp(k_w * tgt_cmd_w, -max_w, max_w);
        }
        v = ramp(v, tv, acc_v, dec_v, dt_phys);
        w = ramp(w, tw, acc_w, dec_w, dt_phys);
        if (std::fabs(w) > 1e-9) {
          x += v / w * (std::sin(th + w * dt_phys) - std::sin(th));
          y += -v / w * (std::cos(th + w * dt_phys) - std::cos(th));
        } else {
          x += v * std::cos(th) * dt_phys;
          y += v * std::sin(th) * dt_phys;
        }
        th = wrap(th + w * dt_phys);
        t += dt_phys;
        if (++phys_i % log_every == 0) {
          H << (firstH ? "" : ",") << "[" << jnum(t) << "," << jnum(x) << "," << jnum(y) << "," << jnum(th) << "]";
          firstH = false;
        }
      };

    while (!done && rclcpp::ok()) {
      // ruido de localización (Gauss-Markov) a tasa de control
      ex = a * ex + b * sigma_xy * N(rng);
      ey = a * ey + b * sigma_xy * N(rng);
      eth = a * eth + b * sigma_yaw * N(rng);
      const double ex_x = x + ex, ex_y = y + ey, ex_th = wrap(th + eth);   // pose estimada en map
      // map->odom tal que (map->odom) o (odom->base) = pose estimada
      const double c = std::cos(eth), s = std::sin(eth);
      const double mo_x = ex_x - (c * x - s * y), mo_y = ex_y - (s * x + c * y);
      auto u0 = Clock::now();
      auto stamp = node->now();
      setTF("map", gframe, mo_x, mo_y, eth, stamp, false);
      setTF(gframe, bframe, x, y, th, stamp, false);
      costmap_ros->updateMap();
      const double um = std::chrono::duration<double, std::milli>(Clock::now() - u0).count();
      if (plan_mode != "fixed_plan") {
        setGTF("map", gframe, mo_x, mo_y, eth, stamp);
        setGTF(gframe, bframe, x, y, th, stamp);
      }
      // R4-G0: replanning periódico (ControllerServer::updateGlobalPath -> setPlannerPath, antes de computar)
      if (plan_mode != "fixed_plan" && t + 1e-9 >= next_replan) {
        next_replan += replan_period;
        nav_msgs::msg::Path np;
        std::string why = "replan";
        if (plan_mode == "replan_same_1hz") {
          // ablación: MISMA geometría del path original, recortada desde la pose más cercana a la estimada (búsqueda
          // monótona hacia adelante, como el plan ya podado que DWB tiene) -> el único efecto es el reset de setPlan
          double best = 1e9;
          size_t jbest = same_idx;
          for (size_t j = same_idx; j < path.poses.size(); ++j) {
            const double dd = std::hypot(path.poses[j].pose.position.x - ex_x, path.poses[j].pose.position.y - ex_y);
            if (dd < best) {best = dd; jbest = j;}
          }
          same_idx = jbest;
          np.header = path.header;
          np.header.stamp = node->now();
          np.poses.assign(path.poses.begin() + static_cast<long>(jbest), path.poses.end());
          why = "same";
        } else {
          geometry_msgs::msg::PoseStamped st, gl;
          st.header.frame_id = "map";
          st.header.stamp = stamp;
          st.pose.position.x = ex_x;
          st.pose.position.y = ex_y;
          st.pose.orientation = yawQ(ex_th);
          gl.header = st.header;
          gl.pose.position.x = r.gx;
          gl.pose.position.y = r.gy;
          gl.pose.orientation = yawQ(r.gyaw);
          {
            auto * gc = gcm->getCostmap();
            std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lk(*(gc->getMutex()));
            unsigned char * cm = gc->getCharMap();
            for (size_t q = 0; q < gpristine.size(); ++q) {g_restored_cells_total += cm[q] != gpristine[q];}
            std::copy(gpristine.begin(), gpristine.end(), cm);
          }
          try {
            np = navfn->createPlan(st, gl);
          } catch (const std::exception & e) {
            why = std::string("excepcion ") + e.what();
            np.poses.clear();
          }
        }
        double plen = 0;
        for (size_t i = 1; i < np.poses.size(); ++i) {
          plen += std::hypot(np.poses[i].pose.position.x - np.poses[i - 1].pose.position.x,
              np.poses[i].pose.position.y - np.poses[i - 1].pose.position.y);
        }
        const bool pok = !np.poses.empty();
        if (pok) {
          dwb->setPlan(np);          // DWBLocalPlanner::setPlan: reset() de todos los critics + traj_generator
          gc_check_xy = true;        // SimpleGoalChecker::reset() (stateful)
          ++n_setplan;
          if (PLAN0.tellp() == 0 && plan_mode == "replan_1hz") {
            bool fp = true;
            for (auto & ps : np.poses) {
              PLAN0 << (fp ? "" : ",") << "[" << jnum(ps.pose.position.x) << "," << jnum(ps.pose.position.y) << "]";
              fp = false;
            }
          }
        } else {
          ++n_plan_fail;   // en el BT real: ComputePathToPose FAILURE (ver informe); aquí se conserva el path anterior
        }
        SP << (firstSP ? "" : ",") << "[" << jnum(t) << "," << pok << "," << np.poses.size() << "," << jnum(plen) << "," <<
          jnum(pok ? np.poses[0].pose.position.x : NAN) << "," << jnum(pok ? np.poses[0].pose.position.y : NAN) << "," <<
          jnum(pok ? std::hypot(np.poses[0].pose.position.x - ex_x, np.poses[0].pose.position.y - ex_y) : NAN) <<
          ",\"" << why << "\"]";
        firstSP = false;
      }
      um_sum += um; um_max = std::max(um_max, um);

      // --- ciclo de controller_server ---
      geometry_msgs::msg::PoseStamped pose;
      pose.header.frame_id = gframe;
      pose.header.stamp = stamp;
      pose.pose.position.x = x;
      pose.pose.position.y = y;
      pose.pose.orientation = yawQ(th);
      geometry_msgs::msg::Twist vel;
      vel.linear.x = std::fabs(v) < min_vx_th ? 0.0 : v;
      vel.angular.z = std::fabs(w) < min_w_th ? 0.0 : w;
      double cv = 0, cw = 0;
      bool ok = true;
      std::string exc;
      auto a0 = Clock::now();
      std::shared_ptr<dwb_msgs::msg::LocalPlanEvaluation> ev;
      try {
        if (dbg_on) {
          // misma ruta que DWBLocalPlanner::computeVelocityCommands(PoseStamped, Twist, GoalChecker*) en Humble,
          // pero pidiendo la evaluación (results != nullptr solo registra, no cambia la selección)
          ev = std::make_shared<dwb_msgs::msg::LocalPlanEvaluation>();
          auto c2 = dwb->computeVelocityCommands(
            nav_2d_utils::poseStampedToPose2D(pose), nav_2d_utils::twist3Dto2D(vel), ev);
          const auto tw = nav_2d_utils::twist2Dto3D(c2.velocity);
          cv = tw.linear.x; cw = tw.angular.z;
        } else {
          auto cmd = dwb->computeVelocityCommands(pose, vel, nullptr);
          cv = cmd.twist.linear.x; cw = cmd.twist.angular.z;
        }
        last_valid = t;
      } catch (const std::exception & e) {
        ok = false; ++n_fail; exc = e.what();
      }
      const double ms = std::chrono::duration<double, std::milli>(Clock::now() - a0).count();
      dwb_ms_sum += ms; dwb_ms_max = std::max(dwb_ms_max, ms); ++n_ctrl;
      if (!ok) {
        if (fail_tol >= 0 && t - last_valid > fail_tol) {
          status = "ABORT_CONTROLLER"; reason = exc; done = true;
        }
      }
      // progress checker (pose en frame del costmap = odom = verdadera), tiempo simulado
      if (!done) {
        if (std::hypot(x - pc_bx, y - pc_by) > pc_radius) {pc_bx = x; pc_by = y; pc_bt = t;} else if (
          t - pc_bt > pc_time) {status = "ABORT_PROGRESS"; reason = "Failed to make progress"; done = true;}
      }
      if (done) {cv = 0; cw = 0;}
      q.push_back({t + latency, cv, cw});
      // goal checker (pose estimada en map vs goal)
      bool reached = false;
      if (!done) {
        bool xy_ok = !gc_check_xy;
        if (gc_check_xy && std::hypot(ex_x - r.gx, ex_y - r.gy) <= gc_xy) {
          xy_ok = true;
          if (gc_stateful) {gc_check_xy = false;}
        }
        reached = xy_ok && std::fabs(wrap(ex_th - r.gyaw)) < gc_yaw;
        if (reached) {status = "SUCCEEDED"; t_goal = t; done = true; q.push_back({t + latency, 0, 0});}
      }
      L << (firstL ? "" : ",") << "[" << jnum(t) << "," << jnum(x) << "," << jnum(y) << "," << jnum(th) << "," <<
        jnum(ex_x) << "," << jnum(ex_y) << "," << jnum(ex_th) << "," << jnum(v) << "," << jnum(w) << "," <<
        jnum(cv) << "," << jnum(cw) << "," << ok << "," << jnum(ms) << "]";
      firstL = false;
      if (dbg_on) {
        const bool full = std::hypot(ex_x - r.gx, ex_y - r.gy) <= dbg_full_radius;
        D << (firstD ? "" : ",") << evalJson(t, pc_bx, pc_by, pc_bt, ok ? ev.get() : nullptr, crit_names, full, exc);
        firstD = false;
      }
      if (!done && t >= t_max) {status = "TIMEOUT"; reason = "t_max"; done = true; q.push_back({t + latency, 0, 0});}
      for (int i = 0; i < phys_per_ctrl; ++i) {physStep();}
    }
    // tras terminar: la base recibe el cero (con latencia) y frena; simular hasta detenerse
    while (stop_steps < 500 && (std::fabs(v) > 1e-4 || std::fabs(w) > 1e-4 || !q.empty())) {
      physStep(); ++stop_steps;
    }
    std::ostringstream o;
    o << "{\"run\":\"" << r.id << "\",\"status\":\"" << status << "\",\"reason\":\"";
    for (char ch : reason) {o << ((ch == '"' || ch == '\\' || ch == '\n') ? ' ' : ch);}
    o << "\",\"t_goal\":" << jnum(t_goal) << ",\"t_end\":" << jnum(t) << ",\"path_len\":" << jnum(path_len) <<
      ",\"goal\":[" << jnum(r.gx) << "," << jnum(r.gy) << "," << jnum(r.gyaw) << "],\"final_true\":[" << jnum(x) <<
      "," << jnum(y) << "," << jnum(th) << "],\"start_lethal_cells\":" << start_lethal << ",\"start_ray_costs\":[" << ray.str() << "]" << ",\"n_ctrl\":" << n_ctrl << ",\"n_fail\":" << n_fail <<
      ",\"dwb_ms_mean\":" << jnum(n_ctrl ? dwb_ms_sum / n_ctrl : NAN) << ",\"dwb_ms_max\":" << jnum(dwb_ms_max) <<
      ",\"updatemap_ms_mean\":" << jnum(n_ctrl ? um_sum / n_ctrl : NAN) << ",\"updatemap_ms_max\":" << jnum(um_max) <<
      ",\"wall_s\":" << jnum(std::chrono::duration<double>(Clock::now() - wall0).count()) <<
      ",\"ctrl\":[" << L.str() << "],\"hi\":[" << H.str() << "]";
    if (plan_mode != "fixed_plan") {
      o << ",\"plan_mode\":\"" << plan_mode << "\",\"n_setplan\":" << n_setplan << ",\"n_plan_fail\":" << n_plan_fail <<
        ",\"setplan\":[" << SP.str() << "],\"plan0\":[" << PLAN0.str() << "],\"g_restored_cells_cum\":" <<
        g_restored_cells_total;
    }
    o << "}";
    out << o.str() << "\n";
    out.flush();
    if (dbg_on) {
      dbg_out << "{\"run\":\"" << r.id << "\",\"critics\":[";
      for (size_t j = 0; j < crit_names.size(); ++j) {dbg_out << (j ? "," : "") << "\"" << crit_names[j] << "\"";}
      dbg_out << "],\"cycle_fields\":[\"t\",\"pc_bx\",\"pc_by\",\"pc_bt\",\"best_index\",\"n_twists\",\"illegal_by_critic\","
        "\"scales\",\"legal[vx,wz,total,weighted...]\",\"exception\"],\"cycles\":[" << D.str() << "]}\n";
      dbg_out.flush();
    }
    RCLCPP_INFO(node->get_logger(), "[%zu/%zu] %s %s t=%.1f", k, runs.size(), r.id.c_str(), status.c_str(), t);
  }
  out.close();
  dwb->deactivate();
  costmap_ros->deactivate();
  if (navfn) {navfn->deactivate();}
  if (gcm) {gcm->deactivate();}
  dwb->cleanup();
  costmap_ros->cleanup();
  costmap_thread.reset();
  node_thread.reset();
  rclcpp::shutdown();
  return 0;
}
