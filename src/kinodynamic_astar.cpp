/**
* This file is part of Fast-Planner.
*
* Copyright 2019 Boyu Zhou, Aerial Robotics Group, Hong Kong University of Science and Technology, <uav.ust.hk>
* Developed by Boyu Zhou <bzhouai at connect dot ust dot hk>, <uv dot boyuzhou at gmail dot com>
* for more information see <https://github.com/HKUST-Aerial-Robotics/Fast-Planner>.
* If you use this code, please cite the respective publications as
* listed on the above website.
*
* Fast-Planner is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* Fast-Planner is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with Fast-Planner. If not, see <http://www.gnu.org/licenses/>.
*/

#include <path_searching/kinodynamic_astar.h>
#include <sstream>

using namespace std;
using namespace Eigen;

namespace path_searching
{

KinodynamicAstar::~KinodynamicAstar()
{
  for (int i = 0; i < allocate_num_; i++)
  {
    delete path_node_pool_[i];
  }
}

int KinodynamicAstar::search(Eigen::Vector3d start_pt, Eigen::Vector3d start_v, Eigen::Vector3d start_a,
                             Eigen::Vector3d end_pt, Eigen::Vector3d end_v, bool init, bool dynamic, double time_start)
{
  start_vel_ = start_v;
  start_acc_ = start_a;

  PathNodePtr cur_node = path_node_pool_[0];
  cur_node->parent = NULL;
  cur_node->state.head(3) = start_pt;
  cur_node->state.tail(3) = start_v;
  cur_node->index = posToIndex(start_pt);
  cur_node->g_score = 0.0;

  Eigen::VectorXd end_state(6);
  Eigen::Vector3i end_index;
  double time_to_goal;

  end_state.head(3) = end_pt;
  end_state.tail(3) = end_v;
  end_index = posToIndex(end_pt);
  cur_node->f_score = lambda_heu_ * estimateHeuristic(cur_node->state, end_state, time_to_goal);
  cur_node->node_state = IN_OPEN_SET;
  open_set_.push(cur_node);
  use_node_num_ += 1;

  if (dynamic)
  {
    time_origin_ = time_start;
    cur_node->time = time_start;
    cur_node->time_idx = timeToIndex(time_start);
    expanded_nodes_.insert(cur_node->index, cur_node->time_idx, cur_node);
  }
  else
    expanded_nodes_.insert(cur_node->index, cur_node);

  PathNodePtr neighbor = NULL;
  PathNodePtr terminate_node = NULL;
  bool init_search = init;
  const int tolerance = ceil(1 / resolution_);

  while (!open_set_.empty())
  {
    cur_node = open_set_.top();

    // Terminate?
    bool reach_horizon = (cur_node->state.head(3) - start_pt).norm() >= horizon_;
    bool near_end = abs(cur_node->index(0) - end_index(0)) <= tolerance &&
                    abs(cur_node->index(1) - end_index(1)) <= tolerance &&
                    abs(cur_node->index(2) - end_index(2)) <= tolerance;

    if (reach_horizon || near_end)
    {
      terminate_node = cur_node;
      retrievePath(terminate_node);
      if (near_end)
      {
        // Check whether shot traj exist
        estimateHeuristic(cur_node->state, end_state, time_to_goal);
        computeShotTraj(cur_node->state, end_state, time_to_goal);
        if (init_search)
          RCLCPP_ERROR(rclcpp::get_logger("kinodynamic_astar"), "Shot in first search loop!");
      }
    }
    if (reach_horizon)
    {
      if (is_shot_succ_)
      {
        std::cout << "reach end" << std::endl;
        return REACH_END;
      }
      else
      {
        std::cout << "reach horizon" << std::endl;
        return REACH_HORIZON;
      }
    }

    if (near_end)
    {
      if (is_shot_succ_)
      {
        std::cout << "reach end" << std::endl;
        return REACH_END;
      }
      else if (cur_node->parent != NULL)
      {
        std::cout << "near end" << std::endl;
        return NEAR_END;
      }
      else
      {
        std::cout << "no path" << std::endl;
        return NO_PATH;
      }
    }
    open_set_.pop();
    cur_node->node_state = IN_CLOSE_SET;
    iter_num_ += 1;

    double res = 1 / 2.0, time_res = 1 / 1.0, time_res_init = 1 / 20.0;
    Eigen::Matrix<double, 6, 1> cur_state = cur_node->state;
    Eigen::Matrix<double, 6, 1> pro_state;
    std::vector<PathNodePtr> tmp_expand_nodes;
    Eigen::Vector3d um;
    double pro_t;
    std::vector<Eigen::Vector3d> inputs;
    std::vector<double> durations;
    if (init_search)
    {
      inputs.push_back(start_acc_);
      for (double tau = time_res_init * init_max_tau_; tau <= init_max_tau_ + 1e-3;
           tau += time_res_init * init_max_tau_)
        durations.push_back(tau);
      init_search = false;
    }
    else
    {
      for (double ax = -max_acc_; ax <= max_acc_ + 1e-3; ax += max_acc_ * res)
        for (double ay = -max_acc_; ay <= max_acc_ + 1e-3; ay += max_acc_ * res)
          for (double az = -max_acc_; az <= max_acc_ + 1e-3; az += max_acc_ * res)
          {
            um << ax, ay, az;
            inputs.push_back(um);
          }
      for (double tau = time_res * max_tau_; tau <= max_tau_; tau += time_res * max_tau_)
        durations.push_back(tau);
    }

    for (int i = 0; i < inputs.size(); ++i)
      for (int j = 0; j < durations.size(); ++j)
      {
        um = inputs[i];
        double tau = durations[j];
        stateTransit(cur_state, pro_state, um, tau);
        pro_t = cur_node->time + tau;

        Eigen::Vector3d pro_pos = pro_state.head(3);

        // Check if in close set
        Eigen::Vector3i pro_id = posToIndex(pro_pos);
        int pro_t_id = timeToIndex(pro_t);
        PathNodePtr pro_node = dynamic ? expanded_nodes_.find(pro_id, pro_t_id) : expanded_nodes_.find(pro_id);
        if (pro_node != NULL && pro_node->node_state == IN_CLOSE_SET)
        {
          if (init_search)
            std::cout << "close" << std::endl;
          continue;
        }

        // Check maximal velocity
        Eigen::Vector3d pro_v = pro_state.tail(3);
        if (fabs(pro_v(0)) > max_vel_ || fabs(pro_v(1)) > max_vel_ || fabs(pro_v(2)) > max_vel_)
        {
          if (init_search)
            std::cout << "vel" << std::endl;
          continue;
        }

        // Check not in the same voxel
        Eigen::Vector3i diff = pro_id - cur_node->index;
        int diff_time = pro_t_id - cur_node->time_idx;
        if (diff.norm() == 0 && ((!dynamic) || diff_time == 0))
        {
          if (init_search)
            std::cout << "same" << std::endl;
          continue;
        }

        // Check safety
        Eigen::Vector3d pos;
        Eigen::Matrix<double, 6, 1> xt;
        bool is_occ = false;
        for (int k = 1; k <= check_num_; ++k)
        {
          double dt = tau * double(k) / double(check_num_);
          stateTransit(cur_state, xt, um, dt);
          pos = xt.head(3);
          if (!edt_environment_->isFree(pos))
          {
            is_occ = true;
            break;
          }
        }
        if (is_occ)
        {
          if (init_search)
            std::cout << "safe" << std::endl;
          continue;
        }

        double time_to_goal, tmp_g_score, tmp_f_score;
        tmp_g_score = (um.squaredNorm() + w_time_) * tau + cur_node->g_score;
        tmp_f_score = tmp_g_score + lambda_heu_ * estimateHeuristic(pro_state, end_state, time_to_goal);

        // Compare nodes expanded from the same parent
        bool prune = false;
        for (int j = 0; j < tmp_expand_nodes.size(); ++j)
        {
          PathNodePtr expand_node = tmp_expand_nodes[j];
          if ((pro_id - expand_node->index).norm() == 0 && ((!dynamic) || pro_t_id == expand_node->time_idx))
          {
            prune = true;
            if (tmp_f_score < expand_node->f_score)
            {
              expand_node->f_score = tmp_f_score;
              expand_node->g_score = tmp_g_score;
              expand_node->state = pro_state;
              expand_node->input = um;
              expand_node->duration = tau;
              if (dynamic)
                expand_node->time = cur_node->time + tau;
            }
            break;
          }
        }

        // This node end up in a voxel different from others
        if (!prune)
        {
          if (pro_node == NULL)
          {
            pro_node = path_node_pool_[use_node_num_];
            pro_node->index = pro_id;
            pro_node->state = pro_state;
            pro_node->f_score = tmp_f_score;
            pro_node->g_score = tmp_g_score;
            pro_node->input = um;
            pro_node->duration = tau;
            pro_node->parent = cur_node;
            pro_node->node_state = IN_OPEN_SET;
            if (dynamic)
            {
              pro_node->time = cur_node->time + tau;
              pro_node->time_idx = timeToIndex(pro_node->time);
            }
            open_set_.push(pro_node);

            if (dynamic)
              expanded_nodes_.insert(pro_id, pro_node->time, pro_node);
            else
              expanded_nodes_.insert(pro_id, pro_node);

            tmp_expand_nodes.push_back(pro_node);

            use_node_num_ += 1;
            if (use_node_num_ == allocate_num_)
            {
              cout << "run out of memory." << endl;
              return NO_PATH;
            }
          }
          else if (pro_node->node_state == IN_OPEN_SET)
          {
            if (tmp_g_score < pro_node->g_score)
            {
              pro_node->state = pro_state;
              pro_node->f_score = tmp_f_score;
              pro_node->g_score = tmp_g_score;
              pro_node->input = um;
              pro_node->duration = tau;
              pro_node->parent = cur_node;
              if (dynamic)
                pro_node->time = cur_node->time + tau;
            }
          }
          else
          {
            cout << "error type in searching: " << pro_node->node_state << endl;
          }
        }
      }
  }

  cout << "open set empty, no path!" << endl;
  cout << "use node num: " << use_node_num_ << endl;
  cout << "iter num: " << iter_num_ << endl;
  return NO_PATH;
}

void KinodynamicAstar::setParam(rclcpp::Node::SharedPtr node)
{
  node->declare_parameter("search.max_tau", -1.0);
  node->declare_parameter("search.init_max_tau", -1.0);
  node->declare_parameter("search.max_vel", -1.0);
  node->declare_parameter("search.max_acc", -1.0);
  node->declare_parameter("search.w_time", -1.0);
  node->declare_parameter("search.horizon", -1.0);
  node->declare_parameter("search.resolution_astar", -1.0);
  node->declare_parameter("search.time_resolution", -1.0);
  node->declare_parameter("search.lambda_heu", -1.0);
  node->declare_parameter("search.allocate_num", -1);
  node->declare_parameter("search.check_num", -1);
  node->declare_parameter("search.optimistic", true);

  max_tau_ = node->get_parameter("search.max_tau").as_double();
  init_max_tau_ = node->get_parameter("search.init_max_tau").as_double();
  max_vel_ = node->get_parameter("search.max_vel").as_double();
  max_acc_ = node->get_parameter("search.max_acc").as_double();
  w_time_ = node->get_parameter("search.w_time").as_double();
  horizon_ = node->get_parameter("search.horizon").as_double();
  resolution_ = node->get_parameter("search.resolution_astar").as_double();
  time_resolution_ = node->get_parameter("search.time_resolution").as_double();
  lambda_heu_ = node->get_parameter("search.lambda_heu").as_double();
  allocate_num_ = node->get_parameter("search.allocate_num").as_int();
  check_num_ = node->get_parameter("search.check_num").as_int();
  optimistic_ = node->get_parameter("search.optimistic").as_bool();
  
  tie_breaker_ = 1.0 + 1.0 / 10000;
}

void KinodynamicAstar::init()
{
  /* ---------- map params ---------- */
  inv_resolution_ = 1.0 / resolution_;
  inv_time_resolution_ = 1.0 / time_resolution_;
  edt_environment_->getMapSize(map_size_3d_);
  edt_environment_->getOrigin(origin_);

  /* ---------- pre-allocated node ---------- */
  path_node_pool_.resize(allocate_num_);
  for (int i = 0; i < allocate_num_; i++)
  {
    path_node_pool_[i] = new PathNode;
  }

  phi_ = Eigen::MatrixXd::Identity(6, 6);
  use_node_num_ = 0;
  iter_num_ = 0;
}

void KinodynamicAstar::reset()
{
  expanded_nodes_.clear();
  path_nodes_.clear();

  std::priority_queue<PathNodePtr, std::vector<PathNodePtr>, NodeComparator> empty_queue;
  open_set_.swap(empty_queue);

  for (int i = 0; i < use_node_num_; i++)
  {
    PathNodePtr node = path_node_pool_[i];
    node->parent = NULL;
    node->node_state = NOT_EXPAND;
  }

  use_node_num_ = 0;
  iter_num_ = 0;
  is_shot_succ_ = false;
  has_path_ = false;
}

void KinodynamicAstar::retrievePath(PathNodePtr end_node)
{
  PathNodePtr cur_node = end_node;
  path_nodes_.push_back(cur_node);

  while (cur_node->parent != NULL)
  {
    cur_node = cur_node->parent;
    path_nodes_.push_back(cur_node);
  }

  reverse(path_nodes_.begin(), path_nodes_.end());
}

double KinodynamicAstar::estimateHeuristic(Eigen::VectorXd x1, Eigen::VectorXd x2, double& optimal_time)
{
  const Vector3d dp = x2.head(3) - x1.head(3);
  const Vector3d v0 = x1.segment(3, 3);
  const Vector3d v1 = x2.segment(3, 3);

  double c1 = -36 * dp.dot(dp);
  double c2 = 24 * (v0 + v1).dot(dp);
  double c3 = -4 * (v0.dot(v0) + v0.dot(v1) + v1.dot(v1));
  double c4 = 0;
  double c5 = w_time_;

  std::vector<double> ts = quartic(c5, c4, c3, c2, c1);

  double v_max = max_vel_ * 0.5;
  double t_bar = (x1.head(3) - x2.head(3)).lpNorm<Infinity>() / v_max;
  ts.push_back(t_bar);

  double cost = 100000000;
  double t_d = t_bar;

  for (auto t : ts)
  {
    if (t < t_bar)
      continue;
    double c = -c1 / (3 * t * t * t) - c2 / (2 * t * t) - c3 / t + w_time_ * t;
    if (c < cost)
    {
      cost = c;
      t_d = t;
    }
  }

  optimal_time = t_d;

  return 1.0 * (1 + tie_breaker_) * cost;
}

bool KinodynamicAstar::computeShotTraj(Eigen::VectorXd state1, Eigen::VectorXd state2, double time_to_goal)
{
  /* ---------- get coefficient ---------- */
  const Vector3d p0 = state1.head(3);
  const Vector3d dp = state2.head(3) - p0;
  const Vector3d v0 = state1.segment(3, 3);
  const Vector3d v1 = state2.segment(3, 3);
  const Vector3d dv = v1 - v0;
  double t_d = time_to_goal;
  MatrixXd coef(3, 4);
  end_vel_ = v1;

  Vector3d a = 1.0 / 6.0 * (-12.0 / (t_d * t_d * t_d) * (dp - v0 * t_d) + 6 / (t_d * t_d) * dv);
  Vector3d b = 0.5 * (6.0 / (t_d * t_d) * (dp - v0 * t_d) - 2 / t_d * dv);
  Vector3d c = v0;
  Vector3d d = p0;

  coef.col(3) = a, coef.col(2) = b, coef.col(1) = c, coef.col(0) = d;

  Vector3d coord, vel, acc;
  VectorXd poly1d, t, polyv, polya;
  Vector3i index;

  Eigen::MatrixXd Tm(4, 4);
  Tm << 0, 1, 0, 0, 0, 0, 2, 0, 0, 0, 0, 3, 0, 0, 0, 0;

  /* ---------- forward checking of trajectory ---------- */
  double t_delta = t_d / 10;
  for (double time = t_delta; time <= t_d; time += t_delta)
  {
    t = VectorXd::Zero(4);
    for (int j = 0; j < 4; j++)
      t(j) = pow(time, j);

    for (int dim = 0; dim < 3; dim++)
    {
      poly1d = coef.row(dim);
      coord(dim) = poly1d.dot(t);
      vel(dim) = (Tm * poly1d).dot(t);
      acc(dim) = (Tm * Tm * poly1d).dot(t);

      if (fabs(vel(dim)) > max_vel_ || fabs(acc(dim)) > max_acc_)
      {
        return false;
      }
    }

    if (!edt_environment_->isFree(coord))
    {
      return false;
    }
  }
  coef_shot_ = coef;
  t_shot_ = t_d;
  return true;
}

void KinodynamicAstar::stateTransit(Eigen::Matrix<double, 6, 1>& state0,
                                    Eigen::Matrix<double, 6, 1>& state1, Eigen::Vector3d um,
                                    double tau)
{
  for (int i = 0; i < 3; ++i)
    phi_(i, i + 3) = tau;

  Eigen::Matrix<double, 6, 1> integral;
  integral.head(3) = 0.5 * pow(tau, 2) * um;
  integral.tail(3) = tau * um;

  state1 = phi_ * state0 + integral;
}

Eigen::Vector3i KinodynamicAstar::posToIndex(Eigen::Vector3d pt)
{
  Vector3i idx = ((pt - origin_) * inv_resolution_).array().floor().cast<int>();
  return idx;
}

int KinodynamicAstar::timeToIndex(double time)
{
  int idx = floor((time - time_origin_) * inv_time_resolution_);
  return idx;
}

std::vector<Eigen::Vector3d> KinodynamicAstar::getKinoTraj(double delta_t)
{
  std::vector<Eigen::Vector3d> state_list;

  /* ---------- get traj of searching ---------- */
  PathNodePtr node = path_nodes_.back();
  Eigen::Matrix<double, 6, 1> x0, xt;

  while (node->parent != NULL)
  {
    double ut = node->duration;
    int num = ceil(ut / delta_t);
    double dt = ut / num;
    x0 = node->parent->state;

    for (int i = 1; i <= num; ++i)
    {
      stateTransit(x0, xt, node->input, dt * i);
      state_list.push_back(xt.head(3));
    }
    node = node->parent;
  }
  reverse(state_list.begin(), state_list.end());

  /* ---------- get traj of one shot ---------- */
  if (is_shot_succ_)
  {
    Eigen::VectorXd x0(6), xt(6);
    x0.head(3) = path_nodes_.back()->state.head(3);
    x0.tail(3) = path_nodes_.back()->state.tail(3);

    double dt = delta_t;
    double t_d = t_shot_;
    int num = ceil(t_d / dt);
    dt = t_d / num;

    for (int i = 1; i <= num; ++i)
    {
      double time = dt * i;
      Eigen::VectorXd t(4);
      t(0) = 1, t(1) = time, t(2) = pow(time, 2), t(3) = pow(time, 3);

      for (int j = 0; j < 3; ++j)
      {
        Eigen::VectorXd coef = coef_shot_.row(j);
        xt(j) = coef.dot(t);
        xt(j + 3) = (coef.tail(3)).dot(t.tail(3));
      }
      state_list.push_back(xt.head(3));
    }
  }

  return state_list;
}

void KinodynamicAstar::getSamples(double& ts, std::vector<Eigen::Vector3d>& point_set,
                                  std::vector<Eigen::Vector3d>& start_end_derivatives)
{
  /* ---------- path of searching ---------- */
  PathNodePtr node = path_nodes_.back();
  std::vector<Eigen::Vector3d> path;
  while (node->parent != NULL)
  {
    path.push_back(node->state.head(3));
    node = node->parent;
  }
  path.push_back(node->state.head(3));
  reverse(path.begin(), path.end());

  /* ---------- convert to traj parameter ---------- */
  int K = path.size();
  Eigen::Vector3d zero(0, 0, 0);
  Eigen::VectorXd d(K);
  Eigen::MatrixXd A(K, 6);

  for (int i = 1; i < K - 1; ++i)
  {
    d(i) = (path[i + 1] - path[i - 1]).norm() / 2.0;
  }
  d(0) = d(1);
  d(K - 1) = d(K - 2);

  for (int i = 0; i < K; ++i)
  {
    A(i, 0) = (1 - 0) / 3.0;
    A(i, 1) = (1 - 0) / 2.0;
    A(i, 2) = (1 - 0);
    A(i, 3) = (1 - 0) / 6.0;
    A(i, 4) = (1 - 0) / 2.0;
    A(i, 5) = (1 - 0);
  }

  Eigen::VectorXd Dp(K);
  for (int i = 0; i < K; ++i)
    Dp(i) = (path[i] - path[0]).norm();

  Eigen::VectorXd c(6);
  c.head(3) = start_acc_;
  c.tail(3) = start_vel_;

  Eigen::VectorXd b = Dp - A * c;
  Eigen::VectorXd x = A.colPivHouseholderQr().solve(b);

  point_set.clear();
  for (int i = 0; i < K; ++i)
  {
    point_set.push_back(path[i]);
  }

  start_end_derivatives.clear();
  start_end_derivatives.push_back(start_vel_);
  start_end_derivatives.push_back(zero);
  start_end_derivatives.push_back(start_acc_);

  start_end_derivatives.push_back(end_vel_);
  start_end_derivatives.push_back(zero);
  start_end_derivatives.push_back(zero);

  ts = 1.0;
}

std::vector<PathNodePtr> KinodynamicAstar::getVisitedNodes()
{
  std::vector<PathNodePtr> visited;
  visited.assign(path_node_pool_.begin(), path_node_pool_.begin() + use_node_num_ - 1);
  return visited;
}

void KinodynamicAstar::setEnvironment(const std::shared_ptr<plan_env::EDTEnvironment>& env)
{
  edt_environment_ = env;
}

std::vector<double> KinodynamicAstar::cubic(double a, double b, double c, double d)
{
  std::vector<double> dts;

  double a0 = c / a;
  double a1 = d / a;
  double a2 = b / a;
  double a3 = 1.0;

  std::vector<double> ts = quartic(a3, a2, a1, a0, 0);
  for (auto t : ts)
  {
    if (t > 0)
      dts.push_back(t);
  }
  return dts;
}

std::vector<double> KinodynamicAstar::quartic(double a, double b, double c, double d, double e)
{
  std::vector<double> dts;

  double a3 = b / a;
  double a2 = c / a;
  double a1 = d / a;
  double a0 = e / a;

  std::vector<double> ys = cubic(1, -a2, a1 * a3 - 4 * a0, 4 * a2 * a0 - a1 * a1 - a3 * a3 * a0);
  ys.push_back(0);

  double y = *std::max_element(ys.begin(), ys.end());
  double w = sqrt(a3 * a3 / 4 - a2 + y);
  double r = sqrt(3 * a3 * a3 / 4 - w * w - 2 * a2 + (4 * a3 * a2 - 8 * a1 - a3 * a3 * a3) / (4 * w));
  double D = sqrt(3 * a3 * a3 / 4 - w * w - 2 * a2 - (4 * a3 * a2 - 8 * a1 - a3 * a3 * a3) / (4 * w));

  std::vector<double> tss;
  tss.push_back(sqrt(3 * a3 * a3 / 4 - w * w - 2 * a2 + (4 * a3 * a2 - 8 * a1 - a3 * a3 * a3) / (4 * w)));
  tss.push_back(sqrt(3 * a3 * a3 / 4 - w * w - 2 * a2 - (4 * a3 * a2 - 8 * a1 - a3 * a3 * a3) / (4 * w)));

  for (auto t : tss)
  {
    if (t > 0)
      dts.push_back(t);
  }

  return dts;
}

}  // namespace path_searching
