#include <algorithm>
#include <array>
#include <chrono>
#include <memory>
#include <mutex>
#include <thread>

#include <QApplication>
#include <QCheckBox>
#include <QDateTime>
#include <QDoubleValidator>
#include <QGridLayout>
#include <QGroupBox>
#include <QLabel>
#include <QLineEdit>
#include <QMainWindow>
#include <QMessageBox>
#include <QPushButton>
#include <QTextEdit>
#include <QVBoxLayout>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "my_arm_config/action/plan_execute_pose.hpp"

using PlanExecutePose = my_arm_config::action::PlanExecutePose;
using GoalHandlePlanExecutePose = rclcpp_action::ClientGoalHandle<PlanExecutePose>;

class QtMoveItClient : public QObject
{
  Q_OBJECT

public:
  explicit QtMoveItClient(QObject * parent = nullptr)
  : QObject(parent)
  {
    node_ = rclcpp::Node::make_shared("qt_moveit_client");
    action_client_ = rclcpp_action::create_client<PlanExecutePose>(node_, "plan_execute_pose");
    executor_.add_node(node_);
    spin_thread_ = std::thread([this]() { executor_.spin(); });
  }

  ~QtMoveItClient() override
  {
    cancelGoal();
    executor_.cancel();
    if (spin_thread_.joinable()) {
      spin_thread_.join();
    }
  }

public slots:
  void sendGoal(
    double x, double y, double z,
    double roll, double pitch, double yaw,
    double velocity, double acceleration,
    const QString & frame_id,
    bool execute, bool wait_for_execution)
  {
    if (!action_client_->wait_for_action_server(std::chrono::seconds(2))) {
      emit resultReady(false, tr("Action server not available"));
      return;
    }

    geometry_msgs::msg::PoseStamped target;
    target.header.stamp = node_->now();
    target.header.frame_id = frame_id.toStdString();
    target.pose.position.x = x;
    target.pose.position.y = y;
    target.pose.position.z = z;

    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    target.pose.orientation = tf2::toMsg(q);

    auto goal_msg = PlanExecutePose::Goal();
    goal_msg.target_pose = target;
    goal_msg.execute = execute;
    goal_msg.wait_for_execution = wait_for_execution;
    goal_msg.velocity_scaling = std::clamp(velocity, 0.01, 1.0);
    goal_msg.acceleration_scaling = std::clamp(acceleration, 0.01, 1.0);

    typename rclcpp_action::Client<PlanExecutePose>::SendGoalOptions options;
    options.goal_response_callback =
      [this](GoalHandlePlanExecutePose::SharedPtr goal_handle) {
        if (!goal_handle) {
          emit resultReady(false, tr("Goal rejected by server"));
          emit goalActive(false);
          return;
        }
        {
          std::lock_guard<std::mutex> lock(goal_mutex_);
          active_goal_ = goal_handle;
        }
        emit goalActive(true);
      };

    options.feedback_callback =
      [this](
        GoalHandlePlanExecutePose::SharedPtr,
        const std::shared_ptr<const PlanExecutePose::Feedback> feedback) {
        const auto message = QString::fromStdString(feedback->status) +
                             tr(" (progress %1%)").arg(feedback->progress * 100.0f, 0, 'f', 1);
        emit feedbackUpdated(message);
      };

    options.result_callback =
      [this](const GoalHandlePlanExecutePose::WrappedResult & result) {
        bool success = result.code == rclcpp_action::ResultCode::SUCCEEDED;
        QString text = result.result ? QString::fromStdString(result.result->message)
                                     : tr("未收到服务器结果");
        if (result.code == rclcpp_action::ResultCode::ABORTED) {
          text = tr("Aborted: %1").arg(text);
        } else if (result.code == rclcpp_action::ResultCode::CANCELED) {
          text = tr("Canceled: %1").arg(text);
        }
        emit resultReady(success, text);
        emit goalActive(false);
        std::lock_guard<std::mutex> lock(goal_mutex_);
        active_goal_.reset();
      };

    action_client_->async_send_goal(goal_msg, options);
  }

  void cancelGoal()
  {
    std::lock_guard<std::mutex> lock(goal_mutex_);
    if (active_goal_) {
      action_client_->async_cancel_goal(active_goal_);
    }
  }

signals:
  void feedbackUpdated(const QString & text);
  void resultReady(bool success, const QString & text);
  void goalActive(bool active);

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<PlanExecutePose>::SharedPtr action_client_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::thread spin_thread_;
  std::mutex goal_mutex_;
  GoalHandlePlanExecutePose::SharedPtr active_goal_;
};

class QtArmWindow : public QMainWindow
{
  Q_OBJECT

public:
  QtArmWindow()
  : QMainWindow(nullptr)
  , client_(new QtMoveItClient(this))
  {
    setWindowTitle(tr("My Arm Motion Console"));
    buildUi();

    connect(client_, &QtMoveItClient::feedbackUpdated, this, &QtArmWindow::handleFeedback);
    connect(client_, &QtMoveItClient::resultReady, this, &QtArmWindow::handleResult);
    connect(client_, &QtMoveItClient::goalActive, this, &QtArmWindow::handleGoalActive);
  }

private slots:
  void handlePlan()
  {
    sendGoal(false);
  }

  void handlePlanAndExecute()
  {
    sendGoal(true);
  }

  void handleCancel()
  {
    client_->cancelGoal();
  }

  void handleFeedback(const QString & msg)
  {
    appendLog(tr("[Feedback] %1").arg(msg));
  }

  void handleResult(bool success, const QString & msg)
  {
    appendLog(success ? tr("[Success] %1").arg(msg) : tr("[Failure] %1").arg(msg));
  }

  void handleGoalActive(bool active)
  {
    plan_button_->setEnabled(!active);
    plan_exec_button_->setEnabled(!active);
    cancel_button_->setEnabled(active);
  }

private:
  void buildUi()
  {
    QWidget * central = new QWidget(this);
    auto * layout = new QVBoxLayout(central);

    auto * pose_group = new QGroupBox(tr("目标位姿"), central);
    auto * pose_layout = new QGridLayout(pose_group);

    createVectorFields(pose_layout);

    layout->addWidget(pose_group);

    auto * control_group = new QGroupBox(tr("控制参数"), central);
    auto * control_layout = new QGridLayout(control_group);

    velocity_edit_ = createLabeledField(control_layout, tr("速度比例"), 0, 0, "");
    acceleration_edit_ = createLabeledField(control_layout, tr("加速度比例"), 1, 0, "");
    frame_edit_ = createLabeledField(control_layout, tr("参考坐标系"), 2, 0, "");

    auto *vel_validator = new QDoubleValidator(0.01, 1.0, 2, velocity_edit_);
    auto *acc_validator = new QDoubleValidator(0.01, 1.0, 2, acceleration_edit_);
    velocity_edit_->setValidator(vel_validator);
    acceleration_edit_->setValidator(acc_validator);
    velocity_edit_->setPlaceholderText(tr("范围: [0.01, 1.0]"));
    acceleration_edit_->setPlaceholderText(tr("范围: [0.01, 1.0]"));
    velocity_edit_->setToolTip(tr("比例越小，规划速度越慢，越安全\n参考: config/joint_limits.yaml 中的 default_velocity_scaling_factor"));
    acceleration_edit_->setToolTip(tr("比例越小，加速度越小，运动更平滑\n参考: config/joint_limits.yaml 中的 default_acceleration_scaling_factor"));

    frame_edit_->setPlaceholderText(tr("例如: world 或 base_link"));
    frame_edit_->setToolTip(tr("必须是 TF 树中存在的坐标系名称\n参考: config/my_arm_with_gripper.srdf 中的 virtual_joint"));
    wait_checkbox_ = new QCheckBox(tr("等待执行完成"), control_group);
    wait_checkbox_->setChecked(true);
    control_layout->addWidget(wait_checkbox_, 3, 0, 1, 2);

    layout->addWidget(control_group);

    auto * button_layout = new QGridLayout();
    plan_button_ = new QPushButton(tr("仅规划"), central);
    plan_exec_button_ = new QPushButton(tr("规划并执行"), central);
    cancel_button_ = new QPushButton(tr("停止执行"), central);

    button_layout->addWidget(plan_button_, 0, 0);
    button_layout->addWidget(plan_exec_button_, 0, 1);
    button_layout->addWidget(cancel_button_, 0, 2);

    layout->addLayout(button_layout);

    log_view_ = new QTextEdit(central);
    log_view_->setReadOnly(true);
    layout->addWidget(log_view_);

    setCentralWidget(central);

    connect(plan_button_, &QPushButton::clicked, this, &QtArmWindow::handlePlan);
    connect(plan_exec_button_, &QPushButton::clicked, this, &QtArmWindow::handlePlanAndExecute);
    connect(cancel_button_, &QPushButton::clicked, this, &QtArmWindow::handleCancel);
  }

  void createVectorFields(QGridLayout * layout)
  {
    const std::array<QString, 3> labels = {tr("X (m)"), tr("Y (m)"), tr("Z (m)")};

    for (int i = 0; i < 3; ++i) {
      position_edits_[i] = createLabeledField(layout, labels[i], i, 0, "");
    }
    auto *x_validator = new QDoubleValidator(-0.8, 0.8, 3, position_edits_[0]);
    auto *y_validator = new QDoubleValidator(-0.8, 0.8, 3, position_edits_[1]);
    auto *z_validator = new QDoubleValidator(0.0, 1.2, 3, position_edits_[2]);
    position_edits_[0]->setValidator(x_validator);
    position_edits_[1]->setValidator(y_validator);
    position_edits_[2]->setValidator(z_validator);
    position_edits_[0]->setPlaceholderText(tr("范围: [-0.8, 0.8] m"));
    position_edits_[1]->setPlaceholderText(tr("范围: [-0.8, 0.8] m"));
    position_edits_[2]->setPlaceholderText(tr("范围: [0.0, 1.2] m"));
    position_edits_[0]->setToolTip(tr("工作空间范围根据 URDF 分析确定\n参考: URDF 关节限制或实际测试"));
    position_edits_[1]->setToolTip(tr("工作空间范围根据 URDF 分析确定\n参考: URDF 关节限制或实际测试"));
    position_edits_[2]->setToolTip(tr("工作空间范围根据 URDF 分析确定\n参考: URDF 关节限制或实际测试"));

    const std::array<QString, 3> rpy_labels = {tr("Roll (rad)"), tr("Pitch (rad)"), tr("Yaw (rad)")};
    // 根据 URDF，所有关节限制都是 -3.14 到 3.14 弧度（-π 到 π）
    for (int i = 0; i < 3; ++i) {
      orientation_edits_[i] = createLabeledField(layout, rpy_labels[i], i, 2, "");
    }
    for (int i = 0; i < 3; ++i) {
      auto *validator = new QDoubleValidator(-3.1416, 3.1416, 4, orientation_edits_[i]);
      orientation_edits_[i]->setValidator(validator);
      orientation_edits_[i]->setPlaceholderText(tr("范围: [-π, π] rad"));
      orientation_edits_[i]->setToolTip(tr("根据 URDF 关节限制: [-π, π] 弧度\n所有关节的 limit 都是 lower=\"-3.14\" upper=\"3.14\""));
    }
  }

  QLineEdit * createLabeledField(
    QGridLayout * layout, const QString & label, int row, int column, const QString & default_value)
  {
    auto * name_label = new QLabel(label, this);
    auto * edit = new QLineEdit(this);
    edit->setText(default_value);
    layout->addWidget(name_label, row, column);
    layout->addWidget(edit, row, column + 1);
    return edit;
  }

  bool extractValues(
    double & x, double & y, double & z,
    double & roll, double & pitch, double & yaw,
    double & velocity, double & acceleration)
  {
    bool ok = true;
    const auto parse = [&](QLineEdit * edit, double & value) {
      bool local_ok = false;
      value = edit->text().toDouble(&local_ok);
      ok &= local_ok;
    };

    parse(position_edits_[0], x);
    parse(position_edits_[1], y);
    parse(position_edits_[2], z);
    parse(orientation_edits_[0], roll);
    parse(orientation_edits_[1], pitch);
    parse(orientation_edits_[2], yaw);
    parse(velocity_edit_, velocity);
    parse(acceleration_edit_, acceleration);

    if (!ok) {
      QMessageBox::warning(this, tr("无效输入"), tr("请检查所有输入是否为数字"));
    }

    return ok;
  }

  void sendGoal(bool execute)
  {
    double x, y, z, roll, pitch, yaw, velocity, acceleration;
    if (!extractValues(x, y, z, roll, pitch, yaw, velocity, acceleration)) {
      return;
    }

    const auto frame = frame_edit_->text();
    if (frame.isEmpty()) {
      QMessageBox::warning(this, tr("无效输入"), tr("参考坐标系不能为空"));
      return;
    }

    appendLog(execute ? tr("开始规划并执行...") : tr("开始规划..."));
    client_->sendGoal(
      x, y, z, roll, pitch, yaw, velocity, acceleration,
      frame, execute, wait_checkbox_->isChecked());
  }

  void appendLog(const QString & text)
  {
    log_view_->append(QStringLiteral("[%1] %2")
                        .arg(QDateTime::currentDateTime().toString("hh:mm:ss"))
                        .arg(text));
  }

  QtMoveItClient * client_{nullptr};
  QLineEdit * position_edits_[3]{};
  QLineEdit * orientation_edits_[3]{};
  QLineEdit * velocity_edit_{nullptr};
  QLineEdit * acceleration_edit_{nullptr};
  QLineEdit * frame_edit_{nullptr};
  QCheckBox * wait_checkbox_{nullptr};
  QPushButton * plan_button_{nullptr};
  QPushButton * plan_exec_button_{nullptr};
  QPushButton * cancel_button_{nullptr};
  QTextEdit * log_view_{nullptr};
};

#include "qt_arm_app.moc"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  QApplication app(argc, argv);
  QtArmWindow window;
  window.resize(600, 500);
  window.show();
  const int exit_code = app.exec();
  rclcpp::shutdown();
  return exit_code;
}

