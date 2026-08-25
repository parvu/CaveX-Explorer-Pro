#ifndef SIC_SLAM_GUI__MANUALCONTROL_HH_
#define SIC_SLAM_GUI__MANUALCONTROL_HH_

#include <gz/gui/Plugin.hh>
#include <gz/transport/Node.hh>

#include <QObject>

namespace sic_slam_gui
{

// Two instances of this one plugin class, distinguished by <mode> in the
// world file's config (see ManualControl.qml): "dpad" (default) shows a
// D-pad (left/fwd/right/rev + center stop) for XY; "depth" shows a
// separate up/down pair plus the Manual toggle. Every button press
// publishes one gz.msgs.StringMsg command ("left"/"forward"/"right"/
// "backward"/"up"/"down"/"stop"/"manual_on"/"manual_off") to
// /sic_slam/manual_cmd; manual_control_node.py (sic_slam, not this
// package -- Qt/gz-gui plugins don't touch the actual thruster topics
// directly) turns held commands into real thrust while the "Manual"
// toggle is on, and does nothing while it's off so an autonomous script
// (ate_circle_demo.py etc.) keeps sole control of the thrusters.
class ManualControl : public gz::gui::Plugin
{
  Q_OBJECT
  Q_PROPERTY(QString mode READ Mode CONSTANT)

public:
  ManualControl();
  ~ManualControl() override;

  void LoadConfig(const tinyxml2::XMLElement * _pluginElem) override;

  QString Mode() const;
  Q_INVOKABLE void SendCommand(const QString & _cmd);

private:
  gz::transport::Node node;
  gz::transport::Node::Publisher cmdPub;
  QString mode{"dpad"};
};

}  // namespace sic_slam_gui

#endif  // SIC_SLAM_GUI__MANUALCONTROL_HH_
