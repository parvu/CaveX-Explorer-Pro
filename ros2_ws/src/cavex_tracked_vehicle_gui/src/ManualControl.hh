#ifndef CAVEX_TRACKED_VEHICLE_GUI__MANUALCONTROL_HH_
#define CAVEX_TRACKED_VEHICLE_GUI__MANUALCONTROL_HH_

#include <gz/gui/Plugin.hh>
#include <gz/transport/Node.hh>

#include <QObject>

namespace cavex_tracked_vehicle_gui
{

// Ported from perception branch's sic_slam_gui::ManualControl, adapted
// for a tracked ground vehicle: no depth axis, no strafe -- the second
// panel (was "depth" mode, up/down for a ROV's vertical thrusters) is
// now "turn" mode, turn-left/turn-right for a differential-track pivot.
// Two instances of this one plugin class, distinguished by <mode>:
// "dpad" (default) shows a D-pad (left/fwd/right/rev + center stop);
// "turn" shows a turn-left/turn-right pair plus the Manual toggle. Every
// button press publishes one gz.msgs.StringMsg command
// ("left"/"forward"/"right"/"backward"/"turn_left"/"turn_right"/"stop"/
// "manual_on"/"manual_off") to /cavex/manual_cmd; manual_gui_bridge.py
// (cavex_tracked_vehicle, not this package -- Qt/gz-gui plugins don't
// touch cmd_vel directly) turns held commands into real /cmd_vel while
// the "Manual" toggle is on.
class ManualControl : public gz::gui::Plugin
{
  Q_OBJECT
  // Not CONSTANT: two instances of this class share one compiled QML
  // resource, and CONSTANT is a hint that lets the QML engine cache/
  // inline a property's value aggressively -- real risk of that caching
  // crossing instance boundaries.
  Q_PROPERTY(QString mode READ Mode NOTIFY ModeChanged)

public:
  ManualControl();
  ~ManualControl() override;

  void LoadConfig(const tinyxml2::XMLElement * _pluginElem) override;

  QString Mode() const;
  Q_INVOKABLE void SendCommand(const QString & _cmd);

signals:
  void ModeChanged();

private:
  gz::transport::Node node;
  gz::transport::Node::Publisher cmdPub;
  QString mode{"dpad"};
};

}  // namespace cavex_tracked_vehicle_gui

#endif  // CAVEX_TRACKED_VEHICLE_GUI__MANUALCONTROL_HH_
