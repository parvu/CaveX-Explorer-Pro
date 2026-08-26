#ifndef CAVEX_TRACKED_VEHICLE_GUI__ACTIONBUTTONS_HH_
#define CAVEX_TRACKED_VEHICLE_GUI__ACTIONBUTTONS_HH_

#include <gz/gui/Plugin.hh>
#include <gz/transport/Node.hh>

#include <QObject>
#include <QString>

namespace cavex_tracked_vehicle_gui
{

// Small corner rectangle with a label and two command buttons -- the
// discrete-action counterpart to sic_slam_gui's InfoLabel (a numeric
// text-input rectangle): track deploy/retract and rover lock/unlock are
// String/Empty commands, not editable numbers, so a button pair fits the
// real message types instead of forcing a DoubleValidator text field.
// Two instances configured via XML (see cavex_world.world's <gui>
// section): <label> (card title text), <button1_label>/<button1_cmd>,
// <button2_label>/<button2_cmd>, <topic> (control topic, StringMsg).
class ActionButtons : public gz::gui::Plugin
{
  Q_OBJECT
  // Not CONSTANT -- same reasoning as ManualControl::mode: two instances
  // share one compiled QML resource.
  Q_PROPERTY(QString label READ Label NOTIFY LabelChanged)
  Q_PROPERTY(QString button1Label READ Button1Label NOTIFY Button1LabelChanged)
  Q_PROPERTY(QString button2Label READ Button2Label NOTIFY Button2LabelChanged)

public:
  ActionButtons();
  ~ActionButtons() override;

  void LoadConfig(const tinyxml2::XMLElement * _pluginElem) override;

  QString Label() const;
  QString Button1Label() const;
  QString Button2Label() const;

  Q_INVOKABLE void SendButton1();
  Q_INVOKABLE void SendButton2();

signals:
  void LabelChanged();
  void Button1LabelChanged();
  void Button2LabelChanged();

private:
  void SendCommand(const std::string & _cmd);

  gz::transport::Node node;
  gz::transport::Node::Publisher controlPub;
  QString label;
  QString button1Label;
  QString button1Cmd;
  QString button2Label;
  QString button2Cmd;
};

}  // namespace cavex_tracked_vehicle_gui

#endif  // CAVEX_TRACKED_VEHICLE_GUI__ACTIONBUTTONS_HH_
