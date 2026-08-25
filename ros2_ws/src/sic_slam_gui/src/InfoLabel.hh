#ifndef SIC_SLAM_GUI__INFOLABEL_HH_
#define SIC_SLAM_GUI__INFOLABEL_HH_

#include <gz/gui/Plugin.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/stringmsg.pb.h>

#include <QObject>
#include <QString>

namespace sic_slam_gui
{

// Editable corner readout: shows a fixed <label>/<unit> plus a live
// numeric value from one gz-transport StringMsg topic (<topic>, plain
// number as text -- formatting the label/unit stays here now, not in the
// publisher, since the value itself needs to be a bare number for the
// input field to edit). Real request: "transform them in an input
// screen -- i want to change them during simulation" -- editing the field
// and pressing Enter publishes the new value to <control_topic>, if one
// is configured; without it, this behaves as a read-only display. Two
// instances are used (current speed, turbidity) with different
// <topic>/<control_topic>/<label>/<unit> config, not two separate plugin
// classes -- see sic_slam_cave_water.world's <gui> section.
class InfoLabel : public gz::gui::Plugin
{
  Q_OBJECT
  Q_PROPERTY(QString text READ Text WRITE SetText NOTIFY TextChanged)
  // Not CONSTANT -- same reasoning as ManualControl::mode: two instances
  // share one compiled QML resource, and CONSTANT's caching hint is a
  // real risk across instance boundaries.
  Q_PROPERTY(QString label READ Label NOTIFY LabelChanged)
  Q_PROPERTY(QString unit READ Unit NOTIFY UnitChanged)

public:
  InfoLabel();
  ~InfoLabel() override;

  void LoadConfig(const tinyxml2::XMLElement * _pluginElem) override;

  QString Text() const;
  void SetText(const QString & _text);
  QString Label() const;
  QString Unit() const;

  Q_INVOKABLE void SendValue(const QString & _value);

signals:
  void TextChanged();
  void LabelChanged();
  void UnitChanged();

private:
  void OnMessage(const gz::msgs::StringMsg & _msg);

  gz::transport::Node node;
  gz::transport::Node::Publisher controlPub;
  bool hasControlTopic{false};
  QString text{"--"};
  QString label;
  QString unit;
};

}  // namespace sic_slam_gui

#endif  // SIC_SLAM_GUI__INFOLABEL_HH_
