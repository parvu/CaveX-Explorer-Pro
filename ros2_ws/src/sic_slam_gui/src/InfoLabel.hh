#ifndef SIC_SLAM_GUI__INFOLABEL_HH_
#define SIC_SLAM_GUI__INFOLABEL_HH_

#include <gz/gui/Plugin.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/stringmsg.pb.h>

#include <QObject>
#include <QString>

namespace sic_slam_gui
{

// Generic corner readout: subscribes to one gz-transport StringMsg topic
// (already-formatted text, e.g. "Current speed: 0.30 m/s" -- formatting
// stays in the publisher, not here, so this plugin has no unit/label
// logic to get wrong) and shows it in InfoLabel.qml's rectangle. Two
// instances are used (current speed, turbidity) with different <topic>/
// <title> config, not two separate plugin classes -- see
// sic_slam_cave_water.world's <gui> section.
class InfoLabel : public gz::gui::Plugin
{
  Q_OBJECT
  Q_PROPERTY(QString text READ Text WRITE SetText NOTIFY TextChanged)

public:
  InfoLabel();
  ~InfoLabel() override;

  void LoadConfig(const tinyxml2::XMLElement * _pluginElem) override;

  QString Text() const;
  void SetText(const QString & _text);

signals:
  void TextChanged();

private:
  void OnMessage(const gz::msgs::StringMsg & _msg);

  gz::transport::Node node;
  QString text{"--"};
};

}  // namespace sic_slam_gui

#endif  // SIC_SLAM_GUI__INFOLABEL_HH_
